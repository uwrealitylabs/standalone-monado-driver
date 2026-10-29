"""Native-interface, independent pixel, fault and lifecycle checks for the CV prototype."""
import ctypes as ct
import json
import tempfile
import time
import unittest
from pathlib import Path

from capture_mock.bindings import ROOT, CALLBACK, Stats, library, native_config
from capture_mock.build import ensure_build
from capture_mock.runner import load_case, run_case, validate_config
import cv2

class CaptureMockTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ensure_build()

    def config(self, **overrides):
        config = load_case("one-camera")
        config.update(width=64, height=64, frames=12, fps_num=120, queue_capacity=32)
        config.update(overrides)
        return config

    def execute(self, config):
        directory = tempfile.TemporaryDirectory(prefix="wrl-capture-test-")
        self.addCleanup(directory.cleanup)
        result = run_case("unit-fixture", config, Path(directory.name) / "run", build=False)
        self.assertTrue(result["prototype_path_verified"], result["checks"])
        return result

    def test_independent_pixel_fixture_and_padding(self):
        fixture = json.loads((ROOT / "capture_mock/fixtures/independent_pattern.json").read_text())
        result = self.execute(self.config(padding=fixture["padding"]))
        image = cv2.imread(str(Path(result["output"]) / "camera-0-session-1-input.png"),
                           cv2.IMREAD_GRAYSCALE)
        self.assertEqual(image[0, :16].tolist(), fixture["expected_header"])
        for pixel in fixture["expected_pixels"]:
            self.assertEqual(int(image[pixel["y"], pixel["x"]]), pixel["value"])
        self.assertEqual(result["streams"]["0"]["processed"], 12)

    def test_four_distinct_overlapping_streams(self):
        result = self.execute(self.config(cameras=4))
        records = [json.loads(line) for line in (Path(result["output"]) / "frames.jsonl").read_text().splitlines()]
        first = {r["camera_id"]: r["image_sha256"] for r in records if r["sequence_number"] == 0}
        self.assertEqual(len(set(first.values())), 4)
        self.assertTrue(result["checks"]["four_streams_overlap"])

    def test_invalid_format_size_and_stride_are_rejected_without_hang(self):
        for kind, message in [(1, "Unsupported frame format"), (2, "Invalid payload size"), (3, "Invalid row stride")]:
            with self.subTest(kind=kind):
                result = self.execute(self.config(malformed_camera=0, malformed_kind=kind))
                stream = result["streams"]["0"]
                self.assertEqual(stream["processed"], 11)
                self.assertEqual(stream["rejected"], 1)
                self.assertIn(message, stream["rejection_messages"][0])

    def test_one_interrupted_stream_does_not_stop_other_cameras(self):
        result = self.execute(self.config(cameras=4, interrupt_camera=2, interrupt_after=3))
        self.assertEqual(result["streams"]["2"]["generated"], 3)
        self.assertEqual(result["streams"]["0"]["processed"], 12)
        self.assertEqual(result["streams"]["3"]["processed"], 12)

    def test_slow_consumer_exercises_real_queue_drop_and_retention(self):
        result = self.execute(self.config(frames=30, fps_num=500, queue_capacity=2, consumer_delay_ms=15))
        self.assertGreater(result["streams"]["0"]["queue_dropped"], 0)
        self.assertEqual(result["streams"]["0"]["retained_valid"], 1)
        self.assertEqual(result["streams"]["0"]["live_frames"], 0)

    def test_stop_recreate_session_and_deterministic_pixels(self):
        first = self.execute(self.config(frames=200, manual_stop_after_ms=100))
        self.assertLess(first["streams"]["0"]["generated"], 200)
        second = self.execute(self.config(session_id=2))
        records = [json.loads(line) for line in (Path(second["output"]) / "frames.jsonl").read_text().splitlines()]
        self.assertTrue(all(r["session_id"] == 2 for r in records))
        repeated = self.execute(self.config())
        repeated_again = self.execute(self.config())
        def hashes(result):
            return [json.loads(line)["image_sha256"] for line in
                    (Path(result["output"]) / "frames.jsonl").read_text().splitlines()]
        self.assertEqual(hashes(repeated), hashes(repeated_again))

    def test_existing_results_are_never_overwritten(self):
        result = self.execute(self.config())
        with self.assertRaises(FileExistsError):
            run_case("unit-fixture", self.config(), Path(result["output"]), build=False)

    def test_timeout_interrupts_future_capture_and_releases_all_buffers(self):
        lib = library()
        cfg = native_config(self.config(), 0)
        callback = CALLBACK(lambda *_: 0)
        handle = lib.wrl_harness_create(ct.byref(cfg), callback, None)
        self.assertTrue(handle)
        try:
            future = time.clock_gettime_ns(time.CLOCK_MONOTONIC) + 2_000_000_000
            self.assertEqual(lib.wrl_harness_start(handle, future), 0)
            before = time.monotonic()
            self.assertEqual(lib.wrl_harness_finish(handle, 1), 1)
            self.assertLess(time.monotonic() - before, 1.0)
            stats = Stats()
            lib.wrl_harness_stats(handle, ct.byref(stats))
            self.assertEqual(stats.live_frames, 0)
            self.assertEqual(stats.producer_done, 1)
        finally:
            lib.wrl_harness_close(handle)


    def test_group_bundle_preserves_four_routes_with_one_source_identity(self):
        result = self.execute(self.config(cameras=4, source_mode="image-bundle"))
        records = [json.loads(line) for line in (Path(result["output"]) / "frames.jsonl").read_text().splitlines()]
        self.assertEqual({r["source_id"] for r in records}, {0x57524C47})
        self.assertEqual({r["camera_id"] for r in records}, {0, 1, 2, 3})
        self.assertEqual(len({r["image_sha256"] for r in records if r["sequence_number"] == 0}), 4)
        self.assertTrue(result["checks"]["shared_source_id_with_distinct_routes"])
        self.assertTrue(all(s["processed"] == 12 for s in result["streams"].values()))

    def test_group_bundle_fault_and_interruption_preserve_other_routes(self):
        for overrides in ({"interrupt_camera": 2, "interrupt_after": 3},
                          {"malformed_camera": 1, "malformed_kind": 1}):
            with self.subTest(overrides=overrides):
                result = self.execute(self.config(cameras=4, source_mode="image-bundle", **overrides))
                self.assertEqual(result["streams"]["0"]["processed"], 12)
                self.assertEqual(result["streams"]["3"]["processed"], 12)

    def test_group_rejects_duplicate_handles_before_starting(self):
        lib = library()
        cfg = native_config(self.config(), 0)
        callback = CALLBACK(lambda *_: 0)
        handle = lib.wrl_harness_create(ct.byref(cfg), callback, None)
        self.assertTrue(handle)
        try:
            duplicates = (ct.c_void_p * 2)(handle, handle)
            self.assertEqual(lib.wrl_harness_group_start(duplicates, 2, 0), -1)
            stats = Stats()
            lib.wrl_harness_stats(handle, ct.byref(stats))
            self.assertEqual(stats.started_ns, 0)
            single = (ct.c_void_p * 1)(handle)
            self.assertEqual(lib.wrl_harness_group_start(single, 1, 0), 0)
            self.assertEqual(lib.wrl_harness_finish(handle, 2000), 0)
            self.assertEqual(lib.wrl_harness_group_start(single, 1, 0), -1)
        finally:
            lib.wrl_harness_close(handle)

    def test_group_stop_recreate_keeps_retained_buffers_valid(self):
        result = self.execute(self.config(cameras=4, source_mode="image-bundle",
                                         frames=200, manual_stop_after_ms=100))
        self.assertTrue(all(s["generated"] < 200 for s in result["streams"].values()))
        second = self.execute(self.config(cameras=4, source_mode="image-bundle", session_id=2))
        self.assertTrue(all(s["live_frames"] == 0 and s["retained_valid"] == 1
                            for s in second["streams"].values()))

    def test_invalid_configuration_rejected_before_native_execution(self):
        with self.assertRaises(ValueError):
            validate_config(self.config(width=0))
        lib = library()
        cfg = native_config(self.config(), 0)
        cfg.fps_num = 0
        callback = CALLBACK(lambda *_: 0)
        self.assertFalse(lib.wrl_harness_create(ct.byref(cfg), callback, None))

if __name__ == "__main__":
    unittest.main()
