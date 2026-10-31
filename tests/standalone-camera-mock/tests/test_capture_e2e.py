"""Targeted end-to-end native Capture integration checks."""
import ctypes as ct,json,tempfile,time,unittest
from pathlib import Path
from capture_e2e.__main__ import build,load_case,run_case,library,E2EStats,SOURCE_CALLBACK,validate
from capture_mock.bindings import Config,native_config
from pipeline_matrix.__main__ import CALLBACK,Stats as PipelineStats
class CaptureE2ETests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):build()
    def config(self,**changes):
        c=load_case("four-l8");c.update(width=64,height=64,frames=12,fps_num=120,queue_capacity=32)
        c.update(changes);return c
    def execute(self,**changes):
        directory=tempfile.TemporaryDirectory(prefix="wrl-e2e-");self.addCleanup(directory.cleanup)
        result=run_case("unit-fixture",self.config(**changes),Path(directory.name)/"run",do_build=False)
        self.assertTrue(result["local_prototype_passed"],result)
        return result
    def test_four_native_l8_sources_reach_cv(self):
        r=self.execute()
        self.assertTrue(all(s["processed"]==12 for s in r["streams"].values()))
        self.assertEqual(r["pipeline"]["processed"],48)
    def test_four_native_sources_survive_rgb_conversion(self):
        r=self.execute(converter="RGB")
        self.assertEqual(r["pipeline"]["processed"],48)
        rows=[json.loads(x) for x in (Path(r["output"])/"frames.jsonl").read_text().splitlines()]
        self.assertEqual(len({x["image_sha256"] for x in rows if x["sequence"]==0}),4)
    def test_padding_direct_and_converted(self):
        for converter in ("none","RGB"):
            with self.subTest(converter=converter):self.execute(padding=16,converter=converter)
    def test_malformed_frames_rejected_before_native_converter(self):
        for kind in (1,2,3):
            with self.subTest(kind=kind):
                r=self.execute(converter="RGB",malformed_camera=1,malformed_kind=kind)
                self.assertEqual(r["streams"]["1"]["guard_rejected"],1)
                self.assertEqual(r["pipeline"]["processed"],47)
    def test_interrupted_camera_does_not_stop_other_sources(self):
        r=self.execute(interrupt_camera=2,interrupt_after=3)
        self.assertEqual(r["streams"]["2"]["processed"],3)
        self.assertEqual(r["streams"]["3"]["processed"],12)
    def test_slow_consumer_keeps_valid_pixels_and_accounts_for_drops(self):
        r=self.execute(converter="RGB",frames=30,fps_num=300,queue_capacity=2,consumer_delay_ms=15)
        self.assertGreater(sum(s["dropped"] for s in r["streams"].values()),0)
        self.assertEqual(r["pipeline"]["live"],0)
    def test_optional_stereo_pairing_orders_native_source_frames(self):
        r=self.execute(converter="RGB",cameras=2,pairing=True)
        self.assertTrue(r["checks"]["pair_0_ordered"])
    def test_manual_stop_and_recreated_session(self):
        stopped=self.execute(converter="RGB",frames=150,manual_stop_after_ms=120)
        self.assertTrue(all(0<s["generated"]<150 for s in stopped["streams"].values()))
        restarted=self.execute(session_id=2)
        self.assertEqual(restarted["pipeline"]["processed"],48)
    def test_future_source_timeout_is_bounded_and_repeatable(self):
        lib=library();c=self.config(cameras=1)
        cfg=(Config*1)(native_config(c,0))
        cb=CALLBACK(lambda *_:0);source_cb=SOURCE_CALLBACK(lambda *_:0)
        handle=lib.e2e_create(cfg,1,2,0,cb,source_cb,None);self.assertTrue(handle)
        try:
            epoch=time.clock_gettime_ns(time.CLOCK_MONOTONIC)+2_000_000_000
            self.assertEqual(lib.e2e_start(handle,epoch),0)
            start=time.monotonic();self.assertEqual(lib.e2e_finish(handle,1),1)
            self.assertLess(time.monotonic()-start,1)
            self.assertEqual(lib.e2e_finish(handle,1),1)
            src=E2EStats();pipe=PipelineStats()
            lib.e2e_stats(handle,ct.byref(src));lib.e2e_pipeline_stats(handle,ct.byref(pipe))
            self.assertEqual(pipe.live,0);self.assertEqual(src.sources[0].producer_done,1)
        finally:lib.e2e_close(handle)
    def test_existing_results_refused_and_invalid_route_rejected(self):
        r=self.execute(cameras=1)
        with self.assertRaises(FileExistsError):
            run_case("unit-fixture",self.config(cameras=1),Path(r["output"]),do_build=False)
        with self.assertRaises(ValueError):validate(self.config(converter="Bayer"))
