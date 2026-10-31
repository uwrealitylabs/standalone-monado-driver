"""Port-specific source guard and historical-output regression checks."""
import json
from pathlib import Path
import tempfile
import unittest

from experiment import compare_normal_pixels, monado_identity, sha, validate_protected_sources


class IntegrationTests(unittest.TestCase):
    def test_repository_identity_records_authored_changes_without_rejecting_them(self):
        identity = monado_identity()
        self.assertTrue(identity["reviewed_sources_match"])
        self.assertEqual(identity["working_tree_clean"], not identity["working_tree_changes"])
        self.assertEqual(len(identity["revision"]), 40)

    def test_changed_protected_source_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "xrt_frame.h"
            path.write_text("reviewed")
            expected = {path.name: sha(path)}
            self.assertEqual(validate_protected_sources(directory, expected), expected)
            path.write_text("changed")
            with self.assertRaisesRegex(RuntimeError, "xrt_frame.h"):
                validate_protected_sources(directory, expected)

    def test_missing_protected_source_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, "missing"):
                validate_protected_sources(directory, {"xrt_frame.h": "not-present"})

    def test_pixel_baseline_checks_identity_completeness_and_content(self):
        baseline = {"prototype_report_id": "historical", "cases": {
            "four-l8": {"config": {}, "cameras": ["0"]}},
            "pixel_sha256_by_camera": {"0": ["a", "b"]}}
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "frames.jsonl"
            summary = {"scenario": "four-l8", "config": {}, "output": directory}
            rows = [{"camera_id": 0, "sequence": i, "image_sha256": h}
                    for i, h in enumerate(("a", "b"))]
            def write(values):
                log.write_text("".join(json.dumps(row) + "\n" for row in values))
            write(rows)
            self.assertEqual(compare_normal_pixels(summary, baseline)["matched_frames"], 2)
            write(rows[:1])
            with self.assertRaisesRegex(RuntimeError, "differ"):
                compare_normal_pixels(summary, baseline)
            write(rows + rows[:1])
            with self.assertRaisesRegex(RuntimeError, "Duplicate"):
                compare_normal_pixels(summary, baseline)
            rows[1]["image_sha256"] = "changed"
            write(rows)
            with self.assertRaisesRegex(RuntimeError, "differ"):
                compare_normal_pixels(summary, baseline)

    def test_pixel_baseline_rejects_changed_fixture_settings(self):
        baseline = {"cases": {"four-l8": {"config": {"frames": 150}}}}
        with self.assertRaisesRegex(RuntimeError, "original normal fixture"):
            compare_normal_pixels({"scenario": "four-l8", "config": {"frames": 1}}, baseline)
