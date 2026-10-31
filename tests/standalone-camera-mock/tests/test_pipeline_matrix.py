"""Meaningful native conversion, pairing, overload and public-frame checks."""
import tempfile,unittest
from pathlib import Path
from pipeline_matrix.__main__ import build,cases,execute
class PipelineMatrixTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):build()
    def config(self,**changes):
        c=cases()[0].copy()
        c.update(input_format="L8",converter="RGB",padding=16,cameras=4,input_order="reverse")
        c.update(changes);return c
    def run_case(self,**changes):
        with tempfile.TemporaryDirectory(prefix="wrl-matrix-test-") as directory:
            result=execute(self.config(**changes),Path(directory)/"run")
        self.assertTrue(result["expected_outcome_met"],result)
        return result
    def test_matrix_has_all_declared_combinations(self):
        values=cases()
        self.assertEqual(len(values),129)
        self.assertEqual(sum(x["kind"]=="format-route" for x in values),108)
        self.assertEqual(sum(x["kind"]=="overload" for x in values),9)
        self.assertEqual(sum(x["kind"]=="pairing" for x in values),12)
    def test_padded_four_camera_l8_to_rgb_conversion(self):
        r=self.run_case()
        self.assertEqual(r["counts"]["processed"],32)
        self.assertEqual(r["counts"]["live"],0)
    def test_r8_is_rejected_by_real_l8_converter(self):
        r=self.run_case(input_format="R8",converter="L8",cameras=1)
        self.assertEqual(r["component_verdict"],"FORMAT_BLOCKED")
        self.assertEqual(r["counts"]["processed"],0)
        self.assertGreaterEqual(r["counts"]["errors"],8)
    def test_rgb_is_rejected_by_real_l8_converter(self):
        self.run_case(input_format="RGB",converter="L8",cameras=1)
    def test_r8_direct_public_observer_remains_usable(self):
        r=self.run_case(input_format="R8",converter="none",cameras=1)
        self.assertEqual(r["counts"]["processed"],8)
    def test_actual_pairing_serializes_reverse_input_and_preserves_source_time(self):
        self.run_case(converter="none",cameras=2,pairing=True,kind="pairing",frames=4,
                      first_pair_skew_ns=500000)
    def test_out_of_window_pair_is_rejected_and_later_pairs_recover(self):
        r=self.run_case(converter="none",cameras=2,pairing=True,kind="pairing",frames=4,
                        first_pair_skew_ns=2000000)
        self.assertEqual(r["counts"]["processed"],6)
        self.assertGreaterEqual(r["counts"]["warnings"],1)
    def test_slow_consumer_has_accounted_drops_and_safe_retention(self):
        r=self.run_case(converter="none",cameras=4,kind="overload",frames=24,
                        queue_capacity=2,consumer_delay_ms=25)
        self.assertGreater(r["unreceived_after_conversion"],0)
        self.assertEqual(r["counts"]["retained_valid"],1)
