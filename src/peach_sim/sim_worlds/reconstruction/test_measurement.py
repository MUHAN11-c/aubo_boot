"""Checks for valid-depth support and annotation crop boundaries."""
import unittest
import numpy as np
from analyze_reference import measure


class MeasurementTests(unittest.TestCase):
    def test_zero_depth_is_not_a_surface(self):
        self.assertIsNone(
            measure(
                np.ones(
                    (20, 20), bool), np.zeros(
                    (20, 20), np.uint16), [
                    0, 0, 20, 20]))

    def test_holes_do_not_bias_depth(self):
        depth = np.full((20, 20), 500, np.uint16)
        depth[:10] = 0
        result = measure(
            np.ones_like(
                depth, dtype=bool), depth, [
                0, 0, 20, 20])
        self.assertAlmostEqual(result['depth_m'], .5)
        self.assertAlmostEqual(result['valid_fraction'], .5)

    def test_sam_leakage_outside_prompt_is_excluded(self):
        depth = np.full((30, 30), 1500, np.uint16)
        depth[10:20, 10:20] = 400
        result = measure(
            np.ones_like(
                depth, dtype=bool), depth, [
                10, 10, 20, 20])
        self.assertAlmostEqual(result['depth_m'], .4)
        self.assertEqual(result['mask_pixels'], 100)


if __name__ == '__main__':
    unittest.main()
