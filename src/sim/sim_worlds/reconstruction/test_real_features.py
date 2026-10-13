"""Behavior checks for real-data screen proxies; no Blender or ROS."""
from pathlib import Path
import tempfile
import unittest

from audit_real_features import frame_record, screen_statistics, valid_depth_statistics
import numpy as np


class RealFeatureTests(unittest.TestCase):
    def test_corrupt_source_preserves_hash_and_error(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / 'Peach_bag' / 'Depth' / '1.png'
            path.parent.mkdir(parents=True)
            path.write_bytes(b'not a png')
            result = frame_record((root, 'Peach_bag', '1'))
            self.assertEqual(len(result['modalities']['Depth']['sha256']), 64)
            self.assertIn('decode_error', result['modalities']['Depth'])
            self.assertEqual(len(result['errors']), 1)

    def test_no_green_has_no_color_measurement(self):
        rgb = np.full((10, 10, 3), [150, 30, 20], dtype=np.uint8)
        result = screen_statistics(rgb)
        self.assertEqual(result['green_fraction'], 0.)
        self.assertIsNone(result['green_rgb_p10_p50_p90'])

    def test_green_counts_pixels_and_excludes_watermark_like_yellow(self):
        rgb = np.full((10, 10, 3), [20, 100, 30], dtype=np.uint8)
        rgb[:2] = [180, 180, 100]
        result = screen_statistics(rgb)
        self.assertAlmostEqual(result['green_fraction'], .8)
        self.assertEqual(result['green_rgb_p10_p50_p90'][1], [20., 100., 30.])
        self.assertAlmostEqual(result['yellow_screen_exclusion_fraction'], .2)

    def test_zero_depth_is_missing_not_surface(self):
        result = valid_depth_statistics(np.zeros((4, 5), dtype=np.uint16))
        self.assertEqual(result['valid_fraction'], 0.)
        self.assertIsNone(result['valid_raw_p10_p50_p90'])

    def test_depth_quantiles_ignore_missing_zero(self):
        depth = np.array([[0, 100], [0, 200]], dtype=np.uint16)
        result = valid_depth_statistics(depth)
        self.assertEqual(result['valid_fraction'], .5)
        self.assertEqual(result['valid_raw_p10_p50_p90'], [110., 150., 190.])


if __name__ == '__main__':
    unittest.main()
