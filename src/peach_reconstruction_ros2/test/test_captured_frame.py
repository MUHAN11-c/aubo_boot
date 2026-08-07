"""CapturedFrame 数据合约：派生字段 / 默认值 / 显式字段保留."""
import unittest

import numpy as np

from peach_reconstruction_ros2.captured_frame import CapturedFrame


def _make(**overrides):
    """按合约造最小 CapturedFrame，可被 overrides 覆盖任一字（防共享默认列表）."""
    kwargs = {
        'rgb': np.zeros((2, 2, 3), dtype=np.uint8),
        'depth_mm': np.zeros((2, 2), dtype=np.uint16),
        'camera_K': {'fx': 1.0, 'fy': 1.0, 'cx': 0.0, 'cy': 0.0},
        'stamp': 1.5,
        'T_base_camera': np.eye(4),
    }
    kwargs.update(overrides)
    return CapturedFrame(**kwargs)


class CapturedFrameTest(unittest.TestCase):
    def test_camera_position_defaults_to_translation(self):
        """camera_position_base 缺省 = T_base_camera 平移列."""
        T = np.eye(4)
        T[:3, 3] = [1.0, 2.0, 3.0]
        f = _make(T_base_camera=T)
        np.testing.assert_allclose(f.camera_position_base, [1.0, 2.0, 3.0])
        self.assertEqual(f.diagnostic_flags, [])
        self.assertEqual(f.target_id, '')
        self.assertIsNone(f.cloud_rgb)

    def test_explicit_fields_kept(self):
        """显式传入的 cloud_base / cloud_rgb / flags / ratio 原样保留."""
        cloud = np.ones((4, 3))
        colors = np.zeros((4, 3), dtype=np.uint8)
        f = _make(target_id='target_3', valid_depth_ratio=0.7,
                  cloud_base=cloud, cloud_rgb=colors,
                  diagnostic_flags=['tf_stale'])
        self.assertIs(f.cloud_base, cloud)
        self.assertIs(f.cloud_rgb, colors)
        self.assertEqual(f.diagnostic_flags, ['tf_stale'])
        self.assertAlmostEqual(f.valid_depth_ratio, 0.7)
        self.assertEqual(f.target_id, 'target_3')

    def test_default_flags_not_shared(self):
        """diagnostic_flags 默认列表不跨实例共享（dataclass 易踩坑）."""
        f1 = _make()
        f2 = _make()
        f1.diagnostic_flags.append('tf_stale')
        self.assertEqual(f2.diagnostic_flags, [])


if __name__ == '__main__':
    unittest.main()
