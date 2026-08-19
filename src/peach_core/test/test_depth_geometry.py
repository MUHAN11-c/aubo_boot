"""
normalize_depth_to_uint16_mm：uint16 缩放 / 32FC1 米制转换 / 无效值与超界处理.

自 peach_pose_ros2/test/test_depth_geometry.py 迁移，行为不变。
"""
import unittest

import numpy as np

from peach_core.depth_geometry import normalize_depth_to_uint16_mm


class NormalizeDepthUint16Test(unittest.TestCase):
    def test_uint16_scaled(self):
        """uint16 × depth_scale_unit：raw×0.25 → 毫米，round 后转回 uint16."""
        depth = np.array([[1000, 2001], [0, 65535]], dtype=np.uint16)
        out = normalize_depth_to_uint16_mm(depth, 0.25)
        self.assertEqual(out.dtype, np.uint16)
        np.testing.assert_array_equal(
            out, np.array([[250, 500], [0, 16384]], dtype=np.uint16))

    def test_uint16_scale_one_passthrough(self):
        """scale=1.0（真毫米数据集）：原样返回（零拷贝），数值不变."""
        depth = np.array([[0, 900], [1234, 65535]], dtype=np.uint16)
        out = normalize_depth_to_uint16_mm(depth, 1.0)
        self.assertIs(out, depth)
        np.testing.assert_array_equal(out, depth)

    def test_uint16_clip_overflow(self):
        """uint16 放大超界时 clip 到 65535."""
        depth = np.array([[65535, 40000]], dtype=np.uint16)
        out = normalize_depth_to_uint16_mm(depth, 2.0)
        np.testing.assert_array_equal(
            out, np.array([[65535, 65535]], dtype=np.uint16))


class NormalizeDepthFloatTest(unittest.TestCase):
    def test_float_meters_to_mm(self):
        """32FC1 按「米」×1000 转毫米，depth_scale_unit 不生效."""
        depth = np.array([[1.0, 0.5], [0.001, 2.0]], dtype=np.float32)
        out = normalize_depth_to_uint16_mm(depth, 0.25)
        self.assertEqual(out.dtype, np.uint16)
        np.testing.assert_array_equal(
            out, np.array([[1000, 500], [1, 2000]], dtype=np.uint16))

    def test_float_nan_negative_inf_to_zero(self):
        """NaN/负值/±Inf 一律置 0（无效深度）."""
        depth = np.array(
            [[np.nan, -1.0], [np.inf, -np.inf]], dtype=np.float32)
        out = normalize_depth_to_uint16_mm(depth, 1.0)
        np.testing.assert_array_equal(out, np.zeros((2, 2), dtype=np.uint16))

    def test_float_clip_to_uint16(self):
        """超 65.535 m 的深度 clip 到 uint16 上限."""
        depth = np.array([[100.0, 65.535]], dtype=np.float64)
        out = normalize_depth_to_uint16_mm(depth, 1.0)
        np.testing.assert_array_equal(
            out, np.array([[65535, 65535]], dtype=np.uint16))

    def test_unsupported_dtype_raises(self):
        """既非 uint16 也非浮点的 dtype 抛 ValueError（调用方丢帧告警）."""
        with self.assertRaises(ValueError):
            normalize_depth_to_uint16_mm(
                np.zeros((2, 2), dtype=np.int16), 1.0)


if __name__ == '__main__':
    unittest.main()
