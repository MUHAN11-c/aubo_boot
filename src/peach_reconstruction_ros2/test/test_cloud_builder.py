"""cloud_builder：反投影解析值对齐 / 无效深度剔除 / 已知 T 点云变换."""
import unittest

import numpy as np

from peach_reconstruction_ros2.cloud_builder import (
    backproject_depth,
    build_cloud_base,
    pack_rgb_bgr,
    transform_points,
    valid_depth_ratio,
)


class BackprojectTest(unittest.TestCase):
    def test_center_and_corners_match_analytic(self):
        """3×3 全 1000 mm 深度 + 已知 K：反投影点与解析值逐点对齐."""
        K = {'fx': 2.0, 'fy': 2.0, 'cx': 1.0, 'cy': 1.0}
        depth = np.full((3, 3), 1000, dtype=np.uint16)  # z = 1 m
        cloud = backproject_depth(depth, K)
        self.assertEqual(cloud.shape, (9, 3))
        # np.nonzero 行主序：(v, u) = (0,0), (0,1), ..., (2,2)
        expected = np.array([
            [-0.5, -0.5, 1.0], [0.0, -0.5, 1.0], [0.5, -0.5, 1.0],
            [-0.5, 0.0, 1.0], [0.0, 0.0, 1.0], [0.5, 0.0, 1.0],
            [-0.5, 0.5, 1.0], [0.0, 0.5, 1.0], [0.5, 0.5, 1.0]])
        np.testing.assert_allclose(cloud, expected, atol=1e-12)

    def test_zero_and_saturated_excluded(self):
        """0 与饱和 65535 视为无效；valid_depth_ratio 同步变化."""
        K = {'fx': 2.0, 'fy': 2.0, 'cx': 1.0, 'cy': 1.0}
        depth = np.full((3, 3), 1000, dtype=np.uint16)
        depth[0, 0] = 0
        depth[1, 1] = 65535
        cloud = backproject_depth(depth, K)
        self.assertEqual(cloud.shape, (7, 3))
        self.assertAlmostEqual(valid_depth_ratio(depth), 7.0 / 9.0)

    def test_stride_downsample(self):
        """stride=2：抽样坐标乘回原图像素后再反投影."""
        K = {'fx': 2.0, 'fy': 2.0, 'cx': 0.0, 'cy': 0.0}
        depth = np.full((4, 4), 2000, dtype=np.uint16)  # z = 2 m
        cloud = backproject_depth(depth, K, stride=2)
        self.assertEqual(cloud.shape, (4, 3))
        np.testing.assert_allclose(cloud[0], [0.0, 0.0, 2.0], atol=1e-12)
        # (u, v) = (2, 0)：x = 2 px × 2 m / 2 = 2 m
        np.testing.assert_allclose(cloud[1], [2.0, 0.0, 2.0], atol=1e-12)


class TransformPointsTest(unittest.TestCase):
    def test_known_T_rotation_and_translation(self):
        """绕 z 转 90° + 平移 (1,2,3)：p_base = R@p + t."""
        T = np.eye(4)
        T[:3, :3] = np.array([[0.0, -1.0, 0.0],
                              [1.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0]])
        T[:3, 3] = [1.0, 2.0, 3.0]
        cloud_camera = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 2.0]])
        cloud_base = transform_points(T, cloud_camera)
        np.testing.assert_allclose(
            cloud_base, [[1.0, 3.0, 3.0], [0.0, 2.0, 5.0]], atol=1e-12)

    def test_build_cloud_base_combines(self):
        """build_cloud_base = 反投影 + 坐标变换 + 有效深度占比（无图时颜色 None）."""
        T = np.eye(4)
        T[:3, 3] = [0.0, 0.0, 1.0]
        K = {'fx': 2.0, 'fy': 2.0, 'cx': 1.0, 'cy': 1.0}
        depth = np.full((3, 3), 1000, dtype=np.uint16)
        cloud, colors, ratio = build_cloud_base(depth, K, T)
        self.assertEqual(cloud.shape, (9, 3))
        self.assertIsNone(colors)
        self.assertAlmostEqual(ratio, 1.0)
        # 中心像素 (u,v)=(1,1)：x=y=0，z=1 m，平移后 z=2 m
        np.testing.assert_allclose(cloud[4], [0.0, 0.0, 2.0], atol=1e-12)


class ColorSamplingTest(unittest.TestCase):
    def test_colors_follow_valid_pixels_row_major(self):
        """左半红右半绿：颜色与有效像素逐点对应（行主序）."""
        K = {'fx': 2.0, 'fy': 2.0, 'cx': 1.0, 'cy': 1.0}
        depth = np.full((3, 4), 1000, dtype=np.uint16)
        depth[0, 0] = 0  # 无效像素不入云，其颜色也不入云
        image = np.zeros((3, 4, 3), dtype=np.uint8)
        image[:, :2] = (0, 0, 255)    # 左半红（BGR）
        image[:, 2:] = (0, 255, 0)    # 右半绿（BGR）
        _cloud, colors, _ratio = build_cloud_base(depth, K, np.eye(4),
                                                  rgb_bgr=image)
        # 12 像素 - 1 无效 = 11 点；行主序：第 0 行有效列为 1,2,3
        self.assertEqual(colors.shape, (11, 3))
        expected = [(0, 255, 0) if u >= 2 else (0, 0, 255)
                    for v in range(3) for u in range(4) if (v, u) != (0, 0)]
        np.testing.assert_array_equal(colors, np.array(expected, dtype=np.uint8))

    def test_pack_rgb_bgr_known_values(self):
        """打包 0xRRGGBB 位模式：已知 BGR 解码回原色."""
        colors = np.array([[0, 0, 255],    # 红（BGR）
                           [0, 255, 0],    # 绿
                           [255, 0, 0],    # 蓝
                           [1, 2, 3]], dtype=np.uint8)
        packed = pack_rgb_bgr(colors)
        self.assertEqual(packed.dtype, np.float32)
        bits = packed.view(np.uint32)
        np.testing.assert_array_equal(
            bits, [0x00FF0000, 0x0000FF00, 0x000000FF, 0x00030201])
        # 解码回原色（RViz RGB8 读取方式）
        r = (bits >> 16) & 0xFF
        g = (bits >> 8) & 0xFF
        b = bits & 0xFF
        np.testing.assert_array_equal(
            np.column_stack([b, g, r]).astype(np.uint8), colors)

    def test_pack_empty(self):
        """空颜色数组 → (0,) float32 空数组."""
        self.assertEqual(pack_rgb_bgr(np.zeros((0, 3), dtype=np.uint8)).shape,
                         (0,))


class ByteOrderTest(unittest.TestCase):
    def test_explicit_little_and_big_endian_depth(self):
        """cv_bridge 零拷贝深度带显式字节序（'<u2'/'>'），结果与原生一致."""
        K = {'fx': 2.0, 'fy': 2.0, 'cx': 1.0, 'cy': 1.0}
        native = np.full((3, 3), 1000, dtype=np.uint16)
        native[0, 0] = 0
        ref, _, _ = build_cloud_base(native, K, np.eye(4))
        # cv_bridge 的 dtype 带显式字节序标记（newbyteorder 构造；
        # 普通 view('<u2') 在同序机器上会被 numpy 归一化为 '='，复现不出）
        little = native.view(np.dtype('<u2').newbyteorder('<'))
        big = native.byteswap().view(np.dtype('>u2').newbyteorder('>'))
        self.assertEqual(little.dtype.byteorder, '<')
        for tagged in (little, big):
            cloud, _, _ = build_cloud_base(tagged, K, np.eye(4))
            np.testing.assert_array_equal(cloud, ref)


if __name__ == '__main__':
    unittest.main()
