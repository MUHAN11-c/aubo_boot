"""
TSDF 体积：合成圆柱多视角积分 + 外参方向守门 + ROI 裁剪 + 半径 sanity.

场景与渲染器/GT 见 synth_scene.py（5 环绕位姿 + 点采样 z-buffer 深度 +
面积均匀可见面 GT）。被测链：CapturedFrame → LocalTsdf.integrate_frame
（内部外参取逆）→ extract_point_cloud → 后处理。

公差依据（2026-08-07 本场景实测，open3d 积分全确定性）：
  - 正确方向：1749 点，质心 vs 面积均匀 GT 误差 4.15mm → 阈值 5mm；
  - TSDF 质心 vs raw 装配质心距 3.23mm → 阈值 10mm（加权方式固有差异）；
  - 径向距离中位数 0.03520m（GT r=0.035，偏 +0.2mm）→ ±5mm 裕量充足；
  - 错误方向（不取逆）质心误差 853mm，约为正确方向的 200 倍 → 方向守门
    阈值取 100mm 且 >10× 正确方向误差。
"""
import unittest

import numpy as np

from peach_reconstruction_ros2.cloud_builder import build_cloud_base
from peach_reconstruction_ros2.tsdf_volume import LocalTsdf
from synth_scene import (
    CYL_CENTER,
    CYL_R,
    cylinder_samples,
    IMG_H,
    IMG_W,
    K,
    make_poses,
    render_depth,
    visible_surface_cloud,
)

_TSDF_KW = {'voxel_length': 0.003, 'sdf_trunc': 0.012, 'depth_trunc': 1.5}


def _make_frames():
    """渲染 5 视角帧数据（纯色 BGR 图 + uint16 毫米深度 + 位姿）."""
    pts, _ = cylinder_samples()
    frames = []
    for T in make_poses():
        depth = render_depth(T, pts)
        rgb = np.full((IMG_H, IMG_W, 3), (0, 200, 0), dtype=np.uint8)
        frames.append((rgb, depth, dict(K), T))
    return frames


def _integrate_all(volume, frames, invert=True):
    """批量积分；invert=False 时故意传错方向（world←camera 当 world→camera）."""
    for rgb, depth, K_dict, T_base_camera in frames:
        rgbd, intrinsic = volume._make_rgbd(rgb, depth, K_dict)
        if invert:
            volume.integrate_frame(rgb, depth, K_dict, T_base_camera)
        else:
            volume._integrate(rgbd, intrinsic, T_base_camera)  # 方向错误
    return volume.extract_cloud()


class TsdfIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """正确方向积分一次，各用例共享；GT 为面积均匀可见面质心."""
        cls.frames = _make_frames()
        cls.xyz, cls.colors = _integrate_all(LocalTsdf(**_TSDF_KW), cls.frames)
        cls.gt_centroid = visible_surface_cloud().mean(axis=0)

    def test_cloud_nonempty_and_colored(self):
        """提取云非空（合成圆柱约千点量级）且带颜色通道."""
        self.assertGreater(self.xyz.shape[0], 500)
        self.assertIsNotNone(self.colors)
        self.assertEqual(self.colors.shape, self.xyz.shape)
        self.assertEqual(self.colors.dtype, np.uint8)

    def test_centroid_matches_gt(self):
        """提取云质心 vs 面积均匀 GT < 5mm（实测 4.15mm）."""
        err_mm = float(np.linalg.norm(
            self.xyz.mean(axis=0) - self.gt_centroid)) * 1000.0
        self.assertLess(err_mm, 5.0)

    def test_tsdf_vs_raw_assembly_centroid(self):
        """TSDF 云质心 vs raw 装配云质心 < 10mm（实测 3.23mm）."""
        raw = np.vstack([build_cloud_base(depth, K_dict, T)[0]
                         for _rgb, depth, K_dict, T in self.frames])
        dist_mm = float(np.linalg.norm(
            raw.mean(axis=0) - self.xyz.mean(axis=0))) * 1000.0
        self.assertLess(dist_mm, 10.0)

    def test_extrinsic_direction_guard(self):
        """外参方向守门：不取逆积分 → 质心误差 100mm+（实测 853mm ≈ 200×）."""
        xyz_bad, _ = _integrate_all(LocalTsdf(**_TSDF_KW), self.frames,
                                    invert=False)
        err_bad_mm = float(np.linalg.norm(
            xyz_bad.mean(axis=0) - self.gt_centroid)) * 1000.0
        err_ok_mm = float(np.linalg.norm(
            self.xyz.mean(axis=0) - self.gt_centroid)) * 1000.0
        self.assertGreater(err_bad_mm, 100.0)
        self.assertGreater(err_bad_mm, 10.0 * err_ok_mm)

    def test_radius_sanity(self):
        """提取云到柱轴径向距离中位数 ≈ 0.035m ±5mm（实测偏 +0.2mm）."""
        radial = np.linalg.norm(self.xyz[:, :2] - CYL_CENTER[:2], axis=1)
        self.assertAlmostEqual(float(np.median(radial)), CYL_R, delta=0.005)

    def test_roi_crop(self):
        """ROI 裁剪：柱心框内点全在框内且非空；远处框剔空."""
        size = (0.30, 0.30, 0.40)
        xyz_in, colors_in = LocalTsdf.crop_to_box(
            self.xyz, self.colors, CYL_CENTER, size)
        self.assertGreater(xyz_in.shape[0], 0)
        half = np.asarray(size) / 2.0
        self.assertTrue(
            (np.abs(xyz_in - CYL_CENTER) <= half + 1e-9).all())
        self.assertIsNotNone(colors_in)
        far, far_colors = LocalTsdf.crop_to_box(
            self.xyz, self.colors, CYL_CENTER + 5.0, size)
        self.assertEqual(far.shape[0], 0)
        self.assertEqual(far_colors.shape[0], 0)


if __name__ == '__main__':
    unittest.main()
