"""
几何二次拟合（refit）：合成圆柱/球 5 视角 → TSDF → refit 精度与失败路径.

场景见 synth_scene.py（圆柱与球同心同半径，位姿/渲染器与 TSDF 测试共用）；
被测链：渲染深度 → LocalTsdf 积分 → ROI/降采样/离群剔除（与节点 finalize
后处理一致）→ refine_geometry。open3d 缺失时经 conftest 整文件跳过。

公差依据（2026-08-10 本场景实测，open3d 积分与 RANSAC 均固定种子确定性）：
  - 圆柱：轴误差 0.008° → 阈值 3°；中心误差 0.744mm → 阈值 5mm；
    半径偏 +0.76mm（TSDF 提取固有径向偏置，test_tsdf_volume 同源）→ ±4mm；
  - 球：球心误差 0.954mm → 阈值 5mm；半径偏 +0.76mm → ±4mm；
  - 洁净合成云 inlier=1.000、rmse≈0.13mm，远低于 ACCEPT 门控
    （inlier≥0.35 且 rmse≤5mm），随机噪声云则应 REOBSERVE/REJECT。
"""
import unittest

import numpy as np

from peach_reconstruction_ros2.geometry_refiner import (
    _cylinder_ends,
    orient_axis_bottom_to_neck,
    refine_geometry,
    RefitConfig,
    STATUS_ACCEPT,
    STATUS_REJECT,
)
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
    SPH_R,
    sphere_samples,
)

_TSDF_KW = {'voxel_length': 0.003, 'sdf_trunc': 0.012, 'depth_trunc': 1.5}
_ROI_SIZE = (0.30, 0.30, 0.40)  # 与 config/reconstruction.yaml local_volume 一致


def _tsdf_cloud(samples_fn):
    """复刻节点 finalize 链：渲染 → 积分 → ROI → 体素降采样 → 统计离群剔除."""
    pts, _ = samples_fn()
    volume = LocalTsdf(**_TSDF_KW)
    for T in make_poses():
        depth = render_depth(T, pts)
        rgb = np.full((IMG_H, IMG_W, 3), (0, 200, 0), dtype=np.uint8)
        volume.integrate_frame(rgb, depth, dict(K), T)
    xyz, colors = volume.extract_cloud()
    xyz, colors = LocalTsdf.crop_to_box(xyz, colors, CYL_CENTER, _ROI_SIZE)
    xyz, colors = LocalTsdf.voxel_downsample(xyz, colors, 0.003)
    xyz, _colors = LocalTsdf.statistical_filter(xyz, colors)
    return xyz


class GeometryRefinerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """圆柱/球两套云各积分一次，refit 结果各用例共享."""
        cls.cyl_xyz = _tsdf_cloud(cylinder_samples)
        cls.sph_xyz = _tsdf_cloud(sphere_samples)
        cls.cyl = refine_geometry(cls.cyl_xyz, 'bag', RefitConfig())
        cls.sph = refine_geometry(cls.sph_xyz, 'fruit', RefitConfig())

    # ── 圆柱线（袋桃）────────────────────────────────────────
    def test_cylinder_fit_ok_and_accept(self):
        """洁净合成云拟合成功且 status=ACCEPT（inlier=1.0，rmse≈0.12mm）."""
        self.assertTrue(self.cyl['ok'], f"reason={self.cyl['reason']}")
        self.assertEqual(self.cyl['kind'], 'cylinder')
        self.assertEqual(self.cyl['status'], STATUS_ACCEPT)
        self.assertGreaterEqual(self.cyl['inlier_ratio'], 0.35)
        self.assertLessEqual(self.cyl['rmse'], 0.005)

    def test_cylinder_axis_accuracy(self):
        """轴误差 < 3°（实测 0.008°，GT 轴 [0,0,1]）."""
        cos = abs(self.cyl['axis'] @ np.array([0.0, 0.0, 1.0]))
        err_deg = float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))
        self.assertLess(err_deg, 3.0)

    def test_cylinder_center_accuracy(self):
        """轴段中点 vs GT 轴心 < 5mm（实测 0.744mm）."""
        err_mm = float(np.linalg.norm(self.cyl['center'] - CYL_CENTER)) * 1000.0
        self.assertLess(err_mm, 5.0)

    def test_cylinder_radius_accuracy(self):
        """半径 ±4mm（实测偏 +0.76mm，TSDF 径向偏置同源）."""
        self.assertAlmostEqual(self.cyl['radius'], CYL_R, delta=0.004)

    def test_cylinder_bottom_neck_order(self):
        """方向消歧：neck.z > bottom.z，且底/颈 x,y 落在轴心 5mm 内."""
        self.assertGreater(self.cyl['neck'][2], self.cyl['bottom'][2])
        for end in (self.cyl['bottom'], self.cyl['neck']):
            radial = float(np.linalg.norm(end[:2] - CYL_CENTER[:2]))
            self.assertLess(radial, 0.005)

    # ── 球线（裸桃）─────────────────────────────────────────
    def test_sphere_fit_ok(self):
        """球拟合成功且 status=ACCEPT."""
        self.assertTrue(self.sph['ok'], f"reason={self.sph['reason']}")
        self.assertEqual(self.sph['kind'], 'sphere')
        self.assertEqual(self.sph['status'], STATUS_ACCEPT)

    def test_sphere_center_accuracy(self):
        """球心误差 < 5mm（实测 0.954mm）."""
        err_mm = float(np.linalg.norm(self.sph['center'] - CYL_CENTER)) * 1000.0
        self.assertLess(err_mm, 5.0)

    def test_sphere_radius_accuracy(self):
        """球半径 ±4mm（实测偏 +0.76mm）."""
        self.assertAlmostEqual(self.sph['radius'], SPH_R, delta=0.004)

    def test_sphere_bottom_neck_convention(self):
        """无方向先验时显式退到 +Z，并发布 defaulted 诊断."""
        axis = self.sph['axis']
        self.assertTrue(np.allclose(axis, [0.0, 0.0, 1.0]))
        self.assertTrue(np.allclose(
            self.sph['bottom'], self.sph['center'] - self.sph['radius'] * axis))
        self.assertTrue(np.allclose(
            self.sph['neck'], self.sph['center'] + self.sph['radius'] * axis))
        self.assertGreater(self.sph['neck'][2], self.sph['bottom'][2])
        self.assertIn('fruit_axis_defaulted', self.sph['flags'])

    def test_sphere_uses_bound_perception_axis(self):
        """球面只精化中心/半径，姿态沿用已绑定目标的果梗方向先验."""
        hint = np.array([1.0, 2.0, -1.0])
        expected = hint / np.linalg.norm(hint)
        result = refine_geometry(
            self.sph_xyz, 'fruit', RefitConfig(), axis_hint=hint)
        np.testing.assert_allclose(result['axis'], expected)
        np.testing.assert_allclose(
            result['bottom'], result['center'] - result['radius'] * expected)
        np.testing.assert_allclose(
            result['neck'], result['center'] + result['radius'] * expected)
        self.assertIn('fruit_axis_from_perception', result['flags'])

    # ── 方向消歧 ────────────────────────────────────────────
    def test_axis_flip_invariance(self):
        """初始轴取反输入，bottom/neck/axis 输出不变（仍 bottom→neck）."""
        cyl = self.cyl
        rel = self.cyl_xyz - cyl['axis_point']
        perp = rel - np.outer(rel @ cyl['axis'], cyl['axis'])
        inl = np.where(
            np.abs(np.linalg.norm(perp, axis=1) - cyl['radius']) <= 0.0035)[0]
        a1, b1, n1 = _cylinder_ends(
            self.cyl_xyz[inl], cyl['axis_point'], cyl['axis'])
        a2, b2, n2 = _cylinder_ends(
            self.cyl_xyz[inl], cyl['axis_point'], -cyl['axis'])
        self.assertTrue(np.allclose(a1, a2))
        self.assertTrue(np.allclose(b1, b2))
        self.assertTrue(np.allclose(n1, n2))
        # 消歧原语：指下的轴必取反
        self.assertTrue(np.allclose(
            orient_axis_bottom_to_neck(np.array([0.0, 0.0, -1.0])),
            [0.0, 0.0, 1.0]))

    # ── 优雅失败与缺省语义 ──────────────────────────────────
    def test_empty_cloud_fails_gracefully(self):
        """空云：ok=False / REJECT / empty_cloud，不抛异常."""
        r = refine_geometry(np.zeros((0, 3)), 'bag', RefitConfig())
        self.assertFalse(r['ok'])
        self.assertEqual(r['reason'], 'empty_cloud')
        self.assertEqual(r['status'], STATUS_REJECT)

    def test_tiny_cloud_fails_gracefully(self):
        """少点（5 点，圆柱/球两线）：ok=False，不抛异常."""
        rng = np.random.default_rng(0)
        tiny = rng.normal(size=(5, 3)) * 0.01
        for kind in ('bag', 'fruit'):
            r = refine_geometry(tiny, kind, RefitConfig())
            self.assertFalse(r['ok'])
            self.assertEqual(r['reason'], 'insufficient_points')
            self.assertEqual(r['status'], STATUS_REJECT)

    def test_random_noise_not_accept(self):
        """随机噪声云：即使假设收敛也不得 ACCEPT（REOBSERVE/REJECT）."""
        rng = np.random.default_rng(1)
        noise = rng.normal(size=(500, 3)) * 0.05
        for kind in ('bag', 'fruit'):
            r = refine_geometry(noise, kind, RefitConfig())
            self.assertNotEqual(r['status'], STATUS_ACCEPT)

    def test_unknown_kind_defaults_to_cylinder(self):
        """未知/空 target_kind 缺省袋桃（圆柱线），与感知包语义一致."""
        for kind in ('', 'unknown_kind'):
            r = refine_geometry(self.cyl_xyz, kind, RefitConfig())
            self.assertTrue(r['ok'])
            self.assertEqual(r['kind'], 'cylinder')


if __name__ == '__main__':
    unittest.main()
