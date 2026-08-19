"""有界 ICP：预热、刚性修正和越界拒绝；E4 复用/全量 target 一致性对照."""
import unittest

import numpy as np

from peach_reconstruction_ros2.cloud_builder import build_cloud_base
from peach_reconstruction_ros2.icp_refiner import (
    BoundedIcp,
    IcpConfig,
    transform_points,
)
from peach_reconstruction_ros2.icp_target_cache import (
    IcpTargetCache,
    IcpTargetRefreshConfig,
)
from peach_reconstruction_ros2.tsdf_volume import LocalTsdf
from synth_scene import (
    cylinder_samples,
    IMG_H,
    IMG_W,
    K,
    make_poses,
    render_depth,
)


def _surface():
    """构造非退化波纹面，避免平面对切向平移不可观."""
    x, y = np.meshgrid(
        np.linspace(-0.05, 0.05, 31),
        np.linspace(-0.04, 0.04, 25))
    z = 0.4 + 0.02 * x + 0.01 * np.sin(50.0 * y)
    return np.column_stack((x.ravel(), y.ravel(), z.ravel()))


class BoundedIcpTest(unittest.TestCase):
    def test_model_warmup_uses_fk(self):
        """模型点不足时不从零猜位姿，直接保留 FK."""
        result = BoundedIcp(IcpConfig()).refine(
            _surface(), np.zeros((10, 3)))
        self.assertEqual(result.mode, 'fk')
        self.assertEqual(result.reason, 'model_warmup')

    def test_small_error_is_refined(self):
        """4 mm FK 平移误差应被 ICP 小范围修正."""
        target = _surface()
        source = target + np.array([0.004, 0.0, 0.0])
        cfg = IcpConfig(min_fitness=0.2, max_rmse=0.01)
        result = BoundedIcp(cfg).refine(source, target)
        self.assertEqual(result.mode, 'icp')
        aligned = transform_points(source, result.correction)
        self.assertLess(np.mean(np.linalg.norm(aligned - target, axis=1)), 0.003)
        self.assertLessEqual(result.translation_m, cfg.max_translation)

    def test_large_correction_is_not_accepted(self):
        """需要超过边界的修正不得作为 ICP 位姿写入 TSDF."""
        target = _surface()
        source = target + np.array([0.03, 0.0, 0.0])
        result = BoundedIcp(IcpConfig()).refine(source, target)
        self.assertNotEqual(result.mode, 'icp')


class IncrementalTargetConsistencyTest(unittest.TestCase):
    """
    E4 正确性红线对照：复用 target 与全量 extract target 的对齐一致.

    场景（synth_scene 合成圆柱，全确定性）：帧 1/2 按真值位姿积分，
    帧 3 装配位姿注入 +3mm 平移误差。
      - 全量路径（旧行为）：TSDF 积分帧 1+2 → 全量 extract 做 target；
      - 复用路径（E4）：TSDF 只积分帧 1 → extract 作基线，帧 2 修正后
        点云增量拼接（IcpTargetCache.append_frame）做 target。
    两路径对帧 3 的有界 ICP 配准结果应一致：修正量差 <2mm / <0.5deg
    （target 形态差异：复用路径帧 2 未经 TSDF 加权平均，密度略不同），
    且都应把注入的 3mm 误差修正回来（≈3mm ± 2mm）。
    """

    def test_reused_target_alignment_matches_full_extract(self):
        """复用 target 配准结果与全量 target 对照（容差 2mm/0.5deg）."""
        pts, _ = cylinder_samples()
        poses = make_poses()
        depths = [render_depth(T, pts) for T in poses[:3]]
        rgb = np.full((IMG_H, IMG_W, 3), (0, 200, 0), dtype=np.uint8)
        tsdf_kw = {'voxel_length': 0.003, 'sdf_trunc': 0.012,
                   'depth_trunc': 1.5}

        # 全量路径：帧 1+2 均入 TSDF，target = 全量 extract
        vol_full = LocalTsdf(**tsdf_kw)
        for i in (0, 1):
            vol_full.integrate_frame(rgb, depths[i], dict(K), poses[i])
        target_full = vol_full.extract_cloud()[0]

        # 复用路径：基线 = TSDF(帧 1) extract；帧 2 点云增量拼接
        vol_reuse = LocalTsdf(**tsdf_kw)
        vol_reuse.integrate_frame(rgb, depths[0], dict(K), poses[0])
        base, _colors = vol_reuse.extract_cloud()
        # min=max=5：钉死周期不刷新，隔离自适应逻辑单测复用语义
        cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=5, max_period=5))
        cache.set_full(base)
        cloud2, _c2, _r2 = build_cloud_base(depths[1], dict(K), poses[1])
        cache.append_frame(cloud2)
        target_reuse = cache.current_target()
        self.assertGreater(target_full.shape[0], 300)
        self.assertGreater(target_reuse.shape[0], 300)

        # 帧 3 注入 +3mm x 平移误差，两种 target 各自配准
        bad_pose = poses[2].copy()
        bad_pose[0, 3] += 0.003
        source, _c3, _r3 = build_cloud_base(depths[2], dict(K), bad_pose)
        cfg = IcpConfig(min_fitness=0.2, max_rmse=0.01)
        icp = BoundedIcp(cfg)
        res_full = icp.refine(source, target_full)
        res_reuse = icp.refine(source, target_reuse)
        self.assertEqual(res_full.mode, 'icp',
                         f'全量路径未接受: {res_full.reason}')
        self.assertEqual(res_reuse.mode, 'icp',
                         f'复用路径未接受: {res_reuse.reason}')
        # 两路径都应把注入的 3mm 误差修正回来
        self.assertAlmostEqual(res_full.translation_m, 0.003, delta=0.002)
        self.assertAlmostEqual(res_reuse.translation_m, 0.003, delta=0.002)
        # 一致性：修正量差远小于 ICP 边界（10mm/3deg）
        self.assertLess(
            abs(res_full.translation_m - res_reuse.translation_m), 0.002)
        self.assertLess(
            abs(res_full.rotation_deg - res_reuse.rotation_deg), 0.5)


if __name__ == '__main__':
    unittest.main()
