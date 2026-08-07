"""
合成圆柱多视角对齐：正例 overlap/质心达标 + 反例外参错误显著恶化.

场景见 synth_scene.py（5 个环绕位姿，相邻平移 ≈43 mm、旋转 ≈12.8°，
落在默认 view_filter 窗口 [20,80]mm / [5,25]deg 内）。输入深度与 GT 均
来自 synth_scene 的独立实现（不 import cloud_builder 内部，避免循环论证）；
被测链为 depth → build_cloud_base → 装配 → overlap 指标。

公差依据（2026-08-07 本场景实测，测试全确定性、无随机）：
  - 正例相邻帧 NN mean ≈0.59 mm / p95 ≈0.92 mm → 阈值 2 mm（>3× 裕量）；
  - 装配质心 vs 射线 GT 误差 ≈0.71 mm → 阈值 5 mm（≈7× 裕量）；
  - 反例（帧 2/3 外参反向 ±20 mm）聚合 mean ≈16.5 mm → 阈值 10 mm；
    健康对与损坏对相差约 25 倍，证明指标对重影敏感。
"""
import unittest

import numpy as np

from peach_reconstruction_ros2.captured_frame import CapturedFrame
from peach_reconstruction_ros2.cloud_builder import build_cloud_base
from peach_reconstruction_ros2.frame_collector import FrameCollector
from peach_reconstruction_ros2.overlap import (
    assembly_overlap_metrics,
    summarize_pairs_mm,
)
from synth_scene import (
    cylinder_samples,
    K,
    make_poses,
    raytrace_cylinder_cloud,
    render_depth,
)


def _assembly_clouds(poses_render, poses_assembly):
    """
    被测链：渲染深度 → build_cloud_base（各自装配位姿）→ 点云列表.

    Args:
        poses_render: 渲染深度用的相机位姿（模拟真实测量，总是正确）.
        poses_assembly: 装配用的 T_base_camera（反例时注入错误外参）.

    Returns
    -------
        cloud_base 列表（每帧 (N, 3) [m]）.

    """
    pts, _ = cylinder_samples()
    clouds = []
    for T_render, T_assembly in zip(poses_render, poses_assembly):
        depth = render_depth(T_render, pts)
        cloud, _colors, _ratio = build_cloud_base(depth, dict(K), T_assembly)
        clouds.append(cloud)
    return clouds


class _FrameStub:
    """最小帧替身：只带 overlap/质心计算所需的 cloud_base."""

    def __init__(self, cloud_base):
        self.cloud_base = cloud_base


class MultiViewAlignmentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """整套场景只建一次：位姿、正例点云、overlap 指标、射线 GT."""
        cls.poses = make_poses()
        cls.clouds = _assembly_clouds(cls.poses, cls.poses)
        cls.metrics = assembly_overlap_metrics(
            [_FrameStub(c) for c in cls.clouds])
        cls.gt_centroid = raytrace_cylinder_cloud().mean(axis=0)

    def test_view_filter_accepts_all_poses(self):
        """5 个位姿逐个过默认 view_filter：全部接受（窗口 [20,80]mm/[5,25]deg）."""
        collector = FrameCollector()
        collector.start('cyl')
        for i, T in enumerate(self.poses):
            ok, reason, trans, rot = collector.check_view(T)
            self.assertTrue(ok, f'pose {i} 被拒: {reason}')
            if i > 0:
                # 构造参数自证：相邻平移/旋转落在 spec 要求区间
                self.assertGreater(trans, 0.030)
                self.assertLess(trans, 0.060)
                self.assertGreater(rot, 10.0)
                self.assertLess(rot, 20.0)
            frame = CapturedFrame(
                rgb=np.zeros((2, 2, 3), dtype=np.uint8),
                depth_mm=np.zeros((2, 2), dtype=np.uint16),
                camera_K={'fx': 1.0, 'fy': 1.0, 'cx': 0.0, 'cy': 0.0},
                stamp=float(i), T_base_camera=T)
            self.assertTrue(collector.add_frame(frame))

    def test_positive_overlap_and_centroid(self):
        """正例：相邻帧 NN mean < 2mm；装配质心 vs 射线 GT < 5mm."""
        for pair in self.metrics['pairs']:
            self.assertLess(pair['mean_mm'], 2.0,
                            f"pair i={pair['i']} mean 超标")
        summary = summarize_pairs_mm(self.metrics['pairs'])
        self.assertLess(summary['mean_mm'], 2.0)
        self.assertLess(summary['p95_mm'], 2.0)
        err_m = np.linalg.norm(
            np.asarray(self.metrics['centroid_base']) - self.gt_centroid)
        self.assertLess(err_m * 1000.0, 5.0)

    def test_negative_corrupted_extrinsics_degrade(self):
        """反例：帧 2/3 外参反向 ±20mm → 聚合 mean > 10mm（重影守门）."""
        bad_poses = [T.copy() for T in self.poses]
        bad_poses[2][:3, 3] += np.array([0.020, 0.0, 0.0])
        bad_poses[3][:3, 3] -= np.array([0.020, 0.0, 0.0])
        # 深度按正确位姿渲染（测量本身没错），仅装配外参错——模拟手眼误差
        clouds_bad = _assembly_clouds(self.poses, bad_poses)
        metrics_bad = assembly_overlap_metrics(
            [_FrameStub(c) for c in clouds_bad])
        summary_bad = summarize_pairs_mm(metrics_bad['pairs'])
        self.assertGreater(summary_bad['mean_mm'], 10.0)


if __name__ == '__main__':
    unittest.main()
