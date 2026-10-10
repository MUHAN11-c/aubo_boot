"""
PF-4 数值不变对拍：BoundedIcp.refine 缓存复用 vs 旧 6×_prepare 路径.

open3d 缺席的环境自动跳过（gate 外仍可在大核/venv 环境复跑）。
"""
from __future__ import annotations

import numpy as np

from peach_harvester.vision.target_reconstruction.integrate import (
    BoundedIcp,
    IcpConfig,
)
import pytest

try:  # open3d 惰性可用性探测（缺库环境逐测跳过，不影响同批其他文件收集）
    import open3d as o3d
    _HAVE_OPEN3D = True
except ImportError:  # pragma: no cover - 依赖环境
    o3d = None
    _HAVE_OPEN3D = False

pytestmark = pytest.mark.skipif(
    not _HAVE_OPEN3D, reason='open3d not available')


def _surfaces(seed: int = 3, n: int = 2500):
    """合成源/目标：目标=源施加小旋转+平移（真值已知，量级在 ICP 界内）."""
    rng = np.random.default_rng(seed)
    z = rng.uniform(0.30, 0.55, size=n)
    theta = rng.uniform(-np.pi, np.pi, size=n)
    radius = 0.05 + rng.normal(0.0, 0.002, size=n)
    source = np.stack(
        [radius * np.cos(theta), radius * np.sin(theta), z], axis=1)
    angle = np.deg2rad(0.8)
    rot = np.array([
        [np.cos(angle), -np.sin(angle), 0.0],
        [np.sin(angle), np.cos(angle), 0.0],
        [0.0, 0.0, 1.0]])
    target = source @ rot.T + np.array([0.004, 0.0, 0.0])
    return source, target


def _reference_refine(icp: BoundedIcp, source, target):
    """旧路径参考实现（PF-4 前：initial 与循环各自 _prepare，共 6 次）."""
    config = icp.config
    identity = np.eye(4, dtype=np.float64)
    o3d_ = o3d
    source_raw = icp._cloud(source)
    target_raw = icp._cloud(target)
    source_fine = icp._prepare(
        source_raw, config.fine_voxel, config.fine_correspondence)
    target_fine = icp._prepare(
        target_raw, config.fine_voxel, config.fine_correspondence)
    initial = o3d_.pipelines.registration.evaluate_registration(
        source_fine, target_fine, config.fine_correspondence, identity)
    correction = identity
    levels = (
        (config.coarse_voxel, config.coarse_correspondence,
         config.coarse_iterations),
        (config.fine_voxel, config.fine_correspondence,
         config.fine_iterations),
    )
    final = initial
    for voxel, correspondence, iterations in levels:
        source_level = icp._prepare(source_raw, voxel, correspondence)
        target_level = icp._prepare(target_raw, voxel, correspondence)
        loss = o3d_.pipelines.registration.TukeyLoss(k=float(correspondence))
        estimator = (
            o3d_.pipelines.registration
            .TransformationEstimationPointToPlane(loss))
        criteria = o3d_.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=int(iterations))
        final = o3d_.pipelines.registration.registration_icp(
            source_level, target_level, float(correspondence),
            correction, estimator, criteria)
        correction = np.asarray(final.transformation, dtype=np.float64)
    from peach_harvester.vision.common.geometry import relative_motion
    translation, rotation = relative_motion(correction, identity)
    return (correction, float(final.fitness), float(final.inlier_rmse),
            translation, rotation)


def test_pf4_refine_numerically_identical_to_reference_path():
    source, target = _surfaces()
    icp = BoundedIcp(IcpConfig(min_points=300))
    ref = _reference_refine(icp, source, target)
    result = icp.refine(source, target)
    # open3d OpenMP 归约序有 ~1e-16 的运行间噪声（参考实现自跑两次也非
    # 逐位相等），逐位断言放宽到 1e-12；系统性差异（缓存键曾漏云身份，
    # 对拍即抓出 1.4e-2 量级漂移）在该容差下不可能通过
    np.testing.assert_allclose(result.correction, ref[0], atol=1e-12, rtol=0.0)
    assert abs(result.fitness - ref[1]) < 1e-12
    assert abs(result.rmse - ref[2]) < 1e-12
    assert abs(result.translation_m - ref[3]) < 1e-12
    assert abs(result.rotation_deg - ref[4]) < 1e-12
    assert result.accepted


def test_pf4_prepare_called_four_times(monkeypatch):
    source, target = _surfaces()
    icp = BoundedIcp(IcpConfig(min_points=300))
    calls = {'n': 0}
    original = BoundedIcp._prepare  # 类访问 staticmethod 即原函数

    def counting(cloud, voxel, correspondence):
        calls['n'] += 1
        return original(cloud, voxel, correspondence)

    monkeypatch.setattr(BoundedIcp, '_prepare', staticmethod(counting))
    icp.refine(source, target)
    # PF-4：initial 的 fine 层被循环第二层复用 → 6 → 4（粗 source/target
    # + fine source/target 各一次）
    assert calls['n'] == 4
