"""
FK 初值约束下的帧到模型 ICP（纯算法层，Open3D 懒加载）.

输入 source 已由机器人 FK 变换到 base 系，target 是当前 TSDF 表面；
ICP 因而只估计一个接近单位阵的小修正 delta_T，最终相机位姿为
T_refined = delta_T @ T_fk。质量或修正边界不满足时，若 FK 本身与模型
一致则回退 FK，否则拒绝该帧，防止错误配准污染不可回滚的 TSDF。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from peach_reconstruction_ros2.tf_utils import relative_motion
from peach_reconstruction_ros2.tsdf_volume import require_open3d


@dataclass(frozen=True)
class IcpConfig:
    """两尺度点到平面 ICP 参数，长度单位均为米."""

    min_points: int = 300
    coarse_voxel: float = 0.006
    fine_voxel: float = 0.003
    coarse_correspondence: float = 0.015
    fine_correspondence: float = 0.007
    coarse_iterations: int = 20
    fine_iterations: int = 10
    min_fitness: float = 0.35
    max_rmse: float = 0.008
    max_translation: float = 0.010
    max_rotation_deg: float = 3.0


@dataclass(frozen=True)
class IcpResult:
    """一次帧到模型配准结果."""

    mode: str
    correction: np.ndarray
    fitness: float
    rmse: float
    translation_m: float
    rotation_deg: float
    reason: str

    @property
    def accepted(self) -> bool:
        """ICP 或 FK 预对齐通过质量门即允许积分."""
        return self.mode in ('icp', 'fk')


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """对 (N,3) 点应用齐次刚体变换，不修改输入."""
    xyz = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    T = np.asarray(transform, dtype=np.float64)
    return xyz @ T[:3, :3].T + T[:3, 3]


def _quality_ok(fitness: float, rmse: float, config: IcpConfig) -> bool:
    """统一的重叠度/RMSE 门，避免 ICP 与 FK 回退采用两套语义."""
    return fitness >= config.min_fitness and rmse <= config.max_rmse


class BoundedIcp:
    """Open3D 鲁棒点到平面 ICP；机器人 FK 是绝对位姿，ICP 只做小修正."""

    def __init__(self, config: IcpConfig):
        """保存不可变配置；Open3D 到第一次 refine 时才导入."""
        self.config = config

    @staticmethod
    def _cloud(points):
        """Numpy 点集转 Open3D PointCloud."""
        o3d = require_open3d()
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(
            np.asarray(points, dtype=np.float64).reshape(-1, 3))
        return cloud

    @staticmethod
    def _prepare(cloud, voxel: float, correspondence: float):
        """降采样并估计点到平面 ICP 所需法向."""
        o3d = require_open3d()
        down = cloud.voxel_down_sample(float(voxel))
        radius = max(2.0 * float(voxel), 2.0 * float(correspondence))
        down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        return down

    def refine(self, source_fk_base: np.ndarray,
               target_base: np.ndarray) -> IcpResult:
        """
        配准当前 FK 点云到 TSDF 模型.

        模型尚未形成时返回 mode=fk 用于首帧/预热；模型形成后先评估
        FK 原位姿，再做粗细两层 ICP。ICP 越界或质量差但 FK 原位姿合格时
        返回 mode=fk；二者都差则 mode=reject。
        """
        source = np.asarray(source_fk_base, dtype=np.float64).reshape(-1, 3)
        target = np.asarray(target_base, dtype=np.float64).reshape(-1, 3)
        identity = np.eye(4, dtype=np.float64)
        if len(source) < self.config.min_points:
            return IcpResult(
                'reject', identity, -1.0, -1.0, 0.0, 0.0,
                'insufficient_source_points')
        if len(target) < self.config.min_points:
            return IcpResult(
                'fk', identity, -1.0, -1.0, 0.0, 0.0, 'model_warmup')

        o3d = require_open3d()
        source_raw = self._cloud(source)
        target_raw = self._cloud(target)
        source_fine = self._prepare(
            source_raw, self.config.fine_voxel,
            self.config.fine_correspondence)
        target_fine = self._prepare(
            target_raw, self.config.fine_voxel,
            self.config.fine_correspondence)
        initial = o3d.pipelines.registration.evaluate_registration(
            source_fine, target_fine,
            self.config.fine_correspondence, identity)

        correction = identity
        levels = (
            (self.config.coarse_voxel,
             self.config.coarse_correspondence,
             self.config.coarse_iterations),
            (self.config.fine_voxel,
             self.config.fine_correspondence,
             self.config.fine_iterations),
        )
        final = initial
        for voxel, correspondence, iterations in levels:
            source_level = self._prepare(source_raw, voxel, correspondence)
            target_level = self._prepare(target_raw, voxel, correspondence)
            loss = o3d.pipelines.registration.TukeyLoss(
                k=float(correspondence))
            estimator = (
                o3d.pipelines.registration.TransformationEstimationPointToPlane(
                    loss))
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=int(iterations))
            final = o3d.pipelines.registration.registration_icp(
                source_level, target_level, float(correspondence),
                correction, estimator, criteria)
            correction = np.asarray(final.transformation, dtype=np.float64)

        translation, rotation = relative_motion(correction, identity)
        icp_quality = _quality_ok(
            float(final.fitness), float(final.inlier_rmse), self.config)
        bounded = (translation <= self.config.max_translation
                   and rotation <= self.config.max_rotation_deg)
        if icp_quality and bounded:
            return IcpResult(
                'icp', correction, float(final.fitness),
                float(final.inlier_rmse), translation, rotation, 'accepted')

        if _quality_ok(
                float(initial.fitness), float(initial.inlier_rmse),
                self.config):
            reason = 'icp_out_of_bounds' if not bounded else 'icp_low_quality'
            return IcpResult(
                'fk', identity, float(initial.fitness),
                float(initial.inlier_rmse), 0.0, 0.0, reason)

        reason = 'icp_out_of_bounds' if not bounded else 'low_overlap'
        return IcpResult(
            'reject', correction, float(final.fitness),
            float(final.inlier_rmse), translation, rotation, reason)


# 显式注册表：编排层按名字选实现，避免在节点中写算法条件分支。
POSE_REFINERS = {'open3d_bounded': BoundedIcp}
