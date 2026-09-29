# -*- coding: utf-8 -*-
"""
模型无关后处理链：工作区过滤 → 开口裁剪 → 碰撞检测 → NMS → 降序 → top-K.

这些步骤不依赖任何具体抓取网络：后端（backends/）只负责「点云 → 原始
GraspList」，本模块把原始结果收敛成可发布的 top-K（无 rclpy 依赖）。
碰撞检测与夹爪几何以本模块的 GripperGeometry 为单一事实源（Marker 渲染同源）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from ivg_graspnet.grasp_core import GraspList
import numpy as np


@dataclass
class GripperGeometry:
    """平行夹爪几何（碰撞判定权威值；visual_* 仅用于 Marker 渲染）."""

    finger_width: float = 0.01
    finger_length: float = 0.06
    # 纯视觉参数（不参与碰撞判定）
    visual_radius: float = 0.005
    visual_depth_base: float = 0.02
    visual_tail_length: float = 0.04


@dataclass
class PostprocessConfig:
    """后处理链参数（节点 ROS 参数透传）."""

    collision_thresh: float = 0.01
    voxel_size: float = 0.01
    approach_dist: float = 0.05
    max_gripper_width: float = 0.1
    max_grasps: int = 5
    nms_translation_thresh: float = 0.03
    nms_rotation_thresh_deg: float = 30.0


def crop_workspace(
    points: np.ndarray,
    workspace: Optional[Tuple[float, float, float, float, float, float]],
) -> np.ndarray:
    """点云坐标系下的盒过滤；workspace=None 表示不过滤."""
    if workspace is None:
        return points
    xmin, xmax, ymin, ymax, zmin, zmax = workspace
    mask = (
        (points[:, 0] >= xmin) & (points[:, 0] <= xmax)
        & (points[:, 1] >= ymin) & (points[:, 1] <= ymax)
        & (points[:, 2] >= zmin) & (points[:, 2] <= zmax)
    )
    return points[mask]


def voxel_down_sample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """
    体素下采样：每体素取均值作为代表点.

    Args:
        points: (N, 3) 点云。
        voxel_size: 体素边长。

    Returns:
        (M, 3) 下采样后的点（M ≤ N）。
    """
    if points.size == 0 or voxel_size <= 0:
        return points.astype(np.float64)

    voxels = np.floor(points / voxel_size).astype(np.int64)
    _, inverse, counts = np.unique(
        voxels, axis=0, return_inverse=True, return_counts=True
    )
    sums = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(sums, inverse, points.astype(np.float64))
    return sums / counts[:, None]


class ModelFreeCollisionDetector:
    """
    无模型碰撞检测器（numpy 实现，无 open3d 依赖）.

    判定逻辑移植自 graspnet-baseline/utils/collision_detector.py（夹爪 AABB
    分区 + 点数/体积 IoU 阈值）；体素下采样由 voxel_down_sample 等价替代。
    夹爪几何来自 GripperGeometry（默认 finger_width=0.01m，finger_length=0.06m）。

    Args:
        scene_points: (N, 3) 场景点云。
        voxel_size: 体素边长（下采样 + IoU 体积归一化用）。
        geometry: 夹爪几何；None 用默认。
    """

    def __init__(self, scene_points, voxel_size: float = 0.005,
                 geometry: Optional[GripperGeometry] = None):
        geom = geometry or GripperGeometry()
        self.finger_width = geom.finger_width
        self.finger_length = geom.finger_length
        self.voxel_size = voxel_size
        self.scene_points = voxel_down_sample(np.asarray(scene_points), voxel_size)

    def detect(
        self,
        grasps: GraspList,
        approach_dist: float = 0.03,
        collision_thresh: float = 0.05,
    ) -> np.ndarray:
        """
        检测每条抓取是否与场景点云碰撞.

        Args:
            grasps: GraspList。
            approach_dist: approach 方向预留通道长度（米），至少为指宽。
            collision_thresh: IoU 碰撞阈值。

        Returns:
            collision_mask (M,) bool。
        """
        approach_dist = max(approach_dist, self.finger_width)
        t = grasps.translations
        rot = grasps.rotation_matrices
        heights = grasps.heights[:, np.newaxis]
        depths = grasps.depths[:, np.newaxis]
        widths = grasps.widths[:, np.newaxis]

        # 场景点变换到每条抓取的局部系（targets @ R：分量 = 与 R 列的点积）
        targets = self.scene_points[np.newaxis, :, :] - t[:, None, :]
        targets = np.matmul(targets, rot)

        mask1 = (targets[:, :, 2] > -heights / 2) & (targets[:, :, 2] < heights / 2)
        mask2 = (targets[:, :, 0] > depths - self.finger_length) & (targets[:, :, 0] < depths)
        mask3 = targets[:, :, 1] > -(widths / 2 + self.finger_width)
        mask4 = targets[:, :, 1] < -widths / 2
        mask5 = targets[:, :, 1] < (widths / 2 + self.finger_width)
        mask6 = targets[:, :, 1] > widths / 2
        mask7 = (
            (targets[:, :, 0] <= depths - self.finger_length)
            & (targets[:, :, 0] > depths - self.finger_length - self.finger_width)
        )
        mask8 = (
            (targets[:, :, 0] <= depths - self.finger_length - self.finger_width)
            & (
                targets[:, :, 0]
                > depths - self.finger_length - self.finger_width - approach_dist
            )
        )

        left_mask = mask1 & mask2 & mask3 & mask4
        right_mask = mask1 & mask2 & mask5 & mask6
        bottom_mask = mask1 & mask3 & mask5 & mask7
        shifting_mask = mask1 & mask3 & mask5 & mask8
        global_mask = left_mask | right_mask | bottom_mask | shifting_mask

        voxel_volume = self.voxel_size ** 3
        left_right_volume = (
            heights * self.finger_length * self.finger_width / voxel_volume
        ).reshape(-1)
        bottom_volume = (
            heights * (widths + 2 * self.finger_width) * self.finger_width / voxel_volume
        ).reshape(-1)
        shifting_volume = (
            heights * (widths + 2 * self.finger_width) * approach_dist / voxel_volume
        ).reshape(-1)
        volume = left_right_volume * 2 + bottom_volume + shifting_volume

        global_iou = global_mask.sum(axis=1) / (volume + 1e-6)
        return global_iou > collision_thresh


def run_postprocess(
    grasps: GraspList,
    scene_points: np.ndarray,
    config: PostprocessConfig,
    geometry: Optional[GripperGeometry] = None,
) -> GraspList:
    """
    完整后处理链：开口裁剪 → 碰撞过滤 → NMS → 降序 → top-K.

    Args:
        grasps: 后端输出的原始 GraspList（未后处理）。
        scene_points: (N, 3) 场景点云（建议为工作区过滤后的原始点云，
            与上游 graspnet-baseline demo 用法一致）。
        config: 后处理参数。
        geometry: 夹爪几何；None 用默认。

    Returns:
        碰撞过滤 + NMS + 分数降序的前 max_grasps 个抓取。
    """
    if len(grasps) == 0:
        return GraspList()

    # 夹爪开口上限（对齐 anygrasp max_gripper_width 语义）
    grasps.clip_widths(config.max_gripper_width)

    # 碰撞检测（用未下采样的工作区点云，与上游一致）
    if config.collision_thresh > 0 and len(grasps) > 0:
        detector = ModelFreeCollisionDetector(
            scene_points, voxel_size=config.voxel_size, geometry=geometry
        )
        collision_mask = detector.detect(
            grasps,
            approach_dist=config.approach_dist,
            collision_thresh=config.collision_thresh,
        )
        grasps = grasps[~np.asarray(collision_mask, dtype=bool)]

    grasps = grasps.nms(
        translation_thresh=config.nms_translation_thresh,
        rotation_thresh=config.nms_rotation_thresh_deg / 180.0 * np.pi,
    )
    grasps.sort_by_score()
    return grasps[: config.max_grasps]
