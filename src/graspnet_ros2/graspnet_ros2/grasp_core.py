# -*- coding: utf-8 -*-
"""
GraspNet 推理纯核：数据容器 + 碰撞检测 + 推理器（无 rclpy 依赖）.

接口风格参考 anygrasp_with_ros（get_grasp(points) → top-K 抓取），后端为
本地 vendored GraspNet-baseline 权重与 graspnet_lib 纯 torch 算子，不需要
AnyGrasp 许可证。get_grasp 内聚完整后处理链：工作区过滤 → 固定点数采样 →
前向 → 解码 → 碰撞检测 → NMS → 按分数排序 → 截取 top-K。

权重加载只走 torch.load(weights_only=True) 安全路径（反序列化受限白名单）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# 17 维抓取向量布局：[score, width, height, depth, R(9 行优先), t(3), object_id]
GRASP_ARRAY_LEN = 17
_IDX_SCORE = 0
_IDX_WIDTH = 1
_IDX_HEIGHT = 2
_IDX_DEPTH = 3
_IDX_ROT = slice(4, 13)
_IDX_TRANS = slice(13, 16)


class Grasp:
    """单个抓取（17 维向量的轻量视图）."""

    def __init__(self, grasp_array: np.ndarray):
        self.grasp_array = np.asarray(grasp_array, dtype=np.float64)

    @property
    def score(self) -> float:
        return float(self.grasp_array[_IDX_SCORE])

    @property
    def width(self) -> float:
        return float(self.grasp_array[_IDX_WIDTH])

    @property
    def height(self) -> float:
        return float(self.grasp_array[_IDX_HEIGHT])

    @property
    def depth(self) -> float:
        return float(self.grasp_array[_IDX_DEPTH])

    @property
    def rotation_matrix(self) -> np.ndarray:
        return self.grasp_array[_IDX_ROT].reshape(3, 3)

    @property
    def translation(self) -> np.ndarray:
        return self.grasp_array[_IDX_TRANS]

    def __repr__(self) -> str:
        return (
            f'Grasp(score={self.score:.3f}, width={self.width:.3f}, '
            f'height={self.height:.3f}, depth={self.depth:.3f}, '
            f'translation={np.round(self.translation, 4).tolist()})'
        )


class GraspList:
    """抓取列表：(M, 17) numpy 数组的容器，支持索引/切片/布尔掩码."""

    def __init__(self, grasp_group_array=None):
        if grasp_group_array is None:
            self.grasp_group_array = np.zeros((0, GRASP_ARRAY_LEN), dtype=np.float64)
        else:
            arr = np.asarray(grasp_group_array, dtype=np.float64)
            if arr.ndim != 2 or arr.shape[1] != GRASP_ARRAY_LEN:
                raise ValueError(
                    f'抓取数组形状应为 (M, {GRASP_ARRAY_LEN})，收到 {arr.shape}'
                )
            self.grasp_group_array = arr

    def __len__(self) -> int:
        return len(self.grasp_group_array)

    def __getitem__(self, item):
        result = self.grasp_group_array[item]
        if isinstance(result, np.ndarray):
            if result.ndim == 2:
                return GraspList(result)
            return Grasp(result)
        return Grasp(result)

    def __repr__(self) -> str:
        head = f'GraspList, count={len(self)}\n'
        if len(self) <= 6:
            return head + ''.join(f'{Grasp(row)}\n' for row in self.grasp_group_array)
        first = ''.join(f'{Grasp(row)}\n' for row in self.grasp_group_array[:3])
        last = ''.join(f'{Grasp(row)}\n' for row in self.grasp_group_array[-3:])
        return head + first + '......\n' + last

    # ---- 批量属性 ----
    @property
    def scores(self) -> np.ndarray:
        return self.grasp_group_array[:, _IDX_SCORE]

    @property
    def widths(self) -> np.ndarray:
        return self.grasp_group_array[:, _IDX_WIDTH]

    @property
    def heights(self) -> np.ndarray:
        return self.grasp_group_array[:, _IDX_HEIGHT]

    @property
    def depths(self) -> np.ndarray:
        return self.grasp_group_array[:, _IDX_DEPTH]

    @property
    def translations(self) -> np.ndarray:
        return self.grasp_group_array[:, _IDX_TRANS]

    @property
    def rotation_matrices(self) -> np.ndarray:
        return self.grasp_group_array[:, _IDX_ROT].reshape(-1, 3, 3)

    # ---- 排序 / NMS ----
    def sort_by_score(self) -> 'GraspList':
        """按分数从高到低排序，原地修改并返回自身."""
        order = np.argsort(self.grasp_group_array[:, _IDX_SCORE])[::-1]
        self.grasp_group_array = self.grasp_group_array[order]
        return self

    def nms(
        self,
        translation_thresh: float = 0.03,
        rotation_thresh: float = 30.0 / 180.0 * np.pi,
    ) -> 'GraspList':
        """
        贪心 NMS：按分数降序保留；平移距离与旋转角差均小于阈值视为重复.

        语义对齐 graspnetAPI 的 nms_grasp（贪心 + 双阈值抑制）。
        """
        gg = self.grasp_group_array
        if len(gg) == 0:
            return GraspList(gg.copy())

        order = np.argsort(-gg[:, _IDX_SCORE])
        translations = gg[:, _IDX_TRANS]
        rotations = gg[:, _IDX_ROT].reshape(-1, 3, 3)

        # 任意两抓取的旋转测地角：trace(R_m^T R_n) = Σ R_m[i,k]·R_n[i,k] → arccos 裁剪
        traces = np.einsum('mik,nik->mn', rotations, rotations)
        rot_angle = np.arccos(np.clip((traces - 1.0) / 2.0, -1.0, 1.0))

        t_dist = np.linalg.norm(translations[:, None, :] - translations[None, :, :], axis=-1)
        conflict = (t_dist < translation_thresh) & (rot_angle < rotation_thresh)

        keep: list[int] = []
        suppressed = np.zeros(len(gg), dtype=bool)
        for idx in order:
            if suppressed[idx]:
                continue
            keep.append(int(idx))
            suppressed |= conflict[idx]
        return GraspList(gg[np.array(keep, dtype=int)])


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
    夹爪几何固定：finger_width=0.01m，finger_length=0.06m。

    Args:
        scene_points: (N, 3) 场景点云。
        voxel_size: 体素边长（下采样 + IoU 体积归一化用）。
    """

    def __init__(self, scene_points, voxel_size: float = 0.005):
        self.finger_width = 0.01
        self.finger_length = 0.06
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


def graspnet_to_ros_rotation(rot: np.ndarray) -> np.ndarray:
    """
    旋转列重排：GraspNet 约定 → ROS 末端约定.

    GraspNet 列含义：col0=approach、col1=width、col2=height；
    ROS 末端约定：X=width、Y=height、Z=approach。
    """
    return np.column_stack([rot[:, 1], rot[:, 2], rot[:, 0]])


@dataclass
class GraspNetConfig:
    """推理配置（launch 参数之外的数值细节）."""

    checkpoint_path: str = ''
    device: str = 'auto'  # 'auto' | 'cuda[:idx]' | 'cpu'
    num_point: int = 20000
    num_view: int = 300
    # 后处理
    collision_thresh: float = 0.01
    voxel_size: float = 0.01
    approach_dist: float = 0.05
    max_gripper_width: float = 0.1
    max_grasps: int = 5
    nms_translation_thresh: float = 0.03
    nms_rotation_thresh_deg: float = 30.0
    # 工作区（点云坐标系下的盒过滤），None 表示不过滤
    workspace: Optional[Tuple[float, float, float, float, float, float]] = None
    # 固定网络超参（与 checkpoint 对应，勿随意改动）
    num_angle: int = 12
    num_depth: int = 4
    cylinder_radius: float = 0.05
    hmin: float = -0.02
    hmax_list: tuple = (0.01, 0.02, 0.03, 0.04)


class GraspNetInference:
    """
    GraspNet 推理器.

    用法：
        inference = GraspNetInference(GraspNetConfig(checkpoint_path=...))
        grasps = inference.get_grasp(points)          # points (N,3) float32，米
    """

    def __init__(self, config: GraspNetConfig):
        self.config = config
        self.device = self._resolve_device()
        self.net = self._load_model()

    # ------------------------------------------------------------------
    def _resolve_device(self):
        import torch

        spec = self.config.device
        if spec in ('', 'auto'):
            return torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        return torch.device(spec)

    def _load_model(self):
        import os

        import torch

        from graspnet_ros2.graspnet_lib import GraspNet

        path = self.config.checkpoint_path
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f'GraspNet 权重不存在: {path!r}')

        net = GraspNet(
            input_feature_dim=0,
            num_view=self.config.num_view,
            num_angle=self.config.num_angle,
            num_depth=self.config.num_depth,
            cylinder_radius=self.config.cylinder_radius,
            hmin=self.config.hmin,
            hmax_list=list(self.config.hmax_list),
        )
        net.to(self.device)
        # 安全加载：weights_only=True 只允许张量/基础类型，防止 pickle 代码执行
        checkpoint = torch.load(path, map_location=self.device, weights_only=True)
        net.load_state_dict(checkpoint['model_state_dict'])
        net.eval()
        return net

    # ------------------------------------------------------------------
    def _crop_workspace(self, points: np.ndarray) -> np.ndarray:
        lims = self.config.workspace
        if lims is None:
            return points
        xmin, xmax, ymin, ymax, zmin, zmax = lims
        mask = (
            (points[:, 0] >= xmin) & (points[:, 0] <= xmax)
            & (points[:, 1] >= ymin) & (points[:, 1] <= ymax)
            & (points[:, 2] >= zmin) & (points[:, 2] <= zmax)
        )
        return points[mask]

    def _sample_points(self, points: np.ndarray) -> np.ndarray:
        """采样/补齐到固定 num_point（与上游 demo 一致：固定种子、不足重复补齐）."""
        n_target = self.config.num_point
        rng = np.random.default_rng(1)
        if len(points) >= n_target:
            idxs = rng.choice(len(points), n_target, replace=False)
        else:
            idxs1 = np.arange(len(points))
            idxs2 = rng.choice(len(points), n_target - len(points), replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        return points[idxs].astype(np.float32)

    # ------------------------------------------------------------------
    def get_grasp(self, points: np.ndarray) -> GraspList:
        """
        对一帧点云做抓取检测.

        Args:
            points: (N, 3) float，点云坐标系下的坐标（米）。

        Returns:
            GraspList：碰撞过滤 + NMS + 分数降序的前 max_grasps 个抓取。
        """
        import torch

        from graspnet_ros2.graspnet_lib import pred_decode

        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        if points.shape[0] == 0:
            return GraspList()

        cropped = self._crop_workspace(points)
        if cropped.shape[0] == 0:
            return GraspList()

        sampled = self._sample_points(cropped)
        cloud_tensor = torch.from_numpy(sampled[np.newaxis]).to(self.device)
        with torch.no_grad():
            end_points = self.net({'point_clouds': cloud_tensor})
            grasp_preds = pred_decode(end_points)
        grasps = GraspList(grasp_preds[0].detach().cpu().numpy())

        # 夹爪开口上限（对齐 anygrasp max_gripper_width 语义）
        if len(grasps) > 0:
            np.clip(
                grasps.grasp_group_array[:, _IDX_WIDTH],
                0.0,
                self.config.max_gripper_width,
                out=grasps.grasp_group_array[:, _IDX_WIDTH],
            )

        # 碰撞检测（用未下采样的工作区点云，与上游一致）
        if self.config.collision_thresh > 0 and len(grasps) > 0:
            detector = ModelFreeCollisionDetector(cropped, voxel_size=self.config.voxel_size)
            collision_mask = detector.detect(
                grasps,
                approach_dist=self.config.approach_dist,
                collision_thresh=self.config.collision_thresh,
            )
            grasps = grasps[~np.asarray(collision_mask, dtype=bool)]

        grasps = grasps.nms(
            translation_thresh=self.config.nms_translation_thresh,
            rotation_thresh=self.config.nms_rotation_thresh_deg / 180.0 * np.pi,
        )
        grasps.sort_by_score()
        return grasps[: self.config.max_grasps]
