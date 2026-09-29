# -*- coding: utf-8 -*-
"""
GraspNet 推理纯核：数据契约（Grasp/GraspList）与旋转约定（无 rclpy 依赖）.

历史沿革：碰撞检测/后处理链已拆至 postprocess（模型无关），GraspNet 前向
+ 解码已拆至 backends/graspnet_torch（模型侧）；本模块只保留跨模块共享的
17 维抓取契约。旧路径需要的符号在文件尾 re-export 保持兼容。
"""

from __future__ import annotations

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

    # ---- 修改 ----
    def clip_widths(self, max_width: float) -> 'GraspList':
        """原地夹爪开口裁剪到 [0, max_width]，返回自身."""
        np.clip(
            self.grasp_group_array[:, _IDX_WIDTH],
            0.0,
            max_width,
            out=self.grasp_group_array[:, _IDX_WIDTH],
        )
        return self

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


def graspnet_to_ros_rotation(rot: np.ndarray) -> np.ndarray:
    """
    旋转列重排：GraspNet 约定 → ROS 末端约定.

    GraspNet 列含义：col0=approach、col1=width、col2=height；
    ROS 末端约定：X=width、Y=height、Z=approach。
    """
    return np.column_stack([rot[:, 1], rot[:, 2], rot[:, 0]])


# ============================================================================
# 兼容转发（历史 import 路径：碰撞检测/下采样/工作区过滤已迁 postprocess）。
# 用 PEP 562 __getattr__ 惰性转发，避免 grasp_core ↔ postprocess 循环 import。
# ============================================================================
_POSTPROCESS_FORWARDS = (
    'GripperGeometry',
    'ModelFreeCollisionDetector',
    'PostprocessConfig',
    'crop_workspace',
    'run_postprocess',
    'voxel_down_sample',
)


def __getattr__(name):
    if name in _POSTPROCESS_FORWARDS:
        from ivg_graspnet import postprocess

        return getattr(postprocess, name)
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
