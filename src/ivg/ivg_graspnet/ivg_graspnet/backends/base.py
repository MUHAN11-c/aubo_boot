# -*- coding: utf-8 -*-
"""
抓取检测后端协议.

任何模型只要实现 detect(points_m) → 原始 GraspList，即可经 registry 注册后
由节点按 `backend` 参数选用。后端只负责「点云 → 原始抓取」（含模型特有的
预处理/解码/约定适配），不负责碰撞/NMS/topK——那属于模型无关的 postprocess。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ivg_graspnet.grasp_core import GraspList
import numpy as np


@dataclass
class BackendInfo:
    """后端元信息（节点日志与运动侧约定）."""

    name: str
    description: str = ''
    # 运动执行侧约定：该后端输出的抓取姿态在执行前是否需要绕 approach（Z）
    # 轴做 180° 修正。GraspNet 家族输出的 approach 与本区末端约定相反
    # （历史补丁 _apply_grasp_z_flip_180），graspnet 系后端置 True；
    # 输出已符合末端约定的新后端置 False（执行端据此跳过翻转）。
    approach_flip_z180: bool = False


class GraspBackend:
    """抓取检测后端基类（非抽象强制，鸭子类型亦可）."""

    def __init__(self, info: BackendInfo):
        self.info = info

    def detect(self, points_m: np.ndarray,
               rgb: Optional[np.ndarray] = None) -> GraspList:
        """
        对一帧点云做抓取检测（原始结果，未后处理）.

        Args:
            points_m: (N, 3) float，点云坐标系下的坐标（米），已过工作区过滤。
            rgb: (N, 3) uint8 或 (H, W, 3) 可选颜色；后端可忽略。

        Returns:
            GraspList（decode 后的原始抓取，未经碰撞/NMS/topK）。
        """
        raise NotImplementedError

    def close(self) -> None:
        """释放资源（显存等）；默认无操作."""
        pass

    def __repr__(self) -> str:
        return f'<{type(self).__name__} {self.info.name}: {self.info.description}>'
