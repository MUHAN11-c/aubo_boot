# -*- coding: utf-8 -*-
"""
匹配段后端协议：从模板库中检索目标对应的最佳模板.

流水线定位：Segmenter（目标掩膜）→ **Matcher（本协议）** →
PoseEstimator.estimate_pose（用匹配到的模板合成抓取/放置位姿）。

geometric 档=现行的「特征距离 + IoU 暴力旋转搜索」两模式；dinov2_template
档=DINOv2 嵌入最近邻检索（FoundPose/CNOS 范式）。MatchOutcome 与现行
select_best_template 返回的五元组语义一一对应。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Optional

import numpy as np


@dataclass
class MatchOutcome:
    """单目标匹配结果（best_idx=-1 表示无匹配）."""

    best_idx: int = -1
    distance: float = float('inf')            # 特征距离（distance 模式）
    confidence: Optional[float] = None        # IoU 置信度（暴力/embedding 模式）
    best_angle_deg: Optional[float] = None    # 面内旋转角（度）
    best_aligned_mask: Optional[np.ndarray] = None  # 对齐后的模板掩膜（调试用）


class Matcher:
    """匹配段基类（鸭子类型亦可）."""

    name = 'base'

    @property
    def mode_label(self) -> str:
        """人读模式标签（节点日志用）."""
        return self.name

    def match(
        self,
        feature: Any,
        target_mask: Optional[np.ndarray],
        templates: List[Any],
        workpiece_template_dir: str,
    ) -> MatchOutcome:
        """
        对单个目标的掩膜/特征检索最佳模板.

        Args:
            feature: ComponentFeature。
            target_mask: Segmenter 产出的目标掩膜（可 None，distance 模式可无掩膜）。
            templates: 已加载模板列表（PoseEstimator.templates）。
            workpiece_template_dir: 模板目录（惰性加载掩膜用）。
        """
        raise NotImplementedError
