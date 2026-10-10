# -*- coding: utf-8 -*-
"""
geometric 匹配档：包装现行 PoseEstimator 两模式匹配（兜底档）.

mode：
  auto        —— brute_force_matching_enabled 保持 pose_estimator 配置段的值
                 （与重构前行为一致，默认档）
  brute_force —— 强制 IoU ±180° 旋转搜索（置信度语义）
  distance    —— 强制半径比+角度特征距离
模式在构造时一次性写入 PoseEstimator，节点不再逐调用直捣内部属性。
"""

from __future__ import annotations

from typing import Any, List, Optional

from ivg_pose_estimation.pipeline.matchers.base import MatchOutcome, Matcher
import numpy as np


class GeometricMatcher(Matcher):
    """几何匹配（委托 PoseEstimator.select_best_template）."""

    name = 'geometric'

    def __init__(self, pose_estimator, mode: str = 'auto'):
        if mode not in ('auto', 'brute_force', 'distance'):
            raise ValueError(f'未知 geometric 匹配模式: {mode!r}')
        self.pose_estimator = pose_estimator
        self.mode = mode
        if mode == 'brute_force':
            pose_estimator.brute_force_matching_enabled = True
        elif mode == 'distance':
            pose_estimator.brute_force_matching_enabled = False

    @property
    def brute_force(self) -> bool:
        return bool(self.pose_estimator.brute_force_matching_enabled)

    @property
    def mode_label(self) -> str:
        return 'geometric/' + ('brute_force' if self.brute_force else 'distance')

    def match(
        self,
        feature: Any,
        target_mask: Optional[np.ndarray],
        templates: List[Any],
        workpiece_template_dir: str,
    ) -> MatchOutcome:
        result = self.pose_estimator.select_best_template(
            feature,
            target_mask=target_mask,
            workpiece_template_dir=workpiece_template_dir,
        )
        best_idx, distance, confidence, best_angle_deg, best_aligned_mask = result
        return MatchOutcome(
            best_idx=int(best_idx),
            distance=float(distance),
            confidence=(
                None if confidence is None else float(confidence)
            ),
            best_angle_deg=best_angle_deg,
            best_aligned_mask=best_aligned_mask,
        )
