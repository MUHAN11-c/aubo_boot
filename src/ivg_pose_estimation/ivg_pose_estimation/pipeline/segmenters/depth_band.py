# -*- coding: utf-8 -*-
"""
depth_band 分割档：直接采用 Preprocessor 深度带阈值产出的连通域掩膜.

轻量兜底档（无模型依赖），与重构前默认行为完全一致。
"""

from __future__ import annotations

from typing import Any, Optional

from ivg_pose_estimation.pipeline.segmenters.base import RefineResult, Segmenter
import numpy as np


class DepthBandSegmenter(Segmenter):
    """透传连通域掩膜."""

    name = 'depth_band'

    def refine(
        self,
        color_bgr: np.ndarray,
        component_mask: Optional[np.ndarray],
        feature: Optional[Any] = None,
        key: Optional[int] = None,
    ) -> RefineResult:
        if component_mask is None or component_mask.size == 0:
            return RefineResult()
        return RefineResult(mask=component_mask.copy(), cutout=None)
