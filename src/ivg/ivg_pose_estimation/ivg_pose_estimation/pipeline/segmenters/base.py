# -*- coding: utf-8 -*-
"""
分割段后端协议：把「初始连通域掩膜」精化为「最终目标掩膜」.

流水线定位（FoundPose/SAM-6D 式分段）：
  Preprocessor(深度带连通域) → FeatureExtractor → **Segmenter（本协议）**
  → Matcher → PoseEstimator.estimate_pose(位姿合成)

depth_band 档直接透传连通域掩膜；rembg_u2net / mobile_sam 档用学习式
分割在 ROI 内精化。段与段之间只经本协议交换 (mask, cutout)。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np


@dataclass
class RefineResult:
    """精化结果（均为全图尺寸；None 表示该段未产出，调用方走兜底）."""

    mask: Optional[np.ndarray] = None      # uint8 0/255 目标掩膜
    cutout: Optional[np.ndarray] = None    # 白底抠图 BGR（模板匹配/可视化用）


class Segmenter:
    """分割段基类（鸭子类型亦可；实现 detect 式 refine 即可入注册表）."""

    name = 'base'

    def refine(
        self,
        color_bgr: np.ndarray,
        component_mask: Optional[np.ndarray],
        feature: Optional[Any] = None,
        key: Optional[int] = None,
    ) -> RefineResult:
        """
        精化单个目标.

        Args:
            color_bgr: 全图 BGR。
            component_mask: Preprocessor 给出的该目标连通域掩膜（可 None）。
            feature: ComponentFeature（用 workpiece_center/radius 提供 ROI 提示）。
            key: 目标索引（供带缓存的实现复用；None 表示不缓存）。

        Returns:
            RefineResult；mask=None 时调用方回退 component_mask。
        """
        raise NotImplementedError

    def clear_cache(self) -> None:
        """清空实现内部缓存（一次 estimate_pose 请求开始时调用）."""
        pass
