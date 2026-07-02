#!/usr/bin/env python3
"""
深度图像预处理模块

简化后的处理流程:
1. 深度阈值二值化 → 工件掩膜
2. 形态学清理（开运算去噪 + 闭运算填洞）
3. 连通域提取 + 面积筛选
4. 可选：彩色图抠出白底工件图像
"""

from __future__ import annotations

import cv2
import numpy as np
from typing import Dict, List, Optional, Tuple
import logging


class Preprocessor:
    """深度图像预处理器"""

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.parameters: Dict[str, float] = {}
        self._init_defaults()

    def _init_defaults(self):
        p = self.parameters
        p["binary_threshold_min"] = 1818.0
        p["binary_threshold_max"] = 2045.0
        p["component_min_area"] = 0.0
        p["component_max_area"] = 100000.0
        p["component_max_count"] = 3.0
        # 形态学清理
        p["morph_open_kernel"] = 3.0       # 开运算核大小（去噪），0=跳过
        p["morph_close_kernel"] = 9.0      # 闭运算核大小（填洞），0=跳过
        p["dilate_mask_kernel"] = 5.0      # 抠图时膨胀掩膜的核大小

    def set_parameters(self, params: Dict[str, float]):
        self.parameters.update(params)

    def get_parameters(self) -> Dict[str, float]:
        return self.parameters.copy()

    def _p(self, key: str, fallback: float) -> float:
        return self.parameters.get(key, fallback)

    # ------------------------------------------------------------------
    def preprocess(
        self,
        depth_image: np.ndarray,
        color_image: Optional[np.ndarray] = None,
        binary_threshold_min: Optional[int] = None,
        binary_threshold_max: Optional[int] = None,
    ) -> Tuple[List[np.ndarray], Optional[np.ndarray]]:
        """预处理主入口。

        Returns:
            (component_masks, preprocessed_color)
        """
        if depth_image is None or depth_image.size == 0:
            return [], None

        if binary_threshold_min is None:
            binary_threshold_min = int(self._p("binary_threshold_min", 1818))
        if binary_threshold_max is None:
            binary_threshold_max = int(self._p("binary_threshold_max", 2045))

        # 1. 深度阈值二值化
        binary = self._threshold(depth_image, binary_threshold_min, binary_threshold_max)

        # 2. 形态学清理
        binary = self._morph_clean(binary)

        # 3. 连通域提取 + 筛选
        masks = self._extract_components(binary)

        # 4. 抠图
        preprocessed_color = None
        if color_image is not None and masks:
            preprocessed_color = self._extract_color(color_image, masks[0])

        return masks, preprocessed_color

    # ------------------------------------------------------------------
    # 内部步骤
    # ------------------------------------------------------------------
    def _threshold(
        self, depth: np.ndarray, lo: int, hi: int
    ) -> np.ndarray:
        """深度阈值二值化：depth ∈ [lo, hi] 且非无效值 → 255"""
        if depth.dtype == np.uint16:
            invalid = (depth == 0) | (depth == 65535)
        elif depth.dtype in (np.float32, np.float64):
            invalid = (depth == 0) | np.isnan(depth)
        else:
            invalid = depth == 0

        binary = np.zeros_like(depth, dtype=np.uint8)
        mask = (depth >= lo) & (depth <= hi) & ~invalid
        binary[mask] = 255
        return binary

    def _morph_clean(self, binary: np.ndarray) -> np.ndarray:
        """形态学清理：开运算去噪 + 闭运算填洞。"""
        open_k = int(self._p("morph_open_kernel", 3))
        if open_k >= 3:
            if open_k % 2 == 0:
                open_k += 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k))
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        close_k = int(self._p("morph_close_kernel", 9))
        if close_k >= 3:
            if close_k % 2 == 0:
                close_k += 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_k, close_k))
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        return binary

    def _extract_components(self, binary: np.ndarray) -> List[np.ndarray]:
        """提取连通域，按面积筛选，返回掩膜列表（面积降序）。"""
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            binary, connectivity=8
        )
        if num_labels <= 1:
            return []

        min_area = self._p("component_min_area", 0)
        max_area = self._p("component_max_area", 100000)
        max_count = int(self._p("component_max_count", 3))

        candidates = []  # (area, label)
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < min_area or area > max_area:
                continue
            candidates.append((area, i))

        candidates.sort(key=lambda x: x[0], reverse=True)
        if max_count > 0:
            candidates = candidates[:max_count]

        masks = []
        for _, label in candidates:
            mask = np.zeros_like(binary, dtype=np.uint8)
            mask[labels == label] = 255
            masks.append(mask)
        return masks

    def _extract_color(
        self, color: np.ndarray, mask: np.ndarray
    ) -> np.ndarray:
        """从彩色图抠出工件区域，白色背景。"""
        h, w = color.shape[:2]
        result = np.full((h, w, 3), 255, dtype=np.uint8)

        dilate_k = int(self._p("dilate_mask_kernel", 5))
        if dilate_k >= 3:
            if dilate_k % 2 == 0:
                dilate_k += 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_k, dilate_k))
            mask = cv2.dilate(mask, kernel, iterations=1)

        result[mask > 0] = color[mask > 0]
        return result
