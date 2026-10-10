# -*- coding: utf-8 -*-
"""
rembg_u2net 分割档：u2net 显著性分割在 ROI 内精化目标掩膜（对照档）.

ROI 由特征圆（workpiece_center/radius）或连通域包围盒给出；rembg/onnxruntime
不可用或模型下载失败时优雅降级（mask=None，调用方回退 depth_band 结果）。
按目标索引缓存，语义与重构前 ros2_communication 的 _rembg_mask_cache 一致。
"""

from __future__ import annotations

from typing import Any, Optional, Tuple

from ivg_pose_estimation.pipeline.segmenters.base import RefineResult, Segmenter
import numpy as np


def resolve_roi_bbox(
    feature: Optional[Any],
    component_mask: Optional[np.ndarray],
) -> Optional[Tuple[int, int, int, int]]:
    """ROI 提示：特征圆优先，连通域包围盒兜底（x, y, w, h）."""
    center = getattr(feature, 'workpiece_center', None)
    radius = getattr(feature, 'workpiece_radius', 0.0)
    if center and radius and float(radius) > 0:
        cx, cy = center
        r = float(radius)
        return (
            int(round(cx - r)), int(round(cy - r)),
            int(round(r * 2)), int(round(r * 2)),
        )
    if component_mask is None or component_mask.size == 0:
        return None
    ys, xs = np.where(component_mask > 0)
    if ys.size == 0 or xs.size == 0:
        return None
    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    return (x0, y0, x1 - x0 + 1, y1 - y0 + 1)


class RembgU2NetSegmenter(Segmenter):
    """u2net（onnxruntime）显著性分割档，带按索引的结果缓存."""

    name = 'rembg_u2net'

    def __init__(self, model: str = 'u2net', prefer_cuda: bool = True):
        self.model = model
        self.prefer_cuda = prefer_cuda
        self._processor = None
        self._mask_cache = {}
        self._cutout_cache = {}

    # ------------------------------------------------------------------
    def _get_processor(self):
        if self._processor is not None:
            return self._processor
        from ivg_pose_estimation.rembg_processor import RemBGProcessor

        self._processor = RemBGProcessor(model=self.model, prefer_cuda=self.prefer_cuda)
        return self._processor

    # ------------------------------------------------------------------
    def refine(
        self,
        color_bgr: np.ndarray,
        component_mask: Optional[np.ndarray],
        feature: Optional[Any] = None,
        key: Optional[int] = None,
    ) -> RefineResult:
        if key is not None and key in self._mask_cache and key in self._cutout_cache:
            return RefineResult(
                mask=self._mask_cache[key], cutout=self._cutout_cache[key]
            )

        bbox = resolve_roi_bbox(feature, component_mask)
        if bbox is None:
            return RefineResult()

        processor = self._get_processor()
        mask, cutout = processor.process_roi(color_bgr, bbox)
        if mask is None or cutout is None:
            return RefineResult()

        if key is not None:
            self._mask_cache[key] = mask
            self._cutout_cache[key] = cutout
        return RefineResult(mask=mask, cutout=cutout)

    def clear_cache(self) -> None:
        self._mask_cache.clear()
        self._cutout_cache.clear()
