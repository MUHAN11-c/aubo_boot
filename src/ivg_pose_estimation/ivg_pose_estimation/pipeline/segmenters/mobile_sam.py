# -*- coding: utf-8 -*-
"""
mobile_sam 分割档：MobileSAM（39M 轻量 SAM）点提示分割在 ROI 语义下精化掩膜.

SAM-6D / BOP 系把「学习式分割」作为位姿流水线起点；本档用 ultralytics
8.3 API + mobile_sam.pt（点提示，labels=[1] 前景），在 3090 上单次 <0.5s。
提示点取特征圆心（无特征时用连通域掩膜质心）；输出与 rembg_u2net 档同构
（全图掩膜 + 白底抠图），下游 Matcher/PoseEstimator 无感切换。
"""

from __future__ import annotations

import os
from typing import Any, Optional

from ivg_pose_estimation.pipeline.segmenters.base import RefineResult, Segmenter
import numpy as np

_WEIGHTS_CANDIDATES = ('mobile_sam.pt',)


def resolve_mobile_sam_weights(weights: str = '') -> str:
    """权重解析：绝对路径 > 相对包 models/（源码树 > ament share）."""
    if weights and os.path.isabs(weights) and os.path.exists(weights):
        return weights
    candidates = []
    pkg_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))  # <pkg_root>/ivg_pose_estimation/pipeline/segmenters
    candidates.append(os.path.join(pkg_root, 'models', weights or 'mobile_sam.pt'))
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(os.path.join(
            get_package_share_directory('ivg_pose_estimation'),
            'models', weights or 'mobile_sam.pt',
        ))
    except Exception:  # noqa: BLE001 - 无 ament 环境（pytest）时跳过
        pass
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return ''


class MobileSamSegmenter(Segmenter):
    """MobileSAM 点提示分割档（ultralytics API，模型惰性加载）."""

    name = 'mobile_sam'

    def __init__(self, weights: str = '', device: str = 'auto'):
        self.weights_spec = weights
        self.device = device
        self._model = None
        self._load_error: Optional[Exception] = None

    # ------------------------------------------------------------------
    def _get_model(self):
        if self._model is not None or self._load_error is not None:
            return self._model
        try:
            from ultralytics import SAM
        except Exception as exc:  # noqa: BLE001 - venv 无 ultralytics 时优雅降级
            self._load_error = exc
            return None
        weights = resolve_mobile_sam_weights(self.weights_spec)
        if not weights:
            self._load_error = FileNotFoundError(
                f'mobile_sam 权重未找到（models/{self.weights_spec or "mobile_sam.pt"}；'
                '用 models/fetch_mobile_sam.sh 获取）'
            )
            return None
        # device 在 predict 时传入（ultralytics 8.3 SAM 构造器无 device 参数）
        self._model = SAM(weights)
        return self._model

    # ------------------------------------------------------------------
    @staticmethod
    def _prompt_point(feature: Optional[Any],
                      component_mask: Optional[np.ndarray]) -> Optional[tuple]:
        center = getattr(feature, 'workpiece_center', None)
        if center:
            return int(round(float(center[0]))), int(round(float(center[1])))
        if component_mask is not None and component_mask.size:
            ys, xs = np.where(component_mask > 0)
            if ys.size:
                return int(xs.mean()), int(ys.mean())
        return None

    # ------------------------------------------------------------------
    def refine(
        self,
        color_bgr: np.ndarray,
        component_mask: Optional[np.ndarray],
        feature: Optional[Any] = None,
        key: Optional[int] = None,
    ) -> RefineResult:
        model = self._get_model()
        if model is None:
            return RefineResult()
        prompt = self._prompt_point(feature, component_mask)
        if prompt is None:
            return RefineResult()

        try:
            kwargs = {'verbose': False}
            if self.device not in ('', 'auto'):
                kwargs['device'] = self.device
            results = model(
                np.ascontiguousarray(color_bgr),
                points=[[prompt[0], prompt[1]]],
                labels=[1],
                **kwargs,
            )
        except Exception:  # noqa: BLE001 - 推理失败走兜底
            return RefineResult()

        masks = getattr(results[0], 'masks', None) if results else None
        if masks is None or masks.data is None or len(masks.data) == 0:
            return RefineResult()
        mask_array = masks.data[0].cpu().numpy().astype(np.uint8)
        # 归一到全图尺寸 + 0/255
        if mask_array.shape[:2] != color_bgr.shape[:2]:
            import cv2

            mask_array = cv2.resize(
                mask_array,
                (color_bgr.shape[1], color_bgr.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
        full_mask = np.where(mask_array > 0, 255, 0).astype(np.uint8)
        if int(full_mask.sum()) == 0:
            return RefineResult()

        # 白底抠图（与 rembg_u2net 档同构，供特征/匹配使用）
        mask_bool = full_mask > 0
        cutout = np.full_like(color_bgr, 255)
        cutout[mask_bool] = color_bgr[mask_bool]
        return RefineResult(mask=full_mask, cutout=cutout)
