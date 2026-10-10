# -*- coding: utf-8 -*-
"""
dinov2_template 匹配档：DINOv2 嵌入最近邻检索模板 ID（FoundPose/CNOS 范式）.

分工：嵌入负责「模板 ID 检索」（对旋转不敏感、跨外观鲁棒）；面内旋转角由
目标与模板的标准化角度差给出（俯视 4DoF 场景）；深度/位姿合成沿用
PoseEstimator.estimate_pose 几何。离线索引见 pipeline/template_index.py。
"""

from __future__ import annotations

from typing import Any, List, Optional

from ivg_pose_estimation.pipeline.matchers.base import MatchOutcome, Matcher
from ivg_pose_estimation.pipeline.template_index import (
    TemplateEmbeddingIndex,
    normalize_angle_180,
)
import numpy as np


class Dinov2TemplateMatcher(Matcher):
    """DINOv2 嵌入检索匹配档（索引按工件目录惰性构建/载入）."""

    name = 'dinov2_template'

    def __init__(
        self,
        pose_estimator,
        model: str = 'dinov2_vits14',
        device: str = 'auto',
        top_k: int = 1,
        min_similarity: float = 0.5,
    ):
        self.pose_estimator = pose_estimator
        self.top_k = max(1, int(top_k))
        self.min_similarity = float(min_similarity)
        self.index = TemplateEmbeddingIndex(model=model, device=device)
        self._index_object_dir: Optional[str] = None

    @property
    def mode_label(self) -> str:
        return f'{self.name}/{self.index.model_name}'

    # ------------------------------------------------------------------
    def _ensure_index(self, workpiece_template_dir: str) -> bool:
        """按需载入/重建该工件目录的嵌入索引."""
        if self._index_object_dir == workpiece_template_dir and (
            self.index.embeddings is not None
        ):
            return True
        ok = self.index.load(workpiece_template_dir)
        if not ok:
            ok = self.index.build(workpiece_template_dir)
        self._index_object_dir = workpiece_template_dir
        return ok

    def _target_crop(self, feature: Any,
                     target_mask: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """取目标图（特征携带的抠图/预处理图），按掩膜包围盒裁剪去背景."""
        image = getattr(feature, 'color_image', None)
        if image is None:
            return None
        crop = image
        if target_mask is not None and target_mask.shape[:2] == image.shape[:2]:
            ys, xs = np.where(target_mask > 0)
            if ys.size:
                crop = image[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
        return crop if crop.size else None

    # ------------------------------------------------------------------
    def match(
        self,
        feature: Any,
        target_mask: Optional[np.ndarray],
        templates: List[Any],
        workpiece_template_dir: str,
    ) -> MatchOutcome:
        if not templates or not self._ensure_index(workpiece_template_dir):
            return MatchOutcome(best_idx=-1)
        crop = self._target_crop(feature, target_mask)
        if crop is None or crop.size == 0:
            return MatchOutcome(best_idx=-1)

        try:
            embedding = self.index.embed(crop)
        except Exception:  # noqa: BLE001 - 嵌入失败（无模型/网络）判无匹配
            return MatchOutcome(best_idx=-1)
        hits = self.index.query(embedding, top_k=self.top_k)
        if not hits:
            return MatchOutcome(best_idx=-1)

        template_id, similarity, template_angle = hits[0]
        if similarity < self.min_similarity:
            return MatchOutcome(best_idx=-1)

        best_idx = next(
            (i for i, t in enumerate(templates) if getattr(t, 'id', None) == template_id),
            -1,
        )
        if best_idx < 0:
            return MatchOutcome(best_idx=-1)

        target_angle = float(getattr(feature, 'standardized_angle_deg', 0.0) or 0.0)
        angle_diff = normalize_angle_180(target_angle - template_angle)
        return MatchOutcome(
            best_idx=best_idx,
            distance=float(max(0.0, 1.0 - similarity)),
            confidence=float(np.clip(similarity, 0.0, 1.0)),
            best_angle_deg=angle_diff,
            best_aligned_mask=None,
        )
