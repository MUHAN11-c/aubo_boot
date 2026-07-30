"""Converged single-mode foreground estimation.

The project has converged on one reliable route: MobileSAM mask intersected
with the dilated measured-depth connected component (``hybrid_dilated``).
Pose geometry and tool safety gates live in ``pipeline.py`` and are unchanged
by foreground construction.  This module is the single public path used by
both the GUI and batch validation.

Learned or inpainted depth is deliberately unsupported: every mask is
intersected with valid measured depth before it can reach pose geometry.
When SAM is unavailable the result is an explicit REOBSERVE, never a silent
fallback.
"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Iterable, Optional

import cv2
import numpy as np

from .contracts import BagGrasp2D, BagGraspReference3D, BagObservation
from .pipeline import (
    RobustBagPosePipeline, RobustFruitPosePipeline, TargetPoseResult,
)


@dataclass(frozen=True)
class ForegroundMode:
    mode_id: str
    label: str
    description: str


FOREGROUND_MODES = (
    ForegroundMode("hybrid_dilated", "SAM∩膨胀深度",
                   "SAM mask intersected with dilated measured-depth component"),
)
MODE_IDS = tuple(mode.mode_id for mode in FOREGROUND_MODES)
MODE_LABELS = {mode.mode_id: mode.label for mode in FOREGROUND_MODES}


class CandidateEstimator:
    """Build the converged foreground mask and evaluate it through the safety pipeline.

    Routes by detection class: ``peach_bag`` (0) → cylinder-axis bag pipeline,
    ``peach_nobag`` (1) → sphere + stem-cavity fruit pipeline.  Both lines share
    the same cylindrical cutting tool, entry/travel formulas and safety gates.
    """

    def __init__(self, pipeline: Optional[RobustBagPosePipeline] = None,
                 dilate_px: int = 5, min_mask_points: int = 50):
        self.pipeline = pipeline or RobustBagPosePipeline()
        self.fruit_pipeline = RobustFruitPosePipeline(tool=self.pipeline.tool)
        self.dilate_px = max(1, int(dilate_px))
        self.min_mask_points = max(1, int(min_mask_points))
        self.last_timings_ms: dict[str, float] = {}
        self._last_mask_timings_ms: dict[str, float] = {}

    def _pipeline_for(self, obs: BagObservation) -> RobustBagPosePipeline:
        class_id = obs.detections[0].get("class_id", 0) if obs.detections else 0
        return self.fruit_pipeline if class_id == 1 else self.pipeline

    def estimate_modes(self, obs: BagObservation, target_id: str, bbox: tuple,
                       sam_mask: Optional[np.ndarray],
                       modes: Optional[Iterable[str]] = None
                       ) -> dict[str, TargetPoseResult]:
        """Evaluate requested modes through identical geometry and safety gates."""
        selected = tuple(modes or MODE_IDS)
        unknown = set(selected) - set(MODE_IDS)
        if unknown:
            raise ValueError(f"unknown foreground modes: {sorted(unknown)}")

        masks = self.build_masks(obs, bbox, sam_mask)
        pipeline = self._pipeline_for(obs)
        kind = "fruit" if pipeline is self.fruit_pipeline else "bag"
        results = {}
        self.last_timings_ms = {}
        for mode in selected:
            started = time.perf_counter()
            mask = masks.get(mode)
            if mask is None:
                results[mode] = self._unavailable(
                    obs, target_id, bbox, mode, "mask_unavailable")
            else:
                results[mode] = pipeline.estimate(
                    obs, target_id, bbox, mask, self._source(mode))
            pose = results[mode].grasp_3d
            results[mode].target_kind = kind
            pose.strategy_id = f"robust_{kind}_pose:{mode}"
            pose.model_version = str(obs.metadata.get("model_version", "unknown"))
            pose.calibration_version = str(obs.metadata.get(
                "calibration_version", "unknown"))
            pose.tool_version = self.pipeline.tool.version
            geometry_ms = (time.perf_counter() - started) * 1000.0
            self.last_timings_ms[mode] = (
                self._last_mask_timings_ms.get(mode, 0.0) + geometry_ms)
        return results

    def build_masks(self, obs: BagObservation, bbox: tuple,
                    sam_mask: Optional[np.ndarray]) -> dict[str, Optional[np.ndarray]]:
        """Return the ROI-local measured-depth mask for the converged mode."""
        started = time.perf_counter()
        self._last_mask_timings_ms = {mode: 0.0 for mode in MODE_IDS}
        x1, y1, x2, y2 = self.pipeline._clip_bbox(bbox, obs.depth.shape)
        roi = obs.depth[y1:y2, x1:x2]
        if roi.size == 0:
            return {mode: None for mode in MODE_IDS}
        valid = self.pipeline._valid_depth(roi)
        depth_mask, _ = self.pipeline._foreground(
            roi, valid, None, bbox, source="depth_fallback")

        mask = None
        sam_roi = self._crop_mask(sam_mask, (x1, y1, x2, y2), obs.depth.shape)
        if sam_roi is not None:
            measured_sam = sam_roi & valid
            k = 2 * (self.dilate_px // 2) + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            expanded_depth = cv2.dilate(depth_mask.astype(np.uint8), kernel) > 0
            mask = self._enough(measured_sam & expanded_depth)

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self._last_mask_timings_ms = {mode: elapsed_ms for mode in MODE_IDS}
        return {"hybrid_dilated": mask}

    def _crop_mask(self, mask: Optional[np.ndarray], bbox: tuple,
                   image_shape: tuple) -> Optional[np.ndarray]:
        if mask is None:
            return None
        x1, y1, x2, y2 = bbox
        arr = np.asarray(mask, dtype=bool)
        if arr.shape[:2] == image_shape[:2]:
            arr = arr[y1:y2, x1:x2]
        expected = (y2 - y1, x2 - x1)
        return arr if arr.shape == expected else None

    def _enough(self, mask: np.ndarray) -> Optional[np.ndarray]:
        return mask if int(mask.sum()) >= self.min_mask_points else None

    @staticmethod
    def _source(mode: str) -> str:
        return {
            "hybrid_dilated": "mobile_sam_dilated_depth_intersection",
        }[mode]

    def _unavailable(self, obs: BagObservation, target_id: str, bbox: tuple,
                     mode: str, reason: str) -> TargetPoseResult:
        x1, y1, x2, y2 = map(int, bbox)
        g2d = BagGrasp2D(
            detection_bbox=(x1, y1, x2 - x1, y2 - y1),
            status="REOBSERVE", diagnostic_flags=[reason])
        g3d = BagGraspReference3D(
            status="REOBSERVE", diagnostic_flags=[reason],
            strategy_id=f"robust_bag_pose:{mode}",
            model_version=str(obs.metadata.get("model_version", "unknown")),
            calibration_version=str(obs.metadata.get(
                "calibration_version", "unknown")),
            tool_version=self.pipeline.tool.version)
        return TargetPoseResult(target_id, g2d, g3d, mode, {})
