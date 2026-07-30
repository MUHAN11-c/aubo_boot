"""Annotation schema and offline ground-truth metrics."""
from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


VALID_STATUSES = {"ACCEPT", "REOBSERVE", "REJECT"}


def load_annotations(path: Optional[Path]) -> dict[str, list[dict]]:
    """Load JSONL annotations grouped by frame id and validate required fields."""
    if path is None:
        return {}
    path = Path(path)
    rows = []
    if path.suffix.lower() == ".jsonl":
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if line.strip():
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{path}:{line_no}: invalid JSON: {exc}") from exc
    else:
        payload = json.loads(path.read_text(encoding="utf-8"))
        rows = payload["annotations"] if isinstance(payload, dict) else payload
    grouped = defaultdict(list)
    for index, row in enumerate(rows):
        for field in ("frame_id", "target_id", "class_id", "bbox", "expected_status"):
            if field not in row:
                raise ValueError(f"annotation {index} missing {field}")
        if len(row["bbox"]) != 4:
            raise ValueError(f"annotation {index} bbox must have four values")
        if row["expected_status"] not in VALID_STATUSES:
            raise ValueError(f"annotation {index} invalid expected_status")
        grouped[str(row["frame_id"])].append(row)
    return dict(grouped)


def bbox_iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = map(float, a)
    bx1, by1, bx2, by2 = map(float, b)
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    union = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    union += max(0.0, bx2 - bx1) * max(0.0, by2 - by1) - inter
    return inter / union if union > 0 else 0.0


def rasterize_segmentation(annotation: dict, shape: tuple[int, int]) -> Optional[np.ndarray]:
    polygons = annotation.get("segmentation")
    if not polygons:
        return None
    mask = np.zeros(shape, dtype=np.uint8)
    for flat in polygons:
        points = np.asarray(flat, dtype=np.float32).reshape(-1, 2).round().astype(np.int32)
        if len(points) >= 3:
            cv2.fillPoly(mask, [points], 1)
    return mask.astype(bool)


class AnnotationMetrics:
    """Accumulate detector, mask, keypoint and selective-status metrics."""

    def __init__(self, annotations: dict[str, list[dict]], modes: tuple[str, ...],
                 min_match_iou: float = 0.5):
        self.annotations = annotations
        self.modes = modes
        self.min_match_iou = min_match_iou
        self.gt_targets = 0
        self.matched_targets = 0
        self.false_positive_detections = 0
        self.mode = {m: defaultdict(list) for m in modes}

    def add_frame(self, frame_id: str, detections: list[dict],
                  results_by_detection: list[dict], image_shape: tuple[int, int]):
        truth = [a for a in self.annotations.get(frame_id, []) if a["class_id"] == 0]
        self.gt_targets += len(truth)
        unused = set(range(len(detections)))
        matches = []
        for ann in truth:
            scored = sorted(((bbox_iou(ann["bbox"], detections[i]["bbox"]), i)
                             for i in unused), reverse=True)
            if scored and scored[0][0] >= self.min_match_iou:
                _, index = scored[0]
                unused.remove(index)
                matches.append((ann, index))
        self.matched_targets += len(matches)
        self.false_positive_detections += len(unused)

        for ann, det_index in matches:
            gt_mask = rasterize_segmentation(ann, image_shape)
            for mode in self.modes:
                result = results_by_detection[det_index][mode]
                pred_status = result.grasp_3d.status
                expected = ann["expected_status"]
                self.mode[mode]["status_correct"].append(pred_status == expected)
                self.mode[mode]["false_accept"].append(
                    pred_status == "ACCEPT" and expected != "ACCEPT")
                self.mode[mode]["accepted"].append(pred_status == "ACCEPT")
                self._add_point_error(mode, "bottom_px", ann, result.grasp_2d.bottom_px)
                self._add_point_error(mode, "neck_px", ann, result.grasp_2d.neck_px)
                if gt_mask is not None and result.grasp_2d.foreground_mask is not None:
                    x1, y1, x2, y2 = map(int, detections[det_index]["bbox"])
                    pred = np.zeros(image_shape, dtype=bool)
                    local = result.grasp_2d.foreground_mask
                    if local.shape == (y2-y1, x2-x1):
                        pred[y1:y2, x1:x2] = local
                        union = np.logical_or(pred, gt_mask).sum()
                        iou = np.logical_and(pred, gt_mask).sum() / max(union, 1)
                        self.mode[mode]["mask_iou"].append(float(iou))

    def _add_point_error(self, mode, key, ann, predicted):
        target = ann.get(key)
        if target is not None and predicted is not None:
            error = float(np.linalg.norm(np.asarray(predicted) - np.asarray(target)))
            self.mode[mode][f"{key}_error_px"].append(error)

    def report(self) -> dict:
        result = {
            "ground_truth_targets": self.gt_targets,
            "matched_targets": self.matched_targets,
            "detection_recall": self.matched_targets / self.gt_targets if self.gt_targets else None,
            "false_positive_detections": self.false_positive_detections,
            "modes": {},
        }
        for mode, metrics in self.mode.items():
            out = {}
            for key, values in metrics.items():
                if not values:
                    continue
                array = np.asarray(values, dtype=float)
                if key in {"false_accept", "accepted", "status_correct"}:
                    out[key + "_count"] = int(array.sum())
                    out[key + "_rate"] = float(array.mean())
                else:
                    out[key] = {
                        "n": len(values), "median": float(np.median(array)),
                        "p95": float(np.percentile(array, 95)),
                    }
            false_accepts = int(np.sum(metrics.get("false_accept", [])))
            accepted = int(np.sum(metrics.get("accepted", [])))
            if accepted:
                from scipy.stats import beta
                upper = (1.0 if false_accepts >= accepted else
                         float(beta.ppf(0.95, false_accepts + 1,
                                        accepted - false_accepts)))
                out["false_accept_given_accept_rate"] = false_accepts / accepted
                out["false_accept_95pct_upper"] = upper
                out["offline_safety_gate_pass"] = upper < 0.01
            else:
                out["false_accept_given_accept_rate"] = None
                out["false_accept_95pct_upper"] = None
                out["offline_safety_gate_pass"] = False
            result["modes"][mode] = out
        return result
