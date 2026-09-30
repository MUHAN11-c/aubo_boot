"""
YOLO bag detector (no ROS).

`Detector` holds the post-processing (clip, class check, cross-class de-duplication) and is
testable with a fake backend. `UltralyticsYoloBackend` is the only real backend; torch and
ultralytics are imported lazily so this module imports under the system python.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
from typing import Protocol, Sequence

import numpy as np


@dataclass(frozen=True)
class Detection:
    bbox: tuple[int, int, int, int]  # x1, y1, x2, y2 [px]; right/bottom exclusive
    class_id: int
    class_name: str
    score: float

    @property
    def width(self) -> int:
        return self.bbox[2] - self.bbox[0]

    @property
    def height(self) -> int:
        return self.bbox[3] - self.bbox[1]

    @property
    def area(self) -> int:
        return self.width * self.height

    @property
    def centre(self) -> tuple[float, float]:
        return 0.5 * (self.bbox[0] + self.bbox[2]), 0.5 * (self.bbox[1] + self.bbox[3])


class YoloBackend(Protocol):
    names: dict[int, str]
    device: str

    def predict(self, bgr: np.ndarray, conf: float, iou: float) -> np.ndarray:
        """(N, 6) float array [x1, y1, x2, y2, score, class_id] in image pixels."""
        ...


def clip_box(xyxy: Sequence[float],
             shape: tuple[int, ...]) -> tuple[int, int, int, int] | None:
    """Round outward to integer pixels and clip to the image; None when empty."""
    h, w = int(shape[0]), int(shape[1])
    x1 = int(np.clip(np.floor(xyxy[0]), 0, w))
    y1 = int(np.clip(np.floor(xyxy[1]), 0, h))
    x2 = int(np.clip(np.ceil(xyxy[2]), 0, w))
    y2 = int(np.clip(np.ceil(xyxy[3]), 0, h))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def box_intersection(a: Sequence[int], b: Sequence[int]) -> int:
    iw = min(a[2], b[2]) - max(a[0], b[0])
    ih = min(a[3], b[3]) - max(a[1], b[1])
    return max(iw, 0) * max(ih, 0)


def box_iou(a: Sequence[int], b: Sequence[int]) -> float:
    inter = box_intersection(a, b)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def dedup_overlapping(dets: Sequence[Detection], ios_threshold: float,
                      frag_ios_threshold: float, frag_area_ratio: float) -> list[Detection]:
    """
    Drop lower-score boxes that duplicate a kept box across classes.

    YOLO NMS is per class, so one bag can come out as both peach_bag and peach_nobag. A box is
    dropped when its intersection over the smaller box (IoS) with a kept, higher-score box is
    >= ios_threshold, or when it is a fragment: IoS >= frag_ios_threshold and its area is
    <= frag_area_ratio of the kept box (a partial box inside a full one).
    """
    kept: list[Detection] = []
    for d in sorted(dets, key=lambda x: x.score, reverse=True):
        duplicate = False
        for k in kept:
            inter = box_intersection(d.bbox, k.bbox)
            small = min(d.area, k.area)
            ios = inter / small if small > 0 else 0.0
            if ios >= ios_threshold:
                duplicate = True
            elif (frag_area_ratio > 0.0 and ios >= frag_ios_threshold
                  and d.area <= frag_area_ratio * k.area):
                duplicate = True
            if duplicate:
                break
        if not duplicate:
            kept.append(d)
    return kept


def weights_path(model_path: str, engine_path: str) -> str:
    """Pick the TensorRT engine when configured and present, else the .pt weights."""
    if engine_path and os.path.isfile(engine_path):
        return engine_path
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f'YOLO weights not found: {model_path}')
    return model_path


def resolve_device(requested: str, allow_cpu: bool) -> str:
    """
    Torch device for inference; refuses a silent CPU fallback unless allow_cpu.

    A CUDA request without a usable GPU raises unless allow_cpu (CPU inference is ~20x slower
    and silently degraded the old stack's frame rate).
    """
    import torch
    if requested.startswith('cuda'):
        if torch.cuda.is_available():
            return requested
        if not allow_cpu:
            raise RuntimeError(f'{requested} requested but CUDA is unavailable '
                               '(set allow_cpu: true to run on CPU)')
        return 'cpu'
    return requested


class UltralyticsYoloBackend:
    """Ultralytics YOLO (.pt or TensorRT .engine); loaded eagerly and warmed up."""

    def __init__(self, model_path: str, engine_path: str, device: str, allow_cpu: bool,
                 imgsz: int, half: bool) -> None:
        from ultralytics import YOLO
        self.weights = weights_path(model_path, engine_path)
        self.device = resolve_device(device, allow_cpu)
        self._imgsz = int(imgsz)
        # FP16 on CPU is unsupported by torch conv kernels.
        self._half = bool(half) and self.device != 'cpu'
        self._model = YOLO(self.weights, task='detect')
        if not self.weights.endswith('.engine'):
            self._model.to(self.device)
        self.names = {int(k): str(v) for k, v in dict(self._model.names).items()}
        self.predict(np.zeros((self._imgsz, self._imgsz, 3), dtype=np.uint8), 0.5, 0.5)

    def predict(self, bgr: np.ndarray, conf: float, iou: float) -> np.ndarray:
        results = self._model.predict(bgr, conf=conf, iou=iou, imgsz=self._imgsz,
                                      half=self._half, device=self.device, verbose=False)
        rows = []
        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue
            xyxy = r.boxes.xyxy.detach().cpu().numpy().reshape(-1, 4)
            score = r.boxes.conf.detach().cpu().numpy().reshape(-1, 1)
            cls = r.boxes.cls.detach().cpu().numpy().reshape(-1, 1)
            rows.append(np.hstack([xyxy, score, cls]))
        if not rows:
            return np.zeros((0, 6))
        return np.vstack(rows).astype(np.float64)


class Detector:
    def __init__(self, backend: YoloBackend, conf: float, iou: float,
                 class_names: Sequence[str], dedup_ios: float, dedup_frag_ios: float,
                 dedup_area_ratio: float) -> None:
        self._backend = backend
        self._conf = float(conf)
        self._iou = float(iou)
        self._class_names = list(class_names)
        self._dedup = (float(dedup_ios), float(dedup_frag_ios), float(dedup_area_ratio))

    @property
    def device(self) -> str:
        return self._backend.device

    def check_class_names(self) -> None:
        """Raise when the weights' class table differs from the configured one."""
        names = self._backend.names
        expected = dict(enumerate(self._class_names))
        if names != expected:
            raise ValueError(f'model classes {names} != configured {expected}')

    def detect(self, bgr: np.ndarray) -> list[Detection]:
        """Detect bags; results sorted by score (descending), clipped and de-duplicated."""
        raw = np.asarray(self._backend.predict(bgr, self._conf, self._iou), dtype=np.float64)
        dets: list[Detection] = []
        for row in raw.reshape(-1, 6):
            if not np.all(np.isfinite(row)) or row[4] < self._conf:
                continue
            box = clip_box(row[:4], bgr.shape)
            cls = int(row[5])
            if box is None or not 0 <= cls < len(self._class_names):
                continue
            dets.append(Detection(box, cls, self._class_names[cls], float(row[4])))
        return dedup_overlapping(dets, *self._dedup)
