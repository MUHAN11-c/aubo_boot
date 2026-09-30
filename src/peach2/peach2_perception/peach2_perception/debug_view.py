"""Debug overlay: boxes, masks, 2D landmarks and track labels (no ROS)."""
from __future__ import annotations

from typing import Sequence

import cv2
import numpy as np

from .observation_builder import CATEGORY_BAG, Measurement, ObservationRecord

_BAG = (0, 200, 0)
_NOBAG = (0, 160, 255)
_UNCONFIRMED = (160, 160, 160)
_BOTTOM = (255, 0, 0)
_NECK = (0, 0, 255)
_TIE = (255, 0, 255)


def _colour(m: Measurement, confirmed: bool) -> tuple[int, int, int]:
    if not confirmed:
        return _UNCONFIRMED
    return _BAG if m.category == CATEGORY_BAG else _NOBAG


def draw_debug(bgr: np.ndarray, records: Sequence[ObservationRecord],
               untracked: Sequence[Measurement] = (), downscale: float = 1.0,
               header: str = '') -> np.ndarray:
    """BGR overlay image, resized by `downscale` (0 < downscale <= 1)."""
    if not 0.0 < downscale <= 1.0:
        raise ValueError('downscale must be in (0, 1]')
    out = bgr.copy()
    items = [(r.measurement, r.confirmed, r.target_id) for r in records]
    items += [(m, False, '?') for m in untracked]
    tint = out.copy()
    for m, confirmed, _ in items:
        if m.mask is not None:
            tint[m.mask] = _colour(m, confirmed)
    out = cv2.addWeighted(tint, 0.35, out, 0.65, 0.0)
    for m, confirmed, tid in items:
        colour = _colour(m, confirmed)
        x1, y1, x2, y2 = m.detection.bbox
        cv2.rectangle(out, (x1, y1), (x2 - 1, y2 - 1), colour, 2)
        label = f'{tid} {m.detection.score:.2f} q{m.mask_quality:.2f}'
        cv2.putText(out, label, (x1, max(y1 - 4, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1,
                    cv2.LINE_AA)
        lm2 = m.landmarks2d
        if lm2 is not None and lm2.ok:
            for px, c in ((lm2.bottom_px, _BOTTOM), (lm2.neck_px, _NECK), (lm2.tie_px, _TIE)):
                if np.all(np.isfinite(px)):
                    cv2.circle(out, (int(round(px[0])), int(round(px[1]))), 4, c, -1)
    if header:
        cv2.putText(out, header, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                    cv2.LINE_AA)
    if downscale < 1.0:
        h, w = out.shape[:2]
        out = cv2.resize(out, (max(int(w * downscale), 1), max(int(h * downscale), 1)),
                         interpolation=cv2.INTER_AREA)
    return out
