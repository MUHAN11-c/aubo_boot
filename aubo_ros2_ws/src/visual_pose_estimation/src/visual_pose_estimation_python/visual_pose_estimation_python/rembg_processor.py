#!/usr/bin/env python3
"""Rembg helper for mask and white-background cutout."""

from __future__ import annotations

import io
from typing import List, Optional, Tuple

import numpy as np
from PIL import Image

class RemBGProcessor:
    """Lightweight rembg wrapper with CUDA preference."""

    def __init__(self, model: str = "u2net", prefer_cuda: bool = True) -> None:
        self.model = model
        self.prefer_cuda = prefer_cuda
        self._session = None
        self._providers: Optional[List[str]] = None
        self._remove = None
        self._init_error: Optional[Exception] = None

    @property
    def providers(self) -> Optional[List[str]]:
        return self._providers

    def _ensure_session(self) -> None:
        if self._session is not None or self._init_error is not None:
            return

        try:
            import onnxruntime as ort
            from rembg import new_session, remove
        except Exception as exc:
            self._init_error = exc
            return

        providers = ["CPUExecutionProvider"]
        available = ort.get_available_providers()
        if self.prefer_cuda and "CUDAExecutionProvider" in available:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]

        self._providers = providers
        self._session = new_session(self.model, providers=providers)
        self._remove = remove

    def process_roi(
        self,
        color_bgr: np.ndarray,
        bbox: Tuple[int, int, int, int],
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Return (full_mask, full_cutout_bgr) for the ROI bbox."""
        self._ensure_session()
        if self._session is None or self._remove is None or self._init_error is not None:
            return None, None

        img_h, img_w = color_bgr.shape[:2]
        x, y, w, h = bbox
        if w <= 0 or h <= 0:
            return None, None

        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(img_w, x + w)
        y1 = min(img_h, y + h)
        if x1 <= x0 or y1 <= y0:
            return None, None

        roi_bgr = color_bgr[y0:y1, x0:x1]
        roi_rgb = roi_bgr[:, :, ::-1]
        pil_img = Image.fromarray(roi_rgb)

        buffer = io.BytesIO()
        pil_img.save(buffer, format="PNG")
        raw = buffer.getvalue()

        result = self._remove(raw, session=self._session)
        result_img = Image.open(io.BytesIO(result)).convert("RGBA")
        rgba = np.array(result_img)
        alpha = rgba[:, :, 3]

        # full-size mask
        full_mask = np.zeros((img_h, img_w), dtype=np.uint8)
        full_mask[y0:y1, x0:x1] = np.where(alpha > 127, 255, 0).astype(np.uint8)

        # white background composite for ROI
        rgb = rgba[:, :, :3].astype(np.float32)
        alpha_f = (alpha.astype(np.float32) / 255.0)[:, :, None]
        white = np.full_like(rgb, 255.0)
        composite_rgb = (alpha_f * rgb + (1.0 - alpha_f) * white).astype(np.uint8)
        composite_bgr = composite_rgb[:, :, ::-1]

        full_cutout = np.full((img_h, img_w, 3), 255, dtype=np.uint8)
        full_cutout[y0:y1, x0:x1] = composite_bgr

        return full_mask, full_cutout
