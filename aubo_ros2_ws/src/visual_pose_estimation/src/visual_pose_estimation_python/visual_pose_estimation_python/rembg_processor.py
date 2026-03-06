#!/usr/bin/env python3
"""Rembg helper for mask and white-background cutout."""

from __future__ import annotations

import io
import json
import time
from typing import List, Optional, Tuple

import numpy as np
from PIL import Image

# #region agent log
_DEBUG_LOG_PATH = "/home/mu/IVG/.cursor/debug.log"
def _debug_log(location: str, message: str, data: dict = None, hypothesis_id: str = None):
    try:
        with open(_DEBUG_LOG_PATH, "a", encoding="utf-8") as f:
            log_entry = {
                "sessionId": "rembg-debug",
                "runId": "run1",
                "hypothesisId": hypothesis_id or "A",
                "location": location,
                "message": message,
                "data": data or {},
                "timestamp": int(time.time() * 1000)
            }
            f.write(json.dumps(log_entry, ensure_ascii=False) + "\n")
    except Exception:
        pass
# #endregion


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
        # #region agent log
        _debug_log("rembg_processor.py:_ensure_session", "检查 RemBG session 状态", {
            "has_session": self._session is not None,
            "has_error": self._init_error is not None,
            "model": self.model,
            "prefer_cuda": self.prefer_cuda
        }, "A")
        # #endregion
        if self._session is not None or self._init_error is not None:
            # #region agent log
            _debug_log("rembg_processor.py:_ensure_session", "Session 已存在或已有错误，跳过初始化", {
                "has_session": self._session is not None,
                "error": str(self._init_error) if self._init_error else None
            }, "A")
            # #endregion
            return

        try:
            import onnxruntime as ort
            from rembg import new_session, remove
            # #region agent log
            _debug_log("rembg_processor.py:_ensure_session", "成功导入 rembg 和 onnxruntime", {}, "A")
            # #endregion
        except Exception as exc:
            self._init_error = exc
            # #region agent log
            _debug_log("rembg_processor.py:_ensure_session", "导入 rembg 失败", {
                "error": str(exc),
                "error_type": type(exc).__name__
            }, "A")
            # #endregion
            return

        providers = ["CPUExecutionProvider"]
        available = ort.get_available_providers()
        # #region agent log
        _debug_log("rembg_processor.py:_ensure_session", "检查可用 providers", {
            "available_providers": available,
            "prefer_cuda": self.prefer_cuda
        }, "A")
        # #endregion
        if self.prefer_cuda and "CUDAExecutionProvider" in available:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
            # #region agent log
            _debug_log("rembg_processor.py:_ensure_session", "启用 CUDA provider", {"providers": providers}, "A")
            # #endregion

        self._providers = providers
        self._session = new_session(self.model, providers=providers)
        self._remove = remove
        # #region agent log
        _debug_log("rembg_processor.py:_ensure_session", "RemBG session 初始化成功", {
            "providers": providers,
            "model": self.model
        }, "A")
        # #endregion

    def process_roi(
        self,
        color_bgr: np.ndarray,
        bbox: Tuple[int, int, int, int],
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Return (full_mask, full_cutout_bgr) for the ROI bbox."""
        # #region agent log
        _debug_log("rembg_processor.py:process_roi", "开始处理 ROI", {
            "bbox": bbox,
            "image_shape": color_bgr.shape[:2] if color_bgr is not None else None
        }, "B")
        # #endregion
        self._ensure_session()
        if self._session is None or self._remove is None or self._init_error is not None:
            # #region agent log
            _debug_log("rembg_processor.py:process_roi", "RemBG session 不可用，返回 None", {
                "has_session": self._session is not None,
                "has_remove": self._remove is not None,
                "error": str(self._init_error) if self._init_error else None
            }, "B")
            # #endregion
            return None, None

        img_h, img_w = color_bgr.shape[:2]
        x, y, w, h = bbox
        if w <= 0 or h <= 0:
            # #region agent log
            _debug_log("rembg_processor.py:process_roi", "无效的 bbox 尺寸", {"bbox": bbox}, "B")
            # #endregion
            return None, None

        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(img_w, x + w)
        y1 = min(img_h, y + h)
        if x1 <= x0 or y1 <= y0:
            # #region agent log
            _debug_log("rembg_processor.py:process_roi", "裁剪后 bbox 无效", {
                "original_bbox": bbox,
                "clipped_bbox": (x0, y0, x1, y1)
            }, "B")
            # #endregion
            return None, None

        roi_bgr = color_bgr[y0:y1, x0:x1]
        roi_rgb = roi_bgr[:, :, ::-1]
        pil_img = Image.fromarray(roi_rgb)

        buffer = io.BytesIO()
        pil_img.save(buffer, format="PNG")
        raw = buffer.getvalue()
        # #region agent log
        _debug_log("rembg_processor.py:process_roi", "准备调用 rembg.remove", {
            "roi_size": (y1 - y0, x1 - x0),
            "providers": self._providers
        }, "B")
        # #endregion

        result = self._remove(raw, session=self._session)
        result_img = Image.open(io.BytesIO(result)).convert("RGBA")
        rgba = np.array(result_img)
        alpha = rgba[:, :, 3]
        # #region agent log
        _debug_log("rembg_processor.py:process_roi", "RemBG 处理完成", {
            "alpha_shape": alpha.shape,
            "alpha_min": int(alpha.min()),
            "alpha_max": int(alpha.max()),
            "alpha_mean": float(alpha.mean())
        }, "B")
        # #endregion

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
        # #region agent log
        _debug_log("rembg_processor.py:process_roi", "RemBG 处理成功，返回结果", {
            "mask_shape": full_mask.shape,
            "cutout_shape": full_cutout.shape,
            "mask_nonzero": int(np.count_nonzero(full_mask))
        }, "B")
        # #endregion

        return full_mask, full_cutout
