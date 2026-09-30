"""
MobileSAM bag segmentation with depth-aware post-processing (no ROS).

One image embedding per frame is shared by every box (the old stack re-encoded the image per
box on its fallback path). Prompt per box: the box expanded by `box_expand_frac` on each side,
a positive point at the box centre, and four negative points just outside the expanded box's
corners. Post-processing: SAM mask intersected with valid+confident depth, depth-discontinuity
pixels removed, the connected component holding the box centre kept, then open/close.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import os
from typing import Protocol, Sequence

import cv2
import numpy as np

from .depth_quality import depth_edges

N_PROMPT_POINTS = 5
LABEL_POSITIVE = 1
LABEL_NEGATIVE = 0
LABEL_PADDING = -1


class SamBackend(Protocol):
    device: str

    def set_image(self, bgr: np.ndarray) -> None:
        """Encode one image; later predict() calls reuse the embedding."""
        ...

    def predict(self, boxes: np.ndarray, points: np.ndarray,
                labels: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Masks for N prompts on the current image.

        boxes (N, 4) xyxy, points (N, K, 2) u,v, labels (N, K) in {1, 0, -1 padding};
        returns (bool masks (N, H, W) at image resolution, scores (N,)).
        """
        ...


@dataclass(frozen=True)
class Prompt:
    box: np.ndarray  # (4,) expanded xyxy [px], float
    points: np.ndarray  # (N_PROMPT_POINTS, 2) u, v [px]
    labels: np.ndarray  # (N_PROMPT_POINTS,) int


@dataclass(frozen=True)
class RefineParams:
    depth_jump_rel: float
    depth_jump_abs_m: float
    seed_frac: float
    morph_kernel_px: int
    min_area_px: int


@dataclass
class MaskResult:
    mask: np.ndarray | None  # (H, W) bool; None when no usable mask
    score: float
    raw_area: int  # SAM mask pixels before refinement
    area: int
    flags: list[str] = field(default_factory=list)


def expand_box(bbox: Sequence[int], shape: tuple[int, ...],
               frac: float) -> tuple[int, int, int, int]:
    h, w = int(shape[0]), int(shape[1])
    x1, y1, x2, y2 = bbox
    dx, dy = frac * (x2 - x1), frac * (y2 - y1)
    return (int(max(np.floor(x1 - dx), 0)), int(max(np.floor(y1 - dy), 0)),
            int(min(np.ceil(x2 + dx), w)), int(min(np.ceil(y2 + dy), h)))


def build_prompt(bbox: Sequence[int], shape: tuple[int, ...], expand_frac: float,
                 neg_offset_px: int) -> Prompt:
    """
    SAM prompt for one detection box.

    Negative points sit `neg_offset_px` diagonally outside the expanded box corners (clipped to
    the image). A negative that clipping pushes into the original box (box at the image border)
    would contradict the positive and is replaced by padding.
    """
    h, w = int(shape[0]), int(shape[1])
    x1, y1, x2, y2 = bbox
    ex = expand_box(bbox, shape, expand_frac)
    o = float(neg_offset_px)
    corners = ((ex[0] - o, ex[1] - o), (ex[2] - 1 + o, ex[1] - o),
               (ex[0] - o, ex[3] - 1 + o), (ex[2] - 1 + o, ex[3] - 1 + o))
    points = [(0.5 * (x1 + x2), 0.5 * (y1 + y2))]
    labels = [LABEL_POSITIVE]
    for u, v in corners:
        u = float(np.clip(u, 0.0, w - 1.0))
        v = float(np.clip(v, 0.0, h - 1.0))
        inside = x1 <= u < x2 and y1 <= v < y2
        points.append((u, v) if not inside else (0.0, 0.0))
        labels.append(LABEL_NEGATIVE if not inside else LABEL_PADDING)
    return Prompt(box=np.asarray(ex, dtype=np.float64), points=np.asarray(points),
                  labels=np.asarray(labels, dtype=np.int64))


def refine_mask(raw_mask: np.ndarray, bbox: Sequence[int], roi: Sequence[int],
                depth_m: np.ndarray, depth_ok: np.ndarray, params: RefineParams,
                score: float = 0.0) -> MaskResult:
    """
    Depth-aware clean-up of one SAM mask inside `roi` (the expanded box).

    A leaf or a neighbouring bag in front of the bag is connected in 2D but separated by a depth
    jump; removing jump pixels splits it off, and the component overlapping the central seed
    region of the detection box (`seed_frac` of its size) is the bag. Falls back to the largest
    component when none touches the seed. The kept component is dilated by one pixel inside the
    depth-valid SAM mask to restore the edge row eroded by the jump removal.
    """
    raw = np.asarray(raw_mask, dtype=bool)
    raw_area = int(np.count_nonzero(raw))
    x1, y1, x2, y2 = roi
    flags: list[str] = []
    base = raw[y1:y2, x1:x2] & np.asarray(depth_ok, dtype=bool)[y1:y2, x1:x2]
    if raw_area == 0:
        return MaskResult(None, score, 0, 0, ['sam_empty'])
    if not np.any(base):
        return MaskResult(None, score, raw_area, 0, ['mask_no_depth'])
    z = np.asarray(depth_m, dtype=np.float32)[y1:y2, x1:x2]
    cut = base & ~depth_edges(z, base, params.depth_jump_rel, params.depth_jump_abs_m)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(cut.astype(np.uint8), connectivity=8)
    if n <= 1:
        return MaskResult(None, score, raw_area, 0, ['mask_fragmented'])
    bx1, by1, bx2, by2 = bbox
    cu, cv_ = 0.5 * (bx1 + bx2) - x1, 0.5 * (by1 + by2) - y1
    hw, hh = 0.5 * params.seed_frac * (bx2 - bx1), 0.5 * params.seed_frac * (by2 - by1)
    su0, su1 = int(max(np.floor(cu - hw), 0)), int(min(np.ceil(cu + hw), cut.shape[1]))
    sv0, sv1 = int(max(np.floor(cv_ - hh), 0)), int(min(np.ceil(cv_ + hh), cut.shape[0]))
    seed_counts = np.bincount(labels[sv0:sv1, su0:su1].ravel(), minlength=n)
    seed_counts[0] = 0
    if seed_counts.max() > 0:
        keep = int(np.argmax(seed_counts))
    else:
        keep = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        flags.append('seed_missed')
    if n > 2:
        flags.append('depth_split')
    comp = (labels == keep).astype(np.uint8)
    comp = cv2.dilate(comp, np.ones((3, 3), np.uint8)) & base.astype(np.uint8)
    k = int(params.morph_kernel_px)
    if k >= 3:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        comp = cv2.morphologyEx(comp, cv2.MORPH_OPEN, kernel)
        comp = cv2.morphologyEx(comp, cv2.MORPH_CLOSE, kernel)
    area = int(np.count_nonzero(comp))
    if area < params.min_area_px:
        return MaskResult(None, score, raw_area, area, flags + ['mask_too_small'])
    full = np.zeros(raw.shape, dtype=bool)
    full[y1:y2, x1:x2] = comp.astype(bool)
    return MaskResult(full, score, raw_area, area, flags)


class MobileSamBackend:
    """MobileSAM via Ultralytics modules, driven directly so points and boxes batch together."""

    def __init__(self, model_path: str, device: str, allow_cpu: bool, imgsz: int) -> None:
        import torch
        from ultralytics.models.sam.build import build_mobile_sam

        from .detector import resolve_device
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f'MobileSAM weights not found: {model_path}')
        self._torch = torch
        self.device = resolve_device(device, allow_cpu)
        self._imgsz = int(imgsz)
        model = build_mobile_sam(checkpoint=model_path)
        model.eval()
        model.to(self.device)
        model.set_imgsz((self._imgsz, self._imgsz))
        self._model = model
        self._mean = torch.tensor([123.675, 116.28, 103.53], device=self.device).view(3, 1, 1)
        self._std = torch.tensor([58.395, 57.12, 57.375], device=self.device).view(3, 1, 1)
        self._features = None
        self._shape = (0, 0)
        self._unpad = (0, 0)
        self._scale = 1.0
        self.set_image(np.zeros((self._imgsz // 2, self._imgsz // 2, 3), dtype=np.uint8))
        s = self._imgsz // 4
        self.predict(np.array([[s, s, 2 * s, 2 * s]], dtype=np.float64),
                     np.array([[[1.5 * s, 1.5 * s]]]), np.array([[LABEL_POSITIVE]]))

    def set_image(self, bgr: np.ndarray) -> None:
        torch = self._torch
        h, w = bgr.shape[:2]
        # Top-left letterbox (SAM's convention): prompts scale by r, no offset.
        r = self._imgsz / float(max(h, w))
        nw, nh = int(round(w * r)), int(round(h * r))
        rgb = cv2.cvtColor(cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR),
                           cv2.COLOR_BGR2RGB)
        with torch.inference_mode():
            x = torch.from_numpy(np.ascontiguousarray(rgb)).to(self.device)
            x = (x.permute(2, 0, 1).float() - self._mean) / self._std
            padded = torch.zeros((1, 3, self._imgsz, self._imgsz), device=self.device)
            padded[0, :, :nh, :nw] = x
            self._features = self._model.image_encoder(padded)
        self._shape, self._unpad, self._scale = (h, w), (nh, nw), r

    def predict(self, boxes: np.ndarray, points: np.ndarray,
                labels: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        torch = self._torch
        import torch.nn.functional as F
        if self._features is None:
            raise RuntimeError('set_image() must be called before predict()')
        h, w = self._shape
        nh, nw = self._unpad
        with torch.inference_mode():
            b = torch.as_tensor(boxes, dtype=torch.float32, device=self.device) * self._scale
            p = torch.as_tensor(points, dtype=torch.float32, device=self.device) * self._scale
            lab = torch.as_tensor(labels, dtype=torch.int32, device=self.device)
            sparse, dense = self._model.prompt_encoder(points=(p, lab), boxes=b, masks=None)
            low, iou = self._model.mask_decoder(
                image_embeddings=self._features,
                image_pe=self._model.prompt_encoder.get_dense_pe(),
                sparse_prompt_embeddings=sparse, dense_prompt_embeddings=dense,
                multimask_output=False)
            up = F.interpolate(low, (self._imgsz, self._imgsz), mode='bilinear',
                               align_corners=False)[..., :nh, :nw]
            up = F.interpolate(up, (h, w), mode='bilinear', align_corners=False)
            masks = (up[:, 0] > self._model.mask_threshold).cpu().numpy()
            scores = iou[:, 0].float().cpu().numpy()
        return masks, scores


class Segmenter:
    def __init__(self, backend: SamBackend, max_boxes: int, box_expand_frac: float,
                 neg_point_offset_px: int, refine: RefineParams) -> None:
        self._backend = backend
        self._max_boxes = int(max_boxes)
        self._expand = float(box_expand_frac)
        self._neg_offset = int(neg_point_offset_px)
        self._refine = refine

    @property
    def device(self) -> str:
        return self._backend.device

    def segment(self, bgr: np.ndarray, bboxes: Sequence[Sequence[int]], depth_m: np.ndarray,
                depth_ok: np.ndarray) -> tuple[list[MaskResult], int]:
        """
        (one MaskResult per box, number of boxes beyond max_boxes).

        Boxes are taken in the given order (the caller puts the most important first); boxes
        beyond max_boxes get a mask-less result flagged `sam_truncated` instead of vanishing.
        """
        if not bboxes:
            return [], 0
        shape = bgr.shape
        used = list(bboxes[:self._max_boxes])
        truncated = len(bboxes) - len(used)
        prompts = [build_prompt(b, shape, self._expand, self._neg_offset) for b in used]
        self._backend.set_image(bgr)
        masks, scores = self._backend.predict(
            np.stack([p.box for p in prompts]), np.stack([p.points for p in prompts]),
            np.stack([p.labels for p in prompts]))
        results = []
        for b, p, m, s in zip(used, prompts, masks, scores):
            roi = tuple(int(x) for x in p.box)
            results.append(refine_mask(m, b, roi, depth_m, depth_ok, self._refine, float(s)))
        results.extend(MaskResult(None, 0.0, 0, 0, ['sam_truncated']) for _ in range(truncated))
        return results, truncated
