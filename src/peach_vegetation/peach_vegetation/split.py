"""Branch / leaf split (ExG+HSV leaf, Frangi ridges). Zero ROS."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Sequence

import cv2
import numpy as np

from peach_vegetation.frangi import (
    frangi_vesselness,
    resolve_device,
    threshold_ridges,
)

LEAF_BGR = (0, 220, 0)
BRANCH_BGR = (0, 0, 255)


@dataclass(frozen=True)
class SplitConfig:
    """Immutable split knobs copied from the ROS param snapshot."""

    device: str = 'auto'
    leaf_exg_min: float = 20.0
    leaf_h_min: int = 35
    leaf_h_max: int = 90
    leaf_s_min: int = 30
    leaf_v_min: int = 30
    branch_sigmas: Sequence[float] = (1, 3, 5, 7, 9)
    branch_beta: float = 0.5
    branch_gamma: float = 15.0
    branch_percentile: float = 88.0
    branch_dilate_px: int = 2
    branch_dark_max: float = 90.0
    branch_exclude_leaf: bool = True


@dataclass
class VegetationMasks:
    """One-frame split result."""

    leaf: np.ndarray
    branch: np.ndarray
    overlay: np.ndarray
    infer_ms: float
    device: str


def config_from_params(params) -> SplitConfig:
    """Copy a yaml param namespace into SplitConfig."""
    return SplitConfig(
        device=str(params.device),
        leaf_exg_min=float(params.leaf.exg_min),
        leaf_h_min=int(params.leaf.h_min),
        leaf_h_max=int(params.leaf.h_max),
        leaf_s_min=int(params.leaf.s_min),
        leaf_v_min=int(params.leaf.v_min),
        branch_sigmas=tuple(float(s) for s in params.branch.sigmas),
        branch_beta=float(params.branch.beta),
        branch_gamma=float(params.branch.gamma),
        branch_percentile=float(params.branch.percentile),
        branch_dilate_px=int(params.branch.dilate_px),
        branch_dark_max=float(params.branch.dark_max),
        branch_exclude_leaf=bool(params.branch.exclude_leaf),
    )


def excess_green(bgr: np.ndarray) -> np.ndarray:
    """2G − R − B as int16, same HxW as the BGR image."""
    blue = bgr[:, :, 0].astype(np.int16)
    green = bgr[:, :, 1].astype(np.int16)
    red = bgr[:, :, 2].astype(np.int16)
    return (2 * green) - red - blue


def _hsv_u8(bgr: np.ndarray) -> np.ndarray:
    """BGR → HSV uint8（cv2；cv_bridge 是硬 exec_depend，cv2 必在）."""
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)


def leaf_mask_bgr(
        bgr: np.ndarray,
        *,
        exg_min: float,
        h_min: int,
        h_max: int,
        s_min: int,
        v_min: int) -> np.ndarray:
    """Union of Excess Green and HSV green window. bool HxW."""
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError('leaf_mask_bgr 需要 HxWx3 BGR')
    exg = excess_green(bgr) >= exg_min
    hsv = _hsv_u8(bgr)
    hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    green = (
        (hue >= h_min) & (hue <= h_max)
        & (sat >= s_min) & (val >= v_min))
    return exg | green


def paint_overlay(bgr: np.ndarray, leaf: np.ndarray, branch: np.ndarray) -> np.ndarray:
    """Copy BGR, paint leaf green then branch red on top."""
    overlay = np.ascontiguousarray(bgr.copy())
    overlay[leaf] = LEAF_BGR
    overlay[branch] = BRANCH_BGR
    return overlay


def bgr_to_gray_u8(bgr: np.ndarray) -> np.ndarray:
    """Rec. 601 luma as uint8."""
    blue = bgr[:, :, 0].astype(np.float32)
    green = bgr[:, :, 1].astype(np.float32)
    red = bgr[:, :, 2].astype(np.float32)
    gray = 0.114 * blue + 0.587 * green + 0.299 * red
    return np.clip(gray, 0, 255).astype(np.uint8)


class FrangiExgSplitter:
    """Single implementation: GPU Frangi twigs + ExG/HSV leaves."""

    def __init__(self, config: SplitConfig):
        """Resolve torch device; first split() loads CUDA kernels."""
        self.config = config
        self.device = resolve_device(config.device)

    def warmup(self) -> None:
        """Run one tiny frame so the first live callback is not a JIT stall."""
        dummy = np.zeros((64, 64, 3), dtype=np.uint8)
        dummy[:, :] = (40, 160, 40)
        dummy[:, 30:34] = (20, 20, 20)
        self.split(dummy)

    def split(self, bgr: np.ndarray) -> VegetationMasks:
        """Segment one BGR uint8 frame into leaf/branch masks and overlay."""
        started = time.perf_counter()
        cfg = self.config
        leaf = leaf_mask_bgr(
            bgr,
            exg_min=cfg.leaf_exg_min,
            h_min=cfg.leaf_h_min,
            h_max=cfg.leaf_h_max,
            s_min=cfg.leaf_s_min,
            v_min=cfg.leaf_v_min,
        )
        gray = bgr_to_gray_u8(bgr)
        vessel = frangi_vesselness(
            gray,
            sigmas=cfg.branch_sigmas,
            beta=cfg.branch_beta,
            gamma=cfg.branch_gamma,
            black_ridges=True,
            device=self.device,
        )
        branch = threshold_ridges(
            vessel,
            gray,
            percentile=cfg.branch_percentile,
            dark_max=cfg.branch_dark_max,
            dilate_px=cfg.branch_dilate_px,
        )
        if cfg.branch_exclude_leaf:
            branch = branch & ~leaf
        overlay = paint_overlay(bgr, leaf, branch)
        infer_ms = (time.perf_counter() - started) * 1000.0
        return VegetationMasks(
            leaf=leaf,
            branch=branch,
            overlay=overlay,
            infer_ms=infer_ms,
            device=self.device,
        )
