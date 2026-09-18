"""
重建帧摄入纯核：from_params + process(rgb, depth, K) → Result.

节点只做 ROS 解码/入环。柱/球 impl 字典仍在 refine.make_refitter。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from peach_harvester.vision.common.geometry import (
    normalize_depth_to_uint16_mm,
)
from peach_harvester.vision.target_reconstruction.refine import make_refitter


@dataclass
class ReconstructionFrame:
    """过门后的 RGB-D：深度已换算为 uint16 毫米."""

    rgb: np.ndarray
    """BGR uint8 (H, W, 3)."""
    depth_mm: np.ndarray
    """uint16 毫米，与 RGB 同尺寸."""
    K: dict
    """{'fx','fy','cx','cy','width','height'}，像素."""


@dataclass
class ReconstructionResult:
    """process() 一拍：ok 则节点入帧环，否则只打 reason."""

    frame: Optional[ReconstructionFrame] = None
    """None=本帧丢弃（分辨率/内参/单位失败）."""
    reason: str = ''
    """丢帧原因，供节点 warning；成功为空串."""

    @property
    def ok(self) -> bool:
        """Return whether ingest gates passed."""
        return self.frame is not None


class ReconstructionSession:
    """yaml 选柱/球 refitter；摄入一帧解码图。不持有 Node."""

    params: object
    """TargetReconstructionParams 快照."""
    refitters: dict
    """{'cylinder': …, 'sphere': …}，由 refine.make_refitter 装配."""

    def __init__(self, params, refitters: dict):
        self.params = params
        self.refitters = refitters

    @classmethod
    def from_params(cls, params):
        """Build cylinder/sphere refitters from yaml ``refitter.*_impl``."""
        return cls(params, {
            'cylinder': make_refitter(params.refitter.cylinder_impl),
            'sphere': make_refitter(params.refitter.sphere_impl),
        })

    def process(self, rgb, depth_raw, camera_k) -> ReconstructionResult:
        """Normalize depth, gate resolution and K. None-frame if drop."""
        try:
            depth_mm = normalize_depth_to_uint16_mm(
                depth_raw, self.params.depth_scale_unit)
        except ValueError as exc:
            return ReconstructionResult(reason=f'深度归一化失败，丢帧: {exc}')
        if rgb.shape[:2] != depth_mm.shape[:2]:
            return ReconstructionResult(
                reason=(
                    f'RGB/深度分辨率不一致 {rgb.shape[:2]} vs '
                    f'{depth_mm.shape[:2]}，丢帧'))
        k = camera_k
        K = {
            'fx': float(k[0]), 'fy': float(k[4]),
            'cx': float(k[2]), 'cy': float(k[5]),
            'width': int(depth_mm.shape[1]), 'height': int(depth_mm.shape[0]),
        }
        intrinsic_values = np.array(
            [K['fx'], K['fy'], K['cx'], K['cy']], dtype=np.float64)
        if (not np.all(np.isfinite(intrinsic_values))
                or K['fx'] <= 0.0 or K['fy'] <= 0.0):
            return ReconstructionResult(
                reason='相机内参含非有限值或 fx/fy≤0，丢帧')
        return ReconstructionResult(
            frame=ReconstructionFrame(rgb=rgb, depth_mm=depth_mm, K=K))
