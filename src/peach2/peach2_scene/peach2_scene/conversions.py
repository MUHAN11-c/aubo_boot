"""Boundary conversions for the node (zero ROS): depth units, TF poses, TargetModel capsules."""
from __future__ import annotations

import math
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from .scene_core import TargetCapsule

DEPTH_UINT16_ENCODINGS = ('16UC1', 'mono16')
DEPTH_FLOAT_ENCODINGS = ('32FC1',)
_UINT16_SATURATED = 65535


def depth_to_metres(depth: np.ndarray, encoding: str, unit_m: float) -> np.ndarray:
    """uint16 counts * unit_m (0 and 65535 invalid) or float metres; invalid -> 0.0."""
    if encoding in DEPTH_UINT16_ENCODINGS:
        raw = np.asarray(depth)
        out = raw.astype(np.float32) * np.float32(unit_m)
        out[(raw == 0) | (raw == _UINT16_SATURATED)] = 0.0
        return out
    if encoding in DEPTH_FLOAT_ENCODINGS:
        out = np.array(depth, dtype=np.float32, copy=True)
        out[~np.isfinite(out) | (out < 0.0)] = 0.0
        return out
    raise ValueError(f'unsupported depth encoding {encoding!r}')


def pose_matrix(translation: Sequence[float], quat_xyzw: Sequence[float]) -> np.ndarray:
    """4x4 homogeneous transform; the quaternion is normalised (TF may carry rounding)."""
    q = np.asarray(quat_xyzw, dtype=np.float64)
    n = float(np.linalg.norm(q))
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError('degenerate quaternion')
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q / n).as_matrix()
    T[:3, 3] = np.asarray(translation, dtype=np.float64)
    return T


def target_capsule(target_id: str, bottom_valid: bool, bottom: Sequence[float],
                   neck_valid: bool, neck: Sequence[float], d95_m: float) -> TargetCapsule | None:
    """Return the capsule of a model with finite valid bottom/neck and d95 > 0, else None."""
    if not (bottom_valid and neck_valid) or not target_id:
        return None
    b = np.asarray(bottom, dtype=np.float64)
    n = np.asarray(neck, dtype=np.float64)
    if not (np.all(np.isfinite(b)) and np.all(np.isfinite(n))):
        return None
    if not (math.isfinite(d95_m) and d95_m > 0.0):
        return None
    return TargetCapsule(target_id, b, n, float(d95_m))
