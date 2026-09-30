"""Landmark containers shared by 2D/3D landmark extraction and fusion."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Landmark:
    """One 3D point; the frame (base_link or camera optical) is chosen by the caller."""

    valid: bool
    position: np.ndarray  # (3,) [m]
    cov: np.ndarray  # (3, 3) [m^2], 1-sigma
    confidence: float


@dataclass
class Landmarks2D:
    ok: bool
    bottom_px: np.ndarray  # (2,) u, v
    neck_px: np.ndarray
    tie_px: np.ndarray
    axis_px: np.ndarray  # (2,) unit, bottom -> neck
    confidence: float
    widths_px: np.ndarray  # smoothed width profile, index 0 = bottom end
    flags: list[str] = field(default_factory=list)


@dataclass
class Landmarks3D:
    ok: bool
    bottom: Landmark
    neck: Landmark
    tie: Landmark
    axis: np.ndarray  # (3,) unit, bottom -> neck
    d95_m: float
    length_m: float
    flags: list[str] = field(default_factory=list)


def invalid_landmark() -> Landmark:
    return Landmark(valid=False, position=np.full(3, np.nan), cov=np.full((3, 3), np.nan),
                    confidence=0.0)


def failed_landmarks2d(flags: list[str]) -> Landmarks2D:
    nan2 = np.full(2, np.nan)
    return Landmarks2D(ok=False, bottom_px=nan2.copy(), neck_px=nan2.copy(), tie_px=nan2.copy(),
                       axis_px=nan2.copy(), confidence=0.0, widths_px=np.zeros(0),
                       flags=list(flags))


def failed_landmarks3d(flags: list[str]) -> Landmarks3D:
    return Landmarks3D(ok=False, bottom=invalid_landmark(), neck=invalid_landmark(),
                       tie=invalid_landmark(), axis=np.full(3, np.nan), d95_m=float('nan'),
                       length_m=float('nan'), flags=list(flags))


def unit(vector: np.ndarray) -> np.ndarray | None:
    """Finite non-zero vector -> unit vector; otherwise None."""
    v = np.asarray(vector, dtype=np.float64).reshape(-1)
    if not np.all(np.isfinite(v)):
        return None
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return None
    return v / n


def axial_lateral_cov(axis: np.ndarray, sigma_lateral_m: float,
                      sigma_axial_m: float) -> np.ndarray:
    """Covariance with independent axial / isotropic lateral 1-sigma about a unit axis."""
    a = np.asarray(axis, dtype=np.float64).reshape(3)
    aa = np.outer(a, a)
    return sigma_lateral_m ** 2 * (np.eye(3) - aa) + sigma_axial_m ** 2 * aa
