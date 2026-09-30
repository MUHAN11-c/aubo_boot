"""
Depth noise model and pinhole back-projection (camera optical frame, metres).

Depth images must already be converted to metres at the driver boundary
(stereo front end 0.25 mm/LSB, Percipio 1 mm/LSB). Invalid depth is 0 or non-finite.
Confidence may be mono8 (0..255) or float (0..1); it is normalised to 0..1 here.
"""
from __future__ import annotations

import numpy as np

# Pixel-centre rounding of the query pixel, 1-sigma [px].
_PIXEL_SIGMA_PX = 0.5
# Floor on the local depth sigma: a perfectly flat window would otherwise claim zero noise.
_DEPTH_SIGMA_FLOOR_M = 0.001


def depth_sigma_m(z_m: np.ndarray, fx_px: float, baseline_m: float,
                  disparity_sigma_px: float = 0.25) -> np.ndarray:
    """Stereo depth 1-sigma: sigma_z = z^2 * sigma_d / (f * b)."""
    if fx_px <= 0.0 or baseline_m <= 0.0:
        raise ValueError('fx_px and baseline_m must be > 0')
    z = np.asarray(z_m, dtype=np.float64)
    return z * z * float(disparity_sigma_px) / (float(fx_px) * float(baseline_m))


def _normalise_confidence(confidence: np.ndarray) -> np.ndarray:
    c = np.asarray(confidence)
    if c.dtype == np.uint8:
        return c.astype(np.float64) / 255.0
    return c.astype(np.float64)


def _weighted_median(values: np.ndarray, weights: np.ndarray) -> float:
    order = np.argsort(values)
    v = values[order]
    w = weights[order]
    cdf = np.cumsum(w)
    idx = int(np.searchsorted(cdf, 0.5 * cdf[-1]))
    return float(v[min(idx, v.size - 1)])


def backproject(u: float, v: float, depth_m: np.ndarray, K: np.ndarray,
                confidence: np.ndarray | None = None,
                win: int = 1) -> tuple[np.ndarray | None, np.ndarray | None]:
    """
    Back-project pixel (u, v) using a confidence-weighted median depth over a window.

    Returns (xyz, cov3x3) in the camera optical frame, or (None, None) when no valid depth.
    The covariance propagates pixel rounding and the robust depth spread of the window
    through the pinhole Jacobian.
    """
    depth = np.asarray(depth_m, dtype=np.float64)
    h, w = depth.shape[:2]
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    if not (np.isfinite(u) and np.isfinite(v)) or fx <= 0.0 or fy <= 0.0:
        return None, None
    ui, vi = int(round(u)), int(round(v))
    if ui < 0 or vi < 0 or ui >= w or vi >= h:
        return None, None
    r = max(int(win), 0)
    u0, u1 = max(ui - r, 0), min(ui + r + 1, w)
    v0, v1 = max(vi - r, 0), min(vi + r + 1, h)
    patch = depth[v0:v1, u0:u1].reshape(-1)
    if confidence is not None:
        weights = _normalise_confidence(confidence)[v0:v1, u0:u1].reshape(-1)
    else:
        weights = np.ones_like(patch)
    ok = np.isfinite(patch) & (patch > 0.0) & np.isfinite(weights) & (weights > 0.0)
    if not np.any(ok):
        return None, None
    z_vals = patch[ok]
    wts = weights[ok]
    z = _weighted_median(z_vals, wts)
    mad = _weighted_median(np.abs(z_vals - z), wts) if z_vals.size > 1 else 0.0
    sigma_z = max(1.4826 * mad, _DEPTH_SIGMA_FLOOR_M)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    jac = np.array([[z / fx, 0.0, (u - cx) / fx],
                    [0.0, z / fy, (v - cy) / fy],
                    [0.0, 0.0, 1.0]])
    cov_in = np.diag([_PIXEL_SIGMA_PX ** 2, _PIXEL_SIGMA_PX ** 2, sigma_z ** 2])
    cov = jac @ cov_in @ jac.T
    return np.array([x, y, z]), cov


def mask_points(mask: np.ndarray, depth_m: np.ndarray, K: np.ndarray,
                confidence: np.ndarray | None = None, min_conf: float = 0.5,
                stride: int = 1) -> np.ndarray:
    """Points (N, 3) in the camera optical frame for mask pixels with valid, confident depth."""
    depth = np.asarray(depth_m, dtype=np.float64)
    m = np.asarray(mask, dtype=bool)
    if m.shape != depth.shape[:2]:
        raise ValueError('mask and depth shapes differ')
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    s = max(int(stride), 1)
    sel = np.zeros_like(m)
    sel[::s, ::s] = m[::s, ::s]
    sel &= np.isfinite(depth) & (depth > 0.0)
    if confidence is not None:
        conf = _normalise_confidence(confidence)
        sel &= np.isfinite(conf) & (conf >= float(min_conf))
    vs, us = np.nonzero(sel)
    if vs.size == 0:
        return np.zeros((0, 3))
    z = depth[vs, us]
    x = (us - K[0, 2]) * z / K[0, 0]
    y = (vs - K[1, 2]) * z / K[1, 1]
    return np.column_stack([x, y, z])
