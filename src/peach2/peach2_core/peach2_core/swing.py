"""Pendulum swing amplitude / period from a short stationary-camera window of bag positions."""
from __future__ import annotations

import numpy as np
from scipy.optimize import minimize_scalar

_MIN_SAMPLES = 8
# The window must hold at least this fraction of one period for a period to be claimed.
_MIN_CYCLES = 0.5
_GRID = 200


def _fit(t: np.ndarray, x: np.ndarray, f: float) -> tuple[float, np.ndarray]:
    """Joint least squares x ~ c + d t + a sin + b cos (shared frequency, per axis)."""
    w = 2.0 * np.pi * f
    A = np.column_stack([np.ones_like(t), t, np.sin(w * t), np.cos(w * t)])
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    res = x - A @ coef
    return float(np.sum(res * res)), coef


def estimate_swing(times_s: np.ndarray, positions: np.ndarray) -> tuple[float, float]:
    """
    (peak amplitude [m], period [s]); (nan, nan) when samples are insufficient.

    A linear drift is fitted jointly with the sinusoid (detrending first would eat part of the
    swing when the window holds about one period). Samples may be unevenly spaced. The peak
    amplitude of the 3D elliptical motion is the largest singular value of the [a b] matrix.
    """
    t = np.asarray(times_s, dtype=np.float64).reshape(-1)
    x = np.asarray(positions, dtype=np.float64)
    if x.ndim == 1:
        x = x[:, None]
    if x.shape[0] != t.size:
        raise ValueError('times_s and positions lengths differ')
    ok = np.isfinite(t) & np.all(np.isfinite(x), axis=1)
    t, x = t[ok], x[ok]
    if t.size < _MIN_SAMPLES:
        return float('nan'), float('nan')
    order = np.argsort(t)
    t, x = t[order] - t[order][0], x[order]
    span = float(t[-1])
    if span <= 0.0:
        return float('nan'), float('nan')
    dt_med = float(np.median(np.diff(t)))
    if dt_med <= 0.0:
        return float('nan'), float('nan')
    f_lo = _MIN_CYCLES / span
    f_hi = 0.5 / dt_med
    if f_hi <= f_lo:
        return float('nan'), float('nan')
    grid = np.linspace(f_lo, f_hi, _GRID)
    costs = [_fit(t, x, f)[0] for f in grid]
    i = int(np.argmin(costs))
    lo_f = grid[max(i - 1, 0)]
    hi_f = grid[min(i + 1, grid.size - 1)]
    if hi_f > lo_f:
        best = minimize_scalar(lambda f: _fit(t, x, f)[0], bounds=(lo_f, hi_f), method='bounded')
        f = float(best.x)
    else:
        f = float(grid[i])
    _, coef = _fit(t, x, f)
    ab = coef[2:4, :].T  # (dims, 2)
    amplitude = float(np.linalg.svd(ab, compute_uv=False)[0])
    return amplitude, 1.0 / f
