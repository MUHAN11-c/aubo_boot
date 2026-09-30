"""2D bag bottom / neck / tie from a segmentation mask (pixel coordinates u, v)."""
from __future__ import annotations

import numpy as np

from .types import failed_landmarks2d, Landmarks2D, unit

_MIN_PIXELS = 50
_PCA_MIN_RATIO = 1.1
# |cos| between the PCA axis and the image gravity above which gravity decides polarity.
_GRAVITY_DECISIVE = 0.5
_GRAVITY_USABLE = 0.1
_TAPER_RATIO = 0.85
_MIN_BIN_PIXELS = 3


def _smooth(values: np.ndarray) -> np.ndarray:
    out = values.copy()
    for i in range(values.size):
        win = values[max(i - 1, 0):i + 2]
        win = win[np.isfinite(win)]
        if np.isfinite(values[i]) and win.size:
            out[i] = float(np.median(win))
    return out


def landmarks_from_mask(mask: np.ndarray, gravity_px: np.ndarray,
                        n_bins: int = 20) -> Landmarks2D:
    """
    Bottom / neck / tie pixels of one bag mask.

    `gravity_px` is the image projection of base -Z (unit or any length; direction only).
    Polarity: gravity decides when it is well aligned with the long axis (bag hangs);
    otherwise the wide end is the bottom; a taper/gravity conflict lowers confidence.
    Widths are robust (2nd..98th percentile) extents across the axis per bin, smoothed.
    Bottom = outer-edge midpoint of the wide end; neck = start of the contiguous narrow-end
    run with width <= w_min / 0.85; tie = narrow-end tip.
    """
    m = np.asarray(mask, dtype=bool)
    vs, us = np.nonzero(m)
    if vs.size < _MIN_PIXELS:
        return failed_landmarks2d(['mask_too_small'])
    if n_bins < 6:
        raise ValueError('n_bins must be >= 6')
    g = unit(np.asarray(gravity_px, dtype=np.float64).reshape(2))
    xy = np.column_stack([us, vs]).astype(np.float64)
    c = xy.mean(axis=0)
    rel = xy - c
    vals, vecs = np.linalg.eigh(rel.T @ rel / rel.shape[0])
    flags: list[str] = []
    if vals[1] < _PCA_MIN_RATIO ** 2 * max(vals[0], 1e-12):
        if g is None:
            return failed_landmarks2d(['axis_ill_conditioned'])
        axis = -g
        flags.append('axis_from_gravity')
    else:
        axis = vecs[:, 1]
    n = np.array([-axis[1], axis[0]])
    t = rel @ axis
    s = rel @ n
    lo, hi = np.percentile(t, [0.5, 99.5])
    if hi - lo < 10.0:
        return failed_landmarks2d(flags + ['mask_too_short'])
    edges = np.linspace(lo, hi, n_bins + 1)
    widths = np.full(n_bins, np.nan)
    mids = np.full(n_bins, np.nan)
    for i in range(n_bins):
        sel = (t >= edges[i]) & (t <= edges[i + 1])
        if np.count_nonzero(sel) < _MIN_BIN_PIXELS:
            continue
        p_lo, p_hi = np.percentile(s[sel], [2.0, 98.0])
        widths[i] = p_hi - p_lo + 1.0
        mids[i] = 0.5 * (p_lo + p_hi)
    if np.count_nonzero(np.isfinite(widths)) < n_bins // 2:
        return failed_landmarks2d(flags + ['width_profile_failed'])
    widths = _smooth(widths)

    quarter = max(n_bins // 4, 1)
    w_start = float(np.nanmedian(widths[:quarter]))
    w_end = float(np.nanmedian(widths[-quarter:]))
    taper_clear = min(w_start, w_end) < _TAPER_RATIO * max(w_start, w_end)
    wide_at_start = w_start >= w_end
    conf = 0.8
    g_cos = float(axis @ g) if g is not None else 0.0
    if abs(g_cos) >= _GRAVITY_DECISIVE:
        bottom_at_start = g_cos < 0.0  # axis points against gravity from the start end
        if taper_clear and bottom_at_start != wide_at_start:
            flags.append('taper_gravity_conflict')
            conf = 0.3
    elif taper_clear:
        bottom_at_start = wide_at_start
        if abs(g_cos) >= _GRAVITY_USABLE and bottom_at_start != (g_cos < 0.0):
            flags.append('taper_gravity_conflict')
            conf = 0.3
        else:
            conf = 0.6
    elif abs(g_cos) >= _GRAVITY_USABLE:
        bottom_at_start = g_cos < 0.0
        flags.append('polarity_from_weak_gravity')
        conf = 0.4
    else:
        return failed_landmarks2d(flags + ['polarity_ambiguous'])
    if not bottom_at_start:
        axis, n = -axis, -n
        t, s = -t, -s
        edges = -edges[::-1]
        widths = widths[::-1].copy()
        mids = -mids[::-1]
    centres_t = 0.5 * (edges[:-1] + edges[1:])

    half = n_bins // 2
    upper = np.arange(half, n_bins)
    upper = upper[np.isfinite(widths[upper])]
    lower = np.arange(0, half)
    lower = lower[np.isfinite(widths[lower])]
    if upper.size == 0 or lower.size == 0:
        return failed_landmarks2d(flags + ['width_profile_failed'])
    i_min = int(upper[np.argmin(widths[upper])])
    w_min = float(widths[i_min])
    if w_min < _TAPER_RATIO * float(np.max(widths[lower])):
        thr = w_min / _TAPER_RATIO
        k = i_min
        while k - 1 >= 0 and np.isfinite(widths[k - 1]) and widths[k - 1] <= thr:
            k -= 1
        t_neck = float(edges[k])
        # Quarter-bin slices: a coarse bin straddling the shoulder reports the shoulder width.
        bin_w = float(edges[1] - edges[0])
        fine_w = max(0.25 * bin_w, 2.0)
        for lo_s in np.arange(edges[max(k - 2, 0)], edges[min(k + 1, n_bins)] - fine_w + 1e-9,
                              0.5 * fine_w):
            sel = (t >= lo_s) & (t < lo_s + fine_w)
            if np.count_nonzero(sel) < _MIN_BIN_PIXELS:
                continue
            p_lo, p_hi = np.percentile(s[sel], [2.0, 98.0])
            if p_hi - p_lo + 1.0 <= thr:
                t_neck = float(lo_s)
                break
        neck_mid = mids[k] if np.isfinite(mids[k]) else 0.0
    else:
        flags.append('neck_at_end_no_taper')
        conf = min(conf, 0.3)
        t_neck = float(centres_t[-1])
        neck_mid = mids[-1] if np.isfinite(mids[-1]) else 0.0
    first = int(lower[0])
    bottom_mid = mids[first] if np.isfinite(mids[first]) else 0.0
    last = int(np.nonzero(np.isfinite(widths))[0][-1])
    tie_mid = mids[last] if np.isfinite(mids[last]) else 0.0
    bottom_px = c + float(edges[0]) * axis + bottom_mid * n
    neck_px = c + t_neck * axis + neck_mid * n
    tie_px = c + float(edges[-1]) * axis + tie_mid * n
    return Landmarks2D(ok=True, bottom_px=bottom_px, neck_px=neck_px, tie_px=tie_px,
                       axis_px=axis.copy(), confidence=conf, widths_px=widths, flags=flags)
