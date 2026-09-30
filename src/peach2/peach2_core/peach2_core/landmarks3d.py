"""
Bag bottom / neck / tie from a single-view (or fused) bag point cloud.

Geometry model: the bag is a body of revolution hanging within 45 deg of vertical; it is
wide at the bottom (fruit) and narrows to a neck where it is tied. Only the camera-facing
surface is usually visible, so the axis line is fitted through per-slice circle centres
(a partial arc's centroid is biased toward the camera by up to 2R/pi, ~25 mm for R = 40 mm).
"""
from __future__ import annotations

import numpy as np

from .types import axial_lateral_cov, failed_landmarks3d, Landmark, Landmarks3D, unit

_MIN_POINTS = 40
_MIN_SPAN_M = 0.03
_MIN_BIN_POINTS = 8
_MIN_FINE_POINTS = 5
_MAX_AXIS_TILT_DEG = 45.0
_PCA_MIN_RATIO = 1.2
_ARC_SECTORS = 12
_MIN_ARC_SECTORS = 3  # >= 90 deg of arc before trusting a circle centre
_MAX_SLICE_RADIUS_M = 0.15
_SIGMA_POINT_FLOOR_M = 0.0005
_SIGMA_LATERAL_FLOOR_M = 0.001
_REFINE_ITERATIONS = 2


def _basis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ref = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(axis, ref)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(axis, e1)


def _clamp_to_cone(axis: np.ndarray, up: np.ndarray, max_deg: float) -> tuple[np.ndarray, bool]:
    a = axis if float(axis @ up) >= 0.0 else -axis
    cos_max = np.cos(np.radians(max_deg))
    if float(a @ up) >= cos_max - 1e-12:
        return a, False
    perp = unit(a - float(a @ up) * up)
    if perp is None:
        return up.copy(), True
    return cos_max * up + np.sin(np.radians(max_deg)) * perp, True


def _robust_sigma(x: np.ndarray) -> float:
    if x.size == 0:
        return 0.0
    return float(1.4826 * np.median(np.abs(x - np.median(x))))


def _tukey(res: np.ndarray, scale: float) -> np.ndarray:
    u = res / (4.685 * scale)
    return np.where(np.abs(u) < 1.0, (1.0 - u * u) ** 2, 0.0)


def _fit_circle(xy: np.ndarray) -> tuple[np.ndarray, float, np.ndarray] | None:
    """
    Kasa start, then Tukey-weighted Gauss-Newton on the geometric distance.

    Kasa alone is biased toward the arc on partial noisy arcs, and trimming around a fit
    already pulled by flying pixels keeps them; the redescending weights drop them.
    Returns (centre, r, residuals of all points).
    """
    if xy.shape[0] < 5:
        return None
    a = np.column_stack([xy[:, 0], xy[:, 1], np.ones(xy.shape[0])])
    b = -(xy[:, 0] ** 2 + xy[:, 1] ** 2)
    try:
        sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    centre = -0.5 * sol[:2]
    r2 = float(centre @ centre - sol[2])
    if not np.isfinite(r2) or r2 <= 0.0:
        return None
    r = float(np.sqrt(r2))
    for _ in range(10):
        diff = xy - centre
        dist = np.maximum(np.linalg.norm(diff, axis=1), 1e-9)
        res = dist - r
        w = _tukey(res - np.median(res), max(_robust_sigma(res), 2e-4))
        if np.count_nonzero(w) < 5:
            return None
        sw = np.sqrt(w)
        jac = np.column_stack([-diff / dist[:, None], -np.ones(xy.shape[0])]) * sw[:, None]
        step, *_ = np.linalg.lstsq(jac, -res * sw, rcond=None)
        centre = centre + step[:2]
        r = r + float(step[2])
        if not np.isfinite(r) or r <= 0.0:
            return None
        if np.linalg.norm(step) < 1e-7:
            break
    return centre, r, np.linalg.norm(xy - centre, axis=1) - r


def _arc_sectors(xy: np.ndarray, centre: np.ndarray) -> int:
    ang = np.arctan2(xy[:, 1] - centre[1], xy[:, 0] - centre[0])
    idx = np.floor((ang + np.pi) / (2.0 * np.pi) * _ARC_SECTORS).astype(int) % _ARC_SECTORS
    counts = np.bincount(idx, minlength=_ARC_SECTORS)
    return int(np.count_nonzero(counts >= 2))


def _slice_centres(pts: np.ndarray, origin: np.ndarray, axis: np.ndarray, edges: np.ndarray):
    """Per-bin axis-line centre estimates; returns (centres, weights, residuals, n_fallback)."""
    e1, e2 = _basis(axis)
    rel = pts - origin
    t = rel @ axis
    centres, weights, residuals = [], [], []
    n_fallback = 0
    for i in range(edges.size - 1):
        sel = (t >= edges[i]) & (t <= edges[i + 1]) if i == edges.size - 2 else \
            (t >= edges[i]) & (t < edges[i + 1])
        if np.count_nonzero(sel) < _MIN_BIN_POINTS:
            continue
        xy = np.column_stack([rel[sel] @ e1, rel[sel] @ e2])
        tc = 0.5 * (edges[i] + edges[i + 1])
        fit = _fit_circle(xy)
        extent = float(np.max(np.ptp(xy, axis=0)))
        if (fit is not None and fit[1] < _MAX_SLICE_RADIUS_M and fit[1] < 3.0 * max(extent, 1e-3)
                and _arc_sectors(xy, fit[0]) >= _MIN_ARC_SECTORS):
            c2, _, res = fit
            residuals.append(res)
            w = float(np.count_nonzero(sel))
        else:
            c2 = np.median(xy, axis=0)
            n_fallback += 1
            w = 0.25 * float(np.count_nonzero(sel))
        centres.append(origin + tc * axis + c2[0] * e1 + c2[1] * e2)
        weights.append(w)
    return (np.array(centres).reshape(-1, 3), np.array(weights), residuals, n_fallback)


def _fit_line(centres: np.ndarray, weights: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Weighted PCA line with Tukey reweighting of slice centres far from the line."""
    w = weights.copy()
    m = np.zeros(3)
    direction = np.zeros(3)
    for _ in range(6):
        wn = w / w.sum()
        m = (wn[:, None] * centres).sum(axis=0)
        d = centres - m
        vals, vecs = np.linalg.eigh((wn[:, None] * d).T @ d)
        direction = vecs[:, int(np.argmax(vals))]
        off = np.linalg.norm(d - np.outer(d @ direction, direction), axis=1)
        tw = _tukey(off, max(1.4826 * float(np.median(off)), 5e-4))
        if np.count_nonzero(tw) < 3:
            break
        w = weights * tw
    return m, direction


def _smooth_nan(values: np.ndarray) -> np.ndarray:
    out = values.copy()
    for i in range(values.size):
        win = values[max(i - 1, 0):i + 2]
        win = win[np.isfinite(win)]
        if np.isfinite(values[i]) and win.size:
            out[i] = float(np.median(win))
    return out


def _slice_radius(t: np.ndarray, d: np.ndarray, lo: float, hi: float, min_pts: int) -> float:
    sel = (t >= lo) & (t < hi)
    if np.count_nonzero(sel) < min_pts:
        return float('nan')
    return float(np.percentile(d[sel], 95))


def landmarks_from_points(points: np.ndarray, gravity: np.ndarray,
                          axis_hint: np.ndarray | None = None,
                          sigma_point_m: np.ndarray | None = None, n_bins: int = 12,
                          taper_ratio: float = 0.85) -> Landmarks3D:
    """
    Bottom / neck / tie of one bag from its points (same frame as `gravity`).

    Axis: PCA (or `axis_hint` when PCA is ill-conditioned) refined through per-slice circle
    centres, oriented against gravity and clamped to 45 deg from -gravity. Radius profile:
    per-slice 95th percentile distance to the axis line. Bottom: 5th-percentile axial position
    (extrapolated to the end under a uniform axial density) on the axis line. Neck: start of
    the contiguous narrow-end segment with radius <= r_min / taper_ratio, refined with
    quarter-bin slices. Covariances: axis-line prediction variance from the circle-centre
    residuals (lateral) and point noise plus bin width (axial; neck >= half a bin).
    """
    flags: list[str] = []
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    if pts.shape[0] < _MIN_POINTS:
        return failed_landmarks3d(['too_few_points'])
    g = unit(gravity)
    if g is None:
        return failed_landmarks3d(['gravity_invalid'])
    up = -g
    if not (0.0 < taper_ratio < 1.0) or n_bins < 4:
        raise ValueError('taper_ratio must be in (0, 1) and n_bins >= 4')

    centre = np.median(pts, axis=0)
    _, sv, vt = np.linalg.svd(pts - centre, full_matrices=False)
    hint = unit(axis_hint) if axis_hint is not None else None
    if sv.size < 2 or sv[0] < _PCA_MIN_RATIO * max(sv[1], 1e-12):
        axis = hint if hint is not None else up.copy()
        flags.append('axis_from_hint' if hint is not None else 'axis_from_gravity')
    else:
        axis = vt[0]
    axis, clamped = _clamp_to_cone(axis, up, _MAX_AXIS_TILT_DEG)

    rel = pts - centre
    t0 = rel @ axis
    d0 = np.linalg.norm(rel - np.outer(t0, axis), axis=1)
    radial_limit = np.median(d0) + 4.0 * max(_robust_sigma(d0), 1e-3)
    keep = d0 <= radial_limit
    if np.count_nonzero(~keep):
        flags.append('radial_outliers_removed')
    pts = pts[keep]
    if pts.shape[0] < _MIN_POINTS:
        return failed_landmarks3d(flags + ['too_few_points'])

    origin = np.median(pts, axis=0)
    residuals: list[np.ndarray] = []
    centres = np.zeros((0, 3))
    weights = np.zeros(0)
    n_fallback = 0
    for _ in range(_REFINE_ITERATIONS):
        t = (pts - origin) @ axis
        lo, hi = np.percentile(t, [1.0, 99.0])
        if hi - lo < _MIN_SPAN_M:
            return failed_landmarks3d(flags + ['bag_axis_too_short'])
        edges = np.linspace(lo, hi, n_bins + 1)
        centres, weights, residuals, n_fallback = _slice_centres(pts, origin, axis, edges)
        if centres.shape[0] < 3:
            flags.append('axis_line_underdetermined')
            break
        origin, new_axis = _fit_line(centres, weights)
        axis, clamped = _clamp_to_cone(new_axis, up, _MAX_AXIS_TILT_DEG)
    if clamped:
        flags.append('axis_gravity_clamped')
    if n_fallback:
        flags.append('slice_centroid_fallback')

    rel = pts - origin
    t = rel @ axis
    d = np.linalg.norm(rel - np.outer(t, axis), axis=1)
    lo, hi = np.percentile(t, [1.0, 99.0])
    if hi - lo < _MIN_SPAN_M:
        return failed_landmarks3d(flags + ['bag_axis_too_short'])
    edges = np.linspace(lo, hi, n_bins + 1)
    bin_w = float(edges[1] - edges[0])
    centres_t = 0.5 * (edges[:-1] + edges[1:])
    upper_edges = edges[1:].copy()
    upper_edges[-1] += 1e-12
    radii = np.array([_slice_radius(t, d, edges[i], upper_edges[i], _MIN_BIN_POINTS)
                      for i in range(n_bins)])
    valid = np.isfinite(radii)
    if int(valid.sum()) < 4:
        return failed_landmarks3d(flags + ['radius_profile_failed'])
    radii_s = _smooth_nan(radii)

    if sigma_point_m is not None:
        sig = np.asarray(sigma_point_m, dtype=np.float64).reshape(-1)
        sigma_p = float(np.median(sig[np.isfinite(sig)])) if np.any(np.isfinite(sig)) else 0.0
    elif residuals:
        sigma_p = _robust_sigma(np.concatenate(residuals))
    else:
        sigma_p = 0.0
    sigma_p = max(sigma_p, _SIGMA_POINT_FLOOR_M)

    quarter = max(n_bins // 4, 1)
    low_r = radii_s[:quarter][np.isfinite(radii_s[:quarter])]
    high_r = radii_s[-quarter:][np.isfinite(radii_s[-quarter:])]
    if low_r.size and high_r.size:
        r_low, r_high = float(np.median(low_r)), float(np.median(high_r))
        if r_high * taper_ratio > r_low:
            flags.append('taper_inverted_gravity_kept')
        elif r_high >= taper_ratio * r_low:
            flags.append('taper_unclear')

    upper = np.arange(n_bins // 2, n_bins)
    upper = upper[np.isfinite(radii_s[upper])]
    lower = np.arange(0, n_bins // 2)
    lower = lower[np.isfinite(radii_s[lower])]
    neck_conf = 0.7
    if upper.size == 0 or lower.size == 0:
        return failed_landmarks3d(flags + ['radius_profile_failed'])
    i_min = int(upper[np.argmin(radii_s[upper])])
    r_min = float(radii_s[i_min])
    r_body = float(np.max(radii_s[lower]))
    if r_min < taper_ratio * r_body:
        r_thr = r_min / taper_ratio
        k = i_min
        while k - 1 >= 0 and np.isfinite(radii_s[k - 1]) and radii_s[k - 1] <= r_thr:
            k -= 1
        t_neck = float(edges[k])
        # Quarter-bin slices around the coarse boundary locate the first narrow slice.
        fine_w = 0.25 * bin_w
        fine_lo = edges[max(k - 1, 0)]
        fine_hi = edges[min(k + 1, n_bins)]
        found = False
        for s in np.arange(fine_lo, fine_hi - fine_w + 1e-9, 0.5 * fine_w):
            if _slice_radius(t, d, s, s + fine_w, _MIN_FINE_POINTS) <= r_thr:
                t_neck = float(s)
                found = True
                break
        if not found:
            neck_conf = 0.4
            flags.append('neck_coarse')
        tie_t = float(hi)
        tie_valid = tie_t - t_neck > 0.5 * bin_w
    else:
        flags.append('neck_at_top_no_taper')
        neck_conf = 0.2
        t_neck = float(centres_t[-1])
        tie_t = float(hi)
        tie_valid = False

    q05, q95 = np.percentile(t, [5.0, 95.0])
    t_bottom = float(q05 - (q95 - q05) * 0.05 / 0.90)
    length = t_neck - t_bottom
    if length < _MIN_SPAN_M:
        return failed_landmarks3d(flags + ['bag_length_degenerate'])

    if centres.shape[0] >= 3:
        tc = (centres - origin) @ axis
        lat = centres - origin - np.outer(tc, axis)
        res_lat = np.linalg.norm(lat, axis=1)
        sigma_c = max(1.4826 * float(np.median(res_lat)), sigma_p)
        sxx = float(np.sum((tc - tc.mean()) ** 2))
        n_c = tc.size

        def sigma_lateral(tq: float) -> float:
            var = sigma_c ** 2 * (1.0 / n_c + (tq - tc.mean()) ** 2 / max(sxx, 1e-9))
            return max(float(np.sqrt(var)), 0.5 * sigma_p, _SIGMA_LATERAL_FLOOR_M)
    else:
        def sigma_lateral(tq: float) -> float:
            return max(float(np.median(d)), _SIGMA_LATERAL_FLOOR_M)

    sig_ax_bottom = float(np.hypot(sigma_p, 0.25 * bin_w))
    sig_ax_neck = max(0.5 * bin_w, sigma_p)
    if neck_conf < 0.7:
        sig_ax_neck = max(sig_ax_neck, bin_w)
    bottom_conf = 0.8 if n_fallback == 0 else 0.5

    bottom = Landmark(True, origin + t_bottom * axis,
                      axial_lateral_cov(axis, sigma_lateral(t_bottom), sig_ax_bottom),
                      bottom_conf)
    neck = Landmark(True, origin + t_neck * axis,
                    axial_lateral_cov(axis, sigma_lateral(t_neck), sig_ax_neck), neck_conf)
    if tie_valid:
        tie = Landmark(True, origin + tie_t * axis,
                       axial_lateral_cov(axis, sigma_lateral(tie_t), bin_w), 0.3)
    else:
        tie = Landmark(False, np.full(3, np.nan), np.full((3, 3), np.nan), 0.0)
    d95 = float(2.0 * np.percentile(d[(t >= t_bottom) & (t <= t_neck)], 95))
    return Landmarks3D(ok=True, bottom=bottom, neck=neck, tie=tie, axis=axis.copy(),
                       d95_m=d95, length_m=float(length), flags=flags)
