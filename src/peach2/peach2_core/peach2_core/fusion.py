"""Multi-view fusion of per-view bag landmarks into one model with 95% uncertainties."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .types import axial_lateral_cov, Landmarks3D, unit

_Z95 = 1.96
_MAD_TO_SIGMA = 1.4826
_HUBER_K = 1.345
_HUBER_ITERATIONS = 10
_AXIS_LINE_CONSISTENCY_DEG = 10.0
_ANGLE_SCALE_FLOOR_RAD = np.radians(2.0)


@dataclass
class ViewSample:
    landmarks: Landmarks3D
    stamp_s: float
    camera_distance_m: float


@dataclass
class FusedModel:
    ok: bool
    bottom: np.ndarray
    neck: np.ndarray
    tie: np.ndarray | None
    axis: np.ndarray
    d95_m: float
    length_m: float
    sigma_lateral95_m: float
    sigma_axial95_m: float
    theta95_deg: float
    n_views: int
    flags: list[str] = field(default_factory=list)
    # 1-sigma covariances consistent with the 95% figures above (for BagLandmark.covariance).
    bottom_cov: np.ndarray | None = None
    neck_cov: np.ndarray | None = None
    fit_rmse_m: float = 0.0
    inlier_ratio: float = 0.0


def _failed(n_views: int, flags: list[str]) -> FusedModel:
    nan3 = np.full(3, np.nan)
    return FusedModel(ok=False, bottom=nan3.copy(), neck=nan3.copy(), tie=None, axis=nan3.copy(),
                      d95_m=float('nan'), length_m=float('nan'),
                      sigma_lateral95_m=float('inf'), sigma_axial95_m=float('inf'),
                      theta95_deg=float('inf'), n_views=n_views, flags=flags)


def _regularized(cov: np.ndarray) -> np.ndarray:
    c = np.asarray(cov, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(c)):
        c = (0.01 ** 2) * np.eye(3)
    return 0.5 * (c + c.T) + 1e-12 * np.eye(3)


def _huber_mean(points: np.ndarray, covs: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """IRLS Huber mean with per-view scale sqrt(trace(C)/3); returns (mean, huber weights)."""
    scale = np.array([np.sqrt(max(np.trace(c) / 3.0, 1e-12)) for c in covs])
    prec = 1.0 / scale ** 2
    x = np.median(points, axis=0)
    hw = np.ones(points.shape[0])
    for _ in range(_HUBER_ITERATIONS):
        r = np.linalg.norm(points - x, axis=1) / scale
        hw = np.where(r > _HUBER_K, _HUBER_K / np.maximum(r, 1e-12), 1.0)
        w = prec * hw
        x_new = (w[:, None] * points).sum(axis=0) / w.sum()
        if np.linalg.norm(x_new - x) < 1e-7:
            x = x_new
            break
        x = x_new
    return x, hw


def _fused_cov(covs: list[np.ndarray], hw: np.ndarray) -> np.ndarray:
    info = sum(w * np.linalg.inv(c) for c, w in zip(covs, hw))
    return np.linalg.inv(info)


def _mad_sigma(values: np.ndarray) -> float:
    if values.size < 2:
        return 0.0
    return float(_MAD_TO_SIGMA * np.median(np.abs(values - np.median(values))))


def _lateral_sigma(cov: np.ndarray, e1: np.ndarray, e2: np.ndarray) -> float:
    B = np.column_stack([e1, e2])
    return float(np.sqrt(max(np.linalg.eigvalsh(B.T @ cov @ B).max(), 0.0)))


def _basis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ref = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(axis, ref)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(axis, e1)


def fuse_views(views: list[ViewSample], min_views: int = 1) -> FusedModel:
    """
    Huber-fuse per-view bottom / neck / axis.

    sigma95 = max(MAD * 1.4826 of view residuals, sigma propagated from the per-view
    covariances into the fused estimate) * 1.96; a single view uses its own covariance.
    Lateral: bottom and neck residuals across the axis; axial: neck residuals along the axis;
    theta: per-view axis deviation vs. the bottom/neck lateral sigma over the bag length.
    """
    usable = [v for v in views
              if v.landmarks.ok and v.landmarks.bottom.valid and v.landmarks.neck.valid
              and unit(v.landmarks.axis) is not None]
    need = max(int(min_views), 1)
    if len(usable) < need:
        return _failed(len(usable), ['insufficient_views'])
    flags: list[str] = []
    axes = np.array([unit(v.landmarks.axis) for v in usable])
    ref = unit(axes.sum(axis=0))
    if ref is None:
        return _failed(len(usable), ['axis_sense_conflict'])
    keep = axes @ ref > 0.0
    if not keep.all():
        flags.append('axis_sense_views_dropped')
        usable = [v for v, k in zip(usable, keep) if k]
        axes = axes[keep]
        if len(usable) < need:
            return _failed(len(usable), flags + ['insufficient_views'])
    n = len(usable)

    bottoms = np.array([v.landmarks.bottom.position for v in usable], dtype=np.float64)
    necks = np.array([v.landmarks.neck.position for v in usable], dtype=np.float64)
    b_covs = [_regularized(v.landmarks.bottom.cov) for v in usable]
    n_covs = [_regularized(v.landmarks.neck.cov) for v in usable]
    bottom, hw_b = _huber_mean(bottoms, b_covs)
    neck, hw_n = _huber_mean(necks, n_covs)

    axis = unit(axes.mean(axis=0))
    for _ in range(_HUBER_ITERATIONS):
        ang = np.arccos(np.clip(axes @ axis, -1.0, 1.0))
        scale = max(_MAD_TO_SIGMA * float(np.median(ang)), _ANGLE_SCALE_FLOOR_RAD)
        w = np.where(ang > _HUBER_K * scale, _HUBER_K * scale / np.maximum(ang, 1e-12), 1.0)
        new_axis = unit((w[:, None] * axes).sum(axis=0))
        if new_axis is None:
            return _failed(n, flags + ['axis_fusion_failed'])
        if np.linalg.norm(new_axis - axis) < 1e-9:
            break
        axis = new_axis

    length = float((neck - bottom) @ axis)
    if length <= 0.0:
        return _failed(n, flags + ['fused_length_nonpositive'])
    line = unit(neck - bottom)
    if np.degrees(np.arccos(np.clip(line @ axis, -1.0, 1.0))) > _AXIS_LINE_CONSISTENCY_DEG:
        flags.append('axis_line_inconsistent')

    e1, e2 = _basis(axis)
    cov_b = _fused_cov(b_covs, hw_b)
    cov_n = _fused_cov(n_covs, hw_n)
    res_b = bottoms - bottom
    res_n = necks - neck
    lat_mad = max(_mad_sigma(np.concatenate([res_b @ e1, res_n @ e1])),
                  _mad_sigma(np.concatenate([res_b @ e2, res_n @ e2])))
    lat_prop_b = _lateral_sigma(cov_b, e1, e2)
    lat_prop_n = _lateral_sigma(cov_n, e1, e2)
    sigma_lat = max(lat_mad, lat_prop_b, lat_prop_n)
    ax_mad_n = _mad_sigma(res_n @ axis)
    ax_prop_n = float(np.sqrt(max(axis @ cov_n @ axis, 0.0)))
    sigma_ax_n = max(ax_mad_n, ax_prop_n)
    sigma_ax_b = max(_mad_sigma(res_b @ axis), float(np.sqrt(max(axis @ cov_b @ axis, 0.0))))
    theta_mad = max(_mad_sigma(axes @ e1), _mad_sigma(axes @ e2))
    theta_prop = float(np.hypot(lat_prop_b, lat_prop_n)) / length
    sigma_theta = max(theta_mad, theta_prop)

    ties = [(v.landmarks.tie.position, _regularized(v.landmarks.tie.cov)) for v in usable
            if v.landmarks.tie.valid and np.all(np.isfinite(v.landmarks.tie.position))]
    tie = None
    if ties:
        tie, _ = _huber_mean(np.array([p for p, _ in ties]), [c for _, c in ties])
    d95 = float(np.median([v.landmarks.d95_m for v in usable]))

    err = np.concatenate([np.linalg.norm(res_b, axis=1), np.linalg.norm(res_n, axis=1)])
    inliers = (hw_b >= 1.0) & (hw_n >= 1.0)
    return FusedModel(
        ok=True, bottom=bottom, neck=neck, tie=tie, axis=axis, d95_m=d95, length_m=length,
        sigma_lateral95_m=_Z95 * sigma_lat, sigma_axial95_m=_Z95 * sigma_ax_n,
        theta95_deg=float(np.degrees(_Z95 * sigma_theta)), n_views=n, flags=flags,
        bottom_cov=axial_lateral_cov(axis, sigma_lat, sigma_ax_b),
        neck_cov=axial_lateral_cov(axis, sigma_lat, sigma_ax_n),
        fit_rmse_m=float(np.sqrt(np.mean(err * err))),
        inlier_ratio=float(np.mean(inliers)))
