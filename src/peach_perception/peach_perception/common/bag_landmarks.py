"""袋颈/袋底几何关键点（半径剖面，第一实现；无 ROS）."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from peach_perception.common.geometry import (
    axis_radial_distance,
    fit_sphere_robust,
    unit_vector as _unit,
)

OCCLUSION_CLEAR = 'clear'
OCCLUSION_LEAF = 'leaf_occluded'
OCCLUSION_BRANCH = 'branch_blocked'
OCCLUSION_NEIGHBOR = 'neighbor_overlap'
OCCLUSION_DAMAGED = 'damaged_or_wet'


@dataclass
class Landmark3D:
    """单个 3D 关键点及其可见性."""

    xyz: Optional[np.ndarray] = None
    confidence: float = 0.0
    visible: bool = False


@dataclass
class BagLandmarks:
    """袋语义端点。球只作袋内果实包络先验."""

    neck_left: Landmark3D = field(default_factory=Landmark3D)
    neck_right: Landmark3D = field(default_factory=Landmark3D)
    bottom_left: Landmark3D = field(default_factory=Landmark3D)
    bottom_right: Landmark3D = field(default_factory=Landmark3D)
    neck_center: Optional[np.ndarray] = None
    bottom_center: Optional[np.ndarray] = None
    bag_axis: Optional[np.ndarray] = None
    d95_m: float = 0.0
    fruit_prior_center: Optional[np.ndarray] = None
    fruit_prior_radius_m: float = 0.0
    occlusion_class: str = OCCLUSION_CLEAR
    sigma_position_m: float = 0.02
    sigma_axis_deg: float = 8.0
    flags: list = field(default_factory=list)


def _mid(left: Landmark3D, right: Landmark3D) -> Optional[np.ndarray]:
    """左右缘中点；缺一侧则用可见侧."""
    if left.visible and right.visible:
        return 0.5 * (left.xyz + right.xyz)
    if left.visible:
        return np.asarray(left.xyz, dtype=np.float64)
    if right.visible:
        return np.asarray(right.xyz, dtype=np.float64)
    return None


def classify_occlusion(
        mask_area_ratio: float, edge_touch: bool, neighbor_gap_m: float,
        valid_depth_ratio: float, bbox_aspect: float) -> str:
    """
    成本敏感遮挡粗分类.

    错误放行枝遮挡代价高于漏检；邻果重叠优先于树叶。
    """
    if neighbor_gap_m >= 0.0 and neighbor_gap_m < 0.04:
        return OCCLUSION_NEIGHBOR
    if valid_depth_ratio < 0.25 and mask_area_ratio > 0.02:
        return OCCLUSION_BRANCH
    if edge_touch and mask_area_ratio < 0.08:
        return OCCLUSION_LEAF
    if bbox_aspect > 2.8 and valid_depth_ratio < 0.45:
        return OCCLUSION_DAMAGED
    if mask_area_ratio < 0.01:
        return OCCLUSION_LEAF
    return OCCLUSION_CLEAR


TAPER_RATIO = 0.85
_PROFILE_BINS = 12
_MIN_SPAN_M = 0.03


def _section_extrema(points: np.ndarray, axis: np.ndarray,
                     band: np.ndarray) -> tuple:
    """轴向分位带内垂直轴的左右极值点."""
    if band.shape[0] < 4:
        return None, None
    radial = band - np.outer(band @ axis, axis)
    centered = radial - np.median(radial, axis=0)
    if centered.shape[0] < 4:
        return None, None
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    lateral = vt[0]
    coord = centered @ lateral
    left = band[int(np.argmin(coord))]
    right = band[int(np.argmax(coord))]
    return left, right


def _long_axis(points: np.ndarray, gravity_u: np.ndarray) -> tuple:
    """袋长轴：点云主方向，符号翻到逆重力；病态则用逆重力."""
    centered = points - np.median(points, axis=0)
    try:
        _, singular, vt = np.linalg.svd(centered, full_matrices=False)
    except np.linalg.LinAlgError:
        return -gravity_u, False
    if singular.size < 2 or float(singular[0]) < 1e-9:
        return -gravity_u, False
    axis = _unit(vt[0])
    if axis is None:
        return -gravity_u, False
    if float(singular[1]) > 1e-9 and float(singular[0] / singular[1]) < 1.2:
        return -gravity_u, False
    if float(axis @ gravity_u) > 0.0:
        axis = -axis
    return axis, True


def _bin_radii(points: np.ndarray, axis: np.ndarray):
    """沿轴分箱，每段垂直半径 90 分位. 失败返回 (None, None, None)."""
    proj = points @ axis
    lo, hi = np.percentile(proj, [5.0, 95.0])
    if hi - lo < _MIN_SPAN_M:
        return None, None, None
    edges = np.linspace(lo, hi, _PROFILE_BINS + 1)
    radii = np.full(_PROFILE_BINS, np.nan, dtype=np.float64)
    for i in range(_PROFILE_BINS):
        if i >= _PROFILE_BINS - 1:
            sel = points[(proj >= edges[i]) & (proj <= edges[i + 1])]
        else:
            sel = points[(proj >= edges[i]) & (proj < edges[i + 1])]
        if sel.shape[0] < 8:
            continue
        mid = sel.mean(axis=0)
        radii[i] = float(np.percentile(
            axis_radial_distance(sel, axis, mid), 90))
    return proj, edges, radii


def _end_radius(radii: np.ndarray, high_end: bool) -> float:
    """一端若干有效段的半径中位数."""
    n_end = max(2, radii.size // 4)
    sl = radii[-n_end:] if high_end else radii[:n_end]
    finite = sl[np.isfinite(sl)]
    if finite.size == 0:
        return float('nan')
    return float(np.median(finite))


def _first_valid(valid: np.ndarray, start: int, stop: int, step: int):
    """半开区间内第一个有效段下标."""
    for i in range(start, stop, step):
        if 0 <= i < valid.size and bool(valid[i]):
            return i
    return None


def _end_bin_index(valid: np.ndarray, high_end: bool) -> Optional[int]:
    """只取端部若干段，不向袋子中部搜收窄."""
    bins = valid.size
    n_end = max(2, bins // 4)
    if high_end:
        found = _first_valid(valid, bins - 1, bins - n_end - 1, -1)
        if found is None:
            found = _first_valid(valid, bins - 1, -1, -1)
        return found
    found = _first_valid(valid, 0, n_end + 1, 1)
    if found is None:
        found = _first_valid(valid, 0, bins, 1)
    return found


def _axial_band(points: np.ndarray, proj: np.ndarray, edges: np.ndarray,
                index: int) -> np.ndarray:
    """第 index 段内的点；末段含上界."""
    lo_e = edges[index]
    hi_e = edges[index + 1]
    if index >= edges.size - 2:
        return points[(proj >= lo_e) & (proj <= hi_e)]
    return points[(proj >= lo_e) & (proj < hi_e)]


def _band_radius(band: np.ndarray, axis) -> float:
    """一段点云垂直轴的 90 分位半径."""
    axis_u = _unit(axis)
    if axis_u is None or band is None or band.shape[0] < 4:
        return float('nan')
    mid = band.mean(axis=0)
    return float(np.percentile(
        axis_radial_distance(band, axis_u, mid), 90))


def _slice_radius(points, axis, origin, t_m: float,
                  half_width_m: float = 0.015) -> float:
    """沿轴在 t 处切片的 90 分位半径."""
    axis_u = _unit(axis)
    if axis_u is None or points is None:
        return float('nan')
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] < 8:
        return float('nan')
    origin_v = np.asarray(origin, dtype=np.float64).reshape(3)
    axial = (pts - origin_v) @ axis_u
    selected = pts[np.abs(axial - float(t_m)) <= float(half_width_m)]
    return _band_radius(selected, axis_u)


def enforce_wide_bottom(bottom, neck, axis, points=None) -> tuple:
    """
    不变量：袋底宽、袋口窄，轴从袋底指向袋口.

    若当前「颈」明显更宽，对调两端并翻转轴.
    """
    axis_u = _unit(axis)
    if axis_u is None or bottom is None or neck is None:
        return bottom, neck, axis, False
    bottom_v = np.asarray(bottom, dtype=np.float64).reshape(3).copy()
    neck_v = np.asarray(neck, dtype=np.float64).reshape(3).copy()
    if float(np.dot(neck_v - bottom_v, axis_u)) < 0.0:
        axis_u = -axis_u
    length = float(np.dot(neck_v - bottom_v, axis_u))
    r_bottom = _slice_radius(points, axis_u, bottom_v, 0.0)
    r_neck = _slice_radius(points, axis_u, bottom_v, max(length, 0.02))
    if not (np.isfinite(r_bottom) and np.isfinite(r_neck)):
        return bottom_v, neck_v, axis_u, False
    if r_bottom + 1e-6 < TAPER_RATIO * r_neck:
        return neck_v, bottom_v, -axis_u, True
    return bottom_v, neck_v, axis_u, False


def clamp_upper_hemisphere(bottom, neck, axis, gravity) -> tuple:
    """
    袋底→袋口只许在上半球：相对逆重力夹角 ≤ 90°（含水平）.

    只翻转符号，不把斜袋长轴改成竖的。True = 发生了对调.
    """
    axis_u = _unit(axis)
    gravity_u = _unit(gravity)
    if (axis_u is None or gravity_u is None
            or bottom is None or neck is None):
        return bottom, neck, axis, False
    bottom_v = np.asarray(bottom, dtype=np.float64).reshape(3).copy()
    neck_v = np.asarray(neck, dtype=np.float64).reshape(3).copy()
    if float(np.dot(neck_v - bottom_v, axis_u)) < 0.0:
        axis_u = -axis_u
    if float(axis_u @ gravity_u) > 0.0:
        return neck_v, bottom_v, -axis_u, True
    return bottom_v, neck_v, axis_u, False


def estimate_bag_landmarks(
        points: np.ndarray, gravity=None,
        mask_area_ratio: float = 0.05, edge_touch: bool = False,
        neighbor_gap_m: float = 1.0, valid_depth_ratio: float = 1.0,
        bbox_aspect: float = 1.0) -> BagLandmarks:
    """
    沿袋长轴做半径剖面：窄头为扎口，宽头为袋底，轴从袋底指向袋口.

    两端差不多粗时才用逆重力当颈。窄头若会把轴翻到朝下则忽略：袋底→袋口
    只许从下往上（左右最多水平）。主轴来自袋底→袋口。球拟合只填先验包络.
    """
    result = BagLandmarks()
    result.occlusion_class = classify_occlusion(
        mask_area_ratio, edge_touch, neighbor_gap_m, valid_depth_ratio,
        bbox_aspect)
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] < 30 or pts.shape[1] != 3:
        result.flags.append('too_few_points')
        return result
    finite = pts[np.isfinite(pts).all(axis=1)]
    if finite.shape[0] < 30:
        result.flags.append('too_few_finite_points')
        return result
    gravity_u = _unit(gravity)
    if gravity_u is None:
        gravity_u = np.array([0.0, 0.0, -1.0])
        result.flags.append('gravity_defaulted')
    long_axis, from_pca = _long_axis(finite, gravity_u)
    if from_pca:
        result.flags.append('axis_from_pca')
    else:
        result.flags.append('axis_from_gravity_prior')
    profile = _bin_radii(finite, long_axis)
    if profile[0] is None:
        result.flags.append('bag_axis_too_short')
        return result
    proj, edges, radii_a = profile
    valid = np.isfinite(radii_a)
    if int(valid.sum()) < 4:
        result.flags.append('radius_profile_failed')
        return result
    r_low = _end_radius(radii_a, False)
    r_high = _end_radius(radii_a, True)
    taper_clear = (
        np.isfinite(r_low) and np.isfinite(r_high)
        and max(r_low, r_high) > 1e-6
        and min(r_low, r_high) < TAPER_RATIO * max(r_low, r_high))
    if taper_clear:
        result.flags.append('taper_neck')
        if r_low < r_high:
            result.flags.append('taper_lower_hemisphere_ignored')
    else:
        result.flags.append('gravity_neck_fallback')
    neck_i = _end_bin_index(valid, True)
    bottom_i = _end_bin_index(valid, False)
    if neck_i is None or bottom_i is None or neck_i == bottom_i:
        result.flags.append('bag_ends_degenerate')
        return result
    neck_band = _axial_band(finite, proj, edges, int(neck_i))
    bottom_band = _axial_band(finite, proj, edges, int(bottom_i))
    if neck_band.shape[0] < 4 or bottom_band.shape[0] < 4:
        result.flags.append('end_band_too_sparse')
        return result
    r_neck_band = _band_radius(neck_band, long_axis)
    r_bottom_band = _band_radius(bottom_band, long_axis)
    if (np.isfinite(r_neck_band) and np.isfinite(r_bottom_band)
            and r_bottom_band + 1e-6 < TAPER_RATIO * r_neck_band):
        proposed = (np.mean(bottom_band, axis=0)
                    - np.mean(neck_band, axis=0))
        if float(proposed @ gravity_u) <= 0.0:
            neck_band, bottom_band = bottom_band, neck_band
            result.flags.append('taper_polarity_swapped')
        else:
            result.flags.append('taper_lower_hemisphere_ignored')
    axis = _unit(np.mean(neck_band, axis=0) - np.mean(bottom_band, axis=0))
    if axis is None:
        axis = long_axis
        result.flags.append('axis_from_profile_sign')
    nl, nr = _section_extrema(finite, axis, neck_band)
    bl, br = _section_extrema(finite, axis, bottom_band)
    if nl is not None:
        result.neck_left = Landmark3D(nl, 0.6, True)
    if nr is not None:
        result.neck_right = Landmark3D(nr, 0.6, True)
    if bl is not None:
        result.bottom_left = Landmark3D(bl, 0.6, True)
    if br is not None:
        result.bottom_right = Landmark3D(br, 0.6, True)
    result.neck_center = _mid(result.neck_left, result.neck_right)
    result.bottom_center = _mid(result.bottom_left, result.bottom_right)
    if result.neck_center is None:
        result.neck_center = np.mean(neck_band, axis=0)
        result.flags.append('neck_from_band')
    if result.bottom_center is None:
        result.bottom_center = np.mean(bottom_band, axis=0)
        result.flags.append('bottom_from_band')
    result.bag_axis = _unit(result.neck_center - result.bottom_center)
    bottom, neck, axis, swapped = enforce_wide_bottom(
        result.bottom_center, result.neck_center, result.bag_axis, finite)
    if swapped:
        result.neck_left, result.bottom_left = (
            result.bottom_left, result.neck_left)
        result.neck_right, result.bottom_right = (
            result.bottom_right, result.neck_right)
        if 'taper_polarity_swapped' not in result.flags:
            result.flags.append('taper_polarity_swapped')
    bottom, neck, axis, up_flipped = clamp_upper_hemisphere(
        bottom, neck, axis, gravity_u)
    if up_flipped:
        result.neck_left, result.bottom_left = (
            result.bottom_left, result.neck_left)
        result.neck_right, result.bottom_right = (
            result.bottom_right, result.neck_right)
        result.flags.append('polarity_upper_hemisphere')
    result.bottom_center = bottom
    result.neck_center = neck
    result.bag_axis = axis
    radial = axis_radial_distance(finite, result.bag_axis, result.bottom_center)
    result.d95_m = float(2.0 * np.percentile(radial, 95))
    sphere = fit_sphere_robust(finite)
    if sphere is not None:
        result.fruit_prior_center = np.asarray(sphere['center'], dtype=np.float64)
        result.fruit_prior_radius_m = float(sphere['radius'])
        result.flags.append('fruit_prior_auxiliary')
    result.sigma_position_m = float(np.clip(
        0.008 + 0.04 * (1.0 - valid_depth_ratio), 0.006, 0.05))
    result.sigma_axis_deg = float(np.clip(
        3.0 + 12.0 * (1.0 - valid_depth_ratio), 2.0, 20.0))
    if result.occlusion_class != OCCLUSION_CLEAR:
        result.flags.append(result.occlusion_class)
    return result
