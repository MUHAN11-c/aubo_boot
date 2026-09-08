"""多视角袋模型融合：关键点定方向，体积定侧向与剪切站（纯核）."""
from __future__ import annotations

from typing import Iterable, Optional

import numpy as np

from peach_perception.common.bag_landmarks import (
    BagLandmarks,
    clamp_upper_hemisphere,
    enforce_wide_bottom,
    OCCLUSION_BRANCH,
    OCCLUSION_DAMAGED,
    OCCLUSION_NEIGHBOR,
)
from peach_perception.common.fitting import (
    angle_between_deg,
    axis_radial_distance,
    unit_vector as _unit,
)
from peach_perception.common.tool_budget import (
    evaluate_sleeve_cut,
    ToolBudgetParams,
)


def _huber_mean(points: np.ndarray, k: float = 0.02) -> Optional[np.ndarray]:
    """三维点 Huber 加权均值."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return None
    center = np.median(pts, axis=0)
    for _ in range(8):
        delta = pts - center
        dist = np.linalg.norm(delta, axis=1)
        weights = np.ones(pts.shape[0])
        far = dist > k
        weights[far] = k / np.maximum(dist[far], 1e-9)
        denom = float(weights.sum())
        if denom < 1e-9:
            break
        center = (weights[:, None] * pts).sum(axis=0) / denom
    return center


def _signed_angle_deg(first, second) -> float:
    """有符号夹角 [deg]；反向为 180°，不取绝对值；退化输入按 180°."""
    angle = angle_between_deg(first, second)
    return 180.0 if angle is None else angle


def median_absolute_deviation_m(points: np.ndarray, center: np.ndarray) -> float:
    """点到中心距离的 MAD→σ [m]."""
    delta = np.linalg.norm(
        np.asarray(points, dtype=np.float64) - center, axis=1)
    if delta.size == 0:
        return 0.0
    return float(1.4826 * np.median(delta))


def _finite_cloud(points) -> Optional[np.ndarray]:
    """有限三维点；不足则 None."""
    if points is None:
        return None
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] < 8 or pts.shape[1] != 3:
        return None
    finite = pts[np.isfinite(pts).all(axis=1)]
    if finite.shape[0] < 8:
        return None
    return finite


def slice_centroid(points, axis, origin, t_m: float,
                   half_width_m: float = 0.02) -> Optional[np.ndarray]:
    """沿轴在 t 处取截面质心；点数不足则 None."""
    axis_u = _unit(axis)
    cloud = _finite_cloud(points)
    origin_v = np.asarray(origin, dtype=np.float64).reshape(3)
    if axis_u is None or cloud is None:
        return None
    axial = (cloud - origin_v) @ axis_u
    selected = cloud[np.abs(axial - float(t_m)) <= float(half_width_m)]
    if selected.shape[0] < 8:
        return None
    return selected.mean(axis=0)


def snap_lateral(point, axis, centroid) -> np.ndarray:
    """把点沿垂直于轴的方向贴到截面质心，轴向坐标不变."""
    axis_u = _unit(axis)
    src = np.asarray(point, dtype=np.float64).reshape(3)
    if axis_u is None or centroid is None:
        return src
    delta = np.asarray(centroid, dtype=np.float64).reshape(3) - src
    return src + delta - float(delta @ axis_u) * axis_u


def envelope_axis_from_cloud(
        points, keypoint_axis, min_aspect: float = 1.0,
        min_length_m: float = 0.05, min_slices: int = 4) -> dict:
    """
    沿关键点轴切 TSDF/点云截面，用截面质心拟合包络主方向.

    接触轴仍由关键点融合给出。本函数只回答体积是否沿该轴延伸。
    轴向跨度小于直径或切片不足时 conditioned=False，不得用 12° 否决.
    """
    empty = {
        'conditioned': False, 'axis': None, 'span_m': 0.0, 'd95_m': 0.0,
        'reason': 'envelope_cloud_insufficient'}
    axis = _unit(keypoint_axis)
    finite = _finite_cloud(points)
    if axis is None or finite is None or finite.shape[0] < 30:
        return empty
    origin = np.median(finite, axis=0)
    axial = (finite - origin) @ axis
    t_lo, t_hi = np.percentile(axial, [5.0, 95.0])
    span = float(t_hi - t_lo)
    d95 = float(2.0 * np.percentile(
        axis_radial_distance(finite, axis, origin), 95))
    if span < min_length_m or (d95 > 1e-6 and span / d95 < min_aspect):
        return {
            'conditioned': False, 'axis': None, 'span_m': span, 'd95_m': d95,
            'reason': 'envelope_axis_ill_conditioned'}
    bins = 8
    edges = np.linspace(t_lo, t_hi, bins + 1)
    centroids = []
    for index in range(bins):
        selected = finite[(axial >= edges[index]) & (axial < edges[index + 1])]
        if selected.shape[0] < 8:
            continue
        centroids.append(selected.mean(axis=0))
    if len(centroids) < min_slices:
        return {
            'conditioned': False, 'axis': None, 'span_m': span, 'd95_m': d95,
            'reason': 'envelope_too_few_slices'}
    centered = np.stack(centroids) - np.mean(centroids, axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    envelope = _unit(vt[0])
    if envelope is None:
        return {
            'conditioned': False, 'axis': None, 'span_m': span, 'd95_m': d95,
            'reason': 'envelope_axis_undefined'}
    if float(envelope @ axis) < 0.0:
        envelope = -envelope
    return {
        'conditioned': True, 'axis': envelope, 'span_m': span, 'd95_m': d95,
        'reason': 'ok'}


def _corridor_clear(
        points, axis, origin, t0: float, t1: float,
        d_inner: float, wall_clearance: float) -> bool:
    """沿套入区间切片，袋径不得超过工具内净空."""
    axis_u = _unit(axis)
    cloud = _finite_cloud(points)
    origin_v = np.asarray(origin, dtype=np.float64).reshape(3)
    if axis_u is None or cloud is None:
        return False
    lo = min(float(t0), float(t1))
    hi = max(float(t0), float(t1))
    if hi - lo < 0.02:
        hi = lo + 0.02
    axial = (cloud - origin_v) @ axis_u
    limit = 0.5 * float(d_inner) - float(wall_clearance)
    bins = 6
    edges = np.linspace(lo, hi, bins + 1)
    seen = 0
    for index in range(bins):
        selected = cloud[(axial >= edges[index]) & (axial < edges[index + 1])]
        if selected.shape[0] < 8:
            continue
        seen += 1
        mid = selected.mean(axis=0)
        if float(np.percentile(
                axis_radial_distance(selected, axis_u, mid), 95)) > limit:
            return False
    return seen >= 3


def _cut_station(
        bottom, neck, axis, length_m: float,
        fruit_center, fruit_r: float,
        params: ToolBudgetParams,
        neck_margin_m: float = 0.0) -> dict:
    """
    轴上剪切参考：袋口（分割贴检测框极限），不取果包络与袋颈中点.

    果距不足时刀仍放在口，只把 safe_band 置假（拦套入，不挪刀）。
    看不见果先验时给出袋口参考，但不宣称果距。
    """
    del neck_margin_m
    bottom_v = np.asarray(bottom, dtype=np.float64).reshape(3)
    axis_u = _unit(axis)
    t_neck = float(length_m)
    fruit_hi = None
    if axis_u is None:
        return {
            'cut': np.asarray(neck, dtype=np.float64).reshape(3),
            't_cut_m': t_neck,
            'cut_to_fruit_m': 0.0,
            'safe_band': False,
        }
    t_cut = t_neck
    t_min = 0.0
    if fruit_center is not None and float(fruit_r) > 1e-6:
        t_fruit = float(
            (np.asarray(fruit_center, dtype=np.float64).reshape(3)
             - bottom_v) @ axis_u)
        fruit_hi = t_fruit + float(fruit_r)
        t_min = fruit_hi + float(params.fruit_safety_clearance)
    cut_to_fruit = 0.0
    if fruit_hi is not None:
        cut_to_fruit = float(t_cut - fruit_hi)
    return {
        'cut': bottom_v + t_cut * axis_u,
        't_cut_m': float(t_cut),
        'cut_to_fruit_m': cut_to_fruit,
        'safe_band': bool(fruit_hi is not None and t_cut >= t_min),
    }


def _majority_sense_views(items: list) -> Optional[list]:
    """口/底朝向多数一致的视角；对打且无多数则 None（否决不平均）."""
    if len(items) <= 1:
        return list(items)
    axes = []
    for item in items:
        axis = _unit(item.bag_axis)
        if axis is None:
            axis = _unit(item.neck_center - item.bottom_center)
        axes.append(axis)
    usable = [(i, ax) for i, ax in enumerate(axes) if ax is not None]
    if not usable:
        return None
    best_i = usable[0][0]
    best_count = -1
    for i, ax in usable:
        count = sum(1 for _, other in usable if float(np.dot(ax, other)) > 0.0)
        if count > best_count:
            best_count = count
            best_i = i
    ref = axes[best_i]
    cluster = [
        items[i] for i, ax in enumerate(axes)
        if ax is not None and float(np.dot(ax, ref)) > 0.0]
    if len(cluster) * 2 <= len(items):
        return None
    return cluster


def fuse_bag_views(
        views: Iterable[BagLandmarks],
        params: ToolBudgetParams | None = None,
        cloud_xyz=None,
        detection_axis=None,
        entry_standoff_m: float = 0.0,
        pregrasp_standoff_m: float = 0.0) -> dict:
    """
    融合多视角袋关键点.

    方向：袋底→袋颈 Huber，且只许上半球（左右最多水平）。口底对打的视角否决不平均。
    定位：体积截面质心只改侧向。
    剪切站：袋口 / 分割贴检测框极限；果距不足只否决 allowed，不挪刀。
    包络长径比不足则跳过 12° 否决。检测轴夹角只诊断，不进接触预算。
    后撤量由调用方传入（节点从 ROS 参数读，不在本函数写死米数）。
    """
    cfg = params or ToolBudgetParams()
    items = [item for item in views if item.neck_center is not None
             and item.bottom_center is not None]
    if not items:
        return {'ok': False, 'reason': 'no_landmark_views', 'allowed': False}
    aligned = _majority_sense_views(items)
    if not aligned:
        return {'ok': False, 'reason': 'landmark_axis_conflict', 'allowed': False}
    n_dropped = len(items) - len(aligned)
    items = aligned
    bottoms = np.stack([item.bottom_center for item in items])
    necks = np.stack([item.neck_center for item in items])
    axes = []
    for item in items:
        axis = _unit(item.bag_axis)
        if axis is not None:
            axes.append(axis)
    bottom = _huber_mean(bottoms)
    neck = _huber_mean(necks)
    axis = _unit(neck - bottom)
    if axis is None and axes:
        axis = _unit(np.mean(np.stack(axes), axis=0))
    if bottom is None or neck is None or axis is None:
        return {'ok': False, 'reason': 'fusion_failed', 'allowed': False}
    d95_values = [item.d95_m for item in items if item.d95_m > 0]
    # 全部视角 d95 缺失（0/负）时回退 0：下游预算把 0 当「无径向散布数据」
    # 处理，不得让 np.median([]) 的 NaN 流进许可与 diagnostics JSON。
    d95 = float(np.median(d95_values)) if d95_values else 0.0
    length = float(np.dot(neck - bottom, axis))
    if length < 0.0:
        axis = -axis
        length = -length
        bottom, neck = neck, bottom
    mid = slice_centroid(cloud_xyz, axis, bottom, 0.5 * max(length, 0.02))
    if mid is None:
        mid = slice_centroid(cloud_xyz, axis, bottom, 0.0)
    if mid is not None:
        bottom = snap_lateral(bottom, axis, mid)
        neck = snap_lateral(neck, axis, mid)
        length = float(np.dot(neck - bottom, axis))
        if length < 0.0:
            axis = -axis
            length = -length
            bottom, neck = neck, bottom
    bottom, neck, axis, width_flipped = enforce_wide_bottom(
        bottom, neck, axis, cloud_xyz)
    bottom, neck, axis, up_flipped = clamp_upper_hemisphere(
        bottom, neck, axis, np.array([0.0, 0.0, -1.0], dtype=np.float64))
    length = float(np.dot(neck - bottom, axis))
    view_sigma = float(np.median([item.sigma_position_m for item in items]))
    sig_p = max(
        view_sigma,
        median_absolute_deviation_m(bottoms, bottom),
        median_absolute_deviation_m(necks, neck), 0.006)
    sig_a = float(np.median([item.sigma_axis_deg for item in items]))
    bottom_err = np.linalg.norm(bottoms - bottom, axis=1)
    neck_err = np.linalg.norm(necks - neck, axis=1)
    combined = np.concatenate([bottom_err, neck_err])
    rmse = float(np.sqrt(np.mean(combined * combined)))
    inlier_ratio = float(np.mean(combined < 0.02))
    envelope = envelope_axis_from_cloud(cloud_xyz, axis)
    axis_conflict_deg = 0.0
    if envelope.get('conditioned'):
        axis_conflict_deg = _signed_angle_deg(axis, envelope.get('axis'))
    detection_conflict_deg = 0.0
    if _unit(detection_axis) is not None:
        detection_conflict_deg = _signed_angle_deg(axis, detection_axis)
    axis_error_deg = max(sig_a, axis_conflict_deg)
    env_d95 = float(envelope.get('d95_m') or 0.0)
    if env_d95 > 1e-6:
        d95 = max(d95, env_d95) if d95 > 1e-6 else env_d95
    fruit_centers = [
        item.fruit_prior_center for item in items
        if item.fruit_prior_center is not None]
    fruit_r_vals = [
        item.fruit_prior_radius_m for item in items
        if item.fruit_prior_radius_m > 0]
    fruit_center = (
        _huber_mean(np.stack(fruit_centers)) if fruit_centers else None)
    fruit_r = float(np.median(fruit_r_vals)) if fruit_r_vals else 0.0
    cut = _cut_station(
        bottom, neck, axis, length, fruit_center, fruit_r, cfg)
    standoff = float(entry_standoff_m)
    entry = bottom - standoff * axis
    pregrasp = entry - float(pregrasp_standoff_m) * axis
    cut_travel = float(np.dot(cut['cut'] - entry, axis))
    if d95 <= 1e-6:
        # 无任何径向尺度证据（逐视角 d95 与体积包络全缺）：d_bag95=0 会拿到
        # 最宽松的径向预算（袋当零宽），保守拒绝而不是放行。12=几何超限族。
        budget = {
            'allowed': False, 'reason': 'bag_d95_missing', 'failure_code': 12}
    else:
        budget = evaluate_sleeve_cut(
            d_bag95=d95, length_m=max(length, 0.05),
            center_lateral95=sig_p, axis_error_deg=axis_error_deg,
            neck_position95=sig_p,
            cut_to_fruit_m=float(cut['cut_to_fruit_m']),
            params=cfg)
    occlusion = items[-1].occlusion_class
    flags = []
    if n_dropped:
        flags.append('landmark_views_vetoed')
    if width_flipped:
        flags.append('taper_polarity_swapped')
    if up_flipped:
        flags.append('polarity_upper_hemisphere')
    if not envelope.get('conditioned'):
        flags.append(str(
            envelope.get('reason') or 'envelope_axis_ill_conditioned'))
    if envelope.get('conditioned') and axis_conflict_deg > 12.0:
        flags.append('keypoint_cloud_axis_conflict')
        budget['allowed'] = False
        budget['reason'] = 'keypoint_cloud_axis_conflict'
        budget['failure_code'] = 3
    if not cut['safe_band']:
        flags.append('cut_band_unavailable')
        if budget.get('allowed'):
            budget['allowed'] = False
            budget['reason'] = 'cut_plane_fruit_clearance'
            budget['failure_code'] = 16
    if occlusion in (
            OCCLUSION_BRANCH, OCCLUSION_NEIGHBOR, OCCLUSION_DAMAGED):
        flags.append(occlusion)
        budget['allowed'] = False
        budget['reason'] = 'occlusion_' + occlusion
        budget['failure_code'] = 3
    corridor = _corridor_clear(
        cloud_xyz, axis, bottom, 0.0, cut['t_cut_m'],
        cfg.d_inner, cfg.wall_clearance)
    model = {
        'ok': True,
        'bottom': bottom,
        'neck': neck,
        'axis': axis,
        'entry': entry,
        'pregrasp': pregrasp,
        'cut_plane_point': cut['cut'],
        'cut_pose': cut['cut'],
        'cut_normal': axis,
        'cut_to_fruit_m': float(cut['cut_to_fruit_m']),
        'cut_travel_m': cut_travel,
        'd95_m': d95,
        'length_m': length,
        'fruit_prior_radius_m': fruit_r,
        'fruit_prior_auxiliary': True,
        'sigma_position_m': sig_p,
        'sigma_axis_deg': sig_a,
        'rmse': rmse,
        'inlier_ratio': inlier_ratio,
        'axis_conflict_deg': axis_conflict_deg,
        'detection_conflict_deg': detection_conflict_deg,
        'envelope_conditioned': bool(envelope.get('conditioned')),
        'envelope_span_m': float(envelope.get('span_m') or 0.0),
        'envelope_d95_m': env_d95,
        'envelope_reason': str(envelope.get('reason') or ''),
        'occlusion_class': occlusion,
        'view_count': len(items),
        'flags': flags,
        'budget': budget,
        'allowed': bool(budget.get('allowed')),
        'reason': budget.get('reason', ''),
        'radial_margin_m': float(budget.get('radial_margin_m', 0.0)),
        'axial_margin_m': float(budget.get('axial_margin_m', 0.0)),
        'corridor_clear': bool(corridor),
    }
    return model
