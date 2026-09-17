"""观察视点候选生成与评分（C++ view_planner.cpp 逐字移植，清洁重写轮 3c-2b）。

spherical_adaptive：从当前相机沿直线截 max_camera_step_m，评分以行程最短
为主；禁止绕球面、不对侧兜圈。常数与评分权重 = C++ 原值（丁组：不调参）；
移植锚 a955cea→466ff5c。纯核零 ROS。

调用方（supervisor 观察循环，3c-2c 接线）：选 top 候选 → MoveTo(KIND_POSE)
→ 等新机位；候选分数仅排序用，不作质量结论。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

Vec3 = Sequence[float]


def _v_sub(a: Vec3, b: Vec3) -> list:
    return [a[i] - b[i] for i in range(3)]


def _v_add(a: Vec3, b: Vec3) -> list:
    return [a[i] + b[i] for i in range(3)]


def _v_scale(a: Vec3, s: float) -> list:
    return [x * s for x in a]


def _v_dot(a: Vec3, b: Vec3) -> float:
    return sum(a[i] * b[i] for i in range(3))


def _v_cross(a: Vec3, b: Vec3) -> list:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]]


def _v_norm(a: Vec3) -> float:
    return math.sqrt(sum(x * x for x in a))


def _safe_unit(value: Vec3, fallback: Vec3) -> list:
    """零向量/非有限退 fallback，防归一化 NaN 污染候选方向。"""
    if not all(math.isfinite(x) for x in value) or _v_norm(value) < 1.0e-9:
        return list(fallback)
    n = _v_norm(value)
    return [x / n for x in value]


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _angle_deg(first: Vec3, second: Vec3) -> float:
    """夹角（度）；退化按 +x 兜底，结果有限且 0–180°。"""
    unit_x = (1.0, 0.0, 0.0)
    a = _safe_unit(first, unit_x)
    b = _safe_unit(second, unit_x)
    c = _clamp(_v_dot(a, b), -1.0, 1.0)
    return math.degrees(math.acos(c))


@dataclass(frozen=True)
class ViewPlannerConfig:
    """默认值与 config/peach_arm.yaml scan.* 同值（原值）。"""

    # 观察半径须高于 PS800-E1 深度额定下限 0.4 m（09-17 由 0.40/0.32 上调）
    observation_radius_m: float = 0.45
    minimum_radius_m: float = 0.42
    azimuth_step_deg: float = 12.0
    azimuth_limit_deg: float = 16.0
    elevation_step_deg: float = 8.0
    elevation_limit_deg: float = 0.0
    preferred_baseline_deg: float = 12.0
    radial_step_m: float = 0.015
    candidate_layers: int = 1
    views_to_minimum_radius: int = 5
    max_camera_step_m: float = 0.15
    workspace_max_reach_m: float = 0.78
    min_camera_height_m: float = 0.06
    protected_zones: tuple = ()


@dataclass
class ViewContext:
    """generate 的上下文（信号来自当前观测缓存）。"""

    target: Vec3
    current_camera_position: Vec3
    observed_directions: list = field(default_factory=list)
    # 画面信号（bbox/邻目标/前景占比）
    image_width: int = 640
    image_height: int = 480
    bbox_valid: bool = False
    bbox_x: int = 0
    bbox_y: int = 0
    bbox_w: int = 0
    bbox_h: int = 0
    neighbor_centers: list = field(default_factory=list)
    foreground_ratio: float = -1.0


@dataclass
class ViewCandidate:
    """候选视点（排序用快照；camera_pose 只含位置，姿态装配在调用方）。"""

    direction_target_to_camera: list = field(default_factory=list)
    radius_m: float = 0.0
    azimuth_deg: float = 0.0
    elevation_deg: float = 0.0
    nearest_baseline_deg: float = 0.0
    motion_angle_deg: float = 0.0
    travel_m: float = 0.0
    score: float = 0.0
    position: list = field(default_factory=list)
    label: str = ''


def _visibility_desired(
        context: ViewContext, target: Vec3, side: Vec3, up: Vec3) -> list:
    """像面期望分量：框心归中 + 贴边裁切回推 + 邻目标避让（前景足够时）。"""
    desired = [0.0, 0.0]
    width = max(1, context.image_width)
    height = max(1, context.image_height)
    if context.bbox_valid and context.bbox_w > 0 and context.bbox_h > 0:
        cx = context.bbox_x + 0.5 * context.bbox_w
        cy = context.bbox_y + 0.5 * context.bbox_h
        desired[0] += (cx - 0.5 * width) / (0.5 * width)
        desired[1] += (cy - 0.5 * height) / (0.5 * height)
        margin_px = 8.0
        clip_left = max(0.0, margin_px - context.bbox_x)
        clip_right = max(0.0, context.bbox_x + context.bbox_w -
                         (width - margin_px))
        clip_top = max(0.0, margin_px - context.bbox_y)
        clip_bottom = max(0.0, context.bbox_y + context.bbox_h -
                          (height - margin_px))
        desired[0] += 1.5 * (clip_right - clip_left) / width
        desired[1] += 1.5 * (clip_bottom - clip_top) / height
    if not (0.0 <= context.foreground_ratio < 0.40):
        for neighbor in context.neighbor_centers:
            rel = _v_sub(neighbor, target)
            desired[0] += 0.35 * _v_dot(rel, side)
            desired[1] += 0.35 * _v_dot(rel, up)
    if math.hypot(*desired) < 1.0e-6:
        desired[0] = 1.0
    n = math.hypot(*desired)
    return [desired[0] / n, desired[1] / n]


def _bbox_too_small(context: ViewContext) -> bool:
    if not context.bbox_valid or context.bbox_w <= 0 or context.bbox_h <= 0:
        return False
    area = context.bbox_w * context.bbox_h
    image = max(1, context.image_width) * max(1, context.image_height)
    return area / image < 0.04


def _mask_fill_low(context: ViewContext) -> bool:
    return 0.0 <= context.foreground_ratio < 0.40


def _clamp_to_step(current: Vec3, goal: Vec3, max_step: float) -> list:
    delta = _v_sub(goal, current)
    n = _v_norm(delta)
    if n <= 1.0e-9 or max_step <= 0.0 or n <= max_step:
        return list(goal)
    return _v_add(current, _scale_helper(delta, max_step / n))


def _scale_helper(a: Vec3, s: float) -> list:
    return _v_scale(a, s)


def _project_look_ray_into_reach(
        target: Vec3, camera: Vec3, min_radius: float, reach: float) -> list:
    """视线（目标→相机）与可达球最近交点：沿射线收进，不绕行。"""
    offset = _v_sub(camera, target)
    radius = _v_norm(offset)
    if radius < 1.0e-9 or reach <= 0.0 or _v_norm(camera) <= reach:
        return list(camera)
    direction = _scale_helper(offset, 1.0 / radius)
    b = 2.0 * _v_dot(target, direction)
    c = _v_dot(target, target) - reach * reach
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return _v_add(
            target, _scale_helper(direction, max(min_radius, 0.5 * radius)))
    root = math.sqrt(disc)
    t_lo = 0.5 * (-b - root)
    t_hi = 0.5 * (-b + root)
    t = _clamp(radius, t_lo, t_hi)
    t = max(t, min_radius)
    return _v_add(target, _scale_helper(direction, t))


def _protected_zone_hit(goal: Vec3, zones) -> bool:
    """轴对齐盒保护区命中（zones: [minx,miny,minz,maxx,maxy,maxz]×N）。"""
    for zone in zones or ():
        if len(zone) < 6:
            continue
        if (zone[0] <= goal[0] <= zone[3] and zone[1] <= goal[1] <= zone[4]
                and zone[2] <= goal[2] <= zone[5]):
            return True
    return False


def look_at_optical(
        camera_position: Vec3, target: Vec3,
        world_up: Vec3 = (0.0, 0.0, 1.0)) -> list:
    """光学系朝向（返回三列基 [x,y,z] 行列表；调用方转四元数）。"""
    unit_z = (0.0, 0.0, 1.0)
    optical_z = _safe_unit(_v_sub(target, camera_position), unit_z)
    down = _v_scale(_safe_unit(world_up, unit_z), -1.0)
    if abs(_v_dot(down, optical_z)) > 0.97:
        down = (0.0, 1.0, 0.0)
    optical_x = _safe_unit(_v_cross(down, optical_z), (1.0, 0.0, 0.0))
    optical_y = _safe_unit(
        _v_cross(optical_z, optical_x), (0.0, 1.0, 0.0))
    return [list(optical_x), list(optical_y), list(optical_z)]


def basis_to_quat(basis: Sequence[Sequence[float]]) -> tuple:
    """旋转矩阵（三列基 [x,y,z]）→ 四元数 (x, y, z, w)。"""
    ex, ey, ez = basis[0], basis[1], basis[2]
    m00, m01, m02 = ex[0], ey[0], ez[0]
    m10, m11, m12 = ex[1], ey[1], ez[1]
    m20, m21, m22 = ex[2], ey[2], ez[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    return (qx / n, qy / n, qz / n, qw / n)


def generate(context: ViewContext,
             config: Optional[ViewPlannerConfig] = None) -> list:
    """候选生成（C++ generate 逐句移植；评分权重原值）。"""
    cfg = config or ViewPlannerConfig()
    target = context.target
    current = context.current_camera_position
    unit_x = (1.0, 0.0, 0.0)
    front = _safe_unit(_v_sub(current, target), unit_x)
    side = _v_cross((0.0, 0.0, 1.0), front)
    if _v_norm(side) < 1.0e-6:
        side = (0.0, 1.0, 0.0)
    side = _safe_unit(side, (0.0, 1.0, 0.0))
    up = _safe_unit(_v_cross(front, side), (0.0, 0.0, 1.0))
    observed = list(context.observed_directions) or [front]

    current_radius = _v_norm(_v_sub(current, target))
    radius0 = _clamp(current_radius, cfg.minimum_radius_m, 2.0)
    want_closer = (_bbox_too_small(context) or _mask_fill_low(context)) and \
        radius0 > cfg.minimum_radius_m + 0.5 * cfg.radial_step_m
    outside_reach = _v_norm(current) > cfg.workspace_max_reach_m
    desired = _visibility_desired(context, target, side, up)
    max_step = max(0.01, cfg.max_camera_step_m)

    result: list = []

    def consider(goal: Vec3, azimuth_deg: float, elevation_deg: float,
                 azimuth_index: int, elevation_index: int, layer: int,
                 tag: str) -> None:
        goal = _clamp_to_step(current, goal, max_step)
        if goal[2] < cfg.min_camera_height_m:
            return
        if _protected_zone_hit(goal, cfg.protected_zones):
            return
        travel = _v_norm(_v_sub(goal, current))
        direction = _safe_unit(_v_sub(goal, target), front)
        motion = _angle_deg(direction, front)
        if travel < 0.012 and motion < 3.0 and not outside_reach:
            return
        nearest = min(_angle_deg(direction, prev) for prev in observed)
        move_side = _v_dot(direction, side)
        move_up = _v_dot(direction, up)
        move = [move_side, move_up]
        move_n = math.hypot(*move)
        if move_n > 1.0e-9:
            move = [move[0] / move_n, move[1] / move_n]
        align = 0.5 * (move[0] * desired[0] + move[1] * desired[1] + 1.0)
        baseline_error = (nearest - cfg.preferred_baseline_deg) / max(
            1.0, cfg.preferred_baseline_deg * 0.7)
        overlap_score = math.exp(-0.5 * baseline_error * baseline_error)
        near_score = 1.0 - _clamp(travel / max_step, 0.0, 1.0)
        radius = _v_norm(_v_sub(goal, target))
        standoff_score = 1.0 - _clamp(
            (radius - cfg.minimum_radius_m) /
            max(0.05, cfg.observation_radius_m), 0.0, 1.0)
        align_w = 0.22 if _mask_fill_low(context) else 0.18
        overlap_w = 0.08 if _mask_fill_low(context) else 0.12
        standoff_w = 0.20 if (want_closer or outside_reach) else 0.10
        near_w = 1.0 - align_w - overlap_w - standoff_w
        candidate = ViewCandidate(
            direction_target_to_camera=list(direction),
            radius_m=radius,
            azimuth_deg=azimuth_deg,
            elevation_deg=elevation_deg,
            nearest_baseline_deg=nearest,
            motion_angle_deg=motion,
            travel_m=travel,
            score=(near_w * near_score + standoff_w * standoff_score +
                   align_w * align + overlap_w * overlap_score),
            position=list(goal),
            label=f'{tag}_a{azimuth_index}_e{elevation_index}_r{layer}')
        result.append(candidate)

    if (outside_reach or
            _v_norm(current) > 0.92 * cfg.workspace_max_reach_m):
        consider(
            _project_look_ray_into_reach(
                target, current, cfg.minimum_radius_m,
                cfg.workspace_max_reach_m),
            0.0, 0.0, 0, 0, 0, 'see_in')

    azimuth_steps = 1
    elevation_steps = 1 if cfg.elevation_limit_deg > 1.0e-6 else 0
    layer_count = 2 if want_closer else 1
    for layer in range(layer_count):
        radius = max(
            cfg.minimum_radius_m, radius0 - layer * cfg.radial_step_m)
        for azimuth_index in range(-azimuth_steps, azimuth_steps + 1):
            for elevation_index in range(-elevation_steps,
                                         elevation_steps + 1):
                if azimuth_index == 0 and elevation_index == 0:
                    continue
                azimuth_deg = azimuth_index * cfg.azimuth_step_deg
                elevation_deg = elevation_index * cfg.elevation_step_deg
                azimuth = math.radians(azimuth_deg)
                elevation = math.radians(elevation_deg)
                direction = _v_add(
                    _v_add(
                        _scale_helper(
                            front, math.cos(elevation) * math.cos(azimuth)),
                        _scale_helper(
                            side, math.cos(elevation) * math.sin(azimuth))),
                    _scale_helper(up, math.sin(elevation)))
                direction = _safe_unit(direction, front)
                consider(
                    _v_add(target, _scale_helper(direction, radius)),
                    azimuth_deg, elevation_deg,
                    azimuth_index, elevation_index, layer, 'see')
    if not result:
        fb = _v_add(
            _scale_helper(
                front, math.cos(math.radians(cfg.azimuth_step_deg))),
            _scale_helper(
                side, math.sin(math.radians(cfg.azimuth_step_deg))))
        consider(
            _v_add(target, _scale_helper(_safe_unit(fb, front), radius0)),
            cfg.azimuth_step_deg, 0.0, 1, 0, 0, 'see_fb')

    result.sort(key=lambda c: (-c.score, c.travel_m))
    return result
