"""停走节拍视角轨迹（纯核：零 Blender / 零 ROS，pytest 可单测）.

复刻产线语义（fast 档）：
- 沿作业道停靠位序列（alley stops）——机器人停、拍、再停；
- 每目标补视取两候选行程短者：A) 沿相机→目标直线 0.15 m 截距、
  B) 绕袋轴 ±30° 摆角。常数与 peach_harvester
  `cycle_core/view_policy.py::supplemental_viewpoint` 原值一致；
  test_viewpoints.py 用 sys.path 双向对拍两实现防漂移；
- 每目标总视数封顶 3（min_views=2 口径）：1 停靠视 + 至多 2 补视。

内参用数据集口径 fx=fy=640 @1280×720（90° 水平视场，对齐 YOLO 训练
分布与 reference 机位）。Blender 里 36 mm 传感器 + 18 mm 镜头恰为
fx=640。坐标一律世界系。
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

Vec3 = Sequence[float]

F_X = F_Y = 640.0
WIDTH, HEIGHT = 1280, 720
SENSOR_MM = 36.0
LENS_MM = 18.0
"""lens_mm = f_x * sensor / width = 640 * 36 / 1280."""

MAX_CAMERA_STEP_M = .15
AZIMUTH_SWEEP_DEG = 30.0
MAX_VIEWS_PER_TARGET = 3

ASSOC_MAX_DIST_M = 3.0
ASSOC_MIN_DIST_M = .35
"""停靠视关联目标的距离窗；下限避开最小对焦/碰撞，上限远距小目标另计."""

PRIMARY_DIST_M = (.55, .75)
"""每目标拍照位距离窗：数据集深度 p10–p90 = 0.33–1.13 m（p50 0.61），
YOLO 训练分布集中在近距；作业道远眺（>2 m）超分布，只作 survey 分层。"""


@dataclass(frozen=True)
class CameraView:
    view_id: str
    eye: Tuple[float, float, float]
    lookat: Tuple[float, float, float]
    kind: str  # 'alley_stop' | 'primary' | 'supplemental'
    stop_id: int
    target_ids: Tuple[int, ...] = ()


def _sub(a: Vec3, b: Vec3) -> List[float]:
    return [a[i] - b[i] for i in range(3)]


def _add(a: Vec3, b: Vec3) -> List[float]:
    return [a[i] + b[i] for i in range(3)]


def _scale(a: Vec3, s: float) -> List[float]:
    return [x * s for x in a]


def _norm(a: Vec3) -> float:
    return sum(x * x for x in a) ** .5


def _unit(a: Vec3) -> List[float]:
    n = _norm(a)
    if n < 1e-12:
        raise ValueError('zero-length vector')
    return [x / n for x in a]


def supplemental_viewpoint(
    target_xyz: Vec3,
    camera_xyz: Vec3,
    bag_axis: Vec3,
    camera_front: Optional[Vec3] = None,
) -> List[float]:
    """补视机位：两候选行程短者（常数同 view_policy 原值，勿单独调参）."""
    unit_axis = _unit(bag_axis) if _norm(bag_axis) > 1e-12 else [0., 0., 1.]

    toward = _sub(target_xyz, camera_xyz)
    dist = _norm(toward)
    if dist <= MAX_CAMERA_STEP_M or dist < 1e-9:
        cand_a = list(target_xyz)
    else:
        cand_a = _add(camera_xyz, _scale(toward, MAX_CAMERA_STEP_M / dist))

    front = camera_front if camera_front is not None else _sub(
        camera_xyz, target_xyz)
    f = _norm(front)
    if f < 1e-9:
        return cand_a
    unit_front = [x / f for x in front]
    keep = sum(unit_front[i] * unit_axis[i] for i in range(3))
    radial = _sub(unit_front, _scale(unit_axis, keep))
    r = _norm(radial)
    if r < 1e-6:
        return cand_a
    unit_radial = [x / r for x in radial]
    other = [
        unit_axis[1] * unit_radial[2] - unit_axis[2] * unit_radial[1],
        unit_axis[2] * unit_radial[0] - unit_axis[0] * unit_radial[2],
        unit_axis[0] * unit_radial[1] - unit_axis[1] * unit_radial[0]]
    import math
    theta = math.radians(AZIMUTH_SWEEP_DEG)
    rotated = _add(
        _scale(unit_radial, math.cos(theta)), _scale(other, math.sin(theta)))
    direction = _add(rotated, _scale(unit_axis, keep))
    d = _norm(direction)
    if d < 1e-9:
        return cand_a
    cand_b = _add(camera_xyz, _scale(direction, MAX_CAMERA_STEP_M / d))

    return cand_a if _norm(_sub(cand_a, camera_xyz)) <= _norm(
        _sub(cand_b, camera_xyz)) else cand_b


def alley_stops(x_start: float, x_end: float, n_stops: int,
                y_alley: float, height: float,
                lookat: Vec3) -> List[CameraView]:
    """沿作业道等距停靠；lookat 为统一注视点（树带中心）."""
    if n_stops < 1:
        raise ValueError('n_stops >= 1 required')
    step = (x_end - x_start) / (n_stops - 1) if n_stops > 1 else 0.0
    views = []
    for i in range(n_stops):
        x = x_start + step * i
        views.append(CameraView(
            view_id=f'stop{i:02d}',
            eye=(x, y_alley, height),
            lookat=tuple(lookat),
            kind='alley_stop',
            stop_id=i))
    return views


def _project_ok(view: CameraView, target_xyz: Vec3) -> Optional[float]:
    """目标是否落在停靠视画面内（留 8% 边距）；返回目标距离 m."""
    forward = _unit(_sub(view.lookat, view.eye))
    world_up = [0., 0., 1.]
    right = _unit([
        forward[1] * world_up[2] - forward[2] * world_up[1],
        forward[2] * world_up[0] - forward[0] * world_up[2],
        forward[0] * world_up[1] - forward[1] * world_up[0]])
    up = [
        right[1] * forward[2] - right[2] * forward[1],
        right[2] * forward[0] - right[0] * forward[2],
        right[0] * forward[1] - right[1] * forward[0]]
    rel = _sub(target_xyz, view.eye)
    z = sum(rel[i] * forward[i] for i in range(3))
    if z <= .1:
        return None
    x = sum(rel[i] * right[i] for i in range(3)) / z * F_X + WIDTH / 2
    y = -sum(rel[i] * up[i] for i in range(3)) / z * F_Y + HEIGHT / 2
    margin_x, margin_y = WIDTH * .08, HEIGHT * .08
    if not (margin_x <= x <= WIDTH - margin_x
            and margin_y <= y <= HEIGHT - margin_y):
        return None
    return z


def build_trajectory(targets: List[dict], stops: List[CameraView],
                     view_dir: Vec3 = (0., -1., 0.),
                     max_dist_m: float = ASSOC_MAX_DIST_M,
                     min_dist_m: float = ASSOC_MIN_DIST_M,
                     ) -> Dict:
    """Survey 停靠关联 + 每目标近距拍照位与补视链 → 轨迹字典.

    targets: manifest['targets'] 条目，须含 id；拍照位取 center_world
    （无则 neck）沿 view_dir 退 PRIMARY_DIST_M，模拟产线"臂靠近冠层
    拍照 + 0.15 m/±30° 补视"的停走节拍。survey 停靠只作远眺分层，
    不进每目标聚合门。
    """
    import random as _random

    views: List[CameraView] = []
    unit_view = _unit(view_dir)
    for stop in stops:
        associated = []
        for t in targets:
            center = t.get('center_world') or t.get('neck')
            if center is None:
                continue
            dist = _project_ok(stop, center)
            if dist is None or not min_dist_m <= dist <= max_dist_m:
                continue
            associated.append(t['id'])
        views.append(CameraView(
            view_id=stop.view_id, eye=stop.eye, lookat=stop.lookat,
            kind='alley_stop', stop_id=stop.stop_id,
            target_ids=tuple(associated)))
    # Per-target primary photo pose + supplemental chain (production cadence).
    rng = _random.Random(240928)
    per_target: Dict[int, List[str]] = {}
    for t in targets:
        center = t.get('center_world') or t.get('neck')
        if center is None:
            continue
        center = list(center)
        dist = rng.uniform(*PRIMARY_DIST_M)
        drop = rng.uniform(-.05, .10)
        eye = _add(center, _scale(unit_view, dist))
        eye = [eye[0], eye[1], center[2] + drop]
        primary = CameraView(
            view_id=f'tgt{t["id"]}_v0', eye=tuple(eye),
            lookat=tuple(center), kind='primary', stop_id=-1,
            target_ids=(t['id'],))
        views.append(primary)
        axis = t.get('axis_world') or [0., 0., 1.]
        chain = [primary.view_id]
        cam = list(eye)
        for k in range(MAX_VIEWS_PER_TARGET - 1):
            nxt = supplemental_viewpoint(center, cam, axis)
            chain.append(f'tgt{t["id"]}_v{k + 1}')
            views.append(CameraView(
                view_id=chain[-1], eye=tuple(nxt),
                lookat=tuple(center), kind='supplemental', stop_id=-1,
                target_ids=(t['id'],)))
            cam = nxt
        per_target[t['id']] = chain
    return {
        'intrinsics': {'fx': F_X, 'fy': F_Y, 'width': WIDTH, 'height': HEIGHT,
                       'sensor_width_mm': SENSOR_MM, 'lens_mm': LENS_MM},
        'constants': {'max_camera_step_m': MAX_CAMERA_STEP_M,
                      'azimuth_sweep_deg': AZIMUTH_SWEEP_DEG,
                      'max_views_per_target': MAX_VIEWS_PER_TARGET,
                      'assoc_dist_window_m': [min_dist_m, max_dist_m],
                      'primary_dist_window_m': list(PRIMARY_DIST_M)},
        'views': [v.__dict__ for v in views],
        'per_target': per_target,
        'target_registry': {t['id']: {
            'name': t['name'],
            'center_world': t.get('center_world') or t.get('neck'),
            'axis_world': t.get('axis_world', [0., 0., 1.]),
            'occlusion': t.get('occlusion', {}).get('level', 'reference'),
        } for t in targets},
    }
