#!/usr/bin/env python3
"""接近包络回放塔的语料采样与护栏判定（零 ROS 纯几何）。

逐字移植自 scripts/sim_field_targets.py、scripts/sim_approach_probe.py、
scripts/analyze_approach_envelope.py（基线 a955cea），唯一改动是案册路径
改为本文件源码相对定位。这是 C++ 护栏（grasp_geometry.hpp /
trajectory_guard.hpp）的 Python 复刻，作回放回归的冻结判据；阶段 2 起
peach_arm 侧以同一案册喂真实纯核 gtest 交叉对账。

不执臂、不规划关节：analytic_ok 不声称累计行程 / 单轴 / PTP 弧绕行
（见 analyze_approach_envelope 原始口径）。
"""
from __future__ import annotations

import math
import random
from pathlib import Path

import yaml

CASES_PATH = (
    Path(__file__).resolve().parents[2] /
    'peach_arm/test/fixtures/field_pregrasp_cases.yaml')

# --- 常量（sim_field_targets.py 原值） ---
CART_MAX_M = 0.80            # mtc_approach_cartesian_max_distance_m
ENTRY_NORM_MAX_M = 1.16
TYPICAL_ENTRY_NORM_M = 1.02
TYPICAL_AXIS_Z_MIN = 0.70
ENTRY_JITTER_M = 0.04
AXIS_TILT_DEG = 20.0
AABB_PAD_M = 0.05
KEEP_R_M = 0.12
KEEP_AXIAL_M = 0.12
TOOL_BODY_LENGTH_M = 0.200
TOOL_BODY_RADIUS_M = 0.060
FRUIT_INFLATION_M = 0.01
FRUIT_RADIUS_FLOOR_M = 0.025
SIM_BAG_DIAMETER_M = 0.06

# --- 常量（sim_approach_probe.py 原值） ---
STANDOFF_M = 0.03            # grasp_standoffs.yaml pregrasp_standoff_m（deploy 注入 0.0）
FINAL_AXIAL_M = 0.05         # v4 approach_final_axial_m（审查 P1-4 更新）
CANOPY_ENTRY_M = 0.05        # v4 approach_canopy_entry_m（世界垂直入冠段）

# --- 常量（analyze_approach_envelope.py 原值） ---
ROLLS_DEG = (0, 30, -30, 60, -60)
TCP_ROT_MAX_DEG = 110.0
CLIMB_SLACK_M = 0.02
KEEP_SAMPLES = 40
STRATA = (
    ('typical', 0.40),       # axis_z>=0.70 且 |entry|<=1.02
    ('mid_tilt', 0.20),      # 0.40<=axis_z<0.70
    ('horizontal', 0.20),    # axis_z<0.40（含 1021 近水平）
    ('far_entry', 0.10),     # 1.02<|entry|<=1.16
    ('long_chord', 0.10),    # 拍照位->入口弦 0.55-0.80 m
)


# --- 向量原语（sim_field_targets.py 原值） ---
def _norm(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def _in_typical_envelope(entry, axis) -> bool:
    n = math.sqrt(sum(v * v for v in entry))
    return n <= TYPICAL_ENTRY_NORM_M and axis[2] >= TYPICAL_AXIS_Z_MIN


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _scale(a, s):
    return [x * s for x in a]


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0])


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def fruit_radius_m(diameter_m, inflation=FRUIT_INFLATION_M,
                    fallback=KEEP_R_M):
    base = diameter_m / 2.0 if diameter_m > 1.0e-6 else fallback
    return max(FRUIT_RADIUS_FLOOR_M, base) + max(0.0, inflation)


def _segment_segment_distance(p1, q1, p2, q2):
    d1 = _sub(q1, p1)
    d2 = _sub(q2, p2)
    r = _sub(p1, p2)
    a = _dot(d1, d1)
    b = _dot(d1, d2)
    c = _dot(d2, d2)
    f = _dot(r, d1)
    g = _dot(r, d2)
    denom = a * c - b * b
    s = 0.0
    t = 0.0
    if a <= 1.0e-12 and c <= 1.0e-12:
        return math.sqrt(_dot(r, r))
    if a <= 1.0e-12:
        t = min(1.0, max(0.0, g / c))
    elif c <= 1.0e-12:
        s = min(1.0, max(0.0, -f / a))
    else:
        s = min(1.0, max(0.0, (b * g - c * f) / denom)) \
            if denom > 1.0e-12 else 0.0
        t = min(1.0, max(0.0, (b * s + g) / c))
        s = min(1.0, max(0.0, (b * t - f) / a))
    closest = _add(r, _sub(_scale(d1, s), _scale(d2, t)))
    return math.sqrt(_dot(closest, closest))


def _axial_of(point, bottom, axis):
    return _dot(_sub(point, bottom), axis)


def tool_body_hits_fruit(tcp, tool_z, bottom, neck, axis, fruit_r):
    """复刻 grasp_geometry.hpp toolCapsuleClearance<=0（有限圆柱，无端球）。"""
    if KEEP_AXIAL_M <= 1.0e-6 or fruit_r <= 1.0e-6:
        return False
    tail = _sub(tcp, _scale(_norm(tool_z), TOOL_BODY_LENGTH_M))
    t0 = _axial_of(tcp, bottom, axis)
    t1 = _axial_of(tail, bottom, axis)
    f0 = _axial_of(bottom, bottom, axis)
    f1 = _axial_of(neck, bottom, axis)
    amin, amax = (t0, t1) if t0 <= t1 else (t1, t0)
    bmin, bmax = (f0, f1) if f0 <= f1 else (f1, f0)
    if amin > bmax + 1.0e-9 or bmin > amax + 1.0e-9:
        return False
    clearance = _segment_segment_distance(tcp, tail, bottom, neck) - \
        TOOL_BODY_RADIUS_M - fruit_r
    return clearance <= 0.0


def _bag_length(case) -> float:
    return math.dist(case['bag_bottom'], case['bag_neck'])


def _photo_tcp(book) -> list:
    return [float(v) for v in book.get('photo_tcp_xyz',
                                       [0.302, -0.232, 0.708])]


def _aabb(points, pad):
    xs, ys, zs = zip(*points)
    return (
        (min(xs) - pad, max(xs) + pad),
        (min(ys) - pad, max(ys) + pad),
        (min(zs) - pad, max(zs) + pad))


def _clamp_upper_hemisphere(bottom, neck, axis):
    """袋底->袋口只许上半球（axis_z>=0，含水平）；只翻符号不改斜袋。"""
    axis = _norm(axis)
    if _dot(_sub(neck, bottom), axis) < 0.0:
        axis = [-v for v in axis]
    if axis[2] < 0.0:
        return list(neck), list(bottom), [-v for v in axis]
    return list(bottom), list(neck), list(axis)


def _tilt_axis(axis, rng, max_deg):
    axis = _norm(axis)
    ref = (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0)
    x = _norm(_cross(ref, axis))
    y = _norm(_cross(axis, x))
    theta = math.radians(rng.uniform(-max_deg, max_deg))
    phi = rng.uniform(0.0, 2.0 * math.pi)
    side = _add(_scale(x, math.cos(phi)), _scale(y, math.sin(phi)))
    tilted = _add(_scale(axis, math.cos(theta)), _scale(side, math.sin(theta)))
    if tilted[2] < 0.0:
        tilted = [-v for v in tilted]
    return _norm(tilted)


def _sample_upper_hemisphere(rng, min_z):
    floor = max(0.0, float(min_z))
    z = rng.uniform(floor, 1.0)
    rho = math.sqrt(max(0.0, 1.0 - z * z))
    phi = rng.uniform(0.0, 2.0 * math.pi)
    return _norm((rho * math.cos(phi), rho * math.sin(phi), z))


def _pose_ok(entry, axis, photo_tcp, box, axis_z_min, entry_norm_max):
    if any(not math.isfinite(v) for v in entry + axis):
        return False
    if axis[2] < axis_z_min:
        return False
    if math.sqrt(sum(v * v for v in entry)) > entry_norm_max:
        return False
    if math.dist(entry, photo_tcp) > CART_MAX_M:
        return False
    for i, (lo, hi) in enumerate(box):
        if entry[i] < lo or entry[i] > hi:
            return False
    return True


def load_casebook() -> dict:
    return yaml.safe_load(CASES_PATH.read_text())


# --- 姿态数学（sim_approach_probe.py 原值） ---
def _quat_to_mat(q):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _mat_to_quat(m):
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s,
                (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s,
            0.25 * s, (m[1][0] - m[0][1]) / s)


def _matmul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3))


def _align_z(rot, axis):
    """grasp_geometry.hpp alignFrameZ 的 Python 复刻（最小旋转，滚转保留）。"""
    z = (rot[0][2], rot[1][2], rot[2][2])
    a = _norm(axis)
    c = max(-1.0, min(1.0, sum(z[i] * a[i] for i in range(3))))
    v = (z[1] * a[2] - z[2] * a[1], z[2] * a[0] - z[0] * a[2],
         z[0] * a[1] - z[1] * a[0])
    vn = math.sqrt(sum(x * x for x in v))
    if vn < 1e-9:
        return rot if c > 0 else _matmul(_quat_to_mat((1, 0, 0, 0)), rot)
    angle = math.acos(c)
    k = tuple(x / vn for x in v)
    s = math.sin(angle / 2)
    dq = (k[0] * s, k[1] * s, k[2] * s, math.cos(angle / 2))
    return _matmul(_quat_to_mat(dq), rot)


def _roll_z(rot, deg):
    half = math.radians(deg) / 2.0
    return _matmul(rot, _quat_to_mat(
        (0, 0, math.sin(half), math.cos(half))))


def _angle_deg(u, v):
    c = max(-1.0, min(1.0, sum(a * b for a, b in zip(_norm(u), _norm(v)))))
    return math.degrees(math.acos(c))


# --- 护栏判定（analyze_approach_envelope.py 原值） ---
def _axial_radial(point, entry, axis):
    delta = [point[i] - entry[i] for i in range(3)]
    s = sum(delta[i] * axis[i] for i in range(3))
    r = math.sqrt(max(0.0, sum(
        (delta[i] - s * axis[i]) ** 2 for i in range(3))))
    return s, r


def _sample_segment(start, end, count=KEEP_SAMPLES):
    n = max(2, count)
    return [
        [(1.0 - t) * start[i] + t * end[i] for i in range(3)]
        for t in (k / n for k in range(n + 1))]


def inspect_keepout(points, entry, axis, audit_climb, fruit_r=None):
    """复刻 grasp_geometry.hpp inspectToolVsFruit（有限圆柱+反爬）。"""
    if KEEP_AXIAL_M <= 1e-6 or len(points) < 2:
        return True, '果实胶囊审查关闭或点列不足'
    radius = fruit_radius_m(SIM_BAG_DIAMETER_M) if fruit_r is None else fruit_r
    bottom = list(entry)
    neck = _add(bottom, _scale(axis, 0.07))
    start_s, _ = _axial_radial(points[0], entry, axis)
    s_max = (start_s if start_s > 0.0 else 0.0) + CLIMB_SLACK_M
    for p in points:
        s, r = _axial_radial(p, entry, axis)
        if audit_climb and s > s_max:
            return False, f'从果上方绕行 s={s:.4f}m > 上限 {s_max:.4f}m'
        if tool_body_hits_fruit(p, axis, bottom, neck, axis, radius):
            return False, (
                f'工具筒体接触果实胶囊 s={s:.4f}m r={r:.4f}m '
                f'R={radius:.4f}m')
    return True, '果实胶囊审查通过'


def quat_geodesic_deg(qa, qb):
    """复刻 trajectory_guard.hpp quatGeodesicDeg（q 与 -q 同一姿态）。"""
    na = math.sqrt(sum(v * v for v in qa))
    nb = math.sqrt(sum(v * v for v in qb))
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    dot = abs(sum(qa[i] * qb[i] for i in range(4))) / (na * nb)
    return 2.0 * math.degrees(math.acos(min(1.0, dot)))


# --- 语料采样（sim_field_targets.py / analyze_approach_envelope.py 原值） ---
def sample_random_cases(templates, n, seed, photo_tcp,
                        envelope='typical'):
    """现场坐标包络内采感知合法随机位姿（同 seed 确定性）。"""
    if envelope not in ('typical', 'algorithm'):
        raise ValueError(f'未知 envelope={envelope!r}')
    rng = random.Random(seed)
    ids = list(templates)
    entries = [templates[k]['entry_xyz'] for k in ids]
    box = _aabb(entries, AABB_PAD_M)
    if envelope == 'typical':
        axis_z_min = TYPICAL_AXIS_Z_MIN
        entry_norm_max = TYPICAL_ENTRY_NORM_M
    else:
        axis_z_min = 0.0
        entry_norm_max = ENTRY_NORM_MAX_M
    hemisphere_floor = axis_z_min
    if envelope == 'algorithm':
        hemisphere_floor = min(templates[k]['axis'][2] for k in ids)
        hemisphere_floor = max(0.0, float(hemisphere_floor))
    out = {}
    attempts = 0
    while len(out) < n and attempts < n * 80:
        attempts += 1
        kind = 'perturb' if rng.random() < 0.6 else 'aabb'
        if kind == 'perturb':
            src_id = rng.choice(ids)
            src = templates[src_id]
            entry = [
                src['entry_xyz'][i] + rng.gauss(0.0, ENTRY_JITTER_M)
                for i in range(3)]
            axis = _tilt_axis(src['axis'], rng, AXIS_TILT_DEG)
            length = _bag_length(src)
        else:
            src_id = rng.choice(ids)
            src = templates[src_id]
            entry = [rng.uniform(lo, hi) for (lo, hi) in box]
            axis = _sample_upper_hemisphere(rng, hemisphere_floor)
            length = _bag_length(src)
        if length < 0.03:
            length = 0.06
        bottom = list(entry)
        neck = _add(bottom, _scale(axis, length))
        bottom, neck, axis = _clamp_upper_hemisphere(bottom, neck, axis)
        entry = list(bottom)
        if not _pose_ok(
                entry, axis, photo_tcp, box, axis_z_min, entry_norm_max):
            continue
        cid = f'rand_{len(out):02d}'
        out[cid] = {
            'stratum': f'random_{envelope}',
            'entry_xyz': [round(v, 6) for v in entry],
            'axis': [round(v, 6) for v in axis],
            'bag_bottom': [round(v, 6) for v in bottom],
            'bag_neck': [round(v, 6) for v in neck],
            'sample_kind': kind,
            'sampled_from': src_id,
        }
    if len(out) < n:
        raise RuntimeError(f'仅采到 {len(out)}/{n} 例')
    return out


def _workspace(templates, photo):
    entries = [t['entry_xyz'] for t in templates.values()]
    box = _aabb(entries, AABB_PAD_M + 0.12)
    lo = [min(box[i][0], photo[i] - CART_MAX_M) for i in range(3)]
    hi = [max(box[i][1], photo[i] + CART_MAX_M) for i in range(3)]
    lo[2] = max(0.15, lo[2])
    hi[2] = min(1.15, hi[2])
    return tuple((lo[i], hi[i]) for i in range(3))


def _axis_in_band(rng, z_lo, z_hi):
    z = rng.uniform(z_lo, z_hi)
    rho = math.sqrt(max(0.0, 1.0 - z * z))
    phi = rng.uniform(0.0, 2.0 * math.pi)
    return _norm((rho * math.cos(phi), rho * math.sin(phi), z))


def _try_entry(rng, box, photo, r_lo, r_hi, chord_lo, chord_hi):
    entry = [rng.uniform(lo, hi) for (lo, hi) in box]
    n = math.sqrt(sum(v * v for v in entry))
    chord = _dist(entry, photo)
    if not (r_lo <= n <= r_hi):
        return None
    if not (chord_lo <= chord <= chord_hi):
        return None
    if n > ENTRY_NORM_MAX_M or chord > CART_MAX_M:
        return None
    return entry


def _fill_stratum(stratum, need, rng, templates, photo, box, ids):
    out = []
    attempts = 0
    limit = max(need * 80, 2000)
    while len(out) < need and attempts < limit:
        attempts += 1
        src = templates[rng.choice(ids)]
        length = max(_bag_length(src), 0.06)
        if stratum == 'typical':
            axis = _axis_in_band(rng, TYPICAL_AXIS_Z_MIN, 1.0)
            entry = _try_entry(
                rng, box, photo, 0.35, TYPICAL_ENTRY_NORM_M, 0.0, CART_MAX_M)
        elif stratum == 'mid_tilt':
            axis = _axis_in_band(rng, 0.40, TYPICAL_AXIS_Z_MIN)
            entry = _try_entry(
                rng, box, photo, 0.35, ENTRY_NORM_MAX_M, 0.0, CART_MAX_M)
        elif stratum == 'horizontal':
            axis = _axis_in_band(rng, 0.0, 0.40)
            entry = _try_entry(
                rng, box, photo, 0.35, ENTRY_NORM_MAX_M, 0.0, CART_MAX_M)
        elif stratum == 'far_entry':
            axis = _axis_in_band(rng, 0.0, 1.0)
            entry = _try_entry(
                rng, box, photo, TYPICAL_ENTRY_NORM_M + 1e-4,
                ENTRY_NORM_MAX_M, 0.0, CART_MAX_M)
        else:  # long_chord
            axis = _axis_in_band(rng, 0.0, 1.0)
            entry = _try_entry(
                rng, box, photo, 0.35, ENTRY_NORM_MAX_M, 0.55, CART_MAX_M)
        if entry is None:
            continue
        bottom, neck, axis = _clamp_upper_hemisphere(
            list(entry), _add(entry, _scale(axis, length)), axis)
        entry = list(bottom)
        if not _pose_ok(
                entry, axis, photo, box, 0.0, ENTRY_NORM_MAX_M):
            continue
        tag = stratum
        if stratum == 'typical' and not _in_typical_envelope(entry, axis):
            continue
        if stratum == 'mid_tilt' and not (0.40 <= axis[2] < TYPICAL_AXIS_Z_MIN):
            continue
        if stratum == 'horizontal' and axis[2] >= 0.40:
            continue
        if stratum == 'far_entry' and math.sqrt(
                sum(v * v for v in entry)) <= (TYPICAL_ENTRY_NORM_M):
            continue
        if stratum == 'long_chord' and _dist(entry, photo) < 0.55:
            continue
        out.append({
            'stratum': tag,
            'entry_xyz': [round(v, 6) for v in entry],
            'axis': [round(v, 6) for v in axis],
            'bag_bottom': [round(v, 6) for v in bottom],
            'bag_neck': [round(v, 6) for v in neck],
            'sampled_from': src.get('run', 'field'),
        })
    return out


def sample_stratified(n, seed, templates, photo, box):
    rng = random.Random(seed)
    ids = list(templates)
    counts = []
    assigned = 0
    for i, (name, frac) in enumerate(STRATA):
        k = int(round(n * frac)) if i < len(STRATA) - 1 else n - assigned
        counts.append((name, max(0, k)))
        assigned += max(0, k)
    cases = {}
    # 现场真袋钉在最前，不占分层配额。
    for src_id, src in templates.items():
        axis = _norm(src['axis'])
        entry = list(src['entry_xyz'])
        bottom, neck, axis = _clamp_upper_hemisphere(
            src.get('bag_bottom', entry),
            src.get('bag_neck', _add(entry, _scale(axis, 0.06))),
            axis)
        entry = list(bottom)
        cid = f'field_{src_id}'
        cases[cid] = {
            'stratum': 'field',
            'entry_xyz': [round(v, 6) for v in entry],
            'axis': [round(v, 6) for v in axis],
            'bag_bottom': [round(v, 6) for v in bottom],
            'bag_neck': [round(v, 6) for v in neck],
            'sampled_from': src_id,
        }
    for name, need in counts:
        packed = _fill_stratum(
            name, need, rng, templates, photo, box, ids)
        for pose in packed:
            cid = f'{name}_{len(cases):05d}'
            cases[cid] = pose
    return cases


def field_cases(templates):
    """分层采样的现场真袋子集（同 sample_stratified 前缀，单独取用）。"""
    out = {}
    for src_id, src in templates.items():
        axis = _norm(src['axis'])
        entry = list(src['entry_xyz'])
        bottom, neck, axis = _clamp_upper_hemisphere(
            src.get('bag_bottom', entry),
            src.get('bag_neck', _add(entry, _scale(axis, 0.06))),
            axis)
        out[f'field_{src_id}'] = {
            'stratum': 'field',
            'entry_xyz': [round(v, 6) for v in bottom],
            'axis': [round(v, 6) for v in axis],
            'bag_bottom': [round(v, 6) for v in bottom],
            'bag_neck': [round(v, 6) for v in neck],
            'sampled_from': src_id,
        }
    return out


def evaluate(case, photo_xyz, photo_rot, photo_quat):
    """单例解析判定（analyze_approach_envelope.evaluate 原值）。"""
    entry = case['entry_xyz']
    axis = _norm(case['axis'])
    aligned = _align_z(photo_rot, axis)
    pregrasp = [entry[i] - STANDOFF_M * axis[i] for i in range(3)]
    # v4：mid=pregrasp−axis·final_axial；staging=mid−ẑ·canopy（世界垂直）
    mid = [pregrasp[i] - FINAL_AXIAL_M * axis[i] for i in range(3)]
    staging = [mid[0], mid[1], mid[2] - CANOPY_ENTRY_M]
    chord_entry = _dist(entry, photo_xyz)
    chord_pre = _dist(pregrasp, photo_xyz)
    chord_stg = _dist(staging, photo_xyz)
    photo_s, photo_r = _axial_radial(photo_xyz, entry, axis)
    stg_s, stg_r = _axial_radial(staging, entry, axis)
    rot_by_roll = {}
    for deg in ROLLS_DEG:
        q = _mat_to_quat(_roll_z(aligned, deg))
        rot_by_roll[deg] = round(quat_geodesic_deg(photo_quat, q), 2)
    rot_ok_rolls = [d for d, deg in rot_by_roll.items()
                    if deg <= TCP_ROT_MAX_DEG]
    stg_pts = _sample_segment(photo_xyz, staging)
    stg_ok, stg_why = inspect_keepout(
        stg_pts, entry, axis, audit_climb=False)
    lin_pts = _sample_segment(staging, pregrasp)
    lin_ok, lin_why = inspect_keepout(
        lin_pts, entry, axis, audit_climb=True)
    typical = _in_typical_envelope(entry, axis)
    perception = (
        axis[2] >= 0.0
        and math.sqrt(sum(v * v for v in entry)) <= ENTRY_NORM_MAX_M
        and chord_entry <= CART_MAX_M)
    # 直连 LIN 兜底资格：起点已在袋底侧且弦不超 0.80（主路径不受此限）。
    lin_fallback = photo_s <= 0.0 and chord_pre <= CART_MAX_M and lin_ok
    reasons = []
    if not perception:
        reasons.append('perception')
    if not lin_ok:
        reasons.append('axial_keepout')
    if not rot_ok_rolls:
        reasons.append('tcp_rotation')
    if stg_s > 0.0:
        reasons.append('staging_not_below_bag')
    analytic_ok = not reasons
    return {
        'stratum': case['stratum'],
        'entry_xyz': case['entry_xyz'],
        'axis': case['axis'],
        'axis_z': round(axis[2], 4),
        'typical': typical,
        'perception_ok': perception,
        'photo_s_m': round(photo_s, 4),
        'staging_s_m': round(stg_s, 4),
        'tcp_rot_ok_rolls': rot_ok_rolls,
        'staging_chord_keepout_ok': stg_ok,
        'axial_keepout_ok': lin_ok,
        'lin_fallback_ok': lin_fallback,
        'analytic_ok': analytic_ok,
        'fail': reasons,
    }


def evaluate_all(cases, book):
    photo_xyz = _photo_tcp(book)
    photo_quat = tuple(float(v) for v in book.get(
        'photo_tcp_xyzw', [-0.005373, -0.009976, -0.037437, 0.999235]))
    photo_rot = _quat_to_mat(photo_quat)
    return {
        cid: evaluate(case, photo_xyz, photo_rot, photo_quat)
        for cid, case in cases.items()}
