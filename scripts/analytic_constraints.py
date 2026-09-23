#!/usr/bin/env python3
"""解析验证：不跑仿真、不碰 DDS，把感知与轨迹约束从源码精确复刻成

注意：本脚本的 staging/折线几何为历史 v1 口径（2026-09-23 前的
果平面折线+staging PTP 候选链）；现行接近为 v4（PTP+垂直入冠+沿轴），
结果不再对照现行系统，仅作历史基线复盘。
纯几何/纯逻辑判定，在 10000 个位姿（现场包络 + 扩展包络 + 系统网格，
尽量覆盖所有情况）上做约束级性质验证与结合度分析。

复刻对象（2026-09-14 源码口径）：
- classifyApproach（grasp_task.cpp）：SKIP/LIN/LIN_ALIGN/STAGING 分档与
  容差（侧向 0.05 m、夹角 20°、轴向窗 [−0.02, standoff+0.02]、LIN 资格
  = 起点 s≤0 ∧ 直连工具有限圆柱不触果实胶囊 ∧ 弦长 ≤0.80）。
- 果实胶囊（grasp_geometry.hpp）：bottom→neck 有限段，半径 =
  max(感知直径/2, 0.025)+0.01；感知直径按 25–50 mm 采样。工具是
  TCP→后方 0.2 m、r=0.06 的有限圆柱（半径只径向，不含端球）。轴向
  投影不重叠则不判侧撞。逐段审查：staging 首段 PTP 弧只查筒体接触，
  其后 LIN 段另查反爬 s ≤ 本段起点 max(s,0)+2 cm。PTP 弧以
  「拍照位→staging 直弦」作保守下界代理（真实关节弧只会更弯）。
- 几何链：入口=拟合袋底；预抓取=入口沿 −axis 退 0.03（grasp_standoffs
  pregrasp 注入 mtc_approach_along_axis_m）；staging=预抓取再退 0.10。
- 套入行程（motion.cpp insertionTravel）：suggested 或 (颈−底)·轴−0.015，
  clamp [0.02, 0.20]。
- 滚转表（grasp_geometry.hpp toolRollsRad）：{0, ±30°, ±60°}；排序惩罚
  4·roll²（manipulation_skills_node.cpp）。
- 09-11 轨迹质量门（trajectory_guard.hpp / yaml）：绕行比 1.8、弦偏离
  0.25 m、回退 0.08 m、TCP 姿态行程绝对 110°（测地线；相对目标余量 20°）。姿态行程对
  R_target = align(R_photo, axis)·Rz(roll) 有闭式
  γ(θ, roll) = 2·acos(cos(θ/2)·cos(roll/2))，θ=拍照位工具 Z 与轴夹角
  （脚本内用四元数数值合成复核该闭式）。
- 感知约束：轴上半球；随机包络 |entry|≤1.16、距拍照位弦 ≤0.80、
  AABB=现场坐标 ±5 cm。

不变量断言（每 pose 必须成立，违例即约束链有 bug）：
 I1 轴向 LIN（staging→预抓取）工具有限圆柱恒不触果实胶囊（s 全程 <0，
    轴向投影不重叠）
 I2 SKIP ⇒ 侧向 ≤0.05 ∧ θ ≤20° ∧ 轴向在窗内
 I3 LIN/LIN_ALIGN 资格 ⇒ 起点 s ≤ 0
 I4 套入行程 ∈ [0.02, 0.20]
 I5 采样轴 z ≥ 0（感知上半球；扩展包络允许到 0）

采样随机源为自实现 64bit LCG（可复现实验用途，非加密），刻意不用
random 模块以保持零依赖与确定性。
与 analyze_approach_envelope.py 的分工：那边做感知包络分层覆盖统计；
本脚本做约束链不变量断言（I1–I5 性质测试）+ 姿态门闭式校验 +
拒发结合度地图，两者互补不互替。
用法：python3 scripts/analytic_constraints.py [--n 10000] [--seed 20260911]
不进 colcon test。"""
from __future__ import annotations

import argparse
import collections
import json
import math
import sys
import time
from pathlib import Path

import yaml

CASES_PATH = (
    Path(__file__).resolve().parents[1] /
    'src/peach_arm/test/fixtures/field_pregrasp_cases.yaml')
RESULTS_DIR = Path(__file__).resolve().parents[1] / 'runs'

# ---- 与源码/yaml 对齐的常量（改动须同步源码） ----
STANDOFF_M = 0.03        # pregrasp_standoff（launch 注入 mtc_approach_along_axis_m）
STAGING_GAP_M = 0.10     # approach_staging_standoff_m
KEEP_R_M = 0.12          # mtc_approach_keepout_radius_m 回退半径
FRUIT_INFLATION_M = 0.01
FRUIT_RADIUS_FLOOR_M = 0.025
TOOL_BODY_LENGTH_M = 0.200
TOOL_BODY_RADIUS_M = 0.060
DIAMETER_MIN_M = 0.025
DIAMETER_MAX_M = 0.050
MAX_LATERAL_M = 0.05     # mtc_approach_max_lateral_m
MAX_ALIGN_DEG = 20.0     # mtc_approach_max_align_deg
CHORD_CAP_M = 0.80       # mtc_approach_cartesian_max_distance_m
DETOUR_RATIO = 1.8       # 09-11 启用
CHORD_DEV_M = 0.25
MAX_ROT_DEG = 110.0      # mtc_approach_max_tcp_rotation_deg
NECK_MARGIN_M = 0.015
MIN_TRAVEL_M, MAX_TRAVEL_M = 0.02, 0.20
ROLLS_DEG = (0.0, 30.0, -30.0, 60.0, -60.0)   # toolRollsRad（5 档）
ROLL_PENALTY = 4.0       # 排序键 += 4·roll²(rad²)
PHOTO_XYZ = (0.302, -0.232, 0.708)
PHOTO_TOOL_Z = (-0.02, 0.01, 1.00)


class Lcg:
    """splitmix64（可复现位姿采样用，非加密）：比裸 LCG 输出混淆强，
    避免 Box–Muller 因输出聚集产生异常巨值。"""

    def __init__(self, seed):
        self.state = int(seed) & ((1 << 64) - 1)

    def next_u64(self):
        self.state = (self.state + 0x9E3779B97F4A7C15) & ((1 << 64) - 1)
        z = self.state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & ((1 << 64) - 1)
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & ((1 << 64) - 1)
        return z ^ (z >> 31)

    def uniform(self, lo=0.0, hi=1.0):
        return lo + (hi - lo) * (self.next_u64() / float(1 << 64))

    def gauss(self, sigma):
        u1 = max(1e-12, self.uniform())
        u2 = self.uniform()
        return sigma * math.sqrt(-2.0 * math.log(u1)) * \
            math.cos(2.0 * math.pi * u2)


def _norm(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def _scale(a, s):
    return [x * s for x in a]


def angle_deg(u, v):
    c = max(-1.0, min(1.0, _dot(_norm(u), _norm(v))))
    return math.degrees(math.acos(c))


def fruit_radius_m(diameter_m):
    base = diameter_m / 2.0 if diameter_m > 1.0e-6 else KEEP_R_M
    return max(FRUIT_RADIUS_FLOOR_M, base) + FRUIT_INFLATION_M


def axial_radial(p, entry, axis):
    d = _sub(p, entry)
    s = _dot(d, axis)
    r = math.sqrt(sum((d[i] - s * axis[i]) ** 2 for i in range(3)))
    return s, r


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
        s = min(1.0, max(0.0, (b * g - c * f) / denom)) if denom > 1.0e-12 else 0.0
        t = min(1.0, max(0.0, (b * s + g) / c))
        s = min(1.0, max(0.0, (b * t - f) / a))
    closest = _add(r, _sub(_scale(d1, s), _scale(d2, t)))
    return math.sqrt(_dot(closest, closest))


def tool_body_hits_fruit(tcp, tool_z, bottom, neck, axis, fruit_r):
    tail = _sub(tcp, _scale(_norm(tool_z), TOOL_BODY_LENGTH_M))
    t0, _ = axial_radial(tcp, bottom, axis)
    t1, _ = axial_radial(tail, bottom, axis)
    f0, _ = axial_radial(bottom, bottom, axis)
    f1, _ = axial_radial(neck, bottom, axis)
    amin, amax = (t0, t1) if t0 <= t1 else (t1, t0)
    bmin, bmax = (f0, f1) if f0 <= f1 else (f1, f0)
    if amin > bmax + 1.0e-9 or bmin > amax + 1.0e-9:
        return False
    clearance = _segment_segment_distance(tcp, tail, bottom, neck) - \
        TOOL_BODY_RADIUS_M - fruit_r
    return clearance <= 0.0


def segment_hits_fruit(a, b, bottom, neck, axis, fruit_r, samples=40):
    """沿弦采样，工具 Z 取果实轴（接近段目标姿态）。"""
    for i in range(samples + 1):
        t = i / samples
        p = _add(_scale(a, 1 - t), _scale(b, t))
        if tool_body_hits_fruit(p, axis, bottom, neck, axis, fruit_r):
            return True
    return False


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def quat_angle_deg(qa, qb):
    dot = abs(sum(qa[i] * qb[i] for i in range(4)))
    n1 = math.sqrt(sum(x * x for x in qa))
    n2 = math.sqrt(sum(x * x for x in qb))
    dot = min(1.0, dot / (n1 * n2))
    return 2.0 * math.degrees(math.acos(dot))


def quat_from_axis_angle(axis, deg):
    half = math.radians(deg) / 2.0
    s = math.sin(half)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half))


def two_vectors_quat(z_from, z_to):
    """Eigen FromTwoVectors(z_from, z_to) 的四元数。"""
    a = _norm(z_from)
    b = _norm(z_to)
    c = max(-1.0, min(1.0, _dot(a, b)))
    axis = (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])
    n = math.sqrt(sum(x * x for x in axis))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0) if c > 0 else (1.0, 0.0, 0.0, 0.0)
    return quat_from_axis_angle(_norm(axis), math.degrees(math.acos(c)))


def classify(entry, axis, start_xyz, start_tool_z, fruit_r, neck):
    """classifyApproach 的解析复刻（start=拍照位）。"""
    delta = _sub(entry, start_xyz)
    axial = _dot(delta, axis)
    lateral = math.sqrt(sum(
        (delta[i] - axial * axis[i]) ** 2 for i in range(3)))
    align_deg = angle_deg(start_tool_z, axis)
    aligned = align_deg <= MAX_ALIGN_DEG
    on_line = lateral <= MAX_LATERAL_M
    short_axial = (-0.02 <= axial <= STANDOFF_M + 0.02 and
                   axial <= CHORD_CAP_M)
    if aligned and on_line and short_axial:
        return 'SKIP', dict(axial=axial, lateral=lateral,
                            align_deg=align_deg)
    pregrasp = _sub(entry, _scale(axis, STANDOFF_M))
    s0, _ = axial_radial(start_xyz, entry, axis)
    chord = math.dist(start_xyz, pregrasp)
    if (s0 <= 0.0 and
        not segment_hits_fruit(
            start_xyz, pregrasp, entry, neck, axis, fruit_r) and
            chord <= CHORD_CAP_M):
        return ('LIN' if align_deg <= 2.0 else 'LIN_ALIGN',
                dict(axial=axial, lateral=lateral, align_deg=align_deg,
                     s0=s0, chord=chord))
    return 'STAGING', dict(axial=axial, lateral=lateral,
                           align_deg=align_deg, s0=s0)


def travel_of(neck, entry, axis, suggested):
    t = suggested if suggested > 1e-6 else _dot(_sub(neck, entry), axis) \
        - NECK_MARGIN_M
    return min(MAX_TRAVEL_M, max(MIN_TRAVEL_M, t))


def orientation_travel_deg(theta_deg, roll_deg):
    """γ = 2·acos(cos(θ/2)cos(roll/2))；另有四元数数值复核。"""
    c = math.cos(math.radians(theta_deg / 2)) * \
        math.cos(math.radians(roll_deg / 2))
    return 2.0 * math.degrees(math.acos(min(1.0, c)))


def numeric_travel_check(rng):
    worst = 0.0
    photo_z = _norm(PHOTO_TOOL_Z)
    for _ in range(2000):
        axis = sample_upper(rng, 0.0)
        theta = angle_deg(photo_z, axis)
        roll = rng.uniform(0.0, 180.0)
        qa = two_vectors_quat(photo_z, axis)
        qb = quat_from_axis_angle(axis, roll)
        numeric = quat_angle_deg((0, 0, 0, 1), quat_mul(qb, qa))
        closed = orientation_travel_deg(theta, roll)
        worst = max(worst, abs(numeric - closed))
    return worst


def sample_upper(rng, min_z):
    z = rng.uniform(max(0.0, min_z), 1.0)
    rho = math.sqrt(max(0.0, 1.0 - z * z))
    phi = rng.uniform(0.0, 2.0 * math.pi)
    return _norm((rho * math.cos(phi), rho * math.sin(phi), z))


def tilt(rng, axis, max_deg):
    axis = _norm(axis)
    ref = (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0)
    x = _norm((
        ref[1] * axis[2] - ref[2] * axis[1],
        ref[2] * axis[0] - ref[0] * axis[2],
        ref[0] * axis[1] - ref[1] * axis[0]))
    y = (axis[1] * x[2] - axis[2] * x[1],
         axis[2] * x[0] - axis[0] * x[2],
         axis[0] * x[1] - axis[1] * x[0])
    th = math.radians(rng.uniform(-max_deg, max_deg))
    ph = rng.uniform(0.0, 2.0 * math.pi)
    side = _add(_scale(x, math.cos(ph)), _scale(y, math.sin(ph)))
    t = _add(_scale(axis, math.cos(th)), _scale(side, math.sin(th)))
    return _norm(t if t[2] >= 0 else _scale(t, -1))


def load_sampler(rng):
    book = yaml.safe_load(CASES_PATH.read_text())
    templates = book['targets_20260909']
    ids = list(templates)
    entries = [templates[k]['entry_xyz'] for k in ids]
    xs, ys, zs = zip(*entries)
    pad = 0.05
    box = ((min(xs) - pad, max(xs) + pad),
           (min(ys) - pad, max(ys) + pad),
           (min(zs) - pad, max(zs) + pad))
    photo = list(PHOTO_XYZ)

    def pose_ok(entry, axis):
        if axis[2] < 0.0:
            return False
        if math.sqrt(sum(v * v for v in entry)) > 1.16:
            return False
        if math.dist(entry, photo) > CHORD_CAP_M:
            return False
        return all(box[i][0] <= entry[i] <= box[i][1] for i in range(3))

    def one(kind):
        src = templates[ids[rng.next_u64() % len(ids)]]
        if kind == 'field':
            entry = [src['entry_xyz'][i] + rng.gauss(0.04)
                     for i in range(3)]
            axis = tilt(rng, src['axis'], 20.0)
        else:
            entry = [rng.uniform(box[i][0] - 0.12, box[i][1] + 0.12)
                     for i in range(3)]
            axis = sample_upper(rng, 0.0)
        length = max(0.03, math.dist(src['bag_bottom'], src['bag_neck']))
        neck = _add(entry, _scale(axis, length))
        diameter = rng.uniform(DIAMETER_MIN_M, DIAMETER_MAX_M)
        return (dict(entry=entry, axis=axis, neck=neck, length=length,
                     diameter=diameter,
                     suggested=float(src.get('travel_m') or 0.0),
                     origin=kind), pose_ok(entry, axis))
    return one


def systematic_poses():
    out = []
    book = yaml.safe_load(CASES_PATH.read_text())['targets_20260909']
    entries = [v['entry_xyz'] for v in book.values()]
    xs, ys, zs_e = zip(*entries)
    box = ((min(xs), max(xs)), (min(ys), max(ys)), (min(zs_e), max(zs_e)))
    spots = [
        (box[0][0], box[1][0], box[2][0]), (box[0][1], box[1][1], box[2][1]),
        (box[0][0], box[1][1], box[2][1]), (box[0][1], box[1][0], box[2][0]),
        tuple(0.5 * (box[i][0] + box[i][1]) for i in range(3)),
    ]
    for z in (0.0, 0.15, 0.27, 0.45, 0.70, 0.90, 1.0):
        rho = math.sqrt(max(0.0, 1.0 - z * z))
        for k in range(8):
            phi = k * math.pi / 4
            axis = _norm((rho * math.cos(phi), rho * math.sin(phi), z))
            for e in spots:
                out.append(dict(entry=list(e), axis=axis,
                                neck=_add(list(e), _scale(axis, 0.07)),
                                length=0.07, diameter=0.04, suggested=0.0,
                                origin='grid'))
    return out


def seg_dev(p, b, c):
    u = _sub(c, b)
    len2 = sum(x * x for x in u)
    if len2 < 1e-12:
        return math.dist(p, b)
    t = max(0.0, min(1.0, _dot(_sub(p, b), u) / len2))
    return math.dist(p, _add(b, _scale(u, t)))


def evaluate(pose):
    entry, axis = pose['entry'], _norm(pose['axis'])
    photo = list(PHOTO_XYZ)
    photo_z = _norm(PHOTO_TOOL_Z)
    fruit_r = fruit_radius_m(pose.get('diameter', 0.04))
    neck = pose['neck']
    kind, m = classify(entry, axis, photo, photo_z, fruit_r, neck)
    pregrasp = _sub(entry, _scale(axis, STANDOFF_M))
    staging = _sub(pregrasp, _scale(axis, STAGING_GAP_M))
    travel = travel_of(pose['neck'], entry, axis, pose['suggested'])

    axial_lin_clean = not segment_hits_fruit(
        staging, pregrasp, entry, neck, axis, fruit_r)
    transit_hit = segment_hits_fruit(
        photo, staging, entry, neck, axis, fruit_r)
    path = math.dist(photo, staging) + STAGING_GAP_M
    chord = math.dist(photo, pregrasp)
    ratio = path / chord if chord >= 0.02 else 0.0
    theta = angle_deg(photo_z, axis)
    gammas = {r: orientation_travel_deg(theta, r) for r in ROLLS_DEG}
    ok_rolls = [r for r in ROLLS_DEG if gammas[r] <= MAX_ROT_DEG]

    rec = dict(
        origin=pose['origin'], kind=kind, axis_z=round(axis[2], 3),
        entry_norm=round(math.sqrt(sum(v * v for v in entry)), 3),
        theta_deg=round(theta, 1), travel_m=round(travel, 4),
        axial_lin_clean=axial_lin_clean, transit_hit=transit_hit,
        ratio=round(ratio, 2), chord=round(chord, 3),
        gamma_deg={str(int(r)): round(gammas[r], 1) for r in ROLLS_DEG},
        ok_rolls=ok_rolls,
        s0=round(m['s0'], 3) if 's0' in m else None,
        lateral=round(m['lateral'], 3), align_deg=round(m['align_deg'], 1),
    )
    v = []
    if not axial_lin_clean:
        v.append('I1')
    if kind == 'SKIP' and not (m['lateral'] <= MAX_LATERAL_M + 1e-9 and
                               m['align_deg'] <= MAX_ALIGN_DEG + 1e-9):
        v.append('I2')
    if kind in ('LIN', 'LIN_ALIGN') and not (m.get('s0', 1.0) <= 0.0):
        v.append('I3')
    if not (MIN_TRAVEL_M - 1e-9 <= travel <= MAX_TRAVEL_M + 1e-9):
        v.append('I4')
    if axis[2] < -1e-9:
        v.append('I5')
    rec['violations'] = v
    reason = None
    if transit_hit:
        reason = 'fruit_capsule_ptp'
    elif ratio > DETOUR_RATIO:
        reason = 'detour_ratio'
    elif not ok_rolls:
        reason = 'orientation_gate'
    elif seg_dev(photo, photo, pregrasp) > CHORD_DEV_M or \
            seg_dev(staging, photo, pregrasp) > CHORD_DEV_M:
        reason = 'chord_dev'
    rec['first_reject'] = reason
    return rec


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--n', type=int, default=10000)
    parser.add_argument('--seed', type=int, default=20260911)
    args = parser.parse_args()

    rng = Lcg(args.seed)
    worst = numeric_travel_check(Lcg(args.seed + 1))
    print(f'姿态行程闭式 vs 四元数数值最大偏差: {worst:.4f}° '
          f'({"一致" if worst < 1e-3 else "不一致!"})')

    one = load_sampler(rng)
    poses = []
    n_field = int(args.n * 0.5)
    n_ext = int(args.n * 0.3)
    attempts = 0
    while len(poses) < n_field + n_ext and attempts < (n_field + n_ext) * 80:
        attempts += 1
        kind = 'field' if len(poses) < n_field else 'extended'
        pose, ok = one(kind)
        if ok:
            poses.append(pose)
    grid = systematic_poses()
    poses.extend(grid)
    print(f'采样 {len(poses)} 位姿（field {n_field} / extended '
          f'{len(poses) - len(grid) - n_field} / grid {len(grid)}）')

    recs = [evaluate(p) for p in poses]
    out_path = RESULTS_DIR / (
        f'analytic_constraints_{time.strftime("%Y%m%d_%H%M%S")}.jsonl')
    with open(out_path, 'a', encoding='utf-8') as sink:
        for r in recs:
            sink.write(json.dumps(r, ensure_ascii=False) + '\n')

    viol = [r for r in recs if r['violations']]
    print(f'\n不变量违例: {len(viol)}/{len(recs)}')
    for r in viol[:8]:
        print('  !', r['violations'], r['origin'], 'axis_z', r['axis_z'],
              'kind', r['kind'])
    kinds = collections.Counter(r['kind'] for r in recs)
    print('分档分布:', dict(kinds))
    for origin in ('field', 'extended', 'grid'):
        sub = [r for r in recs if r['origin'] == origin]
        if not sub:
            continue
        rej = collections.Counter(
            r['first_reject'] for r in sub if r['first_reject'])
        clean = sum(1 for r in sub if not r['first_reject'])
        print(f'{origin}: {len(sub)} 位姿 | 解析预检放行 {clean} '
              f'({clean / len(sub):.0%}) | 拒发分布 {dict(rej)}')
    print('\n姿态门可行域（θ=拍照Z与轴夹角 → 可行滚转档）:')
    for lo in range(0, 91, 15):
        sub = [r for r in recs if lo <= r['theta_deg'] < lo + 15]
        if not sub:
            continue
        worst_r = max(sub, key=lambda r: r['theta_deg'])
        print(f'  θ∈[{lo:2d},{lo + 15:2d})°: n={len(sub):4d} '
              f'最少可行档={min(len(r["ok_rolls"]) for r in sub)} '
              f'（最差例 θ={worst_r["theta_deg"]:.0f}° '
              f'ok_rolls={worst_r["ok_rolls"]}）')
    pen60 = ROLL_PENALTY * math.radians(60.0) ** 2
    pen30 = ROLL_PENALTY * math.radians(30.0) ** 2
    print(f'\n滚转排序惩罚: ±30° → +{pen30:.2f} rad², '
          f'±60° → +{pen60:.2f} rad²（典型关节距离² ~0.5–3 rad²）')
    print(f'结果已写入 {out_path}')
    return 1 if viol else 0


if __name__ == '__main__':
    sys.exit(main())
