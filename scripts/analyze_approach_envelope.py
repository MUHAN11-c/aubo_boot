#!/usr/bin/env python3
"""感知包络 + 接近护栏的解析覆盖（不执臂、不调 ExecuteTarget）。

对照现行约束（peach_perception 上半球 clamp；peach_arm 弦长 /
果实胶囊+反爬 / TCP 姿态测地线绝对 110°、相对起止余量 20°）：在算法包络内分层抽 N 个入口+轴，对每例
复刻 C++ 的闭式审查。主路径是 staging PTP，**弦 keepout 只作 LIN 对照、
不计入 analytic_ok**（PTP 弧须 FK）。不规划关节，因此 **不** 声称累计
行程 12 rad / 单轴 6.1 rad、也不声称 PTP 弧笛卡尔绕行比。

用法：
  python3 scripts/analyze_approach_envelope.py --n 10000 --seed 20260911
不进 colcon test。
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_approach_probe import (  # noqa: E402
    STAGING_GAP_M, STANDOFF_M, _align_z, _angle_deg, _mat_to_quat,
    _quat_to_mat, _roll_z)
from sim_field_targets import (  # noqa: E402
    AABB_PAD_M, CART_MAX_M, ENTRY_NORM_MAX_M, KEEP_AXIAL_M,
    SIM_BAG_DIAMETER_M, TYPICAL_AXIS_Z_MIN, TYPICAL_ENTRY_NORM_M, _aabb, _add,
    _bag_length, _clamp_upper_hemisphere, _in_typical_envelope, _norm,
    _pose_ok, _scale, fruit_radius_m, load_casebook, tool_body_hits_fruit)

RESULTS_DIR = Path(__file__).resolve().parents[1] / 'runs'
ROLLS_DEG = (0, 30, -30, 60, -60)
TCP_ROT_MAX_DEG = 110.0
TCP_ROT_SLACK_DEG = 20.0
CLIMB_SLACK_M = 0.02
KEEP_SAMPLES = 40
# 分层配额合计 --n；按比例缩放。
STRATA = (
    ('typical', 0.40),       # axis_z≥0.70 ∧ |entry|≤1.02
    ('mid_tilt', 0.20),      # 0.40≤axis_z<0.70
    ('horizontal', 0.20),    # axis_z<0.40（含 1021 近水平）
    ('far_entry', 0.10),     # 1.02<|entry|≤1.16
    ('long_chord', 0.10),    # 拍照位→入口弦 0.55–0.80 m
)


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


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
    """复刻 trajectory_guard.hpp quatGeodesicDeg（q 与 −q 同一姿态）。"""
    na = math.sqrt(sum(v * v for v in qa))
    nb = math.sqrt(sum(v * v for v in qb))
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    dot = abs(sum(qa[i] * qb[i] for i in range(4))) / (na * nb)
    return 2.0 * math.degrees(math.acos(min(1.0, dot)))


def _workspace(templates, photo):
    entries = [t['entry_xyz'] for t in templates.values()]
    box = _aabb(entries, AABB_PAD_M + 0.12)
    # 算法包络允许把入口扩到 |entry|≤1.16 且拍照弦 ≤0.80，不锁死现场 AABB。
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
        if stratum == 'far_entry' and math.sqrt(sum(v * v for v in entry)) <= (
                TYPICAL_ENTRY_NORM_M):
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
        if len(packed) < need:
            print(
                f'警告: 分层 {name} 只采到 {len(packed)}/{need}',
                file=sys.stderr)
        for pose in packed:
            cid = f'{name}_{len(cases):05d}'
            cases[cid] = pose
    return cases


def evaluate(case, photo_xyz, photo_rot, photo_quat):
    entry = case['entry_xyz']
    axis = _norm(case['axis'])
    aligned = _align_z(photo_rot, axis)
    pregrasp = [entry[i] - STANDOFF_M * axis[i] for i in range(3)]
    staging = [
        entry[i] - (STANDOFF_M + STAGING_GAP_M) * axis[i] for i in range(3)]
    chord_entry = _dist(entry, photo_xyz)
    chord_pre = _dist(pregrasp, photo_xyz)
    chord_stg = _dist(staging, photo_xyz)
    photo_s, photo_r = _axial_radial(photo_xyz, entry, axis)
    stg_s, stg_r = _axial_radial(staging, entry, axis)
    rot_by_roll = {}
    for deg in ROLLS_DEG:
        q = _mat_to_quat(_roll_z(aligned, deg))
        rot_by_roll[deg] = round(quat_geodesic_deg(photo_quat, q), 2)
    rot_ok_rolls = [d for d, deg in rot_by_roll.items() if deg <= TCP_ROT_MAX_DEG]
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
        'sampled_from': case.get('sampled_from'),
        'entry_xyz': case['entry_xyz'],
        'axis': case['axis'],
        'axis_z': round(axis[2], 4),
        'entry_norm_m': round(math.sqrt(sum(v * v for v in entry)), 4),
        'typical': typical,
        'perception_ok': perception,
        'chord_entry_m': round(chord_entry, 4),
        'chord_pregrasp_m': round(chord_pre, 4),
        'chord_staging_m': round(chord_stg, 4),
        'photo_s_m': round(photo_s, 4),
        'photo_r_m': round(photo_r, 4),
        'staging_s_m': round(stg_s, 4),
        'staging_r_m': round(stg_r, 4),
        'flip_deg': round(_angle_deg(
            (photo_rot[0][2], photo_rot[1][2], photo_rot[2][2]), axis), 2),
        'tcp_rot_deg': rot_by_roll,
        'tcp_rot_ok_rolls': rot_ok_rolls,
        'staging_chord_keepout_ok': stg_ok,
        'staging_chord_keepout_reason': stg_why,
        'axial_keepout_ok': lin_ok,
        'axial_keepout_reason': lin_why,
        'lin_fallback_ok': lin_fallback,
        'analytic_ok': analytic_ok,
        'fail': reasons,
        'unobserved': ['joint_travel', 'ptp_cartesian_detour', 'ik_self_collision'],
    }


def _summarize(rows):
    n = len(rows)
    ok = sum(1 for r in rows if r['analytic_ok'])
    typical = [r for r in rows if r['typical']]
    typical_ok = sum(1 for r in typical if r['analytic_ok'])
    chord_hit = sum(1 for r in rows if not r['staging_chord_keepout_ok'])
    print(f'解析通过 {ok}/{n}；typical 子集 {typical_ok}/{len(typical)}')
    print(f'拍照→staging 直连弦穿囊（LIN 对照，不计 analytic_ok）{chord_hit}/{n}')
    fail_c = collections.Counter()
    for r in rows:
        if not r['analytic_ok']:
            fail_c[tuple(r['fail'])] += 1
    print('分层样本:')
    strata_ok = collections.Counter()
    strata_n = collections.Counter()
    for r in rows:
        strata_n[r['stratum']] += 1
        if r['analytic_ok']:
            strata_ok[r['stratum']] += 1
    for name, _ in STRATA:
        print(f'  {name}: {strata_ok[name]}/{strata_n[name]}')
    if strata_n['field']:
        print(f'  field: {strata_ok["field"]}/{strata_n["field"]}')
    print('失败组合 (gate 并集):')
    for keys, c in fail_c.most_common(12):
        print(f'  {list(keys) or ["(none)"]}: {c}')
    # 覆盖：axis_z / |entry| / 弦长 分箱不得空。
    def hist(getter, edges, label):
        buckets = [0] * (len(edges) - 1)
        for r in rows:
            v = getter(r)
            for i in range(len(edges) - 1):
                if edges[i] <= v < edges[i + 1] or (
                        i == len(edges) - 2 and v == edges[i + 1]):
                    buckets[i] += 1
                    break
        print(label + ': ' + ', '.join(
            f'[{edges[i]:g},{edges[i+1]:g})={buckets[i]}'
            for i in range(len(buckets))))
        empty = [i for i, c in enumerate(buckets) if c == 0]
        return empty
    empty_z = hist(
        lambda r: r['axis_z'],
        (0.0, 0.2, 0.4, 0.7, 0.85, 1.01), 'axis_z 覆盖')
    empty_e = hist(
        lambda r: r['entry_norm_m'],
        (0.3, 0.7, 0.9, 1.02, 1.16), '|entry| 覆盖')
    empty_c = hist(
        lambda r: r['chord_entry_m'],
        (0.0, 0.35, 0.55, 0.70, 0.80), '拍照弦 覆盖')
    if empty_z or empty_e or empty_c:
        print('警告: 有空分箱，覆盖不足', file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--n', type=int, default=10000)
    parser.add_argument('--seed', type=int, default=20260911)
    args = parser.parse_args()
    book = load_casebook()
    templates = book['targets_20260909']
    photo_xyz = [float(v) for v in book.get(
        'photo_tcp_xyz', [0.302, -0.232, 0.708])]
    photo_quat = tuple(float(v) for v in book.get(
        'photo_tcp_xyzw', [-0.005373, -0.009976, -0.037437, 0.999235]))
    photo_rot = _quat_to_mat(photo_quat)
    box = _workspace(templates, photo_xyz)
    cases = sample_stratified(
        args.n, args.seed, templates, photo_xyz, box)
    out_path = RESULTS_DIR / (
        f'analyze_approach_envelope_{time.strftime("%Y%m%d_%H%M%S")}.jsonl')
    rows = []
    t0 = time.time()
    with open(out_path, 'a', encoding='utf-8') as sink:
        for i, (cid, case) in enumerate(cases.items(), 1):
            rec = evaluate(case, photo_xyz, photo_rot, photo_quat)
            rec['case'] = cid
            rows.append(rec)
            sink.write(json.dumps(rec, ensure_ascii=False) + '\n')
            if i % 2000 == 0:
                print(f'… {i}/{len(cases)}', flush=True)
    print(f'用时 {time.time() - t0:.2f}s；写入 {out_path}（{len(rows)} 行，'
          f'含 field {sum(1 for r in rows if r["stratum"]=="field")}）')
    _summarize(rows)
    print('未观测（需规划/执臂）: joint_travel, ptp_cartesian_detour, '
          'ik_self_collision')
    return 0


if __name__ == '__main__':
    sys.exit(main())
