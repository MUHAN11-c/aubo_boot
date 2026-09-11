#!/usr/bin/env python3
"""只读可达性验证：对 analytic_constraints.py 的同一批位姿，在 staging
与预抓取两个关键路点按现行滚转表逐档查询 /compute_ik（不起周期、不下
发运动、不碰 ExecuteTarget——与 sim_approach_probe 同类的只读诊断），
再与约束解析判定合成「预计可达」结论。

可达口径（分层，诚实边界）：
 L1 staging IK    — PTP 目标关节解存在（限位即服务内置）；
 L2 pregrasp IK   — 轴向 LIN 终点解存在（同滚转）；
 L3 约束放行      — analytic_constraints 的 first_reject 为空（keepout/
                    绕行/姿态门/直弦代理）。
 「预计可达」= L1 ∧ L2 ∧ L3。Pilz LIN 的同构型连续性与 PTP 弧形状
 仍属仿真/真机范畴（解析不声称）；本脚本回答的是「路点解存在 + 约束
 不拒」。
随机源与采样器与 analytic_constraints 共享（同 seed 同位姿，可对照）。
用法：python3 scripts/analytic_reachability.py [--n 10000] [--seed 20260911]
不进 colcon test。"""
from __future__ import annotations

import argparse
import collections
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analytic_constraints import (  # noqa: E402
    PHOTO_TOOL_Z, PHOTO_XYZ, RESULTS_DIR, ROLLS_DEG, STAGING_GAP_M,
    STANDOFF_M, Lcg, _add, _dot, _norm, _scale, _sub, angle_deg,
    evaluate, load_sampler, systematic_poses)
from sim_field_targets import load_casebook  # noqa: E402

JOINT_ORDER = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def quat_to_mat(q):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def mat_to_quat(m):
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


def two_vectors_quat(z_from, z_to):
    a = _norm(z_from)
    b = _norm(z_to)
    c = max(-1.0, min(1.0, _dot(a, b)))
    axis = (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])
    n = math.sqrt(sum(x * x for x in axis))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0) if c > 0 else (1.0, 0.0, 0.0, 0.0)
    half = math.acos(c) / 2.0
    s = math.sin(half)
    u = _norm(axis)
    return (u[0] * s, u[1] * s, u[2] * s, math.cos(half))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--n', type=int, default=10000)
    parser.add_argument('--seed', type=int, default=20260911)
    parser.add_argument('--limit', type=int, default=0,
                        help='只跑前 N 例（调试用；0=全部）')
    args = parser.parse_args()

    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.srv import GetPositionFK, GetPositionIK
    from rclpy.node import Node

    rclpy.init()
    node = Node('analytic_reachability')
    fk_cli = node.create_client(GetPositionFK, '/compute_fk')
    ik_cli = node.create_client(GetPositionIK, '/compute_ik')
    for cli in (fk_cli, ik_cli):
        if not cli.wait_for_service(timeout_sec=10.0):
            print(f'服务不可用: {cli.srv_name}', file=sys.stderr)
            return 2

    def spin(fut, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok() and not fut.done():
            rclpy.spin_once(node, timeout_sec=0.001)
        return fut.result() if fut.done() else None

    book = load_casebook()
    photo_joints = [float(book['photo_joints'][n]) for n in JOINT_ORDER]

    def fk(joints):
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['tcp']
        req.robot_state.joint_state.name = list(JOINT_ORDER)
        req.robot_state.joint_state.position = list(joints)
        res = spin(fk_cli.call_async(req), 5.0)
        if not res or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose
        return (p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z,
                p.orientation.w)

    photo = fk(photo_joints)
    if not photo:
        print('拍照位 FK 失败', file=sys.stderr)
        return 2
    photo_xyz = photo[:3]
    photo_q = photo[3:]
    print(f'photo tcp=({photo_xyz[0]:.3f},{photo_xyz[1]:.3f},'
          f'{photo_xyz[2]:.3f})（FK 实时值）')

    def ik(xyz, quat):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'manipulator_e5'
        req.ik_request.ik_link_name = 'tcp'
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 50_000_000
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped.header.frame_id = 'base_link'
        req.ik_request.pose_stamped.pose.position.x = xyz[0]
        req.ik_request.pose_stamped.pose.position.y = xyz[1]
        req.ik_request.pose_stamped.pose.position.z = xyz[2]
        q = req.ik_request.pose_stamped.pose.orientation
        q.x, q.y, q.z, q.w = quat
        req.ik_request.robot_state.joint_state.name = list(JOINT_ORDER)
        req.ik_request.robot_state.joint_state.position = list(photo_joints)
        res = spin(ik_cli.call_async(req), 5.0)
        return res is not None and res.error_code.val == 1

    rng = Lcg(args.seed)
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
    poses.extend(systematic_poses())
    if args.limit > 0:
        poses = poses[:args.limit]
    total = len(poses)
    print(f'采样 {total} 位姿（与 analytic_constraints 同 seed 同批）')

    # 目标姿态：align(photo_R, axis)（keep-roll 基准）再逐档 Rz(roll)。
    def roll_quats(axis):
        base = two_vectors_quat(_norm(PHOTO_TOOL_Z), axis)
        out = []
        for roll_deg in ROLLS_DEG:
            half = math.radians(roll_deg) / 2.0
            rz = (0.0, 0.0, math.sin(half), math.cos(half))
            out.append((roll_deg, quat_mul(base, rz)))
        return out

    t0 = time.time()
    recs = []
    for idx, pose in enumerate(poses):
        entry, axis = pose['entry'], _norm(pose['axis'])
        pregrasp = _sub(entry, _scale(axis, STANDOFF_M))
        staging = _sub(pregrasp, _scale(axis, STAGING_GAP_M))
        quats = roll_quats(axis)

        # staging 全档查询；pregrasp 只对 staging 可行档复核，取到首个
        # 双可行档即不再查后续档（难例才付满额调用代价）。
        staging_ok_rolls = []
        pregrasp_ok_rolls = []
        for roll_deg, q in quats:
            if not ik(staging, q):
                continue
            staging_ok_rolls.append(roll_deg)
            if not pregrasp_ok_rolls and ik(pregrasp, q):
                pregrasp_ok_rolls.append(roll_deg)

        con = evaluate(dict(
            entry=entry, axis=axis, neck=pose['neck'], length=pose['length'],
            suggested=pose['suggested'], origin=pose['origin']))
        rec = dict(
            origin=pose['origin'], axis_z=round(axis[2], 3),
            entry_norm=round(math.sqrt(sum(v * v for v in entry)), 3),
            theta_deg=con['theta_deg'],
            staging_ik=bool(staging_ok_rolls),
            pregrasp_ik=bool(pregrasp_ok_rolls),
            n_staging_rolls=len(staging_ok_rolls),
            constraint_ok=con['first_reject'] is None,
            first_reject=con['first_reject'], kind=con['kind'],
        )
        rec['reachable'] = (rec['staging_ik'] and rec['pregrasp_ik']
                            and rec['constraint_ok'])
        recs.append(rec)
        if (idx + 1) % 500 == 0:
            rate = sum(1 for r in recs if r['reachable']) / len(recs)
            print(f'  {idx + 1}/{total} 预计可达率 {rate:.0%} '
                  f'({time.time() - t0:.0f}s)', flush=True)

    out_path = RESULTS_DIR / (
        f'analytic_reachability_{time.strftime("%Y%m%d_%H%M%S")}.jsonl')
    with open(out_path, 'a', encoding='utf-8') as sink:
        for r in recs:
            sink.write(json.dumps(r, ensure_ascii=False) + '\n')

    print(f'\n总预计可达: {sum(1 for r in recs if r["reachable"])}/{total}')
    for origin in ('field', 'extended', 'grid'):
        sub = [r for r in recs if r['origin'] == origin]
        if not sub:
            continue
        reach = sum(1 for r in sub if r['reachable'])
        l1 = sum(1 for r in sub if r['staging_ik'])
        l2 = sum(1 for r in sub if r['pregrasp_ik'])
        l3 = sum(1 for r in sub if r['constraint_ok'])
        print(f'{origin}: n={len(sub)} | 可达 {reach} ({reach/len(sub):.0%}) '
              f'| staging IK {l1} ({l1/len(sub):.0%}) '
              f'| pregrasp IK {l2} ({l2/len(sub):.0%}) '
              f'| 约束放行 {l3} ({l3/len(sub):.0%})')
    # 不可达归因（按首要卡点）
    cause = collections.Counter()
    for r in recs:
        if r['reachable']:
            continue
        if not r['staging_ik']:
            cause['staging 无 IK'] += 1
        elif not r['pregrasp_ik']:
            cause['pregrasp 无 IK'] += 1
        else:
            cause['约束拒发: ' + str(r['first_reject'])] += 1
    print('不可达归因:', dict(cause.most_common()))
    # |entry| 分桶
    print('按 |entry| 分桶可达率:')
    for lo in (0.80, 0.90, 1.00, 1.05, 1.10):
        sub = [r for r in recs if lo <= r['entry_norm'] < lo + 0.05]
        if sub:
            reach = sum(1 for r in sub if r['reachable'])
            print(f'  |entry|∈[{lo:.2f},{lo + 0.05:.2f}): '
                  f'{reach}/{len(sub)} ({reach/len(sub):.0%})')
    print(f'结果已写入 {out_path}')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
