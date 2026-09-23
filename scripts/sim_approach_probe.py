#!/usr/bin/env python3
"""诊断：接近段失败根因——目标位姿 IK 可达性 vs 沿弦 slerp 可行性。

注意：本脚本的 staging/折线几何为历史 v1 口径（2026-09-23 前的
果平面折线+staging PTP 候选链）；现行接近为 v4（PTP+垂直入冠+沿轴），
结果不再对照现行系统，仅作历史基线复盘。

离线复刻 ``sim_field_targets.py`` 的随机位姿（同 N 同 seed 确定性），
对每个位姿量化四件事：
1. 姿态翻转角：拍照位工具 Z 与袋轴夹角（LIN 到 G 必须 slerp 掉的姿态量）；
2. 目标可达性：G 点（袋底平面外沿 r=keepout）、轴上预抓取、预抓取正下方
   staging（沿轴再退 approach_staging_standoff_m）的逐滚转 /compute_ik
   （拍照位种子，avoid_collisions）；
3. 沿弦可行性：拍照位→G 单路点 /compute_cartesian_path 的 fraction
   （与 Pilz LIN 同类约束：沿弦逐步 IK、同构型延续）；
4. 设计形状核验：staging→预抓取的轴向直线 fraction。

目标 IK 全过而 fraction≈0 ⇒ 瓶颈在「边走边转」的弦中段；目标 IK 也不过
⇒ 位姿本身无解（工作空间边缘，滚转救不了）。纯只读诊断，不动臂。
不进 colcon test。

用法：
  python3 scripts/sim_approach_probe.py --random 100 --seed 20260910
  python3 scripts/sim_approach_probe.py --random 16 --seed 20260910 --rolls 0 90
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_field_targets import (  # noqa: E402
    KEEP_R_M, load_casebook, sample_random_cases)

JOINT_ORDER = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)
STANDOFF_M = 0.03  # grasp_standoffs.yaml pregrasp_standoff_m
STAGING_GAP_M = 0.10  # peach_arm.yaml approach_staging_standoff_m
DEFAULT_ROLLS = (0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180)
RESULTS_DIR = Path(__file__).resolve().parents[1] / 'runs'


def _norm(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


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


def taut_g(start_xyz, entry, axis, radius_m=KEEP_R_M):
    """grasp_geometry.hpp tautGPosition 的 Python 复刻（起点=拍照位 TCP）。"""
    delta = [start_xyz[i] - entry[i] for i in range(3)]
    axial = sum(delta[i] * axis[i] for i in range(3))
    radial = [delta[i] - axial * axis[i] for i in range(3)]
    rn = math.sqrt(sum(v * v for v in radial))
    if rn < 1e-4:
        ref = (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0)
        radial = (
            axis[1] * ref[2] - axis[2] * ref[1],
            axis[2] * ref[0] - axis[0] * ref[2],
            axis[0] * ref[1] - axis[1] * ref[0])
        rn = math.sqrt(sum(v * v for v in radial))
        if rn < 1e-9:
            return None
    unit = [v / rn for v in radial]
    return [entry[i] - 0.01 * axis[i] + radius_m * unit[i] for i in range(3)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--random', type=int, default=16, metavar='N')
    parser.add_argument('--seed', type=int, default=20260910)
    parser.add_argument(
        '--rolls', type=int, nargs='+', default=list(DEFAULT_ROLLS),
        help='要逐个探测的刀口滚转（度）')
    args = parser.parse_args()

    book = load_casebook()
    templates = book['targets_20260909']
    photo_tcp = [float(v) for v in book.get(
        'photo_tcp_xyz', [0.302, -0.232, 0.708])]
    photo_joints = [float(book['photo_joints'][n]) for n in JOINT_ORDER]
    cases = sample_random_cases(templates, args.random, args.seed, photo_tcp)

    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node('sim_approach_probe')
    fk_cli = node.create_client(GetPositionFK, '/compute_fk')
    ik_cli = node.create_client(GetPositionIK, '/compute_ik')
    cart_cli = node.create_client(GetCartesianPath, '/compute_cartesian_path')
    for cli in (fk_cli, ik_cli, cart_cli):
        if not cli.wait_for_service(timeout_sec=10.0):
            print(f'服务不可用: {cli.srv_name}', file=sys.stderr)
            return 2

    def spin(fut, timeout=30.0):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok() and not fut.done():
            rclpy.spin_once(node, timeout_sec=0.02)
        return fut.result() if fut.done() else None

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
    photo_xyz = photo[:3]
    photo_rot = _quat_to_mat(photo[3:])
    tool_z = (photo_rot[0][2], photo_rot[1][2], photo_rot[2][2])
    print(f'photo tcp=({photo_xyz[0]:.3f},{photo_xyz[1]:.3f},'
          f'{photo_xyz[2]:.3f}) '
          f'tool_z=({tool_z[0]:.2f},{tool_z[1]:.2f},{tool_z[2]:.2f})')

    def ik_probe(xyz, rot, want_joints=False):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'manipulator_e5'
        req.ik_request.ik_link_name = 'tcp'
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 100_000_000
        req.ik_request.avoid_collisions = True
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        q = _mat_to_quat(rot)
        pose.orientation.x, pose.orientation.y = q[0], q[1]
        pose.orientation.z, pose.orientation.w = q[2], q[3]
        req.ik_request.pose_stamped.header.frame_id = 'base_link'
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.robot_state.joint_state.name = list(JOINT_ORDER)
        req.ik_request.robot_state.joint_state.position = list(photo_joints)
        res = spin(ik_cli.call_async(req), 5.0)
        ok = res is not None and res.error_code.val == 1
        if want_joints and ok:
            return list(res.solution.joint_state.position)
        return ok

    def cart_fraction(goal_xyz, goal_rot, start_joints=None):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.start_state = RobotState()
        req.start_state.joint_state.name = list(JOINT_ORDER)
        req.start_state.joint_state.position = list(
            photo_joints if start_joints is None else start_joints)
        req.group_name = 'manipulator_e5'
        req.link_name = 'tcp'
        req.waypoints = [Pose()]
        req.waypoints[0].position.x = goal_xyz[0]
        req.waypoints[0].position.y = goal_xyz[1]
        req.waypoints[0].position.z = goal_xyz[2]
        q = _mat_to_quat(goal_rot)
        req.waypoints[0].orientation.x, req.waypoints[0].orientation.y = q[0], q[1]
        req.waypoints[0].orientation.z, req.waypoints[0].orientation.w = q[2], q[3]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        res = spin(cart_cli.call_async(req), 30.0)
        return None if res is None else float(res.fraction)

    out_path = RESULTS_DIR / (
        f'sim_approach_probe_{time.strftime("%Y%m%d_%H%M%S")}.jsonl')
    records = []
    with open(out_path, 'a', encoding='utf-8') as sink:
        for cid, case in cases.items():
            entry = case['entry_xyz']
            axis = _norm(case['axis'])
            flip_deg = _angle_deg(photo_rot[2], axis)
            aligned = _align_z(photo_rot, axis)
            g = taut_g(photo_xyz, entry, axis)
            pregrasp = [entry[i] - STANDOFF_M * axis[i] for i in range(3)]
            staging = [
                entry[i] - (STANDOFF_M + STAGING_GAP_M) * axis[i]
                for i in range(3)]
            record = {
                'case': cid, 'entry_xyz': entry, 'axis': case['axis'],
                'flip_deg': round(flip_deg, 1),
                'axis_z': round(axis[2], 2),
                'g_xyz': [round(v, 3) for v in g] if g else None,
                'entry_norm_m': round(math.dist(entry, (0, 0, 0)), 3),
                'staging_xyz': [round(v, 3) for v in staging],
                'staging_norm_m': round(math.dist(staging, (0, 0, 0)), 3),
                'ik_g': {}, 'ik_pregrasp': {}, 'ik_staging': {},
                'cart_g_fraction': {},
                'cart_staging_to_pregrasp': None,
            }
            for deg in args.rolls:
                rot = _roll_z(aligned, deg)
                if g:
                    record['ik_g'][deg] = ik_probe(g, rot)
                record['ik_pregrasp'][deg] = ik_probe(pregrasp, rot)
                record['ik_staging'][deg] = ik_probe(staging, rot)
            if g:
                for deg in (0, 60, -60, 120):
                    record['cart_g_fraction'][deg] = cart_fraction(
                        g, _roll_z(aligned, deg))
            # 设计形状核验：先解 staging（roll=0）IK 关节解，再以它为起点
            # 算 staging→预抓取的轴向直线 fraction（起点姿态已对齐）。
            staging_joints = ik_probe(staging, aligned, want_joints=True)
            record['cart_staging_to_pregrasp'] = (
                cart_fraction(pregrasp, aligned, staging_joints)
                if isinstance(staging_joints, list) else None)
            ok_g = sum(1 for v in record['ik_g'].values() if v)
            ok_p = sum(1 for v in record['ik_pregrasp'].values() if v)
            ok_s = sum(1 for v in record['ik_staging'].values() if v)
            best_frac = max(
                (v for v in record['cart_g_fraction'].values()
                 if v is not None), default=None)
            ax_frac = record['cart_staging_to_pregrasp']
            print(f"{cid}: flip={flip_deg:5.1f}° axis_z={axis[2]:.2f} "
                  f"|entry|={record['entry_norm_m']:.2f} "
                  f"IK_G={ok_g}/{len(args.rolls)} "
                  f"IK_pre={ok_p}/{len(args.rolls)} "
                  f"IK_stg={ok_s}/{len(args.rolls)} "
                  f"cartG_frac_best="
                  f"{best_frac if best_frac is None else round(best_frac, 2)} "
                  f"axialLIN="
                  f"{ax_frac if ax_frac is None else round(ax_frac, 2)}",
                  flush=True)
            sink.write(json.dumps(record, ensure_ascii=False) + '\n')
            sink.flush()
    print(f'探测结果已写入 {out_path}')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
