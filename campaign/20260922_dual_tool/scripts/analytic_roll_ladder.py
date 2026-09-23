#!/usr/bin/env python3
"""PTP+垂直进入设计的解析法评估（用户裁定：仿真链太慢，改解析迭代）。

对网格 20 例 + 随机包络 N 例逐位姿算：
  staging = 预抓取沿 −axis 退 standoff（keep-roll/±30°/±60° 滚转梯子）
  预抓取 = 入口沿 −axis 退 along_axis_m
两关键路点各查 /compute_ik（move_group 需在跑；不起周期、不 MTC、秒级），
再查预抓取→入口的沿轴 LIN 可达性代理（末端点 IK）。输出逐滚转成功率，
指导滚转梯子取舍。结果写 campaign/analysis/injection/analytic_roll_ladder.*.

用法: analytic_roll_ladder.py [--random 60] [--seed 20260922] [--envelope typical]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))

OUTER = ROOT / 'campaign/20260922_dual_tool/analysis/injection'

ALONG_AXIS_M = 0.03   # peach_arm.yaml approach_along_axis_m
STANDOFF_M = 0.10     # approach_staging_standoff_m
ROLLS_DEG = (0, 30, -30, 60, -60)
SEED_JITTER_RAD = 0.6   # 拍照位邻域随机重启（逃 KDL 局部盆）
SEED_TRIES_PER_ROLL = 4  # 每滚转 1 拍照位 + 4 抖动种子（≈v1 的 5 种子/滚转）


def quat_from_axis_z(axis, roll_deg=0.0):
    """工具 Z 对齐 axis 的姿态（绕 axis 自转=滚转）。z-x-y 欧拉近似：
    R = Rz(yaw)·Rx(pitch) 把 +Z 转到 axis，再右乘 Rz(roll)。"""
    a = [v / math.sqrt(sum(v * v for v in axis)) for v in axis]
    yaw = math.atan2(a[1], a[0])
    pitch = math.acos(max(-1.0, min(1.0, a[2])))
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    # R = Rz(yaw)*Ry(-pitch)? 用标准：先绕Y -pitch 再绕Z yaw 使 +Z→axis
    import numpy as np
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    R = Rz @ Ry
    if roll_deg:
        r = math.radians(roll_deg)
        cr, sr = math.cos(r), math.sin(r)
        R = R @ np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]], dtype=float)
    return R


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--random', type=int, default=60)
    parser.add_argument('--seed', type=int, default=20260922)
    parser.add_argument('--envelope', choices=('typical', 'algorithm'),
                        default='typical')
    args = parser.parse_args()

    import rclpy
    from moveit_msgs.srv import GetPositionIK
    from moveit_msgs.msg import (
        PositionIKRequest, RobotState, Constraints, OrientationConstraint)
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose, PoseStamped
    from sensor_msgs.msg import JointState

    import sim_field_targets as sim

    grid = sim.load_grid_cases()
    book = sim.load_casebook()
    photo_tcp = sim._photo_tcp(book)
    rnd = sim.sample_random_cases(
        book['targets_20260909'], args.random, args.seed, photo_tcp,
        args.envelope)
    cases = {**grid, **rnd}

    rclpy.init()
    node = rclpy.create_node('analytic_roll_ladder')
    cli = node.create_client(GetPositionIK, '/compute_ik')
    if not cli.wait_for_service(timeout_sec=15.0):
        print('/compute_ik 不可用（move_group 未起？）', file=sys.stderr)
        return 1
    # call_async 的响应要 executor 在转才会回（rclpy 陷阱），后台 spin
    import threading
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    # 种子=拍照位关节（真实链从拍照位 PTP 出发，IK 种子同源）
    JOINT_ORDER = sim.JOINT_ORDER
    photo = sim.load_casebook()['photo_joints']
    seed_home = [float(photo[n]) for n in JOINT_ORDER]
    import random as _random

    def make_seed(jitter: bool) -> RobotState:
        st = RobotState()
        st.joint_state.name = list(JOINT_ORDER)
        if jitter:
            st.joint_state.position = [
                v + _random.gauss(0.0, SEED_JITTER_RAD) for v in seed_home]
        else:
            st.joint_state.position = list(seed_home)
        return st

    import numpy as np

    def ik_ok(pos, R, seed: RobotState) -> bool:
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'manipulator_e5'
        p = req.ik_request.pose_stamped
        p.header.frame_id = 'world'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = pos
        Rv = R
        tr = math.sqrt(max(0.0, 1 + Rv[0][0] + Rv[1][1] + Rv[2][2]))
        q = Pose()
        if tr > 1e-9:
            s = 0.5 / tr
            q.orientation.w = 0.5 * tr
            q.orientation.x = (Rv[2][1] - Rv[1][2]) * s
            q.orientation.y = (Rv[0][2] - Rv[2][0]) * s
            q.orientation.z = (Rv[1][0] - Rv[0][1]) * s
        else:
            q.orientation.w, q.orientation.x = 0.0, 1.0
        p.pose.orientation = q.orientation
        req.ik_request.robot_state = seed
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 200_000_000
        fut = cli.call_async(req)
        deadline = time.time() + 2.0
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        if not fut.done():
            return False
        return fut.result().error_code.val == 1

    per_roll = {r: 0 for r in ROLLS_DEG}
    both_any = 0
    rows = []
    for cid, case in cases.items():
        entry = case['entry_xyz']
        axis = case['axis']
        pregrasp = [entry[i] - axis[i] * ALONG_AXIS_M for i in range(3)]
        staging = [pregrasp[i] - axis[i] * STANDOFF_M for i in range(3)]
        hit_any = False
        first_roll = None
        for roll in ROLLS_DEG:
            R = quat_from_axis_z(axis, roll)
            ok = False
            for try_i in range(SEED_TRIES_PER_ROLL + 1):
                if ik_ok(staging, R, make_seed(try_i > 0)) and ik_ok(
                    pregrasp, R, make_seed(try_i > 0)):
                    ok = True
                    break
            if ok:
                per_roll[roll] += 1
                if not hit_any:
                    first_roll = roll
                hit_any = True
        if hit_any:
            both_any += 1
        rows.append({'case': cid, 'kind': 'grid' if cid in grid else 'random',
                     'ok': hit_any, 'first_roll_deg': first_roll})

    n = len(rows)
    summary = {
        'n': n,
        'any_roll_success': both_any,
        'any_roll_rate': round(both_any / n, 4) if n else 0,
        'per_roll_success': {str(r): per_roll[r] for r in ROLLS_DEG},
        'grid_ok': sum(1 for r in rows if r['kind'] == 'grid' and r['ok']),
        'grid_n': sum(1 for r in rows if r['kind'] == 'grid'),
        'random_ok': sum(1 for r in rows if r['kind'] == 'random' and r['ok']),
        'random_n': sum(1 for r in rows if r['kind'] == 'random'),
        'params': {'along_axis_m': ALONG_AXIS_M, 'standoff_m': STANDOFF_M,
                   'rolls_deg': ROLLS_DEG, 'envelope': args.envelope,
                   'seed': args.seed},
    }
    OUTER.mkdir(parents=True, exist_ok=True)
    (OUTER / 'analytic_roll_ladder.json').write_text(
        json.dumps({'summary': summary, 'rows': rows}, ensure_ascii=False,
                   indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
