#!/usr/bin/env python3
"""Mock 轨迹回放：用真机过程坐标当抓取位姿，按 MoveIt2 官方抓取管线规划。

对齐（不是本地自造绕路）：
- MTC GenerateGraspPose：绕接近轴按 angle_delta 采样刀口滚转
  https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/
- Fallbacks：先 Pilz LIN（工业笛卡尔直线），再 CartesianPath 插值
  https://github.com/moveit/moveit_task_constructor/blob/master/demo/src/fallbacks_move_to.cpp
  https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/
- 接触段不用 OMPL / PTP（OMPL Connect 会绕障抬腕，即 1740）

须已起 ``harvest_system.launch.py hardware_mode:=mock camera_enabled:=false``。
不进 colcon test。``--planner ptp`` 只作绕行对照。
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml

JOINT_ORDER = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)
GROUP = 'manipulator_e5'
TIP = 'tcp'
BASE = 'base_link'
DETOUR_LIMITS = (2.2, 0.25, 0.08)  # ratio, chord_dev_m, recede_m
# GenerateGraspPose 默认 angle_delta=π/12；这里 30° 并优先小角。
TOOL_ROLLS_DEG = (0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180)

CASES_PATH = (
    Path(__file__).resolve().parents[1] /
    'src/peach_arm/config/field_pregrasp_cases.yaml')


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
        s = math.sqrt(t + 1.0) * 2.0
        return (
            float((m[2][1] - m[1][2]) / s), float((m[0][2] - m[2][0]) / s),
            float((m[1][0] - m[0][1]) / s), float(0.25 * s))
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return (
            float(0.25 * s), float((m[0][1] + m[1][0]) / s),
            float((m[0][2] + m[2][0]) / s), float((m[2][1] - m[1][2]) / s))
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return (
            float((m[0][1] + m[1][0]) / s), float(0.25 * s),
            float((m[1][2] + m[2][1]) / s), float((m[0][2] - m[2][0]) / s))
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return (
        float((m[0][2] + m[2][0]) / s), float((m[1][2] + m[2][1]) / s),
        float(0.25 * s), float((m[1][0] - m[0][1]) / s))


def _matmul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3))


def _align_frame_z(current_xyzw, axis):
    rot = _quat_to_mat(current_xyzw)
    z = (rot[0][2], rot[1][2], rot[2][2])
    an = math.sqrt(sum(v * v for v in axis))
    a = tuple(v / an for v in axis)
    zn = math.sqrt(sum(v * v for v in z))
    z = tuple(v / zn for v in z)
    c = max(-1.0, min(1.0, z[0] * a[0] + z[1] * a[1] + z[2] * a[2]))
    v = (
        z[1] * a[2] - z[2] * a[1],
        z[2] * a[0] - z[0] * a[2],
        z[0] * a[1] - z[1] * a[0],
    )
    vn = math.sqrt(sum(x * x for x in v))
    if vn < 1e-9:
        return current_xyzw if c > 0 else (1.0, 0.0, 0.0, 0.0)
    angle = math.acos(c)
    axis_u = tuple(x / vn for x in v)
    half = angle / 2.0
    s = math.sin(half)
    dq = (axis_u[0] * s, axis_u[1] * s, axis_u[2] * s, math.cos(half))
    return _mat_to_quat(_matmul(_quat_to_mat(dq), rot))


def _roll_about_tool_z(xyzw, deg):
    half = math.radians(float(deg)) / 2.0
    rz = (0.0, 0.0, math.sin(half), math.cos(half))
    return _mat_to_quat(_matmul(_quat_to_mat(xyzw), _quat_to_mat(rz)))


def _dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _point_to_segment(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]
    length2 = sum(v * v for v in ab)
    if length2 < 1e-16:
        return _dist(p, a)
    t = sum((p[i] - a[i]) * ab[i] for i in range(3)) / length2
    t = max(0.0, min(1.0, t))
    closest = [a[i] + t * ab[i] for i in range(3)]
    return _dist(p, closest)


def inspect_detour(points):
    if len(points) < 2:
        return {
            'path_m': 0.0, 'chord_m': 0.0, 'ratio': 0.0,
            'max_dev_m': 0.0, 'max_recede_m': 0.0, 'allowed': True,
            'reason': '点列不足'}
    start, goal = points[0], points[-1]
    chord = _dist(start, goal)
    path = 0.0
    max_dev = 0.0
    max_recede = 0.0
    for i in range(1, len(points)):
        path += _dist(points[i - 1], points[i])
        max_dev = max(max_dev, _point_to_segment(points[i], start, goal))
        max_recede = max(max_recede, max(0.0, _dist(points[i], goal) - chord))
    ratio = path / chord if chord >= 0.02 else 0.0
    reason = '笛卡尔短路径门通过'
    allowed = True
    lim_ratio, lim_dev, lim_recede = DETOUR_LIMITS
    if max_recede > lim_recede:
        allowed, reason = False, f'TCP 回退 {max_recede:.3f}m > {lim_recede}m'
    elif max_dev > lim_dev:
        allowed, reason = False, f'TCP 相对弦偏离 {max_dev:.3f}m > {lim_dev}m'
    elif chord >= 0.02 and ratio > lim_ratio:
        allowed, reason = False, f'TCP 绕行比 {ratio:.2f} > {lim_ratio}'
    return {
        'path_m': path, 'chord_m': chord, 'ratio': ratio,
        'max_dev_m': max_dev, 'max_recede_m': max_recede,
        'allowed': allowed, 'reason': reason}


def _joint_state(names, positions):
    from sensor_msgs.msg import JointState
    msg = JointState()
    msg.name = list(names)
    msg.position = [float(p) for p in positions]
    return msg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', default='1757', choices=['1757', '1740'])
    parser.add_argument('--planner', default='lin', choices=['lin', 'ptp'])
    parser.add_argument('--execute', action='store_true',
                        help='规划过护栏后才下发；默认只规划')
    parser.add_argument('--vel', type=float, default=0.10)
    args = parser.parse_args()

    import rclpy
    from geometry_msgs.msg import Pose, Quaternion
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import (
        Constraints, JointConstraint, OrientationConstraint,
        PositionConstraint, RobotState,
    )
    from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from shape_msgs.msg import SolidPrimitive

    cases = yaml.safe_load(CASES_PATH.read_text())
    case = cases['cases'][args.case]
    photo = [float(cases['photo_joints'][n]) for n in JOINT_ORDER]
    print(f"case={args.case} {case['request_id']}:{case['target_id']}")
    print(f"note={case['note']}")
    print(f"field_detour_ratio={case['field_detour_ratio']}")

    rclpy.init()
    node = Node('replay_field_pregrasp')
    plan_cli = node.create_client(GetMotionPlan, '/plan_kinematic_path')
    fk_cli = node.create_client(GetPositionFK, '/compute_fk')
    cart_cli = node.create_client(GetCartesianPath, '/compute_cartesian_path')
    exec_cli = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
    if not plan_cli.wait_for_service(timeout_sec=15.0):
        print('无 /plan_kinematic_path：先起 mock harvest_system', file=sys.stderr)
        rclpy.shutdown()
        return 2
    fk_cli.wait_for_service(timeout_sec=15.0)
    cart_cli.wait_for_service(timeout_sec=15.0)

    def call_srv(cli, req, timeout=20.0):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        return fut.result()

    def call_fk(joint_traj):
        points = []
        for pt in joint_traj.points:
            req = GetPositionFK.Request()
            req.header.frame_id = BASE
            req.fk_link_names = [TIP]
            req.robot_state.joint_state = _joint_state(
                joint_traj.joint_names, pt.positions)
            res = call_srv(fk_cli, req, 5.0)
            if res is None or not res.pose_stamped:
                return []
            p = res.pose_stamped[0].pose.position
            points.append((float(p.x), float(p.y), float(p.z)))
        return points

    def pose_from_xyz_xyzw(xyz, xyzw):
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation = Quaternion(
            x=float(xyzw[0]), y=float(xyzw[1]),
            z=float(xyzw[2]), w=float(xyzw[3]))
        return pose

    def constraints_from_pose(pose: Pose) -> Constraints:
        cons = Constraints()
        pos = PositionConstraint()
        pos.header.frame_id = BASE
        pos.link_name = TIP
        pos.weight = 1.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]
        origin = Pose()
        origin.position = pose.position
        origin.orientation.w = 1.0
        pos.constraint_region.primitives.append(box)
        pos.constraint_region.primitive_poses.append(origin)
        cons.position_constraints.append(pos)
        ori = OrientationConstraint()
        ori.header.frame_id = BASE
        ori.link_name = TIP
        ori.orientation = pose.orientation
        ori.absolute_x_axis_tolerance = 0.05
        ori.absolute_y_axis_tolerance = 0.05
        ori.absolute_z_axis_tolerance = 0.05
        ori.weight = 1.0
        cons.orientation_constraints.append(ori)
        return cons

    def plan_named_joints(start_is_diff, start_positions, goal_positions, planner):
        req = GetMotionPlan.Request()
        r = req.motion_plan_request
        r.group_name = GROUP
        r.pipeline_id = 'pilz_industrial_motion_planner'
        r.planner_id = planner
        r.num_planning_attempts = 1
        r.allowed_planning_time = 2.0
        r.max_velocity_scaling_factor = args.vel
        r.max_acceleration_scaling_factor = args.vel
        if start_is_diff:
            r.start_state.is_diff = True
        else:
            r.start_state = RobotState()
            r.start_state.joint_state = _joint_state(
                JOINT_ORDER, start_positions)
        goal = Constraints()
        for name, value in zip(JOINT_ORDER, goal_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        r.goal_constraints.append(goal)
        return call_srv(plan_cli, req)

    def plan_pose(start_positions, pose, planner):
        req = GetMotionPlan.Request()
        r = req.motion_plan_request
        r.group_name = GROUP
        r.pipeline_id = 'pilz_industrial_motion_planner'
        r.planner_id = planner
        r.num_planning_attempts = 1
        r.allowed_planning_time = 2.0
        r.max_velocity_scaling_factor = args.vel
        r.max_acceleration_scaling_factor = args.vel
        r.start_state = RobotState()
        r.start_state.joint_state = _joint_state(JOINT_ORDER, start_positions)
        r.goal_constraints.append(constraints_from_pose(pose))
        return call_srv(plan_cli, req)

    def plan_ok(result):
        return (
            result is not None and
            result.motion_plan_response.error_code.val == 1)

    def end_joints(result):
        traj = result.motion_plan_response.trajectory.joint_trajectory
        last = traj.points[-1].positions
        return [float(last[traj.joint_names.index(n)]) for n in JOINT_ORDER]

    def plan_cartesian(start_positions, poses, max_step=0.01):
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE
        req.start_state = RobotState()
        req.start_state.joint_state = _joint_state(
            JOINT_ORDER, start_positions)
        req.group_name = GROUP
        req.link_name = TIP
        req.waypoints = list(poses)
        req.max_step = float(max_step)
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = args.vel
        req.max_acceleration_scaling_factor = args.vel
        return call_srv(cart_cli, req)

    def _fail_plan(label, result):
        if result is None:
            print(f'{label} 无响应', file=sys.stderr)
            return
        code = result.motion_plan_response.error_code.val
        print(f'{label} 规划失败 error_code={code}', file=sys.stderr)

    photo_plan = plan_named_joints(True, None, photo, 'PTP')
    if not plan_ok(photo_plan):
        _fail_plan('拍照位 PTP', photo_plan)
        rclpy.shutdown()
        return 3
    print('拍照位 PTP 规划成功')

    fk_req = GetPositionFK.Request()
    fk_req.header.frame_id = BASE
    fk_req.fk_link_names = [TIP]
    fk_req.robot_state.joint_state = _joint_state(JOINT_ORDER, photo)
    fk_res = call_srv(fk_cli, fk_req, 5.0)
    if fk_res is None or not fk_res.pose_stamped:
        print('拍照位 FK 失败', file=sys.stderr)
        rclpy.shutdown()
        return 3
    photo_pose = fk_res.pose_stamped[0].pose
    photo_xyz = (
        float(photo_pose.position.x),
        float(photo_pose.position.y),
        float(photo_pose.position.z))
    photo_q = (
        float(photo_pose.orientation.x), float(photo_pose.orientation.y),
        float(photo_pose.orientation.z), float(photo_pose.orientation.w))
    axis = [float(v) for v in case['axis']]
    goal_xyz = [float(v) for v in case['pregrasp_xyz']]
    aligned_q = _align_frame_z(photo_q, axis)
    print(
        f'goal xyz={goal_xyz} '
        f'photo_tcp=({photo_xyz[0]:.3f},{photo_xyz[1]:.3f},{photo_xyz[2]:.3f})')

    planned_parts = []
    used = None
    if args.planner == 'ptp':
        used = 'PTP'
        approach = plan_pose(
            photo, pose_from_xyz_xyzw(goal_xyz, aligned_q), 'PTP')
        if not plan_ok(approach):
            _fail_plan('接近 PTP', approach)
            rclpy.shutdown()
            return 4
        planned_parts.append(('approach_ptp', approach))
    else:
        # Alternatives(grasp rolls) → Fallbacks(Pilz LIN, CartesianPath)
        for deg in TOOL_ROLLS_DEG:
            q = _roll_about_tool_z(aligned_q, deg)
            aligned = pose_from_xyz_xyzw(photo_xyz, q)
            goal = pose_from_xyz_xyzw(goal_xyz, q)
            align = plan_pose(photo, aligned, 'LIN')
            if not plan_ok(align):
                continue
            translate = plan_pose(end_joints(align), goal, 'LIN')
            if not plan_ok(translate):
                continue
            used = f'LIN-align+LIN roll={deg}'
            planned_parts.append(('align', align))
            planned_parts.append(('translate', translate))
            print(f'Pilz LIN 通过（刀口滚转 {deg}°）')
            break
        if used is None:
            print('Pilz LIN 全部滚转失败，Fallbacks → CartesianPath',
                  file=sys.stderr)
            for deg in TOOL_ROLLS_DEG:
                q = _roll_about_tool_z(aligned_q, deg)
                goal = pose_from_xyz_xyzw(goal_xyz, q)
                cart = plan_cartesian(photo, [goal])
                frac = 0.0 if cart is None else float(cart.fraction)
                print(f'  cartesian roll={deg} fraction={frac:.3f}')
                if cart is None or frac < 0.95:
                    continue
                used = f'cartesian roll={deg}'

                class _Wrap:
                    pass
                wrap = _Wrap()
                wrap.motion_plan_response = _Wrap()
                wrap.motion_plan_response.trajectory = cart.solution
                wrap.motion_plan_response.error_code = cart.error_code
                planned_parts.append(('cartesian', wrap))
                break
        if used is None:
            print('官方 Fallbacks 未完成（LIN+CartesianPath）', file=sys.stderr)
            rclpy.shutdown()
            return 4

    traj_pts = []
    for _, planned in planned_parts:
        traj = planned.motion_plan_response.trajectory.joint_trajectory
        traj_pts.extend(call_fk(traj))
    report = inspect_detour(traj_pts)
    if not traj_pts:
        print(f'plan {used}: FK 点列空', file=sys.stderr)
        rclpy.shutdown()
        return 4
    zmin, zmax = min(p[2] for p in traj_pts), max(p[2] for p in traj_pts)
    print(
        f"plan {used}: points={len(traj_pts)} path={report['path_m']:.3f}m "
        f"chord={report['chord_m']:.3f}m ratio={report['ratio']:.3f} "
        f"dev={report['max_dev_m']:.3f}m recede={report['max_recede_m']:.3f}m "
        f"z={zmin:.3f}..{zmax:.3f}")
    print(f"guard allowed={report['allowed']} ({report['reason']})")
    if not report['allowed']:
        print('护栏拒绝，不下发', file=sys.stderr)
        rclpy.shutdown()
        return 5
    if not args.execute:
        print('只规划；过护栏。加 --execute 才下发 mock 控制器')
        rclpy.shutdown()
        return 0
    if not exec_cli.wait_for_server(timeout_sec=5.0):
        print('无 /execute_trajectory', file=sys.stderr)
        rclpy.shutdown()
        return 6
    for label, planned in [('photo_ptp', photo_plan), *planned_parts]:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = planned.motion_plan_response.trajectory
        send = exec_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
        gh = send.result()
        if gh is None or not gh.accepted:
            print(f'{label} 执行被拒', file=sys.stderr)
            rclpy.shutdown()
            return 7
        result_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(node, result_fut, timeout_sec=60.0)
        result = result_fut.result()
        ok = result is not None and result.result.error_code.val == 1
        print(f'{label} execute={"ok" if ok else "fail"}')
        if not ok:
            rclpy.shutdown()
            return 8
    print('mock 执行完成')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
