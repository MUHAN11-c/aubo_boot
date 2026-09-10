#!/usr/bin/env python3
"""mock 全链路回放：注入真机过程记录的目标几何，驱动真实 peach_manipulation 管线。

与 ``replay_field_pregrasp.py``（Python 侧复刻 MoveIt 官方管线）不同，本脚本
走**真实 C++ 节点**：向感知/重建话题注入 runs/ 账本记录的 entry/axis/底/颈几何，
再对 ``/peach_manipulation_node/execute_target`` 发 ``PREGRASP_ONLY`` 周期
（skip_observation，跳过扫描段），验证阶段执行器的完整接近轨迹设计——
classifyApproach 分档 → 刀口滚转 → Pilz LIN → CIRC/staging → CartesianPath
→ 预抓取停位验证（tool_axis/sleeve_mouth/cutting_plane TF 残差门）。

须已起 ``harvest_system.launch.py hardware_mode:=mock camera_enabled:=false``
（RViz2 随栈启动；lifecycle 拉齐全部节点）。mock 无相机与 io 控制器，本脚本
补发：``/aubo_io_controller/robot_status``（仿真就绪态）与
camera_link→camera_depth_optical_frame 静态 TF。仅仿真使用，不碰真机。

不进 colcon test。

用法：
  python3 scripts/sim_field_targets.py --list
  python3 scripts/sim_field_targets.py --case all      # targets_20260909 全部
  python3 scripts/sim_field_targets.py --case 1437_0 1639_1
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

CASES_PATH = (
    Path(__file__).resolve().parents[1] /
    'src/peach_manipulation/config/field_pregrasp_cases.yaml')
NODE = 'peach_manipulation_node'
RESULTS_DIR = Path(__file__).resolve().parents[1] / 'runs'
JOINT_ORDER = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)
PHOTO_JOINTS = (0.425083, 0.195177, 1.677740, 1.461739, -0.500161, 0.038621)
GOAL_TIMEOUT_S = 300.0


def _norm(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def _axis_from_quat(xyzw):
    x, y, z, w = xyzw
    return _norm((2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)))


def _optical_from_link_quat():
    # camera_link(REP-103: x前y左z上) → optical(z前x右y下) 的固定旋转四元数。
    # 列 = optical 基在 link 系像：x=(0,-1,0) y=(0,0,-1) z=(1,0,0)。
    m = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s, 0.25 * s)
    s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
    return (0.25 * s, (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)


def load_cases() -> dict:
    data = yaml.safe_load(CASES_PATH.read_text())
    return data['targets_20260909']


def ensure_enabled() -> None:
    """运行期改参（空闲态全量生效）：开执行与抓取两档，工具保持关。"""
    for key in ('execution.enabled', 'grasp.enabled'):
        cmd = ['ros2', 'param', 'set', f'/{NODE}', key, 'true']
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if out.returncode != 0:
            raise RuntimeError(f'ros2 param set {key} 失败: {out.stderr.strip()}')


def wait_active(timeout_s: float = 120.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        out = subprocess.run(
            ['ros2', 'lifecycle', 'get', f'/{NODE}'],
            capture_output=True, text=True, timeout=20)
        if 'active [3]' in out.stdout:
            return
        time.sleep(2.0)
    raise RuntimeError(f'{NODE} 未在 {timeout_s}s 内 Active')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--case', nargs='+', default=['all'],
        help='targets_20260909 用例 id；all=全部')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()

    cases = load_cases()
    if args.list:
        for cid, case in cases.items():
            e = case['entry_xyz']
            print(f"{cid}: entry=({e[0]:.3f},{e[1]:.3f},{e[2]:.3f}) "
                  f"axis_z={case['axis'][2]:.2f} run={case['run']}")
        return 0

    selected = list(cases) if args.case == ['all'] else args.case
    unknown = [c for c in selected if c not in cases]
    if unknown:
        print(f'未知用例: {unknown}', file=sys.stderr)
        return 2

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped, Vector3
    from peach_interfaces.action import ExecuteTarget
    from peach_interfaces.msg import (
        BagFitting, BagFittingArray, BagGrasp2D, BagGraspCandidate,
        BagGraspCandidateArray, GraspDecision, PeachTargetObservation,
        PeachTargetObservationArray, ReconstructionStatus,
    )
    from rclpy.action import ActionClient
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import JointState as JointState_
    from std_msgs.msg import Header
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster
    from builtin_interfaces.msg import Time

    rclpy.init()
    node = Node('sim_field_targets')

    latched = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    volatile = QoSProfile(depth=10)

    group = ReentrantCallbackGroup()
    obs_pub = node.create_publisher(
        PeachTargetObservationArray, '/peach/perception/target_observations', volatile)
    diag_pub = node.create_publisher(
        ReconstructionStatus, '/peach/reconstruction/diagnostics', latched)
    dec_pub = node.create_publisher(
        GraspDecision, '/peach/reconstruction/grasp_decision', latched)
    refined_pub = node.create_publisher(
        BagGraspCandidateArray, '/peach/reconstruction/refined_pose', latched)
    refit_pub = node.create_publisher(
        BagFittingArray, '/peach/reconstruction/refined_diagnostics', latched)
    rs_pub = node.create_publisher(
        _robot_status_type(), '/aubo_io_controller/robot_status', volatile)
    tf_broadcaster = StaticTransformBroadcaster(node)

    action_cli = ActionClient(node, ExecuteTarget, f'/{NODE}/execute_target')
    ack_cli = node.create_client(Trigger, f'/{NODE}/acknowledge_recovery')

    state = {'case': None, 'snapshot_id': 1000, 'capturing': False,
             'joint_samples': []}
    lock = threading.Lock()

    def header(frame='base_link'):
        h = Header()
        h.stamp = node.get_clock().now().to_msg()
        h.frame_id = frame
        return h

    def point(xyz):
        p = Point()
        p.x, p.y, p.z = (float(v) for v in xyz)
        return p

    def vector(xyz):
        v = Vector3()
        v.x, v.y, v.z = (float(v) for v in xyz)
        return v

    def set_case(case):
        with lock:
            state['case'] = case
            state['snapshot_id'] += 1

    def make_observation(case):
        axis = case['axis']
        # 观测入口姿态：Z=袋轴（与 CheckReachability/MovePregrasp 同一口径），
        # X 用世界 z 叉乘避免退化。
        z = _norm(axis)
        ref = (0.0, 0.0, 1.0) if abs(z[2]) < 0.9 else (1.0, 0.0, 0.0)
        x = _norm((ref[1] * z[2] - ref[2] * z[1], ref[2] * z[0] - ref[0] * z[2],
                   ref[0] * z[1] - ref[1] * z[0]))
        y = (z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2],
             z[0] * x[1] - z[1] * x[0])
        m = ((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2]))
        q = _mat_to_quat(m)

        obs = PeachTargetObservation()
        obs.header = header()
        obs.target_id = case['target_id']
        obs.priority = 1
        obs.confirmed = True
        obs.selected = True
        obs.harvest_status = 'SELECTED'
        obs.tracking_status = PeachTargetObservation.OBSERVED
        obs.camera_distance_m = 0.7
        obs.confidence = 0.9
        cand = BagGraspCandidate()
        cand.header = header()
        cand.target_id = case['target_id']
        cand.entry_pose.position = point(case['entry_xyz'])
        cand.entry_pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        cand.bag_bottom = point(case['bag_bottom'])
        cand.bag_neck = point(case['bag_neck'])
        cand.translation_direction = vector(case['axis'])
        cand.bag_diameter_upper_m = 0.06
        cand.suggested_travel_m = float(case.get('travel_m') or 0.0)
        cand.confidence = 0.9
        cand.status = BagGraspCandidate.ACCEPT
        obs.candidate = cand
        c2d = BagGrasp2D()
        c2d.target_id = case['target_id']
        c2d.bbox_x, c2d.bbox_y, c2d.bbox_w, c2d.bbox_h = 200, 150, 120, 160
        c2d.confidence = 0.9
        c2d.status = BagGrasp2D.ACCEPT
        obs.candidate_2d = c2d
        obs.fitting.foreground_ratio = 0.9
        obs.mask.width = 0  # 节点回退 640x480
        return obs

    def on_obs():
        with lock:
            case = state['case']
        if case is None:
            return
        msg = PeachTargetObservationArray()
        msg.header = header()
        with lock:
            state['snapshot_id'] += 1
            msg.snapshot_id = state['snapshot_id']
        msg.harvest_run_id = f"sim_{case_id(case)}"
        msg.target_set_locked = True
        msg.target_count = 1
        msg.selected_target_id = case['target_id']
        msg.collecting_count = 1
        msg.pending_count = 0
        obs = make_observation(case)
        obs.selected = True
        msg.observations.append(obs)
        obs_pub.publish(msg)

    def on_diag(_=None):
        with lock:
            case = state['case']
        if case is None:
            return
        msg = ReconstructionStatus()
        msg.header = header()
        msg.harvest_run_id = f"sim_{case_id(case)}"
        msg.selected_target_id = case['target_id']
        msg.state = 'READY'
        msg.target_id = case['target_id']
        msg.target_center_base = [
            0.5 * (case['bag_bottom'][i] + case['bag_neck'][i]) for i in range(3)]
        msg.captured_views = 3
        msg.valid_depth_ratio = 0.7
        msg.max_baseline_deg = 20.0
        msg.mean_nearest_baseline_deg = 12.0
        for direction in ((0.3, -0.9, 0.3), (0.5, -0.7, -0.2), (-0.2, -0.9, 0.4)):
            msg.view_directions.append(vector(direction))
        diag_pub.publish(msg)

    def on_refined(_=None):
        with lock:
            case = state['case']
        if case is None:
            return
        refined = BagGraspCandidate()
        refined.header = header()
        refined.target_id = case['target_id']
        refined.entry_pose.position = point(case['entry_xyz'])
        refined.bag_bottom = point(case['bag_bottom'])
        refined.bag_neck = point(case['bag_neck'])
        refined.translation_direction = vector(case['axis'])
        refined.bag_diameter_upper_m = 0.06
        refined.suggested_travel_m = float(case.get('travel_m') or 0.0)
        refined.confidence = 0.9
        refined.status = BagGraspCandidate.ACCEPT
        arr = BagGraspCandidateArray()
        arr.header = header()
        arr.candidates.append(refined)
        refined_pub.publish(arr)

        fit = BagFitting()
        fit.header = header()
        fit.target_id = case['target_id']
        fit.target_kind = 'bag'
        fit.cylinder_rms_m = 0.004
        fit.cylinder_inlier_ratio = 0.9
        fit.valid_depth_ratio = 0.7
        fit.foreground_ratio = 0.9
        fit.n_points = 2000
        fit.status = BagFitting.ACCEPT
        farr = BagFittingArray()
        farr.header = header()
        farr.fittings.append(fit)
        refit_pub.publish(farr)

    def on_decision(_=None):
        with lock:
            case = state['case']
        if case is None:
            return
        msg = GraspDecision()
        msg.header = header()
        msg.harvest_run_id = f"sim_{case_id(case)}"
        msg.target_id = case['target_id']
        msg.allowed = True
        msg.reason = 'sim_full_budget'
        dec_pub.publish(msg)

    def on_robot_status(_=None):
        msg = _robot_status_type()()
        msg.mode = 2
        msg.e_stopped = 0
        msg.drives_powered = 1
        msg.motion_possible = 1
        msg.in_motion = 0
        msg.in_error = 0
        rs_pub.publish(msg)

    node.create_timer(0.4, on_obs, callback_group=group)
    node.create_timer(0.5, on_diag, callback_group=group)
    node.create_timer(0.5, on_refined, callback_group=group)
    node.create_timer(0.5, on_decision, callback_group=group)
    node.create_timer(0.1, on_robot_status, callback_group=group)

    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id = 'camera_link'
    tf.child_frame_id = 'camera_depth_optical_frame'
    q = _optical_from_link_quat()
    tf.transform.rotation.x, tf.transform.rotation.y = q[0], q[1]
    tf.transform.rotation.z, tf.transform.rotation.w = q[2], q[3]
    tf_broadcaster.sendTransform(tf)

    # 实测绕行检测数据源：周期期间持续采样 /joint_states（事后 FK 成 TCP 点列）。
    # joint_state_broadcaster 的发布顺序不保证与 URDF 一致（mock 实测为字母序），
    # 必须按 name 映射，不得按位置截取。
    def on_joint_states(message):
        with lock:
            if not state['capturing'] or len(message.position) < 6:
                return
            by_name = dict(zip(message.name, message.position))
            values = [by_name.get(n) for n in JOINT_ORDER]
            if any(v is None for v in values):
                return
            state['joint_samples'].append(
                (time.monotonic(), [float(v) for v in values]))

    node.create_subscription(
        JointState_, '/joint_states', on_joint_states, 10)

    def measure_tcp_detour(samples):
        """采样点经 /compute_fk 成 TCP 点列，按 trajectory_guard 同口径算
        弦/路径/绕行比/相对弦偏离/回退。点列不足返回 None。"""
        from moveit_msgs.srv import GetPositionFK
        fk_cli = node.create_client(GetPositionFK, '/compute_fk')
        if not fk_cli.wait_for_service(timeout_sec=5.0):
            return None
        picked = []
        last_t = -1.0
        for stamp, positions in samples:
            if stamp - last_t >= 0.1 or last_t < 0:
                picked.append(positions)
                last_t = stamp

        def fk(positions):
            req = GetPositionFK.Request()
            req.header.frame_id = 'base_link'
            req.fk_link_names = ['tcp']
            req.robot_state.joint_state = JointState_()
            req.robot_state.joint_state.name = list(JOINT_ORDER)
            req.robot_state.joint_state.position = list(positions)
            fut = fk_cli.call_async(req)
            res = spin_until(fut, 5.0)
            if res is None or not res.pose_stamped:
                return None
            p = res.pose_stamped[0].pose.position
            return (float(p.x), float(p.y), float(p.z))

        points = [p for p in (fk(s) for s in picked) if p is not None]
        if len(points) < 2:
            return None
        start, goal = points[0], points[-1]
        chord = math.dist(start, goal)
        head = None
        if len(points) >= 4:
            head = points[3]  # 前 3 个点之后的位置，区分「起点残留」与途中绕行

        def seg_dist(p, a, b):
            ab = [b[i] - a[i] for i in range(3)]
            len2 = sum(v * v for v in ab)
            if len2 < 1e-16:
                return math.dist(p, a)
            t = max(0.0, min(1.0, sum(
                (p[i] - a[i]) * ab[i] for i in range(3)) / len2))
            return math.dist(p, [a[i] + t * ab[i] for i in range(3)])

        path = 0.0
        max_dev = 0.0
        max_recede = 0.0
        for i in range(1, len(points)):
            path += math.dist(points[i - 1], points[i])
            max_dev = max(max_dev, seg_dist(points[i], start, goal))
            max_recede = max(
                max_recede, max(0.0, math.dist(points[i], goal) - chord))
        ratio = path / chord if chord >= 0.02 else 0.0
        return {
            'points': len(points), 'path_m': round(path, 3),
            'chord_m': round(chord, 3), 'ratio': round(ratio, 2),
            'max_dev_m': round(max_dev, 3),
            'max_recede_m': round(max_recede, 3),
            'start_xyz': [round(v, 3) for v in start],
            'head_xyz': [round(v, 3) for v in head] if head else None,
            'end_xyz': [round(v, 3) for v in goal],
        }

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    def safe_spin():
        while rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception as error:
                print(f'注入回调异常: {error}', file=sys.stderr)

    spin = threading.Thread(target=safe_spin, daemon=True)
    spin.start()

    def case_id(case):
        return f"{case['run']}:{case['target_id']}"

    def spin_until(fut, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            if fut.done():
                return fut.result()
            time.sleep(0.05)
        return None

    def reset_photo_pose():
        """用例开始前把 mock 臂送回拍照位：直连 JTC 的两段关节插值
        （当前位→零位→拍照位）。Hold 构型回拍照位现场用示教器；MoveIt PTP
        在翻转构型下 IK 落错支，仿真用无 IK 的插值等价复位，不碰真机。"""
        try:
            from control_msgs.action import FollowJointTrajectory
            from rclpy.task import Future
            from sensor_msgs.msg import JointState
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

            states: list[JointState] = []

            def on_joint_state(message):
                if len(states) < 1:
                    states.append(message)

            js_sub = node.create_subscription(
                JointState, '/joint_states', on_joint_state, 10)
            deadline = time.time() + 5.0
            while not states and time.time() < deadline:
                time.sleep(0.05)
            node.destroy_subscription(js_sub)
            if not states:
                return False
            current = [float(v) for v in states[0].position][:6]
            if len(current) != len(JOINT_ORDER):
                return False

            def segment(start, end):
                points = []
                steps = 24
                for i in range(1, steps + 1):
                    t = i / steps
                    points.append(
                        [s + (e - s) * t for s, e in zip(start, end)])
                return points

            traj = JointTrajectory()
            traj.joint_names = list(JOINT_ORDER)
            start = current
            t_offset = 0.0
            for end in ([0.0] * len(JOINT_ORDER), list(PHOTO_JOINTS)):
                for values in segment(start, end):
                    t_offset += 0.12
                    point = JointTrajectoryPoint()
                    point.positions = values
                    point.time_from_start.sec = int(t_offset)
                    point.time_from_start.nanosec = int(
                        (t_offset - int(t_offset)) * 1e9)
                    traj.points.append(point)
                start = end

            jtc_cli = ActionClient(
                node, FollowJointTrajectory,
                '/joint_trajectory_controller/follow_joint_trajectory')
            if not jtc_cli.wait_for_server(timeout_sec=5.0):
                return False
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            send = jtc_cli.send_goal_async(goal)
            gh = spin_until(send, 10.0)
            if gh is None or not gh.accepted:
                return False
            result = spin_until(gh.get_result_async(), 90.0)
            return result is not None and result.status == 4  # STATUS_SUCCEEDED
        except Exception as error:
            print(f'  拍照位复位异常: {error}', file=sys.stderr)
            return False

    def run_case(cid, case):
        print(f'\n=== case {cid} entry=' +
              str([round(v, 3) for v in case['entry_xyz']]) +
              f" axis_z={case['axis'][2]:.2f} ===", flush=True)
        reset_ok = reset_photo_pose()
        if not reset_ok:
            print('  警告: 拍照位复位失败，本用例从当前位起跑', file=sys.stderr)
        set_case(case)
        time.sleep(1.5)  # 锁定集/精化锁存就位
        with lock:
            state['joint_samples'] = []
            state['capturing'] = True
        if not action_cli.wait_for_server(timeout_sec=10.0):
            return {'case': cid, 'error': 'execute_target action 不可用'}
        goal = ExecuteTarget.Goal()
        goal.request_id = f'sim_{cid}'
        goal.run_id = f'sim_{case["run"]}'
        goal.cycle_id = f'sim_{cid}:{case["target_id"]}'
        goal.target_id = case['target_id']
        goal.mode = ExecuteTarget.Goal.PREGRASP_ONLY
        goal.skip_observation = True
        t0 = time.time()
        send = action_cli.send_goal_async(goal)
        gh = spin_until(send, 15.0)
        if gh is None or not gh.accepted:
            return {'case': cid, 'error': 'goal 被拒'}
        result = spin_until(gh.get_result_async(), GOAL_TIMEOUT_S)
        elapsed = time.time() - t0
        with lock:
            state['capturing'] = False
            samples = list(state['joint_samples'])
        if result is None:
            return {'case': cid, 'error': 'goal 超时', 'elapsed_s': elapsed}
        res = result.result
        tcp = measure_tcp_detour(samples) if samples else None
        detour_detail = ''
        detour = False
        if tcp:
            # 转移级上限（2.5/0.40/0.15）为本管线最宽笛卡尔门；实测超它即绕行。
            over = []
            if tcp['chord_m'] >= 0.02 and tcp['ratio'] > 2.5:
                over.append(f"绕行比 {tcp['ratio']}>2.5")
            if tcp['max_dev_m'] > 0.40:
                over.append(f"弦偏离 {tcp['max_dev_m']}>0.40")
            if tcp['max_recede_m'] > 0.15:
                over.append(f"回退 {tcp['max_recede_m']}>0.15")
            detour = bool(over)
            detour_detail = '；'.join(over)
        out = {
            'case': cid,
            'run': case['run'],
            'target_id': case['target_id'],
            'entry_xyz': list(case['entry_xyz']),
            'axis': list(case['axis']),
            'reset_ok': reset_ok,
            'status': int(result.status),
            'outcome': int(res.outcome),
            'completion_level': int(res.completion_level),
            'failure_code': int(res.failure_code),
            'reason': res.reason,
            'recovery_required': bool(res.recovery_required),
            'pregrasp_passed': bool(res.pregrasp.passed),
            'pregrasp_axis_deg': float(res.pregrasp.axis_angle_deg),
            'pregrasp_lateral_m': float(res.pregrasp.lateral_error_m),
            'stage_names': list(res.stage_names),
            'stage_durations_s': [d.sec + d.nanosec / 1e9 for d in res.stage_durations],
            'elapsed_s': elapsed,
            'tcp_detour': tcp,
            'detour_flag': detour,
            'detour_detail': detour_detail,
        }
        ack_cli.wait_for_service(timeout_sec=5.0)
        ack = ack_cli.call_async(Trigger.Request())
        spin_until(ack, 10.0)
        return out

    wait_active()
    ensure_enabled()
    print(f'{NODE} Active；execution/grasp 已开（tool 保持关）')

    outcomes = []
    out_path = RESULTS_DIR / f'sim_field_targets_{time.strftime("%Y%m%d_%H%M%S")}.jsonl'
    with open(out_path, 'a', encoding='utf-8') as sink:
        for cid in selected:
            record = run_case(cid, cases[cid])
            outcomes.append(record)
            sink.write(json.dumps(record, ensure_ascii=False) + '\n')
            sink.flush()
            print(f"  -> outcome={record.get('outcome')} "
                  f"completion={record.get('completion_level')} "
                  f"pregrasp_passed={record.get('pregrasp_passed')} "
                  f"reason={record.get('reason', record.get('error'))[:120]}",
                  flush=True)
            if record.get('tcp_detour'):
                tcp = record['tcp_detour']
                print(f"  [绕行检测] 点数={tcp['points']} 弦={tcp['chord_m']}m "
                      f"路径={tcp['path_m']}m 比={tcp['ratio']} "
                      f"偏离={tcp['max_dev_m']}m 回退={tcp['max_recede_m']}m "
                      f"起={tcp['start_xyz']} 早段={tcp['head_xyz']} "
                      f"终={tcp['end_xyz']}"
                      + (f"  ⚠ 绕行: {record['detour_detail']}"
                         if record['detour_flag'] else '  ✓ 门内'),
                      flush=True)
            time.sleep(0.5)

    print(f'\n结果已写入 {out_path}')
    passed = sum(
        1 for r in outcomes
        if r.get('outcome') == ExecuteTarget.Result.SUCCEEDED)
    print(f'成功 {passed}/{len(outcomes)}')
    rclpy.shutdown()
    return 0


def _robot_status_type():
    from aubo_msgs.msg import RobotStatus
    return RobotStatus


def _mat_to_quat(m):
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return (0.25 * s, (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return ((m[0][1] + m[1][0]) / s, 0.25 * s,
                (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s,
            0.25 * s, (m[1][0] - m[0][1]) / s)


def _constraints_type():
    from moveit_msgs.msg import Constraints
    return Constraints


def _joint_constraint_type():
    from moveit_msgs.msg import JointConstraint
    return JointConstraint


if __name__ == '__main__':
    sys.exit(main())
