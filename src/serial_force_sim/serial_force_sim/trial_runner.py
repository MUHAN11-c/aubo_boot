#!/usr/bin/env python3
"""批量 CoP → TCP 对齐试验：多场景 × 重复，逐组写 JSONL。

每组试验:
  1. JTC 复位到 HOME
  2. 以 (tilt_deg, tilt_yaw_deg) 参数启动 fake_force.py（每场景重新定义虚拟平面）
  3. 采初始 CoP / 5 点力
  4. 走 IK + JTC 闭环（与 cop_control.py 同一控制律）
  5. 采最终 CoP，追加一行 JSON

用法:
  ros2 run serial_force_sim cop_trials \
      --tilt 1,2,3,4,5 --yaw 0,90 --repeats 2
统计:
  ros2 run serial_force_sim cop_analyze <jsonl>
默认输出到 $ROS_HOME/serial_force_sim/。
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

from serial_force_sim.common import (CENTER_MM, CH, HOME, JN, MAX_ROT_DEG,
                                     R2q, aaR, cop, q2R)

ROS_HOME = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros')))
OUT_DIR = ROS_HOME / 'serial_force_sim'


class TrialNode(Node):
    def __init__(self):
        super().__init__('cop_trial_runner')
        self.f = [0.0] * 5
        self.q = None
        self.n_msgs = 0
        self.create_subscription(
            Float64MultiArray, '/force/points', self.on_force, 10)
        self.create_subscription(
            JointState, '/joint_states', self.on_joint, 50)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.jtc = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        if not self.ik_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('compute_ik service unavailable')
        if not self.jtc.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('JTC action unavailable')

    def on_force(self, msg):
        if len(msg.data) == 5:
            self.f = [float(v) for v in msg.data]
            self.n_msgs += 1

    def on_joint(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JN):
            self.q = [d[j] for j in JN]

    def move_home(self, dur=2.5, timeout=12.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JN
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in HOME]
        pt.time_from_start = Duration(
            sec=int(dur), nanosec=int((dur - int(dur)) * 1e9))
        goal.trajectory.points = [pt]
        fut = self.jtc.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        res = rf.result()
        return res is not None and res.result.error_code == 0

    def solve(self, pose, timeout=2.0):
        if self.q is None:
            return None
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = 'manipulator_e5'
        r.avoid_collisions = False
        r.timeout.sec = 0
        r.timeout.nanosec = 300000000
        r.pose_stamped = PoseStamped()
        r.pose_stamped.header.frame_id = 'base_link'
        r.pose_stamped.pose = pose
        r.robot_state.joint_state.name = JN
        r.robot_state.joint_state.position = list(self.q)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        try:
            return [sol[j] for j in JN]
        except KeyError:
            return None

    def send(self, q, dur=0.8, timeout=8.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JN
        pt = JointTrajectoryPoint()
        pt.positions = q
        pt.time_from_start = Duration(sec=0, nanosec=int(dur * 1e9))
        goal.trajectory.points = [pt]
        fut = self.jtc.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        res = rf.result()
        return res is not None and res.result.error_code == 0


def spin_for(node, dur):
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.02)


def sample(node, dur):
    ps, ts, fs = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.02)
        p, t = cop(node.f)
        ps.append(p)
        ts.append(t)
        fs.append(list(node.f))
    return (np.mean(ps, axis=0), float(np.mean(ts)),
            [round(float(v), 4) for v in np.mean(fs, axis=0)])


def wait_force(node, n0, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        _, tot = cop(node.f)
        if node.n_msgs > n0 + 20 and tot > 0.05:
            return True
    return False


def start_force(tilt, yaw, log_path):
    cmd = [sys.executable, '-m', 'serial_force_sim.fake_force',
           '--ros-args', '-p', f'tilt_deg:={tilt}',
           '-p', f'tilt_yaw_deg:={yaw}']
    log = open(log_path, 'ab')
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    return proc, log


def stop_force(proc, log):
    if proc is not None and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3)
    if log is not None:
        log.close()


def control(node, max_steps, center_mm, max_rot_deg):
    steps = []
    cum = 0.0
    sign = -1.0
    prev = None
    for i in range(1, max_steps + 1):
        rclpy.spin_once(node, timeout_sec=0.05)
        p, tot = cop(node.f)
        norm = float(np.hypot(p[0], p[1]))
        if tot < 0.05:
            return steps, cum, 'no_contact'
        if norm < center_mm:
            return steps, cum, 'centered'
        if prev is not None and norm > prev * 1.05:
            sign *= -1.0
        prev = norm
        angle = sign * min(0.02, max(0.006, 0.0035 * norm))
        if abs(cum) + abs(angle) > math.radians(max_rot_deg):
            return steps, cum, 'rotation_limit'
        try:
            tf = node.buf.lookup_transform('base_link', 'tcp', Time())
        except Exception as exc:
            node.get_logger().warn(f'tcp TF unavailable: {exc}')
            return steps, cum, 'tf_fail'
        tr = tf.transform
        R = q2R(tr.rotation)
        axis = np.array([-p[1], p[0], 0.0])
        if np.linalg.norm(axis) < 1e-9:
            return steps, cum, 'degenerate'
        R_new = R @ aaR(axis, angle)
        x, y, z, w = R2q(R_new)
        pose = Pose()
        pose.position.x = tr.translation.x
        pose.position.y = tr.translation.y
        pose.position.z = tr.translation.z
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = w
        q_sol = node.solve(pose)
        if q_sol is None:
            return steps, cum, 'ik_fail'
        ok = node.send(q_sol)
        steps.append({
            'i': i,
            'cop_before': [round(float(p[0]), 3), round(float(p[1]), 3)],
            'total_before': round(tot, 4),
            'angle_deg': round(math.degrees(angle), 3),
            'jtc_ok': bool(ok),
        })
        if not ok:
            return steps, cum, 'jtc_fail'
        cum += angle
        spin_for(node, 0.9)
    return steps, cum, 'max_steps'


def run_trial(node, tilt, yaw, rep, out_path, log_dir,
              center_mm, max_rot_deg, max_steps):
    log_path = log_dir / f'force_t{tilt:+.1f}_y{yaw:+.0f}_{rep}.log'
    node.get_logger().info(
        f'=== trial tilt={tilt} yaw={yaw} repeat={rep} ===')
    home_ok = node.move_home()
    spin_for(node, 0.4)
    n0 = node.n_msgs
    proc = log = None
    try:
        proc, log = start_force(tilt, yaw, log_path)
        if not wait_force(node, n0):
            return {'tilt_deg': tilt, 'tilt_yaw_deg': yaw, 'repeat': rep,
                    'error': 'no_force_data', 'home_ok': home_ok}
        spin_for(node, 0.6)
        p0, t0, f0 = sample(node, 0.6)
        t_start = time.monotonic()
        steps, cum, reason = control(node, max_steps, center_mm, max_rot_deg)
        dur = time.monotonic() - t_start
        spin_for(node, 0.6)
        p1, t1, f1 = sample(node, 0.6)
    finally:
        stop_force(proc, log)
    p0n = float(np.hypot(p0[0], p0[1]))
    p1n = float(np.hypot(p1[0], p1[1]))
    rec = {
        'tilt_deg': tilt,
        'tilt_yaw_deg': yaw,
        'repeat': rep,
        'home_ok': bool(home_ok),
        'initial': {'cop': [round(float(p0[0]), 3), round(float(p0[1]), 3)],
                    'cop_norm': round(p0n, 3),
                    'total': round(t0, 4), 'forces': f0},
        'final': {'cop': [round(float(p1[0]), 3), round(float(p1[1]), 3)],
                  'cop_norm': round(p1n, 3),
                  'total': round(t1, 4), 'forces': f1},
        'reduction_pct': round((1.0 - p1n / p0n) * 100.0, 2) if p0n > 1e-6 else None,
        'success': bool(p1n < center_mm),
        'reason': reason,
        'n_steps': len(steps),
        'rotation_deg': round(math.degrees(cum), 3),
        'duration_s': round(dur, 2),
        'steps': steps,
    }
    with open(out_path, 'a') as fh:
        fh.write(json.dumps(rec) + '\n')
    print(json.dumps({k: rec[k] for k in (
        'tilt_deg', 'tilt_yaw_deg', 'repeat', 'initial', 'final',
        'reduction_pct', 'success', 'reason', 'n_steps',
        'rotation_deg', 'duration_s')}), flush=True)
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tilt', default='1,2,3,4,5')
    ap.add_argument('--yaw', default='0,90')
    ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--out',
                    default=str(OUT_DIR / 'force_align_sim_trials.jsonl'))
    ap.add_argument('--log-dir',
                    default=str(OUT_DIR / 'force_align_sim_logs'))
    ap.add_argument('--center-mm', type=float, default=CENTER_MM)
    ap.add_argument('--max-rot-deg', type=float, default=MAX_ROT_DEG)
    ap.add_argument('--max-steps', type=int, default=12)
    args = ap.parse_args()
    tilts = [float(v) for v in args.tilt.split(',') if v]
    yaws = [float(v) for v in args.yaw.split(',') if v]
    out_path = Path(args.out)
    log_dir = Path(args.log_dir)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = TrialNode()
    try:
        for tilt in tilts:
            for yaw in yaws:
                for rep in range(1, args.repeats + 1):
                    try:
                        run_trial(node, tilt, yaw, rep, out_path, log_dir,
                                  args.center_mm, args.max_rot_deg,
                                  args.max_steps)
                    except Exception as exc:  # keep the sweep alive
                        node.get_logger().error(f'trial failed: {exc}')
                        with open(out_path, 'a') as fh:
                            fh.write(json.dumps({
                                'tilt_deg': tilt, 'tilt_yaw_deg': yaw,
                                'repeat': rep, 'error': repr(exc),
                            }) + '\n')
    finally:
        node.move_home()
        node.destroy_node()
        rclpy.shutdown()
    print(f'wrote {out_path}', flush=True)


if __name__ == '__main__':
    main()
