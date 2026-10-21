#!/usr/bin/env python3
"""单次 CoP → TCP 对齐闭环（仿真演示）。

读 /force/points 算 CoP，绕 CoP 的垂轴把 TCP 转一个小角度（IK + JTC），
直到 |CoP| < 门限、累计旋转到上限或步数用完。用于快速看一眼闭环效果；
批量统计用 trial_runner.py。

用法:
  ros2 run serial_force_sim cop_control \
      [--center-mm 1.2] [--max-rot-deg 5] [--max-steps 12]
"""
import argparse
import math
import time

import numpy as np
import rclpy
import tf2_ros
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

from serial_force_sim.common import (CENTER_MM, JN, MAX_ROT_DEG, R2q,
                                     aaR, cop, q2R)


class CopControl(Node):
    def __init__(self, center_mm, max_rot_deg, max_steps):
        super().__init__('force_align_cop_control')
        self.center_mm = center_mm
        self.max_rot = math.radians(max_rot_deg)
        self.max_steps = max_steps
        self.f = [0.0] * 5
        self.q = None
        self.create_subscription(
            Float64MultiArray, '/force/points', self.cbf, 10)
        self.create_subscription(JointState, '/joint_states', self.cbq, 10)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.jtc = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self.sign = -1.0
        self.cum = 0.0
        self.ik.wait_for_service()
        self.jtc.wait_for_server()

    def cbf(self, m):
        if len(m.data) == 5:
            self.f = [float(v) for v in m.data]

    def cbq(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JN):
            self.q = [d[j] for j in JN]

    def solve(self, pose):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = 'manipulator_e5'
        r.avoid_collisions = False
        r.timeout.sec = 0
        r.timeout.nanosec = 200000000
        r.pose_stamped = PoseStamped()
        r.pose_stamped.header.frame_id = 'base_link'
        r.pose_stamped.pose = pose
        r.robot_state.joint_state.name = JN
        r.robot_state.joint_state.position = self.q
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in JN]

    def send(self, q):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = JN
        pt = JointTrajectoryPoint()
        pt.positions = q
        pt.time_from_start.sec = 1
        pt.time_from_start.nanosec = 0
        g.trajectory.points = [pt]
        fut = self.jtc.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=10)
        res = rf.result()
        return res is not None and res.result.error_code == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--center-mm', type=float, default=CENTER_MM)
    ap.add_argument('--max-rot-deg', type=float, default=MAX_ROT_DEG)
    ap.add_argument('--max-steps', type=int, default=12)
    args = ap.parse_args()

    rclpy.init()
    n = CopControl(args.center_mm, args.max_rot_deg, args.max_steps)
    n.get_logger().info(
        f'center={args.center_mm} mm, max rot={args.max_rot_deg} deg, '
        f'max steps={args.max_steps}')
    last = 0.0
    prev = None
    t0 = time.monotonic()
    reason = 'max_steps'
    steps = 0
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.monotonic() - last < 1.5:
            continue
        last = time.monotonic()
        p, tot = cop(n.f)
        norm = float(np.hypot(p[0], p[1]))
        n.get_logger().info(
            f'CoP=({p[0]:.1f},{p[1]:.1f}) |p|={norm:.1f} total={tot:.2f}')
        if tot < 0.05:
            reason = 'no_contact'
            if time.monotonic() - t0 > 12:
                n.get_logger().info('no force; stop')
                break
            continue
        if norm < n.center_mm:
            reason = 'centered'
            n.get_logger().info('centered')
            break
        if steps >= n.max_steps:
            reason = 'max_steps'
            break
        if prev is not None and norm > prev * 1.05:
            n.sign *= -1.0
            n.get_logger().info(f'grew flip {n.sign:+.0f}')
        prev = norm
        angle = n.sign * min(0.02, max(0.006, 0.0035 * norm))
        if abs(n.cum) + abs(angle) > n.max_rot:
            reason = 'rotation_limit'
            n.get_logger().info('rotation limit; stop')
            break
        try:
            tf = n.buf.lookup_transform('base_link', 'tcp', Time())
        except Exception as e:
            n.get_logger().warn(f'TF {e}')
            continue
        tr = tf.transform
        R = q2R(tr.rotation)
        axis = np.array([-p[1], p[0], 0.0])
        Rn = R @ aaR(axis, angle)
        x, y, z, w = R2q(Rn)
        pose = Pose()
        pose.position.x = tr.translation.x
        pose.position.y = tr.translation.y
        pose.position.z = tr.translation.z
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = w
        q = n.solve(pose)
        if q is None:
            n.get_logger().warn('IK failed')
            reason = 'ik_fail'
            break
        ok = n.send(q)
        n.get_logger().info(f'rot {math.degrees(angle):+.2f} deg JTC={ok}')
        if not ok:
            reason = 'jtc_fail'
            break
        n.cum += angle
        steps += 1
        time.sleep(0.5)
    n.get_logger().info(
        f'done: reason={reason}, steps={steps}, '
        f'rotation={math.degrees(n.cum):+.2f} deg')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
