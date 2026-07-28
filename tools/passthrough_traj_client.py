#!/usr/bin/env python3
"""Passthrough 轨迹测试客户端（参考 UR ur_robot_driver/scripts/example_move.py）。

以**当前关节位置为基准**叠加小幅偏移构造轨迹，sim/real 均可安全使用。
用法:
  python3 tools/passthrough_traj_client.py [轨迹名] [次数]
轨迹名:
  wave_shoulder  肩关节 ±0.2 rad 往返（默认）
  wave_all       六关节依次 ±0.1 rad
  sine_shoulder  肩关节 0.15 rad 正弦 2 周期（密集路点，检验重采样/RIB 流控）
示例:
  python3 tools/passthrough_traj_client.py wave_shoulder 3
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
]
ACTION_NAME = "/aubo_passthrough_trajectory_controller/follow_joint_trajectory"


def point(pos, t_sec):
    p = JointTrajectoryPoint()
    p.positions = list(pos)
    p.velocities = [0.0] * 6
    p.accelerations = [0.0] * 6
    p.time_from_start = Duration(sec=int(t_sec), nanosec=int((t_sec % 1) * 1e9))
    return p


def build_wave_shoulder(base):
    a, b = list(base), list(base)
    a[0] += 0.2
    b[0] -= 0.2
    return [point(a, 2.0), point(b, 4.0), point(base, 6.0)]


def build_wave_all(base):
    pts, t = [], 0.0
    for j in range(6):
        up, dn = list(base), list(base)
        up[j] += 0.1
        dn[j] -= 0.1
        pts += [point(up, t + 1.0), point(dn, t + 2.0)]
        t += 2.0
    pts.append(point(base, t + 1.0))
    return pts


def build_sine_shoulder(base, amp=0.15, period=2.0, cycles=2, dt=0.1):
    pts, t = [], dt
    while t <= period * cycles + 1e-9:
        q = list(base)
        q[0] += amp * math.sin(2 * math.pi * t / period)
        pts.append(point(q, t))
        t += dt
    pts.append(point(base, t + 1.0))
    return pts


BUILDERS = {
    "wave_shoulder": build_wave_shoulder,
    "wave_all": build_wave_all,
    "sine_shoulder": build_sine_shoulder,
}


class TrajClient(Node):
    def __init__(self):
        super().__init__("passthrough_traj_client")
        self._base = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def _on_js(self, msg):
        if self._base is not None:
            return
        try:
            self._base = [msg.position[msg.name.index(j)] for j in JOINTS]
        except ValueError:
            pass

    def wait_base(self, timeout=5.0):
        end = time.time() + timeout
        while self._base is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._base is None:
            raise RuntimeError("未收到 /joint_states，硬件/控制器未就绪")

    def run_once(self, traj_name):
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        traj.points = BUILDERS[traj_name](self._base)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"action 服务器不可用: {ACTION_NAME}")
        t0 = time.time()
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            print("goal 被拒绝")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        dt = time.time() - t0
        expect = traj.points[-1].time_from_start.sec + \
            traj.points[-1].time_from_start.nanosec * 1e-9
        status = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                  GoalStatus.STATUS_CANCELED: "CANCELED",
                  GoalStatus.STATUS_ABORTED: "ABORTED"}.get(wrapped.status, str(wrapped.status))
        print(f"[{traj_name}] {status}  error_code={wrapped.result.error_code}  "
              f"耗时 {dt:.2f}s（轨迹标称 {expect:.2f}s）  {wrapped.result.error_string}")
        return wrapped.status == GoalStatus.STATUS_SUCCEEDED


def main():
    traj_name = sys.argv[1] if len(sys.argv) > 1 else "wave_shoulder"
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    if traj_name not in BUILDERS:
        print(f"未知轨迹 {traj_name}，可选: {', '.join(BUILDERS)}")
        return 1
    rclpy.init()
    node = TrajClient()
    node.wait_base()
    ok = all(node.run_once(traj_name) for _ in range(count))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
