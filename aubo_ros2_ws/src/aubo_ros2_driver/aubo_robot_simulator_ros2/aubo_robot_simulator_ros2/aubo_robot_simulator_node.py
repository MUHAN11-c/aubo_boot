#!/usr/bin/env python3
"""
ROS2 轨迹插值节点：替代 ROS1 aubo_robot_simulator。

- 订阅 joint_path_command (trajectory_msgs/JointTrajectory)，与 aubo_ros2_trajectory_action 输出一致
- 使用 5 次多项式插值，按 motion_update_rate（默认 200Hz）生成中间点
- 发布 moveItController_cmd (trajectory_msgs/JointTrajectoryPoint)，由 ros1_bridge 桥接到 ROS1 aubo_driver

参数（与 aubo_e5_moveit_bridge.launch 中 aubo_controller 一致）：
- motion_update_rate: 200
- minimum_buffer_size: 600（launch 中设置，降低节流阈值以减轻速度因子 0.1 时卡顿）
- joint_names: 与 MoveIt 一致
"""

import copy
import queue
import threading
import time as _time
import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int32MultiArray, Float32MultiArray


def duration_to_sec(d: DurationMsg) -> float:
    return float(d.sec) + 1e-9 * float(d.nanosec)


def sec_to_duration(sec: float) -> DurationMsg:
    d = DurationMsg()
    d.sec = int(sec)
    d.nanosec = int(round((sec - int(sec)) * 1e9))
    return d


def _quintic_interpolate(last: JointTrajectoryPoint, curr: JointTrajectoryPoint, t: float, T: float) -> tuple:
    """5 次多项式插值，返回 (positions, velocities, accelerations)。t 为段内时间 [0,T]。"""
    N = len(last.positions)
    def _pad(x, n):
        return list(x)[:n] + [0.0] * (n - len(x)) if len(x) < n else list(x)[:n]
    v_last = _pad(last.velocities, N)
    a_last = _pad(last.accelerations, N)
    v_curr = _pad(curr.velocities, N)
    a_curr = _pad(curr.accelerations, N)
    T2, T3 = T * T, T * T * T
    T4 = T3 * T
    T5 = T4 * T
    h = [curr.positions[i] - last.positions[i] for i in range(N)]
    a1 = v_last
    a2 = [0.5 * a_last[i] for i in range(N)]
    a3 = [0.5 / T3 * (20 * h[i] - (8 * v_curr[i] + 12 * v_last[i]) * T - (3 * a_last[i] - a_curr[i]) * T2) for i in range(N)]
    a4 = [0.5 / T4 * (-30 * h[i] + (14 * v_curr[i] + 16 * v_last[i]) * T + (3 * a_last[i] - 2 * a_curr[i]) * T2) for i in range(N)]
    a5 = [0.5 / T5 * (12 * h[i] - 6 * (v_curr[i] + v_last[i]) * T + (a_curr[i] - a_last[i]) * T2) for i in range(N)]
    t2, t3 = t * t, t * t * t
    t4 = t3 * t
    t5 = t4 * t
    pos = [last.positions[i] + a1[i] * t + a2[i] * t2 + a3[i] * t3 + a4[i] * t4 + a5[i] * t5 for i in range(N)]
    vel = [a1[i] + 2 * a2[i] * t + 3 * a3[i] * t2 + 4 * a4[i] * t3 + 5 * a5[i] * t4 for i in range(N)]
    acc = [2 * a2[i] + 6 * a3[i] * t + 12 * a4[i] * t2 + 20 * a5[i] * t3 for i in range(N)]
    return (pos, vel, acc)


# 与 ROS2 MoveIt 一致
DEFAULT_JOINT_NAMES = [
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
]


class MotionControllerSimulator:
    """5 次多项式插值 + 按固定频率发布 moveItController_cmd。"""

    def __init__(self, num_joints: int, update_rate: float = 200.0, minimum_buffer_size: int = 2000):
        self.lock = threading.Lock()
        self.cancel_trajectory = 0
        self.update_rate = update_rate
        self.minimum_buffer_size = minimum_buffer_size
        self.motion_buffer = queue.Queue()
        self.joint_names = list(DEFAULT_JOINT_NAMES[:num_joints])
        self.joint_positions = [0.0] * num_joints
        self.joint_velocities = [0.0] * num_joints
        self.joint_accelerations = [0.0] * num_joints
        self.rib_buffer_size = 0
        self.controller_connected_flag = 1
        self.sig_shutdown = False
        self.sig_stop = False

    def add_motion_waypoint(self, point: JointTrajectoryPoint) -> None:
        self.motion_buffer.put(point)

    def get_joint_positions(self):
        with self.lock:
            return list(self.joint_positions)

    def is_in_motion(self) -> bool:
        return not self.motion_buffer.empty()

    def stop(self) -> None:
        with self.lock:
            self._clear_buffer()
            self.sig_stop = True

    def _clear_buffer(self) -> None:
        while not self.motion_buffer.empty():
            try:
                self.motion_buffer.get_nowait()
            except queue.Empty:
                break

    def shutdown(self) -> None:
        self.sig_shutdown = True


class AuboRobotSimulatorNode(Node):
    def __init__(self):
        super().__init__("aubo_robot_simulator")
        self.declare_parameter("motion_update_rate", 200.0)
        self.declare_parameter("minimum_buffer_size", 2000)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        motion_update_rate = self.get_parameter("motion_update_rate").value
        minimum_buffer_size = self.get_parameter("minimum_buffer_size").value
        self.joint_names = self.get_parameter("joint_names").value

        self.get_logger().info(
            "motion_update_rate=%.1f Hz, minimum_buffer_size=%d"
            % (motion_update_rate, minimum_buffer_size)
        )
        num_joints = len(self.joint_names)
        self.motion_ctrl = MotionControllerSimulator(
            num_joints,
            update_rate=motion_update_rate,
            minimum_buffer_size=minimum_buffer_size,
        )

        # 发布 moveItController_cmd -> ros1_bridge -> ROS1 aubo_driver
        self.moveit_cmd_pub = self.create_publisher(
            JointTrajectoryPoint,
            "moveItController_cmd",
            2000,
        )
        self.joint_path_sub = self.create_subscription(
            JointTrajectory,
            "joint_path_command",
            self.trajectory_callback,
            10,
        )
        # RIB 状态由 ROS1 aubo_driver 发布，经 bridge 到 ROS2（可选）
        self.rib_status_sub = self.create_subscription(
            Int32MultiArray,
            "/aubo_driver/rib_status",
            self.rib_status_callback,
            10,
        )
        self.real_pose_sub = self.create_subscription(
            Float32MultiArray,
            "/aubo_driver/real_pose",
            self.real_pose_callback,
            10,
        )

        self.motion_thread = threading.Thread(target=self._motion_worker)
        self.motion_thread.daemon = True
        self.motion_thread.start()
        self.get_logger().info("aubo_robot_simulator_ros2 started (interpolation in ROS2)")

    def real_pose_callback(self, msg: Float32MultiArray) -> None:
        if not self.motion_ctrl.is_in_motion() and len(msg.data) >= len(self.motion_ctrl.joint_positions):
            with self.motion_ctrl.lock:
                for i in range(min(len(msg.data), len(self.motion_ctrl.joint_positions))):
                    self.motion_ctrl.joint_positions[i] = float(msg.data[i])

    def rib_status_callback(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 3:
            self.motion_ctrl.rib_buffer_size = int(msg.data[0])
            self.motion_ctrl.controller_connected_flag = int(msg.data[2])

    def _remap_order(self, ordered_keys, value_keys, values):
        mapping = dict(zip(value_keys, values))
        return [mapping[k] for k in ordered_keys]

    def _to_controller_order(self, msg_joint_names, point: JointTrajectoryPoint) -> JointTrajectoryPoint:
        pt = JointTrajectoryPoint()
        n = len(self.joint_names)
        pt.positions = self._remap_order(self.joint_names, msg_joint_names, list(point.positions)[:n] if point.positions else [0.0] * n)
        pt.velocities = self._remap_order(self.joint_names, msg_joint_names, list(point.velocities)[:n] if point.velocities else [0.0] * n)
        pt.accelerations = self._remap_order(self.joint_names, msg_joint_names, list(point.accelerations)[:n] if point.accelerations else [0.0] * n)
        pt.time_from_start = point.time_from_start
        return pt

    def trajectory_callback(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        self.get_logger().info("Received trajectory with %d points" % len(msg.points))
        if self.motion_ctrl.is_in_motion():
            self.get_logger().warn("New trajectory while in motion, replacing")
            self.motion_ctrl.stop()
        pts = msg.points
        for point in pts:
            self.motion_ctrl.add_motion_waypoint(self._to_controller_order(msg.joint_names, point))
        if pts:
            pt0 = self._to_controller_order(msg.joint_names, pts[0])
            n = len(pt0.positions)
            with self.motion_ctrl.lock:
                if not self.motion_ctrl.sig_stop:
                    self.motion_ctrl.joint_positions = list(pt0.positions)
                    self.motion_ctrl.joint_velocities = list(pt0.velocities)[:n] if len(pt0.velocities) == n else [0.0] * n
                    self.motion_ctrl.joint_accelerations = list(pt0.accelerations)[:n] if len(pt0.accelerations) == n else [0.0] * n
            # 保持单线程发布：仅 motion_worker 发布命令，避免 callback 与 worker 在轨迹边界并发发布导致突变
    def _move_to(self, point: JointTrajectoryPoint, dur: float) -> None:
        import time
        throttle_sleep = max(min(dur, 0.002), 0.0005)
        throttle_wait = 0.0
        while self.motion_ctrl.rib_buffer_size > self.motion_ctrl.minimum_buffer_size:
            time.sleep(throttle_sleep)
            throttle_wait += throttle_sleep
        if self.motion_ctrl.rib_buffer_size == 0 and self.motion_ctrl.controller_connected_flag == 0:
            time.sleep(dur)
        n = len(point.positions)
        v = list(point.velocities) if len(point.velocities) == n else [0.0] * n
        a = list(point.accelerations) if len(point.accelerations) == n else [0.0] * n
        with self.motion_ctrl.lock:
            if not self.motion_ctrl.sig_stop:
                self.motion_ctrl.joint_positions = list(point.positions)
                self.motion_ctrl.joint_velocities = v
                self.motion_ctrl.joint_accelerations = a
            else:
                self.motion_ctrl.sig_stop = False

    def _publish_cmd(self, point_time_sec: float, source: str = "unknown") -> None:
        msg = JointTrajectoryPoint()
        msg.time_from_start = sec_to_duration(point_time_sec)
        with self.motion_ctrl.lock:
            msg.positions = list(self.motion_ctrl.joint_positions)
            msg.velocities = list(self.motion_ctrl.joint_velocities)
            msg.accelerations = list(self.motion_ctrl.joint_accelerations)
        self.moveit_cmd_pub.publish(msg)

    def _motion_worker(self) -> None:
        update_duration_sec = 1.0 / self.motion_ctrl.update_rate if self.motion_ctrl.update_rate > 0 else 0.01
        last_goal_point = JointTrajectoryPoint()
        last_goal_point.time_from_start = sec_to_duration(0.0)
        with self.motion_ctrl.lock:
            last_goal_point.positions = list(self.motion_ctrl.joint_positions)
            n = len(last_goal_point.positions)
            last_goal_point.velocities = [0.0] * n
            last_goal_point.accelerations = [0.0] * n

        import time
        move_duration_sec = 0.0
        while not self.motion_ctrl.sig_shutdown:
            try:
                current_goal_point = self.motion_ctrl.motion_buffer.get(timeout=0.5)
            except queue.Empty:
                continue
            except Exception:
                continue
            try:
                intermediate_goal_point = copy.deepcopy(current_goal_point)
                curr_sec = duration_to_sec(current_goal_point.time_from_start)
                last_sec = duration_to_sec(last_goal_point.time_from_start)
                is_new_traj = curr_sec <= last_sec

                if is_new_traj:
                    move_duration_sec = curr_sec
                else:
                    T = curr_sec - last_sec
                    n_steps = max(1, round(T / update_duration_sec))
                    _timing_wall_start = _time.perf_counter()
                    for _step_idx in range(1, n_steps):
                        if self.motion_ctrl.cancel_trajectory != 0:
                            break
                        t = min(_step_idx * update_duration_sec, T)
                        pos, vel, acc = _quintic_interpolate(last_goal_point, current_goal_point, t, T)
                        intermediate_goal_point.positions = pos
                        intermediate_goal_point.velocities = vel
                        intermediate_goal_point.accelerations = acc
                        intermediate_goal_point.time_from_start = sec_to_duration(last_sec + t)
                        self._move_to(intermediate_goal_point, update_duration_sec)
                        self._publish_cmd(last_sec + t, "motion_worker_step")
                        _wait = _step_idx * update_duration_sec - (_time.perf_counter() - _timing_wall_start)
                        if _wait > 0.0001:
                            _time.sleep(_wait)
                    tt = last_sec + max(0, n_steps - 1) * update_duration_sec
                    move_duration_sec = curr_sec - tt if tt < curr_sec else 0.0
                    seg_elapsed = _time.perf_counter() - _timing_wall_start
                    if seg_elapsed > T * 1.01 and move_duration_sec > 0:
                        move_duration_sec = 0.0

                dur_val = move_duration_sec if move_duration_sec > 0 else 0.0
                if dur_val > 0:
                    time.sleep(dur_val)
                self._move_to(current_goal_point, dur_val)
                self._publish_cmd(curr_sec, "motion_worker_final")
                last_goal_point = copy.deepcopy(current_goal_point)
            except Exception as e:
                self.get_logger().error("motion_worker exception: %s" % str(e))

        self.get_logger().info("Motion worker exited")


def main(args=None):
    rclpy.init(args=args)
    node = AuboRobotSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.motion_ctrl.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
