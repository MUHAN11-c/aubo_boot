#!/usr/bin/env python3
"""
ROS2 轨迹插值节点：替代 ROS1 aubo_robot_simulator。

- 订阅 joint_path_command (trajectory_msgs/JointTrajectory)，与 aubo_ros2_trajectory_action 输出一致
- 使用 5 次多项式插值，按 motion_update_rate（默认 200Hz）生成中间点
- 发布 moveItController_cmd (trajectory_msgs/JointTrajectoryPoint)，由 ros1_bridge 桥接到 ROS1 aubo_driver

参数（与 aubo_e5_moveit_bridge.launch 中 aubo_controller 一致）：
- motion_update_rate: 200
- minimum_buffer_size: 2000
- joint_names: 与 MoveIt 一致
"""

import copy
import json
import os
import queue
import threading
import time as _time
import rclpy
from rclpy.node import Node

from rclpy.duration import Duration
from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int32MultiArray, Float32MultiArray

from .trajectory_speed import scale_trajectory_speed, resample_trajectory_min_interval


def duration_to_sec(d: DurationMsg) -> float:
    return float(d.sec) + 1e-9 * float(d.nanosec)


def sec_to_duration(sec: float) -> DurationMsg:
    d = DurationMsg()
    d.sec = int(sec)
    d.nanosec = int(round((sec - int(sec)) * 1e9))
    return d


def _motion_analysis_log(event_type: str, **kwargs) -> None:
    """移植前后对比：MOTION_LOG_FILE 由启动脚本设置，记录关键数据用于分析抖动"""
    path = os.environ.get("MOTION_LOG_FILE")
    if not path:
        return
    entry = {"ts_ms": int(_time.time() * 1000), "side": "after", "event": event_type, **kwargs}
    try:
        with open(path, "a") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
    except Exception:
        pass


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
        self.declare_parameter("velocity_scale_factor", 1.0)
        self.declare_parameter("cartesian_resample_threshold", 0.095)
        self.declare_parameter("cartesian_min_segment_interval", 0.09)
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
        if len(msg.points) == 0:
            return
        self.get_logger().info("Received trajectory with %d points" % len(msg.points))
        if self.motion_ctrl.is_in_motion():
            self.get_logger().warn("New trajectory while in motion, replacing")
            self.motion_ctrl.stop()
        # 与 ROS1 一致：callback 内每次读取参数，再速度缩放、笛卡尔抽稀（逻辑与 ROS1 trajectory_callback 一致）
        velocity_scale_factor = self.get_parameter("velocity_scale_factor").value
        cartesian_threshold = self.get_parameter("cartesian_resample_threshold").value
        target_interval = self.get_parameter("cartesian_min_segment_interval").value
        self.get_logger().info("The velocity scale factor of the trajetory is: %s" % str(velocity_scale_factor))
        new_traj = scale_trajectory_speed(msg, velocity_scale_factor)
        pts = new_traj.points
        if len(pts) >= 3 and target_interval > 0:
            intervals = [
                duration_to_sec(pts[i].time_from_start) - duration_to_sec(pts[i - 1].time_from_start)
                for i in range(1, len(pts))
            ]
            min_iv = min(intervals)
            if min_iv < cartesian_threshold:
                new_traj = resample_trajectory_min_interval(new_traj, target_interval)
                n_before, pts = len(pts), new_traj.points
                self.get_logger().info(
                    "Cartesian resampled: %d -> %d points (target_interval=%.3fs, align to joint ~0.095s)"
                    % (n_before, len(pts), target_interval)
                )
        for point in pts:
            pt = self._to_controller_order(msg.joint_names, point)
            self.motion_ctrl.add_motion_waypoint(pt)
        if len(pts) >= 2:
            t0 = duration_to_sec(pts[0].time_from_start)
            t1 = duration_to_sec(pts[-1].time_from_start)
            intervals = [duration_to_sec(pts[i].time_from_start) - duration_to_sec(pts[i - 1].time_from_start) for i in range(1, len(pts))]
            _motion_analysis_log("trajectory_received", num_points=len(pts), duration_sec=round(t1 - t0, 4),
                                 min_interval=round(min(intervals), 4), max_interval=round(max(intervals), 4),
                                 avg_interval=round((t1 - t0) / (len(pts) - 1), 4) if len(pts) > 1 else 0)

    def _move_to(self, point: JointTrajectoryPoint, dur: float) -> None:
        import time
        throttle_sleep = max(min(dur, 0.002), 0.0005)
        block_count = 0
        total_sleep_ms = 0.0
        rib_at_start = 0
        while self.motion_ctrl.rib_buffer_size > self.motion_ctrl.minimum_buffer_size:
            if block_count == 0:
                rib_at_start = self.motion_ctrl.rib_buffer_size
            block_count += 1
            total_sleep_ms += throttle_sleep * 1000
            time.sleep(throttle_sleep)
        if block_count > 0:
            _motion_analysis_log("throttle", block_count=block_count, rib_buffer_size=rib_at_start, sleep_ms=round(total_sleep_ms, 2), dur_sec=round(float(dur), 4))
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

    def _publish_cmd(self) -> None:
        now_ms = int(_time.time() * 1000)
        if not hasattr(self, "_publish_count"):
            self._publish_count = 0
            self._last_publish_ts_ms = now_ms
        self._publish_count += 1
        msg = JointTrajectoryPoint()
        now_ns = self.get_clock().now().nanoseconds
        msg.time_from_start = sec_to_duration(now_ns * 1e-9)
        with self.motion_ctrl.lock:
            msg.positions = list(self.motion_ctrl.joint_positions)
            msg.velocities = list(self.motion_ctrl.joint_velocities)
            msg.accelerations = list(self.motion_ctrl.joint_accelerations)
        self.moveit_cmd_pub.publish(msg)
        if self._publish_count % 200 == 0:
            _motion_analysis_log("publish_dt", dt_ms=now_ms - self._last_publish_ts_ms, count=self._publish_count)
        self._last_publish_ts_ms = now_ms

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
                    # 与 ROS1 一致：N 以 positions 为准，velocities/accelerations 不足 N 时用 0 填充
                    N = len(last_goal_point.positions)
                    def _pad(x, n):
                        return list(x)[:n] + [0.0] * (n - len(x)) if len(x) < n else list(x)[:n]
                    v_last = _pad(last_goal_point.velocities, N)
                    a_last = _pad(last_goal_point.accelerations, N)
                    v_curr = _pad(current_goal_point.velocities, N)
                    a_curr = _pad(current_goal_point.accelerations, N)
                    T = curr_sec - last_sec
                    T2, T3 = T * T, T * T * T
                    T4 = T3 * T
                    T5 = T4 * T
                    a1 = v_last
                    a2 = [0.5 * a_last[i] for i in range(N)]
                    h = [current_goal_point.positions[i] - last_goal_point.positions[i] for i in range(N)]
                    a3 = [
                        0.5 / T3 * (20 * h[i] - (8 * v_curr[i] + 12 * v_last[i]) * T - (3 * a_last[i] - a_curr[i]) * T2)
                        for i in range(N)
                    ]
                    a4 = [
                        0.5 / T4 * (-30 * h[i] + (14 * v_curr[i] + 16 * v_last[i]) * T + (3 * a_last[i] - 2 * a_curr[i]) * T2)
                        for i in range(N)
                    ]
                    a5 = [
                        0.5 / T5 * (12 * h[i] - 6 * (v_curr[i] + v_last[i]) * T + (a_curr[i] - a_last[i]) * T2)
                        for i in range(N)
                    ]
                    # 整数步数控制：避免浮点累积导致 num_steps 不稳定（19步 vs 20步）
                    n_steps = max(1, round(T / update_duration_sec))
                    num_steps = 0
                    # 精确计时：记录 segment 起始 wall-clock，通过累积误差补偿保证 200Hz 均匀发布
                    _timing_wall_start = _time.perf_counter()
                    for _step_idx in range(n_steps):
                        if self.motion_ctrl.cancel_trajectory != 0:
                            break
                        t1 = _step_idx * update_duration_sec
                        t2, t3 = t1 * t1, t1 * t1 * t1
                        t4 = t3 * t1
                        t5 = t4 * t1
                        # 与 ROS1 公式一致；整列表赋值以兼容 ROS2 消息类型
                        intermediate_goal_point.positions = [
                            last_goal_point.positions[i] + a1[i] * t1 + a2[i] * t2 + a3[i] * t3 + a4[i] * t4 + a5[i] * t5
                            for i in range(N)
                        ]
                        intermediate_goal_point.velocities = [
                            a1[i] + 2 * a2[i] * t1 + 3 * a3[i] * t2 + 4 * a4[i] * t3 + 5 * a5[i] * t4 for i in range(N)
                        ]
                        intermediate_goal_point.accelerations = [
                            2 * a2[i] + 6 * a3[i] * t1 + 12 * a4[i] * t2 + 20 * a5[i] * t3 for i in range(N)
                        ]
                        num_steps += 1
                        self._move_to(intermediate_goal_point, update_duration_sec)
                        self._publish_cmd()
                        # 累积误差补偿：等待到该步骤对应的 wall-clock 时刻，实现均匀 200Hz 发布
                        _expected_elapsed = (_step_idx + 1) * update_duration_sec
                        _actual_elapsed = _time.perf_counter() - _timing_wall_start
                        _wait = _expected_elapsed - _actual_elapsed
                        if _wait > 0.0001:
                            _time.sleep(_wait)
                    tt = last_sec + n_steps * update_duration_sec
                    _motion_analysis_log("segment", T=round(T, 4), num_steps=num_steps)
                    move_duration_sec = curr_sec - tt if tt < curr_sec else 0.0

                dur_val = move_duration_sec if move_duration_sec > 0 else 0.0
                if dur_val > 0:
                    time.sleep(dur_val)
                self._move_to(current_goal_point, dur_val)
                self._publish_cmd()
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
