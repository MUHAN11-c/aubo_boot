#!/usr/bin/env python3
"""
对比 /demo_robot_status 中的末端位姿 与 MoveIt2 计算的末端位姿（/moveit2_tcp_pose）。

使用方法:
1. 启动机器人驱动与 demo_driver（确保正在发布 /demo_robot_status）。
2. 启动 MoveIt2 以及脚本 moveit2_tcp_pose_publisher.py，确保正在发布 /moveit2_tcp_pose。
3. 运行本脚本:
   ros2 run demo_driver compare_demo_robot_status_with_moveit.py
   或:
   ros2 run demo_driver compare_demo_robot_status_with_moveit.py \
     --ros-args -p robot_status_topic:=/demo_robot_status -p moveit_pose_topic:=/moveit2_tcp_pose
"""

import math
import time
import os
import csv

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from demo_interface.msg import RobotStatus


def quat_to_rpy(q):
    """将四元数转换为 RPY（单位: rad）。"""
    # q: geometry_msgs.msg.Quaternion
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class RobotVsMoveItPoseMonitor(Node):
    def __init__(self):
        super().__init__("robot_vs_moveit_pose_monitor")

        # 参数
        self.declare_parameter("robot_status_topic", "/demo_robot_status")
        self.declare_parameter("moveit_pose_topic", "/moveit2_tcp_pose")
        self.declare_parameter(
            "log_file",
            "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/scripts/robot_vs_moveit_pose.csv",
        )

        robot_status_topic = self.get_parameter("robot_status_topic").value
        moveit_pose_topic = self.get_parameter("moveit_pose_topic").value
        self.log_file = self.get_parameter("log_file").value

        # 确保日志目录存在
        log_dir = os.path.dirname(self.log_file)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

        self.get_logger().info(
            f"监听机器人状态话题: {robot_status_topic} 与 MoveIt2 位姿话题: {moveit_pose_topic}"
        )
        self.get_logger().info(f"日志文件: {self.log_file}")

        # 缓存最近一次的位姿
        self.robot_pose = None  # type: Pose | None
        self.robot_stamp = None
        self.moveit_pose = None  # type: Pose | None
        self.moveit_stamp = None
        self.last_print_time = 0.0

        # 订阅 /demo_robot_status
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            robot_status_topic,
            self.robot_status_callback,
            10,
        )

        # 订阅 MoveIt2 计算的末端位姿（由 moveit2_tcp_pose_publisher.py 发布）
        self.moveit_pose_sub = self.create_subscription(
            PoseStamped,
            moveit_pose_topic,
            self.moveit_pose_callback,
            10,
        )

        self.get_logger().info("机器人 vs MoveIt2 末端位姿对比节点已启动。")

    def robot_status_callback(self, msg: RobotStatus):
        """接收机器人状态中的笛卡尔位姿。"""
        # 假设 RobotStatus.cartesian_position 类型为 geometry_msgs/Pose
        self.robot_pose = msg.cartesian_position
        self.robot_stamp = self.get_clock().now()

        # 每次收到机器人状态时尝试对比一次
        self.compare_if_ready()

    def moveit_pose_callback(self, msg: PoseStamped):
        """接收 MoveIt2 通过 TF 计算的末端位姿。"""
        self.moveit_pose = msg.pose
        self.moveit_stamp = self.get_clock().now()

    def compare_if_ready(self):
        """当两种位姿都就绪时，计算和打印差异。"""
        if self.robot_pose is None or self.moveit_pose is None:
            return

        now = time.time()
        # 防止打印过于频繁，这里限制为 5Hz
        if now - self.last_print_time < 0.2:
            return
        self.last_print_time = now

        r_pose = self.robot_pose
        m_pose = self.moveit_pose

        # 位置差异（m 和 mm）
        dx = m_pose.position.x - r_pose.position.x
        dy = m_pose.position.y - r_pose.position.y
        dz = m_pose.position.z - r_pose.position.z
        dist_mm = math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0

        # 欧拉角 (RPY)
        r_r, r_p, r_y = quat_to_rpy(r_pose.orientation)
        m_r, m_p, m_y = quat_to_rpy(m_pose.orientation)

        dr = m_r - r_r
        dp = m_p - r_p
        dyaw = m_y - r_y

        self.get_logger().info("\n" + "-" * 80)
        self.get_logger().info("【机器人实际末端位姿（来自 /demo_robot_status）】")
        self.get_logger().info(
            f"  位置: ({r_pose.position.x:.6f}, {r_pose.position.y:.6f}, {r_pose.position.z:.6f}) m"
        )
        self.get_logger().info(
            "  四元数: "
            f"(w={r_pose.orientation.w:.6f}, "
            f"x={r_pose.orientation.x:.6f}, "
            f"y={r_pose.orientation.y:.6f}, "
            f"z={r_pose.orientation.z:.6f})"
        )
        self.get_logger().info(
            f"  欧拉角RPY: ({math.degrees(r_r):.2f}°, {math.degrees(r_p):.2f}°, {math.degrees(r_y):.2f}°)"
        )

        self.get_logger().info("【MoveIt2 计算的末端位姿（来自 /moveit2_tcp_pose）】")
        self.get_logger().info(
            f"  位置: ({m_pose.position.x:.6f}, {m_pose.position.y:.6f}, {m_pose.position.z:.6f}) m"
        )
        self.get_logger().info(
            "  四元数: "
            f"(w={m_pose.orientation.w:.6f}, "
            f"x={m_pose.orientation.x:.6f}, "
            f"y={m_pose.orientation.y:.6f}, "
            f"z={m_pose.orientation.z:.6f})"
        )
        self.get_logger().info(
            f"  欧拉角RPY: ({math.degrees(m_r):.2f}°, {math.degrees(m_p):.2f}°, {math.degrees(m_y):.2f}°)"
        )

        self.get_logger().info("【位置差异】(MoveIt - Robot)")
        self.get_logger().info(f"  ΔX = {dx * 1000.0:+.2f} mm")
        self.get_logger().info(f"  ΔY = {dy * 1000.0:+.2f} mm")
        self.get_logger().info(f"  ΔZ = {dz * 1000.0:+.2f} mm")
        self.get_logger().info(f"  3D 距离 = {dist_mm:.2f} mm")

        # 四元数分量差
        dw = m_pose.orientation.w - r_pose.orientation.w
        dqx = m_pose.orientation.x - r_pose.orientation.x
        dqy = m_pose.orientation.y - r_pose.orientation.y
        dqz = m_pose.orientation.z - r_pose.orientation.z

        # 利用四元数内积计算整体姿态夹角（度）
        dot = (
            r_pose.orientation.w * m_pose.orientation.w
            + r_pose.orientation.x * m_pose.orientation.x
            + r_pose.orientation.y * m_pose.orientation.y
            + r_pose.orientation.z * m_pose.orientation.z
        )
        dot = max(min(dot, 1.0), -1.0)
        angle_deg = math.degrees(2.0 * math.acos(abs(dot)))

        self.get_logger().info("【姿态差异】(MoveIt - Robot)")
        self.get_logger().info(
            f"  ΔRoll  = {math.degrees(dr):+.2f}°  "
            f"ΔPitch = {math.degrees(dp):+.2f}°  "
            f"ΔYaw   = {math.degrees(dyaw):+.2f}°"
        )
        self.get_logger().info(
            f"  四元数差: Δw={dw:+.6f}, Δx={dqx:+.6f}, Δy={dqy:+.6f}, Δz={dqz:+.6f}"
        )
        self.get_logger().info(f"  姿态夹角 ≈ {angle_deg:.3f}°")

        # 记录到 CSV，便于后续分析
        now_msg = self.get_clock().now().to_msg()
        timestamp = now_msg.sec + now_msg.nanosec * 1e-9
        self.append_log_row(
            timestamp=timestamp,
            r_pose=r_pose,
            m_pose=m_pose,
            r_r=r_r,
            r_p=r_p,
            r_y=r_y,
            m_r=m_r,
            m_p=m_p,
            m_y=m_y,
            dx=dx,
            dy=dy,
            dz=dz,
            dist_mm=dist_mm,
            dw=dw,
            dqx=dqx,
            dqy=dqy,
            dqz=dqz,
            dr=dr,
            dp=dp,
            dyaw=dyaw,
            angle_deg=angle_deg,
        )

    def append_log_row(
        self,
        timestamp,
        r_pose,
        m_pose,
        r_r,
        r_p,
        r_y,
        m_r,
        m_p,
        m_y,
        dx,
        dy,
        dz,
        dist_mm,
        dw,
        dqx,
        dqy,
        dqz,
        dr,
        dp,
        dyaw,
        angle_deg,
    ):
        """将当前一次对比结果追加写入 CSV 文件。"""
        if not self.log_file:
            return

        file_existed = os.path.exists(self.log_file)
        with open(self.log_file, mode="a", newline="") as f:
            writer = csv.writer(f)
            if not file_existed:
                writer.writerow(
                    [
                        "timestamp",
                        "robot_x",
                        "robot_y",
                        "robot_z",
                        "robot_qw",
                        "robot_qx",
                        "robot_qy",
                        "robot_qz",
                        "robot_roll_rad",
                        "robot_pitch_rad",
                        "robot_yaw_rad",
                        "moveit_x",
                        "moveit_y",
                        "moveit_z",
                        "moveit_qw",
                        "moveit_qx",
                        "moveit_qy",
                        "moveit_qz",
                        "moveit_roll_rad",
                        "moveit_pitch_rad",
                        "moveit_yaw_rad",
                        "dx_m",
                        "dy_m",
                        "dz_m",
                        "dist_mm",
                        "dq_w",
                        "dq_x",
                        "dq_y",
                        "dq_z",
                        "droll_rad",
                        "dpitch_rad",
                        "dyaw_rad",
                        "angle_deg",
                    ]
                )

            writer.writerow(
                [
                    timestamp,
                    r_pose.position.x,
                    r_pose.position.y,
                    r_pose.position.z,
                    r_pose.orientation.w,
                    r_pose.orientation.x,
                    r_pose.orientation.y,
                    r_pose.orientation.z,
                    r_r,
                    r_p,
                    r_y,
                    m_pose.position.x,
                    m_pose.position.y,
                    m_pose.position.z,
                    m_pose.orientation.w,
                    m_pose.orientation.x,
                    m_pose.orientation.y,
                    m_pose.orientation.z,
                    m_r,
                    m_p,
                    m_y,
                    dx,
                    dy,
                    dz,
                    dist_mm,
                    dw,
                    dqx,
                    dqy,
                    dqz,
                    dr,
                    dp,
                    dyaw,
                    angle_deg,
                ]
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotVsMoveItPoseMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

