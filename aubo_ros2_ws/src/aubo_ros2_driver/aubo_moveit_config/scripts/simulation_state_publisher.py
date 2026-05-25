#!/usr/bin/env python3
"""
仿真模式 /robot_status 发布节点。

仿真模式下没有 aubo_state_broadcaster (依赖真实 SDK 的 onWaypoint 回调)，
/robot_status 话题无发布者，导致前端位姿卡片始终显示全零。

本节点通过订阅 /joint_states + TF 查询 base_link→tool_tcp 变换
来合成 /robot_status 消息，使前端在仿真模式下也能显示末端位姿。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import Header
from ivg_interfaces.msg import RobotStatus
import tf2_ros
from tf2_ros.buffer import Buffer


class SimulationStatePublisher(Node):
    def __init__(self):
        super().__init__("simulation_state_publisher")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_frame", "tool_tcp")
        self.declare_parameter("publish_hz", 50.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.tip_frame = self.get_parameter("tip_frame").value
        self.publish_hz = self.get_parameter("publish_hz").value

        # 关节名 (与 aubo_state_broadcaster 保持一致)
        self.joint_names = [
            "shoulder_joint", "upperArm_joint", "foreArm_joint",
            "wrist1_joint", "wrist2_joint", "wrist3_joint",
        ]

        # 缓存
        self.latest_js = None  # JointState

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 订阅 joint_states
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

        # 发布 /robot_status
        qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.status_pub = self.create_publisher(RobotStatus, "/robot_status", qos)

        dt = 1.0 / max(self.publish_hz, 1.0)
        self.pub_timer = self.create_timer(dt, self._publish_tick)

        self.get_logger().info(
            f"Simulation state publisher ready: {self.base_frame}→{self.tip_frame} @ {self.publish_hz}Hz"
        )

    def _on_joint_state(self, msg: JointState):
        self.latest_js = msg

    def _publish_tick(self):
        msg = RobotStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.is_online = True
        msg.enable = True
        msg.in_motion = False
        msg.planning_status = "idle"

        # 关节角度
        js = self.latest_js
        if js is not None and len(js.position) >= 6:
            for i, name in enumerate(self.joint_names):
                try:
                    idx = js.name.index(name)
                    rad = js.position[idx]
                except ValueError:
                    rad = 0.0
                msg.joint_position_rad[i] = rad
                msg.joint_position_deg[i] = rad * 180.0 / math.pi

        # 从 TF 获取末端位姿
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.tip_frame, rclpy.time.Time()
            )
            msg.cartesian_position = Pose()
            msg.cartesian_position.position.x = t.transform.translation.x
            msg.cartesian_position.position.y = t.transform.translation.y
            msg.cartesian_position.position.z = t.transform.translation.z
            msg.cartesian_position.orientation = t.transform.rotation

            msg.cartesian_position_xyz = Point()
            msg.cartesian_position_xyz.x = t.transform.translation.x
            msg.cartesian_position_xyz.y = t.transform.translation.y
            msg.cartesian_position_xyz.z = t.transform.translation.z

            # 四元数 → RPY
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            sinr_cosp = 2.0 * (qw * qx + qy * qz)
            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            sinp = 2.0 * (qw * qy - qz * qx)
            pitch = math.asin(max(-1.0, min(1.0, sinp)))
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            msg.cartesian_rpy = Vector3(x=roll, y=pitch, z=yaw)
        except Exception:
            pass  # TF 未就绪时跳过位姿字段

        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
