#!/usr/bin/env python3
"""
aubo_driver_mode 状态节点 — 提供参数 + latched topic 双通道。

参数:  /aubo_mode 节点上的 aubo_driver_mode (real / simulation)
话题:  /aubo/mode (std_msgs/String, transient_local) — 无需知道节点名即可订阅

查询方式:
  ros2 param get /aubo_mode aubo_driver_mode
  ros2 topic echo /aubo/mode --once
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class AuboModeNode(Node):
    def __init__(self):
        super().__init__("aubo_mode")
        self.declare_parameter("aubo_driver_mode", "unknown")
        self.mode = self.get_parameter("aubo_driver_mode").value

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(String, "/aubo/mode", qos)

        # 立即发一次 (transient_local 命中 native 订阅者)
        self._publish()

        # 每 5s 重发 — rosbridge/WebSocket 用 volatile QoS 拿不到 latch，
        # 定时重发保证前端在连接后 5s 内收到
        self.timer = self.create_timer(5.0, self._publish)

        self.get_logger().info(f"aubo_driver_mode={self.mode}")

    def _publish(self):
        msg = String()
        msg.data = self.mode
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AuboModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
