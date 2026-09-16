"""lifecycle 旗标桥：nav2_lifecycle_manager.is_active → 闩锁 Bool 话题。

nav2_lm 只提供 manage_nodes/is_active 服务，无「全部 Active」闩锁话题
（调研核实，2026-09-16）。本桥 1 Hz 轮询 is_active，把结果发到
/peach/lifecycle/managed_nodes_activated（transient_local depth 1），
消费方（supervisor require_managed_stack 等）零改动。

清洁重写轮阶段 5：替换自研 peach_lifecycle_manager（其名单/顺序语义由
nav2_lm node_names 承接：按序 bring-up、逆序拆除；bond_timeout=0.0 管
rclpy 节点——源码守卫已核实 0=真禁用 bond；进程死检由 supervisor
HeartbeatWatchdog 承担，bondpy 升级路径成文于 REFACTORING）。
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class LifecycleFlagBridge(Node):

    def __init__(self) -> None:
        super().__init__('peach_lifecycle_flag_bridge')
        self.declare_parameter(
            'is_active_service', '/peach_lifecycle_manager/is_active')
        self.declare_parameter('poll_hz', 1.0)
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(
            Bool, '/peach/lifecycle/managed_nodes_activated', latched)
        self._client = self.create_client(
            Trigger,
            self.get_parameter('is_active_service').value)
        hz = float(self.get_parameter('poll_hz').value)
        self._last = None
        self._timer = self.create_timer(
            max(0.1, 1.0 / max(hz, 0.1)), self._poll)

    def _poll(self) -> None:
        if not self._client.wait_for_service(timeout_sec=0.0):
            return  # nav2_lm 未起：不发（消费者保持默认 false）
        request = Trigger.Request()
        future = self._client.call_async(request)
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        try:
            response = future.result()
        except Exception:  # noqa: BLE001 服务瞬断
            return
        if response is None:
            return
        active = bool(response.success)
        if active is not self._last:
            self._last = active
            self.get_logger().info(
                f'托管栈 Active={active}（桥自 nav2_lm is_active）')
        msg = Bool()
        msg.data = active
        self._pub.publish(msg)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = LifecycleFlagBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
