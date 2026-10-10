"""
Lifecycle flag bridge: nav2_lifecycle_manager.is_active → latched Bool.

The nav2 lifecycle manager only exposes manage_nodes/is_active services,
with no latched "all Active" topic (verified 2026-09-16). This bridge polls
is_active at 1 Hz and publishes the result to
/peach/lifecycle/managed_nodes_activated (transient_local depth 1),
keeping consumers (supervisor require_managed_stack etc.) unchanged.
"""
from __future__ import annotations

from peach_bringup.params import attach_flag_bridge
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
    """Bridges nav2_lm is_active service to a latched Bool topic."""

    def __init__(self) -> None:
        """Attach parameters, create publisher, service client, and timer."""
        super().__init__('peach_lifecycle_flag_bridge')
        # 参数一行接入（config/bringup.yaml 直读 + 校验）；服务名/频率构造期捕获
        self._params = attach_flag_bridge(self)
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(
            Bool, '/peach/lifecycle/managed_nodes_activated', latched)
        self._client = self.create_client(
            Trigger, self._params.is_active_service)
        hz = float(self._params.poll_hz)
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
    """Entry point for the lifecycle flag bridge node."""
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
