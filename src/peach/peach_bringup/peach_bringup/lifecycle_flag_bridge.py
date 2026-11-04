"""
Lifecycle flag bridge: nav2_lifecycle_manager.is_active → latched Bool.

The nav2 lifecycle manager only exposes manage_nodes/is_active services,
with no latched "all Active" topic (verified 2026-09-16). This bridge polls
is_active at 1 Hz and publishes the result to
/peach/lifecycle/managed_nodes_activated (transient_local depth 1),
keeping consumers (supervisor require_managed_stack etc.) unchanged.
"""
from __future__ import annotations

import time

from peach_bringup.params import attach_flag_bridge
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
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
        # GPL 参数接入（config/flag_bridge_parameters.yaml）；服务名/频率构造期捕获
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
        period = max(0.1, 1.0 / max(hz, 0.1))
        self._reply_timeout = max(1.0, 2 * period)
        self._pending = None
        self._deadline = 0.0
        self._timer = self.create_timer(period, self._poll)

    def _poll(self) -> None:
        ready = self._client.service_is_ready()
        now = time.monotonic()
        if self._pending is not None:
            if ready and now < self._deadline:
                return
            self._client.remove_pending_request(self._pending)
            self._pending = None
            self._publish(False)
        elif not ready:
            self._publish(False)
        if ready:
            self._deadline = now + self._reply_timeout
            self._pending = self._client.call_async(Trigger.Request())
            self._pending.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        if future is not self._pending:
            return
        self._pending = None
        try:
            response = future.result()
        except Exception:  # noqa: BLE001 服务瞬断
            response = None
        self._publish(bool(response and response.success and time.monotonic() < self._deadline))

    def _publish(self, active: bool) -> None:
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
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
