"""
Opt-in client: send one RunHarvest after managed-stack and self-check readiness.

The launch parameter defaults to false. Scheduling a batch does not enable
execution, contact or tool output; those remain subject to ROS-side gates
and operator authorization. One wall-clock deadline covers all readiness waits.
"""
from __future__ import annotations

import time

from peach_bringup.params import attach_autostart_client
from peach_interfaces.action import RunHarvest

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from std_msgs.msg import Bool


class AutostartClient(Node):
    """Watches the managed-stack flag and sends RunHarvest when ready."""

    def __init__(self) -> None:
        """Initialize subscribers, action client, and the readiness timer."""
        super().__init__('peach_autostart_client')
        # GPL 参数接入（config/autostart_parameters.yaml）；scene_key/intent
        # 发批时读取（热生效），wait_stack_timeout_s 构造期捕获
        self._params = attach_autostart_client(self)
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._stack_ready = False
        self._selfcheck_passed = False
        self._sent = False
        self._sub = self.create_subscription(
            Bool, '/peach/lifecycle/managed_nodes_activated',
            self._on_flag, latched)
        self._selfcheck_sub = self.create_subscription(
            Bool, '/peach/observability/selfcheck_passed',
            self._on_selfcheck, latched)
        self._client = ActionClient(
            self, RunHarvest, '/peach_supervisor/run_harvest')
        self._deadline = time.monotonic() + float(
            self._params.wait_stack_timeout_s)
        self._timer = self.create_timer(0.5, self._tick)

    def _on_flag(self, msg: Bool) -> None:
        self._stack_ready = bool(msg.data)

    def _on_selfcheck(self, msg: Bool) -> None:
        # 软门+autostart 硬等（用户裁定）：自动批须等自检绿；手动批不受限
        self._selfcheck_passed = bool(msg.data)
        if not self._sent and self._params.require_selfcheck and not self._selfcheck_passed:
            self.get_logger().warning(
                'autostart：自检转红，暂缓自动开批（等待下一轮自检通过）')

    def _tick(self) -> None:
        if self._sent:
            return
        if time.monotonic() >= self._deadline:
            self.get_logger().error(
                'autostart：等待托管栈、自检或 RunHarvest 服务端超时，放弃自动开批')
            self._timer.cancel()
            return
        if not self._stack_ready or (
                self._params.require_selfcheck and not self._selfcheck_passed):
            return
        if not self._client.wait_for_server(timeout_sec=0.0):
            return
        goal = RunHarvest.Goal()
        goal.request_id = 'auto_' + time.strftime('%Y%m%dT%H%M%S')
        goal.scene_key = str(self._params.scene_key)
        goal.intent = int(self._params.intent)
        self.get_logger().info(
            f'autostart：托管栈就绪，自动开批 {goal.request_id}'
            f'（scene_key={goal.scene_key}）')
        self._client.send_goal_async(goal)
        self._sent = True
        self._timer.cancel()


def main(argv=None) -> None:
    """Entry point for the autostart client node."""
    rclpy.init(args=argv)
    node = AutostartClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
