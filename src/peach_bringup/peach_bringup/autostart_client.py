"""
Autostart client: sends RunHarvest when the managed stack becomes Active.

Original "launch never auto-sends RunHarvest" red line removed per user
decision (2026-09-16). Autostart is now a deployment parameter.
Authorization semantics converge on red line 3: the operator launching
on real hardware constitutes human authorization.
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
        # 参数一行接入（config/bringup.yaml 直读 + 校验）；scene_key/intent
        # 发批时读取（热生效），wait_stack_timeout_s 构造期捕获
        self._params = attach_autostart_client(self)
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._stack_ready = False
        self._selfcheck_passed = not bool(self._params.require_selfcheck)
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
        if bool(msg.data):
            self._stack_ready = True

    def _on_selfcheck(self, msg: Bool) -> None:
        # 软门+autostart 硬等（用户裁定）：自动批须等自检绿；手动批不受限
        if bool(msg.data):
            self._selfcheck_passed = True
        else:
            self._selfcheck_passed = False
            if self._sent:
                return
            self.get_logger().warning(
                'autostart：自检转红，暂缓自动开批（等待下一轮自检通过）')

    def _tick(self) -> None:
        if self._sent or not self._stack_ready or not self._selfcheck_passed:
            if not self._stack_ready and time.monotonic() > self._deadline:
                self.get_logger().error(
                    'autostart：等待托管栈就绪超时，放弃自动开批')
                self._timer.cancel()
            elif (self._stack_ready and not self._selfcheck_passed
                    and time.monotonic() > self._deadline):
                self.get_logger().error(
                    'autostart：托管栈就绪但自检未通过，放弃自动开批'
                    '（require_selfcheck=true；手动 RunHarvest 不受限）')
                self._timer.cancel()
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
