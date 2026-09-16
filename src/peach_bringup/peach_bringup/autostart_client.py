"""autostart 客户端：托管栈 Active 后自动发 RunHarvest（清洁重写轮阶段 5）。

原「launch 绝不自动 RunHarvest」红线已按用户核定删除（2026-09-16）：
autostart 成为部署参数。授权语义收拢红线 3——real 上操作员发起
launch/操作台指令即人的授权。

request_id 自动生成 auto_<UTC 时间戳>（不复用）；scene_key 用 launch 参数。
mock/real 同一管线；真机部署档由操作员决定是否开启本客户端。
"""
from __future__ import annotations

import time

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

from peach_interfaces.action import RunHarvest


class AutostartClient(Node):

    def __init__(self) -> None:
        super().__init__('peach_autostart_client')
        self.declare_parameter('scene_key', 'default')
        self.declare_parameter('intent', 0)  # INTENT_PICK_ALL
        self.declare_parameter('wait_stack_timeout_s', 90.0)
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._stack_ready = False
        self._sent = False
        self._sub = self.create_subscription(
            Bool, '/peach/lifecycle/managed_nodes_activated',
            self._on_flag, latched)
        self._client = ActionClient(
            self, RunHarvest, '/peach_supervisor/run_harvest')
        self._deadline = time.monotonic() + float(
            self.get_parameter('wait_stack_timeout_s').value)
        self._timer = self.create_timer(0.5, self._tick)

    def _on_flag(self, msg: Bool) -> None:
        if bool(msg.data):
            self._stack_ready = True

    def _tick(self) -> None:
        if self._sent or not self._stack_ready:
            if not self._stack_ready and time.monotonic() > self._deadline:
                self.get_logger().error(
                    'autostart：等待托管栈就绪超时，放弃自动开批')
                self._timer.cancel()
            return
        if not self._client.wait_for_server(timeout_sec=0.0):
            return
        goal = RunHarvest.Goal()
        goal.request_id = 'auto_' + time.strftime('%Y%m%dT%H%M%S')
        goal.scene_key = str(self.get_parameter('scene_key').value)
        goal.intent = int(self.get_parameter('intent').value)
        self.get_logger().info(
            f'autostart：托管栈就绪，自动开批 {goal.request_id}'
            f'（scene_key={goal.scene_key}）')
        self._client.send_goal_async(goal)
        self._sent = True
        self._timer.cancel()


def main(argv=None) -> None:
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
