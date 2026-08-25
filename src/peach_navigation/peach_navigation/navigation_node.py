"""
作业位导航适配：NavigateToWorksite 服务端.

仅 Lifecycle Active 才接目标. 现行 impl=reserved_stub 不调 Nav2、不发
cmd_vel，把固定座当作已到位. SLAM / AMCL / 代价地图预留.
"""
from __future__ import annotations

from peach_interfaces.action import NavigateToWorksite
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn


class NavigationNode(LifecycleNode):
    """NavigateToWorksite 服务端；调度层是唯一客户端."""

    def __init__(self):
        super().__init__('peach_navigation_node')
        self.declare_parameter('impl', 'reserved_stub')
        self.declare_parameter('worksite_frame', 'base_link')
        self._active = False
        self._cb = ReentrantCallbackGroup()
        self._server = None

    def on_configure(self, state):
        del state
        self._server = ActionServer(
            self, NavigateToWorksite, '~/navigate_to_worksite',
            execute_callback=self._execute,
            goal_callback=self._goal_if_active,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb)
        impl = str(self.get_parameter('impl').value)
        self.get_logger().info(
            f'navigation configured impl={impl}; Nav2 reserved')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._active = True
        return super().on_activate(state)

    def on_deactivate(self, state):
        self._active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        del state
        self._active = False
        self._server = None
        return TransitionCallbackReturn.SUCCESS

    def _goal_if_active(self, goal_request):
        del goal_request
        if not self._active:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _accept_cancel(self, cancel_request):
        del cancel_request
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        """reserved_stub：立即回报已到位. 其他 impl 尚未接线."""
        result = NavigateToWorksite.Result()
        impl = str(self.get_parameter('impl').value)
        site_id = str(goal_handle.request.site_id or '')
        feedback = NavigateToWorksite.Feedback()
        feedback.progress = 1.0
        feedback.message = impl
        goal_handle.publish_feedback(feedback)
        if impl != 'reserved_stub':
            result.arrived = False
            result.failure_code = 'impl_not_wired'
            self.get_logger().error(
                f'navigation impl={impl} not wired; refusing {site_id or "-"}')
            goal_handle.abort()
            return result
        result.arrived = True
        result.failure_code = 'reserved_stub'
        goal_handle.succeed()
        return result


def main(args=None):
    """启动导航适配节点."""
    rclpy.init(args=args)
    node = NavigationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
