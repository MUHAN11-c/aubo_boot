"""
作业位导航适配：NavigateToWorksite 服务端.

仅 Lifecycle Active 才接目标. 现行 impl=reserved_stub 不调 Nav2、不发
cmd_vel，把固定座当作已到位. 本仓发布目标位姿/袋轴/作业状态，消费外部
车辆到位且静止；路径主权在外部导航栈.
"""
from __future__ import annotations

from peach_interfaces.action import NavigateToWorksite
from peach_interfaces.msg import (
    GraspDecision,
    HarvestOperationStatus,
    HarvestState,
    HarvestTargetReport,
    VehicleState,
)
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class NavigationNode(LifecycleNode):
    """NavigateToWorksite 服务端；调度层是唯一动作客户端."""

    def __init__(self):
        super().__init__('peach_navigation_node')
        self._param_listener = None
        self._params = None
        self._active = False
        self._cb = ReentrantCallbackGroup()
        self._server = None
        self._harvest_state = None
        self._grasp_decision = None
        self._vehicle_state = None
        self._site_id = ''
        self._timer = None

    def on_configure(self, state):
        del state
        try:
            from peach_navigation.navigation_parameters import peach_navigation_node
            self._param_listener = peach_navigation_node.ParamListener(self)
            self._params = self._param_listener.get_params()
            latched = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._pub_target = self.create_lifecycle_publisher(
                HarvestTargetReport, '/peach/navigation/target_report', latched)
            self._pub_arm = self.create_lifecycle_publisher(
                HarvestOperationStatus, '/peach/navigation/arm_status', latched)
            self._pub_vehicle = self.create_lifecycle_publisher(
                VehicleState, '/peach/navigation/vehicle_state', latched)
            self._sub_vehicle = self.create_subscription(
                VehicleState, '/peach/navigation/vehicle_state',
                self._on_vehicle, latched, callback_group=self._cb)
            self._sub_harvest = self.create_subscription(
                HarvestState, '/peach_task_executor/state',
                self._on_harvest, latched, callback_group=self._cb)
            self._sub_decision = self.create_subscription(
                GraspDecision, '/peach/reconstruction/grasp_decision',
                self._on_decision, latched, callback_group=self._cb)
            self._server = ActionServer(
                self, NavigateToWorksite, '~/navigate_to_worksite',
                execute_callback=self._execute,
                goal_callback=self._goal_if_active,
                cancel_callback=self._accept_cancel,
                callback_group=self._cb)
            self._timer = self.create_timer(1.0, self._publish_adapter)
            self.get_logger().info(
                f'navigation configured impl={self._params.impl} '
                f'worksite_frame={self._params.worksite_frame}; '
                'external path stack owns cmd_vel')
        except Exception as exc:  # noqa: BLE001 接线失败停在 Unconfigured
            self.get_logger().error(f'configure 失败: {exc}')
            return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._active = True
        self._publish_adapter()
        return super().on_activate(state)

    def on_deactivate(self, state):
        self._active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        del state
        self._active = False
        self._params = None
        self._param_listener = None
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        if self._server is not None:
            self._server.destroy()
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

    def _on_vehicle(self, msg: VehicleState) -> None:
        self._vehicle_state = msg

    def _on_harvest(self, msg: HarvestState) -> None:
        self._harvest_state = msg
        self._publish_adapter()

    def _on_decision(self, msg: GraspDecision) -> None:
        self._grasp_decision = msg
        self._publish_adapter()

    def _publish_adapter(self) -> None:
        """采摘→外部导航：目标报告与臂占用；stub 同时合成车辆静止."""
        if not self._active:
            return
        harvest = self._harvest_state
        decision = self._grasp_decision
        target_id = str(getattr(harvest, 'target_id', '') or '')
        report = HarvestTargetReport()
        report.header.stamp = self.get_clock().now().to_msg()
        report.header.frame_id = str(self._params.worksite_frame)
        report.target_id = target_id
        report.site_id = self._site_id
        if decision is not None and bool(decision.allowed):
            report.pose.pose.position = decision.entry
            report.bag_axis = decision.axis
        batch = int(getattr(harvest, 'batch_state', 0) or 0)
        phase = int(getattr(harvest, 'target_phase', 0) or 0)
        if batch in (6, 8):
            report.status = HarvestTargetReport.COMPLETED
        elif batch == 7:
            report.status = HarvestTargetReport.ABANDONED
        elif phase in (5, 6, 7):
            report.status = HarvestTargetReport.IN_CONTACT
        elif target_id:
            report.status = HarvestTargetReport.ASSIGNED
        else:
            report.status = HarvestTargetReport.IDLE
        self._pub_target.publish(report)

        arm = HarvestOperationStatus()
        arm.header = report.header
        arm.target_id = target_id
        arm.site_id = self._site_id
        if report.status == HarvestTargetReport.IN_CONTACT:
            arm.phase = HarvestOperationStatus.CONTACT
            arm.resume_allowed = False
        elif report.status == HarvestTargetReport.ABANDONED:
            arm.phase = HarvestOperationStatus.ABANDONED
            arm.resume_allowed = False
        elif report.status == HarvestTargetReport.COMPLETED:
            arm.phase = HarvestOperationStatus.COMPLETED
            arm.resume_allowed = True
        elif report.status == HarvestTargetReport.ASSIGNED:
            arm.phase = HarvestOperationStatus.BUSY
            arm.resume_allowed = False
        else:
            arm.phase = HarvestOperationStatus.IDLE
            arm.resume_allowed = True
        if phase == 8:
            arm.phase = HarvestOperationStatus.ARM_CLEAR
            arm.resume_allowed = True
        self._pub_arm.publish(arm)

        impl = str(self._params.impl)
        if impl == 'reserved_stub':
            vehicle = VehicleState()
            vehicle.header = report.header
            vehicle.target_id = target_id
            vehicle.site_id = self._site_id
            vehicle.matched = True
            vehicle.arrived = True
            vehicle.stationary = True
            vehicle.fresh = True
            vehicle.freshness_s = 0.0
            vehicle.message = 'reserved_stub'
            self._vehicle_state = vehicle
            self._pub_vehicle.publish(vehicle)

    def _execute(self, goal_handle):
        """reserved_stub：立即回报已到位. 其他 impl 尚未接线."""
        result = NavigateToWorksite.Result()
        impl = str(self._params.impl)
        site_id = str(goal_handle.request.site_id or '')
        self._site_id = site_id
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
        self._publish_adapter()
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
