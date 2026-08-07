"""
MoveIt 客户端封装 — MoveGroup action 位姿约束规划/执行（失败分类返回）.

目标栈为 AUBO E5（规划组 manipulator_e5，链 base_link→wrist3_Link，
ompl 管线）。位姿目标写法参照 aubo_hand_eye_calibration 已验证格式：
PositionConstraint（球域容差）+ OrientationConstraint（三轴容差）→
/move_action；plan_only=true 只规划不执行（可达性验证用）。失败分类
plan_failed / unreachable / execution_failed 供执行器决策。

注意：MoveIt 在本机为源码 overlay，运行前须 source
~/ws_moveit/install/setup.bash（/opt/ros/jazzy 无 moveit 包）。
"""
import time
from typing import Tuple

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

# 规划类错误（plan_only 时视为不可达；执行模式下视为规划失败）
_PLAN_CODES = {
    MoveItErrorCodes.PLANNING_FAILED,
    MoveItErrorCodes.INVALID_MOTION_PLAN,
    MoveItErrorCodes.START_STATE_IN_COLLISION,
    MoveItErrorCodes.GOAL_IN_COLLISION,
    getattr(MoveItErrorCodes, 'NO_IK_SOLUTION', -31),
    getattr(MoveItErrorCodes, 'GOAL_VIOLATES_PATH_CONSTRAINTS', -26),
}


class MoveItClient:
    """MoveGroup 封装：位姿目标 plan&execute + 阻塞等待 + 失败分类."""

    def __init__(self, node, group_name: str = 'manipulator_e5',
                 base_frame: str = 'base_link', tool_frame: str = 'wrist3_Link',
                 velocity_scaling: float = 0.01,
                 acceleration_scaling: float = 0.01,
                 planning_time_sec: float = 5.0, planning_attempts: int = 2,
                 position_tolerance_m: float = 0.01,
                 orientation_tolerance_rad: float = 0.1):
        """
        构造客户端（不连接；首次调用时等 server）.

        Args:
            node: 宿主节点（建 action client 与日志）.
            group_name: MoveIt 规划组（AUBO E5 为 manipulator_e5）.
            base_frame: 基座系（约束表达坐标系）.
            tool_frame: TCP 连杆（wrist3_Link，位姿约束挂它）.
            velocity_scaling: 最大速度缩放（真机先压 0.01）.
            acceleration_scaling: 最大加速度缩放.
            planning_time_sec: 单次规划时限 (s).
            planning_attempts: 规划尝试次数.
            position_tolerance_m: 位置球域容差 (m).
            orientation_tolerance_rad: 姿态三轴容差 (rad).

        Returns
        -------
            无返回值（None）.

        """
        self._node = node
        self.group_name = group_name
        self.base_frame = base_frame
        self.tool_frame = tool_frame
        self.velocity_scaling = float(velocity_scaling)
        self.acceleration_scaling = float(acceleration_scaling)
        self.planning_time_sec = float(planning_time_sec)
        self.planning_attempts = int(planning_attempts)
        self.position_tolerance_m = float(position_tolerance_m)
        self.orientation_tolerance_rad = float(orientation_tolerance_rad)
        self._client = ActionClient(node, MoveGroup, '/move_action')

    def server_ready(self, timeout_sec: float = 5.0) -> bool:
        """
        等 MoveGroup action server 可用.

        Args:
            timeout_sec: 等待上限 (s).

        Returns
        -------
            True=可用.

        """
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def plan_and_execute(self, pose: Pose, plan_only: bool = False,
                         cancel_check=None) -> Tuple[bool, str]:
        """
        位姿目标规划/执行（阻塞等结果；需多线程执行器配合）.

        Args:
            pose: 目标位姿（base_frame 系）.
            plan_only: True 只规划不执行（可达性验证）.
            cancel_check: 可调用，返回 True 表示外部请求取消（轮询间隔内生效）.

        Returns
        -------
            (success, category)：category ∈ ''（成功）、'rejected'、
            'cancelled'、'plan_failed'、'unreachable'（plan_only 规划失败）、
            'execution_failed'.

        """
        goal = self._build_goal(pose, plan_only)
        send_future = self._client.send_goal_async(goal)
        if not self._wait(send_future, cancel_check):
            return False, 'cancelled'
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, 'rejected'
        result_future = handle.get_result_async()
        if not self._wait(result_future, cancel_check):
            self._client._cancel_goal_async(handle)
            return False, 'cancelled'
        wrapped = result_future.result()
        code = wrapped.result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            return True, ''
        return False, self._classify(code, plan_only)

    @staticmethod
    def _wait(future, cancel_check, period: float = 0.02) -> bool:
        """轮询等 future 完成；cancel_check 命中提前返回 False."""
        while not future.done():
            if cancel_check is not None and cancel_check():
                return False
            time.sleep(period)
        return True

    @staticmethod
    def _classify(code: int, plan_only: bool) -> str:
        """错误码 → 失败类别（MoveIt MoveItErrorCodes）."""
        if code in _PLAN_CODES:
            return 'unreachable' if plan_only else 'plan_failed'
        if code in (MoveItErrorCodes.CONTROL_FAILED,
                    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA,
                    MoveItErrorCodes.TIMED_OUT,
                    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE):
            return 'execution_failed'
        return 'plan_failed'

    def _build_goal(self, pose: Pose, plan_only: bool) -> MoveGroup.Goal:
        """组装 MoveGroup goal（位姿球域 + 三轴姿态容差，同 hand_eye 写法）."""
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.group_name
        request.num_planning_attempts = self.planning_attempts
        request.allowed_planning_time = self.planning_time_sec
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling

        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.base_frame
        position.link_name = self.tool_frame
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self.position_tolerance_m]
        position.constraint_region.primitives = [primitive]
        position.constraint_region.primitive_poses = [pose]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.base_frame
        orientation.link_name = self.tool_frame
        orientation.orientation = pose.orientation
        tol = self.orientation_tolerance_rad
        orientation.absolute_x_axis_tolerance = tol
        orientation.absolute_y_axis_tolerance = tol
        orientation.absolute_z_axis_tolerance = tol
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = bool(plan_only)
        goal.planning_options.replan = not plan_only
        if not plan_only:
            goal.planning_options.replan_attempts = 2
        return goal
