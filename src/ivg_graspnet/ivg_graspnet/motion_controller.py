#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
抓取运动控制模块（MoveIt2 客户端，纯库类）.

提供 move_to_pose（关节空间规划执行）与 run_grasp_approach（XY→姿态→Z 下降的
抓取接近）。本工作区适配与修复：

- 规划组 / 末端 link 参数化，默认对齐本区 aubo_e5_moveit_config
  （group=manipulator_e5，ee_link=tcp，frame=base_link）。
- 所有异步等待改为 add_done_callback + Event 的 wait_future，
  节点可由外部 MultiThreadedExecutor spin（旧版 rclpy.spin_until_future_complete
  在节点已入 executor 时会抛 "Node has already been added to an executor"）。
"""

from __future__ import annotations

import threading
import time

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

# 模块默认常量（对齐本区 aubo_e5_moveit_config/aubo_e5.srdf）
DEFAULT_GROUP_NAME = 'manipulator_e5'
DEFAULT_BASE_FRAME = 'base_link'
DEFAULT_EE_LINK = 'tcp'
CARTESIAN_MAX_POINTS_FOR_EXECUTION = 60  # 笛卡尔轨迹最大点数；超过则回退关节空间
ARC_PATH_MAX_RETRIES = 3
ARC_PATH_RETRY_DELAY_SEC = 0.5

# 抓取坐标系下绕 Z 轴 180°（approach 不变，绕 approach 翻转）
_QUAT_Z_180 = (0.0, 0.0, 1.0, 0.0)


def wait_future(future, timeout_sec: float):
    """阻塞等待 rclpy Future 完成（executor 无关，返回结果或 None=超时）."""
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout_sec):
        return None
    return future.result()


class GraspMotionController(Node):
    """抓取运动控制器：MoveGroup action / CartesianPath 服务 / ExecuteTrajectory."""

    def __init__(self, node_name: str = 'grasp_motion_controller'):
        super().__init__(node_name)
        self.declare_parameter('planning_group', DEFAULT_GROUP_NAME)
        self.declare_parameter('base_frame', DEFAULT_BASE_FRAME)
        self.declare_parameter('ee_link', DEFAULT_EE_LINK)
        self.group_name = self.get_parameter('planning_group').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_link = self.get_parameter('ee_link').value

        self._move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    # ------------------------------------------------------------------
    def move_to_pose(
        self,
        target_pose: Pose,
        velocity_scaling: float = 0.15,
        acceleration_scaling: float = 0.1,
    ) -> bool:
        """关节空间规划并执行到位姿目标（位置球 2mm + 姿态容差 0.01 rad）."""
        logger = self.get_logger()
        logger.info(
            '[关节空间] 开始位姿规划: '
            f'group={self.group_name}, ee={self.ee_link}, '
            f'vel={velocity_scaling:.2f}, acc={acceleration_scaling:.2f}'
        )
        stamp = self.get_clock().now().to_msg()
        goal_constraints = _build_pose_goal_constraints(
            self.ee_link, target_pose, self.base_frame, stamp, 0.002, 0.01
        )
        req = MotionPlanRequest()
        req.workspace_parameters.header = Header(frame_id=self.base_frame, stamp=stamp)
        req.start_state = RobotState()
        req.group_name = self.group_name
        req.goal_constraints = [goal_constraints]
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = velocity_scaling
        req.max_acceleration_scaling_factor = acceleration_scaling

        opts = PlanningOptions()
        opts.plan_only = False

        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            logger.error('MoveGroup action /move_action 不可用，请确认 move_group 已启动')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options = opts
        future = self._move_group_client.send_goal_async(goal_msg)
        goal_handle = wait_future(future, timeout_sec=10.0)
        if goal_handle is None:
            logger.error('MoveGroup send_goal 超时')
            return False
        if not goal_handle.accepted:
            logger.error('MoveGroup goal 被拒绝')
            return False

        result = wait_future(goal_handle.get_result_async(), timeout_sec=60.0)
        if result is None:
            logger.error('MoveGroup 执行超时 (60 s)')
            return False
        if result.result.error_code.val != 1:
            logger.error(f'关节空间到位姿失败: error_code={result.result.error_code.val}')
            return False
        logger.info('[关节空间] 到位姿执行完成')
        return True

    # ------------------------------------------------------------------
    def run_grasp_approach(
        self,
        pose_ee: Pose,
        height_above: float = 0.05,
        velocity_scaling: float = 0.15,
        acceleration_scaling: float = 0.1,
    ) -> bool:
        """
        抓取接近：当前 → 目标XY+安全高度（保持姿态）→ 切换抓取姿态 → Z 下降.

        笛卡尔规划前对 GraspNet 姿态做局部 Z 轴 180° 修正；回退关节空间时撤销。
        """
        logger = self.get_logger()
        pose_for_plan = _apply_grasp_z_flip_180(pose_ee)
        current_pose = self._get_current_ee_pose()
        gx, gy, gz = pose_for_plan.position.x, pose_for_plan.position.y, pose_for_plan.position.z
        z_above = gz + height_above
        current_ori = _pose_from_pose_msg(current_pose)[1]
        grasp_ori = _pose_from_pose_msg(pose_for_plan)[1]
        grasp_ori_short = _quat_same_hemisphere(current_ori, grasp_ori)
        p1 = Pose()
        _copy_pose(p1, (gx, gy, z_above), current_ori)
        p2 = Pose()
        _copy_pose(p2, (gx, gy, z_above), grasp_ori_short)
        p3 = Pose()
        _copy_pose(p3, (gx, gy, gz), grasp_ori_short)
        waypoints = [current_pose, p1, p2, p3]
        logger.info('抓取接近 waypoints: 4 点 (当前 → 上方同姿态 → 上方抓取姿态 → 最终)')

        resp = None
        for attempt in range(1, ARC_PATH_MAX_RETRIES + 1):
            resp = self._compute_cartesian_path(waypoints)
            if resp is None:
                return False
            logger.info(
                f'抓取接近笛卡尔路径完成度: {resp.fraction * 100.0:.2f}% '
                f'(尝试 {attempt}/{ARC_PATH_MAX_RETRIES})'
            )
            if resp.fraction >= 1.0:
                break
            if attempt < ARC_PATH_MAX_RETRIES:
                time.sleep(ARC_PATH_RETRY_DELAY_SEC)
        if resp is None:
            return False
        if resp.fraction < 1.0:
            segment_descriptions = [
                'waypoint[0]→[1]（当前 → 上方同姿态，笛卡尔 XY）',
                'waypoint[1]→[2]（上方同姿态 → 上方抓取姿态，姿态旋转）',
                'waypoint[2]→[3]（上方抓取姿态 → 最终，笛卡尔 Z 下降）',
            ]
            segment_idx = min(2, int(resp.fraction * 3))
            logger.warning(
                f'笛卡尔路径未达 100%, fraction={resp.fraction:.2f}；'
                f'截断发生在第 {segment_idx + 1} 段: {segment_descriptions[segment_idx]}'
            )
            target_pose = _pose_unflip_if_needed(p3)
            return self.move_to_pose(
                target_pose,
                velocity_scaling=velocity_scaling,
                acceleration_scaling=acceleration_scaling,
            )
        if not resp.solution.joint_trajectory.points:
            logger.error('笛卡尔路径解为空')
            return False
        num_points = len(resp.solution.joint_trajectory.points)
        if num_points > CARTESIAN_MAX_POINTS_FOR_EXECUTION:
            logger.warning(
                f'笛卡尔轨迹点数过多 ({num_points} > {CARTESIAN_MAX_POINTS_FOR_EXECUTION})，'
                '改用关节空间到位姿目标'
            )
            target_pose = _pose_unflip_if_needed(p3)
            return self.move_to_pose(
                target_pose,
                velocity_scaling=velocity_scaling,
                acceleration_scaling=acceleration_scaling,
            )
        if not self._execute_trajectory(resp.solution):
            return False
        logger.info('抓取接近运动完成')
        return True

    # ------------------------------------------------------------------
    def _get_current_ee_pose(self):
        """TF 获取当前末端位姿（base_frame ← ee_link）；20 次重试后仍失败则抛出."""
        logger = self.get_logger()
        logger.info(f'[状态获取] 查询当前末端位姿: TF {self.base_frame} -> {self.ee_link}')
        t = None
        for attempt in range(20):
            try:
                t = self._tf_buffer.lookup_transform(
                    self.base_frame, self.ee_link,
                    rclpy.time.Time(), rclpy.duration.Duration(seconds=0.12),
                )
                break
            except Exception:
                if attempt < 19:
                    time.sleep(0.03)
                else:
                    raise
        current_pose = Pose()
        current_pose.position.x = t.transform.translation.x
        current_pose.position.y = t.transform.translation.y
        current_pose.position.z = t.transform.translation.z
        current_pose.orientation = t.transform.rotation
        return current_pose

    def _compute_cartesian_path(self, waypoints):
        """调 /compute_cartesian_path（max_step 0.01m，避障开）."""
        logger = self.get_logger()
        logger.info(f'[笛卡尔] 调用 /compute_cartesian_path: waypoints={len(waypoints)}')
        if not self._cartesian_client.wait_for_service(timeout_sec=15.0):
            logger.error('服务 /compute_cartesian_path 不可用')
            return None
        req = GetCartesianPath.Request()
        req.header = Header()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state = RobotState()
        req.group_name = self.group_name
        req.link_name = self.ee_link
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = 0.0
        req.avoid_collisions = True
        req.path_constraints = Constraints()
        if hasattr(req, 'cartesian_speed_limited_link'):
            req.cartesian_speed_limited_link = ''
        if hasattr(req, 'max_cartesian_speed'):
            req.max_cartesian_speed = 0.0
        resp = wait_future(self._cartesian_client.call_async(req), timeout_sec=30.0)
        if resp is None:
            logger.error('compute_cartesian_path 超时')
            return None
        logger.info(
            f'[笛卡尔] 规划返回: fraction={resp.fraction:.3f}, '
            f'points={len(resp.solution.joint_trajectory.points)}'
        )
        return resp

    def _execute_trajectory(self, trajectory: RobotTrajectory) -> bool:
        """执行 MoveIt 规划出的关节轨迹."""
        logger = self.get_logger()
        if not self._execute_client.wait_for_server(timeout_sec=5.0):
            logger.error('Action /execute_trajectory 不可用')
            return False
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        future = self._execute_client.send_goal_async(goal_msg)
        goal_handle = wait_future(future, timeout_sec=5.0)
        if goal_handle is None:
            logger.error('send_goal 超时')
            return False
        if not goal_handle.accepted:
            logger.error('ExecuteTrajectory goal 被拒绝')
            return False
        result = wait_future(goal_handle.get_result_async(), timeout_sec=60.0)
        if result is None:
            logger.error('ExecuteTrajectory 执行超时 (60 s)')
            return False
        if result.result.error_code.val != 1:
            logger.error(f'ExecuteTrajectory 失败: error_code={result.result.error_code.val}')
            return False
        logger.info('[轨迹执行] 执行成功')
        return True


# ----------------------------------------------------------------------
# 位姿工具
# ----------------------------------------------------------------------
def _pose_from_pose_msg(pose: Pose):
    """Pose → ((x,y,z), (qx,qy,qz,qw))."""
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def _copy_pose(dest: Pose, pos, ori) -> None:
    dest.position.x, dest.position.y, dest.position.z = pos[0], pos[1], pos[2]
    dest.orientation.x = ori[0]
    dest.orientation.y = ori[1]
    dest.orientation.z = ori[2]
    dest.orientation.w = ori[3]


def _quat_same_hemisphere(q_ref, q):
    """返回与 q 同旋转、与 q_ref 同半球（点积≥0）的四元数，避免 180° 长路径."""
    dot = float(np.dot(np.asarray(q_ref, dtype=float), np.asarray(q, dtype=float)))
    if dot >= 0:
        return q
    return tuple(-np.asarray(q, dtype=float))


def _quat_mul(q1, q2):
    """四元数乘法 q1 * q2（局部系下施加 q2），(qx,qy,qz,qw)."""
    return tuple((Rotation.from_quat(q1) * Rotation.from_quat(q2)).as_quat())


def _apply_grasp_z_flip_180(pose: Pose) -> Pose:
    """局部 Z 轴 180° 修正 GraspNet 与末端 Z 方向约定不一致导致的反转."""
    ori = _pose_from_pose_msg(pose)[1]
    new_ori = _quat_mul(ori, _QUAT_Z_180)
    out = Pose()
    out.position.x = pose.position.x
    out.position.y = pose.position.y
    out.position.z = pose.position.z
    out.orientation.x = new_ori[0]
    out.orientation.y = new_ori[1]
    out.orientation.z = new_ori[2]
    out.orientation.w = new_ori[3]
    return out


def _pose_unflip_if_needed(pose: Pose) -> Pose:
    """关节空间回退时撤销 180° 修正."""
    out = Pose()
    out.position.x = pose.position.x
    out.position.y = pose.position.y
    out.position.z = pose.position.z
    ori_unflip = _quat_mul(
        (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ),
        _QUAT_Z_180,
    )
    out.orientation.x = ori_unflip[0]
    out.orientation.y = ori_unflip[1]
    out.orientation.z = ori_unflip[2]
    out.orientation.w = ori_unflip[3]
    return out


def _build_pose_goal_constraints(
    link_name, pose, frame_id, stamp=None,
    position_tolerance=0.002, orientation_tolerance=0.01,
):
    """位姿 → MoveIt goal_constraints（位置球 + 姿态欧拉角容差）."""
    if stamp is None:
        stamp = rclpy.time.Time().to_msg()
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp

    pos_constraint = PositionConstraint()
    pos_constraint.header = header
    pos_constraint.link_name = link_name
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [float(position_tolerance)]
    region = BoundingVolume()
    region.primitives = [sphere]
    center = Pose()
    center.position.x = pose.position.x
    center.position.y = pose.position.y
    center.position.z = pose.position.z
    center.orientation.w = 1.0
    region.primitive_poses = [center]
    pos_constraint.constraint_region = region
    pos_constraint.weight = 1.0

    ori_constraint = OrientationConstraint()
    ori_constraint.header = header
    ori_constraint.link_name = link_name
    ori_constraint.orientation = pose.orientation
    ori_constraint.absolute_x_axis_tolerance = orientation_tolerance
    ori_constraint.absolute_y_axis_tolerance = orientation_tolerance
    ori_constraint.absolute_z_axis_tolerance = orientation_tolerance
    ori_constraint.parameterization = OrientationConstraint.XYZ_EULER_ANGLES
    ori_constraint.weight = 1.0

    constraints = Constraints()
    constraints.position_constraints = [pos_constraint]
    constraints.orientation_constraints = [ori_constraint]
    return constraints
