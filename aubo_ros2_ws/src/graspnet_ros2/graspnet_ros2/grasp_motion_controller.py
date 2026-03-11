#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
抓取运动控制模块：通过 MoveIt2 笛卡尔路径实现三阶段一条轨迹（XY → 姿态旋转 → Z 下降）。

与 moveit_arcline_demo.cpp / runArcPathSequence 一致：4 个 waypoints，一次 computeCartesianPath，
一次 execute；不调用 /move_to_pose 等现有服务，仅用 MoveIt2 的 compute_cartesian_path 服务与
execute_trajectory action。

MoveIt2 源码参考：/home/mu/ws_moveit/src/moveit2（move_group 默认 capability：
MoveGroupCartesianPathService，服务名 compute_cartesian_path，见 capability_names.h /
cartesian_path_service_capability.cpp）。运行本模块前需先启动 move_group 节点。
"""

from __future__ import annotations

import time
from typing import Optional, TYPE_CHECKING

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    RobotState,
    RobotTrajectory,
    Constraints,
    MotionPlanRequest,
    PlanningOptions,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

if TYPE_CHECKING:
    from tf2_ros import Buffer


# 与 move_to_pose_server / moveit_arcline_demo 一致
DEFAULT_GROUP_NAME = 'manipulator'   # MoveIt 规划组名
DEFAULT_BASE_FRAME = 'base_link'      # 规划与 waypoints 的参考坐标系
DEFAULT_EE_LINK = 'tool_tcp'          # aubo SRDF 末端 link，空则 move_group 用 group tip
DEFAULT_JOINT_NAMES = [
    # aubo 6 轴关节名，与 move_group 的 manipulator 组一致
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
]
EEF_STEP = 0.01          # 末端执行器笛卡尔步长 (m)，路径插值分辨率
JUMP_THRESHOLD = 0.0     # 关节空间跳变阈值，0 表示不允许大跳变
COMPUTE_CARTESIAN_PATH_SERVICE = '/compute_cartesian_path'  # 本环境 move_group 实际发布名（无 /move_group 前缀）
EXECUTE_TRAJECTORY_ACTION = '/execute_trajectory'
MOVE_GROUP_ACTION = '/move_action'  # 关节空间规划并执行；若 move_group 带命名空间则改为 /move_group/move_action
# 笛卡尔轨迹点数超过此值时改为关节空间回退（避免轨迹过长导致执行慢或异响）
CARTESIAN_MAX_POINTS_FOR_EXECUTION = 60


def _make_robot_state_from_joint_state(joint_state: JointState, frame_id: str = '') -> RobotState:
    """从 sensor_msgs/JointState 构造 moveit_msgs/RobotState（仅 joint_state 部分）。"""
    state = RobotState()  # MoveIt 规划起点状态
    state.joint_state.header = Header()
    state.joint_state.header.frame_id = frame_id
    state.joint_state.header.stamp = rclpy.time.Time().to_msg()
    state.joint_state.name = list(joint_state.name)
    state.joint_state.position = list(joint_state.position)
    state.joint_state.velocity = list(joint_state.velocity) if joint_state.velocity else []
    state.joint_state.effort = list(joint_state.effort) if joint_state.effort else []
    return state


def _pose_from_pose_msg(pose: Pose) -> tuple:
    """返回 (pos, ori)：pos=(x,y,z)，ori=(qx,qy,qz,qw)，用于 waypoint 构造。"""
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def _copy_pose(dest: Pose, pos: tuple, ori: tuple) -> None:
    """将位置 pos=(x,y,z) 和姿态 ori=(qx,qy,qz,qw) 写入 dest。"""
    dest.position.x, dest.position.y, dest.position.z = pos[0], pos[1], pos[2]
    dest.orientation.x, dest.orientation.y, dest.orientation.z, dest.orientation.w = ori[0], ori[1], ori[2], ori[3]


def _quat_same_hemisphere(q_ref: tuple, q: tuple) -> tuple:
    """返回与 q 同旋转的四元数，且与 q_ref 在同一半球（点积≥0），避免插值走 180° 长路径。
    q_ref, q 均为 (qx, qy, qz, qw)。
    """
    dot = q_ref[0] * q[0] + q_ref[1] * q[1] + q_ref[2] * q[2] + q_ref[3] * q[3]
    if dot >= 0:
        return q
    return (-q[0], -q[1], -q[2], -q[3])


def _quat_mul(q1: tuple, q2: tuple) -> tuple:
    """四元数乘法 q1 * q2，均为 (qx, qy, qz, qw)。"""
    x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
    x2, y2, z2, w2 = q2[0], q2[1], q2[2], q2[3]
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


# 抓取坐标系下绕 Z 轴转 180° 的四元数（approach 方向不变，绕 approach 翻转）
_QUAT_Z_180 = (0.0, 0.0, 1.0, 0.0)


def _apply_grasp_z_flip_180(pose: Pose) -> Pose:
    """在抓取位姿的局部坐标系下绕 Z 轴旋转 180°，返回新 Pose（位置不变）。
    用于修正 GraspNet 与机械臂末端 Z（approach）方向约定不一致导致的 180° 反转。
    """
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


def _build_pose_goal_constraints(
    link_name: str,
    pose: Pose,
    frame_id: str,
    stamp=None,
    position_tolerance: float = 0.002,
    orientation_tolerance: float = 0.01,
) -> Constraints:
    """由位姿 (位置+四元数) 构造 MoveIt goal_constraints：位置球体 + 姿态容差。"""
    if stamp is None:
        stamp = rclpy.time.Time().to_msg()
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp

    pos_constraint = PositionConstraint()
    pos_constraint.header = header
    pos_constraint.link_name = link_name
    pos_constraint.target_point_offset.x = 0.0
    pos_constraint.target_point_offset.y = 0.0
    pos_constraint.target_point_offset.z = 0.0
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
    center.orientation.x = 0.0
    center.orientation.y = 0.0
    center.orientation.z = 0.0
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


def _run_joint_space_move_to_pose(
    node: Node,
    target_pose: Pose,
    base_frame: str,
    ee_link: str,
    group_name: str,
    start_state: RobotState,
    *,
    position_tolerance: float = 0.002,
    orientation_tolerance: float = 0.01,
    allowed_planning_time: float = 5.0,
    velocity_scaling: float = 0.3,
) -> bool:
    """
    关节空间规划并执行到目标位姿（位置+四元数）。用于笛卡尔失败时的回退。
    通过 MoveGroup action 发送 MotionPlanRequest（goal_constraints 为位姿约束），plan_only=False 即规划并执行。
    """
    logger = node.get_logger()
    stamp = node.get_clock().now().to_msg()
    goal_constraints = _build_pose_goal_constraints(
        ee_link, target_pose, base_frame, stamp, position_tolerance, orientation_tolerance
    )
    req = MotionPlanRequest()
    req.workspace_parameters.header = Header(frame_id=base_frame, stamp=stamp)
    req.start_state = start_state
    req.group_name = group_name
    req.goal_constraints = [goal_constraints]
    req.num_planning_attempts = 10
    req.allowed_planning_time = allowed_planning_time
    req.max_velocity_scaling_factor = velocity_scaling
    req.max_acceleration_scaling_factor = 1.0

    opts = PlanningOptions()
    opts.plan_only = False

    action_client = ActionClient(node, MoveGroup, MOVE_GROUP_ACTION)
    if not action_client.wait_for_server(timeout_sec=10.0):
        logger.error(f'MoveGroup action {MOVE_GROUP_ACTION} 不可用，请确认 move_group 已启动且发布该 action')
        return False

    goal_msg = MoveGroup.Goal()
    goal_msg.request = req
    goal_msg.planning_options = opts

    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done():
        logger.error('MoveGroup send_goal 超时')
        return False
    try:
        goal_handle = future.result()
    except Exception as e:
        logger.error(f'MoveGroup send_goal 异常: {e}')
        return False
    if not goal_handle.accepted:
        logger.error('MoveGroup goal 被拒绝')
        return False

    result_future = goal_handle.get_result_async()
    timeout_sec = 60.0
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=int(timeout_sec))
    while node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if result_future.done():
            break
    if not result_future.done():
        logger.error(f'MoveGroup 执行超时 ({timeout_sec:.0f} s)')
        return False
    try:
        result = result_future.result().result
    except Exception as e:
        logger.error(f'MoveGroup result 异常: {e}')
        return False

    if result.error_code.val != 1:  # SUCCESS
        logger.error(f'关节空间到位姿失败: error_code={result.error_code.val}')
        return False
    logger.info('关节空间到位姿完成')
    return True


def run_grasp_approach(
    node: Node,
    pose_ee: Pose,
    height_above: float = 0.05,
    *,
    base_frame: str = DEFAULT_BASE_FRAME,
    ee_link: str = DEFAULT_EE_LINK,
    group_name: str = DEFAULT_GROUP_NAME,
    joint_names: Optional[list[str]] = None,
    velocity_scaling: float = 0.3,
    eef_step: float = EEF_STEP,
    jump_threshold: float = JUMP_THRESHOLD,
    tf_buffer: Optional['Buffer'] = None,
    flip_grasp_z_180: bool = True,
) -> bool:
    """
    执行抓取接近：一条笛卡尔路径（4 waypoints），一次规划一次执行。

    Waypoints:
      [0] 当前末端位姿
      [1] (gx, gy, z_above)，姿态与当前相同（笛卡尔 XY）
      [2] (gx, gy, z_above)，姿态为 pose_ee（姿态旋转）
      [3] pose_ee 完整位姿（笛卡尔 Z 下降）

    Args:
        node: rclpy 节点（用于 TF、服务、action、日志）
        pose_ee: base_link 下 end-effector 目标位姿（已含 gripper_tip 补偿）
        height_above: 抓取点上方安全高度 (m)，z_above = pose_ee.z + height_above
        base_frame: 规划/waypoints 参考系
        ee_link: 末端 link 名（move_group 的 tip）
        group_name: 规划组名
        joint_names: 用于从 /joint_states 过滤出 manipulator 关节；None 则用 DEFAULT_JOINT_NAMES
        velocity_scaling: 执行时速度缩放（若 move_group 支持）
        eef_step: 笛卡尔步长 (m)
        jump_threshold: 跳变阈值，0 表示不允许
        tf_buffer: 可选；若传入则复用该 TF 缓存（避免新建 buffer 收不到 base_link），否则内部新建并等待
        flip_grasp_z_180: 若 True，在抓取局部坐标系下绕 Z（approach）转 180° 再规划，用于修正 GraspNet 与末端约定不一致导致的 180° 反转

    Returns:
        True 表示成功，False 表示失败。
    """
    joint_names = joint_names or DEFAULT_JOINT_NAMES
    logger = node.get_logger()

    # 可选：绕抓取 Z 轴 180° 修正（GraspNet 与末端 approach 约定不一致时避免多转 180°）
    if flip_grasp_z_180:
        pose_ee = _apply_grasp_z_flip_180(pose_ee)

    # 1) 当前末端位姿（TF base_frame -> ee_link），优先复用传入的 tf_buffer
    from tf2_ros import Buffer, TransformListener
    if tf_buffer is not None:
        buffer = tf_buffer
    else:
        buffer = Buffer()
        _listener = TransformListener(buffer, node)
        time.sleep(0.5)
    try:
        for attempt in range(10):
            try:
                t = buffer.lookup_transform(
                    base_frame, ee_link, rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)
                )
                break
            except Exception:
                if attempt < 9:
                    rclpy.spin_once(node, timeout_sec=0.2)
                    time.sleep(0.2)
                else:
                    raise
        current_pose = Pose()  # 当前末端在 base_frame 下的位姿，waypoints[0]
        current_pose.position.x = t.transform.translation.x
        current_pose.position.y = t.transform.translation.y
        current_pose.position.z = t.transform.translation.z
        current_pose.orientation = t.transform.rotation
    except Exception as e:
        logger.error(f'获取当前末端位姿失败 (TF {base_frame} -> {ee_link}): {e}')
        return False

    gx = pose_ee.position.x   # 抓取目标 x (m)
    gy = pose_ee.position.y   # 抓取目标 y (m)
    gz = pose_ee.position.z   # 抓取目标 z (m)
    z_above = gz + height_above  # 安全高度：抓取点正上方 z，waypoints[1][2] 使用

    # 2) 当前关节状态（用于 compute_cartesian_path 的 start_state）
    joint_state_msg = [None]  # 用列表包装以便在闭包中赋值

    def cb(msg: JointState):
        if joint_state_msg[0] is None:
            name_to_pos = dict(zip(msg.name, msg.position))  # 关节名 -> 关节位置，便于按 joint_names 顺序取
            if not all(n in name_to_pos for n in joint_names):
                return
            js = JointState()  # 仅含 manipulator 关节、且按 joint_names 顺序
            js.header = msg.header
            js.name = list(joint_names)
            js.position = [name_to_pos[n] for n in joint_names]
            joint_state_msg[0] = js

    sub = node.create_subscription(JointState, '/joint_states', cb, 10)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if joint_state_msg[0] is not None:
            break
    node.destroy_subscription(sub)
    if joint_state_msg[0] is None:
        logger.error('未收到 /joint_states，无法构造 start_state')
        return False

    start_state = _make_robot_state_from_joint_state(joint_state_msg[0], base_frame)  # 规划起点

    # 3) 构造 4 个 waypoints（当前 -> 上方同姿态 -> 上方抓取姿态 -> 最终）
    # 抓取姿态四元数做“同半球”处理，避免 [1]->[2] 或 [2]->[3] 插值时走 180° 长路径（Z 轴反转）
    current_ori = _pose_from_pose_msg(current_pose)[1]
    grasp_ori = _pose_from_pose_msg(pose_ee)[1]
    grasp_ori_short = _quat_same_hemisphere(current_ori, grasp_ori)

    waypoints = []
    waypoints.append(current_pose)
    # [1] 抓取点正上方，姿态与当前相同（笛卡尔 XY 阶段）
    p1 = Pose()
    _copy_pose(p1, (gx, gy, z_above), current_ori)
    waypoints.append(p1)
    # [2] 抓取点正上方，姿态改为抓取姿态（姿态旋转阶段，同半球避免 180° 插值）
    p2 = Pose()
    _copy_pose(p2, (gx, gy, z_above), grasp_ori_short)
    waypoints.append(p2)
    # [3] 最终抓取位姿（笛卡尔 Z 下降），与 [2] 同四元数符号
    p3 = Pose()
    p3.position.x = pose_ee.position.x
    p3.position.y = pose_ee.position.y
    p3.position.z = pose_ee.position.z
    p3.orientation.x = grasp_ori_short[0]
    p3.orientation.y = grasp_ori_short[1]
    p3.orientation.z = grasp_ori_short[2]
    p3.orientation.w = grasp_ori_short[3]
    waypoints.append(p3)

    logger.info('笛卡尔路径 waypoints: 4 点 (当前 -> 上方同姿态 -> 上方抓取姿态 -> 最终)')

    # 4) 调用 compute_cartesian_path 服务（需已启动 move_group，如 graspnet_demo_points.launch.py）
    client = node.create_client(GetCartesianPath, COMPUTE_CARTESIAN_PATH_SERVICE)
    wait_timeout = 15.0  # 留足时间供 DDS 发现（launch 在另一终端时）
    logger.info(f'正在等待 MoveIt2 服务 {COMPUTE_CARTESIAN_PATH_SERVICE}（最多 {int(wait_timeout)} 秒）…')
    if not client.wait_for_service(timeout_sec=wait_timeout):
        logger.error(
            f'服务 {COMPUTE_CARTESIAN_PATH_SERVICE} 不可用。'
            '请先在一个终端启动含 move_group 的 launch（如 graspnet_demo_points.launch.py），'
            '待出现 "You can start planning now!" 后，在本终端再次运行本节点；'
            '两终端需 source 同一工作空间（如 aubo_ros2_ws/install/setup.bash）。'
        )
        return False

    req = GetCartesianPath.Request()
    req.header = Header()
    req.header.frame_id = base_frame
    req.header.stamp = node.get_clock().now().to_msg()
    req.start_state = start_state
    req.group_name = group_name
    req.link_name = ee_link
    req.waypoints = waypoints
    req.max_step = eef_step
    req.jump_threshold = jump_threshold
    req.prismatic_jump_threshold = 0.0
    req.revolute_jump_threshold = 0.0
    req.avoid_collisions = True
    req.path_constraints = Constraints()
    if hasattr(req, 'cartesian_speed_limited_link'):
        req.cartesian_speed_limited_link = ''
    if hasattr(req, 'max_cartesian_speed'):
        req.max_cartesian_speed = 0.0

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    if not future.done():
        logger.error('compute_cartesian_path 超时')
        return False
    try:
        resp = future.result()  # GetCartesianPath 响应，含 solution、fraction
    except Exception as e:
        logger.error(f'compute_cartesian_path 异常: {e}')
        return False

    if resp.fraction < 1.0:
        # 4 个 waypoints → 3 段： [0→1] [1→2] [2→3]，用 fraction 推断截断发生在哪一段
        num_waypoints = 4
        num_segments = num_waypoints - 1
        segment_idx = min(num_segments - 1, int(resp.fraction * num_segments))
        segment_descriptions = [
            'waypoint[0]→[1]（当前 → 上方同姿态，笛卡尔 XY）',
            'waypoint[1]→[2]（上方同姿态 → 上方抓取姿态，姿态旋转）',
            'waypoint[2]→[3]（上方抓取姿态 → 最终，笛卡尔 Z 下降）',
        ]
        logger.warning(
            f'笛卡尔路径未达 100%, fraction={resp.fraction:.2f}；'
            f'截断发生在第 {segment_idx + 1} 段: {segment_descriptions[segment_idx]}'
        )
        # 关节空间回退：若笛卡尔用了 Z 轴 180° 修正，回退时用未翻转的抓取位姿，避免机械臂多转 180°
        if flip_grasp_z_180:
            p3_fallback = Pose()
            p3_fallback.position.x = p3.position.x
            p3_fallback.position.y = p3.position.y
            p3_fallback.position.z = p3.position.z
            ori_unflip = _quat_mul(
                (p3.orientation.x, p3.orientation.y, p3.orientation.z, p3.orientation.w),
                _QUAT_Z_180,
            )
            p3_fallback.orientation.x = ori_unflip[0]
            p3_fallback.orientation.y = ori_unflip[1]
            p3_fallback.orientation.z = ori_unflip[2]
            p3_fallback.orientation.w = ori_unflip[3]
            target_pose = p3_fallback
            logger.info('改用关节空间到位姿目标（未做 Z 轴 180° 修正的抓取位姿，避免多转 180°）')
        else:
            target_pose = p3
            logger.info('改用关节空间到位姿目标（抓取位姿 p3）')
        return _run_joint_space_move_to_pose(
            node, target_pose, base_frame, ee_link, group_name, start_state,
            velocity_scaling=velocity_scaling,
        )
    if not resp.solution.joint_trajectory.points:
        logger.error('笛卡尔路径解为空')
        return False

    num_points = len(resp.solution.joint_trajectory.points)
    if num_points > CARTESIAN_MAX_POINTS_FOR_EXECUTION:
        logger.warning(
            f'笛卡尔路径轨迹点数过多 ({num_points} > {CARTESIAN_MAX_POINTS_FOR_EXECUTION})，改用关节空间到位姿目标'
        )
        if flip_grasp_z_180:
            p3_fallback = Pose()
            p3_fallback.position.x = p3.position.x
            p3_fallback.position.y = p3.position.y
            p3_fallback.position.z = p3.position.z
            ori_unflip = _quat_mul(
                (p3.orientation.x, p3.orientation.y, p3.orientation.z, p3.orientation.w),
                _QUAT_Z_180,
            )
            p3_fallback.orientation.x = ori_unflip[0]
            p3_fallback.orientation.y = ori_unflip[1]
            p3_fallback.orientation.z = ori_unflip[2]
            p3_fallback.orientation.w = ori_unflip[3]
            target_pose = p3_fallback
            logger.info('改用关节空间到位姿目标（未做 Z 轴 180° 修正的抓取位姿，避免多转 180°）')
        else:
            target_pose = p3
            logger.info('改用关节空间到位姿目标（抓取位姿 p3）')
        return _run_joint_space_move_to_pose(
            node, target_pose, base_frame, ee_link, group_name, start_state,
            velocity_scaling=velocity_scaling,
        )

    logger.info(f'笛卡尔路径规划成功，轨迹点数: {num_points}')

    # 5) 执行轨迹（ExecuteTrajectory action）
    action_client = ActionClient(node, ExecuteTrajectory, EXECUTE_TRAJECTORY_ACTION)
    if not action_client.wait_for_server(timeout_sec=5.0):
        logger.error(f'Action {EXECUTE_TRAJECTORY_ACTION} 不可用')
        return False

    goal_msg = ExecuteTrajectory.Goal()  # 目标：要执行的轨迹
    goal_msg.trajectory = resp.solution

    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        logger.error('send_goal 超时')
        return False
    try:
        goal_handle = future.result()  # 服务端返回的 goal handle，用于查结果
    except Exception as e:
        logger.error(f'send_goal 异常: {e}')
        return False
    if not goal_handle.accepted:
        logger.error('ExecuteTrajectory goal 被拒绝')
        return False

    result_future = goal_handle.get_result_async()  # 等待执行结束的 Future
    # 等待执行完成（轨迹可能较长）
    timeout_sec = 60.0
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=int(timeout_sec))
    while node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if result_future.done():
            break
    if not result_future.done():
        logger.error(f'ExecuteTrajectory 执行超时 ({timeout_sec:.0f} s)')
        return False
    try:
        result = result_future.result().result  # ExecuteTrajectory.Result，含 error_code
    except Exception as e:
        logger.error(f'ExecuteTrajectory result 异常: {e}')
        return False

    if result.error_code.val != 1:  # moveit_msgs MoveItErrorCodes SUCCESS = 1
        logger.error(f'ExecuteTrajectory 失败: error_code={result.error_code.val}')
        return False

    logger.info('抓取接近运动完成')
    return True
