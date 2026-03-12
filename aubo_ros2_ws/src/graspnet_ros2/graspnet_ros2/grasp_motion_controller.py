#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
抓取/放置运动控制模块（继承 Node，纯类方法）：
  - GraspMotionController: 继承 Node，提供 move_to_pose、run_arc_path_sequence、run_grasp_approach
  - PublishGraspsClient: 继承 GraspMotionController

除必要业务输入外，其余 MoveIt 配置（group、ee_link、容差、速度、阈值等）均使用模块默认常量。
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.action import ActionClient  # MoveGroup、ExecuteTrajectory action 客户端
from geometry_msgs.msg import Pose  # 位姿（位置 + 四元数）
from moveit_msgs.srv import GetCartesianPath  # 笛卡尔路径规划服务
from moveit_msgs.action import ExecuteTrajectory, MoveGroup  # 轨迹执行、关节空间规划 action
from moveit_msgs.msg import (
    RobotState,             # 规划起点（空表示用 MoveIt 当前状态）
    RobotTrajectory,       # 笛卡尔/关节规划结果，ExecuteTrajectory 输入
    Constraints,           # 位姿约束容器（position + orientation）
    MotionPlanRequest,     # MoveGroup / compute_cartesian_path 请求
    PlanningOptions,       # plan_only 等规划选项
    PositionConstraint,    # 位姿约束：位置球体
    OrientationConstraint, # 位姿约束：姿态欧拉角容差
    BoundingVolume,       # 位置约束区域（含 primitives）
)
from std_msgs.msg import Header  # frame_id、stamp（TF 查询时间戳）
from shape_msgs.msg import SolidPrimitive  # 球形容差区域（PositionConstraint）


# 模块默认常量
DEFAULT_GROUP_NAME = 'manipulator'   # MoveIt 规划组名
DEFAULT_BASE_FRAME = 'base_link'      # 规划与 waypoints 的参考坐标系
DEFAULT_EE_LINK = 'tool_tcp'          # aubo SRDF 末端 link，空则 move_group 用 group tip
CARTESIAN_MAX_POINTS_FOR_EXECUTION = 60  # 笛卡尔轨迹最大点数；超过则回退关节空间（避免执行慢或异响）
ARC_PATH_MAX_RETRIES = 3  # 笛卡尔路径最大重试次数
ARC_PATH_RETRY_DELAY_SEC = 0.5  # 笛卡尔路径重试间隔（秒）


class GraspMotionController(Node):
    """
    抓取运动控制器：继承 Node，直接作为 ROS2 节点使用。

    提供方法：
      - move_to_pose: 关节空间规划并执行到位姿目标
      - run_arc_path_sequence: 多段笛卡尔路径一次规划执行
      - run_grasp_approach: 抓取接近流程（XY → 姿态 → Z 垂直下降）
    """

    def __init__(self, node_name: str = 'grasp_motion_controller'):
        """初始化节点，无需传入外部 node。"""
        super().__init__(node_name)

    def move_to_pose(
        self,
        target_pose: Pose,
        velocity_scaling: float = 0.15,
        acceleration_scaling: float = 0.1,
    ) -> bool:
        """
        关节空间规划并执行到目标位姿（位置+四元数）。

        使用 MoveGroup action（与 C++ MoveGroupInterface 客户端一致），
        位姿约束为位置球体 2mm + 姿态欧拉角容差 0.01 rad。

        Args:
            target_pose: 目标位姿（base_link 下）
            velocity_scaling: 关节速度缩放 [0~1]
            acceleration_scaling: 关节加速度缩放 [0~1]

        Returns:
            True 成功，False 失败
        """
        logger = self.get_logger()
        move_group_action = '/move_action'  # MoveGroup action 名称
        logger.info(
            '[关节空间] 开始位姿规划: '
            f'start_state=MoveIt当前状态, vel={velocity_scaling:.2f}, acc={acceleration_scaling:.2f}'
        )
        stamp = self.get_clock().now().to_msg()  # TF 查询时间戳
        goal_constraints = _build_pose_goal_constraints(
            DEFAULT_EE_LINK,
            target_pose,
            DEFAULT_BASE_FRAME,
            stamp,
            0.002,   # 位置球形容差 (m)
            0.01,    # 姿态容差 (rad)
        )
        req = MotionPlanRequest()
        req.workspace_parameters.header = Header(frame_id=DEFAULT_BASE_FRAME, stamp=stamp)
        req.start_state = RobotState()
        req.group_name = DEFAULT_GROUP_NAME
        req.goal_constraints = [goal_constraints]
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0  # 单次规划超时 (s)
        req.max_velocity_scaling_factor = velocity_scaling
        req.max_acceleration_scaling_factor = acceleration_scaling

        opts = PlanningOptions()
        opts.plan_only = False  # False: 规划后立即执行

        logger.info(f'[关节空间] 连接 MoveGroup action: {move_group_action}')
        action_client = _get_or_create_client(
            self, '_gm_move_group_client',
            lambda: ActionClient(self, MoveGroup, move_group_action),
        )
        if not action_client.wait_for_server(timeout_sec=10.0):
            logger.error(f'MoveGroup action {move_group_action} 不可用，请确认 move_group 已启动且发布该 action')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options = opts
        logger.info('[关节空间] 已发送 MoveGroup 规划执行请求')
        future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
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
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        if not result_future.done():
            logger.error('MoveGroup 执行超时 (60 s)')
            return False
        try:
            result = result_future.result().result
        except Exception as e:
            logger.error(f'MoveGroup result 异常: {e}')
            return False
        if result.error_code.val != 1:  # moveit_msgs: 1=SUCCESS
            logger.error(f'关节空间到位姿失败: error_code={result.error_code.val}')
            return False
        logger.info('[关节空间] 到位姿执行完成')
        return True

    def run_arc_path_sequence(self, segments: list[dict]) -> bool:
        """
        多段笛卡尔路径：一次规划、一次执行（等价 C++ runArcPathSequence）。

        从当前位姿出发，按 segments 顺序沿各轴叠加位移，形成 waypoints 后调用
        compute_cartesian_path；若 fraction<1 或点数过多则回退关节空间到位姿目标。

        Args:
            segments: [{'axis':'x','offset':0.1}, {'axis':'z','offset':-0.05}, ...]
                      axis 仅支持 x/y/z，offset 为沿该轴位移 (m)

        Returns:
            True 成功，False 失败
        """
        logger = self.get_logger()
        if not segments:
            logger.info('[run_arc_path_sequence] segments 为空，不运动')
            return True
        current_pose = _get_current_ee_pose(self, DEFAULT_BASE_FRAME, DEFAULT_EE_LINK)
        if current_pose is None:
            return False
        waypoints = [current_pose]  # 起点为当前末端位姿
        curr_pos, curr_ori = _pose_from_pose_msg(current_pose)
        p = Pose()  # 累积位姿，每段在其基础上叠加 offset
        _copy_pose(p, curr_pos, curr_ori)
        for seg in segments:
            axis = str(seg.get('axis', '')).lower()   # x/y/z
            offset = float(seg.get('offset', 0.0))    # 沿 axis 位移 (m)
            if axis == 'x':
                p.position.x += offset
            elif axis == 'y':
                p.position.y += offset
            elif axis == 'z':
                p.position.z += offset
            else:
                logger.error(f'不支持的 axis: {seg.get("axis")}（仅支持 x/y/z）')
                return False
            wp = Pose()
            _copy_pose(wp, (p.position.x, p.position.y, p.position.z), curr_ori)
            waypoints.append(wp)
        final_pose = waypoints[-1]  # 笛卡尔失败时关节空间回退目标
        for attempt in range(1, ARC_PATH_MAX_RETRIES + 1):
            resp = _compute_cartesian_path(self, waypoints)
            if resp is None:
                return False
            logger.info(f'多段笛卡尔路径完成度: {resp.fraction * 100.0:.2f}% (尝试 {attempt}/{ARC_PATH_MAX_RETRIES})')
            if resp.fraction >= 1.0 and resp.solution.joint_trajectory.points:
                num_points = len(resp.solution.joint_trajectory.points)
                if num_points <= CARTESIAN_MAX_POINTS_FOR_EXECUTION:  # 点数过多则回退关节空间
                    if _execute_trajectory(self, resp.solution):
                        logger.info('多段笛卡尔路径执行完成')
                        return True
                    return False
                logger.warning(
                    f'笛卡尔轨迹点数过多 ({num_points} > {CARTESIAN_MAX_POINTS_FOR_EXECUTION})，改用关节空间'
                )
                return self.move_to_pose(final_pose)
            if attempt < ARC_PATH_MAX_RETRIES:
                time.sleep(ARC_PATH_RETRY_DELAY_SEC)
        logger.warning('多段笛卡尔路径重试后仍未达 100%，改用关节空间到最终位姿')
        return self.move_to_pose(final_pose)

    def run_grasp_approach(
        self,
        pose_ee: Pose,
        height_above: float = 0.05,
        velocity_scaling: float = 0.15,
        acceleration_scaling: float = 0.1,
    ) -> bool:
        """
        抓取接近流程：4 waypoints 一次笛卡尔规划执行。

        路径：当前位姿 -> 目标 XY+安全高度（保持当前姿态）-> 同位置切换抓取姿态 -> 目标 Z 下降。
        笛卡尔前对 GraspNet 姿态做局部 Z 轴 180° 修正；回退关节空间时取消修正。

        Args:
            pose_ee: 抓取目标位姿（end_effector，base_link 下）
            height_above: 目标点上方安全高度 (m)
            velocity_scaling: 关节空间回退速度缩放
            acceleration_scaling: 关节空间回退加速度缩放

        Returns:
            True 成功，False 失败
        """
        logger = self.get_logger()
        pose_for_plan = _apply_grasp_z_flip_180(pose_ee, logger)  # GraspNet 姿态修正
        current_pose = _get_current_ee_pose(self, DEFAULT_BASE_FRAME, DEFAULT_EE_LINK)
        if current_pose is None:
            return False
        gx, gy, gz = pose_for_plan.position.x, pose_for_plan.position.y, pose_for_plan.position.z
        z_above = gz + height_above  # 安全高度
        current_ori = _pose_from_pose_msg(current_pose)[1]
        grasp_ori = _pose_from_pose_msg(pose_for_plan)[1]
        grasp_ori_short = _quat_same_hemisphere(current_ori, grasp_ori, logger)  # 选短路径
        p1 = Pose()  # 目标 XY + 安全高度，保持当前姿态
        _copy_pose(p1, (gx, gy, z_above), current_ori)
        p2 = Pose()  # 同位置，切换为抓取姿态
        _copy_pose(p2, (gx, gy, z_above), grasp_ori_short)
        p3 = Pose()  # 目标位置 + 抓取姿态（最终下降）
        _copy_pose(p3, (gx, gy, gz), grasp_ori_short)
        waypoints = [current_pose, p1, p2, p3]  # 4 点：当前 -> XY -> 姿态 -> Z
        logger.info('笛卡尔路径 waypoints: 4 点 (当前 -> 上方同姿态 -> 上方抓取姿态 -> 最终)')
        resp = None
        for attempt in range(1, ARC_PATH_MAX_RETRIES + 1):  # 笛卡尔规划重试
            resp = _compute_cartesian_path(self, waypoints)
            if resp is None:
                return False
            logger.info(f'抓取接近笛卡尔路径完成度: {resp.fraction * 100.0:.2f}% (尝试 {attempt}/{ARC_PATH_MAX_RETRIES})')
            if resp.fraction >= 1.0:
                break
            if attempt < ARC_PATH_MAX_RETRIES:
                time.sleep(ARC_PATH_RETRY_DELAY_SEC)
        if resp is None:
            return False
        if resp.fraction < 1.0:  # 路径被截断，回退关节空间到 p3
            num_segments = 3  # 对应 3 段：XY / 姿态旋转 / Z 下降
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
            target_pose = _pose_unflip_if_needed(p3, logger)  # 取消 180° 修正
            return self.move_to_pose(
                target_pose,
                velocity_scaling=velocity_scaling,
                acceleration_scaling=acceleration_scaling,
            )
        if not resp.solution.joint_trajectory.points:
            logger.error('笛卡尔路径解为空')
            return False
        num_points = len(resp.solution.joint_trajectory.points)
        if num_points > CARTESIAN_MAX_POINTS_FOR_EXECUTION:  # 点数过多，回退关节空间
            logger.warning(
                f'笛卡尔路径轨迹点数过多 ({num_points} > {CARTESIAN_MAX_POINTS_FOR_EXECUTION})，改用关节空间到位姿目标'
            )
            target_pose = _pose_unflip_if_needed(p3)  # 取消 180° 修正
            return self.move_to_pose(
                target_pose,
                velocity_scaling=velocity_scaling,
                acceleration_scaling=acceleration_scaling,
            )
        if not _execute_trajectory(self, resp.solution):
            return False
        logger.info('抓取接近运动完成')
        return True


def _get_or_create_client(node: Node, attr: str, create_fn):
    """
    按 attr 从 node 获取或创建并缓存客户端，避免重复创建 ActionClient/ServiceClient。

    Args:
        node: ROS2 节点（GraspMotionController 实例），用于存储缓存属性
        attr: 缓存属性名，如 '_gm_move_group_client'
        create_fn: 无参可调用对象，用于创建新客户端

    Returns:
        已缓存或新创建的客户端实例
    """
    client = getattr(node, attr, None)  # 从 node 上查找已缓存的客户端
    if client is None:
        client = create_fn()
        setattr(node, attr, client)
    return client


def _pose_from_pose_msg(pose: Pose) -> tuple:
    """
    从 Pose 消息解析为元组，便于 waypoint 构造。

    Returns:
        (pos, ori): pos=(x,y,z)，ori=(qx,qy,qz,qw)
    """
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def _copy_pose(dest: Pose, pos: tuple, ori: tuple) -> None:
    """
    将位置和姿态写入目标 Pose（原地修改 dest）。

    Args:
        dest: 目标 Pose 消息
        pos: (x, y, z) 位置
        ori: (qx, qy, qz, qw) 四元数姿态
    """
    dest.position.x, dest.position.y, dest.position.z = pos[0], pos[1], pos[2]
    dest.orientation.x, dest.orientation.y, dest.orientation.z, dest.orientation.w = ori[0], ori[1], ori[2], ori[3]


def _quat_same_hemisphere(q_ref: tuple, q: tuple, logger=None) -> tuple:
    """返回与 q 同旋转的四元数，且与 q_ref 在同一半球（点积≥0），避免插值走 180° 长路径。
    点积公式: dot = q_ref·q = qx1*qx2 + qy1*qy2 + qz1*qz2 + qw1*qw2。若 dot<0 则取 -q。
    """
    q_ref_arr = np.array(q_ref, dtype=float)
    q_arr = np.array(q, dtype=float)
    dot = float(np.dot(q_ref_arr, q_arr))
    if dot >= 0:
        out = q
        if logger is not None:
            logger.info(f'[quat_same_hemisphere] dot={dot:.4f}≥0 → 保持q, 返回值={tuple(round(x, 6) for x in out)}')
        return out
    out = tuple(-q_arr)
    if logger is not None:
        logger.info(f'[quat_same_hemisphere] dot={dot:.4f}<0 → 取-q, 返回值={tuple(round(x, 6) for x in out)}')
    return out


def _quat_mul(q1: tuple, q2: tuple, logger=None) -> tuple:
    """四元数乘法 q1 * q2，均为 (qx, qy, qz, qw)。公式: result = q1 @ q2（scipy Rotation 组合）。
    右乘 q2 表示在 q1 局部坐标系下施加 q2 旋转。
    """
    r = Rotation.from_quat(q1) * Rotation.from_quat(q2)
    result = tuple(r.as_quat())
    if logger is not None:
        logger.info(f'[_quat_mul] 结果={tuple(round(x, 6) for x in result)}')
    return result


# 抓取坐标系下绕 Z 轴转 180° 的四元数 (qx,qy,qz,qw)，approach 方向不变，绕 approach 翻转
_QUAT_Z_180 = (0.0, 0.0, 1.0, 0.0)


def _apply_grasp_z_flip_180(pose: Pose, logger=None) -> Pose:
    """在抓取位姿的局部坐标系下绕 Z 轴旋转 180°，返回新 Pose（位置不变）。
    用于修正 GraspNet 与机械臂末端 Z（approach）方向约定不一致导致的 180° 反转。
    logger: 可选，ROS2 节点日志，用于输出四元数乘法对比。
    """
    ori = _pose_from_pose_msg(pose)[1]
    new_ori = _quat_mul(ori, _QUAT_Z_180, logger)
    out = Pose()
    out.position.x = pose.position.x
    out.position.y = pose.position.y
    out.position.z = pose.position.z
    out.orientation.x = new_ori[0]
    out.orientation.y = new_ori[1]
    out.orientation.z = new_ori[2]
    out.orientation.w = new_ori[3]
    return out


def _get_current_ee_pose(
    node: Node,
    base_frame: str = DEFAULT_BASE_FRAME,
    ee_link: str = DEFAULT_EE_LINK,
) -> Pose | None:
    """
    通过 TF 获取当前末端执行器在 base_frame 下的位姿。

    Args:
        node: ROS2 节点，用于 TF 监听器
        base_frame: 参考坐标系（默认 base_link）
        ee_link: 末端 link 名（默认 tool_tcp）

    Returns:
        当前位姿，失败返回 None
    """
    logger = node.get_logger()
    t_start = time.perf_counter()
    logger.info(f'[状态获取] 查询当前末端位姿: TF {base_frame} -> {ee_link}')
    from tf2_ros import Buffer, TransformListener
    # 复用 TF 监听上下文，避免每次调用都重新建监听器导致冷启动延迟
    buffer = getattr(node, '_gm_tf_buffer', None)  # TF 缓冲
    listener = getattr(node, '_gm_tf_listener', None)  # TF 监听器
    if buffer is None or listener is None:
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        setattr(node, '_gm_tf_buffer', buffer)
        setattr(node, '_gm_tf_listener', listener)
        # 首次创建监听器时给 TF 缓冲少量时间累积数据
        time.sleep(0.15)
    try:
        t = None  # TF 变换结果
        for attempt in range(20):  # 最多重试 20 次，应对 TF 尚未就绪
            try:
                t = buffer.lookup_transform(
                    base_frame, ee_link, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.12)
                )
                break
            except Exception:
                if attempt < 19:
                    time.sleep(0.03)
                else:
                    raise
        if t is None:
            return None
        current_pose = Pose()
        current_pose.position.x = t.transform.translation.x
        current_pose.position.y = t.transform.translation.y
        current_pose.position.z = t.transform.translation.z
        current_pose.orientation = t.transform.rotation
        elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        logger.info(
            '[状态获取] 当前末端位姿已获取: '
            f'pos=({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}), '
            f'耗时={elapsed_ms:.0f}ms'
        )
        return current_pose
    except Exception as e:
        logger.error(f'获取当前末端位姿失败 (TF {base_frame} -> {ee_link}): {e}')
        return None


def _build_pose_goal_constraints(
    link_name: str,
    pose: Pose,
    frame_id: str,
    stamp=None,
    position_tolerance: float = 0.002,
    orientation_tolerance: float = 0.01,
) -> Constraints:
    """
    由位姿构造 MoveIt goal_constraints：位置球体 + 姿态欧拉角容差。

    Args:
        link_name: 约束施加的 link
        pose: 目标位姿
        frame_id: 位姿所在坐标系
        stamp: TF 查询时间戳（None 则用当前时间）
        position_tolerance: 位置球体半径（米）
        orientation_tolerance: 姿态欧拉角容差（弧度）

    Returns:
        Constraints 消息
    """
    # stamp 用于 TF 查询时间戳
    if stamp is None:
        stamp = rclpy.time.Time().to_msg()
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp

    pos_constraint = PositionConstraint()  # 位置约束：目标点周围球体
    pos_constraint.header = header
    pos_constraint.link_name = link_name
    sphere = SolidPrimitive()  # 球形容差区域
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

    ori_constraint = OrientationConstraint()  # 姿态约束：欧拉角容差
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


def _compute_cartesian_path(
    node: Node,
    waypoints: list[Pose],
) -> GetCartesianPath.Response | None:
    """
    调用 MoveIt compute_cartesian_path 服务进行笛卡尔路径规划。

    Args:
        node: ROS2 节点
        waypoints: 笛卡尔路径路点列表（base_link 下）

    Returns:
        服务响应（含 fraction、solution.joint_trajectory），失败返回 None
    """
    logger = node.get_logger()
    cartesian_service = '/compute_cartesian_path'  # move_group 提供的笛卡尔规划服务
    logger.info(
        f'[笛卡尔] 调用服务 {cartesian_service}: '
        f'waypoints={len(waypoints)}, start_state=MoveIt当前状态'
    )
    client = _get_or_create_client(
        node, '_gm_cartesian_client',
        lambda: node.create_client(GetCartesianPath, cartesian_service),
    )
    if not client.wait_for_service(timeout_sec=15.0):
        logger.error(f'服务 {cartesian_service} 不可用')
        return None
    req = GetCartesianPath.Request()
    req.header = Header()
    req.header.frame_id = DEFAULT_BASE_FRAME
    req.header.stamp = node.get_clock().now().to_msg()
    req.start_state = RobotState()
    req.group_name = DEFAULT_GROUP_NAME
    req.link_name = DEFAULT_EE_LINK
    req.waypoints = waypoints
    req.max_step = 0.01  # 笛卡尔路径步长（米），越小轨迹越密
    req.jump_threshold = 0.0  # 0 表示禁用跳变检测
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
        return None
    try:
        resp = future.result()
        logger.info(
            f'[笛卡尔] 规划返回: fraction={resp.fraction:.3f}, '
            f'points={len(resp.solution.joint_trajectory.points)}'
        )
        return resp
    except Exception as e:
        logger.error(f'compute_cartesian_path 异常: {e}')
        return None


def _execute_trajectory(node: Node, trajectory: RobotTrajectory) -> bool:
    """
    通过 ExecuteTrajectory action 执行 MoveIt 生成的关节轨迹。

    Args:
        node: ROS2 节点
        trajectory: RobotTrajectory 消息（含 joint_trajectory）

    Returns:
        执行成功返回 True
    """
    logger = node.get_logger()
    execute_action = '/execute_trajectory'  # move_group 的轨迹执行 action
    logger.info(f'[轨迹执行] 连接 ExecuteTrajectory action: {execute_action}')
    action_client = _get_or_create_client(
        node, '_gm_execute_client',
        lambda: ActionClient(node, ExecuteTrajectory, execute_action),
    )
    if not action_client.wait_for_server(timeout_sec=5.0):
        logger.error(f'Action {execute_action} 不可用')
        return False
    goal_msg = ExecuteTrajectory.Goal()
    goal_msg.trajectory = trajectory
    logger.info('[轨迹执行] 已发送轨迹执行请求')
    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        logger.error('send_goal 超时')
        return False
    try:
        goal_handle = future.result()
    except Exception as e:
        logger.error(f'send_goal 异常: {e}')
        return False
    if not goal_handle.accepted:
        logger.error('ExecuteTrajectory goal 被拒绝')
        return False
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)
    if not result_future.done():
        logger.error('ExecuteTrajectory 执行超时 (60 s)')
        return False
    try:
        result = result_future.result().result
    except Exception as e:
        logger.error(f'ExecuteTrajectory result 异常: {e}')
        return False
    if result.error_code.val != 1:  # 1 表示 SUCCESS
        logger.error(f'ExecuteTrajectory 失败: error_code={result.error_code.val}')
        return False
    logger.info('[轨迹执行] 执行成功')
    return True


def _pose_unflip_if_needed(pose: Pose, logger=None) -> Pose:
    """
    关节空间回退时，取消局部 Z 轴 180° 修正。

    笛卡尔规划前会对 GraspNet 姿态做 _apply_grasp_z_flip_180；
    回退到关节空间时需撤销该修正，避免关节规划多转 180°。

    Args:
        pose: 已翻转的 Pose
        logger: 可选日志

    Returns:
        未翻转姿态的 Pose（位置不变）
    """
    out = Pose()
    out.position.x = pose.position.x
    out.position.y = pose.position.y
    out.position.z = pose.position.z
    ori_unflip = _quat_mul(
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        _QUAT_Z_180,
        logger,
    )
    out.orientation.x = ori_unflip[0]
    out.orientation.y = ori_unflip[1]
    out.orientation.z = ori_unflip[2]
    out.orientation.w = ori_unflip[3]
    return out
