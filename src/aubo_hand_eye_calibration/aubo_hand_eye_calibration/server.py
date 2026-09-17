"""
ROS 2 action server for safe, automatic eye-in-hand calibration.

状态机:
    idle -> preflighting -> planning -> plan_ready (plan_only 结束)
         -> moving -> settling -> capturing -> (逐位姿循环)
         -> returning -> solving -> complete / quality_failed / failed
    任意运动阶段可取消: -> cancelling -> cancelled
激活 (人工确认, 独立服务): -> activated
"""

import asyncio
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from aubo_msgs.action import RunHandEyeCalibration
from aubo_msgs.srv import ActivateHandEyeCalibration
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Transform
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .board import board_from_node
from .defaults import (
    DEFAULT_BASE_FRAME,
    DEFAULT_CAMERA_OPTICAL_FRAME,
    DEFAULT_CAMERA_ROOT_FRAME,
    DEFAULT_WRIST_FRAME,
)
from .detector import CheckerboardDetector
from .joint_calib import JointObservation, solve_joint_intrinsics_hand_eye
from .solver import (
    CalibrationResult,
    CalibrationSample,
    solve_hand_eye,
    VALID_METHODS,
)
from .stats import mad_mask_1d
from .storage import (
    activate_candidate,
    load_active_extrinsics,
    write_candidate,
)
from .transforms import (
    inverse,
    mean_transform,
    transform_from_xyz_quat,
    transform_to_xyz_quat,
)
from .viewpoints import candidate_viewpoints, select_diverse


def _matrix_from_transform(transform):
    translation = transform.translation
    rotation = transform.rotation
    return transform_from_xyz_quat(
        [translation.x, translation.y, translation.z],
        [rotation.x, rotation.y, rotation.z, rotation.w],
    )


def _pose_from_matrix(matrix):
    xyz, quaternion = transform_to_xyz_quat(matrix)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = quaternion
    return pose


def _transform_message(matrix):
    xyz, quaternion = transform_to_xyz_quat(matrix)
    message = Transform()
    message.translation.x, message.translation.y, message.translation.z = xyz
    (
        message.rotation.x,
        message.rotation.y,
        message.rotation.z,
        message.rotation.w,
    ) = quaternion
    return message


class CalibrationServer(Node):
    def __init__(self):
        super().__init__('hand_eye_calibration_server')
        self._group = ReentrantCallbackGroup()
        defaults = Path(get_package_share_directory(
            'aubo_hand_eye_calibration')) / 'config'
        self.declare_parameter(
            'poses_file', str(defaults / 'poses.yaml'),
            ParameterDescriptor(description='标定位姿定义文件 (yaml) 路径'))
        self.declare_parameter(
            'base_frame', DEFAULT_BASE_FRAME,
            ParameterDescriptor(description='机器人基座坐标系'))
        self.declare_parameter(
            'wrist_frame', DEFAULT_WRIST_FRAME,
            ParameterDescriptor(description='腕部 (法兰) 坐标系'))
        self.declare_parameter(
            'camera_root_frame', DEFAULT_CAMERA_ROOT_FRAME,
            ParameterDescriptor(description='相机安装座坐标系 (外参发布目标)'))
        self.declare_parameter(
            'camera_optical_frame', DEFAULT_CAMERA_OPTICAL_FRAME,
            ParameterDescriptor(description='相机光学坐标系 (标定求解目标)'))
        self.declare_parameter(
            'move_group', 'manipulator_e5',
            ParameterDescriptor(description='MoveIt 规划组名'))
        self.declare_parameter(
            'image_topic', '/camera/color/image_raw',
            ParameterDescriptor(description='彩色图像话题'))
        self.declare_parameter(
            'camera_info_topic', '/camera/color/camera_info',
            ParameterDescriptor(description='相机内参话题'))
        self.declare_parameter(
            'board_columns', 11,
            ParameterDescriptor(description='棋盘格内角点列数'))
        self.declare_parameter(
            'board_rows', 8,
            ParameterDescriptor(description='棋盘格内角点行数'))
        self.declare_parameter(
            'board_square_size_m', 0.020,
            ParameterDescriptor(description='棋盘格格宽 (m, 需实测)'))
        self.declare_parameter(
            'frames_per_pose', 5,
            ParameterDescriptor(
                description='每个位姿采集的帧数 (剔除离群帧后取均值)'))
        self.declare_parameter(
            'min_samples', 12,
            ParameterDescriptor(description='求解所需最少有效样本数'))
        self.declare_parameter(
            'joint_velocity_threshold', 0.01,
            ParameterDescriptor(
                description='判定机械臂静止的关节速度阈值 (rad/s)'))
        self.declare_parameter(
            'settle_duration_s', 0.5,
            ParameterDescriptor(description='到位后判定静止所需的持续时长 (s)'))
        self.declare_parameter(
            'stable_timeout_s', 10.0,
            ParameterDescriptor(description='等待机械臂静止的超时时间 (s)'))
        self.declare_parameter(
            'sample_timeout_s', 4.0,
            ParameterDescriptor(description='单个位姿采集足量帧的超时时间 (s)'))
        self.declare_parameter(
            'velocity_scaling', 0.1,
            ParameterDescriptor(description='MoveIt 速度缩放因子'))
        self.declare_parameter(
            'acceleration_scaling', 0.1,
            ParameterDescriptor(description='MoveIt 加速度缩放因子'))
        self.declare_parameter(
            'planning_attempts', 5,
            ParameterDescriptor(description='MoveIt 规划尝试次数'))
        self.declare_parameter(
            'planning_time_s', 5.0,
            ParameterDescriptor(description='MoveIt 单次规划时限 (s)'))
        self.declare_parameter(
            'position_tolerance_m', 0.001,
            ParameterDescriptor(description='目标位置约束容差 (m)'))
        self.declare_parameter(
            'orientation_tolerance_rad', 0.01,
            ParameterDescriptor(description='目标姿态约束容差 (rad)'))
        self.declare_parameter(
            'max_reprojection_rms_px', 1.0,
            ParameterDescriptor(
                description='质量门: 求解样本重投影 RMS 上限 (px)'))
        # 单帧门: detector 逐帧丢弃重投影超限的观测。
        # 历史实现是 max_reprojection_rms_px × 1.5 的隐式耦合,
        # 2026-09-17 起为独立参数, 默认值等于当时的等效值
        self.declare_parameter(
            'per_frame_reprojection_rms_px', 1.5,
            ParameterDescriptor(
                description='单帧质量门: 丢弃重投影 RMS 超限的观测 (px)'))
        self.declare_parameter(
            'max_translation_rms_m', 0.003,
            ParameterDescriptor(description='质量门: 平移一致性 RMS 上限 (m)'))
        self.declare_parameter(
            'max_rotation_rms_deg', 0.5,
            ParameterDescriptor(description='质量门: 旋转一致性 RMS 上限 (deg)'))
        self.declare_parameter(
            'min_rotation_span_deg', 20.0,
            ParameterDescriptor(description='质量门: 样本旋转覆盖跨度下限 (deg)'))
        # 求解方法: auto|tsai|park|horaud|andreff|daniilidis;
        # goal.method 为空串时取此参数
        self.declare_parameter(
            'solver_method', 'auto',
            ParameterDescriptor(
                description='求解方法 auto|tsai|park|horaud|andreff|daniilidis, '
                            'goal.method 为空串时生效'))
        # auto 档: 由当前图像定位固定棋盘格并自动生成保持视野的视点。
        # 全部参数相机无关 —— 几何量运行时取自 camera_info
        self.declare_parameter(
            'pose_source', 'poses',
            ParameterDescriptor(
                description='位姿来源 poses|auto (goal.pose_source 为空串时生效)'))
        self.declare_parameter(
            'solve_target', 'hand_eye',
            ParameterDescriptor(
                description='求解目标 hand_eye|joint (joint=内参+外参联合求解)'))
        self.declare_parameter(
            'auto_min_poses', 12,
            ParameterDescriptor(description='auto 档最少有效采集视点数'))
        self.declare_parameter(
            'auto_extra_candidates', 24,
            ParameterDescriptor(
                description='auto 档冗余候选数 (预检跳过不可达视点时补位)'))
        self.declare_parameter(
            'auto_polar_degrees', [0.0, 15.0, 30.0, 45.0],
            ParameterDescriptor(description='视点极角列表 (相对板法向, deg)'))
        self.declare_parameter(
            'auto_azimuth_step_deg', 60.0,
            ParameterDescriptor(description='视点方位角步进 (deg)'))
        self.declare_parameter(
            'auto_fill_fractions', [0.40, 0.55, 0.70],
            ParameterDescriptor(
                description='板宽占画面宽的目标比例 (反推视点距离, 适配任意焦距)'))
        self.declare_parameter(
            'auto_margin_fraction', 0.08,
            ParameterDescriptor(description='FOV 掩码画面余量 (比例)'))
        self.declare_parameter(
            'auto_board_border_squares', 1.0,
            ParameterDescriptor(
                description='板外沿相对内角点格的扩展格数 (FOV 掩码用)'))
        self.declare_parameter(
            'auto_distance_range_m', [0.15, 1.20],
            ParameterDescriptor(description='视点距离夹取范围 [m]'))
        self.declare_parameter(
            'auto_min_span_deg', 30.0,
            ParameterDescriptor(description='视点腕部旋转跨度下限 (deg)'))
        self.declare_parameter(
            'joint_max_reprojection_rms_px', 0.8,
            ParameterDescriptor(description='joint 档质量门: 总重投影 RMS 上限 (px)'))
        self.declare_parameter(
            'joint_frames_per_pose', 2,
            ParameterDescriptor(
                description='joint 档每视点参与求解的代表帧数 (按帧内 RMS 排序取前 N)'))
        self.declare_parameter(
            'initial_extrinsics_file', '',
            ParameterDescriptor(
                description='auto 档初始外参文件 (空串=存储目录 active.yaml); '
                            '用于定位板与生成视点, 不做名义回退'))

        self._board = board_from_node(self)
        self._detector = CheckerboardDetector(
            self._board,
            max_reprojection_rms_px=float(
                self.get_parameter('per_frame_reprojection_rms_px').value),
        )
        self._bridge = CvBridge()
        self._camera_info = None
        # 元素: (monotonic_stamp, observation)
        self._observations = deque(maxlen=30)
        self._observations_lock = threading.Lock()
        self._board_visible = False
        self._stable_since = None
        self._velocity_warned = False
        self._goal_lock = threading.Lock()
        self._busy = False
        self._active_move_goal = None
        self._pose_status = []

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False)
        self._move_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self._group)
        self._reload_client = self.create_client(
            Trigger, '/hand_eye_extrinsics_publisher/reload',
            callback_group=self._group)

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._camera_info_callback,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback,
            qos_profile_sensor_data, callback_group=self._group)
        self.create_subscription(
            # Raw is the canonical base topic exposed by image_transport.
            Image,
            self.get_parameter('image_topic').value,
            self._image_callback,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self._preview_publisher = self.create_publisher(
            CompressedImage, '~/preview/compressed', qos_profile_sensor_data)
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = self.create_publisher(
            String, '~/status', status_qos)
        self._action_server = ActionServer(
            self,
            RunHandEyeCalibration,
            '~/run',
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self._group,
        )
        self._activate_service = self.create_service(
            ActivateHandEyeCalibration,
            '~/activate',
            self._activate,
            callback_group=self._group,
        )
        self._publish_status('idle', 'ready', board=self._board.describe())

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _camera_info_callback(self, message):
        self._camera_info = message

    def _joint_state_callback(self, message):
        threshold = float(
            self.get_parameter('joint_velocity_threshold').value)
        if not message.velocity:
            # 驱动不带速度字段时无法判断运动状态, 按静止处理并提示一次
            if not self._velocity_warned:
                self._velocity_warned = True
                self.get_logger().warning(
                    'JointState has no velocity field; '
                    'settle detection falls back to time-based wait')
            moving = False
        else:
            moving = max(abs(value) for value in message.velocity) >= threshold
        if moving:
            self._stable_since = None
        elif self._stable_since is None:
            self._stable_since = time.monotonic()

    def _image_callback(self, message):
        if self._camera_info is None:
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, 'bgr8')
            camera_matrix = np.asarray(
                self._camera_info.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(self._camera_info.d, dtype=np.float64)
            observation = self._detector.detect(
                image, camera_matrix, distortion)
            self._board_visible = observation is not None
            annotated = self._detector.annotate(image, observation)
            success, encoded = cv2.imencode(
                '.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if success:
                preview = CompressedImage()
                preview.header = message.header
                preview.format = 'jpeg'
                preview.data = encoded.tobytes()
                self._preview_publisher.publish(preview)
            if observation is not None:
                with self._observations_lock:
                    self._observations.append((
                        time.monotonic(), observation))
        except (cv2.error, ValueError) as error:
            self.get_logger().warning(
                f'checkerboard processing failed: {error}')

    # ------------------------------------------------------------------
    # Action 入口与取消
    # ------------------------------------------------------------------
    def _goal(self, _request):
        with self._goal_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle):
        if self._active_move_goal is not None:
            self._active_move_goal.cancel_goal_async()
        self._publish_status('cancelling', 'cancel requested')
        return CancelResponse.ACCEPT

    def _release_goal(self):
        with self._goal_lock:
            self._busy = False

    # ------------------------------------------------------------------
    # 状态与反馈
    # ------------------------------------------------------------------
    def _publish_status(self, stage, detail, **extra):
        document = {
            'stage': stage,
            'detail': detail,
            'board': self._board.describe(),
            'board_visible': self._board_visible,
            'stamp': datetime.now(timezone.utc).isoformat(),
            **extra,
        }
        message = String()
        message.data = json.dumps(document)
        self._status_publisher.publish(message)

    def _feedback(self, goal_handle, stage, detail, index, count, accepted):
        progress = float(index / count) if count else 0.0
        feedback = RunHandEyeCalibration.Feedback()
        feedback.stage = stage
        feedback.detail = detail
        feedback.pose_index = index
        feedback.pose_count = count
        feedback.accepted_samples = accepted
        feedback.progress = progress
        goal_handle.publish_feedback(feedback)
        self._publish_status(
            stage, detail, pose_index=index, pose_count=count,
            accepted_samples=accepted, progress=progress,
            poses=self._pose_status)

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    def _lookup(self, parent, child):
        transform = self._tf_buffer.lookup_transform(
            parent, child, rclpy.time.Time())
        return _matrix_from_transform(transform.transform)

    def _load_poses(self):
        path = Path(self.get_parameter('poses_file').value)
        with path.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        result = []
        for item in data['poses']:
            result.append(transform_from_xyz_quat(
                item['position_m'], item['quaternion_xyzw']))
        return result

    def _move_goal(self, target, plan_only, start_state=None):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.get_parameter('move_group').value
        request.num_planning_attempts = int(
            self.get_parameter('planning_attempts').value)
        request.allowed_planning_time = float(
            self.get_parameter('planning_time_s').value)
        request.max_velocity_scaling_factor = float(
            self.get_parameter('velocity_scaling').value)
        request.max_acceleration_scaling_factor = float(
            self.get_parameter('acceleration_scaling').value)
        if start_state is not None:
            request.start_state = start_state

        tolerance = float(self.get_parameter('position_tolerance_m').value)
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.get_parameter('base_frame').value
        position.link_name = self.get_parameter('wrist_frame').value
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [tolerance]
        position.constraint_region.primitives = [primitive]
        position.constraint_region.primitive_poses = [_pose_from_matrix(target)]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation_tolerance = float(
            self.get_parameter('orientation_tolerance_rad').value)
        orientation.header.frame_id = self.get_parameter('base_frame').value
        orientation.link_name = self.get_parameter('wrist_frame').value
        orientation.orientation = _pose_from_matrix(target).orientation
        orientation.absolute_x_axis_tolerance = orientation_tolerance
        orientation.absolute_y_axis_tolerance = orientation_tolerance
        orientation.absolute_z_axis_tolerance = orientation_tolerance
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = bool(plan_only)
        goal.planning_options.replan = not plan_only
        if not plan_only:
            goal.planning_options.replan_attempts = 2
        return goal

    async def _send_move(self, goal_handle, target, plan_only,
                         start_state=None):
        move_goal_handle = await self._move_client.send_goal_async(
            self._move_goal(target, plan_only, start_state))
        if not move_goal_handle.accepted:
            return None, 'MoveGroup rejected goal'
        self._active_move_goal = move_goal_handle
        wrapped = await move_goal_handle.get_result_async()
        self._active_move_goal = None
        if wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            if goal_handle.is_cancel_requested:
                raise asyncio.CancelledError
            return None, (
                f'MoveIt failed with code {wrapped.result.error_code.val}')
        return wrapped.result, ''

    @staticmethod
    def _final_state(planning_result):
        trajectory = planning_result.planned_trajectory.joint_trajectory
        state = RobotState()
        state.is_diff = True
        if trajectory.points:
            state.joint_state.name = list(trajectory.joint_names)
            state.joint_state.position = list(trajectory.points[-1].positions)
        return state

    # ------------------------------------------------------------------
    # 流程各阶段
    # ------------------------------------------------------------------
    async def _preflight_and_plan(self, goal_handle, poses, return_pose=None,
                                  skip_failures=False, required=0):
        if self._camera_info is None:
            raise RuntimeError('CameraInfo is unavailable')
        with self._observations_lock:
            if not self._observations:
                raise RuntimeError(
                    f'checkerboard ({self._board.describe()}) is not visible')
        for parent, child in (
            (self.get_parameter('base_frame').value,
             self.get_parameter('wrist_frame').value),
            (self.get_parameter('camera_root_frame').value,
             self.get_parameter('camera_optical_frame').value),
        ):
            try:
                self._lookup(parent, child)
            except TransformException as error:
                raise RuntimeError(
                    f'missing TF {parent} <- {child}: {error}') from error
        if not self._move_client.server_is_ready():
            if not self._move_client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError('MoveGroup action server is unavailable')

        state = None
        validation_poses = list(poses)
        if return_pose is not None:
            validation_poses.append(return_pose)
        count = len(validation_poses)
        valid_indices = []
        for index, pose in enumerate(validation_poses):
            if goal_handle.is_cancel_requested:
                raise asyncio.CancelledError
            label = (
                f'validating pose {index + 1}/{count}'
                if index < len(poses) else 'validating return pose')
            self._feedback(
                goal_handle, 'planning', label, index, count, 0)
            result, error = await self._send_move(
                goal_handle, pose, plan_only=True, start_state=state)
            is_return_pose = index >= len(self._pose_status)
            if result is None:
                if not is_return_pose and skip_failures:
                    # auto 档: 不可达/碰撞视点跳过, 由冗余候选补位
                    self._pose_status[index]['status'] = 'skipped'
                    self._pose_status[index]['reason'] = error
                    self._feedback(
                        goal_handle, 'planning',
                        f'{label} -> skipped ({error})', index + 1, count, 0)
                    continue
                if not is_return_pose:
                    self._pose_status[index]['status'] = 'plan_failed'
                    self._pose_status[index]['reason'] = error
                self._feedback(
                    goal_handle, 'planning', label, index + 1, count, 0)
                raise RuntimeError(
                    f'pose {index + 1} cannot be planned: {error}')
            if not is_return_pose:
                self._pose_status[index]['status'] = 'planned'
                valid_indices.append(index)
            state = self._final_state(result)
        if skip_failures and len(valid_indices) < required:
            raise RuntimeError(
                f'only {len(valid_indices)} of {len(poses)} viewpoints are '
                f'plannable; need {required}')
        self._feedback(goal_handle, 'planning', 'all poses plannable',
                       count, count, 0)
        if skip_failures:
            return [poses[index] for index in valid_indices]
        return poses

    def _generate_auto_poses(self):
        """由当前图像定位固定棋盘格, 生成保持视野的候选视点 (腕部位姿)."""
        if self._camera_info is None:
            raise RuntimeError('CameraInfo is unavailable')
        with self._observations_lock:
            latest = self._observations[-1][1] if self._observations else None
        if latest is None:
            raise RuntimeError(
                f'checkerboard ({self._board.describe()}) is not visible; '
                'place it in front of the camera first')
        base_from_wrist = self._lookup(
            self.get_parameter('base_frame').value,
            self.get_parameter('wrist_frame').value)
        configured = self.get_parameter('initial_extrinsics_file').value
        try:
            x0 = load_active_extrinsics(
                str(Path(configured)) if configured else None)
        except ValueError as error:
            raise RuntimeError(str(error)) from error
        camera_matrix = np.asarray(
            self._camera_info.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(
            self._camera_info.d, dtype=np.float64)
        image_size = (
            int(self._camera_info.width), int(self._camera_info.height))
        base_from_target = base_from_wrist @ x0 @ latest.camera_from_target
        if (not np.all(np.isfinite(base_from_target))
                or np.linalg.norm(base_from_target[:3, 3]) > 2.0):
            raise RuntimeError(
                'board estimate from initial extrinsics is not plausible '
                '(board center farther than 2 m from base); refusing to '
                'generate viewpoints')
        viewpoints = candidate_viewpoints(
            self._board,
            base_from_target,
            x0,
            camera_matrix,
            distortion,
            image_size,
            polar_degrees=[
                float(value)
                for value in self.get_parameter('auto_polar_degrees').value],
            azimuth_step_deg=float(
                self.get_parameter('auto_azimuth_step_deg').value),
            fill_fractions=[
                float(value)
                for value in self.get_parameter('auto_fill_fractions').value],
            margin_fraction=float(
                self.get_parameter('auto_margin_fraction').value),
            border_squares=float(
                self.get_parameter('auto_board_border_squares').value),
            distance_range_m=tuple(
                float(value) for value in
                self.get_parameter('auto_distance_range_m').value),
        )
        minimum = int(self.get_parameter('auto_min_poses').value)
        if len(viewpoints) < minimum:
            raise RuntimeError(
                f'FOV-qualified viewpoints {len(viewpoints)} < {minimum}; '
                'adjust board placement or relax auto_margin_fraction')
        wanted = minimum + int(
            self.get_parameter('auto_extra_candidates').value)
        min_span = float(self.get_parameter('auto_min_span_deg').value)
        selected, span = select_diverse(viewpoints, wanted, min_span)
        if span < min_span:
            raise RuntimeError(
                f'viewpoint rotation span {span:.1f}deg below '
                f'{min_span:.1f}deg; widen auto_polar_degrees')
        self._publish_status(
            'planning',
            f'auto: {len(selected)} viewpoints around board '
            f'(span {span:.1f}deg, board {self._board.describe()})',
            board_estimate={
                'xyz_m': [
                    round(float(value), 4)
                    for value in base_from_target[:3, 3]]},
        )
        return ([viewpoint.base_from_wrist for viewpoint in selected],
                [viewpoint.metadata() for viewpoint in selected])

    async def _wait_stable(self, goal_handle):
        timeout = float(self.get_parameter('stable_timeout_s').value)
        settle = float(self.get_parameter('settle_duration_s').value)
        deadline = time.monotonic() + timeout
        # 每次移动结束后重新计时, 避免上一段静止期被计入
        self._stable_since = None
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                raise asyncio.CancelledError
            if (
                self._stable_since is not None
                and time.monotonic() - self._stable_since >= settle
            ):
                return
            await asyncio.sleep(0.05)
        raise RuntimeError('robot did not become stationary')

    async def _capture(self, goal_handle, started_at, want_frames=False):
        """
        收集 frames_per_pose 帧 (观测, 腕部TF) 配对, 剔除离群帧后取均值.

        返回 (base_from_gripper, camera_from_target, reprojection_rms,
        frame_records, kept_frames); frame_records 为逐帧观测明细 (含 kept
        剔除标记), kept_frames 为 (观测, 腕部TF) 逐帧配对 (joint 档用).
        """
        count = int(self.get_parameter('frames_per_pose').value)
        deadline = time.monotonic() + float(
            self.get_parameter('sample_timeout_s').value)
        frames = []
        seen = set()
        while time.monotonic() < deadline and len(frames) < count:
            if goal_handle.is_cancel_requested:
                raise asyncio.CancelledError
            with self._observations_lock:
                available = list(self._observations)
            for stamp, observation in available:
                if stamp >= started_at and stamp not in seen:
                    seen.add(stamp)
                    try:
                        base_from_gripper = self._lookup(
                            self.get_parameter('base_frame').value,
                            self.get_parameter('wrist_frame').value)
                    except TransformException:
                        continue
                    frames.append((observation, base_from_gripper))
            await asyncio.sleep(0.04)
        if len(frames) < count:
            return None
        frames = frames[:count]

        # 按重投影 RMS 剔除离群帧
        inlier_mask = mad_mask_1d([
            observation.reprojection_rms_px
            for observation, _ in frames])
        kept = [frame for frame, keep in zip(frames, inlier_mask) if keep]
        if not kept:
            # 全部被剔除时回退为全部保留, kept 标记同步置真
            kept = frames
            inlier_mask = np.ones(len(frames), dtype=bool)

        # 逐帧观测记录 (存入位姿记录与 candidate yaml, 供前端展开核对)
        frame_records = []
        for (observation, transform), keep in zip(frames, inlier_mask):
            wrist_xyz, wrist_quat = transform_to_xyz_quat(transform)
            target_xyz, target_quat = transform_to_xyz_quat(
                observation.camera_from_target)
            frame_records.append({
                'reprojection_rms_px': float(observation.reprojection_rms_px),
                'base_from_gripper': {
                    'xyz': wrist_xyz, 'quat_xyzw': wrist_quat},
                'camera_from_target': {
                    'xyz': target_xyz, 'quat_xyzw': target_quat},
                'kept': bool(keep),
            })

        camera_from_target = mean_transform([
            observation.camera_from_target for observation, _ in kept])
        base_from_gripper = mean_transform([
            transform for _, transform in kept])
        reprojection = float(np.sqrt(np.mean([
            observation.reprojection_rms_px ** 2
            for observation, _ in kept])))
        return (base_from_gripper, camera_from_target, reprojection,
                frame_records, kept)

    # ------------------------------------------------------------------
    # Action 主流程
    # ------------------------------------------------------------------
    async def _execute(self, goal_handle):
        response = RunHandEyeCalibration.Result()
        try:
            # goal.method 为空串时回退到 solver_method 参数; 非法值直接终止
            method = goal_handle.request.method.strip() or str(
                self.get_parameter('solver_method').value)
            if method not in VALID_METHODS:
                raise ValueError(
                    f'非法的求解方法: {method}'
                    f'（可选: {"/".join(VALID_METHODS)}）')
            pose_source = (
                goal_handle.request.pose_source.strip()
                or str(self.get_parameter('pose_source').value))
            if pose_source not in ('poses', 'auto'):
                raise ValueError(
                    f'非法的位姿来源: {pose_source}（可选: poses|auto）')
            solve_target = (
                goal_handle.request.solve_target.strip()
                or str(self.get_parameter('solve_target').value))
            if solve_target not in ('hand_eye', 'joint'):
                raise ValueError(
                    f'非法的求解目标: {solve_target}（可选: hand_eye|joint）')
            viewpoint_metadata = None
            if pose_source == 'auto':
                poses, viewpoint_metadata = self._generate_auto_poses()
            else:
                poses = self._load_poses()
            self._pose_status = [
                {
                    'pose_index': index,
                    'status': 'pending',
                    **({'viewpoint': viewpoint_metadata[index]}
                       if viewpoint_metadata else {}),
                }
                for index in range(len(poses))
            ]
            initial_wrist = self._lookup(
                self.get_parameter('base_frame').value,
                self.get_parameter('wrist_frame').value)
            self._feedback(
                goal_handle, 'preflighting', 'checking camera, TF and MoveIt',
                0, len(poses), 0)
            poses = await self._preflight_and_plan(
                goal_handle,
                poses,
                initial_wrist if goal_handle.request.return_to_start else None,
                skip_failures=(pose_source == 'auto'),
                required=int(self.get_parameter('auto_min_poses').value),
            )
            if viewpoint_metadata is not None:
                # 预检可能跳过不可达视点: 对齐元数据/状态与有效位姿列表
                kept_indices = [
                    index for index, entry in enumerate(self._pose_status)
                    if entry['status'] == 'planned']
                skipped = [entry for entry in self._pose_status
                           if entry['status'] == 'skipped']
                viewpoint_metadata = [
                    viewpoint_metadata[index] for index in kept_indices]
                self._pose_status = [
                    {'pose_index': new_index, 'status': 'planned',
                     'viewpoint': viewpoint_metadata[new_index]}
                    for new_index in range(len(kept_indices))
                ] + skipped
            if goal_handle.request.plan_only:
                response.success = True
                response.message = f'all {len(poses)} poses are plannable'
                goal_handle.succeed()
                self._publish_status(
                    'plan_ready', response.message,
                    progress=1.0, pose_count=len(poses),
                    poses=self._pose_status)
                return response

            samples = []
            joint_observations = []
            manifests = []
            want_frames = solve_target == 'joint'
            for index, pose in enumerate(poses):
                if goal_handle.is_cancel_requested:
                    raise asyncio.CancelledError
                entry = self._pose_status[index]
                self._feedback(
                    goal_handle, 'moving',
                    f'moving to pose {index + 1}/{len(poses)}',
                    index, len(poses), len(samples))
                move_result, error = await self._send_move(
                    goal_handle, pose, plan_only=False)
                if move_result is None:
                    entry['status'] = 'move_failed'
                    entry['reason'] = error
                    raise RuntimeError(
                        f'pose {index + 1} execution failed: {error}')
                entry['status'] = 'settling'
                self._feedback(
                    goal_handle, 'settling',
                    f'waiting for robot to settle at pose {index + 1}',
                    index, len(poses), len(samples))
                await self._wait_stable(goal_handle)
                capture = await self._capture(
                    goal_handle, time.monotonic(), want_frames=want_frames)
                if capture is None:
                    entry['status'] = 'capture_failed'
                    entry['reason'] = 'checkerboard detection timeout'
                    manifests.append({
                        'pose_index': index + 1, 'accepted': False,
                        'reason': 'checkerboard detection timeout',
                    })
                    self._feedback(
                        goal_handle, 'capturing',
                        f'pose {index + 1} rejected: detection timeout',
                        index + 1, len(poses), len(samples))
                    continue
                (base_from_gripper, camera_from_target, reprojection,
                 frame_records, kept_frames) = capture
                if want_frames:
                    # joint 档用逐帧原始配对 (不做 SE3 均值):
                    # 每视点取帧内重投影 RMS 最小的前 N 帧角点
                    frame_limit = int(
                        self.get_parameter('joint_frames_per_pose').value)
                    ranked = sorted(
                        kept_frames,
                        key=lambda pair: pair[0].reprojection_rms_px,
                    )[:frame_limit]
                    for frame_index, (observation, frame_transform) in \
                            enumerate(ranked):
                        joint_observations.append(JointObservation(
                            frame_transform,
                            observation.corners.reshape(-1, 2).astype(
                                np.float64),
                            f'pose_{index + 1:02d}_f{frame_index + 1}',
                        ))
                samples.append(CalibrationSample(
                    base_from_gripper,
                    camera_from_target,
                    reprojection,
                    f'pose_{index + 1:02d}',
                ))
                entry['status'] = 'sampled'
                entry['reprojection_rms_px'] = reprojection
                entry['frames'] = frame_records
                manifests.append({
                    'pose_index': index + 1,
                    'accepted': True,
                    'reprojection_rms_px': reprojection,
                    'frames': frame_records,
                })
                self._feedback(
                    goal_handle, 'capturing',
                    f'pose {index + 1} sampled ({len(samples)} accepted)',
                    index + 1, len(poses), len(samples))

            if goal_handle.request.return_to_start:
                self._feedback(
                    goal_handle, 'returning', 'returning to initial pose',
                    len(poses), len(poses), len(samples))
                returned, error = await self._send_move(
                    goal_handle, initial_wrist, plan_only=False)
                if returned is None:
                    raise RuntimeError(f'failed to return to start: {error}')

            self._feedback(
                goal_handle, 'solving', 'solving hand-eye transform',
                len(poses), len(poses), len(samples))
            joint = None
            if solve_target == 'joint':
                if not joint_observations:
                    raise RuntimeError('joint solve: no captured frames')
                joint = solve_joint_intrinsics_hand_eye(
                    joint_observations,
                    self._board,
                    (int(self._camera_info.width),
                     int(self._camera_info.height)),
                    K0=np.asarray(
                        self._camera_info.k,
                        dtype=np.float64).reshape(3, 3),
                    D0=np.asarray(self._camera_info.d, dtype=np.float64),
                    joint_max_reprojection_rms_px=float(
                        self.get_parameter(
                            'joint_max_reprojection_rms_px').value),
                    max_translation_rms_m=float(
                        self.get_parameter('max_translation_rms_m').value),
                    max_rotation_rms_deg=float(
                        self.get_parameter('max_rotation_rms_deg').value),
                    min_rotation_span_deg=float(
                        self.get_parameter('min_rotation_span_deg').value),
                )
                result = CalibrationResult(
                    gripper_from_camera=joint.gripper_from_camera,
                    base_from_target=joint.base_from_target,
                    method=f'joint:{joint.init_method}',
                    accepted_indices=[
                        index for index, view in enumerate(joint.per_view)
                        if view['accepted']],
                    rejected_indices=[
                        index for index, view in enumerate(joint.per_view)
                        if not view['accepted']],
                    translation_rms_m=joint.translation_rms_m,
                    rotation_rms_deg=joint.rotation_rms_deg,
                    reprojection_rms_px=joint.reprojection_rms_px,
                    rotation_span_deg=joint.rotation_span_deg,
                    passed=joint.passed,
                    failures=list(joint.failures),
                    sample_errors=[
                        {'index': index, 'accepted': view['accepted'],
                         'rms_px': view['rms_px'],
                         'sample_id': view['sample_id']}
                        for index, view in enumerate(joint.per_view)],
                    method_scores=[],
                    refine_stats=dict(joint.polish),
                )
            else:
                result = solve_hand_eye(
                    samples,
                    min_samples=int(self.get_parameter('min_samples').value),
                    max_reprojection_rms_px=float(
                        self.get_parameter('max_reprojection_rms_px').value),
                    max_translation_rms_m=float(
                        self.get_parameter('max_translation_rms_m').value),
                    max_rotation_rms_deg=float(
                        self.get_parameter('max_rotation_rms_deg').value),
                    min_rotation_span_deg=float(
                        self.get_parameter('min_rotation_span_deg').value),
                    method=method,
                )
            camera_root_from_optical = self._lookup(
                self.get_parameter('camera_root_frame').value,
                self.get_parameter('camera_optical_frame').value)
            wrist_from_root = (
                result.gripper_from_camera @ inverse(camera_root_from_optical))
            candidate_id = datetime.now(timezone.utc).strftime(
                '%Y%m%dT%H%M%S_%fZ')
            frames = {
                'base': self.get_parameter('base_frame').value,
                'wrist': self.get_parameter('wrist_frame').value,
                'camera_root': self.get_parameter('camera_root_frame').value,
                'camera_optical':
                    self.get_parameter('camera_optical_frame').value,
            }
            path = write_candidate(
                candidate_id,
                result.gripper_from_camera,
                wrist_from_root,
                result.base_from_target,
                result,
                frames,
                self._board.metadata(),
                manifests,
                method_scores=result.method_scores,
                sample_errors=result.sample_errors,
                refine_stats=result.refine_stats,
                intrinsics=(
                    joint.intrinsics_document() if joint is not None
                    else None),
                viewpoints=viewpoint_metadata,
            )
            response.success = bool(result.passed)
            joint_summary = ''
            if joint is not None:
                k_matrix = joint.camera_matrix
                joint_summary = (
                    f'joint RMS {joint.reprojection_rms_px:.3f}px; '
                    f'K fx={k_matrix[0, 0]:.2f} fy={k_matrix[1, 1]:.2f} '
                    f'cx={k_matrix[0, 2]:.1f} cy={k_matrix[1, 2]:.1f}; ')
                response.camera_matrix = [
                    float(value) for value in k_matrix.reshape(-1)]
                response.distortion_coefficients = [
                    float(value) for value in np.ravel(joint.distortion)]
                response.joint_reprojection_rms_px = float(
                    joint.reprojection_rms_px)
            response.message = (
                f'{joint_summary}quality gates passed; review before activation'
                if result.passed else
                f'{joint_summary}' + '; '.join(result.failures)
            )
            response.candidate_id = candidate_id
            response.wrist_to_camera_optical = _transform_message(
                result.gripper_from_camera)
            response.accepted_samples = len(result.accepted_indices)
            response.rejected_samples = (
                sum(1 for entry in manifests if not entry['accepted'])
                + len(result.rejected_indices))
            response.translation_rms_m = result.translation_rms_m
            response.rotation_rms_deg = result.rotation_rms_deg
            response.reprojection_rms_px = result.reprojection_rms_px
            response.result_file = str(path)
            xyz, quaternion = transform_to_xyz_quat(result.gripper_from_camera)
            target_xyz, target_quat = transform_to_xyz_quat(
                result.base_from_target)
            # 求解明细区段: 方法打分/逐样本残差/精化统计/标定板基座系估计,
            # 与 candidate yaml 中的同名字段结构一致
            result_details = {
                'method': result.method,
                'method_scores': result.method_scores,
                'sample_errors': result.sample_errors,
                'refine_stats': result.refine_stats,
                'base_from_target': {
                    'xyz': target_xyz, 'quat_xyzw': target_quat},
            }
            metrics = {
                'accepted_samples': response.accepted_samples,
                'rejected_samples': response.rejected_samples,
                'translation_rms_m': result.translation_rms_m,
                'rotation_rms_deg': result.rotation_rms_deg,
                'reprojection_rms_px': result.reprojection_rms_px,
                'rotation_span_deg': result.rotation_span_deg,
            }
            gates = {
                'min_samples': int(self.get_parameter('min_samples').value),
                'max_reprojection_rms_px': float(
                    self.get_parameter('max_reprojection_rms_px').value),
                'max_translation_rms_m': float(
                    self.get_parameter('max_translation_rms_m').value),
                'max_rotation_rms_deg': float(
                    self.get_parameter('max_rotation_rms_deg').value),
                'min_rotation_span_deg': float(
                    self.get_parameter('min_rotation_span_deg').value),
            }
            if joint is not None:
                metrics['joint_reprojection_rms_px'] = (
                    joint.reprojection_rms_px)
                result_details['intrinsics'] = {
                    'camera_matrix': [
                        float(value)
                        for value in joint.camera_matrix.reshape(-1)],
                    'distortion': [
                        float(value)
                        for value in np.ravel(joint.distortion)],
                    'distortion_model': 'plumb_bob',
                    'image_size': list(joint.image_size),
                }
            if viewpoint_metadata is not None:
                result_details['viewpoints'] = viewpoint_metadata
            if goal_handle.is_cancel_requested or not goal_handle.is_active():
                # 求解/落盘期间收到取消：goal 已进 CANCELING，再 succeed/
                # abort 属非法状态迁移（抛异常且 goal 永卡 CANCELING），
                # 改走 canceled 收口
                response.success = False
                response.message = 'calibration cancelled during solve'
                goal_handle.canceled()
                self._publish_status(
                    'cancelled', response.message, poses=self._pose_status)
                return response
            try:
                if result.passed:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
            except Exception:
                # 取消请求与终态迁移竞态：迁移已非法时回退 canceled，
                # 避免 goal 卡死在 CANCELING
                response.success = False
                response.message = 'calibration cancelled; robot hold requested'
                try:
                    goal_handle.canceled()
                except Exception:
                    pass
                self._publish_status(
                    'cancelled', response.message, poses=self._pose_status)
                return response
            self._publish_status(
                'complete' if result.passed else 'quality_failed',
                response.message,
                candidate_id=candidate_id,
                quality_passed=bool(result.passed),
                result_file=str(path),
                method=result.method,
                metrics=metrics,
                gates=gates,
                result=result_details,
                extrinsics={
                    'wrist_from_camera_optical': {
                        'xyz_m': xyz, 'quaternion_xyzw': quaternion},
                },
                poses=self._pose_status,
            )
            return response
        except asyncio.CancelledError:
            response.success = False
            response.message = 'calibration cancelled; robot hold requested'
            goal_handle.canceled()
            self._publish_status(
                'cancelled', response.message, poses=self._pose_status)
            return response
        except Exception as error:  # Action boundary: report all failures.
            response.success = False
            response.message = str(error)
            goal_handle.abort()
            self._publish_status(
                'failed', response.message, poses=self._pose_status)
            self.get_logger().error(f'calibration failed: {error}')
            return response
        finally:
            self._active_move_goal = None
            self._release_goal()

    def _activate(self, request, response):
        with self._goal_lock:
            busy = self._busy
        if busy:
            response.success = False
            response.message = 'cannot activate while calibration is running'
            return response
        try:
            path = activate_candidate(request.candidate_id)
            reload_note = 'extrinsics publisher not available; restart it to apply'
            if self._reload_client.service_is_ready():
                future = self._reload_client.call_async(Trigger.Request())
                deadline = time.monotonic() + 5.0
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.02)
                if future.done() and future.result() is not None:
                    reload_response = future.result()
                    if not reload_response.success:
                        raise RuntimeError(
                            f'extrinsics reload failed: '
                            f'{reload_response.message}')
                    reload_note = 'extrinsics reloaded'
                else:
                    reload_note = 'extrinsics reload timed out; reload manually'
            response.success = True
            response.message = f'candidate activated; {reload_note}'
            response.active_result_file = str(path)
            self._publish_status(
                'activated', response.message,
                candidate_id=request.candidate_id)
        except (OSError, KeyError, TypeError, ValueError, RuntimeError,
                yaml.YAMLError) as error:
            response.success = False
            response.message = str(error)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
