# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""ROS 2 action server for safe, automatic eye-in-hand calibration.

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
from .detector import CheckerboardDetector
from .solver import CalibrationSample, solve_hand_eye, VALID_METHODS
from .storage import activate_candidate, write_candidate
from .transforms import (
    inverse,
    mean_transform,
    transform_from_xyz_quat,
    transform_to_xyz_quat,
)


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


def _mad_filter(values, threshold=3.5):
    """返回 MAD 内点掩码; 样本太少时全部保留。"""
    values = np.asarray(values, dtype=np.float64)
    if values.size < 4:
        return np.ones(values.size, dtype=bool)
    median = np.median(values)
    mad = np.median(np.abs(values - median))
    scale = max(1.4826 * mad, 1e-9)
    return np.abs(values - median) / scale <= threshold


class CalibrationServer(Node):
    def __init__(self):
        super().__init__('hand_eye_calibration_server')
        self._group = ReentrantCallbackGroup()
        defaults = Path(get_package_share_directory(
            'aubo_hand_eye_calibration')) / 'config'
        self.declare_parameter('poses_file', str(defaults / 'poses.yaml'))
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('wrist_frame', 'wrist3_Link')
        self.declare_parameter('camera_root_frame', 'camera_link')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('move_group', 'manipulator_e5')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('board_columns', 11)
        self.declare_parameter('board_rows', 8)
        self.declare_parameter('board_square_size_m', 0.020)
        self.declare_parameter('frames_per_pose', 5)
        self.declare_parameter('min_samples', 12)
        self.declare_parameter('joint_velocity_threshold', 0.01)
        self.declare_parameter('settle_duration_s', 0.5)
        self.declare_parameter('stable_timeout_s', 10.0)
        self.declare_parameter('sample_timeout_s', 4.0)
        self.declare_parameter('velocity_scaling', 0.15)
        self.declare_parameter('acceleration_scaling', 0.15)
        self.declare_parameter('planning_attempts', 5)
        self.declare_parameter('planning_time_s', 5.0)
        self.declare_parameter('position_tolerance_m', 0.001)
        self.declare_parameter('orientation_tolerance_rad', 0.01)
        self.declare_parameter('max_reprojection_rms_px', 1.0)
        self.declare_parameter('max_translation_rms_m', 0.003)
        self.declare_parameter('max_rotation_rms_deg', 0.5)
        self.declare_parameter('min_rotation_span_deg', 20.0)
        # 求解方法: auto|tsai|park|horaud|andreff|daniilidis;
        # goal.method 为空串时取此参数
        self.declare_parameter('solver_method', 'auto')

        self._board = board_from_node(self)
        self._detector = CheckerboardDetector(
            self._board,
            max_reprojection_rms_px=float(
                self.get_parameter('max_reprojection_rms_px').value) * 1.5,
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
    async def _preflight_and_plan(self, goal_handle, poses, return_pose=None):
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
                if not is_return_pose:
                    self._pose_status[index]['status'] = 'plan_failed'
                    self._pose_status[index]['reason'] = error
                self._feedback(
                    goal_handle, 'planning', label, index + 1, count, 0)
                raise RuntimeError(
                    f'pose {index + 1} cannot be planned: {error}')
            if not is_return_pose:
                self._pose_status[index]['status'] = 'planned'
            state = self._final_state(result)
        self._feedback(goal_handle, 'planning', 'all poses plannable',
                       count, count, 0)

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

    async def _capture(self, goal_handle, started_at):
        """收集 frames_per_pose 帧 (观测, 腕部TF) 配对, 剔除离群帧后取均值。

        返回 (base_from_gripper, camera_from_target, reprojection_rms,
        frame_records); frame_records 为逐帧观测明细 (含 kept 剔除标记)。
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
        inlier_mask = _mad_filter([
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
        return base_from_gripper, camera_from_target, reprojection, frame_records

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
            poses = self._load_poses()
            self._pose_status = [
                {'pose_index': index, 'status': 'pending'}
                for index in range(len(poses))
            ]
            initial_wrist = self._lookup(
                self.get_parameter('base_frame').value,
                self.get_parameter('wrist_frame').value)
            self._feedback(
                goal_handle, 'preflighting', 'checking camera, TF and MoveIt',
                0, len(poses), 0)
            await self._preflight_and_plan(
                goal_handle,
                poses,
                initial_wrist if goal_handle.request.return_to_start else None,
            )
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
            manifests = []
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
                capture = await self._capture(goal_handle, time.monotonic())
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
                base_from_gripper, camera_from_target, reprojection, \
                    frame_records = capture
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
            )
            response.success = bool(result.passed)
            response.message = (
                'quality gates passed; review before activation'
                if result.passed else '; '.join(result.failures)
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
            if result.passed:
                goal_handle.succeed()
            else:
                goal_handle.abort()
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
