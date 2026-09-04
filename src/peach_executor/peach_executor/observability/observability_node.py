"""
桃子采摘链路监控 Web：只读观测 + 鉴权手动调试操作面（同端口 8090）.

只读面（2026-08-13 起）：回答「现在跑到哪一步、各步数据是什么、当前
参数是什么」，问题定位依靠过程监测。调试操作面（2026-09 融合，决策
0007 推翻条款执行）：POST /api/debug/<action> 转发到既有动作/服务，
三重门控——debug.enabled 总开关、X-Debug-Token 令牌、运动类另需
debug.motion_enabled——全部默认关；每次操作（含被拒）写审计 JSONL。
技能侧 ExecutionAuthority 等既有安全门不受影响：Web 只是又一个客户端。
"""

from __future__ import annotations

import hmac
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from aubo_msgs.msg import RobotStatus
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Path as NavPath
from peach_executor.batch import resolve_runs_root
from peach_interfaces.msg import (
    BagFittingArray,
    BagGraspCandidateArray,
    CanonicalEvent,
    GraspDecision,
    GraspHypothesis,
    HarvestState,
    PeachTargetObservationArray,
    ReconstructionStatus,
)
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import MarkerArray

from . import http_server
from .audit import DebugAudit
from .debug_actions import DebugBridge, is_motion
from .params import declare as _declare_params
from .params import from_params as _from_params
from .recorder import (
    candidate_array,
    fitting_array,
    grasp_decision,
    grasp_hypothesis,
    harvest_event,
    parse_json_text,
    reconstruction_status,
    Recorder,
    robot_status,
    target_observations,
    vector_stamped,
)
from .ros_viz import marker_array_from_dicts, path_from_xyz
from .state import MetricsSampler, ObservabilityState
from .tcp_trajectory import (
    build_selection_marker_dicts,
    build_tcp_marker_dicts,
    downsample_path,
    TcpPathBuffer,
)


# 过程监测需要回答「当前以什么参数在跑」：按节点分组的只读参数白名单。
PARAM_WATCHLIST = {
    '/peach_scene_perception_node': [
        'yolo_conf', 'min_detection_conf',
        'target_memory.match_radius_m', 'target_memory.recovery_scale',
        'target_memory.confirm_frames',
        'harvest.min_collect_frames', 'harvest.lock_settle_frames',
        'harvest.max_collect_s', 'harvest.priority_prefer_lower_first',
    ],
    '/peach_target_reconstruction_node': [
        'capture.min_views', 'capture.recommended_views',
        'capture.min_mask_depth_ratio', 'capture.require_target_mask',
        'icp.enable', 'tsdf.enable',
    ],
    '/peach_manipulation_node': [
        'moveit.velocity_scaling', 'moveit.acceleration_scaling',
        'moveit.transit_velocity_scaling', 'moveit.transit_acceleration_scaling',
        'scan.observation_radius_m', 'scan.minimum_radius_m',
        'scan.frame_wait_s', 'scan.maximum_moves',
        'quality.minimum_views', 'quality.minimum_baseline_deg',
        'quality.maximum_refined_rmse_m',
        'execution.enabled', 'grasp.enabled', 'tool.enabled',
        'photo_pose_joint_tolerance_rad', 'photo_pose_max_joint_vel_rad_s',
    ],
    '/peach_executor': [
        'execution_enabled', 'survey_wait_s', 'survey_dwell_s',
        'empty_survey_limit',
        'persist_ledger',
    ],
}


def _parameter_scalar(value) -> object:
    """rcl_interfaces/ParameterValue → 标量（数组取列表，未设置给 None）."""
    kind = value.type
    if kind == 1:
        return bool(value.bool_value)
    if kind == 2:
        return int(value.integer_value)
    if kind == 3:
        return float(value.double_value)
    if kind == 4:
        return str(value.string_value)
    if kind == 6:
        return [bool(item) for item in value.bool_array_value]
    if kind == 7:
        return [int(item) for item in value.integer_array_value]
    if kind == 8:
        return [float(item) for item in value.double_array_value]
    if kind == 9:
        return [str(item) for item in value.string_array_value]
    return None


class ObservabilityNode(LifecycleNode):
    """订阅采摘链路各阶段输出，提供只读监控 API 与鉴权调试操作面."""

    def __init__(self):
        """声明参数；订阅与 HTTP 等到 configure / activate."""
        super().__init__('peach_observability')
        self._param_listener = None
        self._params = None
        self._state = ObservabilityState()
        self._http = None
        self._metrics = None
        self._recorder = None
        self._subs = []
        self._param_clients = {}
        self._param_timer = None
        self._param_inflight = set()
        # 轮询定时器/参数服务回调与订阅回调共享再入组（触发状态镜像并发写）。
        self._cb = ReentrantCallbackGroup()
        # 重建镜像合并缓存：调试 JSON 明细（tsdf/registration/refined 等）与
        # 最近一次许可镜像——类型化诊断到达时并入，保持镜像/落盘信息不缩水
        self._recon_debug_extra: dict = {}
        self._recon_decision_value: dict | None = None
        self._tf_buffer = None
        self._tf_listener = None
        self._traj_timer = None
        self._tcp_path = None
        self._tcp_path_pub = None
        self._tcp_marker_pub = None
        self._traj_landmarks = {}
        self._last_viz_ns = 0
        self._last_marker_sig = None
        self._traj_ctx = {
            'moving': False,
            'target_id': '',
            'phase': 0,
            'skill': '',
        }
        self._traj_run_id = ''
        # 调试操作面（默认关；debug.enabled=true 才建桥）
        self._debug_bridge: DebugBridge | None = None
        self._debug_audit: DebugAudit | None = None

    def on_configure(self, state):
        del state
        try:
            # 官方 generate_parameter_library_py 装载链：on_configure 内声明
            # （declare 期校验非法值即失败，节点停在 Unconfigured 可查日志）
            self._param_listener = _declare_params(self)
            self._params = _from_params(self._param_listener.get_params())
        except Exception as exc:  # noqa: BLE001 参数库校验异常类型跨 rclpy 版本
            self.get_logger().error(f'参数非法: {exc}')
            return TransitionCallbackReturn.FAILURE
        self._recorder = Recorder(
            root_dir=str(resolve_runs_root(self._params.record_root_dir)),
            enabled=self._params.record_enabled,
            save_images=self._params.record_save_images,
            save_clouds=self._params.record_save_clouds,
            on_info=lambda info: self._state.update('record', 'info', info),
            log_warning=lambda msg: self.get_logger().warning(msg))
        self._create_subscriptions()
        self._create_param_watchers()
        self._create_metrics_sampler()
        self._create_tcp_sampler()
        self._create_debug_bridge()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        result = super().on_activate(state)
        self.start_http()
        return result

    def on_deactivate(self, state):
        self._stop_runtime()
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self._stop_runtime()
        self._release_resources()
        return super().on_cleanup(state)

    def _topic(self, parameter: str) -> str:
        """从不可变快照取话题名（启动期建订阅用）."""
        return self._params.topics[parameter]

    def _subscribe(self, *args, **kwargs):
        """建订阅并记下句柄，供 on_cleanup 释放."""
        sub = self.create_subscription(*args, **kwargs)
        self._subs.append(sub)
        return sub

    def _create_subscriptions(self) -> None:
        """建立全部只读订阅."""
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._subscribe(
            PeachTargetObservationArray,
            self._topic('target_observations_topic'),
            self._targets_callback, reliable_qos)
        self._subscribe(
            String, self._topic('harvest_state_topic'),
            self._harvest_callback, latched_qos)
        self._subscribe(
            String, self._topic('reconstruction_status_topic'),
            self._recon_status_callback, latched_qos)
        self._subscribe(
            ReconstructionStatus,
            self._topic('reconstruction_diagnostics_topic'),
            self._recon_diagnostics_callback, latched_qos)
        # 调试明细（tsdf/registration/overlap/refined/逐机位）：并入镜像与落盘，
        # 保持「类型化后过程数据不缩水」；类型化字段为准，明细键补充
        self._subscribe(
            String, self._topic('reconstruction_diagnostics_debug_topic'),
            self._recon_debug_callback, latched_qos)
        self._subscribe(
            GraspDecision, self._topic('grasp_decision_topic'),
            self._recon_decision_callback, latched_qos)
        self._subscribe(
            BagGraspCandidateArray, self._topic('refined_pose_topic'),
            lambda msg: self._state.update(
                'refined', 'pose', candidate_array(msg)), latched_qos)
        self._subscribe(
            Vector3Stamped, self._topic('refined_axis_topic'),
            lambda msg: self._state.update(
                'refined', 'axis', vector_stamped(msg)), latched_qos)
        self._subscribe(
            BagFittingArray, self._topic('refined_diagnostics_topic'),
            lambda msg: self._state.update(
                'refined', 'diagnostics', fitting_array(msg)), latched_qos)
        self._subscribe(
            String, self._topic('manipulation_status_topic'),
            self._manipulation_callback, latched_qos)
        self._subscribe(
            GraspHypothesis, self._topic('grasp_hypothesis_topic'),
            self._grasp_hypothesis_callback, latched_qos)
        self._subscribe(
            HarvestState, self._topic('task_executor_state_topic'),
            self._task_executor_callback, latched_qos)
        self._subscribe(
            CanonicalEvent, self._topic('task_executor_events_topic'),
            self._events_callback,
            QoSProfile(
                depth=50,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE))
        self._subscribe(
            RobotStatus, self._topic('robot_status_topic'),
            self._robot_status_callback, reliable_qos)
        # 记录器图像/点云订阅：只在对应开关开启时建立（省带宽）
        if self._params.record_enabled:
            if self._params.record_save_images:
                self._subscribe(
                    Image, self._topic('debug_image_topic'),
                    self._recorder.handle_image, reliable_qos)
                # 真相流画布并行落盘（raw_img_* 前缀）：与稳定流 img_* 成对，
                # 筛选前后对比不依赖 RViz
                self._subscribe(
                    Image, self._topic('debug_image_raw_topic'),
                    self._recorder.handle_raw_image, reliable_qos)
            if self._params.record_save_clouds:
                self._subscribe(
                    PointCloud2, self._topic('tsdf_cloud_topic'),
                    self._recorder.handle_cloud, latched_qos)

    def _robot_status_callback(self, message: RobotStatus) -> None:
        """机械臂柜侧状态；in_motion 给 TCP 采样当运动标记."""
        self._traj_ctx['moving'] = bool(message.in_motion)
        self._state.update('robot', 'status', robot_status(message))

    def _harvest_callback(self, message: String) -> None:
        """
        感知采摘计划：进状态缓存并喂记录器（perception.jsonl 并入）.

        JSON 全量透传：发布侧新增键（如阶段 D1 的 anchor_stale_target_ids/
        out_of_view_target_ids/dropped_target_ids/lighting/low_light_quality）
        无需本侧改动即进入 /api/state 镜像。
        """
        value = parse_json_text(message.data)
        self._state.update('perception', 'harvest', value)
        self._recorder.handle_harvest(value)
        self._record_job()

    def _recon_status_callback(self, message: String) -> None:
        """重建状态文本：进状态缓存并喂记录器（reconstruction.jsonl）."""
        value = parse_json_text(message.data, 'state')
        self._state.update('reconstruction', 'status', value)
        self._recorder.handle_reconstruction('status', value)

    def _recon_diagnostics_callback(
            self, message: ReconstructionStatus) -> None:
        """
        重建 1Hz 结构化诊断：消息字段重建镜像 dict 并喂记录器.

        合并调试 JSON 明细（tsdf/registration/refined 等）与最近许可镜像，
        保持镜像/落盘键集与旧裸 JSON 契约一致（reconstruction_final 摘要
        与前端 tsdf 计时依赖这些键）。
        """
        merged = dict(self._recon_debug_extra)
        merged.update(reconstruction_status(message))
        if self._recon_decision_value is not None:
            merged['grasp_decision'] = self._recon_decision_value
        center = merged.get('target_center_base')
        if isinstance(center, list):
            self._traj_landmarks['reconstruction_center'] = center
        self._state.update('reconstruction', 'diagnostics', merged)
        self._recorder.handle_reconstruction('diagnostics', merged)

    def _recon_debug_callback(self, message: String) -> None:
        """重建调试明细 JSON：只更新合并缓存，不直接落盘（防重复记录）."""
        self._recon_debug_extra = parse_json_text(message.data)

    def _recon_decision_callback(self, message: GraspDecision) -> None:
        """重建抓取许可：消息字段重建镜像 dict 并喂记录器."""
        value = grasp_decision(message)
        self._recon_decision_value = value
        self._traj_landmarks['grasp_entry'] = value.get('entry')
        self._traj_landmarks['grasp_pregrasp'] = value.get('pregrasp')
        self._traj_landmarks['axis'] = value.get('axis')
        self._traj_landmarks['target_id'] = value.get('target_id') or ''
        self._state.update('reconstruction', 'grasp_decision', value)
        self._recorder.handle_reconstruction('grasp_decision', value)
        self._record_job()

    def _manipulation_callback(self, message: String) -> None:
        """技能节点状态：进状态缓存并喂记录器（manipulation.jsonl）."""
        value = parse_json_text(message.data)
        self._traj_ctx['skill'] = str(value.get('state') or '')
        self._state.update('manipulation', 'status', value)
        self._recorder.handle_manipulation(value)
        self._record_job()

    def _grasp_hypothesis_callback(self, message: GraspHypothesis) -> None:
        """技能抓取假设：进状态缓存并并入 manipulation.jsonl."""
        value = grasp_hypothesis(message)
        self._state.update('manipulation', 'hypothesis', value)
        self._recorder.handle_hypothesis(value)
        self._record_job()

    def _events_callback(self, message: CanonicalEvent) -> None:
        """批次事件进环形缓冲，供前端事件时间线消费."""
        try:
            value = harvest_event(message)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning(f'事件转换失败: {error}')
            return
        # 缓冲上限来自启动期快照（A9）：已校验 >= 1，运行期不再直读参数
        self._state.append_event(value, self._params.event_buffer_size)
        self._recorder.handle_event(value)

    def _task_executor_callback(self, message: HarvestState) -> None:
        """把调度节点类型化状态转换为稳定的浏览器对象."""
        value = {
            'revision': message.revision,
            'run_id': message.run_id,
            'cycle_id': message.cycle_id,
            'target_id': message.target_id,
            'operation_mode': message.operation_mode,
            'batch_state': message.batch_state,
            'target_phase': message.target_phase,
            'action_active': message.action_active,
            'auto_start_enabled': message.auto_start_enabled,
            'execution_enabled': message.execution_enabled,
            'grasp_enabled': message.grasp_enabled,
            'tool_enabled': message.tool_enabled,
            'recovery_required': message.recovery_required,
            'progress': message.progress,
            'message': message.message,
            'blockers': list(message.blockers),
        }
        self._state.update('task_executor', 'state', value)
        self._traj_ctx['target_id'] = str(message.target_id or '')
        self._traj_ctx['phase'] = int(message.target_phase or 0)
        run_id = str(message.run_id or '')
        if run_id and run_id != self._traj_run_id:
            self._traj_run_id = run_id
            if self._tcp_path is not None:
                self._tcp_path.clear()
        self._recorder.handle_state(value)
        self._record_job()

    def _targets_callback(self, message) -> None:
        try:
            value = target_observations(message)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning(f'目标快照转换失败: {error}')
            return
        self._state.update('perception', 'targets', value)
        self._recorder.handle_targets(value)

    # ------------------------------------------------------------------
    # 参数镜像：周期轮询白名单节点的 get_parameters，只读不写
    # ------------------------------------------------------------------
    def _create_param_watchers(self) -> None:
        """为白名单节点建 get_parameters 客户端并启动轮询定时器."""
        self._param_clients = {
            name: self.create_client(
                GetParameters, f'{name}/get_parameters',
                callback_group=self._cb)
            for name in PARAM_WATCHLIST
        }
        # 服务未就绪时静默跳过（节点可能未启动），不刷错误日志
        self._param_inflight = set()
        self._param_timer = self.create_timer(
            self._params.param_poll_period_s, self._poll_params,
            callback_group=self._cb)

    def _poll_params(self) -> None:
        """对就绪的参数服务发起异步查询（在途请求去重）."""
        for name, client in self._param_clients.items():
            if name in self._param_inflight or not client.service_is_ready():
                continue
            request = GetParameters.Request()
            request.names = list(PARAM_WATCHLIST[name])
            self._param_inflight.add(name)
            future = client.call_async(request)
            future.add_done_callback(
                lambda fut, node_name=name: self._on_params(node_name, fut))

    def _on_params(self, node_name: str, future) -> None:
        """落参数镜像到状态缓存；异常仅降级为空镜像."""
        self._param_inflight.discard(node_name)
        try:
            response = future.result()
        except (RuntimeError, rclpy.exceptions.RCLError):
            return
        if response is None:
            return
        values = {
            name: _parameter_scalar(value)
            for name, value in zip(
                PARAM_WATCHLIST[node_name], response.values)
        }
        self._state.update_params(node_name, values)

    # ------------------------------------------------------------------
    # 性能采样：独立线程写 state，绝不占用 ROS 回调线程
    # ------------------------------------------------------------------
    def _create_metrics_sampler(self) -> None:
        """按快照参数建性能采样线程（GPU 不可用时自动降级为 None）."""
        self._metrics = MetricsSampler(
            self._params.metrics_period_s,
            list(self._params.metrics_process_patterns),
            self._metrics_callback,
            lambda msg: self.get_logger().warning(msg))

    def _create_tcp_sampler(self) -> None:
        """Latest TF 采 tcp；重建积分仍用精确时间戳，这里只给监控/落盘."""
        if not self._params.trajectory_enabled:
            return
        self._tcp_path = TcpPathBuffer(
            max_points=self._params.trajectory_max_points,
            min_step_m=self._params.trajectory_min_step_m)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._tcp_path_pub = self.create_publisher(
            NavPath, self._topic('tcp_path_topic'), latched)
        self._tcp_marker_pub = self.create_publisher(
            MarkerArray, self._topic('tcp_markers_topic'), latched)
        self._traj_timer = self.create_timer(
            self._params.trajectory_period_s, self._sample_tcp,
            callback_group=self._cb)

    def _publish_tcp_summary(self) -> None:
        """把路径摘要写入 /api/state 的 robot.tcp（不含全点列）."""
        if self._tcp_path is None:
            return
        summary = self._tcp_path.summary()
        summary['frame_id'] = self._params.trajectory_base_frame
        summary['tip_frame'] = self._params.trajectory_tip_frame
        self._state.update('robot', 'tcp', summary)

    def _sample_tcp(self) -> None:
        """周期查询 base←tip；位移过门槛才落盘."""
        if (
            self._tcp_path is None or
            self._tf_buffer is None or
            self._params is None
        ):
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._params.trajectory_base_frame,
                self._params.trajectory_tip_frame,
                rclpy.time.Time())
        except TransformException:
            self._tcp_path.note_tf_fail()
            self._publish_tcp_summary()
            self._maybe_publish_tcp_viz()
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp
        kept = self._tcp_path.maybe_append({
            't': float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            'x': float(translation.x),
            'y': float(translation.y),
            'z': float(translation.z),
            'qx': float(rotation.x),
            'qy': float(rotation.y),
            'qz': float(rotation.z),
            'qw': float(rotation.w),
            'moving': self._traj_ctx['moving'],
            'target_id': self._traj_ctx['target_id'],
            'phase': self._traj_ctx['phase'],
            'skill': self._traj_ctx['skill'],
        })
        if kept is not None and self._recorder is not None:
            self._recorder.handle_tcp(kept)
        self._publish_tcp_summary()
        self._maybe_publish_tcp_viz(force=kept is not None)

    def _landmarks_now(self) -> dict:
        """作业票坐标优先，重建许可镜像补缺（入口/预抓取/轴）."""
        job = self._state.snapshot().get('job') or {}
        coords = job.get('coords') or {}
        grasp = job.get('grasp') or {}
        merged = dict(self._traj_landmarks)
        mapping = {
            'perception_entry': coords.get('perception_entry'),
            'perception_bottom': coords.get('perception_bottom'),
            'perception_neck': coords.get('perception_neck'),
            'reconstruction_center': coords.get('reconstruction_center'),
            'grasp_entry': coords.get('grasp_entry'),
            'grasp_pregrasp': coords.get('grasp_pregrasp'),
            'axis': grasp.get('axis') or coords.get('refined_axis'),
        }
        for key, value in mapping.items():
            if value:
                merged[key] = value
        merged['target_id'] = (
            job.get('target_id') or merged.get('target_id') or '')
        return merged

    def _maybe_publish_tcp_viz(self, force: bool = False) -> None:
        """Path / MarkerArray 约 5 Hz；点写入或超时则发（transient_local）."""
        now_ns = self.get_clock().now().nanoseconds
        if not force and now_ns - self._last_viz_ns < 200_000_000:
            return
        self._last_viz_ns = now_ns
        self._publish_tcp_viz()

    def _publish_tcp_viz(self) -> None:
        """与网页同源：downsample 后的 Path + MarkerArray."""
        if (
            self._tcp_path_pub is None or
            self._tcp_marker_pub is None or
            self._params is None
        ):
            return
        exported = (
            self._tcp_path.export() if self._tcp_path is not None else {})
        frame_id = self._params.trajectory_base_frame
        xyz, phases = downsample_path(
            exported.get('xyz') or [], exported.get('phase') or [], 800)
        stamp = self.get_clock().now()
        self._tcp_path_pub.publish(path_from_xyz(xyz, stamp, frame_id))
        markers = build_tcp_marker_dicts(
            xyz, phases, self._landmarks_now(), frame_id)
        markers.extend(
            self._selection_markers(frame_id))
        # 反闪烁（R3 稳定呈现）：仅当图元集合（ns+id）变化时才发 DELETEALL
        # 头，否则只重发 ADD 更新位姿——避免 20Hz 删-建在 RViz 里闪帧
        sig = tuple((m.get('ns', ''), m.get('id', 0)) for m in markers)
        if sig == self._last_marker_sig and markers and \
                markers[0].get('action') == 3:
            markers = markers[1:]
        self._last_marker_sig = sig
        self._tcp_marker_pub.publish(
            marker_array_from_dicts(markers, stamp, frame_id))

    def _selection_markers(self, frame_id: str) -> list:
        """选择叠加输入：最近 targets_filtered 事件 + 当前观测位置."""
        snapshot = self._state.snapshot()
        targets = snapshot.get('perception', {}).get('targets') or {}
        observations = targets.get('observations') or []
        selected_id = str(
            (snapshot.get('job') or {}).get('target_id')
            or targets.get('selected_target_id') or '')
        filtered: dict = {}
        for event in reversed(self._state.events_snapshot() or []):
            if event.get('code') != 'targets_filtered':
                continue
            try:
                payload = json.loads(event.get('message') or '{}')
                value = payload.get('filtered')
                if isinstance(value, dict):
                    filtered = {str(k): str(v) for k, v in value.items()}
            except (ValueError, TypeError):
                pass
            break
        return build_selection_marker_dicts(
            observations, selected_id, filtered, frame_id)

    def _metrics_callback(self, sample: dict) -> None:
        """性能采样落状态缓存并喂记录器（metrics.jsonl）."""
        self._state.update('metrics', 'sample', sample)
        self._recorder.handle_metrics(sample)

    # ------------------------------------------------------------------
    # 手动调试操作面（默认关；门控链：enabled → token → motion → 审计）
    # ------------------------------------------------------------------
    def _create_debug_bridge(self) -> None:
        """装配审计器；debug.enabled=true 才建转发桥（否则 POST 503 仍可审计）."""
        self._debug_audit = DebugAudit(
            str(resolve_runs_root(self._params.record_root_dir)),
            self._params.debug_audit_enabled,
            lambda msg: self.get_logger().warning(msg))
        if not self._params.debug_enabled:
            self.get_logger().info('手动调试操作面未启用（debug.enabled=false）')
            return
        self._debug_bridge = DebugBridge(
            self, self._params.debug_endpoints,
            self._params.debug_action_timeout_s,
            self._params.debug_motion_enabled,
            lambda msg: self.get_logger().warning(msg))
        self._debug_bridge.open_all()
        token_state = '已设置' if self._params.debug_token else '【空=全部拒绝】'
        motion_state = '放行' if self._params.debug_motion_enabled else '默认拒绝'
        self.get_logger().warning(
            f'*** 手动调试操作面已启用：token={token_state}；'
            f'运动类操作={motion_state}。技能侧既有安全门照常生效，'
            '但请勿将端口暴露到不受信网络 ***')

    def debug_command(self, action: str, payload: dict, headers) -> tuple:
        """
        调试操作门控链（HttpBackend 窄接口）：鉴权 → 运动 → 桥 → 审计.

        Args:
            action: 调试端点键.
            payload: 已解析 JSON 请求体.
            headers: HTTP 请求头（取 X-Debug-Token）.

        Returns
        -------
            (http_status, 响应 dict)；每次调用（含被拒）均写审计.

        """
        token = ''
        try:
            token = str(headers.get('X-Debug-Token') or '')
        except AttributeError:
            pass
        expected = self._params.debug_token if self._params else ''
        row = {'action': action, 'args': payload}

        def audit(accepted: bool, status: int, message: str) -> tuple:
            row.update({'accepted': accepted, 'status': status,
                        'message': message})
            if self._debug_audit is not None:
                self._debug_audit.record(row)
            return status, {'accepted': accepted, 'message': message}

        if self._debug_bridge is None or self._params is None:
            return audit(False, 503, '调试操作面未启用（debug.enabled=false）')
        if not expected or not hmac.compare_digest(
                token.encode('utf-8'), expected.encode('utf-8')):
            return audit(False, 401, '令牌缺失或不匹配')
        if is_motion(action, payload) and not self._params.debug_motion_enabled:
            return audit(
                False, 423,
                '运动类操作被拒绝：debug.motion_enabled=false（会动臂/机位的'
                '操作须显式开启该开关；技能侧安全门仍会独立复核）')
        try:
            status, body = self._debug_bridge.command(action, payload)
        except (TypeError, ValueError, RuntimeError, OSError, KeyError,
                AttributeError) as exc:
            return audit(False, 500, f'调试桥异常: {exc}')
        accepted = bool(body.get('accepted'))
        row.update({'accepted': accepted, 'status': status,
                    'result': body.get('result')})
        if self._debug_audit is not None:
            self._debug_audit.record(row)
        return status, body

    # ------------------------------------------------------------------
    # HttpBackend 窄接口实现（http_server 只依赖这三个方法）
    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        """浏览器状态快照（GET /api/state 的载荷，含 debug 段）."""
        snapshot = self._state.snapshot()
        snapshot['debug'] = {
            'enabled': bool(
                self._params.debug_enabled if self._params else False),
            'motion_enabled': bool(
                self._params.debug_motion_enabled if self._params else False),
            'token_required': bool(
                self._params.debug_token if self._params else False),
            'recent': (
                self._debug_bridge.state().get('recent', [])
                if self._debug_bridge is not None else []),
        }
        return snapshot

    def trajectory(self) -> dict:
        """末端轨迹 + 当前作业票路标（GET /api/trajectory）."""
        payload = {
            'frame_id': (
                self._params.trajectory_base_frame if self._params else
                'base_link'),
            'tip_frame': (
                self._params.trajectory_tip_frame if self._params else 'tcp'),
            'enabled': bool(
                self._params.trajectory_enabled if self._params else False),
            'tf_ok': False,
            'tf_failures': 0,
            't': [],
            'xyz': [],
            'moving': [],
            'phase': [],
            'metrics': {},
            'landmarks': {},
        }
        if self._tcp_path is not None:
            payload.update(self._tcp_path.export())
            payload['frame_id'] = self._params.trajectory_base_frame
            payload['tip_frame'] = self._params.trajectory_tip_frame
            payload['enabled'] = True
        landmarks = self._landmarks_now()
        payload['landmarks'] = landmarks
        xyz, phases = downsample_path(
            payload.get('xyz') or [], payload.get('phase') or [], 800)
        payload['markers'] = build_tcp_marker_dicts(
            xyz, phases, landmarks, payload['frame_id'])
        payload['topics'] = {
            'path': (
                self._topic('tcp_path_topic') if self._params else
                '/peach/observability/tcp_path'),
            'markers': (
                self._topic('tcp_markers_topic') if self._params else
                '/peach/observability/markers'),
        }
        return payload

    def _record_job(self) -> None:
        """作业票指纹变化时写入 job.jsonl."""
        if self._recorder is None:
            return
        self._recorder.handle_job(self._state.snapshot().get('job') or {})

    def ensure_active(self) -> None:
        """
        进入 Active 并开 HTTP.

        整栈 include 时 launch 的 lifecycle EmitEvent 经常匹配不到本节点
        （不进 lifecycle 名单），这里在 spin 前自行转换。
        """
        label = self._state_machine.current_state[1]
        if label == 'unconfigured':
            self.trigger_configure()
            label = self._state_machine.current_state[1]
        if label == 'inactive':
            self.trigger_activate()

    def start_http(self) -> None:
        """按快照参数提示非回环风险并启动 HTTP 服务线程."""
        if self._http is not None:
            return
        host = self._params.host
        port = self._params.port
        if host not in ('127.0.0.1', 'localhost'):
            self.get_logger().warning(
                f'*** 安全提示：Web 监控台监听在非回环地址 {host}:{port}，'
                '调试操作面虽有令牌门控，仍严禁暴露到公网或不受信网络 ***')
        web_root = Path(get_package_share_directory(
            'peach_executor')) / 'web'
        self._http = http_server.start_http(
            host, port, web_root, self, self.get_logger().debug)
        if self._metrics is not None:
            self._metrics.start()
        shown_host = '127.0.0.1' if host == '0.0.0.0' else host
        mode = '监控+手动调试（鉴权）' if self._params.debug_enabled \
            else '只读监控'
        self.get_logger().info(f'桃子采摘 Web 控制台（{mode}）: '
                               f'http://{shown_host}:{port}')

    def _stop_runtime(self) -> None:
        """停 HTTP 与性能采样；订阅仍保留到 cleanup."""
        if self._metrics is not None:
            self._metrics.stop()
        if self._http is not None:
            self._http.shutdown()
            self._http.server_close()
            self._http = None

    def _release_resources(self) -> None:
        """释放 configure 期资源，允许再次 configure."""
        if self._traj_timer is not None:
            self.destroy_timer(self._traj_timer)
            self._traj_timer = None
        if self._tcp_path_pub is not None:
            self.destroy_publisher(self._tcp_path_pub)
            self._tcp_path_pub = None
        if self._tcp_marker_pub is not None:
            self.destroy_publisher(self._tcp_marker_pub)
            self._tcp_marker_pub = None
        self._tf_listener = None
        self._tf_buffer = None
        self._tcp_path = None
        if self._param_timer is not None:
            self.destroy_timer(self._param_timer)
            self._param_timer = None
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs.clear()
        for client in list(self._param_clients.values()):
            self.destroy_client(client)
        self._param_clients.clear()
        self._param_inflight.clear()
        if self._debug_bridge is not None:
            self._debug_bridge.close()
            self._debug_bridge = None
        self._debug_audit = None
        if self._recorder is not None:
            self._recorder.close()
            self._recorder = None
        self._metrics = None
        self._params = None

    def destroy_node(self):
        """停止 HTTP/性能采样/记录器后销毁 ROS 节点."""
        self._stop_runtime()
        if self._recorder is not None:
            self._recorder.close()
            self._recorder = None
        super().destroy_node()


def main(args=None):
    """运行 ROS 与只读 HTTP 监控；HTTP 在 Lifecycle Active 后才监听."""
    rclpy.init(args=args)
    node = ObservabilityNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    node.ensure_active()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
