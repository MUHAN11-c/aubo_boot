"""
桃子采摘链路监控 Web：过程记录 / 监控 / 轨迹 + 单步调试（同端口 8090）.

只读面回答「现在跑到哪一步、坐标是什么、TCP 怎么走的」；过程记录为会话
级 MCAP bag（决策 0019：随节点启停开合，栈停自动出 bag_report 并按预算
回收旧 bag）。节点只接线；阶段时间线 / 账本直播 / 地标合并在 pipeline.py。调试 POST /api/debug/<action> 转发既有动作/服务（决策 0018：
无令牌）。debug.enabled 默认开（回环）；动臂另需 debug.motion_enabled
（默认关→423）。技能侧 ExecutionAuthority 等既有安全门不受影响：Web 只是
又一个客户端。
"""

from __future__ import annotations

import json
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from aubo_msgs.msg import JointStatus, RobotStatus
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Path as NavPath
from peach_common.lifecycle import ensure_lifecycle_active
from peach_common.paths import runs_root as resolve_runs_root
from peach_interfaces.msg import (
    BagFittingArray,
    BagGraspCandidateArray,
    CanonicalEvent,
    GraspDecision,
    GraspHypothesis,
    HarvestState,
    PeachTargetObservationArray,
    ReconstructionStatus,
    SceneSnapshot,
)
from rcl_interfaces.msg import Log as RosoutLog
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import (
    DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import MarkerArray

from . import bag_report
from . import http_server
from . import pipeline
from . import retention
from .catch_all_recorder import CatchAllRecorder
from .debug_actions import DebugAudit, DebugBridge, is_motion
from .params import ObservabilityParams
from .recorder import Recorder
from .state import (
    merge_joint_hardware,
    MetricsSampler,
    ObservabilityState,
    parse_json_text,
    to_candidate_array,
    to_fitting_array,
    to_grasp_decision,
    to_grasp_hypothesis,
    to_harvest_event,
    to_joint_state,
    to_joint_status,
    to_reconstruction_status,
    to_robot_status,
    to_target_observations,
    to_task_executor_state,
    to_vector_stamped,
)
from .tcp_trajectory import (
    build_selection_marker_dicts,
    build_tcp_marker_dicts,
    downsample_path,
    marker_array_from_dicts,
    path_from_xyz,
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
    '/peach_arm': [
        'moveit.velocity_scaling', 'moveit.acceleration_scaling',
        'moveit.transit_velocity_scaling', 'moveit.transit_acceleration_scaling',
        'scan.observation_radius_m', 'scan.minimum_radius_m',
        'scan.frame_wait_s', 'scan.maximum_moves',
        'quality.minimum_views', 'quality.minimum_baseline_deg',
        'execution.enabled', 'grasp.enabled', 'tool.enabled',
        'photo_pose_joint_tolerance_rad', 'photo_pose_max_joint_vel_rad_s',
    ],
    '/peach_supervisor': [
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


# 核心订阅表（镜像 + 落盘）：(消息类型, 话题参数名, 回调属性名, QoS 档名)。
# W10 表驱动化：新增订阅=表加一行，不再往 _create_subscriptions 堆平铺。
_CORE_SUBSCRIPTIONS = (
    (PeachTargetObservationArray, 'target_observations_topic',
     '_targets_callback', 'reliable'),
    (String, 'harvest_state_topic', '_harvest_callback', 'latched'),
    (String, 'reconstruction_status_topic',
     '_recon_status_callback', 'latched'),
    (ReconstructionStatus, 'reconstruction_diagnostics_topic',
     '_recon_diagnostics_callback', 'latched'),
    # 调试明细（tsdf/registration/overlap/refined/逐机位）：并入镜像与落盘，
    # 保持「类型化后过程数据不缩水」；类型化字段为准，明细键补充
    (String, 'reconstruction_diagnostics_debug_topic',
     '_recon_debug_callback', 'latched'),
    (GraspDecision, 'grasp_decision_topic', '_recon_decision_callback',
     'latched'),
    (BagGraspCandidateArray, 'refined_pose_topic', '_refined_pose_callback',
     'latched'),
    (Vector3Stamped, 'refined_axis_topic', '_refined_axis_callback',
     'latched'),
    (BagFittingArray, 'refined_diagnostics_topic',
     '_refined_diagnostics_callback', 'latched'),
    (String, 'manipulation_status_topic', '_manipulation_callback',
     'latched'),
    (GraspHypothesis, 'grasp_hypothesis_topic', '_grasp_hypothesis_callback',
     'latched'),
    (HarvestState, 'task_executor_state_topic', '_task_executor_callback',
     'latched'),
    (CanonicalEvent, 'task_executor_events_topic', '_events_callback',
     'events'),
    (RobotStatus, 'robot_status_topic', '_robot_status_callback',
     'reliable'),
    (JointState, 'joint_states_topic', '_joint_state_callback', 'sensor'),
    (JointStatus, 'joint_status_topic', '_joint_status_callback',
     'reliable'),
)

# bag 专用 raw 订阅（只落盘不镜像）：(消息类型, 话题参数名, QoS 档名,
# 门控参数属性名)。'/rosout' 无参数名，用 None 标记直连固定话题。
_RAW_SUBSCRIPTIONS = (
    (RosoutLog, None, 'reliable', 'record_rosout'),
    (Image, 'debug_image_topic', 'reliable', 'record_save_images'),
    # 真相流画布（raw 前缀）：与稳定流成对进 bag，筛选前后对比不依赖 RViz
    (Image, 'debug_image_raw_topic', 'reliable', 'record_save_images'),
    (PointCloud2, 'tsdf_cloud_topic', 'latched', 'record_save_clouds'),
    # SceneSnapshot 单发闩锁（M13）：落盘订阅须 transient_local，VOLATILE
    # 会在记录节点晚于发布启动时永久丢单发快照
    (SceneSnapshot, 'scene_snapshot_topic', 'latched', None),
    (TFMessage, 'tf_topic', 'tf', None),
    (TFMessage, 'tf_static_topic', 'tf_static', None),
)


class ObservabilityNode(LifecycleNode):
    """订阅采摘链路各阶段输出，提供过程监控 API 与单步调试 POST."""

    def __init__(self):
        """声明参数；订阅与 HTTP 等到 configure / activate."""
        super().__init__('peach_observability')
        self._params = None
        self._state = ObservabilityState()
        self._http = None
        self._metrics = None
        self._recorder = None
        self._catch_all = None
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
        self._joint_state = {}
        self._joint_status = {}
        self._joints_push_t = 0.0
        # TCP 可视化缓存（RViz 5Hz 与 HTTP /api/trajectory 共享一次构建）
        self._viz_bundle = ([], [], [])
        self._viz_bundle_sig = None
        # configure 期体积回收线程（rglob 全部 bag 目录，大库下秒级）
        self._sweep_thread: threading.Thread | None = None
        # 派生话题（job/metrics 进 bag）与作业票指纹去重
        self._job_pub = None
        self._metrics_pub = None
        self._last_job_key = None
        # 栈停自动报告线程（bag 收尾后生成 bag_report.md/json + 体积回收）
        self._report_thread: threading.Thread | None = None
        # 调试 POST（debug.enabled=true 才建桥）
        self._debug_bridge: DebugBridge | None = None
        self._debug_audit: DebugAudit | None = None
        # 全流程时序跟踪器（pipeline.py 纯核）：调度 FSM / 技能周期
        self._fsm_tracker = pipeline.StageTracker(limit=150)
        self._arm_tracker = pipeline.StageTracker(limit=150)
        # 批次账本直播（runs/<request_id>/ledger.json；run_id==request_id）
        self._ledger_watch: pipeline.LedgerWatch | None = None
        self._ledger_request_id = ''
        # /diagnostics 双轨（W15：对齐 peach_arm W5 / vegetation 的做法）
        self._diag: Updater | None = None

    def on_configure(self, state):
        del state
        try:
            self._params = ObservabilityParams.attach(self)
        except Exception as exc:  # noqa: BLE001 参数库校验异常类型跨 rclpy 版本
            self.get_logger().error(f'参数非法: {exc}')
            return TransitionCallbackReturn.FAILURE
        runs_root = resolve_runs_root(self._params.record_root_dir)
        self._ledger_watch = pipeline.LedgerWatch(runs_root)
        # 会话 bag 开启前先跑一次体积回收（超预算清最旧 bag，写审计）。
        # rglob 全部 bag 目录在 20GB 库下秒级阻塞 configure，挪后台线程；
        # 与停栈报告线程的 sweep 极小概率并发删同一目录，失败侧跳过并告警
        self._sweep_thread = threading.Thread(
            target=retention.sweep,
            args=(runs_root, self._params.record_max_total_bag_gb),
            kwargs={'log_warning': lambda msg: self.get_logger().warning(msg)},
            name='peach-retention', daemon=True)
        self._sweep_thread.start()
        self._recorder = Recorder(
            root_dir=str(runs_root),
            enabled=self._params.record_enabled,
            queue_depth=self._params.record_queue_depth,
            on_info=lambda info: self._state.update('record', 'info', info),
            log_warning=lambda msg: self.get_logger().warning(msg))
        self._job_pub = self.create_publisher(
            String, self._topic('job_topic'), 10)
        self._metrics_pub = self.create_publisher(
            String, self._topic('metrics_topic'), 10)
        self._create_subscriptions()
        self._create_param_watchers()
        self._create_metrics_sampler()
        self._create_tcp_sampler()
        self._create_debug_bridge()
        self._create_diagnostics()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        result = super().on_activate(state)
        self.start_http()
        # 全量录制（record.level 门控）：activate 后计算图已可见，通配
        # 发现订阅域内全部话题自动进会话 bag；仿真复现/问题分析用
        if (self._recorder is not None and self._recorder.enabled
                and self._params.record_level in ('all', 'std')):
            self._catch_all = CatchAllRecorder(
                self, self._recorder, level=self._params.record_level)
            self._catch_all.start()
        return result

    def on_deactivate(self, state):
        if self._catch_all is not None:
            self._catch_all.stop()
            self._catch_all = None
        self._stop_runtime()
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self._stop_runtime()
        self._release_resources()
        return super().on_cleanup(state)

    def on_shutdown(self, state):
        """Shutdown 迁移：收尾 bag 并起自动报告（名单管理路径走 destroy_node）."""
        del state
        self._spawn_report(self._close_recorder())
        return TransitionCallbackReturn.SUCCESS

    def _record_raw(self, topic_parameter: str, message) -> None:
        """把原始消息按话题参数名解析后入会话 bag（未启用时记录器自丢）."""
        if self._recorder is None:
            return
        self._recorder.handle_raw(self._topic(topic_parameter), message)

    def _raw_cb(self, topic_parameter: str):
        """订阅回调工厂：原样把消息送进会话 bag（bag 专用订阅用）."""
        def callback(message) -> None:
            self._record_raw(topic_parameter, message)
        return callback

    def _rosout_callback(self, message) -> None:
        """全量节点日志进会话 bag（stamp/level/logger/name/msg 可离线回放）."""
        if self._recorder is not None:
            self._recorder.handle_raw('/rosout', message)

    def _topic(self, parameter: str) -> str:
        """从不可变快照取话题名（启动期建订阅用）."""
        return self._params.topics[parameter]

    def _subscribe(self, *args, **kwargs):
        """建订阅并记下句柄，供 on_cleanup 释放."""
        sub = self.create_subscription(*args, **kwargs)
        self._subs.append(sub)
        return sub

    def _create_subscriptions(self) -> None:
        """按订阅表建立全部只读订阅（核心镜像表 + bag raw 表）."""
        profiles = {
            'latched': QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE),
            'reliable': QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE),
            'sensor': qos_profile_sensor_data,
            'events': QoSProfile(
                depth=50,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE),
            'tf': QoSProfile(
                depth=200, reliability=ReliabilityPolicy.RELIABLE),
            'tf_static': QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE),
        }
        for msg_type, param, callback_name, profile in _CORE_SUBSCRIPTIONS:
            self._subscribe(
                msg_type, self._topic(param),
                getattr(self, callback_name), profiles[profile])
        # 记录器图像/点云订阅：只在对应开关开启时建立（省带宽），直接进 bag
        if not self._params.record_enabled:
            return
        for msg_type, param, profile, gate in _RAW_SUBSCRIPTIONS:
            if gate is not None and not getattr(self._params, gate):
                continue
            if param is not None:
                self._subscribe(
                    msg_type, self._topic(param), self._raw_cb(param),
                    profiles[profile])
            else:
                # /rosout 全量进 bag：全部节点日志（stamp/level/logger）可
                # 回放，测试复盘最低完备集（record.rosout 门控，默认开）
                self._subscribe(
                    msg_type, '/rosout', self._rosout_callback,
                    profiles[profile])

    def _robot_status_callback(self, message: RobotStatus) -> None:
        """机械臂柜侧状态；in_motion 给 TCP 采样当运动标记."""
        self._traj_ctx['moving'] = bool(message.in_motion)
        self._state.update('robot', 'status', to_robot_status(message))
        self._record_raw('robot_status_topic', message)

    def _joint_state_callback(self, message: JointState) -> None:
        """实际关节角/速度；与 joint_status 合成硬件表（约 10 Hz 推镜像）."""
        self._joint_state = to_joint_state(message)
        self._record_raw('joint_states_topic', message)
        self._push_joints()

    def _joint_status_callback(self, message: JointStatus) -> None:
        """柜侧关节电流/温度/跟随误差；与 /joint_states 合成硬件表."""
        self._joint_status = to_joint_status(message)
        self._record_raw('joint_status_topic', message)
        self._push_joints()

    def _push_joints(self) -> None:
        """把最新关节实际量与柜侧明细合成 robot.joints（限 10 Hz）."""
        now = time.monotonic()
        if now - self._joints_push_t < 0.1:
            return
        self._joints_push_t = now
        self._state.update(
            'robot', 'joints',
            merge_joint_hardware(self._joint_state, self._joint_status))

    def _harvest_callback(self, message: String) -> None:
        """
        感知采摘计划：进状态缓存并进会话 bag.

        JSON 全量透传：发布侧新增键（如阶段 D1 的 anchor_stale_target_ids/
        out_of_view_target_ids/dropped_target_ids/lighting/low_light_quality）
        无需本侧改动即进入 /api/state 镜像。
        """
        value = parse_json_text(message.data)
        self._state.update('perception', 'harvest', value)
        self._record_raw('harvest_state_topic', message)
        self._publish_job()

    def _recon_status_callback(self, message: String) -> None:
        """重建状态文本：进状态缓存并进会话 bag."""
        value = parse_json_text(message.data, 'state')
        self._state.update('reconstruction', 'status', value)
        self._record_raw('reconstruction_status_topic', message)

    def _recon_diagnostics_callback(
            self, message: ReconstructionStatus) -> None:
        """
        重建 1Hz 结构化诊断：消息字段重建镜像 dict 并进会话 bag.

        bag 录原始话题；合并体（调试明细 ∪ 类型化字段 ∪ 最近许可镜像）
        由 bag_report 离线重放，镜像侧保持合并以便前端消费。
        """
        merged = dict(self._recon_debug_extra)
        merged.update(to_reconstruction_status(message))
        if self._recon_decision_value is not None:
            merged['grasp_decision'] = self._recon_decision_value
        center = merged.get('target_center_base')
        if isinstance(center, list):
            self._traj_landmarks['reconstruction_center'] = center
        self._state.update('reconstruction', 'diagnostics', merged)
        self._record_raw('reconstruction_diagnostics_topic', message)

    def _recon_debug_callback(self, message: String) -> None:
        """重建调试明细 JSON：更新镜像缓存（bag 另录原始话题，防缺料）."""
        self._recon_debug_extra = parse_json_text(message.data)
        self._record_raw('reconstruction_diagnostics_debug_topic', message)

    def _recon_decision_callback(self, message: GraspDecision) -> None:
        """重建抓取许可：消息字段重建镜像 dict 并进会话 bag."""
        value = to_grasp_decision(message)
        self._recon_decision_value = value
        self._traj_landmarks['grasp_entry'] = value.get('entry')
        self._traj_landmarks['grasp_pregrasp'] = value.get('pregrasp')
        self._traj_landmarks['axis'] = value.get('axis')
        self._traj_landmarks['target_id'] = value.get('target_id') or ''
        self._state.update('reconstruction', 'grasp_decision', value)
        self._record_raw('grasp_decision_topic', message)
        self._publish_job()

    def _refined_pose_callback(self, message) -> None:
        """精化位姿：镜像进缓存，原始消息进会话 bag."""
        self._state.update('refined', 'pose', to_candidate_array(message))
        self._record_raw('refined_pose_topic', message)

    def _refined_axis_callback(self, message) -> None:
        """精化轴线：镜像进缓存，原始消息进会话 bag."""
        self._state.update('refined', 'axis', to_vector_stamped(message))
        self._record_raw('refined_axis_topic', message)

    def _refined_diagnostics_callback(self, message) -> None:
        """精化质量：镜像进缓存，原始消息进会话 bag."""
        self._state.update(
            'refined', 'diagnostics', to_fitting_array(message))
        self._record_raw('refined_diagnostics_topic', message)

    def _manipulation_callback(self, message: String) -> None:
        """技能节点状态：进状态缓存、喂周期时序跟踪器并进会话 bag."""
        value = parse_json_text(message.data)
        self._traj_ctx['skill'] = str(value.get('state') or '')
        self._state.update('manipulation', 'status', value)
        # 技能周期阶段时序：CycleState/消息/目标任一变化记一条转移，
        # 段时长=到下一转移的间隔（服务器侧权威，页面刷新不丢）
        now = time.time()
        entry = {
            'state': str(value.get('state') or ''),
            'message': str(value.get('message') or ''),
            'target_id': str(value.get('target_id') or ''),
            'running': bool(value.get('running')),
            'recovery': bool(value.get('contact_recovery_required')),
        }
        key = tuple(entry.items())
        if self._arm_tracker.feed(key, entry, now):
            self._state.update(
                'pipeline', 'arm', self._arm_tracker.export(now))
        self._record_raw('manipulation_status_topic', message)
        self._publish_job()

    def _grasp_hypothesis_callback(self, message: GraspHypothesis) -> None:
        """技能抓取假设：进状态缓存并进会话 bag."""
        value = to_grasp_hypothesis(message)
        self._state.update('manipulation', 'hypothesis', value)
        self._record_raw('grasp_hypothesis_topic', message)
        self._publish_job()

    def _events_callback(self, message: CanonicalEvent) -> None:
        """批次事件进环形缓冲与会话 bag，供前端时间线与离线报告消费."""
        self._record_raw('task_executor_events_topic', message)
        try:
            value = to_harvest_event(message)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning(f'事件转换失败: {error}')
            return
        # 缓冲上限来自启动期快照（A9）：已校验 >= 1，运行期不再直读参数
        self._state.append_event(value, self._params.event_buffer_size)

    def _task_executor_callback(self, message: HarvestState) -> None:
        """调度类型化状态：镜像 + 轨迹上下文 + FSM 时序 + 落盘 + 作业票."""
        value = to_task_executor_state(message)
        self._state.update('task_executor', 'state', value)
        self._track_run_context(value)
        self._track_fsm(message)
        self._record_raw('task_executor_state_topic', message)
        self._publish_job()

    def _track_run_context(self, value: dict) -> None:
        """轨迹上下文与账本 run_id 跟随（换 run 清空上一轮轨迹）."""
        self._traj_ctx['target_id'] = str(value['target_id'] or '')
        self._traj_ctx['phase'] = int(value['target_phase'] or 0)
        run_id = str(value['run_id'] or '')
        if run_id and run_id != self._traj_run_id:
            self._traj_run_id = run_id
            if self._tcp_path is not None:
                self._tcp_path.clear()
        # run_id 即账本目录名（supervisor：_run_id = goal.request_id）
        if run_id:
            self._ledger_request_id = run_id

    def _track_fsm(self, message: HarvestState) -> None:
        """调度 FSM 时序：状态/相位/消息/使能任一变化记一条转移."""
        entry = {
            'batch_state': int(message.batch_state or 0),
            'target_phase': int(message.target_phase or 0),
            'message': str(message.message or ''),
            'run_id': str(message.run_id or ''),
            'cycle_id': str(message.cycle_id or ''),
            'target_id': str(message.target_id or ''),
            'recovery_required': bool(message.recovery_required),
            'execution_enabled': bool(message.execution_enabled),
            'grasp_enabled': bool(message.grasp_enabled),
            'tool_enabled': bool(message.tool_enabled),
            'state_seq': int(getattr(message, 'state_seq', 0) or 0),
        }
        key = (entry['run_id'], entry['cycle_id'], entry['target_id'],
               entry['batch_state'], entry['target_phase'], entry['message'],
               entry['recovery_required'], entry['execution_enabled'],
               entry['grasp_enabled'], entry['tool_enabled'])
        now = time.time()
        if self._fsm_tracker.feed(key, entry, now):
            self._state.update(
                'pipeline', 'fsm', self._fsm_tracker.export(now))

    def _targets_callback(self, message) -> None:
        try:
            value = to_target_observations(message)
        except (AttributeError, TypeError, ValueError) as error:
            self.get_logger().warning(f'目标快照转换失败: {error}')
            return
        self._state.update('perception', 'targets', value)
        self._record_raw('target_observations_topic', message)

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
        """对就绪的参数服务发起异步查询（在途请求去重）+ 刷批次账本."""
        for name, client in self._param_clients.items():
            if name in self._param_inflight or not client.service_is_ready():
                continue
            request = GetParameters.Request()
            request.names = list(PARAM_WATCHLIST[name])
            self._param_inflight.add(name)
            future = client.call_async(request)
            future.add_done_callback(
                lambda fut, node_name=name: self._on_params(node_name, fut))
        self._refresh_ledger()

    def _refresh_ledger(self) -> None:
        """账本直播：mtime 变化才刷镜像（run_id 即 request_id）."""
        if self._ledger_watch is None:
            return
        payload = self._ledger_watch.refresh(self._ledger_request_id)
        if payload is not None:
            self._state.update('ledger', 'live', payload)

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
        # 轨迹经 /tf 全量进会话 bag，报告侧离线重算；此处只驱动 Web/RViz
        self._publish_tcp_summary()
        self._maybe_publish_tcp_viz(force=kept is not None)

    def _landmarks_now(self) -> dict:
        """作业票坐标优先，重建许可镜像补缺（入口/预抓取/轴）."""
        return pipeline.merge_job_landmarks(
            self._traj_landmarks, self._state.job())

    def _maybe_publish_tcp_viz(self, force: bool = False) -> None:
        """Path / MarkerArray 约 5 Hz；点写入或超时则发（transient_local）."""
        now_ns = self.get_clock().now().nanoseconds
        if not force and now_ns - self._last_viz_ns < 200_000_000:
            return
        self._last_viz_ns = now_ns
        self._publish_tcp_viz()

    def _tcp_viz_bundle(self, exported: dict, frame_id: str) -> tuple:
        """
        (downsample xyz, phases, TCP marker 字典)；带缓存共享给 RViz 与 HTTP.

        轨迹末点/点数与路标都未变时直接命中缓存——/api/trajectory 每个网页
        0.4s 轮询、RViz 5Hz 发布，两次构建同Marker 集是纯重复功。
        """
        xyz_flat = exported.get('xyz') or []
        landmarks = self._landmarks_now()
        signature = (
            len(xyz_flat),
            tuple(xyz_flat[-3:]) if xyz_flat else (),
            json.dumps(landmarks, ensure_ascii=False, sort_keys=True),
        )
        if signature != self._viz_bundle_sig:
            xyz, phases = downsample_path(
                xyz_flat, exported.get('phase') or [], 800)
            self._viz_bundle = (
                xyz, phases,
                build_tcp_marker_dicts(xyz, phases, landmarks, frame_id))
            self._viz_bundle_sig = signature
        return self._viz_bundle

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
        xyz, phases, markers = self._tcp_viz_bundle(exported, frame_id)
        stamp = self.get_clock().now()
        self._tcp_path_pub.publish(path_from_xyz(xyz, stamp, frame_id))
        markers = list(markers)
        markers.extend(self._selection_markers(frame_id))
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

    def _create_diagnostics(self) -> None:
        """建 /diagnostics 周期任务：录制队列健康 + 订阅摄入活度."""
        self._diag = Updater(self, period=5.0)
        self._diag.setHardwareID('peach_observability')
        self._diag.add('session_recorder', self._diag_recorder)
        self._diag.add('ingest_liveness', self._diag_ingest)

    def _diag_recorder(self, stat) -> object:
        """会话录制健康：丢帧>0 报 WARN（盘速掉队），否则 OK."""
        info = self._recorder.info() if self._recorder is not None else {}
        if not info.get('enabled'):
            stat.summary(DiagnosticStatus.OK, 'record disabled by config')
        elif info.get('drops', 0) > 0:
            stat.summary(
                DiagnosticStatus.WARN,
                f"bag queue drops={info['drops']} (disk behind, oldest dropped)")
        else:
            stat.summary(DiagnosticStatus.OK, 'recording')
        stat.add('queue_size', str(info.get('queue_size')))
        stat.add('queue_depth', str(info.get('queue_depth')))
        stat.add('drops', str(info.get('drops', 0)))
        stat.add('session', str(info.get('session')))
        return stat

    def _diag_ingest(self, stat) -> object:
        """订阅摄入活度：最热键年龄 ≤10s OK / ≤60s WARN / 更久 STALE."""
        ages = self._state.topic_ages()
        if not ages:
            stat.summary(DiagnosticStatus.WARN, 'no ingest yet')
            stat.add('watched_keys', '0')
            return stat
        newest = min(ages.values())
        oldest = sorted(ages.items(), key=lambda item: -item[1])[:3]
        if newest <= 10.0:
            stat.summary(DiagnosticStatus.OK, f'ingesting (newest {newest}s)')
        elif newest <= 60.0:
            stat.summary(DiagnosticStatus.WARN, f'quiet (newest {newest}s)')
        else:
            stat.summary(DiagnosticStatus.STALE, f'silent {newest}s')
        stat.add('watched_keys', str(len(ages)))
        stat.add('newest_age_s', str(newest))
        stat.add(
            'oldest_keys',
            ', '.join(f'{key}:{age}s' for key, age in oldest))
        return stat

    def _metrics_callback(self, sample: dict) -> None:
        """性能采样：镜像进状态缓存，并发布 JSON 进会话 bag."""
        self._state.update('metrics', 'sample', sample)
        if self._metrics_pub is not None:
            message = String(data=json.dumps(sample, ensure_ascii=False))
            self._metrics_pub.publish(message)
            self._record_raw('metrics_topic', message)

    # ------------------------------------------------------------------
    # 单步调试（enabled → 运动门 → 审计；无令牌）
    # ------------------------------------------------------------------
    def _create_debug_bridge(self) -> None:
        """装配审计器；debug.enabled=true 才建转发桥（否则 POST 503 仍可审计）."""
        self._debug_audit = DebugAudit(
            str(resolve_runs_root(self._params.record_root_dir)),
            self._params.debug_audit_enabled,
            lambda msg: self.get_logger().warning(msg))
        if not self._params.debug_enabled:
            self.get_logger().info('调试操作面未启用（debug.enabled=false）')
            return
        self._debug_bridge = DebugBridge(
            self, self._params.debug_endpoints,
            self._params.debug_action_timeout_s,
            self._params.debug_motion_enabled,
            lambda msg: self.get_logger().warning(msg))
        self._debug_bridge.open_all()
        motion_state = '放行' if self._params.debug_motion_enabled else '默认拒绝'
        self.get_logger().warning(
            f'*** 调试操作面已启用：运动类操作={motion_state}。'
            '技能侧既有安全门照常生效；请保持 HTTP 回环绑定 ***')

    def debug_command(self, action: str, payload: dict, headers) -> tuple:
        """
        调试操作门控（HttpBackend 窄接口）：总开关 → 运动 → 桥 → 审计.

        Args:
            action: 调试端点键.
            payload: 已解析 JSON 请求体.
            headers: HTTP 请求头（兼容旧客户端；不再校验令牌）.

        Returns
        -------
            (http_status, 响应 dict)；每次调用（含被拒）均写审计.

        """
        del headers
        row = {'action': action, 'args': payload}

        def audit(accepted: bool, status: int, message: str) -> tuple:
            row.update({'accepted': accepted, 'status': status,
                        'message': message})
            if self._debug_audit is not None:
                self._debug_audit.record(row)
            return status, {'accepted': accepted, 'message': message}

        if self._debug_bridge is None or self._params is None:
            return audit(False, 503, '调试操作面未启用（debug.enabled=false）')
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
            bundle_source = {
                'xyz': payload.get('xyz') or [],
                'phase': payload.get('phase') or [],
            }
        else:
            bundle_source = {'xyz': [], 'phase': []}
        # 与 RViz 发布共享同一份 downsample+markers（轨迹/路标未变时命中缓存）
        _, _, markers = self._tcp_viz_bundle(bundle_source, payload['frame_id'])
        payload['markers'] = markers
        payload['landmarks'] = self._landmarks_now()
        payload['topics'] = {
            'path': (
                self._topic('tcp_path_topic') if self._params else
                '/peach/observability/tcp_path'),
            'markers': (
                self._topic('tcp_markers_topic') if self._params else
                '/peach/observability/markers'),
        }
        return payload

    def _publish_job(self) -> None:
        """作业票指纹变化时发布 JSON 话题并进会话 bag（bag_report 离线消费）."""
        if self._job_pub is None:
            return
        job = self._state.job()
        if not job:
            return
        key = json.dumps({
            'target_id': job.get('target_id'),
            'active_id': job.get('active_id'),
            'why': job.get('why'),
            'stages': [
                (item.get('id'), item.get('status'))
                for item in job.get('stages') or []],
            'allowed': (job.get('grasp') or {}).get('allowed'),
            'reason': (job.get('grasp') or {}).get('reason'),
            'skill': (job.get('motion') or {}).get('state'),
            'flags': job.get('flags'),
        }, ensure_ascii=False, sort_keys=True)
        if key == self._last_job_key:
            return
        self._last_job_key = key
        message = String(data=json.dumps(job, ensure_ascii=False))
        self._job_pub.publish(message)
        self._record_raw('job_topic', message)

    def ensure_active(self) -> None:
        """
        进入 Active 并开 HTTP.

        整栈 include 时 launch 的 lifecycle EmitEvent 经常匹配不到本节点
        （不进 lifecycle 名单），这里在 spin 前自行转换（共享 helper）。
        """
        ensure_lifecycle_active(self)

    def start_http(self) -> None:
        """按快照参数提示非回环风险并启动 HTTP 服务线程."""
        if self._http is not None:
            return
        host = self._params.host
        port = self._params.port
        if host not in ('127.0.0.1', 'localhost'):
            self.get_logger().warning(
                f'*** 安全提示：Web 监听在非回环地址 {host}:{port}，'
                '严禁暴露到公网或不受信网络 ***')
        web_root = Path(get_package_share_directory(
            'peach_observability')) / 'web'
        self._http = http_server.start_http(
            host, port, web_root, self, self.get_logger().debug)
        if self._metrics is not None:
            self._metrics.start()
        shown_host = '127.0.0.1' if host == '0.0.0.0' else host
        mode = '过程监控+单步调试' if self._params.debug_enabled \
            else '只读监控'
        self.get_logger().info(f'感知抓取过程页（{mode}）: '
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
        if self._sweep_thread is not None:
            self._sweep_thread.join(timeout=5.0)
            self._sweep_thread = None
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
        self._fsm_tracker.reset()
        self._arm_tracker.reset()
        self._ledger_watch = None
        self._ledger_request_id = ''
        self._diag = None
        self._spawn_report(self._close_recorder())
        self._metrics = None
        self._params = None

    # ------------------------------------------------------------------
    # 会话收尾：bag 关闭 → 后台线程自动出报告 + 体积回收（决策 0019）
    # ------------------------------------------------------------------
    def _close_recorder(self):
        """关 bag（排空写队列）；返回 bag 目录，未启用/重复调用给 None."""
        recorder, self._recorder = self._recorder, None
        if recorder is None:
            return None
        return recorder.close()

    def _spawn_report(self, bag_dir) -> None:
        """收尾后起报告线程（自动出 bag_report 并按预算回收）；无 bag 跳过."""
        if bag_dir is None or (
                self._report_thread is not None
                and self._report_thread.is_alive()):
            return
        params = self._params
        runs_root = str(
            resolve_runs_root(params.record_root_dir)) if params else str(
            Path(bag_dir).parent.parent)

        def finalize() -> None:
            try:
                bag_report.generate_session_report(
                    bag_dir,
                    base_frame=(
                        params.trajectory_base_frame if params
                        else 'base_link'),
                    tip_frame=(
                        params.trajectory_tip_frame if params else 'tcp'),
                    ledger_root=runs_root,
                    max_total_bag_gb=(
                        params.record_max_total_bag_gb if params else 0.0),
                    keep=[Path(bag_dir)],
                    log=lambda msg: self.get_logger().info(str(msg)))
            except Exception as error:  # noqa: BLE001 报告失败不阻塞退出
                self.get_logger().warning(f'bag 自动报告失败: {error}')

        self._report_thread = threading.Thread(
            target=finalize, name='peach-bag-report', daemon=False)
        self._report_thread.start()

    def join_report(self, timeout: float = 55.0) -> None:
        """进程退出前等报告线程收尾（防悬挂；上限对齐 launch 停栈窗 60s）."""
        if self._report_thread is not None:
            self._report_thread.join(timeout=timeout)

    def destroy_node(self):
        """停止 HTTP/性能采样，收尾 bag 并起自动报告后销毁 ROS 节点."""
        self._stop_runtime()
        self._spawn_report(self._close_recorder())
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
        node.join_report()
        if rclpy.ok():
            rclpy.shutdown()
