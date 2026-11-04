"""
ObservabilityParams：GPL 参数的扁平快照.

``config/observability_parameters.yaml`` 声明参数，``config/observability.yaml``
提供部署值。``ObservabilityParams.attach(node)`` 绑定 ParamListener 和整数
边界校验，成功的参数修改原地刷新快照。话题名集中进 topics；调试端点集中进
debug_endpoints。
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Mapping, Tuple

from peach_observability.observability_parameters import (
    peach_observability as _Generated,
)
from rcl_interfaces.msg import SetParametersResult

from .startup_facts import normalize_facts, parse_facts_json, selfcheck_overrides_from_facts

_INT_BOUNDS = {
    'port': (1, 65535),
    'event_buffer_size': (1, None),
    'log_buffer_size': (1, None),
    'record.queue_depth': (1, None),
    'trajectory.max_points': (100, None),
}


def _int_bounds(parameters):
    """Enforce integer bounds without broken GPL IntegerRange descriptors."""
    for param in parameters:
        bounds = _INT_BOUNDS.get(param.name)
        if bounds is None:
            continue
        low, high = bounds
        value = int(param.value)
        if (low is not None and value < low) or (
                high is not None and value > high):
            return SetParametersResult(
                successful=False, reason=f'{param.name} out of range')
    return SetParametersResult(successful=True)


TOPIC_NAMES = (
    'target_observations_topic',
    'harvest_state_topic',
    'reconstruction_status_topic',
    'reconstruction_diagnostics_topic',
    'reconstruction_diagnostics_debug_topic',
    'grasp_decision_topic',
    'refined_pose_topic',
    'refined_axis_topic',
    'refined_diagnostics_topic',
    'manipulation_status_topic',
    'grasp_hypothesis_topic',
    'task_executor_state_topic',
    'task_executor_events_topic',
    'robot_status_topic',
    'joint_states_topic',
    'joint_status_topic',
    'tf_topic',
    'tf_static_topic',
    'scene_snapshot_topic',
    'job_topic',
    'metrics_topic',
    'debug_image_topic',
    'debug_image_raw_topic',
    'tsdf_cloud_topic',
    'tcp_path_topic',
    'tcp_markers_topic',
)

DEBUG_ENDPOINTS = (
    'run_harvest_action',
    'control_service',
    'begin_scene_service',
    'survey_action',
    'execute_action',
    'build_action',
    'check_reachability_service',
    'photo_pose_service',
    'preview_approach_service',
    'preview_full_service',
    'ack_recovery_service',
    'arm_service',
    'skill_cancel_service',
    'recon_save_session_service',
    'recon_reset_service',
    'recon_finalize_service',
    'recon_query_service',
    'manage_nodes_service',
    'set_enables_service',
)


@dataclass
class ObservabilityParams:
    """Live parameter snapshot; topics / endpoints are read-only maps."""

    host: str
    """HTTP 绑定地址."""
    port: int
    """8090 调试页端口."""
    param_poll_period_s: float
    """节点参数镜像轮询周期 [s]."""
    event_buffer_size: int
    """批次事件环形缓冲条数."""
    log_buffer_size: int
    """/api/state.logs 环形缓冲条数（/rosout，丢最旧）."""
    metrics_period_s: float
    """系统/GPU 采样周期 [s]."""
    metrics_process_patterns: Tuple[str, ...]
    """计入进程表的命令行子串."""
    record_enabled: bool
    """会话 bag 总开关."""
    record_root_dir: str
    """runs/session_* 根目录."""
    record_save_images: bool
    """bag 是否收图像."""
    record_save_clouds: bool
    """bag 是否收点云."""
    record_rosout: bool
    """bag 是否收 /rosout."""
    record_level: str
    """bag 话题档（如 'default'）."""
    record_max_total_bag_gb: float
    """bag 总容量上限 [GB]；0=不限."""
    record_queue_depth: int
    """bag 写队列深度上限；满时丢最旧并计数."""
    trajectory_enabled: bool
    """TCP 轨迹采样开关."""
    trajectory_base_frame: str
    """轨迹参考系."""
    trajectory_tip_frame: str
    """轨迹末端系."""
    trajectory_period_s: float
    """轨迹采样周期 [s]."""
    trajectory_min_step_m: float
    """轨迹最小步长 [m]."""
    trajectory_max_points: int
    """轨迹点上限."""
    topics: Mapping[str, str]
    """监控订阅话题名表（只读）."""
    debug_enabled: bool
    """8090 调试客户端总开关."""
    debug_motion_enabled: bool
    """8090 是否允许发运动类动作（仍过 authorizeStage）."""
    debug_action_timeout_s: float
    """调试动作等待上限 [s]."""
    debug_audit_enabled: bool
    """调试审计 jsonl 开关."""
    debug_endpoints: Mapping[str, str]
    """调试动作/服务名表（只读）."""
    startup_facts: str
    """harvest_system 注入的启动事实 JSON（空=独立起栈）."""
    selfcheck_enabled: bool
    """启动自检总开关."""
    selfcheck_period_s: float
    """周期复检间隔 [s]；0=只跑初次+手动."""
    selfcheck_initial_settle_s: float
    """托管栈激活后静置等待 [s]（模型加载/MoveIt 就绪缓冲）."""
    selfcheck_initial_timeout_s: float
    """managed 旗标未到时的初次检查兜底超时 [s]."""
    selfcheck_rate_window_s: float
    """帧率探针滑动窗 [s]."""
    selfcheck_camera_probe_enabled: bool
    """相机帧率探针（camera_enabled 时由 launch 置 true）."""
    selfcheck_camera_min_hz: float
    """相机流最低帧率 [Hz]."""
    selfcheck_disk_min_gb: float
    """runs 根所在盘最低剩余 [GB]."""
    selfcheck_expected_joints: Tuple[str, ...]
    """MUST 关节序期望（默认六名冻结序）."""
    selfcheck_expected_tcp_norm_m: float
    """wrist3_Link→tcp 平移模长期望 [m]；-1=跳过档案比对."""
    selfcheck_moveit_expected: bool
    """是否要求 move_group 在场."""
    selfcheck_force_expected: bool
    """是否要求 /force/points 在场."""
    selfcheck_model_paths: Tuple[str, ...]
    """启动期须在位的模型权重路径清单（空=SKIP）."""
    selfcheck_color_image_topic: str
    """相机帧率探针：color 话题."""
    selfcheck_depth_image_topic: str
    """相机帧率探针：depth 话题."""
    robot_status_probe_enabled: bool
    """robot_status 新鲜度探针（真机由 launch 置 true）."""

    @classmethod
    def attach(cls, node) -> 'ObservabilityParams':
        """Declare GPL parameters and keep this snapshot live on set."""
        node.add_on_set_parameters_callback(_int_bounds)
        try:
            listener = _Generated.ParamListener(node)
        finally:
            node.remove_on_set_parameters_callback(_int_bounds)
        # rclpy prepends callbacks: reject before the generated listener commits
        # its cache/user callback, while retaining startup declaration checks.
        node.add_on_set_parameters_callback(_int_bounds)
        snapshot = from_params(listener.get_params())

        def _commit(params):
            _copy_fields(snapshot, from_params(params))

        listener.set_user_callback(_commit)
        return snapshot


def _copy_fields(dst: ObservabilityParams, src: ObservabilityParams) -> None:
    """Overwrite dst in place so the node keeps one object."""
    for field in fields(src):
        setattr(dst, field.name, getattr(src, field.name))


def from_params(raw) -> ObservabilityParams:
    """Flatten the yaml namespace into ObservabilityParams."""
    topics = MappingProxyType(
        {name: str(getattr(raw, name)).strip() for name in TOPIC_NAMES})
    endpoints = raw.debug.endpoints
    debug_endpoints = MappingProxyType(
        {name: str(getattr(endpoints, name)).strip()
         for name in DEBUG_ENDPOINTS})
    snapshot = ObservabilityParams(
        host=str(raw.host).strip(),
        port=int(raw.port),
        param_poll_period_s=float(raw.param_poll_period_s),
        event_buffer_size=int(raw.event_buffer_size),
        log_buffer_size=int(raw.log_buffer_size),
        metrics_period_s=float(raw.metrics_period_s),
        metrics_process_patterns=tuple(
            str(item) for item in raw.metrics_process_patterns),
        record_enabled=bool(raw.record.enabled),
        record_root_dir=str(raw.record.root_dir),
        record_save_images=bool(raw.record.save_images),
        record_save_clouds=bool(raw.record.save_clouds),
        record_rosout=bool(getattr(raw.record, 'rosout', True)),
        record_level=str(getattr(raw.record, 'level', 'std')).strip() or 'std',
        record_max_total_bag_gb=float(raw.record.max_total_bag_gb),
        record_queue_depth=int(getattr(raw.record, 'queue_depth', 512)),
        trajectory_enabled=bool(raw.trajectory.enabled),
        trajectory_base_frame=str(raw.trajectory.base_frame).strip(),
        trajectory_tip_frame=str(raw.trajectory.tip_frame).strip(),
        trajectory_period_s=float(raw.trajectory.period_s),
        trajectory_min_step_m=float(raw.trajectory.min_step_m),
        trajectory_max_points=int(raw.trajectory.max_points),
        topics=topics,
        debug_enabled=bool(raw.debug.enabled),
        debug_motion_enabled=bool(raw.debug.motion_enabled),
        debug_action_timeout_s=float(raw.debug.action_timeout_s),
        debug_audit_enabled=bool(raw.debug.audit_enabled),
        debug_endpoints=debug_endpoints,
        startup_facts=str(getattr(raw, 'startup_facts', '') or ''),
        selfcheck_enabled=bool(getattr(raw.selfcheck, 'enabled', True)),
        selfcheck_period_s=float(raw.selfcheck.period_s),
        selfcheck_initial_settle_s=float(raw.selfcheck.initial_settle_s),
        selfcheck_initial_timeout_s=float(raw.selfcheck.initial_timeout_s),
        selfcheck_rate_window_s=float(raw.selfcheck.rate_window_s),
        selfcheck_camera_probe_enabled=bool(
            getattr(raw.selfcheck, 'camera_probe_enabled', False)),
        selfcheck_camera_min_hz=float(raw.selfcheck.camera_min_hz),
        selfcheck_disk_min_gb=float(raw.selfcheck.disk_min_gb),
        selfcheck_expected_joints=tuple(
            str(item) for item in raw.selfcheck.expected_joints),
        selfcheck_expected_tcp_norm_m=float(
            raw.selfcheck.expected_tcp_norm_m),
        selfcheck_moveit_expected=bool(raw.selfcheck.moveit_expected),
        selfcheck_force_expected=bool(raw.selfcheck.force_expected),
        selfcheck_model_paths=tuple(
            item.strip() for item in str(raw.selfcheck.model_paths).split(',')
            if item.strip()),
        selfcheck_color_image_topic=str(
            raw.selfcheck.color_image_topic).strip(),
        selfcheck_depth_image_topic=str(
            raw.selfcheck.depth_image_topic).strip(),
        robot_status_probe_enabled=bool(
            getattr(raw, 'robot_status_probe_enabled', False)),
    )
    facts = normalize_facts(parse_facts_json(snapshot.startup_facts))
    for key, value in selfcheck_overrides_from_facts(facts).items():
        setattr(snapshot, key, value)
    return snapshot
