"""
ObservabilityParams：generate_parameter_library_py 官方生成参数之上的不可变参数快照（A9）.

声明 / 类型 / 默认值 / 中文描述 / 范围校验的权威源统一为
config/observability_parameters.yaml（根键=节点名），由
generate_parameter_library_py 在构建期生成
peach_executor/observability_parameters.py；旧 DEFAULTS 手工描述表与
load_params 手写数值校验（端口越界、周期/缓冲非正）已由参数库校验器承担，
declare 期即拒绝非法值。运行路径只持有快照引用，不再逐回调 get_parameter。

本模块只依赖标准库与鸭子类型 Params 快照，不 import rclpy，可被无 ROS
上下文的单测直接装载。
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping, Tuple

# 话题键（*_topic）集中进 topics 映射，供快照只读索引
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
    'debug_image_topic',
    'debug_image_raw_topic',
    'tsdf_cloud_topic',
    'tcp_path_topic',
    'tcp_markers_topic',
)

# 手动调试操作面的目标端点名（debug.endpoints.* 子组，集中进 endpoints 映射）
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
)


@dataclass(frozen=True)
class ObservabilityParams:
    """
    启动期静态装载的不可变参数快照（运行路径只持有本快照引用）.

    topics 以 MappingProxyType 包装保证不可变；
    metrics_process_patterns 固化为 tuple。
    """

    host: str
    port: int
    param_poll_period_s: float
    event_buffer_size: int
    metrics_period_s: float
    metrics_process_patterns: Tuple[str, ...]
    record_enabled: bool
    record_root_dir: str
    record_save_images: bool
    record_save_clouds: bool
    trajectory_enabled: bool
    trajectory_base_frame: str
    trajectory_tip_frame: str
    trajectory_period_s: float
    trajectory_min_step_m: float
    trajectory_max_points: int
    topics: Mapping[str, str]
    debug_enabled: bool
    debug_motion_enabled: bool
    debug_token: str
    debug_action_timeout_s: float
    debug_audit_enabled: bool
    debug_endpoints: Mapping[str, str]


def declare(node) -> object:
    """
    生成 generate_parameter_library_py 的 ParamListener 并集中声明全部参数.

    Args:
        node: rclpy 节点（声明参数+挂 on_set 校验）.

    Returns
    -------
        ParamListener：调用方持有并用于读取 Params 快照（建议在 on_configure
        内创建，declare 期校验失败→TransitionCallbackReturn.FAILURE）.

    """
    # 构建期由 setup.py 的 generate_parameter_module 生成.
    from peach_executor.observability_parameters import peach_observability
    return peach_observability.ParamListener(node)


def from_params(p) -> ObservabilityParams:
    """
    从生成的 Params 快照集中装载为 frozen 快照.

    数值范围（port/周期/缓冲）已由参数库校验器承担；此处仅做字符串 strip 与
    只读容器转换。

    Args:
        p: peach_observability.Params（declare 后 get_params() 快照）.

    Returns
    -------
        ObservabilityParams（frozen；topics 为 MappingProxyType）.

    """
    topics = MappingProxyType(
        {name: str(getattr(p, name)).strip() for name in TOPIC_NAMES})
    endpoints = getattr(p.debug, 'endpoints')
    debug_endpoints = MappingProxyType(
        {name: str(getattr(endpoints, name)).strip()
         for name in DEBUG_ENDPOINTS})
    return ObservabilityParams(
        host=str(p.host).strip(),
        port=int(p.port),
        param_poll_period_s=float(p.param_poll_period_s),
        event_buffer_size=int(p.event_buffer_size),
        metrics_period_s=float(p.metrics_period_s),
        metrics_process_patterns=tuple(str(item) for item in p.metrics_process_patterns),
        record_enabled=bool(p.record.enabled),
        record_root_dir=str(p.record.root_dir),
        record_save_images=bool(p.record.save_images),
        record_save_clouds=bool(p.record.save_clouds),
        trajectory_enabled=bool(p.trajectory.enabled),
        trajectory_base_frame=str(p.trajectory.base_frame).strip(),
        trajectory_tip_frame=str(p.trajectory.tip_frame).strip(),
        trajectory_period_s=float(p.trajectory.period_s),
        trajectory_min_step_m=float(p.trajectory.min_step_m),
        trajectory_max_points=int(p.trajectory.max_points),
        topics=topics,
        debug_enabled=bool(p.debug.enabled),
        debug_motion_enabled=bool(p.debug.motion_enabled),
        debug_token=str(p.debug.token),
        debug_action_timeout_s=float(p.debug.action_timeout_s),
        debug_audit_enabled=bool(p.debug.audit_enabled),
        debug_endpoints=debug_endpoints,
    )
