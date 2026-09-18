"""
ObservabilityParams：yaml 直读后的扁平快照.

部署事实源 ``peach_harvester/config/observability.yaml``。节点一行
``ObservabilityParams.attach(node)``：声明叶子并挂规则校验（端口/周期/
缓冲越界启动期拒绝、运行期非法 set 即拒），``ros2 param set`` 原地刷新本
对象。话题名集中进 topics；debug 端点集中进 debug_endpoints。
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Mapping, Tuple

from peach_harvester.supervisor.param_rules import check as _check
from peach_harvester.supervisor.params import peach_observability

_RULES = {  # 键 -> 校验规则表（启动期非法即拒启；运行期非法 set 即拒）
    'port': (('bounds', 1.0, 65535.0),),
    'param_poll_period_s': (('gt', 0.0),),
    'event_buffer_size': (('gt_eq', 1),),
    'metrics_period_s': (('gt', 0.0),),
    'trajectory.period_s': (('gt', 0.0),),
    'trajectory.min_step_m': (('gt', 0.0),),
    'trajectory.max_points': (('gt_eq', 100),),
    'record.max_total_bag_gb': (('gt_eq', 0.0),),
    'debug.action_timeout_s': (('gt', 0.0),),
}


def _validate(name, value):
    """逐条规则校验；返回拒绝理由或 None."""
    for rule in _RULES.get(name, ()):
        why = _check(rule, value, name)
        if why:
            return why
    return None


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
    record_bag_topics: Tuple[str, ...]
    """显式收录话题列表."""
    record_max_total_bag_gb: float
    """bag 总容量上限 [GB]；0=不限."""
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
    debug_token: str
    """预留；现行无鉴权."""
    debug_action_timeout_s: float
    """调试动作等待上限 [s]."""
    debug_audit_enabled: bool
    """调试审计 jsonl 开关."""
    debug_endpoints: Mapping[str, str]
    """调试动作/服务名表（只读）."""

    @classmethod
    def attach(cls, node) -> 'ObservabilityParams':
        """Declare yaml leaves and keep this snapshot live on param set."""
        holder = []

        def _commit(raw):
            if holder:
                _copy_fields(holder[0], from_params(raw))

        raw = peach_observability.attach(
            node, on_commit=_commit, validate=_validate)
        snapshot = from_params(raw)
        holder.append(snapshot)
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
    return ObservabilityParams(
        host=str(raw.host).strip(),
        port=int(raw.port),
        param_poll_period_s=float(raw.param_poll_period_s),
        event_buffer_size=int(raw.event_buffer_size),
        metrics_period_s=float(raw.metrics_period_s),
        metrics_process_patterns=tuple(
            str(item) for item in raw.metrics_process_patterns),
        record_enabled=bool(raw.record.enabled),
        record_root_dir=str(raw.record.root_dir),
        record_save_images=bool(raw.record.save_images),
        record_save_clouds=bool(raw.record.save_clouds),
        record_rosout=bool(getattr(raw.record, 'rosout', True)),
        record_level=str(getattr(raw.record, 'level', 'std')).strip() or 'std',
        record_bag_topics=tuple(
            str(item).strip() for item in raw.record.bag_topics),
        record_max_total_bag_gb=float(raw.record.max_total_bag_gb),
        trajectory_enabled=bool(raw.trajectory.enabled),
        trajectory_base_frame=str(raw.trajectory.base_frame).strip(),
        trajectory_tip_frame=str(raw.trajectory.tip_frame).strip(),
        trajectory_period_s=float(raw.trajectory.period_s),
        trajectory_min_step_m=float(raw.trajectory.min_step_m),
        trajectory_max_points=int(raw.trajectory.max_points),
        topics=topics,
        debug_enabled=bool(raw.debug.enabled),
        debug_motion_enabled=bool(raw.debug.motion_enabled),
        debug_token=str(raw.debug.token),
        debug_action_timeout_s=float(raw.debug.action_timeout_s),
        debug_audit_enabled=bool(raw.debug.audit_enabled),
        debug_endpoints=debug_endpoints,
    )
