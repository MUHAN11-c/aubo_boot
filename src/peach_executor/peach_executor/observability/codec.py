from __future__ import annotations
"""监控落盘记录器 + 消息→JSON 转换。"""

import json
import math
from typing import Any


_STATUS_NAMES = {0: 'ACCEPT', 1: 'REOBSERVE', 2: 'REJECT'}
# 与 PeachTargetObservation.msg 常量一一对应（阶段 D1 追加 4/5）：
# OUT_OF_VIEW=出画（检测框触图像边缘后消失）、DEPTH_VOID=深度空洞
# （掩膜内有效深度占比低于阈值）；沿用既有英文 token 风格，前端徽标
# 配色表（web/app.js trackingChip）按 token 键控。
_TRACKING_NAMES = {
    0: 'OBSERVED', 1: 'OCCLUDED', 2: 'LOST', 3: 'INVALID',
    4: 'OUT_OF_VIEW', 5: 'DEPTH_VOID',
}
_SEVERITY_NAMES = {0: 'INFO', 1: 'WARNING', 2: 'ERROR', 3: 'AUDIT'}


def parse_json_text(text: str, fallback_key: str = 'text') -> dict:
    """解析 String JSON；普通文本以指定键保留."""
    try:
        value = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return {fallback_key: str(text)}
    return value if isinstance(value, dict) else {'value': value}


def stamp_seconds(header) -> float:
    """把 ROS Header 时间戳转换为秒."""
    if header is None:
        return 0.0
    stamp = header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def to_point(point_message) -> list[float]:
    """转换 geometry_msgs Point/Vector3."""
    return [
        float(point_message.x),
        float(point_message.y),
        float(point_message.z),
    ]


def to_candidate(candidate_message) -> dict:
    """转换抓取候选，保留浏览器需要的几何与版本信息."""
    pose = candidate_message.entry_pose
    return {
        'target_id': candidate_message.target_id,
        'entry_position': to_point(pose.position),
        'entry_quaternion_xyzw': [
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        ],
        'bag_bottom': to_point(candidate_message.bag_bottom),
        'bag_neck': to_point(candidate_message.bag_neck),
        'translation_direction': to_point(
            candidate_message.translation_direction),
        'diameter_m': float(candidate_message.bag_diameter_upper_m),
        'travel_m': float(candidate_message.suggested_travel_m),
        'confidence': float(candidate_message.confidence),
        'status': _STATUS_NAMES.get(
            int(candidate_message.status), str(candidate_message.status)),
        'diagnostic_flags': list(candidate_message.diagnostic_flags),
        'strategy_id': candidate_message.strategy_id,
        'model_version': candidate_message.model_version,
        'calibration_version': candidate_message.calibration_version,
        'tool_version': candidate_message.tool_version,
    }


def to_fitting(fitting_message) -> dict:
    """转换几何拟合质量消息."""
    diameter = float(fitting_message.bag_diameter_upper_m)
    if diameter <= 0.0 and float(fitting_message.fruit_radius_m) > 0.0:
        diameter = 2.0 * float(fitting_message.fruit_radius_m)
    return {
        'target_id': fitting_message.target_id,
        'target_kind': fitting_message.target_kind,
        'status': _STATUS_NAMES.get(
            int(fitting_message.status), str(fitting_message.status)),
        'axis_confidence': float(fitting_message.axis_confidence),
        'valid_depth_ratio': float(fitting_message.valid_depth_ratio),
        'n_points': int(fitting_message.n_points),
        'error_budget_mm': float(fitting_message.error_budget_mm),
        'radial_clearance_mm': float(fitting_message.radial_clearance_mm),
        'diameter_m': diameter,
        'cylinder_rms_m': float(fitting_message.cylinder_rms_m),
        'sphere_rms_m': float(fitting_message.sphere_rms_m),
        'inlier_ratio': max(
            float(fitting_message.cylinder_inlier_ratio),
            float(fitting_message.sphere_inlier_ratio),
        ),
        'diagnostic_flags': list(fitting_message.diagnostic_flags),
    }


def to_target_observations(message) -> dict:
    """转换全局目标快照；故意排除大体积 mask 像素."""
    observations = []
    for item in message.observations:
        observations.append({
            'target_id': item.target_id,
            'priority': int(item.priority),
            'confirmed': bool(item.confirmed),
            'selected': bool(item.selected),
            'harvest_status': item.harvest_status,
            'tracking_status': _TRACKING_NAMES.get(
                int(item.tracking_status), str(item.tracking_status)),
            'camera_distance_m': float(item.camera_distance_m),
            'confidence': float(item.confidence),
            'candidate': to_candidate(item.candidate),
            'fitting': to_fitting(item.fitting),
            'mask': {
                'width': int(item.mask.width),
                'height': int(item.mask.height),
                'stamp': stamp_seconds(item.mask.header),
            },
            'diagnostic_flags': list(item.diagnostic_flags),
        })
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'snapshot_id': int(message.snapshot_id),
        'scene_epoch': int(getattr(message, 'scene_epoch', 0) or 0),
        'harvest_run_id': message.harvest_run_id,
        'target_set_locked': bool(message.target_set_locked),
        'target_count': int(message.target_count),
        'selected_target_id': message.selected_target_id,
        'collecting_count': int(getattr(message, 'collecting_count', 0) or 0),
        'pending_count': int(getattr(message, 'pending_count', 0) or 0),
        'observations': observations,
    }


def to_candidate_array(message) -> dict:
    """转换抓取候选数组."""
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'candidates': [to_candidate(item) for item in message.candidates],
    }


def to_fitting_array(message) -> dict:
    """转换拟合诊断数组."""
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'fittings': [to_fitting(item) for item in message.fittings],
    }


def to_vector_stamped(message) -> dict:
    """转换带时间戳向量."""
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'xyz': to_point(message.vector),
    }


def to_harvest_event(message) -> dict:
    """转换调度过程/审计事件（CanonicalEvent / HarvestEvent）为时间线条目."""
    return {
        'stamp': stamp_seconds(message.header),
        'sequence': int(message.sequence),
        'severity': int(message.severity),
        'severity_name': _SEVERITY_NAMES.get(
            int(message.severity), str(message.severity)),
        'code': message.code,
        'message': message.message,
        'request_id': message.request_id,
        'run_id': message.run_id,
        'cycle_id': getattr(message, 'cycle_id', ''),
        'state_seq': int(getattr(message, 'state_seq', 0) or 0),
        'target_id': message.target_id,
        'details': {
            item.key: item.value for item in message.details},
    }


def to_robot_status(message) -> dict:
    """转换机械臂状态（aubo_msgs/RobotStatus，简化 industrial 语义）."""
    return {
        'mode': int(message.mode),
        'e_stopped': int(message.e_stopped),
        'drives_powered': int(message.drives_powered),
        'motion_possible': int(message.motion_possible),
        'in_motion': int(message.in_motion),
        'in_error': int(message.in_error),
        'error_code': int(message.error_code),
    }


def _valid_scalar(value) -> float | None:
    """无效标量约定（-1，见 ReconstructionStatus.msg）→ None；其余转 float."""
    value = float(value)
    return value if value >= 0.0 else None


def to_reconstruction_status(message) -> dict:
    """
    结构化重建诊断（ReconstructionStatus）→ 浏览器镜像 dict.

    键名沿用旧 JSON 契约（state/target_id/captured_views/tf_latency_ms…），
    前端无需改动；无效标量（-1）折回 None 让前端按「无数据」渲染。
    view_coverage 只保留类型化后的摘要键；逐机位明细与 tsdf/registration/
    overlap/refined 由 diagnostics_debug 调试话题在 observability 层合并补充。
    """
    center = [float(v) for v in message.target_center_base]
    depth_ratio = _valid_scalar(message.valid_depth_ratio)
    return {
        'stamp': stamp_seconds(message.header),
        'harvest_run_id': message.harvest_run_id,
        'selected_target_id': message.selected_target_id,
        'state': message.state,
        'target_id': message.target_id,
        'target_center_base': (
            None if center == [-1.0, -1.0, -1.0] else center),
        'captured_views': int(message.captured_views),
        'rejected_views': int(message.rejected_views),
        'tf_failures': int(message.tf_failures),
        'tf_latency_ms': _valid_scalar(message.tf_latency_ms),
        'valid_depth_ratio': depth_ratio,
        'view_coverage': {
            'max_baseline_deg': _valid_scalar(message.max_baseline_deg),
            'mean_nearest_baseline_deg': _valid_scalar(
                message.mean_nearest_baseline_deg),
            'valid_depth_ratio_mean': depth_ratio,
        },
        'view_directions': [to_point(v) for v in message.view_directions],
    }


def to_grasp_hypothesis(message) -> dict:
    """技能侧抓取假设（GraspHypothesis）→ 浏览器镜像；不构成运动指令."""
    pose = message.entry_pose
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'target_id': message.target_id,
        'entry_position': to_point(pose.position),
        'entry_quaternion_xyzw': [
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        ],
        'standoff_m': float(message.standoff_m),
        'travel_m': float(message.travel_m),
        'envelope_clearance_m': float(message.envelope_clearance_m),
        'rank_score': float(message.rank_score),
        'diagnostic_flags': list(message.diagnostic_flags),
    }


def to_grasp_decision(message) -> dict:
    """
    抓取许可（GraspDecision）→ 浏览器镜像 dict.

    融合几何与 allowed 独立：有轴则透出入口/剪切参考供预抓取目视；
    allowed 只表示套入/剪切接触许可。
    """
    value = {
        'stamp': stamp_seconds(message.header),
        'harvest_run_id': message.harvest_run_id,
        'target_id': message.target_id,
        'allowed': bool(message.allowed),
        'reason': message.reason,
    }
    axis = message.axis
    has_geom = (axis.x * axis.x + axis.y * axis.y + axis.z * axis.z) > 0.25
    if message.allowed or has_geom:
        value.update({
            'entry': to_point(message.entry),
            'pregrasp': to_point(message.pregrasp),
            'cut_pose': to_point(message.cut_pose),
            'axis': to_point(message.axis),
            'diameter_m': float(message.diameter_m),
            'rmse_m': float(message.rmse_m),
            'inlier_ratio': float(message.inlier_ratio),
        })
    return value


def finite_or_none(value: Any) -> Any:
    """把非有限浮点递归转换为 JSON null."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: finite_or_none(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite_or_none(item) for item in value]
    return value
