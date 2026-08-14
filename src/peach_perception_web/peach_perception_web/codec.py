# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/BSD-3-Clause
"""ROS 消息到浏览器 JSON 的无状态转换工具."""

from __future__ import annotations

import json
import math
from typing import Any

_STATUS_NAMES = {0: 'ACCEPT', 1: 'REOBSERVE', 2: 'REJECT'}
_TRACKING_NAMES = {0: 'OBSERVED', 1: 'OCCLUDED', 2: 'LOST', 3: 'INVALID'}
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


def point(point_message) -> list[float]:
    """转换 geometry_msgs Point/Vector3."""
    return [
        float(point_message.x),
        float(point_message.y),
        float(point_message.z),
    ]


def candidate(candidate_message) -> dict:
    """转换抓取候选，保留浏览器需要的几何与版本信息."""
    pose = candidate_message.entry_pose
    return {
        'target_id': candidate_message.target_id,
        'entry_position': point(pose.position),
        'entry_quaternion_xyzw': [
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        ],
        'bag_bottom': point(candidate_message.bag_bottom),
        'bag_neck': point(candidate_message.bag_neck),
        'translation_direction': point(
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


def fitting(fitting_message) -> dict:
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


def target_observations(message) -> dict:
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
            'candidate': candidate(item.candidate),
            'fitting': fitting(item.fitting),
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
        'harvest_run_id': message.harvest_run_id,
        'target_set_locked': bool(message.target_set_locked),
        'target_count': int(message.target_count),
        'selected_target_id': message.selected_target_id,
        'observations': observations,
    }


def candidate_array(message) -> dict:
    """转换抓取候选数组."""
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'candidates': [candidate(item) for item in message.candidates],
    }


def fitting_array(message) -> dict:
    """转换拟合诊断数组."""
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'fittings': [fitting(item) for item in message.fittings],
    }


def vector_stamped(message) -> dict:
    """转换带时间戳向量."""
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'xyz': point(message.vector),
    }


def harvest_event(message) -> dict:
    """转换编排器过程/审计事件（HarvestEvent）为时间线条目."""
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
        'cycle_id': message.cycle_id,
        'target_id': message.target_id,
        'details': {
            item.key: item.value for item in message.details},
    }


def robot_status(message) -> dict:
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


def finite_or_none(value: Any) -> Any:
    """把非有限浮点递归转换为 JSON null."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: finite_or_none(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite_or_none(item) for item in value]
    return value
