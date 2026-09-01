from __future__ import annotations
"""监控落盘记录器 + 消息→JSON 转换。"""

import csv
import io
import json
import math
from pathlib import Path
import queue
import re
import threading
import time
from typing import Any

import numpy as np


# === codec.py ===

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


def _valid_scalar(value) -> float | None:
    """无效标量约定（-1，见 ReconstructionStatus.msg）→ None；其余转 float."""
    value = float(value)
    return value if value >= 0.0 else None


def reconstruction_status(message) -> dict:
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
        'view_directions': [point(v) for v in message.view_directions],
    }


def grasp_hypothesis(message) -> dict:
    """技能侧抓取假设（GraspHypothesis）→ 浏览器镜像；不构成运动指令."""
    pose = message.entry_pose
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'target_id': message.target_id,
        'entry_position': point(pose.position),
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


def grasp_decision(message) -> dict:
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
            'entry': point(message.entry),
            'pregrasp': point(message.pregrasp),
            'cut_pose': point(message.cut_pose),
            'axis': point(message.axis),
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


# === recorder.py ===

# target 终局事件码（触发点云保存与图像索引，并进入逐目标 outcome 表）；
# target_operator_skipped（A13 拆分）：操作员跳过独立于系统判定的 target_skipped
TERMINAL_TARGET_CODES = {
    'target_succeeded', 'target_skipped', 'target_failed',
    'target_rejected', 'target_canceled', 'target_operator_skipped',
}
# 需要在 image_index.jsonl 里关联最近调试图的事件码
IMAGE_EVENT_CODES = {'round_locked', 'photo_pose_reached'} | TERMINAL_TARGET_CODES
# HarvestState batch_state：1 DISCOVERY / 2 RUNNING / 3 PAUSE_PENDING /
# 4 PAUSED / 5 MAINTENANCE 都算批次活动期
ACTIVE_BATCH_STATES = {1, 2, 3, 4, 5, 9}
# 终局：6 COMPLETED / 7 RECOVERY_REQUIRED / 8 INTERRUPTED
TERMINAL_BATCH_STATES = {6, 7, 8}
# 终局：6 COMPLETED / 7 RECOVERY_REQUIRED / 8 INTERRUPTED
# （2026-08 消息契约删除预留的 FAULT，后续状态顺序前移）
TERMINAL_BATCH_STATES = {6, 7, 8}
_BATCH_STATE_NAMES = {
    6: 'COMPLETED', 7: 'RECOVERY_REQUIRED', 8: 'INTERRUPTED',
}
# target_phase 枚举名（与 peach_interfaces/HarvestState.msg 一致）
_PHASE_NAMES = {
    0: 'IDLE', 1: 'SELECTING', 2: 'OBSERVING', 3: 'FINALIZING',
    4: 'VALIDATING', 5: 'APPROACHING', 6: 'TOOL_ACTION', 7: 'RETREATING',
    8: 'COMPLETING', 9: 'TARGET_SUCCEEDED', 10: 'TARGET_SKIPPED',
    11: 'TARGET_FAILED',
}
# 逐目标阶段耗时表的工作阶段列（不含 IDLE 与三个终局相）
_WORKING_PHASES = [
    'SELECTING', 'OBSERVING', 'FINALIZING', 'VALIDATING',
    'APPROACHING', 'TOOL_ACTION', 'RETREATING', 'COMPLETING',
]
# PointField datatype → numpy 类型串（仅本模块用到的）
_FIELD_DTYPES = {6: 'u4', 7: 'f4', 8: 'f8'}
# jsonl 写缓冲批量阈值（条）：达到即一次性落盘；未达阈值时由写线程队列
# 轮询超时（0.2s 空闲）兜底 flush，兼顾吞吐与可见时延。
_JSONL_BATCH_LINES = 64


def _sanitize(text: str) -> str:
    """文件名安全的 id（保留字母数字._-，其余折成下划线）."""
    cleaned = re.sub(r'[^A-Za-z0-9_.-]+', '_', str(text or ''))
    return cleaned.strip('_') or 'none'


def session_folder(root, kind: str, now: float | None = None) -> Path:
    """批次/空闲目录命名：<root>/<kind>_<yyyyMMdd_HHmmss>（kind=run|idle）."""
    stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(now or time.time()))
    return Path(root) / f'{kind}_{stamp}'


def slim_targets(value: dict) -> dict:
    """目标快照瘦身：剔除 mask 字段，保留 candidate/fitting 摘要."""
    observations = []
    for item in value.get('observations') or []:
        entry = {key: item.get(key) for key in (
            'target_id', 'priority', 'confirmed', 'selected',
            'harvest_status', 'tracking_status', 'camera_distance_m',
            'confidence', 'diagnostic_flags')}
        entry['candidate'] = item.get('candidate')
        entry['fitting'] = item.get('fitting')
        observations.append(entry)
    return {
        'stamp': value.get('stamp'),
        'snapshot_id': value.get('snapshot_id'),
        'harvest_run_id': value.get('harvest_run_id'),
        'target_set_locked': value.get('target_set_locked'),
        'target_count': value.get('target_count'),
        'selected_target_id': value.get('selected_target_id'),
        'observations': observations,
    }


def xyz_rgb_from_pointcloud2(message):
    """
    手写 numpy 解析 PointCloud2：返回 (xyz float32 Nx3, rgb uint8 Nx3|None).

    rgb 字段按 RViz 约定为 float32 位打包（r<<16|g<<8|b），也接受
    uint32 打包与 rgba 命名；非有限点剔除；空云返回 None。
    """
    count = int(message.width) * int(message.height)
    if count <= 0:
        return None
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        return None
    point_step = int(message.point_step)
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    if raw.size < count * point_step:
        return None
    raw = raw[:count * point_step].reshape(count, point_step)
    endian = '>' if message.is_bigendian else '<'

    def column(name):
        field = fields[name]
        dtype = _FIELD_DTYPES.get(int(field.datatype))
        if dtype is None:
            raise ValueError(f'不支持的 PointField 类型: {name}={field.datatype}')
        offset = int(field.offset)
        view = np.ascontiguousarray(raw[:, offset:offset + 4])
        return view.view(f'{endian}{dtype}')[:, 0]

    xyz = np.stack(
        [column('x'), column('y'), column('z')], axis=1).astype('<f4')
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    if xyz.shape[0] == 0:
        return None
    rgb = None
    rgb_name = 'rgb' if 'rgb' in fields else 'rgba' if 'rgba' in fields else None
    if rgb_name is not None:
        packed = column(rgb_name)
        if int(fields[rgb_name].datatype) == 7:  # float32 位打包 → 按位看
            packed = packed.astype('<f4').view('<u4')
        packed = packed[finite]
        rgb = np.stack([
            (packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF,
        ], axis=1).astype(np.uint8)
    return xyz, rgb


def write_ply(path, xyz: np.ndarray, rgb: np.ndarray | None = None) -> int:
    """写 binary_little_endian PLY（xyz float32 + 可选 uchar rgb），返回点数."""
    count = int(xyz.shape[0])
    if count == 0:
        return 0
    header_lines = [
        'ply', 'format binary_little_endian 1.0',
        'comment peach_observability recorder',
        f'element vertex {count}',
        'property float x', 'property float y', 'property float z',
    ]
    if rgb is not None:
        header_lines += [
            'property uchar red', 'property uchar green', 'property uchar blue']
    header_lines.append('end_header')
    with open(path, 'wb') as stream:
        stream.write(('\n'.join(header_lines) + '\n').encode('ascii'))
        if rgb is None:
            np.asarray(xyz, dtype='<f4').tofile(stream)
        else:
            dtype = np.dtype([
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
            ])
            blob = np.empty(count, dtype=dtype)
            blob['x'], blob['y'], blob['z'] = (
                xyz[:, 0], xyz[:, 1], xyz[:, 2])
            blob['red'], blob['green'], blob['blue'] = (
                rgb[:, 0], rgb[:, 1], rgb[:, 2])
            blob.tofile(stream)
    return count


def summarize_metrics(records: list[dict]) -> dict:
    """metrics.jsonl 记录列表 → CPU/内存/GPU 均值与峰值统计."""
    stats = {'samples': len(records)}

    def aggregate(pick):
        values = [pick(item) for item in records]
        values = [float(v) for v in values
                  if v is not None and np.isfinite(float(v))]
        if not values:
            return None
        return {'mean': round(float(np.mean(values)), 2),
                'max': round(float(np.max(values)), 2)}

    stats['cpu_percent'] = aggregate(lambda item: item.get('cpu_percent'))
    stats['memory_percent'] = aggregate(lambda item: item.get('memory_percent'))
    stats['gpu_utilization_percent'] = aggregate(
        lambda item: (item.get('gpu') or {}).get('utilization_percent'))
    stats['gpu_memory_used_mb'] = aggregate(
        lambda item: (item.get('gpu') or {}).get('memory_used_mb'))
    return stats


def summarize_tcp(records: list[dict]) -> dict:
    """tcp_trajectory.jsonl → 路径长/弦长/绕行比（与网页同源算法）."""
    from .tcp_trajectory import path_metrics
    xyz = []
    for item in records:
        try:
            xyz.append((float(item['x']), float(item['y']), float(item['z'])))
        except (KeyError, TypeError, ValueError):
            continue
    metrics = path_metrics(xyz)
    metrics['samples'] = len(records)
    return metrics


def _reason_text(message: str) -> str:
    """
    事件 message → 人读原因（摘要「原因」列）.

    message 是 JSON：优先取 failure_code/reason 字段（终局事件已由执行器
    并入 outcome 细节）；只剩 code 本身时给空（码已在 outcome 列，不重复）；
    非 JSON 原样返回。
    """
    if not message:
        return ''
    try:
        payload = json.loads(message)
    except (json.JSONDecodeError, TypeError, ValueError):
        return str(message)
    if not isinstance(payload, dict):
        return str(payload)
    for key in ('failure_code', 'reason', 'message'):
        value = payload.get(key)
        if value and key != 'code':
            return str(value)
    rest = {key: val for key, val in payload.items() if key != 'code'}
    return json.dumps(rest, ensure_ascii=False) if rest else ''


def build_target_rows(events: list[dict], priorities: dict) -> list[dict]:
    """从事件流推导逐目标 outcome 表（派发/终局时刻与耗时）."""
    dispatched = {}
    rows = {}
    for event in events:
        code = event.get('code')
        target_id = event.get('target_id') or ''
        stamp = float(event.get('stamp') or 0.0)
        if code == 'target_dispatched' and target_id:
            dispatched[target_id] = stamp
            rows.setdefault(target_id, {
                'target_id': target_id,
                'priority': priorities.get(target_id),
                'outcome': 'unfinished', 'reason': '',
                'dispatched_at': stamp, 'finished_at': None,
            })
        elif code in TERMINAL_TARGET_CODES and target_id:
            row = rows.setdefault(target_id, {
                'target_id': target_id,
                'priority': priorities.get(target_id),
                'outcome': None, 'reason': '',
                'dispatched_at': dispatched.get(target_id),
                'finished_at': None,
            })
            row['outcome'] = code.replace('target_', '')
            row['reason'] = _reason_text(event.get('message') or '')
            row['finished_at'] = stamp
            if row['dispatched_at'] is None:
                row['dispatched_at'] = dispatched.get(target_id)
    result = []
    for row in rows.values():
        start = row.get('dispatched_at')
        end = row.get('finished_at')
        row['duration_s'] = (round(end - start, 2)
                             if start and end else None)
        result.append(row)
    result.sort(key=lambda row: (row.get('dispatched_at') is None,
                                 row.get('dispatched_at') or 0.0))
    return result


def phase_durations(states: list[dict]) -> list[dict]:
    """
    从 state.jsonl 的 target_phase 跃迁推导逐周期各阶段耗时.

    相邻两条（revision 去重后的）状态记录的 recorded_at 差值记到前一条的
    target_phase 上，按 (cycle_id, target_id) 聚合计入对应工作阶段；最后
    一条记录的阶段无后继，不计（尾部截断误差）。
    """
    cycles = []
    index = {}
    for prev, nxt in zip(states, states[1:]):
        start = float(prev.get('recorded_at') or 0.0)
        end = float(nxt.get('recorded_at') or 0.0)
        phase = _PHASE_NAMES.get(prev.get('target_phase'))
        if phase not in _WORKING_PHASES or end <= start:
            continue
        key = (prev.get('cycle_id') or '', prev.get('target_id') or '')
        if key not in index:
            index[key] = len(cycles)
            cycles.append({
                'cycle_id': key[0], 'target_id': key[1],
                'phases': {}, 'total_s': 0.0,
            })
        row = cycles[index[key]]
        row['phases'][phase] = round(row['phases'].get(phase, 0.0) + end - start, 2)
        row['total_s'] = round(row['total_s'] + end - start, 2)
    return cycles


def event_statistics(events: list[dict]) -> dict:
    """事件统计：按 code 与 severity 计数."""
    by_code = {}
    by_severity = {}
    for event in events:
        code = event.get('code') or 'unknown'
        by_code[code] = by_code.get(code, 0) + 1
        severity = event.get('severity_name') or str(event.get('severity'))
        by_severity[severity] = by_severity.get(severity, 0) + 1
    return {'total': len(events), 'by_code': by_code, 'by_severity': by_severity}


def perception_stats(records: list[dict]) -> dict:
    """perception.jsonl → 帧率估算（帧间隔中位数）与目标数变化范围."""
    frames = [item for item in records if item.get('kind') == 'targets']
    stamps = [float((item.get('data') or {}).get('stamp')
                    or item.get('recorded_at') or 0.0) for item in frames]
    stamps = [stamp for stamp in stamps if stamp > 0]
    intervals = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    counts = [int((item.get('data') or {}).get('target_count') or 0)
              for item in frames]
    median = float(np.median(intervals)) if intervals else None
    return {
        'frames': len(frames),
        'median_interval_s': round(median, 3) if median else None,
        'fps': round(1.0 / median, 2) if median else None,
        'target_count_min': min(counts) if counts else None,
        'target_count_max': max(counts) if counts else None,
    }


def reconstruction_final(records: list[dict]) -> dict:
    """reconstruction.jsonl 最后一条 diagnostics 的关键指标终值."""
    diagnostics = [item.get('data') or {} for item in records
                   if item.get('topic') == 'diagnostics']
    if not diagnostics:
        return {}
    last = diagnostics[-1]
    coverage = last.get('view_coverage') or {}
    tsdf = last.get('tsdf') or {}
    decision = last.get('grasp_decision') or {}
    result = {
        'state': last.get('state'),
        'target_id': last.get('target_id'),
        'captured_views': last.get('captured_views'),
        'rejected_views': last.get('rejected_views'),
        'tf_failures': last.get('tf_failures'),
        'tf_latency_ms': last.get('tf_latency_ms'),
        'cloud_points': last.get('cloud_points'),
        'max_baseline_deg': coverage.get('max_baseline_deg'),
        'mean_nearest_baseline_deg': coverage.get('mean_nearest_baseline_deg'),
        'tsdf_points': tsdf.get('points'),
        'tsdf_integrate_time_s': tsdf.get('integrate_time_s'),
        'grasp_allowed': decision.get('allowed'),
        'grasp_reason': decision.get('reason'),
        'skipped_views': last.get('skipped_views'),
        'skip_reasons': last.get('skip_reasons'),
        'last_skip_code': last.get('last_skip_code'),
        'last_skip_reason': last.get('last_skip_reason'),
    }
    return {key: value for key, value in result.items() if value is not None}


def job_per_target(records: list[dict]) -> list[dict]:
    """job.jsonl → 逐目标最后一张作业票（过程线终态 + 坐标）."""
    by_id = {}
    for item in records:
        job = item if 'stages' in item else (item.get('data') or item)
        target_id = str(job.get('target_id') or '')
        if not target_id:
            continue
        by_id[target_id] = job
    return list(by_id.values())


def reconstruction_per_target(records: list[dict]) -> list[dict]:
    """reconstruction.jsonl → 逐目标最大视角数与门禁跳过码."""
    by_id = {}
    for item in records:
        if item.get('topic') != 'diagnostics':
            continue
        data = item.get('data') or {}
        tid = str(data.get('target_id') or '')
        if not tid:
            continue
        row = by_id.setdefault(tid, {
            'target_id': tid,
            'captured_views_max': 0,
            'skipped_views_max': 0,
            'skip_reasons': {},
            'last_state': '',
        })
        row['captured_views_max'] = max(
            row['captured_views_max'], int(data.get('captured_views') or 0))
        row['skipped_views_max'] = max(
            row['skipped_views_max'], int(data.get('skipped_views') or 0))
        reasons = data.get('skip_reasons') or {}
        if isinstance(reasons, dict):
            for key, value in reasons.items():
                row['skip_reasons'][key] = max(
                    row['skip_reasons'].get(key, 0), int(value or 0))
        row['last_state'] = data.get('state') or row['last_state']
        if data.get('last_skip_code'):
            row['last_skip_code'] = data.get('last_skip_code')
            row['last_skip_reason'] = data.get('last_skip_reason')
    return list(by_id.values())


def _clock(stamp: float | None) -> str:
    """把 epoch 秒折成本地可读时刻；空给 —."""
    if not stamp:
        return '—'
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(stamp))


def _fmt_xyz(value) -> str:
    """base_link 坐标三点写成 0.000,0.000,0.000；空给 —."""
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return '—'
    try:
        return ','.join(f'{float(item):.3f}' for item in value[:3])
    except (TypeError, ValueError):
        return '—'


def build_summary_csv(rows: list[dict]) -> str:
    """逐目标 outcome 表 → CSV 文本."""
    buffer = io.StringIO()
    writer = csv.writer(buffer)
    writer.writerow(['target_id', 'priority', 'outcome', 'reason',
                     'dispatched_at', 'finished_at', 'duration_s'])
    for row in rows:
        writer.writerow([
            row.get('target_id'), row.get('priority'),
            row.get('outcome'), row.get('reason'),
            _clock(row.get('dispatched_at')), _clock(row.get('finished_at')),
            row.get('duration_s') if row.get('duration_s') is not None else '',
        ])
    return buffer.getvalue()


def build_summary_markdown(run_id: str, started: float | None,
                           ended: float | None, batch_state, rounds: int,
                           rows: list[dict], phases: list[dict],
                           event_stats: dict, perception: dict, recon: dict,
                           metrics: dict,
                           recon_targets: list[dict] | None = None,
                           jobs: list[dict] | None = None,
                           tcp: dict | None = None,
                           ledger_hint: str | None = None) -> str:
    """批次概览 + 验收门对照 + 逐目标/逐阶段耗时 + 事件/感知/重建/性能统计."""
    counts = {}
    for row in rows:
        counts[row['outcome']] = counts.get(row['outcome'], 0) + 1
    duration = (round(ended - started, 1)
                if started and ended and ended > started else None)
    lines = [
        f'# 采摘批次摘要 `{run_id}`', '',
        '## 批次概览', '',
        f"- 终局状态：{_BATCH_STATE_NAMES.get(batch_state, '未知/未终止')}",
        f'- 复扫轮数：{rounds}',
        f'- 开始时间：{_clock(started)}',
        f'- 结束时间：{_clock(ended)}',
        f'- 总时长：{duration} s' if duration is not None else '- 总时长：—',
        '- 终局计数：' + (
            ', '.join(f'{k}={v}' for k, v in sorted(counts.items()))
            if counts else '无'),
        '- 会话根：' + (
            f'`runs/{ledger_hint}/`（账本/感知/会话同根，R7 单根目录）'
            if ledger_hint else '（事件流无 request_id）'),
        '', '## 验收门对照（口径见 docs/testing.md 量化基线）', '',
        '| 指标 | 实测 | 门 | 判定 |',
        '|---|---:|---|---|',
    ]
    # 门值与 testing.md 基线一致：相机 ≥2.0 FPS（设计 2.5）、接触期 tf_failures=0、
    # PREGRASP_ONLY 里程碑=至少一颗到预抓取停住（succeeded）。
    fps = perception.get('fps') if perception else None
    lines.append(
        f"| 感知帧率 FPS | {fps if fps is not None else '—'} | ≥ 2.0 | "
        + ('—' if fps is None else ('✓' if fps >= 2.0 else '✗')) + ' |')
    tf_failures = (recon or {}).get('tf_failures')
    lines.append(
        f"| 重建 TF 失败次数 | {tf_failures if tf_failures is not None else '—'} | = 0 | "
        + ('—' if tf_failures is None else ('✓' if tf_failures == 0 else '✗'))
        + ' |')
    attempted = sum(
        counts.get(key, 0)
        for key in ('succeeded', 'skipped', 'failed', 'canceled'))
    reached = counts.get('succeeded', 0)
    gate_text = '—' if attempted == 0 else ('✓' if reached > 0 else '✗')
    lines.append(
        f'| 到预抓取停住（succeeded） | {reached} | ≥1（有派发目标时） | '
        + gate_text + ' |')
    lines += [
        '', '## 逐目标 outcome', '',
        '| target_id | 优先级 | outcome | 耗时 s | 原因 |',
        '|---|---:|---|---:|---|',
    ]
    for row in rows:
        lines.append(
            f"| {row.get('target_id')} | {row.get('priority') or '—'} | "
            f"{row.get('outcome')} | "
            f"{row.get('duration_s') if row.get('duration_s') is not None else '—'} | "
            f"{(row.get('reason') or '').replace('|', '/')} |")
    lines += [
        '', '## 每阶段耗时统计（按目标周期）', '',
        '| 周期 | 目标 | ' + ' | '.join(_WORKING_PHASES) + ' | 合计 s |',
        '|---|---|' + '---:|' * (len(_WORKING_PHASES) + 1),
    ]
    written = 0
    for item in phases:
        # 0.0s 伪行过滤：相位只有尾随瞬态（如仅 SELECTING 一瞬）的空周期
        # 不进表，避免误导逐阶段解读
        if float(item.get('total_s') or 0) <= 0.0:
            continue
        written += 1
        cells = [str(item['phases'].get(name, '—')) for name in _WORKING_PHASES]
        lines.append(
            f"| {item['cycle_id'] or '—'} | {item['target_id'] or '—'} | "
            + ' | '.join(cells) + f" | {item['total_s']} |")
    if not written:
        lines.append('| — | — | ' + ' | '.join(['—'] * len(_WORKING_PHASES)) + ' | — |')
    lines += ['', '## 事件统计', '', f"- 事件总数：{event_stats.get('total', 0)}"]
    for severity, count in sorted(event_stats.get('by_severity', {}).items()):
        lines.append(f'- severity {severity}：{count}')
    for code, count in sorted(event_stats.get('by_code', {}).items()):
        lines.append(f'- `{code}`：{count}')
    lines += ['', '## 感知统计', '',
              f"- 记录帧数：{perception.get('frames', 0)}",
              f"- 帧间隔中位数：{perception.get('median_interval_s')} s"
              f"（≈{perception.get('fps')} FPS）",
              f"- 目标数范围：{perception.get('target_count_min')}"
              f" ~ {perception.get('target_count_max')}",
              '', '## 重建关键指标终值', '']
    if recon:
        for key, value in recon.items():
            lines.append(f'- {key}：{value}')
    else:
        lines.append('- （无重建诊断记录）')
    lines += ['', '## 逐目标重建视角', '']
    if recon_targets:
        lines.append(
            '| target_id | captured_views_max | skipped_views_max | '
            'last_state | last_skip |')
        lines.append('|---|---:|---:|---|---|')
        for item in recon_targets:
            skip = item.get('last_skip_code') or ''
            reasons = item.get('skip_reasons') or {}
            if reasons:
                skip = skip + ' ' + json.dumps(reasons, ensure_ascii=False)
            lines.append(
                f"| {item.get('target_id')} | {item.get('captured_views_max')} | "
                f"{item.get('skipped_views_max')} | {item.get('last_state') or '—'} | "
                f"{(skip or '—').replace('|', '/')} |")
    else:
        lines.append('- （无逐目标诊断）')
    lines += ['', '## 逐目标作业票（感知→抓取）', '']
    if jobs:
        lines.append(
            '| target_id | 当前环节 | 抓取许可 | 原因/档位 | '
            '感知入口 | 重建中心 | 预抓取 | 抓取进入 |')
        lines.append('|---|---|---|---|---|---|---|---|')
        for item in jobs:
            grasp = item.get('grasp') or {}
            coords = item.get('coords') or {}
            flags = item.get('flags') or {}
            gate = []
            if not flags.get('grasp_enabled', True):
                gate.append('抓取档关')
            if not flags.get('tool_enabled', True):
                gate.append('工具档关')
            why = item.get('why') or ''
            if gate:
                why = ('、'.join(gate) + '；' + why).strip('；')
            lines.append(
                f"| {item.get('target_id')} | {item.get('active_id') or '—'} | "
                f"{'是' if grasp.get('allowed') else '否'} | "
                f"{(why or grasp.get('reason') or '—').replace('|', '/')} | "
                f"{_fmt_xyz(coords.get('perception_entry'))} | "
                f"{_fmt_xyz(coords.get('reconstruction_center'))} | "
                f"{_fmt_xyz(coords.get('grasp_pregrasp'))} | "
                f"{_fmt_xyz(coords.get('grasp_entry'))} |")
    else:
        lines.append('- （无作业票记录）')
    lines += ['', '## 运行性能统计', '',
              f"- 性能采样条数：{metrics.get('samples', 0)}"]
    for key, label in (('cpu_percent', 'CPU %'), ('memory_percent', '内存 %'),
                       ('gpu_utilization_percent', 'GPU 利用率 %'),
                       ('gpu_memory_used_mb', 'GPU 显存 MB')):
        item = metrics.get(key)
        if item:
            lines.append(f"- {label}：均值 {item['mean']} / 峰值 {item['max']}")
    lines += ['', '## 末端 TCP 轨迹', '']
    if tcp and tcp.get('samples'):
        ratio = tcp.get('detour_ratio')
        ratio_text = '—' if ratio is None else str(ratio)
        lines.append(f'- 采样点数：{tcp.get("samples")}')
        lines.append(f'- 路径长：{tcp.get("path_length_m")} m')
        lines.append(f'- 起止弦：{tcp.get("chord_m")} m')
        lines.append(f'- 绕行比（路径/弦）：{ratio_text}（直线≈1）')
        lines.append(f'- 相对弦最大偏离：{tcp.get("max_dev_m")} m')
        lines.append(
            f'- Z 范围：{tcp.get("z_min_m")} ~ {tcp.get("z_max_m")} m'
            f'（Δz={tcp.get("dz_m")}）')
    else:
        lines.append('- （无 tcp_trajectory.jsonl）')
    lines.append('')
    return '\n'.join(lines)


def _read_jsonl(path: Path) -> list[dict]:
    """读取一个 jsonl 文件为 dict 列表（不存在/坏行跳过）."""
    records = []
    if not path.exists():
        return records
    with open(path, encoding='utf-8') as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return records


class Recorder:
    """监控数据分类落盘：observability 回调入口 + 队列 + 守护写线程."""

    def __init__(self, root_dir='runs', enabled: bool = True,
                 save_images: bool = True, save_clouds: bool = True,
                 on_info=None, log_warning=lambda msg: None):
        """初始化记录器并启动写线程；on_info 在目录切换时回调."""
        self._root = Path(root_dir)
        self._enabled = bool(enabled)
        self._save_images = bool(save_images)
        self._save_clouds = bool(save_clouds)
        self._on_info = on_info or (lambda info: None)
        self._log_warning = log_warning
        self._lock = threading.RLock()
        self._queue = queue.Queue()
        self._stop = threading.Event()
        # 目录状态机：idle（无批次）/ run（批次执行期）
        self._active_kind = None
        self._active_dir = None
        self._batch_run_id = ''
        self._last_state_revision = None
        self._last_harvest_text = None
        self._last_job_key = None
        self._image_seq = 0
        self._raw_image_seq = 0
        self._last_image_name = None
        self._latest_cloud = None
        self._cv_unavailable = False
        self._thread = threading.Thread(
            target=self._writer_loop, name='peach-web-recorder', daemon=True)
        self._thread.start()
        if self._enabled:
            # 节点启动即开 idle 目录，承接首个批次开始前的记录
            self._enter_idle()
        self._on_info(self.info())

    # ------------------------------------------------------------------
    # observability 回调入口（全部只入队/换引用，绝不阻塞）
    # ------------------------------------------------------------------
    def info(self) -> dict:
        """当前记录状态（前端状态栏显示用）."""
        with self._lock:
            return {
                'enabled': self._enabled,
                'directory': (str(self._active_dir)
                              if self._active_dir else None),
            }

    def handle_event(self, event: dict) -> None:
        """事件全量入 events.jsonl；关键事件记图像索引，终局存点云."""
        if not self._enabled:
            return
        with self._lock:
            self._enqueue_jsonl('events.jsonl', event)
            code = event.get('code') or ''
            if code in IMAGE_EVENT_CODES:
                self._enqueue_jsonl('image_index.jsonl', {
                    'event': code,
                    'event_stamp': event.get('stamp'),
                    'target_id': event.get('target_id') or '',
                    'image': self._last_image_name,
                })
            if self._save_clouds and code in TERMINAL_TARGET_CODES:
                self._save_latest_cloud(event)

    def handle_state(self, state: dict) -> None:
        """调度状态：驱动批次目录开合，按 revision 去重入 state.jsonl."""
        if not self._enabled:
            return
        with self._lock:
            batch = state.get('batch_state')
            run_id = str(state.get('run_id') or '')
            if run_id and run_id != self._batch_run_id:
                self._batch_run_id = run_id
            if batch in ACTIVE_BATCH_STATES and self._active_kind != 'run':
                self._open_batch()
            if state.get('revision') != self._last_state_revision:
                self._last_state_revision = state.get('revision')
                self._enqueue_jsonl('state.jsonl', state)
            if batch in TERMINAL_BATCH_STATES and self._active_kind == 'run':
                self._close_batch(batch)

    def handle_harvest(self, harvest: dict) -> None:
        """感知采摘计划：内容变化时并入 perception.jsonl."""
        if not self._enabled:
            return
        text = json.dumps(harvest, ensure_ascii=False, sort_keys=True)
        with self._lock:
            if self._active_kind != 'run':
                return
            if text == self._last_harvest_text:
                return
            self._last_harvest_text = text
            self._enqueue_jsonl(
                'perception.jsonl', {'kind': 'harvest', 'data': harvest})

    def handle_targets(self, value: dict) -> None:
        """目标快照逐帧记录（剔除 mask，保留 candidate/fitting 摘要）."""
        if not self._enabled:
            return
        with self._lock:
            if self._active_kind != 'run':
                return
            self._enqueue_jsonl(
                'perception.jsonl',
                {'kind': 'targets', 'data': slim_targets(value)})

    def handle_reconstruction(self, topic: str, value: dict) -> None:
        """重建 status/diagnostics/grasp_decision 每次更新都记录."""
        if not self._enabled:
            return
        with self._lock:
            if self._active_kind != 'run':
                return
            self._enqueue_jsonl(
                'reconstruction.jsonl', {'topic': topic, 'data': value})

    def handle_manipulation(self, value: dict) -> None:
        """技能节点 status 每次更新都记录."""
        if not self._enabled:
            return
        with self._lock:
            self._enqueue_jsonl('manipulation.jsonl', {'data': value})

    def handle_hypothesis(self, value: dict) -> None:
        """技能抓取假设：变化写入 manipulation.jsonl."""
        if not self._enabled:
            return
        with self._lock:
            self._enqueue_jsonl(
                'manipulation.jsonl',
                {'topic': 'hypothesis', 'data': value})

    def handle_job(self, job: dict) -> None:
        """当前果实作业票：环节/档位/许可变化时写入 job.jsonl."""
        if not self._enabled or not job:
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
        with self._lock:
            if key == self._last_job_key:
                return
            self._last_job_key = key
            self._enqueue_jsonl('job.jsonl', job)

    def handle_metrics(self, sample: dict) -> None:
        """性能采样：每条一行入 metrics.jsonl."""
        if not self._enabled:
            return
        with self._lock:
            self._enqueue_jsonl('metrics.jsonl', sample)

    def handle_tcp(self, point: dict) -> None:
        """末端 TCP 采样点：位移过门槛后写入 tcp_trajectory.jsonl."""
        if not self._enabled or not point:
            return
        with self._lock:
            if self._active_dir is None:
                return
            self._enqueue_jsonl('tcp_trajectory.jsonl', point)

    def handle_image(self, message) -> None:
        """逐帧保存调试图 JPEG；文件名记序号供事件索引关联."""
        if not (self._enabled and self._save_images):
            return
        with self._lock:
            self._image_seq += 1
            name = f'img_{self._image_seq:05d}_{self._stamp_name(None)}.jpg'
            self._last_image_name = name
            self._enqueue(
                'image', message, self._active_dir / 'images' / name)

    def handle_raw_image(self, message) -> None:
        """真相流画布（筛选前）并行落盘：raw_ 前缀、独立计数、不碰事件索引."""
        if not (self._enabled and self._save_images):
            return
        with self._lock:
            self._raw_image_seq += 1
            name = (
                f'raw_img_{self._raw_image_seq:05d}_'
                f'{self._stamp_name(None)}.jpg')
            self._enqueue(
                'image', message, self._active_dir / 'images' / name)

    def handle_cloud(self, message) -> None:
        """缓存最新 TSDF 点云（只在 target 终局时落盘）."""
        if self._enabled and self._save_clouds:
            self._latest_cloud = message

    def close(self) -> None:
        """节点关闭：批次未结则补 summary，排空队列后停写线程."""
        if self._enabled:
            with self._lock:
                if self._active_kind == 'run' and self._active_dir is not None:
                    self._enqueue_summary(self._active_dir, None)
        self._queue.join()
        self._stop.set()
        self._thread.join(timeout=5.0)

    # ------------------------------------------------------------------
    # 内部：目录状态机与任务入队（调用时必须已持锁）
    # ------------------------------------------------------------------
    def _open_batch(self) -> None:
        """
        批次开始：关闭 idle，开批次目录.

        单根会话目录（R7）：已知批次 run_id（=request_id）时目录即
        ``runs/<request_id>/``——与账本、感知 datastore、重建 session
        同根（exist_ok：账本可能先建）；无 run_id 回退 run_<时间戳>。
        """
        self._active_kind = 'run'
        self._active_dir = (
            Path(self._root) / self._batch_run_id
            if self._batch_run_id else session_folder(self._root, 'run'))
        self._last_state_revision = None
        self._last_job_key = None
        self._enqueue('mkdir', self._active_dir)
        self._on_info(self.info())

    def _close_batch(self, batch_state) -> None:
        """批次终局：生成 summary 后回到新的 idle 目录."""
        self._enqueue_summary(self._active_dir, batch_state)
        self._enter_idle()

    def _enter_idle(self) -> None:
        """开新的 idle_<时间戳> 目录承接无批次记录."""
        self._active_kind = 'idle'
        self._active_dir = session_folder(self._root, 'idle')
        self._enqueue('mkdir', self._active_dir)
        self._on_info(self.info())

    def _enqueue(self, kind, *payload) -> None:
        self._queue.put((kind,) + payload)

    def _enqueue_jsonl(self, name: str, record: dict) -> None:
        """统一补墙钟时间戳后入 jsonl 写任务."""
        payload = {'recorded_at': round(time.time(), 3)}
        payload.update(record)
        self._enqueue('jsonl', self._active_dir / name, payload)

    def _enqueue_summary(self, run_dir: Path, batch_state) -> None:
        """把 summary 生成任务入队（统计全部从该目录 jsonl 现算）."""
        self._enqueue('summary', run_dir, batch_state)

    def _save_latest_cloud(self, event: dict) -> None:
        if self._latest_cloud is None:
            return
        target = _sanitize(event.get('target_id') or '')
        stamp = self._stamp_name(event.get('stamp'))
        self._enqueue('cloud', self._latest_cloud,
                      self._active_dir / 'clouds' / f'{target}_{stamp}.ply')

    @staticmethod
    def _stamp_name(stamp) -> str:
        """事件时间戳 → 文件名片段 YYYYmmdd_HHMMSS.mmm（空取当前时刻）."""
        try:
            value = float(stamp or 0.0)
        except (TypeError, ValueError):
            value = 0.0
        if value <= 0.0:
            value = time.time()
        base = time.strftime('%Y%m%d_%H%M%S', time.localtime(value))
        return f'{base}.{int((value % 1) * 1000):03d}'

    # ------------------------------------------------------------------
    # 写线程：唯一执行盘写的地方
    # ------------------------------------------------------------------
    def _writer_loop(self) -> None:
        """取任务执行；jsonl 先进路径缓冲，批量/空闲/退出前三处 flush."""
        pending = {}  # Path -> [record]，仅本线程访问

        def flush() -> None:
            """把缓冲记录按路径批量追加落盘；单文件失败告警丢弃，不影响其他."""
            for path, records in pending.items():
                try:
                    path.parent.mkdir(parents=True, exist_ok=True)
                    with open(path, 'a', encoding='utf-8') as stream:
                        for record in records:
                            stream.write(
                                json.dumps(record, ensure_ascii=False) + '\n')
                except Exception as error:  # 写盘失败只降级告警，不阻断采集
                    self._log_warning(
                        f'落盘失败（丢弃 {len(records)} 条）{path}: {error}')
            pending.clear()

        while not self._stop.is_set() or not self._queue.empty():
            try:
                job = self._queue.get(timeout=0.2)
            except queue.Empty:
                flush()  # 队列空闲一个轮询周期：间隔 flush 兜底可见时延
                continue
            try:
                if job[0] == 'jsonl':
                    pending.setdefault(job[1], []).append(job[2])
                    if sum(len(r) for r in pending.values()) >= \
                            _JSONL_BATCH_LINES:
                        flush()
                else:
                    # mkdir/summary/image/cloud 与既有 jsonl 记录有先后语义
                    # （summary 读本目录 jsonl 现算），执行前必须先 flush。
                    flush()
                    self._run_job(job)
            except Exception as error:  # 单个任务失败只告警，线程不死
                self._log_warning(f'记录任务失败（跳过）: {error}')
            finally:
                self._queue.task_done()
        flush()  # close() 排空队列后的最后一次 drain

    def _run_job(self, job) -> None:
        kind = job[0]
        if kind == 'mkdir':
            job[1].mkdir(parents=True, exist_ok=True)
        elif kind == 'image':
            _, message, path = job
            self._write_image(message, path)
        elif kind == 'cloud':
            _, message, path = job
            self._write_cloud(message, path)
        elif kind == 'summary':
            _, run_dir, batch_state = job
            self._write_summary(run_dir, batch_state)

    def _write_image(self, message, path: Path) -> None:
        """cv_bridge/cv2 延迟导入写 JPEG(q85)；不可用告警一次并跳过."""
        if self._cv_unavailable:
            return
        try:
            import cv2
            import cv_bridge
        except ImportError:
            self._cv_unavailable = True
            self._log_warning(
                'cv_bridge/cv2 不可用：图像保存整体跳过（仅告警一次）')
            return
        try:
            bridge = cv_bridge.CvBridge()
            try:
                image = bridge.imgmsg_to_cv2(message, 'bgr8')
            except Exception:  # 非 bgr8 编码时原样兜底
                image = bridge.imgmsg_to_cv2(message, 'passthrough')
            path.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(
                    str(path), image, [cv2.IMWRITE_JPEG_QUALITY, 85]):
                raise OSError('cv2.imwrite 返回失败')
        except Exception as error:
            self._log_warning(f'调试图保存失败 {path.name}: {error}')

    def _write_cloud(self, message, path: Path) -> None:
        """PointCloud2 → binary PLY；空云跳过."""
        parsed = xyz_rgb_from_pointcloud2(message)
        if parsed is None:
            return
        xyz, rgb = parsed
        path.parent.mkdir(parents=True, exist_ok=True)
        write_ply(path, xyz, rgb)

    def _write_summary(self, run_dir: Path, batch_state) -> None:
        """读本目录全部 jsonl 现算统计，写 summary.csv 与 summary.md."""
        run_dir.mkdir(parents=True, exist_ok=True)
        events = _read_jsonl(run_dir / 'events.jsonl')
        states = _read_jsonl(run_dir / 'state.jsonl')
        perception = _read_jsonl(run_dir / 'perception.jsonl')
        reconstruction = _read_jsonl(run_dir / 'reconstruction.jsonl')
        jobs = job_per_target(_read_jsonl(run_dir / 'job.jsonl'))
        metrics = _read_jsonl(run_dir / 'metrics.jsonl')
        tcp_records = _read_jsonl(run_dir / 'tcp_trajectory.jsonl')
        run_id = next(
            (item.get('run_id') for item in reversed(states)
             if item.get('run_id')),
            next((item.get('run_id') for item in reversed(events)
                  if item.get('run_id')), 'unknown'))
        priorities = {}
        for item in perception:
            if item.get('kind') != 'targets':
                continue
            for obs in (item.get('data') or {}).get('observations') or []:
                if obs.get('target_id'):
                    priorities[obs['target_id']] = obs.get('priority')
        rounds = [int(m.group(1)) for event in events
                  if event.get('code') in ('round_started', 'round_completed')
                  for m in [re.search(r'第\s*(\d+)\s*轮',
                                      event.get('message') or '')]
                  if m]
        stamps = [float(item.get('recorded_at') or 0.0)
                  for item in events + states if item.get('recorded_at')]
        rows = build_target_rows(events, priorities)
        recon_targets = reconstruction_per_target(reconstruction)
        # 配对账本提示：账本按 request_id 落 runs/<request_id>/，与监控
        # run 目录（run_<墙钟>）不同树；摘要头部互引避免人工对时间戳。
        request_hint = next(
            (item.get('request_id') for item in reversed(events)
             if item.get('request_id')), '')
        markdown = build_summary_markdown(
            run_id,
            min(stamps) if stamps else None,
            max(stamps) if stamps else None,
            batch_state, max(rounds) if rounds else 1, rows,
            phase_durations(states), event_statistics(events),
            perception_stats(perception), reconstruction_final(reconstruction),
            summarize_metrics(metrics), recon_targets, jobs,
            summarize_tcp(tcp_records), ledger_hint=request_hint)
        (run_dir / 'summary.csv').write_text(
            build_summary_csv(rows), encoding='utf-8')
        (run_dir / 'summary.md').write_text(markdown, encoding='utf-8')
