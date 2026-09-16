"""
bag 分析报告（纯核）+ ``peach_bag_report`` CLI.

输入 bag_reader 的 {topic: [(t_ns, dict|None)]} 流（或同构合成流，供无 ROS
单测直接构造），输出与会话 bag 同目录的 ``bag_report.md`` / ``bag_report.
json``：会话概览、验收门三行对照（口径沿用旧 summary.md / docs/testing.md
量化基线）、按 request_id 分批的逐目标 outcome 与阶段耗时、事件/感知/重建/
性能统计、逐目标作业票、TCP 轨迹（/tf 离线重算，沿用 3 mm 静止门槛）。

本模块只依赖标准库与 numpy；bag 读取（rosbag2_py）在 generate 层懒加载。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from .path_metrics import path_metrics

# 契约话题名（图名契约冻结；bag_reader 流键即话题名）
T_EVENTS = '/peach_supervisor/events'
T_STATE = '/peach_supervisor/state'
T_TARGETS = '/peach/perception/target_observations'
T_HARVEST = '/peach/perception/harvest_state'
T_RECON_STATUS = '/peach/reconstruction/status'
T_RECON_DIAG = '/peach/reconstruction/diagnostics'
T_RECON_DEBUG = '/peach/reconstruction/diagnostics_debug'
T_DECISION = '/peach/reconstruction/grasp_decision'
T_JOB = '/peach/observability/job'
T_METRICS = '/peach/observability/metrics'
T_TF = '/tf'
T_TF_STATIC = '/tf_static'

# target 终局事件码（进入逐目标 outcome 表；词表与 harvest_fsm 一致）
TERMINAL_TARGET_CODES = {
    'target_succeeded', 'target_skipped', 'target_failed',
    'target_rejected', 'target_canceled', 'target_operator_skipped',
}
# HarvestState.batch_state 枚举名（与 peach_interfaces/HarvestState.msg 一致）
BATCH_STATE_NAMES = {
    0: 'WAITING_READY', 1: 'DISCOVERY', 2: 'RUNNING', 3: 'PAUSE_PENDING',
    4: 'PAUSED', 5: 'MAINTENANCE', 6: 'COMPLETED', 7: 'RECOVERY_REQUIRED',
    8: 'INTERRUPTED', 9: 'NAVIGATING',
}
TERMINAL_BATCH_STATES = {6, 8}
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
        if value:
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
    从状态流的 target_phase 跃迁推导逐周期各阶段耗时.

    相邻两条状态记录的 recorded_at 差值记到前一条的 target_phase 上，按
    (cycle_id, target_id) 聚合计入对应工作阶段；最后一条的阶段无后继，
    不计（尾部截断误差）。
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
    """目标快照流 → 帧率估算（帧间隔中位数）与目标数变化范围."""
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
    """重建诊断流最后一条的关键指标终值."""
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
    """作业票流 → 逐目标最后一张作业票（过程线终态 + 坐标）."""
    by_id = {}
    for item in records:
        job = item if 'stages' in item else (item.get('data') or item)
        target_id = str(job.get('target_id') or '')
        if not target_id:
            continue
        by_id[target_id] = job
    return list(by_id.values())


def reconstruction_per_target(records: list[dict]) -> list[dict]:
    """重建诊断流 → 逐目标最大视角数与门禁跳过码."""
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


def summarize_metrics(records: list[dict]) -> dict:
    """性能采样记录列表 → CPU/内存/GPU 均值与峰值统计."""
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
    """TCP 轨迹点 → 路径长/弦长/绕行比（与网页同源算法）."""
    xyz = []
    for item in records:
        try:
            xyz.append((float(item['x']), float(item['y']), float(item['z'])))
        except (KeyError, TypeError, ValueError):
            continue
    metrics = path_metrics(xyz)
    metrics['samples'] = len(records)
    return metrics


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


# ----------------------------------------------------------------------
# bag 流 → 旧 jsonl 记录形状的适配层（报告 builders 全部复用旧口径）
# ----------------------------------------------------------------------
def _events_records(streams: dict) -> list[dict]:
    """事件流 → 旧 events.jsonl 记录形状（recorded_at=bag 接收时刻）."""
    return [dict(recorded_at=round(t_ns / 1e9, 3), **(value or {}))
            for t_ns, value in (streams.get(T_EVENTS) or []) if value]


def _state_records(streams: dict) -> list[dict]:
    """状态流 → 旧 state.jsonl 记录形状."""
    return [dict(recorded_at=round(t_ns / 1e9, 3), **(value or {}))
            for t_ns, value in (streams.get(T_STATE) or []) if value]


def _targets_records(streams: dict) -> list[dict]:
    """目标快照流 → 旧 perception.jsonl 的 targets 记录形状."""
    return [{'recorded_at': round(t_ns / 1e9, 3), 'kind': 'targets',
             'data': value}
            for t_ns, value in (streams.get(T_TARGETS) or []) if value]


def _json_string_records(stream: list) -> list[dict]:
    """std_msgs/String JSON 流 → 解析后记录（坏 JSON 跳过）."""
    out = []
    for t_ns, value in stream or []:
        if not value:
            continue
        text = value.get('data') if isinstance(value, dict) else None
        try:
            payload = json.loads(text) if isinstance(text, str) else text
        except (json.JSONDecodeError, TypeError):
            continue
        if isinstance(payload, dict):
            out.append({'recorded_at': round(t_ns / 1e9, 3), 'data': payload})
    return out


def merge_reconstruction_streams(streams: dict) -> list[dict]:
    """
    status/diagnostics/diagnostics_debug/grasp_decision 四路原始流 → 旧合并体.

    与 observability_node 镜像合并逻辑一致（2026-09-15 起 bag 只录原始
    话题）：类型化诊断并入调试明细缓存与最近许可镜像，status 单独成路。
    """
    timeline = []
    for t_ns, value in (streams.get(T_RECON_STATUS) or []):
        timeline.append((t_ns, 'status', value))
    for t_ns, value in (streams.get(T_RECON_DIAG) or []):
        timeline.append((t_ns, 'diagnostics', value))
    for t_ns, value in (streams.get(T_RECON_DEBUG) or []):
        timeline.append((t_ns, 'debug', value))
    for t_ns, value in (streams.get(T_DECISION) or []):
        timeline.append((t_ns, 'grasp_decision', value))
    timeline.sort(key=lambda item: item[0])
    records: list[dict] = []
    debug_extra: dict = {}
    decision_value: dict | None = None
    for t_ns, topic, value in timeline:
        stamp = round(t_ns / 1e9, 3)
        if topic == 'debug':
            if isinstance(value, dict):
                debug_extra = value
            continue
        if topic == 'grasp_decision':
            decision_value = value if isinstance(value, dict) else None
            records.append(
                {'recorded_at': stamp, 'topic': topic,
                 'data': decision_value or {}})
            continue
        if topic == 'status':
            payload = _json_string_records([(t_ns, value)])
            data = payload[0]['data'] if payload else {}
            records.append({'recorded_at': stamp, 'topic': 'status',
                            'data': data})
            continue
        merged = dict(debug_extra)
        merged.update(value or {})
        if decision_value is not None:
            merged['grasp_decision'] = decision_value
        records.append({'recorded_at': stamp, 'topic': 'diagnostics',
                        'data': merged})
    return records


def _quat_mul(a, b):
    """四元数乘法（xyzw 顺序）."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _quat_rotate(q, v):
    """四元数旋转向量 v（xyzw 顺序）."""
    qv = np.array([q[0], q[1], q[2]])
    w = q[3]
    return (v + 2.0 * np.cross(qv, np.cross(qv, v) + w * v))


def _compose_chain(edges, base_frame: str, tip_frame: str):
    """
    沿边表 BFS 合成 base→tip 位姿；不通给 None.

    edges: {(parent, child): (xyz(3,), quat(xyzw,))}，取各边最新值。
    """
    children = {}
    for (parent, child) in edges:
        children.setdefault(parent, []).append(child)
    path = []
    stack = [(base_frame, [base_frame])]
    while stack:
        node, trail = stack.pop()
        if node == tip_frame:
            path = trail
            break
        for child in children.get(node) or []:
            stack.append((child, trail + [child]))
    if not path:
        return None
    trans = np.zeros(3)
    rot = np.array([0.0, 0.0, 0.0, 1.0])
    for parent, child in zip(path, path[1:]):
        p_xyz, p_rot = edges[(parent, child)]
        trans = trans + _quat_rotate(rot, p_xyz)
        rot = _quat_mul(rot, p_rot)
    norm = float(np.linalg.norm(rot))
    if norm < 1.0e-9:
        return None
    return trans, rot / norm


def tcp_points_from_tf(streams: dict, base_frame: str = 'base_link',
                       tip_frame: str = 'tcp',
                       min_step_m: float = 0.003) -> list[dict]:
    """
    /tf + /tf_static 树合成 → TCP 轨迹点（3mm 静止门槛，时间升序）.

    与在线采样器同口径（latest TF）：每条动态 TF 批次后尝试合成一次，
    静态边长期有效；合成路径经 BFS 动态求取，臂链+定链（URDF fixed）
    都能还原。
    """
    timeline = [(t_ns, True, value)
                for t_ns, value in (streams.get(T_TF_STATIC) or [])]
    timeline += [(t_ns, False, value)
                 for t_ns, value in (streams.get(T_TF) or [])]
    timeline.sort(key=lambda item: item[0])

    static_edges: dict = {}
    dynamic_edges: dict = {}
    points: list[dict] = []
    last_xyz = None
    for t_ns, is_static, value in timeline:
        if not value:
            continue
        edges = static_edges if is_static else dynamic_edges
        for transform in value.get('transforms') or []:
            parent = str((transform.get('header') or {}).get('frame_id') or '')
            child = str(transform.get('child_frame_id') or '')
            tf = transform.get('transform') or {}
            trans = tf.get('translation') or {}
            rot = tf.get('rotation') or {}
            edges[(parent, child)] = (
                np.array([float(trans.get('x') or 0.0),
                          float(trans.get('y') or 0.0),
                          float(trans.get('z') or 0.0)]),
                np.array([float(rot.get('x') or 0.0),
                          float(rot.get('y') or 0.0),
                          float(rot.get('z') or 0.0),
                          float(rot.get('w') or 1.0)]))
        if is_static:
            continue
        composed = _compose_chain(
            {**static_edges, **dynamic_edges}, base_frame, tip_frame)
        if composed is None:
            continue
        xyz, quat = composed
        if last_xyz is not None and float(
                np.linalg.norm(xyz - last_xyz)) < min_step_m:
            continue
        points.append({
            't': t_ns / 1e9,
            'x': float(xyz[0]), 'y': float(xyz[1]), 'z': float(xyz[2]),
            'qx': float(quat[0]), 'qy': float(quat[1]),
            'qz': float(quat[2]), 'qw': float(quat[3]),
        })
        last_xyz = xyz
    return points


def annotate_tf(points: list[dict], states: list[dict]) -> list[dict]:
    """按时间把状态的 target_phase/target_id 贴到轨迹点（返回新表）."""
    annotated = []
    index = 0
    for point in points:
        while (index + 1 < len(states)
               and float(states[index + 1].get('recorded_at') or 0.0)
               <= float(point.get('t') or 0.0)):
            index += 1
        row = dict(point)
        if states:
            row['target_id'] = str(states[index].get('target_id') or '')
            row['phase'] = int(states[index].get('target_phase') or 0)
        annotated.append(row)
    return annotated


# ----------------------------------------------------------------------
# 会话报告组装与渲染
# ----------------------------------------------------------------------
def _outcome_counts(rows: list[dict]) -> dict:
    """逐目标 outcome 计数."""
    counts = {}
    for row in rows:
        counts[row['outcome']] = counts.get(row['outcome'], 0) + 1
    return counts


def _gate_rows(perception: dict, recon: dict, rows: list[dict]) -> list[list]:
    """验收门三行（口径沿用旧 summary / docs/testing.md 量化基线）."""
    fps = perception.get('fps') if perception else None
    fps_row = ['感知帧率 FPS', fps if fps is not None else '—', '≥ 2.0',
               '—' if fps is None else ('✓' if fps >= 2.0 else '✗')]
    tf_failures = (recon or {}).get('tf_failures')
    tf_row = ['重建 TF 失败次数',
              tf_failures if tf_failures is not None else '—', '= 0',
              '—' if tf_failures is None else
              ('✓' if tf_failures == 0 else '✗')]
    counts = _outcome_counts(rows)
    attempted = sum(
        counts.get(key, 0)
        for key in ('succeeded', 'skipped', 'failed', 'canceled'))
    reached = counts.get('succeeded', 0)
    gate_text = '—' if attempted == 0 else ('✓' if reached > 0 else '✗')
    return [fps_row, tf_row,
            ['到预抓取停住（succeeded）', reached, '≥1（有派发目标时）',
             gate_text]]


def _ledger_paths(request_ids, ledger_root) -> dict:
    """request_id → 存在的 ledger.json 路径（报告只提示，不改账本）."""
    found = {}
    root = Path(ledger_root) if ledger_root else None
    if root is None or not root.is_dir():
        return found
    for request_id in request_ids:
        safe = str(request_id or '').strip()
        if not safe or any(token in safe for token in ('/', '\\', '..')):
            continue
        candidate = root / safe / 'ledger.json'
        if candidate.exists():
            found[safe] = str(candidate)
    return found


def build_session_report(streams: dict, *, bag_dir: str = '',
                         session_dir: str = '', base_frame: str = 'base_link',
                         tip_frame: str = 'tcp',
                         ledger_root=None) -> tuple[dict, str]:
    """输入 bag 流 → (报告 dict, markdown 文本)；全部统计现算，无增量状态."""
    events = _events_records(streams)
    states = _state_records(streams)
    targets = _targets_records(streams)
    reconstruction = merge_reconstruction_streams(streams)
    jobs_records = [
        item['data'] for item in _json_string_records(streams.get(T_JOB) or [])]
    metrics_records = [
        item['data'] for item in _json_string_records(streams.get(T_METRICS) or [])]

    priorities = {}
    for item in targets:
        for obs in (item.get('data') or {}).get('observations') or []:
            if obs.get('target_id'):
                priorities[obs['target_id']] = obs.get('priority')

    rows = build_target_rows(events, priorities)
    phases = phase_durations(states)
    event_stats = event_statistics(events)
    perception = perception_stats(targets)
    recon = reconstruction_final(reconstruction)
    recon_targets = reconstruction_per_target(reconstruction)
    jobs = job_per_target(jobs_records)
    metrics = summarize_metrics(metrics_records)
    tcp_points = annotate_tf(
        tcp_points_from_tf(streams, base_frame, tip_frame), states)
    tcp = summarize_tcp(tcp_points)
    gates = _gate_rows(perception, recon, rows)

    stamps = [float(item.get('recorded_at') or 0.0)
              for item in events + states if item.get('recorded_at')]
    # 并入全流时间戳：会话墙钟区间以 bag 实际覆盖为准（空批会话也有时长）
    stamps += [t_ns / 1e9
               for records in (streams or {}).values()
               for t_ns, _ in records]
    started = min(stamps) if stamps else None
    ended = max(stamps) if stamps else None
    duration = (round(ended - started, 1)
                if started and ended and ended > started else None)

    # 分批：events 按 request_id 分组；批次终局取该 run_id 最后一条状态
    request_ids = []
    for event in events:
        request_id = str(event.get('request_id') or '')
        if request_id and request_id not in request_ids:
            request_ids.append(request_id)
    last_state_by_run = {}
    for state in states:
        run_id = str(state.get('run_id') or '')
        if run_id:
            last_state_by_run[run_id] = state
    ledgers = _ledger_paths(request_ids + list(last_state_by_run), ledger_root)
    requests = []
    for request_id in request_ids:
        sub_events = [item for item in events
                      if str(item.get('request_id') or '') == request_id]
        sub_rows = build_target_rows(sub_events, priorities)
        state = last_state_by_run.get(request_id) or {}
        requests.append({
            'request_id': request_id,
            'started_at': min(
                (float(item.get('recorded_at') or 0.0) for item in sub_events
                 if item.get('recorded_at')), default=None) or None,
            'ended_at': max(
                (float(item.get('recorded_at') or 0.0) for item in sub_events
                 if item.get('recorded_at')), default=None) or None,
            'batch_state': BATCH_STATE_NAMES.get(
                int(state.get('batch_state') or -1), '未见状态'),
            'terminal': int(state.get('batch_state') or -1)
            in TERMINAL_BATCH_STATES,
            'outcome_counts': _outcome_counts(sub_rows),
            'targets': sub_rows,
            'ledger': ledgers.get(request_id),
        })

    report = {
        'session': {
            'bag': str(bag_dir), 'session_dir': str(session_dir),
            'started_at': started, 'ended_at': ended, 'duration_s': duration,
        },
        'requests': requests,
        'gates': {
            'rows': gates,
            'fps': perception.get('fps'),
            'tf_failures': recon.get('tf_failures'),
            'succeeded': _outcome_counts(rows).get('succeeded', 0),
        },
        'targets': rows,
        'phase_durations': phases,
        'events': event_stats,
        'perception': perception,
        'reconstruction': recon,
        'reconstruction_per_target': recon_targets,
        'jobs': jobs,
        'metrics': metrics,
        'tcp': tcp,
    }
    return report, render_markdown(report)


def render_markdown(report: dict) -> str:
    """报告 dict → markdown（表格式样沿用旧 summary.md）."""
    session = report.get('session') or {}
    lines = [
        f"# 采摘会话分析 `{Path(session.get('session_dir') or 'bag').name}`",
        '', '## 会话概览', '',
        f"- bag：`{session.get('bag') or '—'}`",
        f"- 开始时间：{_clock(session.get('started_at'))}",
        f"- 结束时间：{_clock(session.get('ended_at'))}",
        ('- 总时长：' + str(session.get('duration_s')) + ' s')
        if session.get('duration_s') is not None else '- 总时长：—',
        '', '## 批次（request）一览', '',
        '| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |',
        '|---|---|---|---|---|---|',
    ]
    requests = report.get('requests') or []
    if not requests:
        lines.append('| — | — | — | — | — | — |')
    for item in requests:
        counts = '，'.join(
            f'{key}={value}' for key, value
            in sorted((item.get('outcome_counts') or {}).items())) or '无'
        lines.append(
            f"| {item.get('request_id')} | {item.get('batch_state')} | "
            f"{_clock(item.get('started_at'))} | {_clock(item.get('ended_at'))} | "
            f'{counts} | '
            f"{'`' + item['ledger'] + '`' if item.get('ledger') else '—'} |")
    lines += [
        '', '## 验收门对照（口径见 docs/testing.md 量化基线）', '',
        '| 指标 | 实测 | 门 | 判定 |', '|---|---:|---|---|',
    ]
    for name, actual, gate, verdict in report['gates']['rows']:
        lines.append(f'| {name} | {actual} | {gate} | {verdict} |')
    lines += ['', '## 逐批次逐目标 outcome', '']
    written = 0
    for item in requests:
        sub_rows = item.get('targets') or []
        if not sub_rows:
            continue
        written += 1
        lines += [f"### {item.get('request_id')}", '',
                  '| target_id | 优先级 | outcome | 耗时 s | 原因 |',
                  '|---|---:|---|---:|---|']
        for row in sub_rows:
            lines.append(
                f"| {row.get('target_id')} | {row.get('priority') or '—'} | "
                f"{row.get('outcome')} | "
                f"{row.get('duration_s') if row.get('duration_s') is not None else '—'} | "
                f"{(row.get('reason') or '').replace('|', '/')} |")
        lines.append('')
    if not written:
        lines.append('- （无派发目标）')
    lines += ['', '## 每阶段耗时统计（按目标周期）', '',
              '| 周期 | 目标 | ' + ' | '.join(_WORKING_PHASES) + ' | 合计 s |',
              '|---|---|' + '---:|' * (len(_WORKING_PHASES) + 1)]
    written = 0
    for item in report.get('phase_durations') or []:
        if float(item.get('total_s') or 0) <= 0.0:
            continue
        written += 1
        cells = [str(item['phases'].get(name, '—')) for name in _WORKING_PHASES]
        lines.append(
            f"| {item['cycle_id'] or '—'} | {item['target_id'] or '—'} | "
            + ' | '.join(cells) + f" | {item['total_s']} |")
    if not written:
        lines.append('| — | — | ' + ' | '.join(['—'] * len(_WORKING_PHASES)) + ' | — |')
    event_stats = report.get('events') or {}
    lines += ['', '## 事件统计', '', f"- 事件总数：{event_stats.get('total', 0)}"]
    for severity, count in sorted((event_stats.get('by_severity') or {}).items()):
        lines.append(f'- severity {severity}：{count}')
    for code, count in sorted((event_stats.get('by_code') or {}).items()):
        lines.append(f'- `{code}`：{count}')
    perception = report.get('perception') or {}
    lines += ['', '## 感知统计', '',
              f"- 记录帧数：{perception.get('frames', 0)}",
              f"- 帧间隔中位数：{perception.get('median_interval_s')} s"
              f"（≈{perception.get('fps')} FPS）",
              f"- 目标数范围：{perception.get('target_count_min')}"
              f" ~ {perception.get('target_count_max')}",
              '', '## 重建关键指标终值', '']
    recon = report.get('reconstruction') or {}
    if recon:
        for key, value in recon.items():
            lines.append(f'- {key}：{value}')
    else:
        lines.append('- （无重建诊断记录）')
    lines += ['', '## 逐目标重建视角', '']
    recon_targets = report.get('reconstruction_per_target') or []
    if recon_targets:
        lines += [
            '| target_id | captured_views_max | skipped_views_max | '
            'last_state | last_skip |', '|---|---:|---:|---|---|']
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
    jobs = report.get('jobs') or []
    if jobs:
        lines += [
            '| target_id | 当前环节 | 抓取许可 | 原因/档位 | '
            '感知入口 | 重建中心 | 预抓取 | 抓取进入 |',
            '|---|---|---|---|---|---|---|---|']
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
    metrics = report.get('metrics') or {}
    lines += ['', '## 运行性能统计', '',
              f"- 性能采样条数：{metrics.get('samples', 0)}"]
    for key, label in (('cpu_percent', 'CPU %'), ('memory_percent', '内存 %'),
                       ('gpu_utilization_percent', 'GPU 利用率 %'),
                       ('gpu_memory_used_mb', 'GPU 显存 MB')):
        item = metrics.get(key)
        if item:
            lines.append(f"- {label}：均值 {item['mean']} / 峰值 {item['max']}")
    tcp = report.get('tcp') or {}
    lines += ['', '## 末端 TCP 轨迹（/tf 重算）', '']
    if tcp.get('samples'):
        ratio = tcp.get('detour_ratio')
        lines += [
            f"- 采样点数：{tcp.get('samples')}",
            f"- 路径长：{tcp.get('path_length_m')} m",
            f"- 起止弦：{tcp.get('chord_m')} m",
            f"- 绕行比（路径/弦）：{'—' if ratio is None else ratio}（直线≈1）",
            f"- 相对弦最大偏离：{tcp.get('max_dev_m')} m",
            f"- Z 范围：{tcp.get('z_min_m')} ~ {tcp.get('z_max_m')} m"
            f"（Δz={tcp.get('dz_m')}）"]
    else:
        lines.append('- （bag 中无 base→tcp 的 TF，检查 /tf 录制别名）')
    lines.append('')
    return '\n'.join(lines)


def write_report(report: dict, markdown: str, out_dir) -> tuple[Path, Path]:
    """写 bag_report.md 与 bag_report.json，返回两文件路径."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    md_path = out / 'bag_report.md'
    json_path = out / 'bag_report.json'
    md_path.write_text(markdown, encoding='utf-8')
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    return md_path, json_path


def generate_session_report(bag_dir, *, out_dir=None,
                            base_frame: str = 'base_link',
                            tip_frame: str = 'tcp', ledger_root=None,
                            max_total_bag_gb: float = 0.0, keep=(),
                            log=lambda msg: print(msg)) -> dict:
    """
    读 bag → 出报告（→ 体积回收）；自动报告与 CLI 共用本入口.

    rosbag2 相关导入在 bag_reader 内懒加载；报告写 out_dir（默认 bag 的
    上一级，即会话目录）。max_total_bag_gb>0 时报告完成后跑一次回收
    （keep 中的目录不删，audit 落 runs/retention_audit.jsonl）。
    """
    from . import bag_reader
    from . import retention

    bag_path = Path(bag_dir)
    out = Path(out_dir) if out_dir else bag_path.parent
    streams = bag_reader.read_bag(bag_path)
    report, markdown = build_session_report(
        streams, bag_dir=str(bag_path), session_dir=str(out),
        base_frame=base_frame, tip_frame=tip_frame, ledger_root=ledger_root)
    md_path, json_path = write_report(report, markdown, out)
    log(f'bag 分析报告已生成：{md_path}')
    deleted = []
    if max_total_bag_gb > 0:
        runs_root = out.parent if out.name.startswith('session_') else out
        deleted = retention.sweep(
            runs_root, max_total_bag_gb, keep=[out],
            log_warning=log)
    return {
        'markdown': str(md_path), 'json': str(json_path),
        'deleted_bags': [str(item.path) for item in deleted],
    }


def main(argv=None) -> int:
    """CLI：``peach_bag_report <bag 目录> [--out 目录] [--base-frame …]``."""
    parser = argparse.ArgumentParser(
        description='peach 会话 bag 分析报告（bag_report.md/json）')
    parser.add_argument('bag', help='bag 目录（runs/session_*/bag 或旧 mcap_*）')
    parser.add_argument('--out', default='', help='报告输出目录（默认 bag 上一级）')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--tip-frame', default='tcp')
    parser.add_argument('--ledger-root', default='', help='账本根（默认 bag 上一级的上一级）')
    args = parser.parse_args(argv)
    bag_path = Path(args.bag)
    ledger_root = args.ledger_root or str(bag_path.parent.parent)
    try:
        result = generate_session_report(
            bag_path, out_dir=args.out or None,
            base_frame=args.base_frame, tip_frame=args.tip_frame,
            ledger_root=ledger_root)
    except Exception as error:  # noqa: BLE001 CLI 顶层兜底
        print(f'报告生成失败: {error}')
        return 1
    print(f"markdown: {result['markdown']}")
    print(f"json: {result['json']}")
    return 0
