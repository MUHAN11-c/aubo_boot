# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/BSD-3-Clause
"""
监控数据分类落盘记录器（只读：订阅 + 写文件，无任何 ROS 写入口）.

目录归属按「编排器批次执行期」划分：batch_state 进入 DISCOVERY/RUNNING
（或节点启动后见到的首个活动批次）开 `run_<时间戳>/`，终局
（COMPLETED/INTERRUPTED/FAULT/RECOVERY_REQUIRED）关闭并生成 summary；
同一批次的所有复扫轮次不拆目录；无批次期间的记录进 `idle_<时间戳>/`
（节点启动时创建，批次开始后关闭，批次结束后再开新的）。

纯函数部分（目录命名、PointCloud2→PLY、summary 各项统计）零 ROS 依赖，
可直接单测；Recorder 只被 gateway 回调调用，所有盘写经队列 + 独立守护
线程执行，不阻塞 ROS 回调与 HTTP 线程。

目录结构（root 默认 web_runs/，相对节点 CWD）：

    <root>/run_<yyyyMMdd_HHmmss>/  # 一次批次一个文件夹
        events.jsonl               # HarvestEvent 全量，一行一条
        state.jsonl                # HarvestState 按 revision 去重
        perception.jsonl           # 感知目标快照逐帧 + harvest_state 变化并入
        reconstruction.jsonl       # 重建 status/diagnostics/grasp_decision 每次更新
        approach.jsonl             # 靠近抓取 status 每次更新
        metrics.jsonl              # 每个性能采样一条
        images/                    # 逐帧调试图 JPEG + image_index.jsonl 事件关联
        clouds/                    # target 终局时刻的 TSDF 点云 PLY
        summary.csv / summary.md   # 批次关闭时生成的统计摘要
"""

from __future__ import annotations

import csv
import io
import json
from pathlib import Path
import queue
import re
import threading
import time

import numpy as np

# target 终局事件码（触发点云保存与图像索引，并进入逐目标 outcome 表）
TERMINAL_TARGET_CODES = {
    'target_succeeded', 'target_skipped', 'target_failed',
    'target_rejected', 'target_canceled',
}
# 需要在 image_index.jsonl 里关联最近调试图的事件码
IMAGE_EVENT_CODES = {'round_locked', 'photo_pose_reached'} | TERMINAL_TARGET_CODES
# HarvestState batch_state：1 DISCOVERY / 2 RUNNING / 3 PAUSE_PENDING /
# 4 PAUSED / 5 MAINTENANCE 都算批次活动期
ACTIVE_BATCH_STATES = {1, 2, 3, 4, 5}
# 终局：6 COMPLETED / 7 FAULT / 8 RECOVERY_REQUIRED / 9 INTERRUPTED
TERMINAL_BATCH_STATES = {6, 7, 8, 9}
_BATCH_STATE_NAMES = {
    6: 'COMPLETED', 7: 'FAULT', 8: 'RECOVERY_REQUIRED', 9: 'INTERRUPTED',
}
# target_phase 枚举名（与 peach_harvest_msgs/HarvestState.msg 一致）
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
        'comment peach_perception_web recorder',
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
            row['reason'] = event.get('message') or ''
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
    }
    return {key: value for key, value in result.items() if value is not None}


def _clock(stamp: float | None) -> str:
    """把 epoch 秒折成本地可读时刻；空给 —."""
    if not stamp:
        return '—'
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(stamp))


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
                           metrics: dict) -> str:
    """批次概览 + 逐目标/逐阶段耗时 + 事件/感知/重建/性能统计 → Markdown."""
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
    for item in phases:
        cells = [str(item['phases'].get(name, '—')) for name in _WORKING_PHASES]
        lines.append(
            f"| {item['cycle_id'] or '—'} | {item['target_id'] or '—'} | "
            + ' | '.join(cells) + f" | {item['total_s']} |")
    if not phases:
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
    lines += ['', '## 运行性能统计', '',
              f"- 性能采样条数：{metrics.get('samples', 0)}"]
    for key, label in (('cpu_percent', 'CPU %'), ('memory_percent', '内存 %'),
                       ('gpu_utilization_percent', 'GPU 利用率 %'),
                       ('gpu_memory_used_mb', 'GPU 显存 MB')):
        item = metrics.get(key)
        if item:
            lines.append(f"- {label}：均值 {item['mean']} / 峰值 {item['max']}")
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
    """监控数据分类落盘：gateway 回调入口 + 队列 + 守护写线程."""

    def __init__(self, root_dir='web_runs', enabled: bool = True,
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
        self._last_state_revision = None
        self._last_harvest_text = None
        self._image_seq = 0
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
    # gateway 回调入口（全部只入队/换引用，绝不阻塞）
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
        """编排器状态：驱动批次目录开合，按 revision 去重入 state.jsonl."""
        if not self._enabled:
            return
        with self._lock:
            batch = state.get('batch_state')
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
            self._enqueue_jsonl(
                'perception.jsonl',
                {'kind': 'targets', 'data': slim_targets(value)})

    def handle_reconstruction(self, topic: str, value: dict) -> None:
        """重建 status/diagnostics/grasp_decision 每次更新都记录."""
        if not self._enabled:
            return
        with self._lock:
            self._enqueue_jsonl(
                'reconstruction.jsonl', {'topic': topic, 'data': value})

    def handle_approach(self, value: dict) -> None:
        """靠近抓取 status 每次更新都记录."""
        if not self._enabled:
            return
        with self._lock:
            self._enqueue_jsonl('approach.jsonl', {'data': value})

    def handle_metrics(self, sample: dict) -> None:
        """性能采样：每条一行入 metrics.jsonl."""
        if not self._enabled:
            return
        with self._lock:
            self._enqueue_jsonl('metrics.jsonl', sample)

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
        """批次开始：关闭 idle，开 run_<时间戳> 目录."""
        self._active_kind = 'run'
        self._active_dir = session_folder(self._root, 'run')
        self._last_state_revision = None
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
        while not self._stop.is_set() or not self._queue.empty():
            try:
                job = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._run_job(job)
            except Exception as error:  # 单个任务失败只告警，线程不死
                self._log_warning(f'记录任务失败（跳过）: {error}')
            finally:
                self._queue.task_done()

    def _run_job(self, job) -> None:
        kind = job[0]
        if kind == 'mkdir':
            job[1].mkdir(parents=True, exist_ok=True)
        elif kind == 'jsonl':
            _, path, record = job
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'a', encoding='utf-8') as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + '\n')
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
        metrics = _read_jsonl(run_dir / 'metrics.jsonl')
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
        markdown = build_summary_markdown(
            run_id,
            min(stamps) if stamps else None,
            max(stamps) if stamps else None,
            batch_state, max(rounds) if rounds else 1, rows,
            phase_durations(states), event_statistics(events),
            perception_stats(perception), reconstruction_final(reconstruction),
            summarize_metrics(metrics))
        (run_dir / 'summary.csv').write_text(
            build_summary_csv(rows), encoding='utf-8')
        (run_dir / 'summary.md').write_text(markdown, encoding='utf-8')
