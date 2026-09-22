#!/usr/bin/env python3
"""P1 感知稳定指标离线复算（战役 README「验收门 P1」）.

用法:
  stability_metrics.py <request_id>                     # 事件流指标
  stability_metrics.py <request_id> --bag <session_bag_dir>  # +几何 3σ（回放 bag）

事件流指标：锁定时延、selected ID 切换、帧缺口（>2s）、frame_skipped 原因分布。
bag 指标：逐 target entry/bottom/neck 3σ [mm]、轴角 3σ [deg]、袋长 3σ [mm]
（订 /peach/perception/target_observations 的 MCAP 回放，须已 source overlay）。
结果写 campaign/20260922_dual_tool/analysis/<rid>/stability_metrics.json。
"""
from __future__ import annotations

import collections
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNS = ROOT / 'runs'
OUTER = ROOT / 'campaign' / '20260922_dual_tool' / 'analysis'

FRAME_GAP_MAX_S = 2.0
# P1 门（percipio ≤8s / stereo ≤2s；stereo 前端由 --frontend stereo 区分）
LOCK_LATENCY_LIMIT_S = {'percipio': 8.0, 'stereo': 2.0}


def load_events(rid: str) -> list[dict]:
    out: list[dict] = []
    for events in sorted((RUNS / rid).glob('perception_data/harvest_*/events.jsonl')):
        for line in events.read_text(encoding='utf-8').splitlines():
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    return out


def event_metrics(events: list[dict]) -> dict:
    frames = [e for e in events if e.get('event') == 'frame_observations']
    locks = [e for e in events if e.get('event') == 'global_targets_locked']
    skips = [e for e in events if e.get('event') == 'frame_skipped']
    drops = [e for e in events if e.get('event') == 'target_dropped']
    res: dict = {
        'frames': len(frames), 'locks': len(locks),
        'frame_skipped': len(skips), 'target_dropped': len(drops),
    }
    skip_reasons = collections.Counter(
        str(e.get('reason') or e.get('code')) for e in skips)
    res['skip_reasons'] = dict(skip_reasons)
    # 锁定时延：首帧观测 → 锁定事件（墙钟 UTC；无锁定则 None）
    if frames and locks:
        def _t(rec: str) -> float:
            from datetime import datetime
            return datetime.fromisoformat(rec).timestamp()
        t0 = _t(frames[0]['recorded_at'])
        t1 = _t(locks[0]['recorded_at'])
        res['lock_latency_s'] = round(max(0.0, t1 - t0), 3)
    else:
        res['lock_latency_s'] = None
    # selected ID 切换（锁定后窗口内）
    switches = 0
    last = None
    for e in frames:
        sel = e.get('selected_target_id')
        if sel is None:
            continue
        if last is not None and sel != last:
            switches += 1
        last = sel
    res['selected_id_switches'] = switches
    # 帧缺口
    stamps = [int(e['stamp_ns']) for e in frames if e.get('stamp_ns')]
    stamps.sort()
    gaps = [
        (stamps[i + 1] - stamps[i]) / 1e9
        for i in range(len(stamps) - 1)]
    res['max_frame_gap_s'] = round(max(gaps), 3) if gaps else None
    res['gaps_over_2s'] = sum(1 for g in gaps if g > FRAME_GAP_MAX_S)
    return res


def geometry_metrics(bag_dir: Path) -> dict:
    """回放 target_observations，逐 target 算几何 3σ（须已 source overlay）."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions  # noqa: PLC0415
    from rclpy.serialization import deserialize_message  # noqa: PLC0415
    from peach_interfaces.msg import PeachTargetObservationArray  # noqa: PLC0415
    from rosidl_runtime_py.utilities import get_message  # noqa: PLC0415

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id='mcap'),
        ConverterOptions('', ''))
    topics = {info.name: info.type for info in reader.get_all_topics_and_types()}
    if '/peach/perception/target_observations' not in topics:
        raise RuntimeError(f'{bag_dir} 无 target_observations 话题')
    type_cls = get_message(topics['/peach/perception/target_observations'])
    series: dict[str, dict[str, list]] = collections.defaultdict(
        lambda: collections.defaultdict(list))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/peach/perception/target_observations':
            continue
        arr = deserialize_message(data, type_cls)
        if not getattr(arr, 'target_set_locked', True):
            continue
        for obs in arr.observations:
            cand = obs.candidate
            s = series[obs.target_id]
            s['entry'].append((
                cand.entry_pose.position.x,
                cand.entry_pose.position.y,
                cand.entry_pose.position.z))
            s['bottom'].append((cand.bag_bottom.x, cand.bag_bottom.y,
                                cand.bag_bottom.z))
            s['neck'].append((cand.bag_neck.x, cand.bag_neck.y,
                              cand.bag_neck.z))
            s['axis'].append((
                cand.translation_direction.x, cand.translation_direction.y,
                cand.translation_direction.z))
            if obs.fitting.bag_length_m > 0:
                s['bag_length'].append(obs.fitting.bag_length_m)

    def sigma3_mm(vals: list[tuple]) -> float | None:
        if len(vals) < 3:
            return None
        dims = list(zip(*vals))
        sig = [math.pstdev(d) for d in dims]
        return round(3.0 * 1000.0 * math.sqrt(sum(s * s for s in sig)), 3)

    def sigma3_axis_deg(axes: list[tuple]) -> float | None:
        if len(axes) < 3:
            return None
        ref = axes[len(axes) // 2]

        def norm(v):
            n = math.sqrt(sum(x * x for x in v)) or 1.0
            return [x / n for x in v]

        rn = norm(ref)
        devs = []
        for ax in axes:
            an = norm(ax)
            dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(rn, an))))
            devs.append(math.degrees(math.acos(dot)))
        return round(3.0 * math.pstdev(devs), 3)

    out = {}
    for tid, s in series.items():
        out[tid] = {
            'n': len(s['entry']),
            'entry_3sigma_mm': sigma3_mm(s['entry']),
            'bottom_3sigma_mm': sigma3_mm(s['bottom']),
            'neck_3sigma_mm': sigma3_mm(s['neck']),
            'axis_3sigma_deg': sigma3_axis_deg(s['axis']),
            'bag_length_3sigma_mm': (
                round(3.0 * 1000.0 * math.pstdev(s['bag_length']), 3)
                if len(s['bag_length']) >= 3 else None),
        }
    return out


def gates(metrics: dict, frontend: str) -> dict:
    limit = LOCK_LATENCY_LIMIT_S.get(frontend, 8.0)
    lat = metrics.get('lock_latency_s')
    return {
        'lock_latency': {
            'value_s': lat, 'limit_s': limit,
            'pass': lat is not None and lat <= limit},
        'no_id_switch': {
            'value': metrics.get('selected_id_switches'),
            'pass': metrics.get('selected_id_switches', 99) == 0},
        'no_gap_over_2s': {
            'value': metrics.get('gaps_over_2s'),
            'pass': metrics.get('gaps_over_2s', 99) == 0},
    }


def main() -> int:
    args = sys.argv[1:]
    if not args:
        print(__doc__, file=sys.stderr)
        return 2
    rid = args[0]
    frontend = 'percipio'
    bag_dir = None
    it = iter(args[1:])
    for flag in it:
        if flag == '--bag':
            bag_dir = Path(next(it))
        elif flag == '--frontend':
            frontend = next(it)
    events = load_events(rid)
    if not events:
        print(f'runs/{rid} 无感知事件', file=sys.stderr)
        return 1
    metrics = event_metrics(events)
    if bag_dir is not None:
        metrics['geometry'] = geometry_metrics(bag_dir)
    report = {'request_id': rid, 'frontend': frontend,
              'metrics': metrics, 'gates': gates(metrics, frontend)}
    out_dir = OUTER / rid
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / 'stability_metrics.json'
    out.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    for name, g in report['gates'].items():
        val = g.get('value', g.get('value_s'))
        print(f"  [{'过' if g['pass'] else '未过'}] {name}: {val}")
    print(f'  → {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
