#!/usr/bin/env python3
"""按 request_id 归集一轮测试的全部证据，出 runs/<rid>/round_report.md。

复盘完备集四源归一（09-17 用户口径：ros2 自动日志 + 会话 bag 应能完整分析）：
  1. runs/<rid>/ledger.json          逐目标 outcome/failure_code/分段耗时
  2. runs/<rid>/perception_data/**   感知/重建 events.jsonl（时间戳逐帧）
  3. ~/.ros/log/<launch 时间戳>/     ros2 自动日志（launch.log 含全部节点 stdout）
  4. runs/session_*/bag              会话 bag（2026-09-17 起 /rosout 全量入袋，
                                     节点日志可回放；此前栈无 /rosout 会在报告标注）

用法：python3 scripts/collect_round.py <request_id> [--runs-root runs] [--log-root ~/.ros/log]
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

SKIP_CODES = (
    'missing_mask', 'robot_not_static', 'same_stamp', 'near_duplicate',
    'target_drift', 'neighbor_gap',
)


def _utc(iso: str) -> float:
    try:
        return datetime.fromisoformat(iso).timestamp()
    except (ValueError, TypeError):
        return 0.0


def load_ledger(run_dir: Path):
    ledger = run_dir / 'ledger.json'
    if not ledger.exists():
        return None, []
    data = json.loads(ledger.read_text())
    rows = []
    for o in data.get('outcomes', []):
        rows.append(o)
    return data, rows


def collect_events(run_dir: Path):
    events = []
    for f in run_dir.glob('perception_data/**/events.jsonl'):
        for line in f.read_text().splitlines():
            try:
                e = json.loads(line)
            except json.JSONDecodeError:
                continue
            if e.get('event') == 'frame_observations':
                continue  # 高频心跳，只留关键事件
            events.append(e)
    events.sort(key=lambda e: _utc(e.get('recorded_at', '')))
    return events


def run_window(events, rows) -> tuple[float, float]:
    stamps = [_utc(e.get('recorded_at', '')) for e in events]
    stamps = [s for s in stamps if s]
    if not stamps:
        now = datetime.now().timestamp()
        return now - 3600.0, now
    return min(stamps), max(stamps) + 60.0


def find_launch_logs(log_root: Path, t0: float, t1: float):
    hits = []
    for d in log_root.iterdir() if log_root.exists() else []:
        if not d.is_dir():
            continue
        try:  # 目录名 2026-09-17-17-04-49-<pid>-<host>-<n>（连字符，本地时区）
            ts = datetime.strptime(
                d.name[:19], '%Y-%m-%d-%H-%M-%S').timestamp()
        except ValueError:
            continue
        if t0 - 300.0 <= ts <= t1 + 300.0:
            hits.append((ts, d))
    return sorted(hits)


def extract_log_lines(launch_dir: Path, t0: float, t1: float, max_lines: int = 40):
    log = launch_dir / 'launch.log'
    if not log.exists():
        return [], str(launch_dir)
    key, out = ('ERROR', 'WARN', 'Found a contact', 'PLANNING_FAILED',
                'Goal finished', 'observe_', 'build_', 'survey_failed'), []
    for line in log.read_text(errors='replace').splitlines():
        if any(k in line for k in key):
            out.append(line.rstrip())
    return out[:max_lines], str(log)


def find_sessions(runs_root: Path, t0: float, t1: float):
    hits = []
    for d in runs_root.glob('session_*'):
        try:
            ts = datetime.strptime(
                d.name[len('session_'):len('session_') + 15],
                '%Y%m%d_%H%M%S').timestamp()
        except ValueError:
            continue
        if t0 - 120.0 <= ts <= t1 + 600.0:
            hits.append(d)
    return sorted(hits)


def bag_has_rosout(session: Path) -> bool:
    meta = session / 'bag' / 'metadata.yaml'
    try:
        return '/rosout' in meta.read_text()
    except OSError:
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('request_id')
    ap.add_argument('--runs-root', default='runs')
    ap.add_argument('--log-root', default=str(Path.home() / '.ros' / 'log'))
    args = ap.parse_args()

    run_dir = Path(args.runs_root) / args.request_id
    if not run_dir.exists():
        print(f'找不到 {run_dir}', file=sys.stderr)
        return 1
    data, rows = load_ledger(run_dir)
    events = collect_events(run_dir)
    t0, t1 = run_window(events, rows)
    fmt = lambda ts: datetime.fromtimestamp(ts).strftime('%H:%M:%S')  # noqa: E731

    lines = []
    add = lines.append
    add(f'# 测试轮归集：{args.request_id}')
    add('')
    add(f'- 归集时间：{datetime.now().isoformat(timespec="seconds")}')
    try:
        head = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'], cwd=run_dir.parent.parent,
            capture_output=True, text=True, timeout=5).stdout.strip()
        add(f'- 代码基线：git {head}')
    except Exception:  # noqa: BLE001
        pass
    add(f'- 运行窗口（本地时区）：{fmt(t0)} → {fmt(t1)}'
        f'（含前后缓冲；由事件时间戳推断）')
    add('')

    add('## 逐目标结果（ledger）')
    add('')
    if rows:
        add('| target | outcome | failure_code | elapsed_s | 段耗时 |')
        add('|--------|---------|--------------|-----------|--------|')
        for o in rows:
            stages = dict(zip(
                o.get('stage_names', []),
                o.get('stage_durations', [])))
            add(f"| {o.get('target_id')} | {o.get('outcome')} "
                f"| {o.get('failure_code', '')} | {o.get('elapsed_s', '')} "
                f"| {stages or o.get('build_status', '')} |")
    else:
        add('（无 outcomes——批在派发前即终止或被取消）')
    add('')

    add('## 感知/重建关键事件（perception_data，时间戳升序）')
    add('')
    add('```')
    for e in events[:60]:
        iso = e.get('recorded_at', '')
        try:
            ts = datetime.fromisoformat(iso).astimezone().strftime('%H:%M:%S.%f')[:-3]
        except ValueError:
            ts = iso[11:23]
        tag = e.get('event', '?')
        detail = e.get('code') or e.get('reason') or ''
        if tag == 'frame_skipped' and e.get('code') in SKIP_CODES:
            detail = f"skip[{e.get('code')}] {e.get('reason', '')[:40]}"
        add(f'{ts} {tag:24s} {detail}')
    if len(events) > 60:
        add(f'… 共 {len(events)} 条（已截断）')
    add('```')
    add('')

    add('## ROS 2 自动日志（~/.ros/log，窗口内 launch 目录）')
    add('')
    launches = find_launch_logs(Path(args.log_root), t0, t1)
    if not launches:
        add('（窗口内未找到 launch 目录——检查 --log-root）')
    for ts, d in launches:
        add(f'### {d.name}')
        key_lines, src = extract_log_lines(d, t0, t1)
        add(f'来源：`{src}`；关键行（ERROR/WARN/碰撞/规划失败/阶段门）：')
        add('```')
        lines.extend(key_lines)
        add('```')
        add('')

    add('## 会话 bag')
    add('')
    sessions = find_sessions(Path(args.runs_root), t0, t1)
    if not sessions:
        add('（窗口内未找到 session 目录）')
    for s in sessions:
        rosout = '✓ 含 /rosout（节点日志可回放：ros2 bag play 后 echo /rosout）' \
            if bag_has_rosout(s) else '✗ 无 /rosout（栈早于 2026-09-17 改动）'
        report = s / 'bag_report.md'
        add(f'- `{s}`：{rosout}'
            + (f'；报告 `{report}`' if report.exists() else ''))
    add('')

    out = run_dir / 'round_report.md'
    out.write_text('\n'.join(lines) + '\n')
    print(f'已生成 {out}（{len(lines)} 行）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
