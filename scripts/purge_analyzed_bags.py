#!/usr/bin/env python3
"""分析完成后删除指定会话 bag/ MCAP，保留 bag_report 与账本.

用法:
  python3 scripts/purge_analyzed_bags.py runs/session_20260922_100001
  python3 scripts/purge_analyzed_bags.py --all-reported runs
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'peach_observability'))

from peach_observability.retention import purge_analyzed_sessions  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'sessions', nargs='*',
        help='session 目录或 bag 目录')
    parser.add_argument(
        '--all-reported', metavar='RUNS',
        help='扫描该 runs 根下已有 bag_report.md 的 session_*/bag 并删除')
    args = parser.parse_args()
    sessions = [Path(item) for item in args.sessions]
    runs_root = ROOT / 'runs'
    if args.all_reported:
        runs_root = Path(args.all_reported)
        for report in runs_root.glob('session_*/bag_report.md'):
            sessions.append(report.parent)
    if not sessions:
        print('未指定会话；给路径或 --all-reported runs', file=sys.stderr)
        return 2
    deleted = purge_analyzed_sessions(
        runs_root, sessions, log_warning=print)
    print(f'已删 {len(deleted)} 个 bag 目录')
    return 0


if __name__ == '__main__':
    sys.exit(main())
