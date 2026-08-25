#!/usr/bin/env python3
"""只读复算归档采摘指标。对照 docs/testing.md 量化基线。

不写 run 目录、不删数据、不依赖 ROS。默认扫描仓库
``_archive/runs/`` 与（若存在）``runs/``。
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path

FPS_RE = re.compile(
    r'帧间隔中位数[：:]\s*([0-9.]+|None)\s*s[（(]≈\s*([0-9.]+|None)\s*FPS')
DURATION_RE = re.compile(r'预计时长\s*([0-9.]+)\s*s')
FRACTION_RE = re.compile(r'min_fraction\s*([0-9.]+)')


def workspace_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'src' / 'peach_interfaces').is_dir():
            return parent
    return Path.cwd()


def iter_jsonl(path: Path):
    try:
        text = path.read_text(encoding='utf-8')
    except OSError:
        return
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            yield json.loads(line)
        except json.JSONDecodeError:
            continue


def load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None


def run_dirs(roots: list[Path]) -> list[Path]:
    found: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        if not root.is_dir():
            continue
        for marker in (
            'ledger.json', 'reconstruction.jsonl', 'summary.md',
            'events.jsonl', 'approach.jsonl',
        ):
            for path in root.rglob(marker):
                parent = path.parent.resolve()
                if parent not in seen:
                    seen.add(parent)
                    found.append(parent)
    return sorted(found)


def summarize(roots: list[Path]) -> str:
    fps_vals: list[float] = []
    skip_counts: Counter[str] = Counter()
    views_max_by_run: list[int] = []
    tf_failures_max = 0
    codes: Counter[str] = Counter()
    stale = 0
    mtc_durations: list[float] = []
    fractions: list[float] = []
    mtc_stage_fail = 0
    ledgers = 0
    recon_files = 0
    summaries = 0

    for folder in run_dirs(roots):
        summary = folder / 'summary.md'
        if summary.is_file():
            summaries += 1
            match = FPS_RE.search(summary.read_text(encoding='utf-8', errors='replace'))
            if match and match.group(1) != 'None' and match.group(2) != 'None':
                try:
                    fps_vals.append(float(match.group(2)))
                except ValueError:
                    pass

        recon = folder / 'reconstruction.jsonl'
        if recon.is_file():
            recon_files += 1
            views_max = 0
            last_skips: dict[str, int] = {}
            for rec in iter_jsonl(recon):
                data = rec.get('data') if isinstance(rec, dict) else None
                if not isinstance(data, dict):
                    continue
                captured = data.get('captured_views')
                if isinstance(captured, int):
                    views_max = max(views_max, captured)
                fails = data.get('tf_failures')
                if isinstance(fails, int):
                    tf_failures_max = max(tf_failures_max, fails)
                skips = data.get('skip_reasons')
                if isinstance(skips, dict) and skips:
                    last_skips = {
                        str(k): int(v) for k, v in skips.items()
                        if isinstance(v, (int, float))
                    }
            if views_max:
                views_max_by_run.append(views_max)
            skip_counts.update(last_skips)

        ledger = load_json(folder / 'ledger.json')
        if isinstance(ledger, dict) and 'outcomes' in ledger:
            ledgers += 1
            for outcome in ledger.get('outcomes') or []:
                if not isinstance(outcome, dict):
                    continue
                code = outcome.get('failure_code')
                if code in (None, '', 0):
                    if outcome.get('outcome') in (0, 'succeeded', 'success'):
                        code = 'succeeded'
                    else:
                        code = str(outcome.get('outcome'))
                codes[str(code)] += 1
                reason = str(outcome.get('reason') or '')
                if 'selected_target_stale' in reason:
                    stale += 1
                for match in DURATION_RE.finditer(reason):
                    mtc_durations.append(float(match.group(1)))
                for match in FRACTION_RE.finditer(reason):
                    fractions.append(float(match.group(1)))
                if '(0/1)' in reason:
                    mtc_stage_fail += 1

        for rec in iter_jsonl(folder / 'approach.jsonl'):
            data = rec.get('data') if isinstance(rec, dict) else rec
            if not isinstance(data, dict):
                continue
            msg = str(data.get('message') or '')
            if 'selected_target_stale' in msg:
                stale += 1
            for match in DURATION_RE.finditer(msg):
                mtc_durations.append(float(match.group(1)))
            for match in FRACTION_RE.finditer(msg):
                fractions.append(float(match.group(1)))
            if '(0/1)' in msg:
                mtc_stage_fail += 1

    skip_total = sum(skip_counts.values())
    static_share = (
        100.0 * skip_counts.get('robot_not_static', 0) / skip_total
        if skip_total else 0.0)
    fps_line = (
        f'{min(fps_vals):.2f}–{max(fps_vals):.2f}（n={len(fps_vals)}）'
        if fps_vals else '未解析到 summary 帧间隔')
    views_line = (
        f'中位 {sorted(views_max_by_run)[len(views_max_by_run)//2]}，'
        f'最大 {max(views_max_by_run)}（n={len(views_max_by_run)}）'
        if views_max_by_run else '无')
    dur_line = (
        f'{min(mtc_durations):.2f}–{max(mtc_durations):.2f} s（n={len(mtc_durations)}）'
        if mtc_durations else '无')
    frac_line = (
        f'{min(fractions):.4f}–{max(fractions):.4f}（n={len(fractions)}）'
        if fractions else '无')
    top_skips = ', '.join(
        f'{k}={v}' for k, v in skip_counts.most_common(6)) or '无'
    top_codes = ', '.join(
        f'{k}={v}' for k, v in codes.most_common(8)) or '无'

    lines = [
        '# replay_metrics',
        '',
        f'根目录：{", ".join(str(p) for p in roots)}',
        f'扫描：summary.md={summaries} reconstruction.jsonl={recon_files} '
        f'ledger.json={ledgers}',
        '',
        '| 门 | 复算 |',
        '|----|------|',
        f'| 相机速率 | {fps_line} |',
        f'| 有效视角 captured_views_max | {views_line} |',
        f'| 静止跳过 robot_not_static | {skip_counts.get("robot_not_static", 0)}/'
        f'{skip_total}（{static_share:.0f}%）；{top_skips} |',
        f'| TF tf_failures 峰值 | {tf_failures_max} |',
        f'| 新鲜度 selected_target_stale | {stale} |',
        f'| 账本 failure_code | {top_codes} |',
        f'| MTC 预计时长 | {dur_line} |',
        f'| Cartesian min_fraction | {frac_line} |',
        f'| MTC 段 (0/1) | {mtc_stage_fail} |',
        '',
        '对照 docs/testing.md 量化基线。本脚本不证明在线变好，只固定口径。',
    ]
    return '\n'.join(lines) + '\n'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'roots', nargs='*', type=Path,
        help='额外数据根（默认仓库 _archive/runs 与 runs/）')
    args = parser.parse_args(argv)
    root = workspace_root()
    roots = [root / '_archive' / 'runs', root / 'runs']
    roots.extend(args.roots)
    existing = [p.resolve() for p in roots if p.is_dir()]
    if not existing:
        print('未找到 _archive/runs 或 runs/', file=sys.stderr)
        return 1
    sys.stdout.write(summarize(existing))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
