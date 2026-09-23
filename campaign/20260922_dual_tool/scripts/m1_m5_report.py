#!/usr/bin/env python3
"""M1–M5 注入矩阵门判定：runs/sim_field_targets_*.jsonl → 指标表.

用法: python3 m1_m5_report.py <jsonl 路径> [jsonl 路径...]
（多文件传同一个 --random 矩阵的 pregrasp/full 两轮即可分别出表）
门（campaign README「验收门」）：
  M1 网格   期望分类 matched 100% 一致
  M2 随机30 PREGRASP SUCCEEDED ≥95%
  M3 随机30 FULL     completion≥3 占比 ≥90%、绕行比 ≤1.70、失败 100% 有码
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

OUTER = Path(__file__).resolve().parents[1] / 'analysis' / 'injection'


def load(path: Path) -> list[dict]:
    rows = []
    for line in path.read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))
    return rows


def _int(v, default: int = -1) -> int:
    try:
        return int(v)
    except (TypeError, ValueError):
        return default


def has_grid(rows: list[dict]) -> bool:
    return bool(rows) and all(str(r.get('case', '')).startswith(('typical', 'tilt', 'travel', 'info', 'lab', 'far', 'bbox', 'tool_', 'near_', 'bag_', 'short_', 'long_', 'occluded_', 'right_', 'deep_', 'mid_')) for r in rows)


def report(path: Path) -> str:
    rows = load(path)
    n = len(rows)
    if not n:
        return f'{path.name}: 空'
    grid = has_grid(rows)
    matched = [r for r in rows if r.get('matched')]
    succeeded = [r for r in rows if _int(r.get('outcome')) == 0]
    full_ok = [
        r for r in succeeded
        if _int(r.get('completion_level')) >= 3 and not r.get('grasped')]
    failures = [
        r for r in rows
        if _int(r.get('outcome')) != 0
        and r.get('expect') not in ('skip_select', 'skip_ik', 'skip_cartesian')
        and r.get('outcome') != 'decision_denied']
    uncoded = [
        r for r in failures
        if not (r.get('reason') or r.get('failure_code') is not None)]
    hangs = [r for r in rows if float(r.get('elapsed_s') or 0) > 300.0]
    detours = [r for r in rows if r.get('detour_flag')]
    lines = [f'## {path.name}（{n} 例{"，网格" if grid else ""}）', '']
    if grid:
        ratio = len(matched) / n
        lines.append(f"- M1 网格期望一致: {len(matched)}/{n} = {ratio:.0%}"
                     f"（门 100%: {'过' if ratio == 1.0 else '未过'}）")
        for r in rows:
            if not r.get('matched'):
                lines.append(
                    f"  - 不一致: {r['case']} expect={r.get('expect')} "
                    f"outcome={r.get('outcome')} completion={r.get('completion_level')} "
                    f"reason={r.get('reason')}")
    elif n >= 30 or n > 0:
        ratio = (len(full_ok) / n) if n else 0
        detour_ratio = (len(detours) / n) if n else 0
        lines += [
            f"- SUCCEEDED: {len(succeeded)}/{n} = {len(succeeded)/n:.0%}",
            f"- completion≥3 且未抓取: {len(full_ok)}/{n} = {ratio:.0%}"
            f"（门 ≥90%: {'过' if ratio >= 0.90 else '未过'}）",
            f"- 绕行比: {detour_ratio:.2f}（门 ≤1.70: "
            f"{'过' if detour_ratio <= 1.70 else '未过'}）",
        ]
    lines += [
        f"- 失败 {len(failures)} 例，无码 {len(uncoded)} 例，"
        f">300s 挂起 {len(hangs)} 例"
        f"（门: 无码 0、挂起 0: "
        f"{'过' if not uncoded and not hangs else '未过'}）",
    ]
    reasons = {}
    for r in failures:
        reasons[str(r.get('reason') or r.get('failure_code'))] = \
            reasons.get(str(r.get('reason') or r.get('failure_code')), 0) + 1
    if reasons:
        lines.append(f"- 失败原因分布: {reasons}")
    return '\n'.join(lines)


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    paths = [Path(p) for p in sys.argv[1:]]
    blocks = [report(p) for p in paths]
    text = '# 注入矩阵 M1–M5 门判定\n\n' + '\n\n'.join(blocks) + '\n'
    print(text)
    OUTER.mkdir(parents=True, exist_ok=True)
    out = OUTER / 'm1_m5_report.md'
    out.write_text(text, encoding='utf-8')
    print(f'→ {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
