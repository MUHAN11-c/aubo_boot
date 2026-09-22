#!/usr/bin/env python3
"""轮次指标汇总：ledger + 感知事件 + bag_report → campaign/analysis/<rid>/.

用法: python3 campaign/20260922_dual_tool/scripts/per_round_summary.py <request_id>
只读 runs/ 现有产物并出 md/json 摘要；不碰 bag 本体（purge 由 collect/purge 链负责）。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNS = ROOT / 'runs'
OUTER = ROOT / 'campaign' / '20260922_dual_tool' / 'analysis'

# 战役轮门（README「验收门」）：FULL 轮 completion≥3（LEVEL_SLEEVE_COMPLETED）
# 占比 ≥90%（skil_reconstruction 干跑结构性到不了 6，成功口径 completion≥3
# 与 sim_field_targets --mode full 的 succeed 判据一致）。
FULL_MIN_COMPLETION = 3
FULL_PASS_RATIO = 0.90
ROUND_MIN_TARGETS = 3


def load_ledger(rid: str) -> dict:
    path = RUNS / rid / 'ledger.json'
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def load_events(rid: str) -> list[dict]:
    out: list[dict] = []
    base = RUNS / rid
    for events in sorted(base.glob('perception_data/harvest_*/events.jsonl')):
        for line in events.read_text(encoding='utf-8').splitlines():
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    return out


def summarize(rid: str) -> dict:
    ledger = load_ledger(rid)
    outcomes = ledger.get('outcomes') or []
    n = len(outcomes)
    # FULL 轮（e2e_full_unrefined_）成功口径 completion≥3（LEVEL_SLEEVE_COMPLETED，
    # 与 sim_field_targets --mode full succeed 判据一致）；PREGRASP/SURVEY 轮
    # 到预抓取停住即 SUCCEEDED（outcome==0，IDL：产品级仍算成功）。
    is_full = rid.startswith('e2e_full_')
    min_completion = FULL_MIN_COMPLETION if is_full else 0
    succeeded = [
        o for o in outcomes
        if int(o.get('outcome', 9)) == 0
        and int(o.get('completion_level', 0) or 0) >= min_completion]
    failures = [o for o in outcomes if o not in succeeded]
    codes = sorted({
        str(o.get('reason') or f"code:{o.get('failure_code')}")
        for o in failures})
    events = load_events(rid)
    locked = [e for e in events if e.get('event') == 'global_targets_locked']
    elapsed = [
        float(o.get('elapsed_s') or 0.0) for o in outcomes if o.get('elapsed_s')]
    hangs = [o for o in outcomes if float(o.get('elapsed_s') or 0) > 300.0]
    ratio = (len(succeeded) / n) if n else 0.0
    return {
        'request_id': rid,
        'round_kind': 'full' if is_full else 'pregrasp_or_survey',
        'min_completion': min_completion,
        'targets': n,
        'succeeded_completion_ge3': len(succeeded),
        'pass_ratio': round(ratio, 4),
        'failure_reasons': codes,
        'over_300s': len(hangs),
        'locks': len(locked),
        'total_elapsed_s': round(sum(elapsed), 1),
        'gate_targets_ge3': n >= ROUND_MIN_TARGETS,
        'gate_ratio_ge_090': ratio >= FULL_PASS_RATIO,
        'gate_no_hang': not hangs,
        'gate_all_failures_coded': all(
            (o.get('reason') or o.get('failure_code') is not None)
            for o in failures),
        'outcomes': [
            {k: o.get(k) for k in (
                'target_id', 'outcome', 'reason', 'completion_level',
                'failure_code', 'elapsed_s', 'quality_score')}
            for o in outcomes],
    }


def write_reports(summary: dict) -> tuple[Path, Path]:
    rid = summary['request_id']
    out_dir = OUTER / rid
    out_dir.mkdir(parents=True, exist_ok=True)
    jpath = out_dir / 'per_round_summary.json'
    jpath.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    gates = summary
    md = [
        f"# 轮次摘要 {rid}（{gates['round_kind']}）", '',
        f"- 目标数: {gates['targets']}（门 ≥{ROUND_MIN_TARGETS}: "
        f"{'过' if gates['gate_targets_ge3'] else '未过'}）",
        f"- completion≥{gates['min_completion']} 占比: "
        f"{gates['pass_ratio']:.0%}（门 ≥{FULL_PASS_RATIO:.0%}: "
        f"{'过' if gates['gate_ratio_ge_090'] else '未过'}）",
        f"- >300s 挂起: {gates['over_300s']}"
        f"（门 0: {'过' if gates['gate_no_hang'] else '未过'}）",
        f"- 失败均带原因码: {'过' if gates['gate_all_failures_coded'] else '未过'}"
        f"（{', '.join(gates['failure_reasons']) or '无失败'}）",
        f"- 锁定事件数: {gates['locks']}；总耗时 {gates['total_elapsed_s']}s",
        '', '| target | outcome | completion | reason | elapsed_s |',
        '|---|---|---|---|---|',
    ]
    for o in gates['outcomes']:
        md.append(
            f"| {o.get('target_id')} | {o.get('outcome')} "
            f"| {o.get('completion_level')} | {o.get('reason')} "
            f"| {o.get('elapsed_s')} |")
    mpath = out_dir / 'per_round_summary.md'
    mpath.write_text('\n'.join(md) + '\n', encoding='utf-8')
    return jpath, mpath


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    rid = sys.argv[1]
    if not (RUNS / rid).is_dir():
        print(f'runs/{rid} 不存在', file=sys.stderr)
        return 1
    summary = summarize(rid)
    jpath, mpath = write_reports(summary)
    print(f"轮次 {rid}: 目标 {summary['targets']}，"
          f"过门占比 {summary['pass_ratio']:.0%}，"
          f"门: targets={summary['gate_targets_ge3']} "
          f"ratio={summary['gate_ratio_ge_090']} "
          f"hang={summary['gate_no_hang']}")
    print(f'  {jpath}')
    print(f'  {mpath}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
