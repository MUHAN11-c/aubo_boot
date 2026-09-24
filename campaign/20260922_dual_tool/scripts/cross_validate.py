#!/usr/bin/env python3
"""三方互证对照脚本（C1 判定薄层，E2E 方案 v2.2 §2.5/§4.3 阶段二）.

输入解析先验（analytic_roll_ladder.json 等）与实跑 jsonl（sim_field_targets
产物），按三分法逐例对照输出分歧表；案册同源性按 ../corpora.yaml 校验。
**薄层边界**：只做 C1 判定，不重写 collect_round/bag_report（C2 复算结论以
引用方式并入归因栏，C3 视频时间戳由人工补写）。

三分法（解析=可行预报，实跑=在线结果）：
  #1 一致通过   解析成功 ∧ 实跑 SUCCEEDED
  #2 合规(下界) 解析失败 ∧ 实跑 SUCCEEDED（解析是全链下界，保守侧可接受）
  #3 互证发现   解析成功 ∧ 实跑失败且有码（护栏/实现回归 或 解析盲区，须归因）
  #4 一致失败   解析失败 ∧ 实跑失败且有码（expect=succeed 则 expect 过期）
  #5 无码失败   实跑失败/无结果且无 failure_code/error_code（hang 类缺陷）

门规则（--gate）：#3 未归因清零 且 #5 为零 才过（#3 可用 --resolved 显式
标记已归因关闭，归因文本写入 --attribution 或人工编辑输出表）。

用法：
  python3 cross_validate.py \
    --prior ../../campaign/20260922_dual_tool/analysis/injection/analytic_roll_ladder.json \
    --runs ../../runs/sim_field_targets_<ts>.jsonl \
    --corpora ../corpora.yaml \
    --out ../../campaign/20260922_dual_tool/analysis/injection/cross_validation_<date>.md [--gate]
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
from pathlib import Path

import yaml

VERDICTS = {
    1: '#1 一致通过',
    2: '#2 合规(下界)',
    3: '#3 互证发现',
    4: '#4 一致失败',
    5: '#5 无码失败',
    6: '门内一致（资格跳过）',   # skip_select 预期行为，不入门（matched 为准）
    7: '可达翻转（expect 模型缺口）',  # skip_* 期望案当轮变可达：边界非确定性，非系统失败
}


def load_prior(path: Path) -> dict:
    data = json.loads(path.read_text())
    return {row['case']: row for row in data.get('rows', []) if row.get('case')}


def load_run(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def row_coded(row: dict) -> bool:
    """有码判定：failure_code(int) / error_code(str) / error 文本任一在案."""
    if row.get('failure_code') is not None:
        return True
    if row.get('error_code'):
        return True
    if row.get('error'):
        return True
    return False


def classify(prior_ok: bool | None, row: dict) -> int:
    outcome = row.get('outcome')
    # skip_select：调度资格门预期跳过（不发 ExecuteTarget），matched 即门内一致
    if outcome == 'skipped_select' or row.get('expect') == 'skip_select':
        return 6 if row.get('matched') else 3
    # skip_*（不可达类期望）当轮翻转可达：sim 不再执行（expect 停在 reachability），
    # 行内无 outcome 无码——边界非确定性，不是系统无码失败
    if (row.get('expect') in ('skip_cartesian', 'skip_ik')
            and row.get('reachable') is True and outcome is None):
        return 7
    live_success = outcome == 0
    coded = row_coded(row)
    if not live_success and not coded:
        return 5
    if prior_ok is None:
        # 无先验行（案册不同源/漏跑）：只报无码缺陷，不入三分法
        return 5 if not coded else 0
    if prior_ok and live_success:
        return 1
    if not prior_ok and live_success:
        return 2
    if prior_ok and not live_success:
        return 3
    return 4


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--prior', required=True, type=Path,
                    help='解析先验 json（analytic_roll_ladder.json 形态: {summary, rows[]}）')
    ap.add_argument('--runs', required=True, nargs='+', type=Path,
                    help='实跑 jsonl（sim_field_targets 产物，可多份）')
    ap.add_argument('--corpora', type=Path,
                    default=Path(__file__).resolve().parent.parent / 'corpora.yaml',
                    help='案册注册表（同源校验）')
    ap.add_argument('--out', type=Path, help='输出 markdown（缺省打印 stdout）')
    ap.add_argument('--gate', action='store_true',
                    help='门模式：#3 未归因清零 且 #5 为零 才退出 0')
    ap.add_argument('--resolved', default='',
                    help='已归因关闭的 #3 案例逗号清单（配 --gate 使用）')
    args = ap.parse_args()

    prior = load_prior(args.prior)
    corpora = yaml.safe_load(args.corpora.read_text())['corpora'] \
        if args.corpora and args.corpora.exists() else {}

    # 实跑多轮同名案例以最新一轮为准（同 case 后行覆盖前行）
    live: dict[str, dict] = {}
    for run in args.runs:
        for row in load_run(run):
            live[row.get('case', '')] = row

    resolved = {x.strip() for x in args.resolved.split(',') if x.strip()}
    counts = {k: 0 for k in (1, 2, 3, 4, 5, 6, 7)}
    lines = [
        '# 三方互证 C1 对照（解析 ↔ 在线）',
        '',
        f'- 生成：{_dt.datetime.now().isoformat(timespec="seconds")}',
        f'- 先验：`{args.prior}`（{len(prior)} 例）',
        f'- 实跑：`{", ".join(str(p) for p in args.runs)}`（{len(live)} 例，同名取最新）',
        f'- 案册注册表：`{args.corpora}`',
        '',
        '| 案例 | 解析ok | 实跑outcome | failure_code | matched | 判定 | 归因（#3 必填） |',
        '|------|--------|------------|--------------|---------|------|----------------|',
    ]
    no_prior = []
    for case in sorted(set(prior) | set(live)):
        prow = prior.get(case)
        row = live.get(case)
        prior_ok = bool(prow.get('ok')) if prow else None
        if row is None:
            no_prior.append(case)
            continue  # 先验有、实跑没跑：留给下一轮 run，不入表
        verdict = classify(prior_ok, row)
        if verdict == 0:
            lines.append(
                f'| {case} | {prior_ok} | {row.get("outcome")} | '
                f'{row.get("failure_code", row.get("error_code", ""))} | '
                f'{row.get("matched")} | ⚠ 无先验（案册不同源？） | 查 corpora.yaml |')
            continue
        counts[verdict] += 1
        attribution = ''
        if verdict == 3:
            attribution = '已归因✔' if case in resolved else '**待归因**'
        elif verdict == 4 and row.get('expect') == 'succeed':
            attribution = 'expect 过期（双败）'
        lines.append(
            f'| {case} | {prior_ok} | {row.get("outcome")} | '
            f'{row.get("failure_code", row.get("error_code", ""))} | '
            f'{row.get("matched")} | {VERDICTS[verdict]} | {attribution} |')

    total = sum(counts.values())
    gate_ok = counts[3] - len(
        [c for c in resolved if c in live and classify(
            bool(prior[c]['ok']) if c in prior else None, live[c]) == 3]) <= 0 \
        and counts[5] == 0
    lines += [
        '',
        '## 三分法计数',
        '',
        f'- 覆盖 {total} 例（先验有而实跑未跑 {len(no_prior)} 例未入表）',
    ] + [f'- {VERDICTS[k]}：{v}' for k, v in counts.items()] + [
        '',
        f'- **门**：{"✅ 过（#3 已清零、#5=0）" if gate_ok else "❌ 未过（#3 待归因或 #5>0）"}',
    ]
    # 案册同源提示（grid 无 seed 不受影响；random 档提示 ladder 遗留）
    ladder = corpora.get('ladder_random_60')
    if ladder:
        lines += [
            '',
            f'> 同源提示：解析梯子 random 档 seed={ladder["seed"]}（{ladder["size"]} 例）'
            f'与 canonical random_100（seed={corpora["random_100"]["seed"]}）不同源——'
            'random 档互证须以 canonical seed 重新生成先验（corpora.yaml `ladder_random_60`）。',
        ]

    text = '\n'.join(lines) + '\n'
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)
        print(f'written {args.out}')
    else:
        print(text)
    if args.gate:
        return 0 if gate_ok else 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
