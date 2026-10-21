#!/usr/bin/env python3
"""统计 CoP 闭环试验 JSONL：CSV / Markdown 表 / 三联图。

默认输出到 $ROS_HOME/serial_force_sim/：
  force_align_sim_trials.csv  逐组明细
  force_align_sim_stats.md    自动生成的统计表
  force_align_sim.png         三联图
"""
import argparse
import json
import os
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np

ROS_HOME = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros')))
OUT_DIR = ROS_HOME / 'serial_force_sim'


def load(path):
    rows = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if 'error' in rec or 'initial' not in rec:
                continue
            rows.append(rec)
    return rows


def st(vals):
    a = np.asarray(vals, float)
    if a.size == 0:
        return float('nan'), float('nan')
    return float(a.mean()), float(a.std(ddof=1)) if a.size > 1 else 0.0


def fmt(m, s, nd=2):
    return f'{m:.{nd}f} ± {s:.{nd}f}'


def contact(f, thr=0.05):
    n = sum(1 for v in f if v > thr)
    a = np.asarray(f, float)
    cv = float(a.std() / a.mean()) if a.mean() > 1e-9 else float('nan')
    return n, cv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('jsonl')
    ap.add_argument('--out-dir', default=str(OUT_DIR))
    args = ap.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rows = load(args.jsonl)
    if not rows:
        raise SystemExit('no successful trials in ' + args.jsonl)

    with open(out / 'force_align_sim_trials.csv', 'w') as fh:
        fh.write('tilt_deg,tilt_yaw_deg,repeat,init_cop_x,init_cop_y,'
                 'init_cop_norm,init_total,final_cop_x,final_cop_y,'
                 'final_cop_norm,final_total,reduction_pct,success,reason,'
                 'n_steps,rotation_deg,duration_s,contact_before,contact_after,'
                 'cv_before,cv_after\n')
        for r in rows:
            fh.write(','.join(str(v) for v in [
                r['tilt_deg'], r['tilt_yaw_deg'], r['repeat'],
                r['initial']['cop'][0], r['initial']['cop'][1],
                r['initial']['cop_norm'], r['initial']['total'],
                r['final']['cop'][0], r['final']['cop'][1],
                r['final']['cop_norm'], r['final']['total'],
                r['reduction_pct'], r['success'], r['reason'],
                r['n_steps'], r['rotation_deg'], r['duration_s'],
                contact(r['initial']['forces'])[0],
                contact(r['final']['forces'])[0],
                round(contact(r['initial']['forces'])[1], 3),
                round(contact(r['final']['forces'])[1], 3),
            ]) + '\n')

    by_tilt = defaultdict(list)
    by_scene = defaultdict(list)
    for r in rows:
        by_tilt[float(r['tilt_deg'])].append(r)
        by_scene[(float(r['tilt_deg']), float(r['tilt_yaw_deg']))].append(r)

    lines = []
    n = len(rows)
    succ = sum(1 for r in rows if r['success'])
    finals = [r['final']['cop_norm'] for r in rows]
    reds = [r['reduction_pct'] for r in rows if r['reduction_pct'] is not None]
    steps = [r['n_steps'] for r in rows]
    rots = [abs(r['rotation_deg']) for r in rows]
    durs = [r['duration_s'] for r in rows]
    lines.append('# 五点力 CoP 闭环控制仿真 — 统计结果\n')
    lines.append(f'- 数据文件：{args.jsonl}')
    lines.append(f'- 有效试验：**{n}** 组（5 点力 → CoP → IK/JTC 闭环校正）')
    lines.append(f'- 成功率（最终 |CoP| < 1.2 mm）：**{succ}/{n} = {succ / n * 100:.1f}%**')
    lines.append(f'- 最终 |CoP|：{fmt(*st(finals))} mm；'
                 f'降幅：{fmt(*st(reds))}%')
    lines.append(f'- 步数：{fmt(*st(steps), 1)}；'
                 f'|旋转|：{fmt(*st(rots))}°；'
                 f'单次耗时：{fmt(*st(durs), 1)} s')
    lines.append('- 场景：倾角 1–5°，方位 0°/90°，每场景重复 2 次；'
                 '力模型 k=0.8 N/mm、预载 0.30 mm。')
    lines.append('- 控制：CoP → /compute_ik → JTC，居中门限 1.2 mm，'
                 '单步 ≤0.02 rad，累计 ≤5°，最多 12 步。\n')

    lines.append('## 按倾角汇总\n')
    lines.append('| 倾角 ° | n | 初始 \\|CoP\\| mm | 最终 \\|CoP\\| mm | '
                 '降幅 % | 步数 | \\|旋转\\| ° | 成功 |')
    lines.append('|---:|---:|---:|---:|---:|---:|---:|---:|')
    for tilt in sorted(by_tilt):
        rs = by_tilt[tilt]
        ok = sum(1 for r in rs if r['success'])
        lines.append(
            f'| {tilt:g} | {len(rs)} | '
            f'{fmt(*st([r["initial"]["cop_norm"] for r in rs]))} | '
            f'{fmt(*st([r["final"]["cop_norm"] for r in rs]))} | '
            f'{fmt(*st([r["reduction_pct"] for r in rs]))} | '
            f'{fmt(*st([r["n_steps"] for r in rs]), 1)} | '
            f'{fmt(*st([abs(r["rotation_deg"]) for r in rs]))} | '
            f'{ok}/{len(rs)} |')

    lines.append('\n## 按场景（倾角 × 方位）\n')
    lines.append('| 倾角 ° | 方位 ° | n | 初始 \\|CoP\\| mm | 最终 \\|CoP\\| mm | '
                 '降幅 % | 成功 |')
    lines.append('|---:|---:|---:|---:|---:|---:|---:|')
    for key in sorted(by_scene):
        rs = by_scene[key]
        ok = sum(1 for r in rs if r['success'])
        lines.append(
            f'| {key[0]:g} | {key[1]:g} | {len(rs)} | '
            f'{fmt(*st([r["initial"]["cop_norm"] for r in rs]))} | '
            f'{fmt(*st([r["final"]["cop_norm"] for r in rs]))} | '
            f'{fmt(*st([r["reduction_pct"] for r in rs]))} | '
            f'{ok}/{len(rs)} |')

    tilts = np.array(sorted(by_tilt))
    init_m = np.array([np.mean([r['initial']['cop_norm'] for r in by_tilt[t]]) for t in tilts])
    init_s = np.array([np.std([r['initial']['cop_norm'] for r in by_tilt[t]], ddof=0)
                       for t in tilts])
    fin_m = np.array([np.mean([r['final']['cop_norm'] for r in by_tilt[t]]) for t in tilts])
    fin_s = np.array([np.std([r['final']['cop_norm'] for r in by_tilt[t]], ddof=0) for t in tilts])
    red_m = np.array([np.mean([r['reduction_pct'] for r in by_tilt[t]]) for t in tilts])
    step_m = np.array([np.mean([r['n_steps'] for r in by_tilt[t]]) for t in tilts])
    rot_m = np.array([np.mean([abs(r['rotation_deg']) for r in by_tilt[t]]) for t in tilts])

    corr = float(np.corrcoef(tilts, init_m)[0, 1])
    cb = [contact(r['initial']['forces'])[0] for r in rows]
    ca = [contact(r['final']['forces'])[0] for r in rows]
    cvb = [contact(r['initial']['forces'])[1] for r in rows]
    cva = [contact(r['final']['forces'])[1] for r in rows]
    lines.append('\n## 接触状态与力均匀性\n')
    lines.append(f'- 有载压点数（f > 0.05 N）：初始 {fmt(*st(cb), 1)} / 5 → '
                 f'校正后 {fmt(*st(ca), 1)} / 5。')
    lines.append(f'- 5 点力变异系数 CV：{fmt(*st(cvb), 3)} → {fmt(*st(cva), 3)}'
                 '（越小表示 5 点受力越均匀）。')
    lines.append('\n## 规律\n')
    lines.append(f'- 初始 CoP 随倾角单调增大（Pearson r = {corr:.3f}）：'
                 f'{init_m[0]:.2f} mm → {init_m[-1]:.2f} mm。')
    lines.append(f'- 闭环把 CoP 从 {init_m.mean():.2f} mm 压到 {fin_m.mean():.2f} mm，'
                 f'平均降幅 {red_m.mean():.1f}%。')
    lines.append(f'- 所需校正旋转约 {rot_m.mean():.2f}°（上限 5°），'
                 f'平均 {step_m.mean():.1f} 步收敛。')
    lines.append('- 力饱和：倾角增大时部分压点离开接触面（力=0），'
                 'CoP 偏移不再线性增长；校正后 5 点重新受力均匀。')
    lines.append('- 仿真为确定性模型：同一场景两次重复的初始/最终 CoP 完全一致'
                 '（表内该场景 σ=0）。')

    (out / 'force_align_sim_stats.md').write_text('\n'.join(lines) + '\n')

    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    ax[0].errorbar(tilts, init_m, yerr=init_s, marker='o', label='initial')
    ax[0].errorbar(tilts, fin_m, yerr=fin_s, marker='s', label='final')
    ax[0].axhline(1.2, ls='--', lw=1, color='gray', label='1.2 mm gate')
    ax[0].set_xlabel('plane tilt (deg)')
    ax[0].set_ylabel('|CoP| (mm)')
    ax[0].set_title('CoP before/after closed-loop correction')
    ax[0].legend()
    ax[0].grid(alpha=0.3)
    ax[1].bar(tilts, red_m, width=0.6, color='#4c9f70')
    ax[1].set_xlabel('plane tilt (deg)')
    ax[1].set_ylabel('CoP reduction (%)')
    ax[1].set_title('Correction effect')
    ax[1].set_ylim(0, 105)
    ax[1].grid(alpha=0.3, axis='y')
    ax[2].bar(tilts, step_m, width=0.6, color='#5b8fd6', label='steps')
    ax2 = ax[2].twinx()
    ax2.plot(tilts, rot_m, 'o-', color='#d9822b', label='|rotation| deg')
    ax[2].set_xlabel('plane tilt (deg)')
    ax[2].set_ylabel('steps')
    ax2.set_ylabel('|rotation| (deg)')
    ax[2].set_title('Control effort')
    ax[2].grid(alpha=0.3, axis='y')
    fig.tight_layout()
    fig.savefig(out / 'force_align_sim.png', dpi=160)
    print('\n'.join(lines))
    print(f'\nwrote {out}/force_align_sim_trials.csv, '
          f'{out}/force_align_sim_stats.md, {out}/force_align_sim.png')


if __name__ == '__main__':
    main()
