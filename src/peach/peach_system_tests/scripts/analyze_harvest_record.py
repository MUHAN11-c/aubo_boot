#!/usr/bin/env python3
"""从一次采摘记录画出论文里常用的运行统计，并导出表格。

口径对齐田间采摘评价，而不是检测榜的 mAP：

- Bac 等，Biosystems Engineering 2014：采摘成功率、单果周期时间。
- Silwal 等，Journal of Field Robotics 2017：周期拆成接近/操作/撤离。
- Sa 等，Sensors 2016（DeepFruits）：检测结果用原图加掩膜做定性对照。
  本记录没有人工框真值，不算 precision / recall。
- Williams 等，Journal of Field Robotics 2019（猕猴桃）：周期时间累积分布、失败分型。
- Lehnert 等，Journal of Field Robotics 2017（甜椒）：袋心在基座系的分布、
  位置与轴向不确定度。

账本 ``outcome=SUCCEEDED`` 且 ``completion_level>=6`` 只表示运动周期走到撤离。
刀具未使能时没有切断确认，不能写成果实已采下（那是完成度 7 / grasped）。

用法::

    python3 src/peach/peach_system_tests/scripts/analyze_harvest_record.py \\
        runs/session_20261020_172443
"""
from __future__ import annotations

import argparse
import csv
import json
import re
from collections import Counter
from datetime import datetime, timedelta
from pathlib import Path

import cv2
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

OUTCOME_NAME = {
    0: '周期完成',
    1: '质量跳过',
    2: '不可达',
    3: '失败',
    4: '取消',
}
STAGE_NAME = {
    'reconfirm': '再确认',
    'approach_insert': '接近与套入',
    'tool': '刀具',
    'retreat': '撤离',
}
STAGE_ORDER = ('reconfirm', 'approach_insert', 'tool', 'retreat')
INK = '#1c1917'
MUTED = '#78716c'
PAPER = '#faf7f2'
ACCENT = '#3f6212'
WARN = '#b45309'
BAD = '#9f1239'
STAGE_COLOR = {
    'reconfirm': '#a8a29e',
    'approach_insert': '#3f6212',
    'tool': '#b45309',
    'retreat': '#57534e',
}


def _font() -> None:
    """用本机思源黑体出中文，避免方框。"""
    candidates = (
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.otf',
    )
    chosen = None
    for path in candidates:
        if Path(path).is_file():
            font_manager.fontManager.addfont(path)
            chosen = 'Noto Sans CJK SC'
            break
    if chosen is None:
        for font in font_manager.fontManager.ttflist:
            if 'Noto Sans CJK SC' in font.name:
                chosen = font.name
                break
    plt.rcParams.update({
        'font.family': chosen or 'DejaVu Sans',
        'axes.unicode_minus': False,
        'figure.facecolor': PAPER,
        'axes.facecolor': PAPER,
        'savefig.facecolor': PAPER,
        'axes.edgecolor': INK,
        'axes.labelcolor': INK,
        'xtick.color': INK,
        'ytick.color': INK,
        'text.color': INK,
        'axes.titleweight': 'medium',
        'axes.titlesize': 13,
        'axes.labelsize': 11,
    })


def _stamp(name: str) -> datetime | None:
    match = re.search(r'(20\d{6})[_T](\d{6})', name)
    if not match:
        return None
    return datetime.strptime(match.group(1) + match.group(2), '%Y%m%d%H%M%S')


def _sessions(parent: Path) -> list[tuple[datetime, Path]]:
    found = []
    for path in parent.iterdir():
        if path.is_dir() and path.name.startswith('session_'):
            stamp = _stamp(path.name)
            if stamp is not None:
                found.append((stamp, path))
    return sorted(found)


def _window(session: Path) -> tuple[datetime, datetime]:
    start = _stamp(session.name) or datetime.min
    end = start + timedelta(hours=6)
    if session.parent.is_dir():
        for stamp, _path in _sessions(session.parent):
            if stamp > start:
                end = stamp
                break
    return start, end


def _in_window(path: Path, start: datetime, end: datetime) -> bool:
    stamp = _stamp(path.name)
    if stamp is None:
        match = re.fullmatch(r'flow(?:10)?_(\d{6})(?:_r\d+)?', path.name)
        if match:
            clock = datetime.strptime(match.group(1), '%H%M%S')
            stamp = start.replace(
                hour=clock.hour, minute=clock.minute, second=clock.second,
                microsecond=0)
    return stamp is not None and start <= stamp < end


def collect(run: Path) -> list[Path]:
    """会话目录本身，加上同一次启动写在 runs 根上的批次和感知目录。"""
    run = run.resolve()
    roots = [run]
    if run.name.startswith('session_') and run.parent.is_dir():
        start, end = _window(run)
        for child in run.parent.iterdir():
            if child == run or not child.is_dir():
                continue
            if child.name.startswith('session_'):
                continue
            if _in_window(child, start, end):
                roots.append(child)
    return roots


def _ledgers(roots: list[Path]) -> list[tuple[str, dict]]:
    found = []
    for root in roots:
        for path in root.rglob('ledger.json'):
            payload = json.loads(path.read_text(encoding='utf-8'))
            if isinstance(payload.get('outcomes'), list):
                found.append((path.parent.name, payload))
    return sorted(found, key=lambda item: item[0])


def _rows(ledgers: list[tuple[str, dict]]) -> list[dict]:
    rows = []
    for batch, payload in ledgers:
        for outcome in payload.get('outcomes') or []:
            item = dict(outcome)
            item['batch'] = batch
            rows.append(item)
    return rows


def _geometry(roots: list[Path]) -> list[dict]:
    """每个批次、每个目标保留最后一条带许可字段的拟合行。"""
    latest: dict[tuple[str, str], dict] = {}
    for root in roots:
        for path in root.rglob('geometry.jsonl'):
            batch = path.parent.name
            for line in path.read_text(encoding='utf-8').splitlines():
                if not line.strip():
                    continue
                row = json.loads(line)
                if 'allowed' not in row and 'd95_m' not in row:
                    continue
                row['batch'] = batch
                key = (batch, str(row.get('target_id')))
                previous = latest.get(key)
                if previous is None or row.get('fused') or not previous.get('fused'):
                    latest[key] = row
    return list(latest.values())


def _events(roots: list[Path]) -> list[dict]:
    found = []
    for root in roots:
        for path in root.rglob('events.jsonl'):
            for line in path.read_text(encoding='utf-8').splitlines():
                if line.strip():
                    found.append(json.loads(line))
    return found


def _mask_samples(roots: list[Path]) -> list[dict]:
    """掩膜面积占比和质心，用来看检测在画面里的分布。"""
    samples = []
    for root in roots:
        for mask_path in root.rglob('masks/*.png'):
            mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            if mask is None or mask.size == 0:
                continue
            region = mask > 0
            area = float(np.count_nonzero(region)) / float(mask.size)
            if not np.any(region):
                continue
            ys, xs = np.nonzero(region)
            samples.append({
                'area': area,
                'u': float(np.mean(xs)) / float(mask.shape[1]),
                'v': float(np.mean(ys)) / float(mask.shape[0]),
            })
    return samples


def _bare(axis) -> None:
    axis.spines['top'].set_visible(False)
    axis.spines['right'].set_visible(False)


def _pairs(roots: list[Path], limit: int = 8) -> list[tuple[Path, Path, str]]:
    grouped: dict[str, list[tuple[Path, Path]]] = {}
    for root in roots:
        for mask in root.rglob('masks/*.png'):
            color = mask.parent.parent / 'colors' / mask.name
            if not color.is_file():
                continue
            label = mask.parent.parent.name
            for parent in mask.parents:
                if parent.name.startswith(('flow', 'round_', 'session_')):
                    label = parent.name
                    break
            grouped.setdefault(label, []).append((color, mask))
    chosen = []
    names = sorted(grouped)
    if not names:
        return []
    per = max(1, limit // len(names))
    for name in names:
        for color, mask in sorted(grouped[name])[:per]:
            chosen.append((color, mask, name))
            if len(chosen) >= limit:
                return chosen
    return chosen


def _overlay(color_path: Path, mask_path: Path) -> np.ndarray:
    color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if color is None or mask is None:
        raise RuntimeError(f'无法读取 {color_path.name}')
    if mask.shape[:2] != color.shape[:2]:
        mask = cv2.resize(
            mask, (color.shape[1], color.shape[0]), interpolation=cv2.INTER_NEAREST)
    region = mask > 0
    painted = color.copy()
    painted[region] = (
        0.45 * painted[region] + 0.55 * np.array([70, 170, 60])
    ).astype(np.uint8)
    contours, _hierarchy = cv2.findContours(
        region.astype(np.uint8) * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(painted, contours, -1, (30, 70, 20), 2)
    return cv2.cvtColor(painted, cv2.COLOR_BGR2RGB)


def _save(figure: plt.Figure, path: Path) -> None:
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def plot_outcomes(rows: list[dict], path: Path) -> None:
    counts = Counter(int(row.get('outcome', -1)) for row in rows)
    labels = [OUTCOME_NAME.get(key, str(key)) for key in sorted(counts)]
    values = [counts[key] for key in sorted(counts)]
    done = sum(1 for row in rows if int(row.get('outcome', -1)) == 0
               and int(row.get('completion_level') or 0) >= 6)
    rate = done / len(rows) if rows else 0.0
    figure, axis = plt.subplots(figsize=(7.2, 4.2))
    colors = [ACCENT if label == '周期完成' else WARN if '跳过' in label or label == '不可达' else BAD
              for label in labels]
    axis.bar(labels, values, color=colors, width=0.62)
    axis.set_ylabel('果数')
    axis.set_title(f'终局分布    运动周期完成 {done}/{len(rows)} = {rate:.0%}')
    axis.spines['top'].set_visible(False)
    axis.spines['right'].set_visible(False)
    _save(figure, path)


def plot_cycle(rows: list[dict], path: Path) -> None:
    done = [row for row in rows if int(row.get('outcome', -1)) == 0]
    elapsed = [float(row['elapsed_s']) for row in done if row.get('elapsed_s')]
    figure, axes = plt.subplots(1, 2, figsize=(9.6, 4.2))
    axes[0].hist(elapsed, bins=8, color=ACCENT, edgecolor=PAPER)
    if elapsed:
        median = float(np.median(elapsed))
        axes[0].axvline(median, color=INK, linewidth=1.2)
        axes[0].set_title(f'单果周期    中位 {median:.1f} s')
    axes[0].set_xlabel('秒')
    axes[0].set_ylabel('果数')
    means = []
    labels = []
    colors = []
    for stage in STAGE_ORDER:
        samples = []
        for row in done:
            names = row.get('stage_names') or []
            durations = row.get('stage_durations') or []
            if stage in names:
                samples.append(float(durations[names.index(stage)]))
        if not samples:
            continue
        labels.append(STAGE_NAME[stage])
        means.append(float(np.mean(samples)))
        colors.append(STAGE_COLOR[stage])
    axes[1].barh(labels, means, color=colors)
    axes[1].set_xlabel('平均秒')
    axes[1].set_title('周期构成')
    for axis in axes:
        _bare(axis)
    _save(figure, path)


def plot_rounds(rows: list[dict], path: Path) -> None:
    batches = []
    rates = []
    for batch in sorted({row['batch'] for row in rows}):
        group = [row for row in rows if row['batch'] == batch]
        done = sum(int(row.get('outcome', -1)) == 0 for row in group)
        batches.append(batch.replace('flow10_', '').replace('flow_', ''))
        rates.append(done / len(group) if group else 0.0)
    figure, axis = plt.subplots(figsize=(8.4, 4.2))
    axis.bar(range(len(batches)), [100 * rate for rate in rates], color=ACCENT, width=0.7)
    axis.set_xticks(range(len(batches)), batches, rotation=35, ha='right')
    axis.set_ylim(0, 100)
    axis.set_ylabel('周期完成率 %')
    axis.set_title('各批完成率')
    _bare(axis)
    _save(figure, path)


def plot_geometry(rows: list[dict], path: Path) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(8.8, 6.2))
    specs = (
        ('d95_m', 1000.0, '袋径 D95 / mm'),
        ('sigma_position_m', 1000.0, '位置不确定度 / mm'),
        ('sigma_axis_deg', 1.0, '轴向不确定度 / °'),
        ('axis_conflict_deg', 1.0, '轴冲突角 / °'),
    )
    for axis, (key, scale, title) in zip(axes.ravel(), specs):
        values = [float(row[key]) * scale for row in rows
                  if isinstance(row.get(key), (int, float)) and float(row[key]) >= 0]
        axis.hist(values, bins=8, color=ACCENT, edgecolor=PAPER)
        axis.set_title(title if not values else
                       f'{title}    中位 {float(np.median(values)):.1f}')
        _bare(axis)
    allowed = sum(bool(row.get('allowed')) for row in rows)
    figure.suptitle(f'重建几何    许可 {allowed}/{len(rows)}', fontsize=13)
    _save(figure, path)


def plot_overlays(pairs: list[tuple[Path, Path, str]], path: Path) -> None:
    if not pairs:
        return
    columns = 4
    rows = int(np.ceil(len(pairs) / columns))
    figure, axes = plt.subplots(rows, columns, figsize=(columns * 2.5, rows * 2.6))
    flat = np.atleast_1d(axes).ravel()
    for axis, (color, mask, batch) in zip(flat, pairs):
        axis.imshow(_overlay(color, mask))
        axis.set_title(batch.replace('flow10_', ''), fontsize=8)
        axis.axis('off')
    for axis in flat[len(pairs):]:
        axis.axis('off')
    figure.suptitle('检测分割：彩色图与掩膜', fontsize=13)
    _save(figure, path)


def plot_cdf(rows: list[dict], path: Path) -> None:
    """单果周期的经验累积分布。田间论文用它看尾部，而不只看平均值。"""
    elapsed = sorted(float(row['elapsed_s']) for row in rows
                     if int(row.get('outcome', -1)) == 0 and row.get('elapsed_s'))
    if not elapsed:
        return
    figure, axis = plt.subplots(figsize=(6.4, 4.2))
    rank = np.arange(1, len(elapsed) + 1) / len(elapsed)
    axis.step(elapsed, rank, where='post', color=ACCENT, linewidth=1.8)
    axis.axhline(0.5, color=MUTED, linewidth=0.8)
    axis.axvline(float(np.median(elapsed)), color=INK, linewidth=1.0)
    axis.set_ylim(0, 1.02)
    axis.set_xlabel('单果周期 / 秒')
    axis.set_ylabel('累积比例')
    axis.set_title(f'周期累积分布    P50 {float(np.median(elapsed)):.1f} s')
    _bare(axis)
    _save(figure, path)


def plot_workspace(geometry: list[dict], outcomes: list[dict], path: Path) -> None:
    """袋心在基座系水平面的位置，按该果终局着色。"""
    status = {(row['batch'], row.get('target_id')): int(row.get('outcome', -1))
              for row in outcomes}
    figure, axis = plt.subplots(figsize=(6.2, 5.4))
    for row in geometry:
        bottom, neck = row.get('bag_bottom'), row.get('bag_neck')
        if not (isinstance(bottom, list) and isinstance(neck, list)):
            continue
        center = 0.5 * (np.asarray(bottom[:3]) + np.asarray(neck[:3]))
        code = status.get((row.get('batch'), row.get('target_id')), 0)
        color = ACCENT if code == 0 else BAD
        axis.scatter(center[0], center[1], s=46, color=color, zorder=3)
    axis.set_aspect('equal', adjustable='datalim')
    axis.set_xlabel('x / m')
    axis.set_ylabel('y / m')
    axis.set_title('袋心分布    绿=周期完成  红=未完成')
    _bare(axis)
    _save(figure, path)


def plot_margins(geometry: list[dict], path: Path) -> None:
    """径向、轴向余量。负值表示按当前刀具预算装不进。"""
    radial, axial = [], []
    for row in geometry:
        if isinstance(row.get('radial_margin_m'), (int, float)) and isinstance(
                row.get('axial_margin_m'), (int, float)):
            radial.append(float(row['radial_margin_m']) * 1000.0)
            axial.append(float(row['axial_margin_m']) * 1000.0)
    if not radial:
        return
    figure, axis = plt.subplots(figsize=(6.2, 5.2))
    axis.axvline(0, color=MUTED, linewidth=0.8)
    axis.axhline(0, color=MUTED, linewidth=0.8)
    axis.scatter(radial, axial, s=42, color=ACCENT)
    axis.set_xlabel('径向余量 / mm')
    axis.set_ylabel('轴向余量 / mm')
    axis.set_title('刀具余量    负值在轴线左侧或下方')
    _bare(axis)
    _save(figure, path)


def plot_skips(events: list[dict], path: Path) -> None:
    """重建拒帧原因。采帧效率按接受帧 /（接受+拒绝）计。"""
    labels = {
        'missing_mask': '缺掩膜',
        'target_drift': '目标漂移',
        'neighbor_gap': '邻距',
        'near_duplicate': '重复视角',
        'static_motion': '未静止',
        'joint_state_unavailable': '无关节',
    }
    skipped = [event for event in events if event.get('event') == 'frame_skipped']
    accepted = sum(event.get('event') == 'frame_accepted' for event in events)
    if not skipped and not accepted:
        return
    counts = Counter(labels.get(event.get('code'), event.get('code') or '其他')
                     for event in skipped)
    names = [name for name, _count in counts.most_common()]
    values = [count for _name, count in counts.most_common()]
    figure, axis = plt.subplots(figsize=(7.2, 4.2))
    axis.barh(names, values, color=WARN)
    total = accepted + len(skipped)
    rate = accepted / total if total else 0.0
    axis.set_xlabel('帧数')
    axis.set_title(f'重建拒帧    接受 {accepted}/{total} = {rate:.0%}')
    _bare(axis)
    _save(figure, path)


def plot_masks(samples: list[dict], path: Path) -> None:
    """掩膜占画面的比例，以及质心是否挤在视野一侧。"""
    if not samples:
        return
    figure, axes = plt.subplots(1, 2, figsize=(9.2, 4.2))
    areas = [100.0 * sample['area'] for sample in samples]
    axes[0].hist(areas, bins=10, color=ACCENT, edgecolor=PAPER)
    axes[0].set_xlabel('掩膜占画面 %')
    axes[0].set_ylabel('张数')
    axes[0].set_title(f'掩膜面积    中位 {float(np.median(areas)):.1f}%')
    axes[1].scatter([sample['u'] for sample in samples],
                    [1.0 - sample['v'] for sample in samples],
                    s=18, color=ACCENT, alpha=0.75)
    axes[1].set_xlim(0, 1)
    axes[1].set_ylim(0, 1)
    axes[1].set_xlabel('画面横向')
    axes[1].set_ylabel('画面向上')
    axes[1].set_title('掩膜质心')
    axes[1].set_aspect('equal')
    for axis in axes:
        _bare(axis)
    _save(figure, path)


def _write_tables(out: Path, rows: list[dict], geometry: list[dict],
                  events: list[dict], masks: list[dict]) -> None:
    with (out / 'outcomes.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            'batch', 'target_id', 'outcome', 'outcome_name', 'completion_level',
            'elapsed_s', 'reason'))
        writer.writeheader()
        for row in rows:
            code = int(row.get('outcome', -1))
            writer.writerow({
                'batch': row.get('batch'),
                'target_id': row.get('target_id'),
                'outcome': code,
                'outcome_name': OUTCOME_NAME.get(code, ''),
                'completion_level': row.get('completion_level'),
                'elapsed_s': row.get('elapsed_s'),
                'reason': row.get('reason'),
            })
    done = [row for row in rows if int(row.get('outcome', -1)) == 0]
    elapsed = [float(row['elapsed_s']) for row in done if row.get('elapsed_s')]
    summary = {
        'fruits': len(rows),
        'motion_completed': len(done),
        'motion_completed_rate': (len(done) / len(rows)) if rows else None,
        'elapsed_s_median': float(np.median(elapsed)) if elapsed else None,
        'geometry_rows': len(geometry),
        'geometry_allowed': sum(bool(row.get('allowed')) for row in geometry),
        'frames_accepted': sum(event.get('event') == 'frame_accepted' for event in events),
        'frames_skipped': sum(event.get('event') == 'frame_skipped' for event in events),
        'mask_images': len(masks),
        'mask_area_percent_median': (
            float(np.median([100.0 * sample['area'] for sample in masks])) if masks else None),
        'note': '运动周期完成不是切断确认。本记录没有检测框真值，不算 precision/recall。',
    }
    (out / 'summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')


def analyze(run: Path, out: Path) -> dict:
    _font()
    roots = collect(run)
    ledgers = _ledgers(roots)
    rows = _rows(ledgers)
    geometry = _geometry(roots)
    events = _events(roots)
    masks = _mask_samples(roots)
    if not rows:
        raise SystemExit(f'{run} 下没有 ledger.json')
    out.mkdir(parents=True, exist_ok=True)
    plot_outcomes(rows, out / '01_outcomes.png')
    plot_cycle(rows, out / '02_cycle_time.png')
    plot_rounds(rows, out / '03_rounds.png')
    if geometry:
        plot_geometry(geometry, out / '04_geometry.png')
    plot_overlays(_pairs(roots), out / '05_detection.png')
    plot_cdf(rows, out / '06_cycle_cdf.png')
    if geometry:
        plot_workspace(geometry, rows, out / '07_workspace.png')
        plot_margins(geometry, out / '08_margins.png')
    plot_skips(events, out / '09_frame_skips.png')
    plot_masks(masks, out / '10_mask_layout.png')
    _write_tables(out, rows, geometry, events, masks)
    return json.loads((out / 'summary.json').read_text(encoding='utf-8'))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, help='一次运行的 session 目录，或含 ledger.json 的目录')
    parser.add_argument('--out', type=Path, default=None, help='图和表的输出目录，默认 <run>/analysis')
    args = parser.parse_args()
    run = args.run.resolve()
    out = args.out.resolve() if args.out else run / 'analysis'
    summary = analyze(run, out)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
