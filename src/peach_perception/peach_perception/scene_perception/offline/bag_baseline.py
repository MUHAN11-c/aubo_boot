"""离线复算：同一目标多视角袋底/袋颈/轴离散度与动态净空."""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

import numpy as np

from peach_perception.common.tool_budget import evaluate_sleeve_cut


def _load_jsonl(path: Path) -> list:
    """读取 jsonl；缺文件返回空列表."""
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text(encoding='utf-8').splitlines():
        text = line.strip()
        if not text:
            continue
        try:
            rows.append(json.loads(text))
        except json.JSONDecodeError:
            continue
    return rows


def _vec(value) -> np.ndarray | None:
    """list/tuple → (3,)；失败 None."""
    if value is None:
        return None
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.size < 3 or not np.all(np.isfinite(arr[:3])):
        return None
    return arr[:3]


def _angle_deg(a, b) -> float:
    """两向量夹角（度）."""
    first, second = _vec(a), _vec(b)
    if first is None or second is None:
        return float('nan')
    n1 = np.linalg.norm(first)
    n2 = np.linalg.norm(second)
    if n1 < 1e-9 or n2 < 1e-9:
        return float('nan')
    cosine = float(np.clip(abs(np.dot(first / n1, second / n2)), 0.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def _spread_m(points) -> dict:
    """点集 RMS / P95 散布 [m]."""
    pts = [p for p in points if p is not None]
    if len(pts) < 2:
        return {'n': len(pts), 'rms_m': None, 'p95_m': None}
    arr = np.stack(pts)
    mean = arr.mean(axis=0)
    dist = np.linalg.norm(arr - mean, axis=1)
    return {
        'n': int(arr.shape[0]),
        'rms_m': float(np.sqrt(np.mean(dist ** 2))),
        'p95_m': float(np.percentile(dist, 95)),
    }


def summarize_run(run_dir: Path) -> dict:
    """汇总一个 runs/<id> 目录的几何 jsonl."""
    rows = []
    rows.extend(_load_jsonl(run_dir / 'geometry.jsonl'))
    for path in sorted((run_dir / 'targets').glob('*/observations.jsonl')):
        rows.extend(_load_jsonl(path))
    grouped = defaultdict(list)
    for row in rows:
        tid = str(row.get('target_id') or '')
        if tid:
            grouped[tid].append(row)
    targets = {}
    angles = []
    d95s = []
    budgets = []
    for tid, items in grouped.items():
        bottoms = [_vec(item.get('bag_bottom') or item.get('bottom'))
                   for item in items]
        necks = [_vec(item.get('bag_neck') or item.get('neck'))
                 for item in items]
        axes = [_vec(item.get('axis') or item.get('bag_axis'))
                for item in items]
        pairwise = []
        finite_axes = [a for a in axes if a is not None]
        for i in range(len(finite_axes)):
            for j in range(i + 1, len(finite_axes)):
                pairwise.append(_angle_deg(finite_axes[i], finite_axes[j]))
        axis_p95 = None
        if pairwise:
            axis_p95 = float(np.nanpercentile(pairwise, 95))
            angles.extend(pairwise)
        d95_vals = [float(item['d95_m']) for item in items
                    if item.get('d95_m') is not None]
        if d95_vals:
            d95s.extend(d95_vals)
        for item in items:
            d95 = float(item.get('d95_m') or 0.0)
            if d95 <= 0.0:
                continue
            lat = float(item.get('sigma_position_m') or 0.01)
            ang = float(item.get('sigma_axis_deg') or 8.0)
            length = float(item.get('length_m') or 0.12)
            budget = evaluate_sleeve_cut(
                d_bag95=d95, length_m=length, center_lateral95=lat,
                axis_error_deg=ang, neck_position95=lat,
                cut_to_fruit_m=float(item.get('cut_to_fruit_m') or 0.03))
            budgets.append(budget)
        targets[tid] = {
            'views': len(items),
            'bottom': _spread_m(bottoms),
            'neck': _spread_m(necks),
            'axis_pairwise_p95_deg': axis_p95,
        }

    def _pct(values, q):
        finite = [v for v in values if v is not None and math.isfinite(v)]
        if not finite:
            return None
        return float(np.percentile(finite, q))

    return {
        'run_dir': str(run_dir),
        'targets': targets,
        'axis_angle_p50_deg': _pct(angles, 50),
        'axis_angle_p95_deg': _pct(angles, 95),
        'd95_p50_m': _pct(d95s, 50),
        'd95_p95_m': _pct(d95s, 95),
        'budget_reject_ratio': (
            None if not budgets else
            float(np.mean([not b['allowed'] for b in budgets]))
        ),
        'n_geometry_rows': len(rows),
    }


def main(argv=None) -> int:
    """命令行：对 runs 根下各批次做几何基线."""
    parser = argparse.ArgumentParser(
        description='套袋几何多视角离散度与动态净空门')
    parser.add_argument(
        '--runs', type=Path, default=Path('runs'),
        help='过程数据根目录')
    parser.add_argument(
        '--output', type=Path, default=None,
        help='可选 JSON 输出路径')
    args = parser.parse_args(argv)
    reports = []
    root = args.runs
    if root.is_dir():
        children = [p for p in root.iterdir() if p.is_dir()]
        if not children:
            reports.append(summarize_run(root))
        for child in sorted(children):
            reports.append(summarize_run(child))
    text = json.dumps(reports, ensure_ascii=False, indent=2)
    if args.output is not None:
        args.output.write_text(text + '\n', encoding='utf-8')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
