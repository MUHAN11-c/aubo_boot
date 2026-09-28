"""矩阵评测：渲染矩阵 → YOLO/MobileSAM 对 IndexOB GT 分层聚合.

分层维度：光照预设 × 遮挡档（none/light/heavy/reference，含渲染深度
反测的真实覆盖率）× 视角类型（primary/supplemental 近距、alley_stop
远眺 survey）。袋级多视聚合按"同一光照内的 3 视序列"计（≥1 / ≥2 视角
检出率，对齐产线一次停靠的观察节拍）。SAM 仅在 primary 视做框提示
掩膜 IoU（GT 框提示口径不变，不冒充端到端分割召回）。深度为理想渲染
值（无噪声模型），逐视导出 depth_mm.png（uint16 毫米）。

用法：
  aubo_py3.12/bin/python src/peach_sim/reconstruction/evaluate_matrix.py \
      [--matrix output/matrix] [--gate | --write-baseline]
"""

import argparse
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

os.environ['OPENCV_IO_ENABLE_OPENEXR'] = '1'

import cv2  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
from depth_io import to_uint16_mm  # noqa: E402

CONF_LEVELS = (.25, .35)
BOX_IOU = .5
MIN_PIXELS = 100
MIN_BOX_PX = 20
OCCLUSION_MARGIN_M = .02
COVERAGE_BUCKETS = ((0.0, .2, 'low'), (.2, .5, 'mid'), (.5, 1.01, 'high'))
"""名义档会被自然冠层本底淹没（实测 none≈0.38、light≈0.14 倒挂），
主分层用深度反测的连续覆盖率桶；名义档仍记录在 manifest 与明细里."""
BASELINE_PATH = HERE / 'baselines' / 'perception_matrix_baseline.json'
GATE_TOLERANCE = .05
LEVELS = ('none', 'light', 'heavy', 'reference')


def coverage_bucket(value):
    if value is None:
        return None
    for low, high, name in COVERAGE_BUCKETS:
        if low <= value < high:
            return name
    return 'high'


def iou(a, b):
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    return inter / max(1, (a[2] - a[0]) * (a[3] - a[1]) +
                       (b[2] - b[0]) * (b[3] - b[1]) - inter)


def load_exr(path):
    img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(path)
    return img


def visible_gt(ids, registry):
    gt = []
    for tid, info in registry.items():
        mask = ids == tid
        ys, xs = np.where(mask)
        if len(xs) < MIN_PIXELS:
            continue
        box = [int(xs.min()), int(ys.min()), int(xs.max() + 1),
               int(ys.max() + 1)]
        if max(box[2] - box[0], box[3] - box[1]) < MIN_BOX_PX:
            continue
        gt.append({'id': tid, 'box': box, 'level': info['occlusion']})
    return gt


def measure_occlusion(ids, depth, box, tid):
    """袋框内被更近物体遮住的像素占比（叶无实例 ID，用深度序反测）."""
    y0, y1 = max(0, box[1]), min(ids.shape[0], box[3])
    x0, x1 = max(0, box[0]), min(ids.shape[1], box[2])
    own = (ids[y0:y1, x0:x1] == tid)
    valid = own & (depth[y0:y1, x0:x1] > 0)
    if valid.sum() < 20:
        return None
    bag_depth = float(np.median(depth[y0:y1, x0:x1][valid]))
    region = depth[y0:y1, x0:x1]
    occluded = (region > 0) & (region < bag_depth - OCCLUSION_MARGIN_M)
    return float(occluded.sum() / max(1, region.size))


def match(gt, detections, bag_classes, conf):
    cand = []
    for j, g in enumerate(gt):
        for k, d in enumerate(detections):
            if d['class'] not in bag_classes or d['confidence'] < conf:
                continue
            overlap = iou(g['box'], d['box'])
            if overlap >= BOX_IOU:
                cand.append((overlap, j, k))
    used_g, used_d = set(), set()
    hits = set()
    for _, j, k in sorted(cand, key=lambda c: -c[0]):
        if j in used_g or k in used_d:
            continue
        used_g.add(j)
        used_d.add(k)
        hits.add(gt[j]['id'])
    return hits


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--matrix', default=str(HERE / 'output/matrix'))
    parser.add_argument('--gate', action='store_true',
                        help='compare against frozen baseline, exit 1 on regress')
    parser.add_argument('--write-baseline', action='store_true',
                        help='(re)freeze current stratified recall as baseline')
    args = parser.parse_args()

    import torch
    torch.set_num_threads(6)
    from ultralytics import SAM, YOLO
    det = YOLO(str(ROOT / 'src/peach_harvester/model/best.pt'))
    sam = SAM(str(ROOT / 'src/peach_harvester/model/mobile_sam.pt'))
    bag_classes = {int(k) for k, v in det.names.items()
                   if 'bag' in v and 'nobag' not in v}

    matrix = Path(args.matrix)
    traj = json.loads((matrix / 'trajectory.json').read_text())
    registry = {int(k): v for k, v in traj['target_registry'].items()}
    lightings = traj['lighting_applied']
    views = traj['views']

    # (lighting, view_kind, view_id) rows; detail only at conf .35
    view_rows = []
    sam_scores = []
    gt_cache = {}
    for view in views:
        vid = view['view_id']
        gt = None
        for lname in lightings:
            vdir = matrix / lname / vid
            rgb_path = vdir / 'rgb.png'
            ids_path = vdir / 'IndexOB_0001.exr'
            if not rgb_path.exists() or not ids_path.exists():
                continue
            rgb = str(rgb_path)
            ids = load_exr(ids_path)[:, :, 0].round().astype(int)
            gt = gt_cache.setdefault(vid, visible_gt(ids, registry))
            depth = load_exr(vdir / 'Depth_0001.exr')[:, :, 0]
            depth = np.where(np.isfinite(depth), depth, 0)
            cv2.imwrite(str(vdir / 'depth_mm.png'), to_uint16_mm(depth))

            pred = det(rgb, conf=min(CONF_LEVELS), device='cpu',
                       verbose=False)[0]
            detections = [{'box': b.xyxy[0].tolist(),
                           'confidence': float(b.conf[0]),
                           'class': int(b.cls[0])} for b in pred.boxes]
            row = {'lighting': lname, 'view': vid, 'kind': view['kind'],
                   'eye': view['eye'], 'gt': len(gt),
                   'detections': len(detections)}
            hits = {}
            for conf in CONF_LEVELS:
                matched = match(gt, detections, bag_classes, conf)
                hits[conf] = matched
                row[f'matches@{conf}'] = len(matched)
                row[f'recall@{conf}'] = len(matched) / len(gt) if gt else None
            detail = []
            for g in gt:
                center = registry[g['id']]['center_world']
                dist = float(np.linalg.norm(
                    np.array(view['eye']) - np.array(center)))
                coverage = measure_occlusion(ids, depth, g['box'], g['id'])
                detail.append({
                    'id': g['id'],
                    'level': g['level'],
                    'dist_m': round(dist, 2),
                    'occlusion_actual': coverage,
                    'coverage_bucket': coverage_bucket(coverage),
                    'matched@0.35': g['id'] in hits[.35]})
            row['detail'] = detail
            view_rows.append(row)

            if view['kind'] == 'primary' and gt:
                boxes = [g['box'] for g in gt[:16]]
                seg = sam(rgb, bboxes=boxes, device='cpu',
                          verbose=False)[0]
                if seg.masks is not None:
                    masks = seg.masks.data.cpu().numpy() > .5
                    for g, m in zip(gt[:16], masks):
                        if m.shape != ids.shape:
                            m = cv2.resize(
                                m.astype('uint8'),
                                (ids.shape[1], ids.shape[0]),
                                interpolation=cv2.INTER_NEAREST) > 0
                        target = ids == g['id']
                        sam_scores.append({
                            'lighting': lname, 'view': vid, 'id': g['id'],
                            'level': g['level'],
                            'prompted_sam_iou': float(
                                (m & target).sum() /
                                max(1, (m | target).sum()))})

    # ---------- aggregate ----------
    def recall(rows, conf):
        gt = sum(r['gt'] for r in rows)
        m = sum(r[f'matches@{conf}'] for r in rows)
        return {'gt': gt, 'matched': m,
                'recall': round(m / gt, 4) if gt else None}

    cells = {}
    for conf in CONF_LEVELS:
        for lname in lightings:
            for kind in ('primary', 'supplemental', 'alley_stop'):
                rows = [r for r in view_rows
                        if r['lighting'] == lname and r['kind'] == kind]
                if rows:
                    a = recall(rows, conf)
                    if a['recall'] is not None:
                        cells[f'recall@{conf}|{lname}|{kind}'] = a['recall']
    # occlusion-level cells at conf .35 over near views
    for lname in lightings:
        for level in LEVELS:
            det_rows = [d for r in view_rows
                        if r['lighting'] == lname
                        and r['kind'] in ('primary', 'supplemental')
                        for d in r['detail'] if d['level'] == level]
            if det_rows:
                cells[f'recall@0.35|{lname}|level={level}'] = round(
                    sum(1 for d in det_rows if d['matched@0.35'])
                    / len(det_rows), 4)
        # measured-coverage buckets are the primary stratification
        for bucket in ('low', 'mid', 'high'):
            det_rows = [d for r in view_rows
                        if r['lighting'] == lname
                        and r['kind'] in ('primary', 'supplemental')
                        for d in r['detail']
                        if d['coverage_bucket'] == bucket]
            if det_rows:
                cells[f'recall@0.35|{lname}|cov={bucket}'] = round(
                    sum(1 for d in det_rows if d['matched@0.35'])
                    / len(det_rows), 4)

    # Per-target multi-view hits within one lighting (production cadence:
    # a target's 3-view sequence under a single lighting).
    hits_by_tl = defaultdict(int)
    for r in view_rows:
        if r['kind'] not in ('primary', 'supplemental'):
            continue
        for d in r['detail']:
            if d['matched@0.35']:
                hits_by_tl[(d['id'], r['lighting'])] += 1
    multi = {}
    for scope, need in (('any_view', 1), ('two_views', 2)):
        for level in LEVELS:
            ids_level = [tid for tid, t in registry.items()
                         if t['occlusion'] == level]
            # denominator = targets actually evaluated (visible somewhere
            # in a near view); numerator = (target, lighting) pairs whose
            # 3-view sequence produced >= need detections.
            evaluated = {tid for tid in ids_level
                         if any(d['id'] == tid and d['level'] == level
                                for row in view_rows
                                if row['kind'] in ('primary', 'supplemental')
                                for d in row['detail'])}
            if evaluated:
                got = sum(1 for tid in evaluated for ln in lightings
                          if hits_by_tl.get((tid, ln), 0) >= need)
                total = len(evaluated) * len(lightings)
                multi[f'{scope}|level={level}'] = round(got / total, 4)

    occlusion_calibration = {}
    for level in LEVELS:
        vals = [d['occlusion_actual'] for r in view_rows
                if r['kind'] in ('primary', 'supplemental')
                for d in r['detail']
                if d['level'] == level and d['occlusion_actual'] is not None]
        if vals:
            occlusion_calibration[level] = {
                'n': len(vals),
                'mean_measured_coverage': round(float(np.mean(vals)), 4),
                'p90': round(float(np.percentile(vals, 90)), 4)}

    sam_by_level = defaultdict(list)
    for s in sam_scores:
        sam_by_level[s['level']].append(s['prompted_sam_iou'])
    sam_summary = {level: {'n': len(v),
                           'mean_iou': round(float(np.mean(v)), 4)}
                   for level, v in sam_by_level.items() if v}

    near = sum(1 for r in view_rows
               if r['kind'] in ('primary', 'supplemental'))
    far = sum(1 for r in view_rows if r['kind'] == 'alley_stop')
    summary = {
        'conf_levels': CONF_LEVELS,
        'box_iou': BOX_IOU,
        'lightings': lightings,
        'near_view_rows': near,
        'far_view_rows': far,
        'cells': cells,
        'multi_view': multi,
        'occlusion_calibration': occlusion_calibration,
        'prompted_sam_by_level': sam_summary,
        'notes': [
            'SAM GT-box prompted on primary views only; not detector recall.',
            'Depth is ideal render (no noise model); depth_mm.png per view.',
            'alley_stop rows are out-of-distribution far survey views.',
            'multi_view aggregates each target 3-view sequence per lighting.']}
    (matrix / 'summary.json').write_text(json.dumps(summary, indent=1))

    lines = ['# 感知验证矩阵报告', '',
             f"近距视行（primary+supplemental）{near}，远眺视行 {far}，"
             f"光照 {len(lightings)} 档。", '',
             '| 分层 | recall@0.35 |', '|---|---|']
    for key in sorted(k for k in cells if k.startswith('recall@0.35')):
        lines.append(f'| {key.split("|", 1)[1]} | {cells[key]:.3f} |')
    lines += ['', '## 袋级多视聚合（conf .35，单光照内 3 视序列）', '',
              '| 口径 | 检出率 |', '|---|---|']
    for key in sorted(multi):
        lines.append(f'| {key} | {multi[key]:.3f} |')
    lines += ['', '## 遮挡标定（深度反测覆盖率）', '',
              '| 档 | n | 均值 | p90 |', '|---|---|---|---|']
    for level in LEVELS:
        c = occlusion_calibration.get(level)
        if c:
            lines.append(f"| {level} | {c['n']} | "
                         f"{c['mean_measured_coverage']:.3f} | "
                         f"{c['p90']:.3f} |")
    lines += ['', '## SAM 框提示掩膜 IoU（primary 视）', '',
              '| 档 | n | 均值 |', '|---|---|---|']
    for level, c in sorted(sam_summary.items()):
        lines.append(f"| {level} | {c['n']} | {c['mean_iou']:.3f} |")
    (matrix / 'report.md').write_text('\n'.join(lines) + '\n')

    # ---------- baseline gate ----------
    if args.write_baseline:
        BASELINE_PATH.parent.mkdir(parents=True, exist_ok=True)
        BASELINE_PATH.write_text(json.dumps(
            {'cells': {k: v for k, v in cells.items()
                       if k.startswith('recall@0.35')},
             'multi_view': multi,
             'tolerance': GATE_TOLERANCE,
             'written': str(matrix)}, indent=1))
        print(f'baseline frozen: {BASELINE_PATH}', flush=True)
    if args.gate:
        if not BASELINE_PATH.exists():
            print('GATE FAIL: no baseline; run --write-baseline first',
                  flush=True)
            raise SystemExit(1)
        base = json.loads(BASELINE_PATH.read_text())
        tol = base.get('tolerance', GATE_TOLERANCE)
        failures = []
        for key, floor in base['cells'].items():
            now = cells.get(key)
            if now is None:
                failures.append(f'{key}: missing in current run')
            elif now < floor - tol:
                failures.append(f'{key}: {now:.3f} < {floor:.3f}-{tol}')
        for key, floor in base.get('multi_view', {}).items():
            now = multi.get(key)
            if now is not None and now < floor - tol:
                failures.append(f'{key}: {now:.3f} < {floor:.3f}-{tol}')
        if failures:
            print('GATE FAIL:', *failures, sep='\n  ', flush=True)
            raise SystemExit(1)
        print('GATE PASS: no stratified regression vs baseline', flush=True)


if __name__ == '__main__':
    main()
