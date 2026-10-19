"""Population depth/size of sim bags vs PeachDataSet, not one registered frame.

Pixel MAE on frame 1200 only checks that one reconstructed patch. The dataset
has no camera poses for the other frames, so the comparable quantity is the
per-box optical-depth and apparent-size distribution (same filters as
measure_priors).

Near sim bags: optical Z in [0.25, 1.30] m (dataset p10–p90 is 0.33–1.13 m).
Orchard/survey views fall outside that window and are reported separately.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ['OPENCV_IO_ENABLE_OPENEXR'] = '1'

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

HERE = Path(__file__).resolve().parent
import sys  # noqa: E402
sys.path.insert(0, str(HERE))
from depth_io import camera_look_axis, optical_depth_m  # noqa: E402
from measure_priors import (DATASET, FX, FY, H, W, _boxes,  # noqa: E402
                            _percentiles)
from render_evidence import validate_render_set  # noqa: E402
from viewpoints import PRIMARY_DIST_M  # noqa: E402

NEAR_M = (0.25, 1.30)
MIN_PIXELS = 100
MIN_BOX_PX = 20


def real_bag_samples(split='Peach_bag', limit=250, class_id='0'):
    """Same box/depth filters as measure_priors.measure_split."""
    rgb_dir = DATASET / split / 'RGB'
    depth_dir = DATASET / split / 'Depth'
    ann_dir = DATASET / split / 'Annotations_VOC' / 'VOC_4label'
    names = sorted(p.name for p in ann_dir.iterdir() if p.name.endswith('.xml'))
    step = max(1, len(names) // limit)
    names = names[::step][:limit]
    samples = []
    for name in names:
        stem = name[:-4]
        depth_path = depth_dir / f'{stem}.png'
        if not depth_path.exists():
            continue
        depth = np.asarray(Image.open(depth_path))
        for cls, x1, y1, x2, y2 in _boxes(ann_dir / name):
            if cls != class_id:
                continue
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(W - 1, x2), min(H - 1, y2)
            if x2 - x1 < 8 or y2 - y1 < 8:
                continue
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            patch = depth[
                cy - (y2 - y1) // 6: cy + (y2 - y1) // 6 + 1,
                cx - (x2 - x1) // 6: cx + (x2 - x1) // 6 + 1]
            valid = patch[(patch > 200) & (patch < 2500)]
            if valid.size < 8:
                continue
            z = float(np.median(valid)) / 1000.0
            samples.append({
                'frame': stem,
                'depth_m': z,
                'width_m': (x2 - x1) / FX * z,
                'height_m': (y2 - y1) / FY * z,
            })
    return samples


def sim_bag_samples_matrix(matrix_dir: Path, lighting: str, traj: dict):
    """Bags visible in a matrix folder; camera pose from trajectory.json."""
    by_id = {v['view_id']: v for v in traj['views']}
    bag_ids = {int(k) for k in traj.get('target_registry', {})}
    names = {int(k): info.get('name', '')
             for k, info in traj.get('target_registry', {}).items()}
    samples = []
    root = matrix_dir / lighting
    if not root.is_dir():
        return samples
    for view_dir in sorted(root.iterdir()):
        if not view_dir.is_dir():
            continue
        spec = by_id.get(view_dir.name)
        if spec is None or spec.get('kind') == 'alley_stop':
            continue
        pos_path = view_dir / 'Position_0001.exr'
        id_path = view_dir / 'IndexOB_0001.exr'
        if not pos_path.is_file() or not id_path.is_file():
            continue
        look = _unit([spec['lookat'][i] - spec['eye'][i] for i in range(3)])
        pos = cv2.imread(str(pos_path), cv2.IMREAD_UNCHANGED)
        id_exr = cv2.imread(str(id_path), cv2.IMREAD_UNCHANGED)
        if pos is None or id_exr is None:
            continue
        ids = id_exr[:, :, 0].round().astype(int)
        depth = optical_depth_m(pos[:, :, :3], spec['eye'], look)
        visible = set(int(x) for x in np.unique(ids) if int(x) in bag_ids)
        for tid in sorted(visible):
            mask = ids == tid
            n = int(mask.sum())
            if n < MIN_PIXELS:
                continue
            ys, xs = np.where(mask)
            box_w = int(xs.max()) - int(xs.min()) + 1
            box_h = int(ys.max()) - int(ys.min()) + 1
            if max(box_w, box_h) < MIN_BOX_PX:
                continue
            z = float(np.median(depth[mask]))
            if not np.isfinite(z) or z <= 0:
                continue
            kind = ('reference_patch'
                    if str(names.get(tid, '')).startswith('Reference/')
                    else spec.get('kind', 'wrap'))
            band = 'near' if NEAR_M[0] <= z <= NEAR_M[1] else 'survey'
            samples.append({
                'view': view_dir.name,
                'id': tid,
                'kind': kind,
                'band': band,
                'depth_m': z,
                'width_m': box_w / FX * z,
                'height_m': box_h / FY * z,
                'pixels': n,
            })
    return samples


def _unit(vec):
    n = sum(x * x for x in vec) ** 0.5
    return [x / n for x in vec]


def sim_bag_samples(out: Path, manifest: dict, views):
    samples = []
    for view in views:
        cam = manifest['cameras'][view]
        look = camera_look_axis(cam['matrix_world'])
        pos = cv2.imread(
            str(out / f'{view}_Position_0001.exr'), cv2.IMREAD_UNCHANGED)
        ids = cv2.imread(
            str(out / f'{view}_IndexOB_0001.exr'),
            cv2.IMREAD_UNCHANGED)[:, :, 0].round().astype(int)
        depth = optical_depth_m(pos[:, :, :3], cam['location'], look)
        for target in manifest['targets']:
            tid = target['id']
            mask = ids == tid
            n = int(mask.sum())
            if n < MIN_PIXELS:
                continue
            ys, xs = np.where(mask)
            box_w = int(xs.max()) - int(xs.min()) + 1
            box_h = int(ys.max()) - int(ys.min()) + 1
            if max(box_w, box_h) < MIN_BOX_PX:
                continue
            z = float(np.median(depth[mask]))
            if not np.isfinite(z) or z <= 0:
                continue
            kind = 'reference_patch' if str(
                target.get('name', '')).startswith('Reference/') else 'wrap'
            band = 'near' if NEAR_M[0] <= z <= NEAR_M[1] else 'survey'
            samples.append({
                'view': view,
                'id': tid,
                'kind': kind,
                'band': band,
                'depth_m': z,
                'width_m': box_w / FX * z,
                'height_m': box_h / FY * z,
                'pixels': n,
            })
    return samples


def wrap_physical_widths(manifest):
    return [float(t['width_m']) for t in manifest['targets']
            if t.get('dimension_fit') == 'fruit_wrap' and t.get('width_m')]


def in_span(x, lo, hi):
    return lo <= x <= hi


def conforming_sim(sim, real, traj):
    """Keep only dataset-like close-ups; drop the rest.

    A primary view is dropped if its aimed bag is outside PRIMARY_DIST_M
    (retreated cameras are not Peach_bag distances). Remaining bags must
    sit in PRIMARY_DIST_M for depth and the real p10–p90 support for
    apparent width. Empty 1200 reference patches are never kept.
    """
    d = _percentiles([s['depth_m'] for s in real])
    w = _percentiles([s['width_m'] for s in real])
    by_id = {v['view_id']: v for v in (traj or {}).get('views', [])}
    by_view = {}
    for sample in sim:
        by_view.setdefault(sample['view'], []).append(sample)
    kept, dropped = [], []
    for view, rows in by_view.items():
        spec = by_id.get(view)
        if spec and spec.get('kind') == 'primary':
            tids = spec.get('target_ids') or ()
            aimed_id = int(tids[0]) if tids else None
            aimed = next((r for r in rows if r['id'] == aimed_id), None)
            if aimed is None or not in_span(
                    aimed['depth_m'], PRIMARY_DIST_M[0], PRIMARY_DIST_M[1]):
                dropped.extend(rows)
                continue
        for row in rows:
            if row['kind'] == 'reference_patch':
                dropped.append(row)
                continue
            if (in_span(row['depth_m'], PRIMARY_DIST_M[0], PRIMARY_DIST_M[1])
                    and in_span(row['width_m'], w['p10'], w['p90'])):
                kept.append(row)
            else:
                dropped.append(row)
    return kept, dropped


def _hist_overlay(ax, series, xlabel, bins):
    colors = ('#3d7ea6', '#c45c26', '#6a8f4e')
    for (label, values), color in zip(series, colors):
        if not values:
            continue
        ax.hist(values, bins=bins, density=True, alpha=0.55, color=color,
                label=f'{label} n={len(values)}')
    ax.set_xlabel(xlabel)
    ax.set_ylabel('density')
    ax.legend(frameon=False, fontsize=8)


def draw_board(real, sim_near, report, dest: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    real_d = [s['depth_m'] for s in real]
    sim_d = [s['depth_m'] for s in sim_near]
    real_w = [s['width_m'] for s in real]
    sim_w = [s['width_m'] for s in sim_near]
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 4.3), dpi=140)
    _hist_overlay(
        axes[0],
        [('Peach_bag real', real_d), ('sim kept', sim_d)],
        'optical depth (m)', np.linspace(0.2, 1.4, 25))
    axes[0].set_title(
        f'kept depth in primary window  [{PRIMARY_DIST_M[0]:.2f}, '
        f'{PRIMARY_DIST_M[1]:.2f}] m', fontsize=8)
    _hist_overlay(
        axes[1],
        [('Peach_bag real', real_w), ('sim kept', sim_w)],
        'apparent width (m)  = box_px · z / fx', np.linspace(0.02, 0.28, 25))
    axes[1].set_title(
        f'kept width in real p10–p90  [{report["keep_width_m"][0]*100:.1f}, '
        f'{report["keep_width_m"][1]*100:.1f}] cm', fontsize=8)
    rd, sd = report['depth_m']['real'], report['depth_m']['sim_near']
    rw, sw = report['width_m']['real'], report['width_m']['sim_near']
    fig.suptitle(
        'Peach_bag vs sim close-ups that fit the real support\n'
        f'depth p50 {rd["p50"]:.2f} vs {sd["p50"]:.2f} m    '
        f'apparent p50 {rw["p50"]*100:.1f} vs {sw["p50"]*100:.1f} cm    '
        f'dropped {report["counts"]["sim_dropped"]}',
        fontsize=10)
    fig.tight_layout()
    fig.savefig(dest, bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, default=HERE / 'output')
    parser.add_argument('--views', default='reference,detail,orchard')
    parser.add_argument('--limit', type=int, default=250)
    parser.add_argument('--matrix', type=Path, default=HERE / 'output' / 'depth_pop')
    parser.add_argument('--matrix-lighting', default='noon')
    parser.add_argument('--min-sim-near', type=int, default=10)
    args = parser.parse_args()
    out = args.out
    views = tuple(args.views.split(','))
    manifest = json.loads((out / 'scene_manifest.json').read_text())
    validate_render_set(out, manifest, views)

    real = real_bag_samples(limit=args.limit)
    sim = sim_bag_samples(out, manifest, views)
    traj_path = args.matrix / 'trajectory.json'
    if traj_path.is_file():
        traj = json.loads(traj_path.read_text())
        sim.extend(sim_bag_samples_matrix(
            args.matrix, args.matrix_lighting, traj))
    else:
        traj = None
    wrap_w = wrap_physical_widths(manifest)
    d_real = _percentiles([s['depth_m'] for s in real])
    w_real = _percentiles([s['width_m'] for s in real])
    sim_near, sim_dropped = conforming_sim(sim, real, traj or {})
    sim_survey = [s for s in sim if s['band'] == 'survey']
    sim_ref = [s for s in sim if s['kind'] == 'reference_patch']
    if len(real) < 50 or len(sim_near) < args.min_sim_near:
        raise RuntimeError(
            f'too few samples real={len(real)} sim_near={len(sim_near)} '
            f'(need ≥{args.min_sim_near}; render noon primaries into '
            f'{args.matrix})')

    report = {
        'near_window_m': list(NEAR_M),
        'keep_depth_m': list(PRIMARY_DIST_M),
        'keep_width_m': [w_real['p10'], w_real['p90']],
        'primary_dist_m': list(PRIMARY_DIST_M),
        'views': list(views),
        'matrix': str(args.matrix) if traj_path.is_file() else None,
        'real_frames_sampled': args.limit,
        'counts': {
            'real': len(real),
            'sim_near': len(sim_near),
            'sim_dropped': len(sim_dropped),
            'sim_survey': len(sim_survey),
            'sim_reference_patch': len(sim_ref),
            'wrap_physical': len(wrap_w),
            'sim_near_by_kind': {},
            'sim_near_by_view': {
                k: sum(1 for s in sim_near if s['view'] == k)
                for k in sorted({s['view'] for s in sim_near})},
        },
        'depth_m': {
            'real': d_real,
            'sim_near': _percentiles([s['depth_m'] for s in sim_near]),
        },
        'width_m': {
            'real': w_real,
            'sim_near': _percentiles([s['width_m'] for s in sim_near]),
            'wrap_physical': _percentiles(wrap_w),
        },
        'notes': [
            'Dropped sim bags/views that do not fit PRIMARY_DIST_M depth '
            '(0.55–0.75 m) or real Peach_bag p10–p90 apparent width, and '
            'primary views whose aimed bag is outside that depth window.',
            'Empty 1200 reference patches are not kept. Frame 1200 MAE is '
            'not a population check.',
            'Sim optical Z is camera-forward from Position and lookat−eye.',
        ],
    }
    by_kind = {}
    for s in sim_near:
        by_kind[s['kind']] = by_kind.get(s['kind'], 0) + 1
    report['counts']['sim_near_by_kind'] = by_kind
    json_path = out / 'dataset_depth_comparison.json'
    json_path.write_text(json.dumps(report, indent=2))
    board = out / 'dataset_depth_comparison.jpg'
    draw_board(real, sim_near, report, board)
    print(json.dumps({
        'real': report['counts']['real'],
        'sim_near': report['counts']['sim_near'],
        'sim_dropped': report['counts']['sim_dropped'],
        'sim_near_by_kind': by_kind,
        'depth_p50_m': [report['depth_m']['real']['p50'],
                        report['depth_m']['sim_near']['p50']],
        'width_p50_m': [report['width_m']['real']['p50'],
                        report['width_m']['sim_near']['p50']],
        'board': str(board),
    }, indent=2), flush=True)


if __name__ == '__main__':
    main()
