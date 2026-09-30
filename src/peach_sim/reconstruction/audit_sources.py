"""Inventory real multimodal evidence; do not mix indoor and orchard statistics."""
from pathlib import Path
import json
import numpy as np
from PIL import Image, ImageDraw, ImageOps
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
DATA = Path('/home/mu/Downloads/PeachDataSet')
report = {
    'dataset': {},
    'project_recordings': {},
    'limitations': [
        'Sparse deterministic samples, not a calibrated survey.',
        'Camera intrinsics from public FOV are approximate.',
        'Indoor radius estimates validate scale plausibility only; do not determine orchard bag distribution.']}
board = Image.new('RGB', (1440, 960), '#18221c')
draw = ImageDraw.Draw(board)
n = 0
for category in ['Peach_bag', 'Peach_nobag', 'Peach_young']:
    p = DATA / category
    files = sorted((p / 'RGB').glob('*.png'), key=lambda q: int(q.stem))
    rows = []
    for k in np.linspace(0, len(files) - 1, 24, dtype=int):
        f = files[k]
        d = np.array(Image.open(p / 'Depth' / f.name))
        ir = np.array(Image.open(p / 'Infrared' / f.name))
        rgb = Image.open(f)
        v = d[d > 0]
        rows.append({'id': f.stem,
                     'rgb_size': list(rgb.size),
                     'depth_shape': list(d.shape),
                     'depth_dtype': str(d.dtype),
                     'depth_nonzero_fraction': float((d > 0).mean()),
                     'depth_raw_p10_p50_p90': np.percentile(v,
                                                            [10,
                                                             50,
                                                             90]).tolist() if len(v) else None,
                     'ir_nonzero_fraction': float((ir > 0).mean())})
        if len(rows) % 3 == 1:
            x = (n % 6) * 240
            y = (n // 6) * 240
            board.paste(ImageOps.contain(rgb, (240, 200)), (x, y))
            draw.text((x + 5, y + 202), f'{category}/{f.stem}', fill='white')
            n += 1
    report['dataset'][category] = {
        'rgb_count': len(files),
        'depth_count': len(
            list(
                (p / 'Depth').glob('*.png'))),
        'ir_count': len(
            list(
                (p / 'Infrared').glob('*.png'))),
        'sampled_frames': rows}
for f in (ROOT / 'src/peach_stereo/test/data').glob('*frames.jsonl'):
    rows = [json.loads(line) for line in f.open()]
    targets = [t for r in rows for t in r.get('targets', {}).values()]
    good = [
        t for t in targets if t.get(
            'mask_depth_ratio',
            0) > .8 and t.get(
            'conf',
            0) > .7]
    report['project_recordings'][str(f.relative_to(ROOT))] = {'frames': len(rows),
                                                              'observations': len(targets),
                                                              'high_support_observations': len(good),
                                                              'diameter_mm_p10_p50_p90': np.percentile([t['radius_mm'] * 2 for t in good],
                                                                                                       [10,
                                                                                                        50,
                                                                                                        90]).tolist() if good else None,
                                                              'depth_mm_p10_p50_p90': np.percentile([t['med'] for t in good],
                                                                                                    [10,
                                                                                                     50,
                                                                                                     90]).tolist() if good else None}
(HERE / 'evidence/source_audit.json').write_text(json.dumps(report, indent=2))
board.save(HERE / 'evidence/reference_contact_sheet.jpg')
print(json.dumps({k: {'rgb_count': v['rgb_count'], 'samples': len(
    v['sampled_frames'])} for k, v in report['dataset'].items()}))
print(report['project_recordings'])

# Keep foliage measurement in one implementation; auditing must not restore
# obsolete random leaf angles and lengths.
from measure_foliage import main as measure_foliage  # noqa: E402
measure_foliage()
