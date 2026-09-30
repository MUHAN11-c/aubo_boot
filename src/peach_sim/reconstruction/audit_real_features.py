#!/usr/bin/env python3
"""Reproducible full-file audit and image-space proxies, never leaf measurements."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw

QUANTILES = [.1, .5, .9]
SPLITS = ('Peach_bag', 'Peach_nobag', 'Peach_young')

CLASS_SEMANTICS = {
    'source_url': 'https://github.com/tsing-luo/Multi-class-peach-RGB-D-dataset',
    'annotation': 'VOC_4label',
    'meaning': 'occlusion status, not maturity or fruit size',
    'labels': {'0': 'non-occluded', '1': 'occluded by leaves',
               '2': 'occluded by branches', '3': 'occluded by fruits'},
    'scope': 'Author README describes bag and naked subsets; local young uses same codes.',
}


def screen_statistics(rgb):
    """RGB green proxy; conservative exclusion for yellow watermark-like pixels."""
    a = np.asarray(rgb, dtype=float)
    luma = a @ np.array([.2126, .7152, .0722])
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    yellow = (r > b * 1.15) & (g > b * 1.15) & (np.abs(r - g) < 30)
    green = (g > r * 1.12) & (g > b * 1.04) & (luma > 25) & (luma < 225) & ~yellow
    return {
        'green_fraction': float(green.mean()),
        'green_rgb_p10_p50_p90': (
            np.quantile(a[green], QUANTILES, axis=0).tolist() if green.any() else None),
        'luminance_p10_p50_p90': np.quantile(luma, QUANTILES).tolist(),
        'yellow_screen_exclusion_fraction': float(yellow.mean()),
    }


def valid_depth_statistics(depth):
    a = np.asarray(depth)
    valid = a > 0
    return {
        'valid_fraction': float(
            valid.mean()), 'valid_raw_p10_p50_p90': np.quantile(
            a[valid], QUANTILES).tolist() if valid.any() else None}


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def image_record(path, root, kind):
    result = {'path': str(path.relative_to(root)), 'sha256': digest(path)}
    with Image.open(path) as image:
        result.update(size=list(image.size), mode=image.mode)
        if kind == 'RGB':
            image = image.convert('RGB')
            image.thumbnail((160, 90), Image.Resampling.BOX)
            result['screen'] = screen_statistics(np.asarray(image))
        elif kind == 'Depth':
            a = np.asarray(image)
            result['dtype'] = str(a.dtype)
            result['depth'] = valid_depth_statistics(a)
    return result


def frame_record(args):
    root, split, ident = args
    result = {'id': ident, 'modalities': {}, 'missing': [], 'shape_mismatches': []}
    for kind in ('RGB', 'Depth', 'Infrared'):
        path = root / split / kind / (ident + '.png')
        if not path.exists():
            result['missing'].append(kind)
            continue
        try:
            rec = image_record(path, root, kind)
            result['modalities'][kind] = rec
            if rec['size'] != [1280, 720]:
                result['shape_mismatches'].append(kind)
        except Exception as exc:
            result['modalities'][kind] = {
                'path': str(
                    path.relative_to(root)),
                'sha256': digest(path),
                'decode_error': str(exc)}
            result.setdefault('errors', []).append(
                {'path': str(path.relative_to(root)), 'error': str(exc)})
    path = root / split / 'Annotations_VOC' / 'VOC_4label' / (ident + '.xml')
    if not path.exists():
        result['missing'].append('VOC_4label')
    else:
        rec = {
            'path': str(
                path.relative_to(root)),
            'sha256': digest(path),
            'class_counts': {},
            'boxes': []}
        try:
            tree = ET.parse(path).getroot()
            size = tree.find('size')
            rec['size'] = [int(size.findtext(k)) for k in ('width', 'height')]
            if rec['size'] != [1280, 720]:
                result['shape_mismatches'].append('VOC_4label')
            for obj in tree.findall('object'):
                name = obj.findtext('name')
                rec['class_counts'][name] = rec['class_counts'].get(name, 0) + 1
                box = obj.find('bndbox')
                rec['boxes'].append([float(box.findtext(k))
                                    for k in ('xmin', 'ymin', 'xmax', 'ymax')])
        except Exception as exc:
            result.setdefault('errors', []).append(
                {'path': str(path.relative_to(root)), 'error': str(exc)})
        result['modalities']['VOC_4label'] = rec
    return result


def summarize(records):
    screens = [r['modalities']['RGB']['screen']
               for r in records if 'screen' in r['modalities'].get('RGB', {})]
    stats = {}
    for key in ('green_fraction', 'yellow_screen_exclusion_fraction'):
        stats[key + '_frame_p10_p50_p90'] = np.quantile([s[key]
                                                        for s in screens], QUANTILES).tolist()
    stats['median_luminance_frame_p10_p50_p90'] = np.quantile(
        [s['luminance_p10_p50_p90'][1] for s in screens], QUANTILES).tolist()
    colors = [s['green_rgb_p10_p50_p90'][1]
              for s in screens if s['green_rgb_p10_p50_p90'] is not None]
    stats['green_frame_median_rgb_p10_p50_p90'] = np.quantile(
        colors, QUANTILES, axis=0).tolist() if colors else None
    stats['dark_proxy_median_luminance_lt_50_frames'] = sum(
        s['luminance_p10_p50_p90'][1] < 50 for s in screens)
    valid = [r['modalities']['Depth']['depth']['valid_fraction']
             for r in records if 'depth' in r['modalities'].get('Depth', {})]
    stats['valid_depth_fraction_frame_p10_p50_p90'] = np.quantile(valid, QUANTILES).tolist()
    stats['zero_valid_depth_frames'] = sum(v == 0 for v in valid)
    ratios = []
    classes = {}
    for r in records:
        for k, count in r['modalities'].get('VOC_4label', {}).get('class_counts', {}).items():
            classes[k] = classes.get(k, 0) + count
        for x0, y0, x1, y1 in r['modalities'].get('VOC_4label', {}).get('boxes', []):
            if x1 > x0:
                ratios.append((y1 - y0) / (x1 - x0))
    stats['voc_four_label_objects'] = classes
    stats['box_h_over_w_p10_p50_p90'] = np.quantile(ratios, QUANTILES).tolist() if ratios else None
    return stats


def select_samples(records):
    rgb = [r for r in records if 'screen' in r['modalities'].get('RGB', {})]

    def luma(r):
        return r['modalities']['RGB']['screen']['luminance_p10_p50_p90'][1]

    white = sorted(rgb, key=lambda r: (abs(luma(r) - 110), int(r['id'])))
    dark = sorted(rgb, key=lambda r: (luma(r), int(r['id'])))
    near = [r for r in rgb if r['modalities'].get('Depth', {}).get('depth', {}).get(
        'valid_raw_p10_p50_p90')
        and r['modalities']['Depth']['depth']['valid_fraction'] > .2
        and luma(r) >= 70]
    near.sort(key=lambda r: (r['modalities']['Depth']['depth']
              ['valid_raw_p10_p50_p90'][1], int(r['id'])))
    return {
        'daylight_proxy': white[0]['id'],
        'low_light_proxy': dark[0]['id'],
        'near_valid_depth_proxy': near[0]['id'] if near else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=Path('/home/mu/Downloads/PeachDataSet'))
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'evidence')
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    root = args.dataset.resolve()
    report = {
        'schema': 1,
        'source': str(root),
        'tool_sha256': digest(Path(__file__)),
        'numpy_version': np.__version__,
        'class_semantics': CLASS_SEMANTICS,
    }
    report.update(
        {'method': {'order': 'split order then numeric frame ID; SHA256 original source files',
                    'rgb': 'all RGB frames; Pillow BOX thumbnail maximum 160x90; Rec709 '
                           'luminance; green G>1.12R and G>1.04B, 25<luminance<225',
                    'watermark_exclusion': 'yellow-like pixels R>1.15B and G>1.15B and '
                                           'abs(R-G)<30 excluded from green screen; '
                                           'conservative chromatic proxy, not exact watermark '
                                           'removal; watermark text may remain',
                    'depth': 'full-frame uint16; only raw values>0; raw quantiles not '
                             'calibrated surface distances',
                    'samples': 'daylight proxy closest median luminance to110; low light '
                               'minimum median luminance; near smallest nonzero median depth '
                               'with valid fraction>.2 and median luminance>=70; numeric ID '
                               'tie-break',
                    'statistics': 'quantiles over repeated image frames/boxes, not independent '
                                  'trees/fruit/leaves'},
         'limitations': ['Green support is a pixel proxy, not leaf-instance masks; '
                         'grass/background and yellow leaves can bias it.',
                         'No calibrated per-device intrinsics or confirmed depth alignment; no '
                         'botanical sizes inferred.',
                         'Repeated observations do not establish independent sample counts or '
                         'orchard population distributions.',
                         'Opaque bags hide fruit and rear paper; neck tie, seam thickness and '
                         'backside remain inferred.',
                         'Close views cannot establish complete tree topology, branch age or '
                         'full crown dimensions.'],
         'visible_features': [{'feature': 'salmon/red-brown paper panels, asymmetric flattened '
                                          'lower edge, broad creases and gathered neck',
                               'paths': ['Peach_bag/RGB/255.png',
                                         'Peach_bag/RGB/746.png',
                                         'Peach_bag/RGB/1304.png'],
                               'status': 'visual observation; no crease amplitude or tie '
                                         'material measured'},
                              {'feature': 'pointed elongated blades, curved/drooping/rolled '
                                          'poses, mixed bright and shaded green, blade overlap',
                               'paths': ['Peach_bag/RGB/255.png',
                                         'Peach_nobag/RGB/161.png',
                                         'Peach_young/RGB/91.png'],
                               'status': 'visual observation; precise leaf sequence and hidden '
                                         'attachment not resolved'},
                              {'feature': 'thicker brown woody axes, finer lateral shoots, '
                                          'irregular junctions, gaps and clustered foliage',
                               'paths': ['Peach_bag/RGB/746.png',
                                         'Peach_nobag/RGB/484.png',
                                         'Peach_young/RGB/91.png'],
                               'status': 'visual observation; branch age inferred from '
                                         'appearance, not labeled'},
                              {'feature': 'mature red/yellow fruit and young pale green fruit, '
                                          'multiple adjacent fruit and partial occlusion',
                               'paths': ['Peach_nobag/RGB/161.png',
                                         'Peach_nobag/RGB/592.png',
                                         'Peach_young/RGB/364.png'],
                               'status': 'visual observation; no independent physical '
                                         'dimensions'}],
         'splits': {}}
    )
    board = Image.new('RGB', (1440, 3 * 294), (20, 20, 20))
    draw = ImageDraw.Draw(board)
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for row, split in enumerate(SPLITS):
            dirs = {k: root / split / k for k in ('RGB', 'Depth', 'Infrared')}
            dirs['VOC_4label'] = root / split / 'Annotations_VOC' / 'VOC_4label'
            indices = {k: {p.stem for p in directory.glob(
                '*.xml' if k == 'VOC_4label' else '*.png')} for k, directory in dirs.items()}
            ids = sorted(set.union(*indices.values()), key=int)
            records = list(pool.map(frame_record, [(root, split, i) for i in ids]))
            selected = select_samples(records)
            report['splits'][split] = {
                'file_counts': {k: len(v) for k, v in indices.items()},
                'missing_ids': {k: sorted(set(ids) - v, key=int) for k, v in indices.items()},
                'decode_or_parse_errors': sum(len(r.get('errors', [])) for r in records),
                'shape_mismatches': sum(len(r['shape_mismatches']) for r in records),
                'summary': summarize(records),
                'selected_samples': selected,
                'frames': records,
            }
            for col, (kind, ident) in enumerate(selected.items()):
                if ident is None:
                    continue
                path = root / split / 'RGB' / (ident + '.png')
                with Image.open(path) as image:
                    image = image.convert('RGB')
                    image.thumbnail((480, 270), Image.Resampling.BOX)
                    board.paste(image, (col * 480, row * 294 + 24))
                draw.text((col * 480 + 5, row * 294 + 5), f'{split}/{ident} {kind}', fill='white')
            print(split, len(ids), 'indexed frames', flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    board_path = args.output / 'real_feature_contact_sheet.jpg'
    board.save(board_path, quality=92)
    report['contact_sheet'] = {'path': board_path.name, 'sha256': digest(board_path)}
    (args.output / 'real_feature_audit.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
