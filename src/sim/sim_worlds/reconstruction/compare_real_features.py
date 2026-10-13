#!/usr/bin/env python3
"""Link real evidence and model witnesses without claiming registered similarity."""

import argparse
import json
from pathlib import Path

from audit_real_features import digest, screen_statistics
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
ANCHORS = ('reference', 'detail', 'orchard')
WITNESSES = ('bag_form_0', 'bag_form_1', 'bag_form_2')
LINKAGES = ('scene_manifest.json', 'geometry_validation.json', 'real_feature_validation.json')


def proxy(path):
    """Use the audit's identical BOX thumbnail and RGB pixel-proxy calculation."""
    with Image.open(path) as image:
        image = image.convert('RGB')
        original_size = list(image.size)
        image.thumbnail((160, 90), Image.Resampling.BOX)
        values = screen_statistics(np.asarray(image))
    return {'path': str(path.resolve()), 'sha256': digest(path),
            'original_size': original_size, 'screen': values}


def linked_json(path):
    """Store original JSON content and its hash to expose evidence lineage."""
    return {'path': str(path.resolve()), 'sha256': digest(path),
            'content': json.loads(path.read_text())}


def board_cell(board, draw, path, col, row, label):
    """Place an unaltered-color thumbnail in a labeled qualitative panel."""
    x, y = col * 640, row * 390 + 60
    with Image.open(path) as image:
        image = image.convert('RGB')
        image.thumbnail((640, 360), Image.Resampling.BOX)
        board.paste(image, (x + (640 - image.width) // 2, y))
    draw.text((x + 8, y - 23), label, fill='white')


def main():
    """Create linked numeric proxies and a qualitative real/model feature board."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--field', type=Path, required=True)
    parser.add_argument('--features', type=Path, required=True)
    parser.add_argument('--audit', type=Path, default=HERE / 'evidence/real_feature_audit.json')
    args = parser.parse_args()
    audit = json.loads(args.audit.read_text())
    root = Path(audit['source'])
    required = ([args.out / (name + '.png') for name in ANCHORS]
                + [args.field / (name + '.png') for name in ('orchard', 'aisle')]
                + [args.features / (name + '.png') for name in WITNESSES]
                + [args.features / 'feature_render_manifest.json'])
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError('Render witnesses are not ready: ' + ', '.join(missing))
    renders = {f'anchor/{name}': args.out / (name + '.png') for name in ANCHORS}
    renders.update({f'field/{name}': args.field / (name + '.png')
                    for name in ('orchard', 'aisle')})
    feature_manifest = json.loads((args.features / 'feature_render_manifest.json').read_text())
    for view in feature_manifest['views']:
        path = args.features / (view['view'] + '.png')
        if digest(path) != view['rgb_sha256']:
            raise ValueError('Isolated render differs from its manifest: ' + str(path))
        renders['isolated/' + view['view']] = path
    scene_manifest = args.out / 'scene_manifest.json'
    if scene_manifest.is_file():
        scene = json.loads(scene_manifest.read_text())
        if scene['source_blend_sha256'] != feature_manifest['source_blend_sha256']:
            raise ValueError('Feature render and validation scene use different source blends.')
    expected_paths = {p for feature in audit['visible_features'] for p in feature['paths']}
    panels = [
        ('bag_form_0', 'Peach_bag/RGB/255.png', args.features / 'bag_form_0.png'),
        ('bag_form_1', 'Peach_bag/RGB/746.png', args.features / 'bag_form_1.png'),
        ('bag_form_2', 'Peach_bag/RGB/1304.png', args.features / 'bag_form_2.png'),
        ('bagged_orchard', 'Peach_bag/RGB/255.png', args.field / 'aisle.png'),
    ]
    assert all(path in expected_paths for _, path, _ in panels)
    indices = {record['path']: record['sha256']
               for split in audit['splits'].values() for frame in split['frames']
               for record in frame['modalities'].values()}
    real_sources = []
    for feature, relative, _ in panels:
        path = root / relative
        sha = digest(path)
        if sha != indices[relative]:
            raise ValueError('Real input differs from the full audit: ' + relative)
        real_sources.append({'feature': feature, 'path': relative, 'sha256': sha})
    existing_linkages = [directory / name for directory in (args.out, args.field)
                         for name in LINKAGES if (directory / name).is_file()]
    missing_linkages = [str(directory / name) for directory in (args.out, args.field)
                        for name in LINKAGES if not (directory / name).is_file()]
    report = {
        'schema': 1,
        'comparison_tool': {'path': str(Path(__file__).resolve()),
                            'sha256': digest(Path(__file__))},
        'audit_tool': {'path': str(HERE / 'audit_real_features.py'),
                       'sha256': digest(HERE / 'audit_real_features.py')},
        'real_audit': {'path': str(args.audit.resolve()), 'sha256': digest(args.audit),
                       'tool_sha256': audit['tool_sha256']},
        'class_semantics': audit.get('class_semantics'),
        'method': audit['method'],
        'real_all_frame_summaries': {split: data['summary']
                                     for split, data in audit['splits'].items()},
        'render_pixel_proxies': {name: proxy(path) for name, path in renders.items()},
        'real_panel_sources': real_sources,
        'geometry_linkages': [linked_json(path) for path in existing_linkages],
        'missing_geometry_linkages': missing_linkages,
        'isolated_render_manifest': linked_json(args.features / 'feature_render_manifest.json'),
        'limits': [
            'Qualitative panels are not registered views of the same tree, lens or illumination.',
            'All-frame real distributions include repeated views, occlusion and varied exposure.',
            'Green pixels include grass/background; yellow watermark exclusion is approximate.',
            'Isolated meshes change background and coverage; '
            'do not infer orchard realism from them.',
            'Only opaque bagged-peach views are presented; interior geometry is inferred.',
            'No pass threshold, calibrated dimensions or statistical independence is claimed.',
        ],
    }
    board = Image.new('RGB', (1280, len(panels) * 390 + 60), (22, 22, 22))
    draw = ImageDraw.Draw(board)
    draw.text((8, 8), 'QUALITATIVE REAL / MODEL: different viewpoints; no registration',
              fill='white')
    for row, (feature, relative, model_path) in enumerate(panels):
        board_cell(board, draw, root / relative, 0, row, 'REAL qualitative: ' + relative)
        board_cell(board, draw, model_path, 1, row, 'MODEL diagnostic: ' + feature)
    board_path = args.out / 'real_feature_comparison.jpg'
    board.save(board_path, quality=92)
    report['comparison_board'] = {'path': str(board_path.resolve()), 'sha256': digest(board_path)}
    (args.out / 'real_feature_comparison.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
