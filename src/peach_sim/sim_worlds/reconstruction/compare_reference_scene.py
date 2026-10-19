"""Compare a same-camera scene render with the real RGB-D anchor and baseline."""

import argparse
import hashlib
import json
from pathlib import Path

from audit_real_features import screen_statistics
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
REAL = Path('/home/mu/Downloads/PeachDataSet/Peach_bag/RGB/1200.png')


def green_mask(rgb):
    """Use a conservative color proxy, not a leaf instance segmentation."""
    a = np.asarray(rgb, dtype=float)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    luma = a @ np.array([.2126, .7152, .0722])
    yellow = (r > b * 1.15) & (g > b * 1.15) & (np.abs(r - g) < 30)
    return (g > r * 1.12) & (g > b * 1.04) & (luma > 25) & (luma < 225) & ~yellow


def digest(path):
    """Hash a source or result."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """Publish image-space errors and unaltered real/model review panels."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--before', type=Path, required=True)
    args = parser.parse_args()
    paths = {'real': REAL, 'before': args.before / 'reference.png',
             'after': args.out / 'reference.png'}
    images = {k: Image.open(p).convert('RGB') for k, p in paths.items()}
    if any(im.size != (1280, 720) for im in images.values()):
        raise ValueError('Comparison requires native-size matched camera views')
    target = green_mask(images['real'])
    metrics = {}
    for name, im in images.items():
        mask = green_mask(im)
        metrics[name] = screen_statistics(im)
        metrics[name]['green_spatial_iou'] = float(
            (mask & target).sum() / max(1, (mask | target).sum()))
    for name, root in (('before', args.before), ('after', args.out)):
        perception = json.loads((root / 'perception_validation.json').read_text())
        comparisons = perception['views']['reference']['reference_comparison']
        metrics[name]['bag_mask_mean_iou'] = float(np.mean(
            [item['source_mask_iou'] for item in comparisons]))
        metrics[name]['bag_depth_mean_mae_m'] = float(np.mean(
            [item['depth_mae_m'] for item in comparisons if 'depth_mae_m' in item]))
        metrics[name]['bag_count'] = len(comparisons)
    board = Image.new('RGB', (1920, 1120), (24, 24, 24))
    draw = ImageDraw.Draw(board)
    for j, (name, im) in enumerate(images.items()):
        board.paste(im.resize((640, 360)), (j * 640, 30))
        draw.text((j * 640 + 10, 10), name + ' / camera 1200', fill='white')
        for i, (box, label) in enumerate((((200, 170, 520, 490), 'bag and leaves'),
                                        ((880, 90, 1200, 410), 'foliage and branches'))):
            board.paste(im.crop(box).resize((640, 320)), (j * 640, 420 + i * 350))
            draw.text((j * 640 + 10, 400 + i * 350), name + ' / ' + label, fill='white')
    path = args.out / 'reference_scene_comparison.jpg'
    board.save(path, quality=94)
    report = {'revision': json.loads((args.out / 'scene_manifest.json').read_text())[
        'modeling_revision'], 'metrics': metrics,
        'source_sha256': {k: digest(p) for k, p in paths.items()},
        'perception_sha256': digest(args.out / 'perception_validation.json'),
        'board_sha256': digest(path),
        'limits': ['Single RGB-D anchor; does not prove whole-orchard reconstruction.',
                   'Approximate intrinsics, no calibrated camera pose or illumination.',
                   'Green proxy is not leaf area; source bag masks are SAM estimates.',
                   'RGB crops preserve source colors; watermark is not a model texture.']}
    (args.out / 'reference_scene_comparison.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
