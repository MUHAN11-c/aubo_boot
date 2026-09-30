"""Estimate local foliage axes from green RGB support and valid depth."""

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
DATA = Path('/home/mu/Downloads/PeachDataSet/Peach_bag')


def estimate_support(rgb, depth):
    """Fit local connected green regions; these are not leaf instance labels."""
    a = np.asarray(rgb, dtype=float)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    yellow = (r > b * 1.15) & (g > b * 1.15) & (np.abs(r - g) < 30)
    green = ((g > r * 1.10) & (g > b * 1.06) & (g > 28) & (g < 220)
             & ~yellow & (depth > 220) & (depth < 850))
    candidates = []
    height, width = depth.shape
    for v in range(24, height - 24, 32):
        for u in range(24, width - 24, 32):
            y0, y1 = max(0, v - 75), min(height, v + 76)
            x0, x1 = max(0, u - 75), min(width, u + 76)
            patch = green[y0:y1, x0:x1].astype('uint8')
            count, labels, stats, centers = cv2.connectedComponentsWithStats(patch, connectivity=8)
            choices = [k for k in range(1, count) if stats[k, cv2.CC_STAT_AREA] >= 100
                       and np.linalg.norm(centers[k] - [u - x0, v - y0]) < 24]
            if not choices:
                continue
            k = min(choices, key=lambda j: np.linalg.norm(centers[j] - [u - x0, v - y0]))
            yy, xx = np.where(labels == k)
            coords = np.column_stack((xx + x0, yy + y0)).astype('float64')
            center, axes, values = cv2.PCACompute2(coords, mean=None)
            ratio = float(values[0, 0] / max(values[1, 0], 1e-6))
            if ratio < 1.8:
                continue
            projected = (coords - center) @ axes.T
            spans = np.quantile(projected, .98, axis=0) - np.quantile(projected, .02, axis=0)
            z = float(np.median(depth[yy + y0, xx + x0])) * .001
            if spans[0] < 22:
                continue
            candidates.append({
                'u': float(center[0, 0]), 'v': float(center[0, 1]), 'depth_m': z,
                'length_m': float(np.clip(spans[0] * z / 640, .025, .16)),
                'width_m': float(np.clip(spans[1] * z / 640, .006, .055)),
                'axis_image': axes[0].tolist(), 'axis_ratio': ratio,
                'support_pixels': len(coords), 'source': '1200 local green-region PCA + depth',
                'inferred': ['leaf_instance_identity', 'surface_normal', 'hidden_attachment'],
                'angle_rad': float(np.arctan2(axes[0, 0], axes[0, 1]))})
    kept = []
    for item in sorted(candidates, key=lambda q: -q['support_pixels'] * q['axis_ratio']):
        if any(np.hypot(item['u'] - k['u'], item['v'] - k['v']) < 22 for k in kept):
            continue
        kept.append(item)
    return sorted(kept, key=lambda q: (q['v'], q['u']))


def main():
    """Write repeatable support records and an overlay for visual inspection."""
    rgb_path, depth_path = DATA / 'RGB/1200.png', DATA / 'Depth/1200.png'
    rgb = Image.open(rgb_path).convert('RGB')
    depth = np.asarray(Image.open(depth_path))
    support = estimate_support(np.asarray(rgb), depth)
    (HERE / 'evidence/foliage_support.json').write_text(json.dumps(support, indent=2))
    draw = ImageDraw.Draw(rgb)
    for item in support:
        direction = np.array(item['axis_image'])
        half = item['length_m'] * 640 / item['depth_m'] / 2
        center = np.array([item['u'], item['v']])
        draw.line([tuple(center - direction * half), tuple(center + direction * half)],
                  fill=(255, 80, 30), width=2)
    rgb.save(HERE / 'evidence/foliage_axis_review.jpg')
    record = {'count': len(support), 'source_sha256': {
        str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (rgb_path, depth_path)},
        'tool_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'limits': ['Axes describe local green regions, possibly overlapping leaves.',
                   'Missing depth and yellow watermark-like pixels excluded.',
                   'No source RGB is used as a model texture.']}
    (HERE / 'evidence/foliage_measurement.json').write_text(json.dumps(record, indent=2))
    print('Foliage support:', len(support))


if __name__ == '__main__':
    main()
