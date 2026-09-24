"""从 PeachDataSet 与现场抓取记录统计套袋几何，供 Blender 场景使用。

Azure Kinect 彩色 RES_720P 视场 90°×59°（中国科学数据 2022, 7(4)），
深度已对齐到 1280×720，单位毫米，拍摄距离约 0.5 m。
"""

from __future__ import annotations

import json
import os
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np
from PIL import Image

DATASET = '/home/mu/Downloads/PeachDataSet'
FIELD = (
    '/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach_arm/test/fixtures/'
    'field_pregrasp_cases.yaml')
OUT = os.path.join(os.path.dirname(__file__), '..', 'data', 'priors.json')

W, H = 1280, 720
HFOV, VFOV = np.deg2rad(90.0), np.deg2rad(59.0)
FX = (W / 2.0) / np.tan(HFOV / 2.0)
FY = (H / 2.0) / np.tan(VFOV / 2.0)
CX, CY = W / 2.0, H / 2.0


def _percentiles(values):
    if not values:
        return None
    arr = np.asarray(values, dtype=np.float64)
    return {
        'n': int(arr.size),
        'p10': float(np.percentile(arr, 10)),
        'p50': float(np.percentile(arr, 50)),
        'p90': float(np.percentile(arr, 90)),
        'mean': float(arr.mean()),
    }


def _boxes(xml_path):
    root = ET.parse(xml_path).getroot()
    boxes = []
    for obj in root.findall('object'):
        box = obj.find('bndbox')
        boxes.append((
            obj.findtext('name'),
            int(float(box.findtext('xmin'))),
            int(float(box.findtext('ymin'))),
            int(float(box.findtext('xmax'))),
            int(float(box.findtext('ymax'))),
        ))
    return boxes


def _unproject(u, v, z_m):
    return np.array([
        (u - CX) / FX * z_m,
        (v - CY) / FY * z_m,
        z_m,
    ])


def measure_split(split, limit=250):
    rgb_dir = os.path.join(DATASET, split, 'RGB')
    depth_dir = os.path.join(DATASET, split, 'Depth')
    ann_dir = os.path.join(DATASET, split, 'Annotations_VOC', 'VOC_4label')
    names = sorted(f for f in os.listdir(ann_dir) if f.endswith('.xml'))
    step = max(1, len(names) // limit)
    names = names[::step][:limit]
    by_class = defaultdict(lambda: defaultdict(list))
    gaps = []
    for name in names:
        stem = os.path.splitext(name)[0]
        rgb_path = os.path.join(rgb_dir, stem + '.png')
        depth_path = os.path.join(depth_dir, stem + '.png')
        if not (os.path.exists(rgb_path) and os.path.exists(depth_path)):
            continue
        rgb = np.asarray(Image.open(rgb_path).convert('RGB'))
        depth = np.asarray(Image.open(depth_path))
        points = []
        for cls, x1, y1, x2, y2 in _boxes(os.path.join(ann_dir, name)):
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
            width = (x2 - x1) / FX * z
            height = (y2 - y1) / FY * z
            color = rgb[y1:y2:4, x1:x2:4].reshape(-1, 3).astype(np.float64)
            # 丢掉近白水印：只留饱和度较高或偏暗的像素
            chroma = color.max(1) - color.min(1)
            keep = color[chroma > 18]
            if keep.shape[0] < 10:
                keep = color
            med = np.median(keep, axis=0)
            bucket = by_class[cls]
            bucket['width_m'].append(width)
            bucket['height_m'].append(height)
            bucket['aspect'].append(height / max(width, 1e-6))
            bucket['depth_m'].append(z)
            bucket['rgb'].append(med.tolist())
            points.append((cls, _unproject(cx, cy, z)))
        bag_pts = [p for cls, p in points if cls == '0']
        if len(bag_pts) >= 2:
            for i, a in enumerate(bag_pts):
                dists = [np.linalg.norm(a - b) for j, b in enumerate(bag_pts) if i != j]
                near = min(dists)
                if near < 0.8:
                    gaps.append(near)
    summary = {}
    for cls, bucket in by_class.items():
        rgb = np.asarray(bucket['rgb'])
        summary[cls] = {
            'width_m': _percentiles(bucket['width_m']),
            'height_m': _percentiles(bucket['height_m']),
            'aspect_h_over_w': _percentiles(bucket['aspect']),
            'depth_m': _percentiles(bucket['depth_m']),
            'rgb_median': [float(x) for x in np.median(rgb, axis=0)],
        }
    return {'classes': summary, 'bag_nearest_gap_m': _percentiles(gaps)}


def measure_field():
    import yaml
    with open(FIELD, encoding='utf-8') as handle:
        doc = yaml.safe_load(handle)
    lengths, tilts = [], []
    for item in doc['targets_20260909'].values():
        bottom = np.asarray(item['bag_bottom'], dtype=np.float64)
        neck = np.asarray(item['bag_neck'], dtype=np.float64)
        axis = neck - bottom
        length = float(np.linalg.norm(axis))
        if length < 1e-4:
            continue
        lengths.append(length)
        tilt = float(np.degrees(np.arccos(np.clip(axis[2] / length, -1.0, 1.0))))
        tilts.append(tilt)
    heights = [item['bag_bottom'][2] for item in doc['targets_20260909'].values()]
    return {
        'bottom_to_neck_m': _percentiles(lengths),
        'axis_tilt_from_up_deg': _percentiles(tilts),
        'bag_bottom_z_base_m': _percentiles(heights),
        'note': 'base_link 系；臂座离地约 0.75 m，世界高度 ≈ z + 0.75',
    }


def main():
    priors = {
        'camera': {
            'sensor': 'Azure Kinect DK color RES_720P, depth aligned',
            'fx': float(FX), 'fy': float(FY), 'cx': CX, 'cy': CY,
            'source': '中国科学数据 2022, 7(4)；视场 90×59 度',
        },
        'orchard_standard': {
            'form': '三主枝自然开心形',
            'trunk_height_m': [0.40, 0.50],
            'scaffold_elevation_deg': [40, 70],
            'tree_height_max_m': 2.5,
            'spacing_m': {'in_row': [3.0, 4.0], 'between_rows': [5.0, 6.0]},
            'source': '湖南省林业局桃树树形；DB41/T 1317-2016',
        },
        'splits': {
            'Peach_bag': measure_split('Peach_bag'),
            'Peach_nobag': measure_split('Peach_nobag', limit=120),
        },
        'field_20260909': measure_field(),
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', encoding='utf-8') as handle:
        json.dump(priors, handle, ensure_ascii=False, indent=2)
    print(json.dumps(priors, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
