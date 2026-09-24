#!/usr/bin/env python3
"""
用本项目感知（YOLO `best.pt` + MobileSAM）闭环验证 3D 渲染结果.

思路：渲染图喂给**真数据集训练出的检测/分割**，再与场景目标清单（GT）逐颗
对表——检测得出、框得对、分割轮廓对，才算"真实还原"（否则合成图只是好看）。
检测口径对齐 `peach_harvester` 感知节点（类别 `peach_bag`/`peach_nobag`）。

用法（须 venv：ultralytics/torch 见 requirements.txt）::

    aubo_py3.12/bin/python scripts/validate_render_perception.py \\
        --images /tmp/peach_sim_run/probe_probe_bag.png \\
        --manifest src/peach_sim/worlds/peach_orchard.manifest.yaml \\
        --camera "0.75,-2.75,1.45,0,0.2,2.35" --fov 1.1

输出：检出数/置信度、GT 投影框与检出框的 IoU/中心偏差、MobileSAM 掩膜面积比。
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np


def project(points: np.ndarray, camera, fov: float, size: tuple[int, int]):
    """世界点 → 像素（相机位姿 xyz+rpy，fov 为水平角，SDF 相机 +X 前）."""
    x0, y0, z0, roll, pitch, yaw = camera
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    forward = np.array([cp * cy, cp * sy, -sp])
    left = np.array([-sy, cy, 0.0])
    up = np.array([sp * cy, sp * sy, cp])
    width, height = size
    focal = (width / 2.0) / math.tan(fov / 2.0)
    origin = np.array([x0, y0, z0])
    depth = (points - origin) @ forward
    x_img = width / 2.0 - ((points - origin) @ left) / np.maximum(depth, 1e-6) * focal
    y_img = height / 2.0 - ((points - origin) @ up) / np.maximum(depth, 1e-6) * focal
    return x_img, y_img, depth


def gt_boxes(manifest: dict, camera, fov: float, size: tuple[int, int],
             depth_max: float = 3.0):
    """
    目标清单 → 像素投影框（袋体 8 角点包围盒，对齐 VOC 方形框口径）.

    早期版本只取袋轴两端的展幅，投影成 35x184 瘦高框（真 GT 是 94x95 方形），
    IoU 天然对不上——这版按袋体三维包络（宽/厚/长）取 8 角点投影。
    """
    boxes = []
    width, height = size
    for entry in manifest['targets']:
        center = np.array(entry['fruit_center'])
        axis = np.array(entry['axis'])
        lateral = np.array([-axis[1], axis[0], 0.0])
        lateral = lateral / max(np.linalg.norm(lateral), 1e-9)
        across = np.cross(lateral, axis)
        half_w = entry['body_diameter'] / 2.0
        half_t = entry['body_thickness'] / 2.0
        half_l = (entry['bottom_to_neck'] + 0.018) / 2.0
        corners = []
        for sw in (-1, 1):
            for st in (-1, 1):
                for sl in (-1, 1):
                    corners.append(center + sw * half_w * lateral
                                   + st * half_t * across + sl * half_l * axis)
        xs, ys, depth = project(np.array(corners), camera, fov, size)
        if depth.mean() < 0.15 or depth.mean() > depth_max:
            continue
        x0, x1 = float(xs.min()), float(xs.max())
        y0, y1 = float(ys.min()), float(ys.max())
        if x1 < 0 or y1 < 0 or x0 > width or y0 > height:
            continue
        boxes.append({'id': entry['id'], 'box': (x0, y0, x1, y1)})
    return boxes


def iou(a, b) -> float:
    ix0, iy0 = max(a[0], b[0]), max(a[1], b[1])
    ix1, iy1 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, ix1 - ix0) * max(0.0, iy1 - iy0)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return inter / max(area_a + area_b - inter, 1e-9)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--images', nargs='+', required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--camera', default='',
                        help='相机位姿 x,y,z,roll,pitch,yaw（与图对应）')
    parser.add_argument('--camera-list', default='',
                        help='多视角评测：每行一个位姿（与 --images 同序）')
    parser.add_argument('--fov', type=float, default=1.1)
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--depth-max', type=float, default=3.0,
                        help='GT 距离窗上限（m）；数据集实拍中位 0.58m')
    parser.add_argument('--segment', action='store_true',
                        help='再跑 MobileSAM 分割比轮廓')
    args = parser.parse_args()

    import yaml
    from ultralytics import YOLO

    manifest = yaml.safe_load(args.manifest.read_text(encoding='utf-8'))
    model = YOLO(str(Path('src/peach_harvester/model/best.pt')))
    camera = tuple(float(v) for v in args.camera.split(',')) if args.camera else None
    camera_list = []
    if args.camera_list:
        for line in Path(args.camera_list).read_text(encoding='utf-8').splitlines():
            if line.strip():
                camera_list.append(tuple(float(v) for v in line.split(',')))

    totals = {'gt': 0, 'hit': 0, 'det': 0}
    for index, image_path in enumerate(args.images):
        if camera_list:
            camera = camera_list[index] if index < len(camera_list) else None
        from PIL import Image
        size = Image.open(image_path).size
        result = model.predict(source=image_path, conf=args.conf,
                               verbose=False)[0]
        detections = []
        for box, conf in zip(result.boxes.xyxy, result.boxes.conf):
            detections.append({'box': tuple(float(v) for v in box),
                               'conf': float(conf)})
        print(f'== {Path(image_path).name}: 检出 {len(detections)} '
              f'conf≥{args.conf}')
        if camera is None or not detections:
            for item in detections[:8]:
                print(f"   conf={item['conf']:.2f} box={tuple(round(v) for v in item['box'])}")
            continue
        gts = gt_boxes(manifest, camera, args.fov, size, args.depth_max)
        print(f'   GT 投影框（视野内）={len(gts)}')
        matched = 0
        ious = []
        for gt in gts:
            best, best_iou = None, 0.0
            for det in detections:
                overlap = iou(gt['box'], det['box'])
                if overlap > best_iou:
                    best, best_iou = det, overlap
            if best_iou >= 0.3:
                matched += 1
                ious.append(best_iou)
        print(f'   命中(IoU≥0.3) = {matched}/{len(gts)} '
              f'检出命中率 = {matched}/{len(detections)} '
              + (f'IoU 中位={np.median(ious):.2f}' if ious else ''))
        totals['gt'] += len(gts)
        totals['hit'] += matched
        totals['det'] += len(detections)
        if args.segment:
            sam = YOLO('src/peach_harvester/model/mobile_sam.pt')
            masks = sam.predict(source=image_path, verbose=False)[0]
            print(f'   MobileSAM 掩膜数={len(masks)}')
    if len(args.images) > 1 and totals['gt']:
        print(f"== 聚合：{len(args.images)} 视角  GT={totals['gt']}  "
              f"检出={totals['det']}  "
              f"召回={totals['hit']/totals['gt']*100:.0f}%")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
