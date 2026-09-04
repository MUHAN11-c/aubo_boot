#!/usr/bin/env python3
"""从 PeachDataSet 套袋 RGB-D 反投影单视角点云模型。

该公开集是检测数据：有对齐的 RGB/深度/框，没有相机轨迹。因此每张图
各自一个相机系模型，不能积成 base_link 下的整树 TSDF。

深度按毫米；彩色内参按 Azure Kinect DK 1280×720 标称视场
（90°×59°）估算，集内无 calib。
"""
from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image

# Azure Kinect 彩色 90°×59° @ 1280×720（无逐帧 calib 时的标称 K）
_W, _H = 1280, 720
_FX = _W / (2.0 * np.tan(np.deg2rad(90.0) / 2.0))
_FY = _H / (2.0 * np.tan(np.deg2rad(59.0) / 2.0))
_CX, _CY = _W / 2.0, _H / 2.0

_LABEL4 = {
    '0': 'clear',
    '1': 'leaf_occluded',
    '2': 'branch_occluded',
    '3': 'fruit_occluded',
}


def _load_rgb(path: Path) -> np.ndarray:
    img = np.asarray(Image.open(path))
    if img.ndim == 3 and img.shape[2] == 4:
        img = img[:, :, :3]
    return np.ascontiguousarray(img)


def _load_depth_m(path: Path) -> np.ndarray:
    raw = np.asarray(Image.open(path))
    depth_mm = raw.astype(np.float32)
    depth_m = depth_mm / 1000.0
    depth_m[depth_mm <= 0] = 0.0
    return depth_m


def _parse_boxes(xml_path: Path) -> list[dict]:
    root = ET.parse(xml_path).getroot()
    boxes = []
    for obj in root.findall('object'):
        box = obj.find('bndbox')
        boxes.append({
            'name': (obj.findtext('name') or '0').strip(),
            'xmin': int(float(box.findtext('xmin'))),
            'ymin': int(float(box.findtext('ymin'))),
            'xmax': int(float(box.findtext('xmax'))),
            'ymax': int(float(box.findtext('ymax'))),
        })
    return boxes


def _cloud_from_rgbd(rgb: np.ndarray, depth_m: np.ndarray,
                     mask: np.ndarray | None,
                     voxel: float) -> o3d.geometry.PointCloud:
    if mask is not None:
        depth_m = depth_m.copy()
        depth_m[~mask] = 0.0
    color = o3d.geometry.Image(np.ascontiguousarray(rgb))
    depth = o3d.geometry.Image(np.ascontiguousarray(depth_m.astype(np.float32)))
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_scale=1.0, depth_trunc=3.5,
        convert_rgb_to_intensity=False)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        _W, _H, _FX, _FY, _CX, _CY)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    if pcd.is_empty():
        return pcd
    if voxel > 0:
        pcd = pcd.voxel_down_sample(voxel)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return pcd


def _score_frame(depth_path: Path, xml_path: Path) -> tuple[float, int, float]:
    depth_m = _load_depth_m(depth_path)
    valid = float((depth_m > 0).mean())
    n_box = len(_parse_boxes(xml_path)) if xml_path.is_file() else 0
    return valid * (1.0 + 0.15 * n_box), n_box, valid


def main() -> int:
    parser = argparse.ArgumentParser(
        description='PeachDataSet 套袋 RGB-D → 单视角 ply 模型')
    parser.add_argument(
        '--dataset', type=Path,
        default=Path('/home/mu/Downloads/PeachDataSet (1)/Peach_bag'))
    parser.add_argument(
        '--out', type=Path,
        default=Path('runs/scene_models/peach_dataset_bag'))
    parser.add_argument('--top', type=int, default=12)
    parser.add_argument('--min-valid', type=float, default=0.18)
    parser.add_argument('--min-boxes', type=int, default=2)
    parser.add_argument('--scene-voxel', type=float, default=0.012)
    parser.add_argument('--bag-voxel', type=float, default=0.004)
    args = parser.parse_args()

    rgb_dir = args.dataset / 'RGB'
    depth_dir = args.dataset / 'Depth'
    xml_dir = args.dataset / 'Annotations_VOC' / 'VOC_4label'
    if not depth_dir.is_dir() or not rgb_dir.is_dir():
        print(f'找不到 RGB/Depth: {args.dataset}', file=__import__('sys').stderr)
        return 2

    ranked = []
    for depth_path in sorted(depth_dir.glob('*.png'),
                             key=lambda p: int(p.stem) if p.stem.isdigit() else p.stem):
        xml_path = xml_dir / f'{depth_path.stem}.xml'
        score, n_box, valid = _score_frame(depth_path, xml_path)
        if valid < args.min_valid or n_box < args.min_boxes:
            continue
        ranked.append((score, n_box, valid, depth_path.stem))
    ranked.sort(reverse=True)
    chosen = ranked[: max(0, args.top)]
    if not chosen:
        print('没有同时满足深度有效比例和框数量的帧')
        return 1

    out = args.out
    scene_dir = out / 'scenes'
    bag_dir = out / 'bags'
    scene_dir.mkdir(parents=True, exist_ok=True)
    bag_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        'source': str(args.dataset),
        'frame': 'camera_optical (Open3D RGBD, x right y down z forward)',
        'intrinsics_assumed': {
            'width': _W, 'height': _H,
            'fx': float(_FX), 'fy': float(_FY),
            'cx': _CX, 'cy': _CY,
            'note': 'Azure Kinect DK color 90x59 deg at 1280x720; dataset has no calib',
        },
        'depth_unit': 'png_int_millimetres',
        'frames': [],
    }
    gallery = []
    bag_index = 0
    for score, n_box, valid, stem in chosen:
        rgb = _load_rgb(rgb_dir / f'{stem}.png')
        depth_m = _load_depth_m(depth_dir / f'{stem}.png')
        if rgb.shape[:2] != depth_m.shape:
            print(f'跳过 {stem}: RGB/深度尺寸不一致')
            continue
        scene = _cloud_from_rgbd(rgb, depth_m, None, args.scene_voxel)
        scene_path = scene_dir / f'{stem}_scene.ply'
        if not scene.is_empty():
            o3d.io.write_point_cloud(str(scene_path), scene, write_ascii=False)
        boxes = _parse_boxes(xml_dir / f'{stem}.xml')
        bag_files = []
        h, w = depth_m.shape
        for i, box in enumerate(boxes):
            pad = 6
            x0 = max(0, box['xmin'] - pad)
            y0 = max(0, box['ymin'] - pad)
            x1 = min(w, box['xmax'] + pad)
            y1 = min(h, box['ymax'] + pad)
            mask = np.zeros((h, w), dtype=bool)
            mask[y0:y1, x0:x1] = True
            bag = _cloud_from_rgbd(rgb, depth_m, mask, args.bag_voxel)
            if len(bag.points) < 80:
                continue
            occ = _LABEL4.get(box['name'], box['name'])
            bag_name = f'{stem}_bag{i:02d}_{occ}.ply'
            o3d.io.write_point_cloud(str(bag_dir / bag_name), bag, write_ascii=False)
            bag_files.append(bag_name)
            shifted = o3d.geometry.PointCloud(bag)
            shifted.translate((bag_index * 0.35, 0.0, 0.0))
            gallery.append(shifted)
            bag_index += 1
        entry = {
            'id': stem,
            'score': round(score, 4),
            'valid_depth_ratio': round(valid, 4),
            'n_boxes': n_box,
            'scene_points': int(len(scene.points)),
            'scene': str(scene_path.relative_to(out)) if scene_path.exists() else None,
            'bags': bag_files,
        }
        manifest['frames'].append(entry)
        print(
            f'{stem}: valid={valid:.2%} boxes={n_box} '
            f'scene={len(scene.points)} bags={len(bag_files)}')

    if gallery:
        merged = gallery[0]
        for extra in gallery[1:]:
            merged += extra
        o3d.io.write_point_cloud(str(out / 'bags_gallery.ply'), merged, write_ascii=False)
    (out / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(f'写出 {out}  场景 {len(manifest["frames"])}  袋实例 {bag_index}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
