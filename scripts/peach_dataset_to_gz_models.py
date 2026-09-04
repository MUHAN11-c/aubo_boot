#!/usr/bin/env python3
"""PeachDataSet 套袋分割 → Gazebo 纸袋模型。

只使用三分集里的 Peach_bag（不要 Peach_nobag / Peach_young）。
VOC_1label 的目标全是袋；用与现场相同的 MobileSAM 按检测框抠袋，
贴图只保留袋像素。视觉网格是 SAM 轮廓 + 深度褶皱再挤出厚度（不是方盒）；
碰撞仍用 15×8×18 cm 规格盒。Gazebo Harmonic 用 SDF PBR albedo，不用 OBJ/MTL。
"""
from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np
from PIL import Image

# Azure Kinect DK 彩色 720P 标称视场 90°×59°
# https://learn.microsoft.com/azure/kinect-dk/hardware-specification
# https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/917
_W, _H = 1280, 720
_FX = _W / (2.0 * np.tan(np.deg2rad(90.0) / 2.0))
_FY = _H / (2.0 * np.tan(np.deg2rad(59.0) / 2.0))
_CX, _CY = _W / 2.0, _H / 2.0

# 工业桃袋 / 果实 / 叶片（网页规格，不是 TSDF AABB）
# 袋平面 15×18 cm 中小果，16×19 大果：农业路、AGROW-TEK、惠农网、鑫丰特种纸
# 充果厚度 ≈ 横径 7–9 cm（中国植物志 5–7(12) cm；福达横径 8.63 cm）
# 单果质量 250–300 g → 15×18 袋（AGROW：250–300 g 用 15×18，350–400 g 用 16×19）
# 叶 7–15 × 2–3.5 cm（Flora of China / 中国植物志）
_BAG_W = 0.15
_BAG_H = 0.18
_BAG_T = 0.08
_BAG_MASS = 0.28
_LEAF_L = 0.11
_LEAF_W = 0.028
_SCALE_MIN, _SCALE_MAX = 0.35, 2.6
_SPEC_SOURCES = {
    'bag_flat_cm': '15x18 (alt 16x19 / 17x20)',
    'bag_refs': [
        'https://wenda.nongyelu.com/a/26494.html',
        'https://www.agrow-tek.com/product/single-layer-white-paper-peach-growing-bags/',
        'https://cn.xinfengspecialpaper.com/product/fruiting-protection-bag/peach-protection-bag/',
    ],
    'fruit_diameter_m': 0.075,
    'leaf_cm': '7-15 x 2-3.5',
    'leaf_ref': 'https://naturelib.net/plantae/prunus-persica/',
}

_LABEL = {
    '0': 'peach_bag',
}
_PAD_PX = 4
_N_BAGS = 12
_SAM_WEIGHTS = (
    Path(__file__).resolve().parents[1]
    / 'src/peach_perception/model/mobile_sam.pt'
)
_PAPER_RGB = np.array([184, 107, 41], dtype=np.uint8)


def _sam_bag_mask(rgb: np.ndarray, box: dict, sam) -> np.ndarray | None:
    """MobileSAM 按 VOC 框抠袋；与 scene_perception 一样走 BGR + box prompt。"""
    import cv2
    x0, y0, x1, y1 = box['xmin'], box['ymin'], box['xmax'], box['ymax']
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    try:
        import torch
        device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        results = sam(
            bgr, bboxes=[(x0, y0, x1, y1)], device=device, verbose=False)
    except Exception:
        return None
    if not results or results[0].masks is None:
        return None
    mask = results[0].masks.data[0].cpu().numpy() > 0.5
    ih, iw = rgb.shape[:2]
    if mask.shape != (ih, iw):
        mask = cv2.resize(
            mask.astype(np.uint8), (iw, ih),
            interpolation=cv2.INTER_NEAREST).astype(bool)
    return mask


def _texture_from_mask(rgb: np.ndarray, box: dict,
                      mask: np.ndarray) -> np.ndarray:
    pad = _PAD_PX
    x0 = max(0, box['xmin'] - pad)
    y0 = max(0, box['ymin'] - pad)
    x1 = min(_W, box['xmax'] + pad)
    y1 = min(_H, box['ymax'] + pad)
    crop = rgb[y0:y1, x0:x1]
    m = mask[y0:y1, x0:x1]
    tex = np.empty_like(crop)
    tex[:] = _PAPER_RGB
    tex[m] = crop[m]
    return tex


def _load_rgb(path: Path) -> np.ndarray:
    img = np.asarray(Image.open(path).convert('RGB'))
    return np.ascontiguousarray(img)


def _load_depth_m(path: Path) -> np.ndarray | None:
    try:
        raw = np.asarray(Image.open(path)).astype(np.float32)
    except (OSError, ValueError):
        return None
    if raw.ndim != 2 or raw.shape != (_H, _W):
        return None
    depth_m = raw / 1000.0
    depth_m[raw <= 0] = 0.0
    return depth_m


def _parse_boxes(xml_path: Path) -> list[dict]:
    root = ET.parse(xml_path).getroot()
    boxes = []
    for obj in root.findall('object'):
        box = obj.find('bndbox')
        boxes.append({
            'cls': (obj.findtext('name') or '0').strip(),
            'xmin': int(float(box.findtext('xmin'))),
            'ymin': int(float(box.findtext('ymin'))),
            'xmax': int(float(box.findtext('xmax'))),
            'ymax': int(float(box.findtext('ymax'))),
        })
    return boxes


def _pinhole_box_m(depth_m: np.ndarray, box: dict) -> tuple[float, float, float]:
    """VOC 框在中位深度下的针孔宽高（米）。用未膨胀的袋框，不含额外叶片 pad。"""
    sl = np.s_[box['ymin']:box['ymax'], box['xmin']:box['xmax']]
    patch = depth_m[sl]
    valid = patch[patch > 0.05]
    if valid.size < 20:
        return 0.0, 0.0, 0.0
    z = float(np.median(valid))
    width_m = (box['xmax'] - box['xmin']) * z / _FX
    height_m = (box['ymax'] - box['ymin']) * z / _FY
    return width_m, height_m, z


def _bag_scale(width_m: float, height_m: float) -> float:
    """把针孔测得的袋高对准 18 cm 工业规格；高不可用时用宽对 15 cm。"""
    if height_m >= 0.06:
        raw = _BAG_H / height_m
    elif width_m >= 0.05:
        raw = _BAG_W / width_m
    else:
        return 1.0
    return float(np.clip(raw, _SCALE_MIN, _SCALE_MAX))


def _apply_scale(verts: np.ndarray, scale: float) -> np.ndarray:
    out = verts * scale
    out[:, 2] -= out[:, 2].min()
    return out


def _optical_to_zup(verts: np.ndarray) -> np.ndarray:
    """光学系 (x右 y下 z前) → Gazebo (x右 y前 z上)，照片立着面对 +Y。"""
    x, y, z = verts[:, 0], verts[:, 1], verts[:, 2]
    return np.column_stack((x, z, -y))


def _irregular_bag_mesh(
        depth_m: np.ndarray, mask: np.ndarray, tex: np.ndarray,
        bounds: tuple[int, int, int, int], z_ref: float,
        stride: int = 2, z_jump: float = 0.022,
        thickness_m: float = 0.07) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """SAM 轮廓 + 深度褶皱的不规则袋壳，再沿视线挤出厚度（不是方盒子）。"""
    x0, y0, x1, y1 = bounds
    th, tw = tex.shape[:2]
    ys = np.arange(y0, y1, stride)
    xs = np.arange(x0, x1, stride)
    hh, ww = len(ys), len(xs)
    grid_z = depth_m[np.ix_(ys, xs)]
    grid_m = mask[np.ix_(ys, xs)]
    index = -np.ones((hh, ww), dtype=np.int32)
    cam = []
    uvs = []
    vid = 0
    z_band = 0.08
    for i, v in enumerate(ys):
        for j, u in enumerate(xs):
            if not grid_m[i, j]:
                continue
            z = float(grid_z[i, j])
            if z < 0.05:
                continue
            if z_ref > 0 and abs(z - z_ref) > z_band:
                continue
            xc = (u + 0.5 - _CX) * z / _FX
            yc = (v + 0.5 - _CY) * z / _FY
            cam.append((xc, yc, z))
            uvs.append(((u + 0.5 - x0) / max(tw, 1),
                        1.0 - (v + 0.5 - y0) / max(th, 1)))
            index[i, j] = vid
            vid += 1
    if vid < 40:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int32), np.zeros((0, 2))
    cam_a = np.asarray(cam, dtype=np.float64)
    uvs_a = np.asarray(uvs, dtype=np.float64)
    front_faces = []

    def _close(a: int, b: int) -> bool:
        return abs(cam_a[a, 2] - cam_a[b, 2]) < z_jump

    for i in range(hh - 1):
        for j in range(ww - 1):
            a = int(index[i, j])
            b = int(index[i, j + 1])
            c = int(index[i + 1, j])
            d = int(index[i + 1, j + 1])
            if a >= 0 and b >= 0 and c >= 0 and _close(a, b) and _close(a, c):
                front_faces.append((a, c, b))
            if b >= 0 and c >= 0 and d >= 0 and _close(b, d) and _close(c, d):
                front_faces.append((b, c, d))
    if len(front_faces) < 40:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int32), np.zeros((0, 2))
    faces_f = np.asarray(front_faces, dtype=np.int32)
    n = len(cam_a)
    back_cam = cam_a.copy()
    back_cam[:, 2] += thickness_m
    verts = np.vstack((_optical_to_zup(cam_a), _optical_to_zup(back_cam)))
    center = verts[:n].mean(axis=0)
    verts = verts - center
    verts[:, 2] -= verts[:, 2].min()
    faces_b = np.column_stack((faces_f[:, 0] + n, faces_f[:, 2] + n, faces_f[:, 1] + n))
    edge_count: dict[tuple[int, int], int] = {}
    for a, b, c in faces_f:
        for e in ((a, b), (b, c), (c, a)):
            key = (e[0], e[1]) if e[0] < e[1] else (e[1], e[0])
            edge_count[key] = edge_count.get(key, 0) + 1
    side = []
    for a, b in edge_count:
        if edge_count[(a, b)] != 1:
            continue
        side.append((a, b, b + n))
        side.append((a, b + n, a + n))
    faces = np.vstack((faces_f, faces_b, np.asarray(side, dtype=np.int32)))
    uvs_all = np.vstack((uvs_a, uvs_a))
    return verts, faces, uvs_all


def _pbr_block(png_abs: Path, fallback: str) -> str:
    """Gazebo Harmonic 不读 OBJ/MTL；颜色必须走 SDF PBR albedo_map 的绝对 file://。"""
    uri = png_abs.resolve().as_posix()
    return (
        f'<material>'
        f'<ambient>{fallback} 1</ambient>'
        f'<diffuse>{fallback} 1</diffuse>'
        f'<pbr><metal>'
        f'<albedo_map>file://{uri}</albedo_map>'
        f'<roughness>0.92</roughness>'
        f'<metalness>0.0</metalness>'
        f'</metal></pbr></material>'
    )


def _write_textured_obj(path: Path, verts: np.ndarray, faces: np.ndarray,
                       uvs: np.ndarray) -> None:
    lines = []
    for x, y, z in verts:
        lines.append(f'v {x:.6f} {y:.6f} {z:.6f}')
    for u, v in uvs:
        lines.append(f'vt {u:.6f} {v:.6f}')
    for a, b, c in faces + 1:
        lines.append(f'f {a}/{a} {b}/{b} {c}/{c}')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _bag_link_xml(png_abs: Path, obj_abs: Path) -> str:
    """不规则袋网格 + PBR 贴图；碰撞仍用 15×8×18 cm 规格盒。"""
    paper = '0.72 0.42 0.16'
    obj_uri = obj_abs.resolve().as_posix()
    return (
        '<link name="link">'
        '<visual name="bag"><geometry><mesh>'
        f'<uri>file://{obj_uri}</uri></mesh></geometry>'
        f'{_pbr_block(png_abs, paper)}</visual>'
        '<collision name="collision"><geometry>'
        f'<box><size>{_BAG_W} {_BAG_T} {_BAG_H}</size></box>'
        '</geometry></collision></link>'
    )


def _write_bag_model(model_dir: Path, name: str, tex: np.ndarray,
                     verts: np.ndarray, faces: np.ndarray,
                     uvs: np.ndarray) -> dict:
    mesh_dir = model_dir / 'meshes'
    mesh_dir.mkdir(parents=True, exist_ok=True)
    png = mesh_dir / 'visual.png'
    obj = mesh_dir / 'visual.obj'
    Image.fromarray(tex).save(png)
    _write_textured_obj(obj, verts, faces, uvs)
    sdf = (
        '<?xml version="1.0" ?>\n<sdf version="1.8">\n'
        f'  <model name="{name}">\n    <static>true</static>\n'
        f'    {_bag_link_xml(png, obj)}\n'
        '  </model>\n</sdf>\n'
    )
    (model_dir / 'model.sdf').write_text(sdf, encoding='utf-8')
    (model_dir / 'model.config').write_text(
        f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>Irregular SAM+depth bagged peach; collision 15x8x18 cm.</description>
</model>
""",
        encoding='utf-8')
    extent = verts.max(axis=0) - verts.min(axis=0)
    return {
        'name': name,
        'kind': 'bag',
        'png': str(png.resolve()),
        'obj': str(obj.resolve()),
        'dir': str(model_dir.resolve()),
        'aabb_m': [round(float(v), 4) for v in extent],
        'collision_m': [_BAG_W, _BAG_T, _BAG_H],
        'vertices': int(len(verts)),
        'faces': int(len(faces)),
    }


def _write_scene_card(model_dir: Path, name: str, tex: np.ndarray) -> dict:
    mesh_dir = model_dir / 'meshes'
    mesh_dir.mkdir(parents=True, exist_ok=True)
    png = mesh_dir / 'visual.png'
    Image.fromarray(tex).save(png)
    w, h = 0.80, 0.45
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="photo">
        <pose>0 0 {h/2:.3f} 0 0 0</pose>
        <geometry><box><size>{w} 0.008 {h}</size></box></geometry>
        {_pbr_block(png, '0.55 0.45 0.30')}
      </visual>
    </link>
  </model>
</sdf>
"""
    (model_dir / 'model.sdf').write_text(sdf, encoding='utf-8')
    (model_dir / 'model.config').write_text(
        f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>PeachDataSet canopy photo card.</description>
</model>
""",
        encoding='utf-8')
    return {
        'name': name,
        'kind': 'scene',
        'png': str(png.resolve()),
        'dir': str(model_dir.resolve()),
    }


def _write_preview_world(out: Path, models: list[dict]) -> Path:
    chunks = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.8">',
        '  <world name="peach_dataset_preview">',
        '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
        '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
        '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
        '    <scene><ambient>0.62 0.62 0.60 1</ambient>',
        '      <background>0.78 0.86 0.92 1</background><grid>true</grid></scene>',
        '    <light type="directional" name="sun">',
        '      <cast_shadows>false</cast_shadows>',
        '      <pose>0 0 8 0 0 0</pose>',
        '      <diffuse>1 1 0.95 1</diffuse>',
        '      <direction>-0.2 0.7 -0.5</direction>',
        '    </light>',
        '    <light type="point" name="fill">',
        '      <pose>0 -1.1 0.55 0 0 0</pose>',
        '      <diffuse>0.85 0.82 0.75 1</diffuse>',
        '      <attenuation><range>8</range></attenuation>',
        '    </light>',
        '    <model name="ground"><static>true</static><link name="link">',
        '      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>6 6</size></plane></geometry>',
        '        <material><diffuse>0.55 0.62 0.48 1</diffuse></material></visual>',
        '      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>6 6</size></plane></geometry></collision>',
        '    </link></model>',
    ]
    bag_i = 0
    for meta in models:
        if meta.get('kind') == 'scene':
            continue
        png = Path(meta['png'])
        obj = Path(meta['obj'])
        name = meta['name']
        col = bag_i % 4
        row = bag_i // 4
        pose = f'{col * 0.38 - 0.57} {-0.05 - row * 0.38} 0 0 0 0'
        bag_i += 1
        chunks.append(
            f'    <model name="{name}"><static>true</static><pose>{pose}</pose>'
            f'{_bag_link_xml(png, obj)}</model>')
    chunks.append(
        '    <gui fullscreen="false">'
        '      <camera_pose>0.15 -1.55 0.42 0 0.32 1.48</camera_pose>'
        '    </gui>')
    chunks.append('  </world>\n</sdf>\n')
    path = out / 'preview.sdf'
    path.write_text('\n'.join(chunks), encoding='utf-8')
    return path


def _collect_candidates(dataset: Path) -> dict[str, list[tuple]]:
    xml_dir = dataset / 'Annotations_VOC' / 'VOC_1label'
    depth_dir = dataset / 'Depth'
    by_cls: dict[str, list[tuple]] = defaultdict(list)
    for xml_path in xml_dir.glob('*.xml'):
        depth_path = depth_dir / f'{xml_path.stem}.png'
        if not depth_path.is_file():
            continue
        depth_m = _load_depth_m(depth_path)
        if depth_m is None:
            continue
        for i, box in enumerate(_parse_boxes(xml_path)):
            cls = box['cls']
            if cls != '0':
                continue
            w = box['xmax'] - box['xmin']
            h = box['ymax'] - box['ymin']
            if w < 90 or h < 90:
                continue
            sl = np.s_[box['ymin']:box['ymax'], box['xmin']:box['xmax']]
            patch = depth_m[sl]
            if patch.size == 0:
                continue
            valid = float((patch > 0.05).mean())
            if valid < 0.22:
                continue
            width_m = (box['xmax'] - box['xmin']) * float(np.median(patch[patch > 0.05])) / _FX
            height_m = (box['ymax'] - box['ymin']) * float(np.median(patch[patch > 0.05])) / _FY
            # 只要接近工业袋尺寸的框（15×18 cm），不要整片树冠大框
            if not (0.06 <= width_m <= 0.28 and 0.08 <= height_m <= 0.30):
                continue
            score = w * h * valid
            by_cls[cls].append((score, xml_path.stem, i, box, valid))
    return by_cls


def _pick(cands: list[tuple], k: int) -> list[tuple]:
    cands = sorted(cands, reverse=True)
    used = set()
    out = []
    for item in cands:
        stem = item[1]
        if stem in used:
            continue
        used.add(stem)
        out.append(item)
        if len(out) >= k:
            break
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--dataset', type=Path,
        default=Path('/home/mu/Downloads/PeachDataSet (1)/Peach_bag'))
    parser.add_argument(
        '--out', type=Path,
        default=Path('runs/scene_models/peach_dataset_gz'))
    args = parser.parse_args()
    dataset = args.dataset
    rgb_dir = dataset / 'RGB'
    depth_dir = dataset / 'Depth'
    if not rgb_dir.is_dir() or not depth_dir.is_dir():
        print(f'找不到 RGB/Depth: {dataset}')
        return 2

    by_cls = _collect_candidates(dataset)
    bags = _pick(by_cls.get('0', []), _N_BAGS * 2)
    print(f'Peach_bag VOC_1label candidates={len(by_cls.get("0", []))} picked={len(bags)}')

    out = args.out
    models_root = out / 'models' / 'Peach_bag'
    models_root.mkdir(parents=True, exist_ok=True)
    from ultralytics import SAM
    if not _SAM_WEIGHTS.is_file():
        print(f'找不到 MobileSAM 权重: {_SAM_WEIGHTS}')
        return 2
    sam = SAM(str(_SAM_WEIGHTS))
    manifest_models = []
    cache_rgb: dict[str, np.ndarray] = {}
    cache_depth: dict[str, np.ndarray] = {}

    def _rgb_depth(stem: str) -> tuple[np.ndarray, np.ndarray]:
        if stem not in cache_rgb:
            try:
                cache_rgb[stem] = _load_rgb(rgb_dir / f'{stem}.png')
            except (OSError, ValueError):
                cache_rgb[stem] = np.zeros((_H, _W, 3), dtype=np.uint8)
            cache_depth[stem] = _load_depth_m(depth_dir / f'{stem}.png')
        rgb, depth_m = cache_rgb[stem], cache_depth[stem]
        if depth_m is None:
            raise FileNotFoundError(f'bad depth {stem}')
        return rgb, depth_m

    for score, stem, idx, box, valid in bags:
        try:
            rgb, depth_m = _rgb_depth(stem)
        except FileNotFoundError:
            print(f'  skip {stem}: bad rgb/depth')
            continue
        raw_w, raw_h, z_med = _pinhole_box_m(depth_m, box)
        mask = _sam_bag_mask(rgb, box, sam)
        if mask is None or float(mask.mean()) < 0.002:
            print(f'  skip {stem}#{idx}: SAM 未抠出袋')
            continue
        tex = _texture_from_mask(rgb, box, mask)
        pad = _PAD_PX
        bounds = (
            max(0, box['xmin'] - pad),
            max(0, box['ymin'] - pad),
            min(_W, box['xmax'] + pad),
            min(_H, box['ymax'] + pad),
        )
        verts, faces, uvs = _irregular_bag_mesh(
            depth_m, mask, tex, bounds, z_med)
        if len(faces) < 40:
            print(f'  skip {stem}#{idx}: 不规则网格太稀')
            continue
        scale = _bag_scale(raw_w, raw_h)
        verts = _apply_scale(verts, scale)
        if len(manifest_models) >= _N_BAGS:
            break
        name = f'peach_bag_{stem}_{idx:02d}'
        meta = _write_bag_model(
            models_root / name, name, tex, verts, faces, uvs)
        meta.update({
            'split': 'Peach_bag',
            'frame': stem,
            'occlusion': 'peach_bag',
            'sam_masked': True,
            'shape': 'irregular_sam_depth_shell',
            'valid_in_box': round(valid, 4),
            'score': round(float(score), 1),
            'pinhole_wh_m': [round(raw_w, 4), round(raw_h, 4)],
            'median_depth_m': round(z_med, 4),
        })
        manifest_models.append(meta)
        print(
            f'  {name}: V={meta["vertices"]} F={meta["faces"]} aabb={meta["aabb_m"]}')

    world = _write_preview_world(out, manifest_models)
    manifest = {
        'source': str(dataset),
        'note': (
            'Only Peach_bag. Visual is an irregular SAM-mask + depth shell '
            '(not a box); collision stays 15x8x18 cm. PBR albedo from bag pixels.'
        ),
        'spec_m': {
            'bag_width': _BAG_W,
            'bag_height': _BAG_H,
            'filled_thickness': _BAG_T,
            'mass_kg': _BAG_MASS,
            'leaf_length': _LEAF_L,
            'leaf_width': _LEAF_W,
        },
        'spec_sources': _SPEC_SOURCES,
        'intrinsics_assumed': {
            'width': _W, 'height': _H,
            'fx': float(_FX), 'fy': float(_FY), 'cx': _CX, 'cy': _CY,
        },
        'models': manifest_models,
        'preview': str(world),
    }
    (out / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(f'wrote {len(manifest_models)} models → {out}')
    return 0 if manifest_models else 1


if __name__ == '__main__':
    raise SystemExit(main())
