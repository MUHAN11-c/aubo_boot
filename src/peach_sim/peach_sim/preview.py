"""
场景正交预览（纯 PIL 投影，不依赖 OpenGL）.

无 GL 环境起不了 gz GUI / ogre2（Qt Quick 直接 abort），这里把生成的
SDF 几何正交投影成俯视（x–y）与侧视（x–z 作业切片）两张线框图：用途是
**摆位自检**（树行/挂果/工位/可达包络一眼可查）与快速看场景，不是渲染效果图。
输入是 ``worlds/peach_orchard.sdf`` 与同名 manifest——画的就是 gz 会加载的那份几何。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw

from .scene import PLATFORM_MODEL

PANEL_W = 900
PANEL_H = 900
MARGIN = 48
_BG = (245, 245, 240)
_AXIS = (120, 120, 120)
_REACH = (200, 60, 60)
_PLATFORM = (70, 90, 120)
_ARM = (160, 110, 40)


Pose = tuple[tuple[float, float, float], tuple[float, float, float]]


@dataclass(frozen=True)
class Shape:
    """一个视觉几何：kind ∈ sphere|cylinder|box|ellipsoid."""

    kind: str
    center: tuple[float, float, float]
    sizes: tuple[float, ...]
    direction: tuple[float, float, float]
    rgb: tuple[int, int, int]


def _color(text: str | None) -> tuple[int, int, int]:
    if not text:
        return (120, 120, 120)
    parts = [float(item) for item in text.split()]
    return tuple(max(0, min(255, int(round(item * 255.0))))
                 for item in parts[:3])


def _rotate(rpy: tuple[float, float, float], vector: tuple[float, float, float]):
    """按 SDF 约定 Rz(yaw)·Ry(pitch)·Rx(roll) 转方向向量."""
    roll, pitch, yaw = rpy
    x, y, z = vector
    cr, sr = math.cos(roll), math.sin(roll)
    y, z = y * cr - z * sr, y * sr + z * cr
    cp, sp = math.cos(pitch), math.sin(pitch)
    x, z = x * cp + z * sp, -x * sp + z * cp
    cy, sy = math.cos(yaw), math.sin(yaw)
    x, y = x * cy - y * sy, x * sy + y * cy
    return (x, y, z)


def parse_shapes(sdf_text: str) -> list[Shape]:
    """把世界 SDF 的 visual 几何摊平成可投影形状（含模型位姿合成）."""
    shapes: list[Shape] = []
    for model in ET.fromstring(sdf_text).iter('model'):
        pose = _pose(model.findtext('pose'))
        for link in model.findall('link'):
            link_pose = _pose(link.findtext('pose'))
            for visual in link.findall('visual'):
                shape = _shape_of(visual, _add(pose, link_pose))
                if shape is not None:
                    shapes.append(shape)
    return shapes


def _pose(text: str | None) -> 'Pose':
    values = [float(item) for item in (text or '0 0 0 0 0 0').split()]
    values += [0.0] * (6 - len(values))
    return (tuple(values[0:3]), tuple(values[3:6]))


def _add(model_pose, link_pose):
    (mx, my, mz), m_rpy = model_pose
    (lx, ly, lz), l_rpy = link_pose
    offset = _rotate(m_rpy, (lx, ly, lz))
    return ((mx + offset[0], my + offset[1], mz + offset[2]),
            (m_rpy[0] + l_rpy[0], m_rpy[1] + l_rpy[1], m_rpy[2] + l_rpy[2]))


def _shape_of(visual: ET.Element, link_pose) -> Shape | None:
    (px, py, pz), rpy = _pose(visual.findtext('pose'))
    center = (px, py, pz)
    geometry = visual.find('geometry')
    if geometry is None or len(geometry) == 0:
        return None
    kind = geometry[0].tag
    shape = geometry[0]
    rgb = _color(visual.findtext('material/ambient'))
    if kind == 'sphere':
        radius = float(shape.findtext('radius', '0.05'))
        sizes = (radius,)
        direction = (0.0, 0.0, 1.0)
    elif kind == 'cylinder':
        sizes = (float(shape.findtext('radius', '0.02')),
                 float(shape.findtext('length', '0.1')))
        direction = _rotate(rpy, (0.0, 0.0, 1.0))
    elif kind == 'box':
        sizes = tuple(float(item) for item in shape.findtext('size', '0 0 0').split())
        direction = _rotate(rpy, (0.0, 0.0, 1.0))
    elif kind == 'ellipsoid':
        sizes = tuple(float(item) for item in shape.findtext('radii', '0 0 0').split())
        direction = _rotate(rpy, (0.0, 0.0, 1.0))
    else:
        return None
    (ox, oy, oz), _ = link_pose
    return Shape(
        kind=kind,
        center=(ox + center[0], oy + center[1], oz + center[2]),
        sizes=sizes,
        direction=direction,
        rgb=rgb)


class _View:
    """世界 (a, b) → 画布像素的等比映射；a/b ∈ {x, y, z}."""

    def __init__(self, box: tuple[float, float, float, float], a: str, b: str,
                 origin: tuple[int, int]) -> None:
        self.a, self.b = a, b
        self.lo_x, self.lo_y, self.hi_x, self.hi_y = box
        self.origin = origin
        span_x = max(self.hi_x - self.lo_x, 1e-6)
        span_y = max(self.hi_y - self.lo_y, 1e-6)
        self.scale = min((PANEL_W - 2 * MARGIN) / span_x,
                         (PANEL_H - 2 * MARGIN) / span_y)

    def _coord(self, point, axis: str) -> float:
        return {'x': point[0], 'y': point[1], 'z': point[2]}[axis]

    def project(self, point) -> tuple[float, float]:
        a_val = self._coord(point, self.a)
        b_val = self._coord(point, self.b)
        px = self.origin[0] + MARGIN + (a_val - self.lo_x) * self.scale
        py = self.origin[1] + PANEL_H - MARGIN - (b_val - self.lo_y) * self.scale
        return (px, py)

    def extent(self, meters: float) -> float:
        return meters * self.scale


def _draw_shape(draw: ImageDraw.ImageDraw, view: _View, shape: Shape) -> None:
    center = shape.center
    if shape.kind in ('sphere', 'ellipsoid'):
        radii = (shape.sizes if shape.kind == 'ellipsoid'
                 else (shape.sizes[0], shape.sizes[0], shape.sizes[0]))
        # 保守包络：取投影面内两轴的外接半径（倾角小，误差可接受）
        rx = view.extent(max(radii[0], radii[2] * 0.5))
        ry = view.extent(max(radii[1], radii[2] * 0.5))
        cx, cy = view.project(center)
        draw.ellipse([cx - rx, cy - ry, cx + rx, cy + ry],
                     fill=shape.rgb, outline=(60, 60, 60))
        return
    if shape.kind == 'cylinder':
        radius, length = shape.sizes
        half = tuple(item * length / 2.0 for item in shape.direction)
        start = view.project(tuple(center[i] - half[i] for i in range(3)))
        end = view.project(tuple(center[i] + half[i] for i in range(3)))
        draw.line([start, end], fill=shape.rgb,
                  width=max(1, int(round(view.extent(radius) * 2))))
        return
    if shape.kind == 'box':
        sx, sy, sz = shape.sizes
        corners = [
            view.project((center[0] + dx * sx / 2.0,
                          center[1] + dy * sy / 2.0,
                          center[2] + dz * sz / 2.0))
            for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)
        ]
        xs = [item[0] for item in corners]
        ys = [item[1] for item in corners]
        draw.rectangle([min(xs), min(ys), max(xs), max(ys)],
                       fill=shape.rgb, outline=(60, 60, 60))


def render_preview(sdf_text: str, manifest: dict,
                   out_path: Path) -> Path:
    """画俯视 + 作业切片侧视，写 PNG，返回路径."""
    params_box = manifest.get('platform', {})
    platform = params_box.get('vehicle_center', [0.0, 0.0, 0.0])
    shapes = parse_shapes(sdf_text)
    image = Image.new('RGB', (PANEL_W * 2 + 20, PANEL_H), _BG)
    draw = ImageDraw.Draw(image)

    top = _view_box(shapes, 'x', 'y', margin_x=6.0, margin_y=6.0)
    side = _view_box(
        [s for s in shapes if abs(s.center[1] - platform[1]) <= 1.6],
        'x', 'z', margin_x=6.0, margin_y=1.0)
    views = ((top, 'x', 'y', (0, 0), 'TOP x-y'),
             (side, 'x', 'z', (PANEL_W + 20, 0), 'SIDE x-z (work slice)'))

    for box, a, b, origin, title in views:
        view = _View(box, a, b, origin)
        draw.rectangle([origin[0], origin[1],
                        origin[0] + PANEL_W - 1, origin[1] + PANEL_H - 1],
                       outline=_AXIS)
        for shape in shapes:
            if (a, b) == ('x', 'z') and abs(shape.center[1] - platform[1]) > 1.6:
                continue
            _draw_shape(draw, view, shape)
        _draw_platform(draw, view, manifest, (a, b))
        _draw_targets(draw, view, manifest, (a, b))
        draw.text((origin[0] + 12, origin[1] + 10), title, fill=_AXIS)
    image.save(out_path)
    return out_path


def _view_box(shapes, a: str, b: str, margin_x: float,
              margin_y: float) -> tuple[float, float, float, float]:
    coords = {'x': 0, 'y': 1, 'z': 2}
    if not shapes:
        return (-1.0, -1.0, 1.0, 1.0)
    a_vals = [shape.center[coords[a]] for shape in shapes]
    b_vals = [shape.center[coords[b]] for shape in shapes]
    return (min(a_vals) - margin_x, min(b_vals) - margin_y,
            max(a_vals) + margin_x, max(b_vals) + margin_y)


def _draw_platform(draw: ImageDraw.ImageDraw, view: _View, manifest: dict,
                   plane: tuple[str, str]) -> None:
    platform = manifest.get('platform', {})
    center = platform.get('vehicle_center', [0.0, 0.0, 0.0])
    mount = platform.get('arm_mount', [0.0, 0.0, 0.75])
    reach = platform.get('reach_max_m', 1.05)
    cx, cy = view.project(center)
    mx, my = view.project(mount)
    radius = view.extent(reach)
    draw.ellipse([mx - radius, my - radius, mx + radius, my + radius],
                 outline=_REACH)
    half = view.extent(0.55 if plane == ('x', 'y') else 0.45)
    draw.rectangle([cx - view.extent(0.9), cy - half,
                    cx + view.extent(0.9), cy + half],
                   outline=_PLATFORM, width=3)
    draw.ellipse([mx - 4, my - 4, mx + 4, my + 4], fill=_ARM)
    label = f'{PLATFORM_MODEL} mount'
    draw.text((mx + 8, my - 16), label, fill=_ARM)


def _draw_targets(draw: ImageDraw.ImageDraw, view: _View, manifest: dict,
                  plane: tuple[str, str]) -> None:
    for entry in manifest.get('targets', []):
        if plane == ('x', 'z') and abs(entry['bag_bottom'][1] -
                                       manifest['platform']['vehicle_center'][1]
                                       ) > 1.6:
            continue
        bottom = entry['bag_bottom']
        neck = entry['bag_neck']
        color = _REACH if entry['reachable'] else (110, 110, 110)
        draw.line([view.project(bottom), view.project(neck)],
                  fill=color, width=2)
        cx, cy = view.project(entry['fruit_center'])
        radius = max(2.0, view.extent(entry['fruit_diameter'] / 2.0))
        draw.ellipse([cx - radius, cy - radius, cx + radius, cy + radius],
                     fill=color)


def main(argv: list[str] | None = None) -> int:
    """读生成产物 → 写预览 PNG."""
    import yaml

    from .scene import render_scene
    from .params import params_from_dict

    parser = argparse.ArgumentParser(
        prog='scene_preview',
        description='把果园世界 SDF 正交投影成俯视/侧视 PNG（无 GL 环境看场景）')
    parser.add_argument('--package-dir', type=Path,
                        default=Path(__file__).resolve().parents[1])
    parser.add_argument('--out', type=Path, default=None)
    args = parser.parse_args(argv)

    raw = yaml.safe_load(
        (args.package_dir / 'config' / 'orchard.yaml').read_text(encoding='utf-8'))
    params, problems = params_from_dict(raw)
    if problems:
        raise SystemExit('场景参数不合法：' + '；'.join(problems))
    scene = render_scene(params)
    out = args.out or (args.package_dir / 'worlds' / 'peach_orchard.preview.png')
    render_preview(scene.world_sdf, scene.manifest, out)
    print(f'已写 {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
