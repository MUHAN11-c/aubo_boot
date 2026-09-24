"""Blender 无头建模：开心形桃树 + 套袋桃。不导出网格。

布局与袋具口径对齐 src/peach_sim/config/orchard.yaml（行距 4 m、株距 2.2 m、
袋底 1.30–1.42 m、簇间距 0.055 m、倾角 46°·u^1.9）。袋形对照
src/peach_sim/reconstruction/geometry.py 的 bag_rings（平底、肩宽、收口）。
"""

from __future__ import annotations

import math
import os
import random
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'src', 'peach_sim', 'reconstruction')))
from geometry import Leaves, bag_rings, paper_bag, tube  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
TEX = os.path.join(ROOT, 'textures')
BLEND = os.path.join(ROOT, 'scene', 'peach_orchard.blend')
RENDER = os.path.join(ROOT, 'renders')

ROW_SPACING = 4.0   # peach_sim orchard.yaml rows.spacing
TREE_SPACING = 2.2   # peach_sim orchard.yaml rows.tree_spacing
ROWS = 2
TREES = 4


def _clear():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def _image(name):
    path = os.path.join(TEX, name)
    image = bpy.data.images.load(path)
    return image


def _principled(name, color, roughness, image=None, alpha=False):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = (*color, 1.0)
    bsdf.inputs['Roughness'].default_value = roughness
    if image is not None:
        tex = mat.node_tree.nodes.new('ShaderNodeTexImage')
        tex.image = image
        mat.node_tree.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
        if alpha and 'Alpha' in tex.outputs:
            mat.node_tree.links.new(tex.outputs['Alpha'], bsdf.inputs['Alpha'])
            mat.blend_method = 'CLIP'
            mat.surface_render_method = 'DITHERED'
    return mat


def _bump(mat, strength):
    nodes = mat.node_tree.nodes
    bsdf = nodes.get('Principled BSDF')
    tex = next((node for node in nodes if node.type == 'TEX_IMAGE'), None)
    if tex is None:
        return
    bump = nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = strength
    mat.node_tree.links.new(tex.outputs['Color'], bump.inputs['Height'])
    mat.node_tree.links.new(bump.outputs['Normal'], bsdf.inputs['Normal'])


def _tile(mat, scale):
    nodes = mat.node_tree.nodes
    tex = next((node for node in nodes if node.type == 'TEX_IMAGE'), None)
    if tex is None:
        return
    tex.extension = 'REPEAT'
    mapping = nodes.new('ShaderNodeMapping')
    mapping.inputs['Scale'].default_value = (scale, scale, 1.0)
    coord = nodes.new('ShaderNodeTexCoord')
    links = mat.node_tree.links
    links.new(coord.outputs['UV'], mapping.inputs['Vector'])
    links.new(mapping.outputs['Vector'], tex.inputs['Vector'])


def _link(obj):
    bpy.context.scene.collection.objects.link(obj)
    return obj


def _cylinder(name, p0, p1, r0, r1, mat, segments=8):
    direction = p1 - p0
    length = direction.length
    if length < 1e-4:
        return None
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=segments, radius=1.0, depth=1.0, location=(0, 0, 0))
    obj = bpy.context.active_object
    obj.name = name
    for vertex in obj.data.vertices:
        end = vertex.co.z + 0.5
        radius = r0 * (1.0 - end) + r1 * end
        vertex.co.x *= radius
        vertex.co.y *= radius
    obj.scale = (1.0, 1.0, length)
    obj.location = (p0 + p1) * 0.5
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = direction.to_track_quat('Z', 'Y')
    obj.data.materials.append(mat)
    for poly in obj.data.polygons:
        poly.use_smooth = True
    return obj


def _swap_uv(mat):
    """Leaves 的 UV 是 (沿叶长, 横叶宽)，贴图中脉沿图像竖直方向，对调后再采样。"""
    nodes = mat.node_tree.nodes
    tex = next(node for node in nodes if node.type == 'TEX_IMAGE')
    coord = nodes.new('ShaderNodeTexCoord')
    separate = nodes.new('ShaderNodeSeparateXYZ')
    combine = nodes.new('ShaderNodeCombineXYZ')
    links = mat.node_tree.links
    links.new(coord.outputs['UV'], separate.inputs['Vector'])
    links.new(separate.outputs['Y'], combine.inputs['X'])
    links.new(separate.outputs['X'], combine.inputs['Y'])
    links.new(combine.outputs['Vector'], tex.inputs['Vector'])


def _leaf_materials(image):
    mats = {}
    for index, green in enumerate((0.34, 0.40, 0.30, 0.46, 0.28)):
        mat = _principled(
            f'leaf{index}', (0.10, green, 0.08), 0.55 + 0.05 * index, image)
        _swap_uv(mat)
        mats[f'leaf{index}'] = mat
    return mats


def _blocks_bag(tip, bags):
    for item in bags:
        offset = tip - item['center']
        if offset.length < 0.09 and offset.dot(item['face']) > 0.02:
            return True
    return False


def _dress(leaves, start, end, rng, count, bags):
    start, end = Vector(start), Vector(end)
    for index in range(count):
        origin = start.lerp(end, (index + 0.4) / count)
        for _ in range(8):
            direction = Vector((
                rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-0.35, 1.2)))
            if direction.length < 1e-4:
                continue
            tip = origin + direction.normalized() * rng.uniform(0.09, 0.16)
            if _blocks_bag(tip, bags):
                continue
            leaves.add(origin, tip, width=rng.uniform(0.030, 0.046))


def _crease(obj):
    """沿袋厚压折。直槽随高度偏斜，再叠一层低频乱褶，不改外轮廓。"""
    for vert in obj.data.vertices:
        if abs(vert.co.y) < 1e-6:
            continue
        groove = 0.0
        slant = 0.02 * (vert.co.z - 0.08)
        for center in (-0.032, 0.0, 0.028):
            groove += math.exp(-((vert.co.x - center - slant) / 0.014) ** 2)
        wave = math.sin(vert.co.z * 28.0 + vert.co.x * 16.0)
        wave *= math.sin(vert.co.z * 9.0)
        diagonal = 0.0
        for slope, offset in ((2.4, 0.04), (-1.7, 0.10), (0.6, 0.02)):
            dist = abs((vert.co.z - offset) - slope * vert.co.x)
            diagonal += math.exp(-((dist / 0.012) ** 2))
        pinch = 0.006 * groove + 0.0022 * max(0.0, wave) + 0.003 * diagonal
        warp = 0.0024 * math.sin(vert.co.x * 55.0 + vert.co.z * 31.0)
        warp *= math.sin(vert.co.z * 22.0 + 0.7)
        vert.co.y += warp - math.copysign(pinch, vert.co.y)
    obj.data.update()


def _hang(name, neck, paper, rng):
    width = rng.uniform(0.12, 0.16)
    height = rng.uniform(0.15, 0.19)
    thickness = rng.uniform(0.035, 0.05)
    tilt = math.radians(46.0 * (rng.random() ** 1.9))
    yaw = rng.uniform(0, math.tau)
    rings = bag_rings(width, height, thickness, rng.randrange(1, 9999))
    obj = paper_bag(name, rings, paper, rng.randrange(1, 9999))
    _crease(obj)
    obj.rotation_euler = (tilt, tilt * 0.2, yaw)
    rot = obj.rotation_euler.to_matrix()
    local_neck = Vector((rings[30][1], 0.0, height * 0.94))
    obj.location = Vector(neck) - rot @ local_neck
    down = (rot @ Vector((0.0, 0.0, -1.0))).normalized()
    center = Vector(neck) + down * (height * 0.48)
    face = (rot @ Vector((0.0, 1.0, 0.0))).normalized()
    tube(name + '/tie', [
        Vector(neck),
        Vector(neck) + rot @ Vector((0.012, 0.0, 0.0)),
        Vector(neck) + rot @ Vector((0.004, 0.008, 0.002)),
    ], [0.0011, 0.0008, 0.0006], paper, 5)
    return {
        'center': center, 'neck': Vector(neck), 'face': face, 'down': down,
        'bottom': center + down * (height * 0.45),
    }


def _cluster(origin, alley, rng, anchors):
    """沿一条果枝串挂。步距对齐 peach_sim：约 14% 贴袋，其余 9–16 cm。"""
    sign = 1.0 if rng.random() < 0.7 else -0.55
    radial = rng.uniform(0.30, 0.50) * sign
    lateral = rng.uniform(-0.28, 0.28)
    direction = Vector((
        alley.x * rng.uniform(-0.2, 0.35),
        rng.choice((-1.0, 1.0)) * rng.uniform(0.8, 1.0),
        rng.uniform(-0.08, 0.16))).normalized()
    anchor = origin + Vector((alley.x * radial, lateral, rng.uniform(1.48, 1.56)))
    necks = []
    step = 0.0
    for _ in range(rng.randint(3, 5)):
        step += rng.uniform(0.02, 0.035) if rng.random() < 0.14 else rng.uniform(0.09, 0.16)
        bottom = anchor + direction * step
        bottom.z = min(1.58, max(1.46, bottom.z))
        tilt = math.radians(46.0 * (rng.random() ** 1.9))
        yaw = rng.uniform(0, math.tau)
        axis_up = Vector((
            math.sin(tilt) * math.cos(yaw),
            math.sin(tilt) * math.sin(yaw),
            math.cos(tilt)))
        necks.append(bottom + axis_up * rng.uniform(0.062, 0.078))
    root = min(anchors, key=lambda point: (point - necks[0]).length)
    return root, necks


def _tree(name, origin, alley, rng, bark, paper, leaves, bags):
    trunk_h = rng.uniform(0.52, 0.64)
    top = origin + Vector((rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02), trunk_h))
    mid = origin + Vector((0.01, 0.012, trunk_h * 0.5))
    tube(f'{name}/trunk', [origin, mid, top], [0.095, 0.072, 0.055], bark, 12)
    tube(f'{name}/flare', [origin, origin + Vector((0, 0, 0.08))], [0.14, 0.09], bark, 10)
    anchors = []
    for index in range(3):
        if index == 0:
            azimuth = math.atan2(alley.y, alley.x) + rng.uniform(-0.35, 0.35)
        else:
            azimuth = math.pi / 2 + (index - 1.5) * 0.9 + rng.uniform(-0.2, 0.2)
        polar = math.radians(rng.uniform(42, 58))
        direction = Vector((
            math.sin(polar) * math.cos(azimuth),
            math.sin(polar) * math.sin(azimuth),
            math.cos(polar)))
        length = rng.uniform(0.8, 1.1)
        end = top + direction * length
        end.z = min(end.z, 2.3)
        bend = top.lerp(end, 0.5) + Vector((0, 0, 0.05))
        tube(f'{name}/scaffold{index}', [top, bend, end], [0.040, 0.028, 0.016], bark, 8)
        _dress(leaves, top, end, rng, 14, bags)
        for t in (0.4, 0.62, 0.82):
            anchors.append(top.lerp(end, t))
        for side in range(4):
            base = top.lerp(end, rng.uniform(0.45, 0.85))
            shoot = Vector((
                direction.x * 0.25 + rng.uniform(-0.7, 0.7),
                direction.y * 0.25 + rng.uniform(-0.7, 0.7),
                rng.uniform(0.15, 0.85))).normalized()
            tip = base + shoot * rng.uniform(0.28, 0.5)
            tip.z = min(tip.z, 2.35)
            tube(f'{name}/lat{index}_{side}', [base, tip], [0.018, 0.009], bark, 6)
            _dress(leaves, base, tip, rng, 8, bags)
            anchors.append(base.lerp(tip, 0.7))
    for cluster in range(rng.randint(2, 3)):
        root, necks = _cluster(origin, alley, rng, anchors)
        lift = root.lerp(necks[0], 0.4) + Vector((0.0, 0.0, 0.04))
        above = [neck + Vector((0.0, 0.0, 0.025)) for neck in necks]
        points = [root, lift, *above]
        radii = [0.008 * (1.0 - index / len(points)) + 0.003 for index in range(len(points))]
        tube(f'{name}/fruit{cluster}', points, radii, bark, 6)
        for index, (neck, high) in enumerate(zip(necks, above)):
            tube(f'{name}/stem{cluster}_{index}', [high, neck], [0.003, 0.002], bark, 5)
        for index, neck in enumerate(necks):
            bags.append(_hang(f'{name}/bag{cluster}_{index}', neck, paper, rng))
        _dress(leaves, points[1], points[-1], rng, 6, bags)
    for index in range(22):
        base = rng.choice(anchors)
        direction = Vector((
            rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(0.4, 1.1))).normalized()
        tip = base + direction * rng.uniform(0.25, 0.55)
        tip.z = min(max(tip.z, 1.45), 2.4)
        tube(f'{name}/crown{index}', [base, tip], [0.011, 0.005], bark, 5)
        _dress(leaves, base, tip, rng, 10, bags)
    _canopy(leaves, anchors, rng, bags)


def _canopy(leaves, anchors, rng, bags):
    """叶从枝上长出，袋面 14 cm 内不插叶。"""
    for anchor in anchors:
        for _ in range(14):
            direction = Vector((
                rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-0.3, 1.1)))
            tip = anchor + direction.normalized() * rng.uniform(0.07, 0.15)
            if _blocks_bag(tip, bags):
                continue
            leaves.add(anchor, tip, width=rng.uniform(0.030, 0.046))


def _ground(soil, grass):
    center = Vector(((ROWS - 1) * ROW_SPACING / 2, (TREES - 1) * TREE_SPACING / 2, 0))
    bpy.ops.mesh.primitive_plane_add(size=22, location=center)
    ground = bpy.context.active_object
    ground.name = 'ground'
    ground.data.materials.append(grass)
    for row in range(ROWS):
        bpy.ops.mesh.primitive_plane_add(
            size=1.0, location=(row * ROW_SPACING, center.y, 0.008))
        strip = bpy.context.active_object
        strip.name = f'soil_{row}'
        strip.scale = (1.1, (TREES - 1) * TREE_SPACING + 2.4, 1.0)
        strip.data.materials.append(soil)


def _tufts(grass):
    bpy.ops.mesh.primitive_plane_add(size=0.08, location=(0, 0, 0))
    proto = bpy.context.active_object
    proto.name = 'tuft_proto'
    proto.rotation_euler = (math.radians(80), 0, 0)
    proto.data.materials.append(grass)
    rng = random.Random(7)
    for _ in range(180):
        x = rng.uniform(-1.2, ROW_SPACING + 1.2)
        y = rng.uniform(-0.8, (TREES - 1) * TREE_SPACING + 0.8)
        if min(abs(x), abs(x - ROW_SPACING)) < 0.45:
            continue
        copy = proto.copy()
        copy.data = proto.data
        copy.location = (x, y, 0.02)
        copy.rotation_euler = (math.radians(rng.uniform(65, 88)), 0, rng.uniform(0, math.tau))
        copy.scale = (rng.uniform(0.6, 1.4), rng.uniform(0.8, 1.8), 1)
        _link(copy)
    proto.hide_render = True


def _world():
    world = bpy.data.worlds.new('orchard')
    bpy.context.scene.world = world
    world.use_nodes = True
    nodes = world.node_tree.nodes
    links = world.node_tree.links
    bg = nodes['Background']
    sky = nodes.new('ShaderNodeTexSky')
    sky.sky_type = 'NISHITA'
    sky.sun_elevation = math.radians(42)
    sky.sun_rotation = math.radians(135)
    links.new(sky.outputs['Color'], bg.inputs['Color'])
    bg.inputs['Strength'].default_value = 0.55
    bpy.ops.object.light_add(type='SUN', location=(0, 0, 8))
    sun = bpy.context.active_object
    sun.data.energy = 3.2
    sun.rotation_euler = (math.radians(48), 0, math.radians(140))
    sun.data.angle = math.radians(6)


def _aim(name, eye, target, lens):
    bpy.ops.object.camera_add(location=eye)
    cam = bpy.context.active_object
    cam.name = name
    cam.data.lens = lens
    cam.data.sensor_width = 36.0
    cam.rotation_euler = (Vector(target) - Vector(eye)).to_track_quat('-Z', 'Y').to_euler()
    return cam


def _cameras(bags):
    scene = bpy.context.scene
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.engine = 'BLENDER_EEVEE_NEXT'
    scene.eevee.taa_render_samples = 32
    alley_focus = Vector((0.35, TREE_SPACING, 1.38))
    _aim('cam_alley', (1.85, TREE_SPACING - 0.9, 1.48), alley_focus, 24)
    if bags:
        near = [item for item in bags if item['center'].x < 1.2]
        near.sort(key=lambda item: abs(item['center'].y - TREE_SPACING))
        group = near[:5] or bags[:5]
        center = sum((item['center'] for item in group), Vector()) / len(group)
        eye = center + Vector((0.95, 0.05, -0.12))
        if eye.x < 1.05:
            eye.x = 1.05
        _aim('cam_up', eye, center, 18)
        _aim('cam_block', center + Vector((1.05, -0.55, 0.12)),
             center + Vector((0.0, 1.1, -0.05)), 20)
    _aim('cam_rows', (ROW_SPACING / 2, -0.6, 1.55),
         (ROW_SPACING / 2, TREE_SPACING * 2, 1.35), 22)
    scene.camera = bpy.data.objects['cam_alley']


def _render_named(name):
    scene = bpy.context.scene
    scene.camera = bpy.data.objects[name]
    path = os.path.join(RENDER, f'{name}.png')
    scene.render.filepath = path
    scene.render.image_settings.file_format = 'PNG'
    bpy.ops.render.render(write_still=True)
    print('rendered', path)


def main():
    _clear()
    os.makedirs(os.path.dirname(BLEND), exist_ok=True)
    os.makedirs(RENDER, exist_ok=True)
    bark = _principled('bark', (0.36, 0.25, 0.16), 0.85, _image('bark.png'))
    paper = _principled('paper', (168 / 255, 52 / 255, 36 / 255), 0.78, _image('paper.png'))
    _bump(paper, 1.1)
    _bump(bark, 0.25)
    soil = _principled('soil', (0.34, 0.24, 0.14), 0.95, _image('soil.png'))
    _tile(soil, 4.0)
    grass = _principled('grass', (0.22, 0.38, 0.12), 0.9, _image('grass.png'))
    _tile(grass, 5.0)
    leaves = Leaves(_leaf_materials(_image('leaf_surface.png')), seed=31)
    _world()
    _ground(soil, grass)
    bags = []
    for row in range(ROWS):
        alley = Vector((1.0 if row == 0 else -1.0, 0.0, 0.0))
        for col in range(TREES):
            origin = Vector((row * ROW_SPACING, col * TREE_SPACING, 0.0))
            rng = random.Random(20260924 + row * 17 + col)
            _tree(f'tree_{row}_{col}', origin, alley, rng, bark, paper, leaves, bags)
    leaves.finish('canopy')
    print('bags', len(bags))
    _cameras(bags)
    bpy.ops.wm.save_as_mainfile(filepath=BLEND)
    for name in ('cam_alley', 'cam_up', 'cam_rows', 'cam_block'):
        _render_named(name)


if __name__ == '__main__':
    main()
