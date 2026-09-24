r"""
Blender 建真实网格并导出 gz 可用模型（headless）.

用法（Blender 自带 Python，勿混进 aubo_py3.12 / numpy 1.26.4 环境）::

    _tools/blender-4.5.14-linux-x64/blender -b -P src/peach_sim/blender/build_assets.py \\
        -- --out src/peach_sim/meshes

产物（供 SDF ``model://peach_sim/meshes/...`` 引用）：

* ``bagged_peach.glb/.dae`` 套袋桃：扁筒袋体（皱褶 + 袋底立体折边）+ 果胀 +
  袋口收拢褶 + 封口铁丝（螺旋 + 翘尾）+ 内衬遮光纸层；尺寸口径 = 名义值，
  场景里按每颗袋的实参做非均匀缩放（宽/厚/长随 config 区间采样）。
* ``peach_tree_a/b.glb/.dae`` 桃树两变体：收分主干 + 根颈 + 开心形主枝 +
  枝组 + 叶幕团块（icosphere 噪声位移出自然轮廓），UV 挂树皮/叶幕贴图。

材质用 glTF PBR（Base Color/Roughness/Normal），贴图取自 worlds/textures/
（peach_sim.textures 同源）。几何全部程序化 + 噪声位移，seed 固定可复现。
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys

import bmesh
import bpy
from mathutils import noise, Vector

# 名义尺寸（米）：与 config/orchard.yaml 的 bag 区间中值一致，实例按比例缩放
BAG_W = 0.075       # 袋宽（含纸；=实例均值，缩放不失真）
BAG_T = 0.052       # 袋厚（扁平）
BAG_L = 0.088       # 袋纸长（=实例均值 bottom_to_neck+neck_length）
FRUIT_D = 0.051     # 果径（裸桃实测 52mm）
NECK_D = 0.030      # 袋颈直径
NECK_L = 0.028      # 袋颈长

TRUNK_H = 0.58
TRUNK_R = 0.052


def _new_material(name: str, color, roughness: float, metallic: float = 0.0,
                  albedo=None, normal=None):
    """Principled 材质；给了贴图就接 Base Color / Normal."""
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    bsdf = nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = (*color, 1.0)
    bsdf.inputs['Roughness'].default_value = roughness
    bsdf.inputs['Metallic'].default_value = metallic
    links = material.node_tree.links
    if albedo and os.path.exists(albedo):
        texture = nodes.new('ShaderNodeTexImage')
        texture.image = bpy.data.images.load(albedo)
        links.new(texture.outputs['Color'], bsdf.inputs['Base Color'])
    if normal and os.path.exists(normal):
        image = nodes.new('ShaderNodeTexImage')
        image.image = bpy.data.images.load(normal)
        image.image.colorspace_settings.name = 'Non-Color'
        converter = nodes.new('ShaderNodeNormalMap')
        links.new(image.outputs['Color'], converter.inputs['Color'])
        links.new(converter.outputs['Normal'], bsdf.inputs['Normal'])
    return material


def _mesh_from_bmesh(name: str, bm: bmesh.BMesh) -> bpy.types.Object:
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def _shade_smooth(obj: bpy.types.Object) -> None:
    for polygon in obj.data.polygons:
        polygon.use_smooth = True


def _cylindrical_uv(obj: bpy.types.Object) -> None:
    """柱面 UV：绕轴 u、沿轴 v（树干/袋体贴图口径）."""
    mesh = obj.data
    if not mesh.uv_layers:
        mesh.uv_layers.new(name='UVMap')
    layer = mesh.uv_layers.active.data
    xs = [vertex.co.x for vertex in mesh.vertices]
    ys = [vertex.co.y for vertex in mesh.vertices]
    zs = [vertex.co.z for vertex in mesh.vertices]
    z_min, z_max = min(zs), max(zs)
    for polygon in mesh.polygons:
        for index in polygon.loop_indices:
            vertex = mesh.vertices[mesh.loops[index].vertex_index].co
            u = (math.atan2(vertex.y, vertex.x) / (2.0 * math.pi)) % 1.0
            v = (vertex.z - z_min) / max(z_max - z_min, 1e-9)
            layer[index].uv = (u, v)
    del xs, ys


def _tapered_tube(bm: bmesh.BMesh, rings, segments: int, creases: float,
                  squash: float = 1.0, seed: int = 0,
                  origin: Vector = None, direction: Vector = None):
    """
    沿 +Z 铺一圈圈截面点并放样成筒：ring = (z, radius, flatten).

    ``creases`` 是纵向褶皱幅度（0=光圆筒），``squash`` 是 Y 向压扁系数。
    """
    rng = random.Random(seed)
    phase = rng.uniform(0.0, math.pi * 2.0)
    rotation = None
    if direction is not None:
        rotation = direction.to_track_quat('Z', 'Y')
    rows = []
    power = 3.6  # 超椭圆指数（真袋是平直折边枕形）：1=椭圆，>2 趋矩形（袋面平直、折缝圆角）
    for z, radius, flatten in rings:
        row = []
        for index in range(segments):
            phi = 2.0 * math.pi * index / segments
            ripple = 1.0 + creases * math.sin(3.0 * phi + phase) \
                + 0.5 * creases * math.sin(7.0 * phi + phase * 1.7)
            c, s = math.cos(phi), math.sin(phi)
            shape = (abs(c) ** power + abs(s) ** power) ** (-1.0 / power)
            point = Vector((
                radius * ripple * shape * c,
                radius * ripple * shape * s * flatten * squash,
                z))
            if rotation is not None:
                point = rotation @ point
            if origin is not None:
                point = point + origin
            row.append(bm.verts.new(point))
        rows.append(row)
    for lower, upper in zip(rows, rows[1:]):
        for index in range(segments):
            bm.faces.new((lower[index], lower[(index + 1) % segments],
                          upper[(index + 1) % segments], upper[index]))
    return rows


def build_bag(out_dir: str, textures: str) -> list[str]:
    """套袋桃：袋体 + 折边 + 收拢颈 + 铁丝 + 内衬 + 果."""
    bm = bmesh.new()
    half_l = BAG_L / 2.0
    body_l = BAG_L - NECK_L

    # 袋体：底部圆鼓（果胀）、中段直筒微收、上段收拢
    rings = [
        (-half_l + 0.004, BAG_W * 0.32, 0.90),
        (-half_l + 0.016, BAG_W * 0.46, 0.88),
        (-half_l + 0.040, BAG_W * 0.50, 0.86),
        (-half_l + body_l * 0.55, BAG_W * 0.50, 0.85),
        (-half_l + body_l * 0.86, BAG_W * 0.46, 0.88),
        (-half_l + body_l, BAG_W * 0.30, 0.92),
    ]
    rows = _tapered_tube(bm, rings, segments=36, creases=0.075, seed=7)
    # 封底（袋底）
    bm.faces.new(list(reversed(rows[0])))
    # 袋底立体折边：底部外翻一圈窄带
    fold = []
    for index in range(28):
        phi = 2.0 * math.pi * index / 28
        fold.append(bm.verts.new((
            BAG_W * 0.52 * math.cos(phi),
            BAG_W * 0.52 * math.sin(phi) * 0.64,
            -half_l + 0.010)))
    for index in range(28):
        bm.faces.new((rows[0][index], rows[0][(index + 1) % 28],
                      fold[(index + 1) % 28], fold[index]))

    # 袋颈：收拢成细管（密褶）
    neck_rings = [
        (-half_l + body_l, NECK_D * 0.62, 0.9),
        (-half_l + body_l + NECK_L * 0.5, NECK_D * 0.5, 0.9),
        (half_l, NECK_D * 0.5, 0.9),
    ]
    neck_rows = _tapered_tube(bm, neck_rings, segments=20, creases=0.10, seed=11)
    bm.faces.new(neck_rows[-1])
    # 袋口散开的纸边（真实袋口翻起的毛边层次）
    rim_rng = random.Random(23)
    top = neck_rows[-1]
    for index in range(0, 20, 3):
        base = top[index]
        tip = bm.verts.new(base.co + Vector((
            rim_rng.uniform(-0.012, 0.012),
            rim_rng.uniform(-0.012, 0.012),
            rim_rng.uniform(0.006, 0.020))))
        side = top[(index + 1) % 20]
        bm.faces.new((base, side, tip))
    # 揉皱不规则位移：真实纸袋是被揉压过的不规则褶皱面（检测器认这种起伏）
    for vertex in bm.verts:
        radius = math.hypot(vertex.co.x, vertex.co.y)
        if radius < 1e-6:
            continue
        crumple = noise.noise(vertex.co * 28.0 + Vector((7.0, 0, 0)))
        broad = noise.noise(vertex.co * 9.0 + Vector((0, 7.0, 0)))
        scale = 1.0 + 0.05 * crumple + 0.03 * broad
        vertex.co.x *= scale
        vertex.co.y *= scale
    body = _mesh_from_bmesh('bag_body', bm)
    body.data.materials.append(_new_material(
        'paper', (0.60, 0.39, 0.33), 0.72,
        albedo=os.path.join(textures, 'paper_bag_albedo.png'),
        normal=os.path.join(textures, 'paper_bag_normal.png')))
    _shade_smooth(body)
    _cylindrical_uv(body)

    # 内衬遮光纸层：袋口深色内圈
    bm = bmesh.new()
    inner = _tapered_tube(bm, [
        (-half_l + body_l + NECK_L * 0.35, NECK_D * 0.42, 1.0),
        (half_l - 0.002, NECK_D * 0.42, 1.0),
    ], segments=18, creases=0.0)
    bm.faces.new(inner[-1])
    liner = _mesh_from_bmesh('inner_paper', bm)
    liner.data.materials.append(_new_material('inner', (0.18, 0.15, 0.13), 0.9))
    _shade_smooth(liner)

    # 封口铁丝：绕袋颈两圈螺旋 + 翘起的线尾
    bm = bmesh.new()
    wire_radius = NECK_D * 0.62
    turns, samples = 2.0, 48
    points = []
    for index in range(samples):
        t = index / (samples - 1.0)
        phi = 2.0 * math.pi * turns * t
        z = -half_l + body_l + NECK_L * (0.15 + 0.7 * t)
        points.append(Vector((wire_radius * math.cos(phi),
                              wire_radius * math.sin(phi), z)))
    tail_start = points[-1]
    for step in range(1, 3):
        points.append(tail_start + Vector((0.003 * step, 0.001 * step,
                                           0.002 * step)))
    radius_wire = 0.0009
    sides = 6
    rows = []
    for center in points:
        rows.append([bm.verts.new((
            center.x + radius_wire * math.cos(2.0 * math.pi * s / sides),
            center.y + radius_wire * math.sin(2.0 * math.pi * s / sides),
            center.z)) for s in range(sides)])
    for lower, upper in zip(rows, rows[1:]):
        for index in range(sides):
            bm.faces.new((lower[index], lower[(index + 1) % sides],
                          upper[(index + 1) % sides], upper[index]))
    wire = _mesh_from_bmesh('tie_wire', bm)
    wire.data.materials.append(_new_material('wire', (0.45, 0.40, 0.30), 0.5,
                                             metallic=0.3))
    _shade_smooth(wire)

    # 果：略扁球 + 缝合线压痕（贴图已有缝合线，几何只做微起伏）
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=24, v_segments=16,
                              radius=FRUIT_D / 2.0)
    for vertex in bm.verts:
        phi = math.atan2(vertex.co.y, vertex.co.x)
        suture = math.exp(-((math.sin(phi)) ** 2) * 40.0)
        vertex.co *= 1.0 - 0.03 * suture
        vertex.co.z *= 1.06
    fruit = _mesh_from_bmesh('fruit', bm)
    fruit.location = (0.0, 0.0, -half_l + FRUIT_D * 0.85)   # 果藏袋腹内
    fruit.data.materials.append(_new_material(
        'fruit_skin', (0.85, 0.78, 0.52), 0.55,
        albedo=os.path.join(textures, 'fruit_skin_albedo.png')))
    _shade_smooth(fruit)

    return _export(out_dir, 'bagged_peach', [body, liner, wire, fruit])


def build_tree(out_dir: str, textures: str, variant: str, seed: int) -> list[str]:
    """桃树：收分主干 + 根颈 + 开心形主枝/枝组 + 叶幕团块."""
    rng = random.Random(seed)
    bark = _new_material(
        'bark', (0.34, 0.26, 0.19), 0.92,
        albedo=os.path.join(textures, 'bark_albedo.png'),
        normal=os.path.join(textures, 'bark_normal.png'))
    leaf = _new_material(
        'leaf', (0.23, 0.35, 0.16), 0.85,
        albedo=os.path.join(textures, 'leaf_albedo.png'),
        normal=os.path.join(textures, 'leaf_normal.png'))

    wood = []
    # 主干两段收分 + 根颈
    bm = bmesh.new()
    _tapered_tube(bm, [
        (0.0, TRUNK_R * 1.5, 1.0),
        (0.05, TRUNK_R * 1.15, 1.0),
        (TRUNK_H * 0.55, TRUNK_R, 1.0),
        (TRUNK_H, TRUNK_R * 0.7, 1.0),
    ], segments=18, creases=0.05, seed=seed)
    trunk = _mesh_from_bmesh('trunk', bm)
    for vertex in trunk.data.vertices:
        vertex.co.x *= 1.0 + 0.05 * noise.noise(vertex.co * 14.0)
        vertex.co.y *= 1.0 + 0.05 * noise.noise(vertex.co * 14.0 + Vector((0, 0, 3)))
    trunk.data.materials.append(bark)
    _shade_smooth(trunk)
    _cylindrical_uv(trunk)
    wood.append(trunk)

    # 开心形主枝 + 枝组（沿行向两侧张开）
    pieces = []  # (center, direction, length, radius)
    branch_objects = []
    for index in range(3):
        azimuth = math.pi / 2.0 + (index - 1) * (0.85 + rng.uniform(-0.1, 0.1))
        polar = math.radians(rng.uniform(45.0, 60.0))
        direction = Vector((math.sin(polar) * math.cos(azimuth),
                            math.sin(polar) * math.sin(azimuth),
                            math.cos(polar)))
        length = rng.uniform(0.60, 0.95)
        start = Vector((0, 0, TRUNK_H * 0.92))
        pieces.append((start + direction * length * 0.5, direction, length,
                       TRUNK_R * 0.42))
        branch_objects.append((start, direction, length, azimuth))
        for shoot in range(rng.randint(2, 3)):
            anchor = start + direction * length * rng.uniform(0.45, 0.9)
            shoot_dir = Vector((
                direction.x * 0.25 + rng.uniform(-0.6, 0.6),
                direction.y * 0.25 + rng.uniform(-0.6, 0.6),
                rng.uniform(-0.1, 0.8))).normalized()
            shoot_len = rng.uniform(0.20, 0.35)
            pieces.append((anchor + shoot_dir * shoot_len * 0.5, shoot_dir,
                           shoot_len, TRUNK_R * 0.2))
    bm = bmesh.new()
    for center, direction, length, radius in pieces:
        _tapered_tube(bm, [
            (-length / 2.0, radius * 1.2, 1.0),
            (length / 2.0, radius * 0.5, 1.0),
        ], segments=10, creases=0.03, seed=seed + int(center.z * 100),
            origin=center, direction=direction)
    branches = _mesh_from_bmesh('branches', bm)
    branches.data.materials.append(bark)
    _shade_smooth(branches)
    wood.append(branches)

    # 叶幕团块：icosphere + 噪声位移（自然轮廓）
    bm = bmesh.new()
    blob_specs = []
    for index in range(rng.randint(11, 15)):
        start, direction, length, _azimuth = branch_objects[
            index % len(branch_objects)]
        anchor = start + direction * length * rng.uniform(0.5, 1.0)
        center = Vector((
            anchor.x * 0.75 + rng.uniform(-0.3, 0.3),
            anchor.y * 0.85 + rng.uniform(-0.4, 0.4),
            rng.uniform(1.15, 2.05)))
        radius = rng.uniform(0.18, 0.32)
        blob_specs.append((Vector(center), radius))
        blob = bmesh.new()
        bmesh.ops.create_icosphere(blob, subdivisions=4, radius=radius)
        for vertex in blob.verts:
            displacement = noise.noise(vertex.co * 6.0 + Vector((seed, 0, 0)))
            fine = noise.noise(vertex.co * 18.0 + Vector((0, seed, 0)))
            leaf_noise = noise.noise(vertex.co * 55.0 + Vector((0, 0, seed)))
            vertex.co *= 1.0 + 0.20 * displacement + 0.08 * fine + 0.05 * leaf_noise
            vertex.co.z *= 0.85
        blob.transform(_translate(center))
        bm_me = bpy.data.meshes.new('blob')
        blob.to_mesh(bm_me)
        blob.free()
        bm.from_mesh(bm_me)
        bpy.data.meshes.remove(bm_me)
    canopy = _mesh_from_bmesh('canopy', bm)
    canopy.data.materials.append(leaf)
    _shade_smooth(canopy)
    canopy_specs = list(blob_specs)

    # 叶卡（alpha clip）：离散叶片制造叶级对比度（真叶有透光缝隙）
    # 几何叶片（不透明）：判定性实验证明 alpha 叶卡在本机 gz/ogre2 几乎不可见
    # （红球对照：相机/URI 通，但叶卡帧高频仅 0.07）→ 改零 alpha 依赖的几何叶
    cards = []
    card_rng = random.Random(seed + 500)
    for _ in range(600):   # 叶卡改密（叶簇尺度细节，几何级高频）
        # 叶卡撒在冠层球壳外侧（半径≈球半径）：球内会被冠层面挡住
        if canopy_specs:
            centre_b, radius_b = canopy_specs[
                card_rng.randrange(len(canopy_specs))]
            direction = Vector((card_rng.uniform(-1, 1), card_rng.uniform(-1, 1),
                                card_rng.uniform(-1, 1))).normalized()
            centre = centre_b + direction * radius_b * card_rng.uniform(0.9, 1.15)
        else:
            centre = Vector((card_rng.uniform(-0.8, 0.8),
                             card_rng.uniform(-0.9, 0.9),
                             card_rng.uniform(1.0, 2.1)))
        size = card_rng.uniform(0.04, 0.09)   # 叶片尺度
        mesh = bpy.data.meshes.new('card')
        card = bpy.data.objects.new('leaf_card', mesh)
        bpy.context.collection.objects.link(card)
        verts = [Vector((-size, -size, 0)), Vector((size, -size, 0)),
                 Vector((size, size, 0)), Vector((-size, size, 0))]
        faces = [(0, 1, 2, 3)]
        mesh.from_pydata(verts, [], faces)
        mesh.update()
        card.location = centre
        card.rotation_euler = (
            card_rng.uniform(0.0, 3.14), card_rng.uniform(0.0, 3.14),
            card_rng.uniform(0.0, 6.28))
        card.data.materials.append(leaf)
        cards.append(card)

    return _export(out_dir, f'peach_tree_{variant}', wood + [canopy] + cards)

    return _export(out_dir, f'peach_tree_{variant}', wood + [canopy])


def _translate(center: Vector):
    from mathutils import Matrix
    return Matrix.Translation(center)


def _unused() -> None:
    """占位（保留导入一致性）."""
    return None


def _export(out_dir: str, name: str, objects) -> list[str]:
    os.makedirs(out_dir, exist_ok=True)
    bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    written = []
    glb = os.path.join(out_dir, f'{name}.glb')
    bpy.ops.export_scene.gltf(filepath=glb, export_format='GLB',
                              use_selection=True, export_yup=True)
    written.append(glb)
    dae = os.path.join(out_dir, f'{name}.dae')
    bpy.ops.wm.collada_export(filepath=dae, selected=True)
    written.append(dae)
    return written


def main() -> int:
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='meshes')
    parser.add_argument('--textures', default=None)
    args = parser.parse_args(argv)

    textures = args.textures or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'worlds', 'textures')
    bpy.ops.wm.read_factory_settings(use_empty=True)

    written = build_bag(args.out, textures)
    written += build_tree(args.out, textures, 'a', seed=11)
    written += build_tree(args.out, textures, 'b', seed=37)
    for path in written:
        print('已写', path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
