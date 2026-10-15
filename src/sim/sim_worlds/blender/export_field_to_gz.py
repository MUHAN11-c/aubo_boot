r"""从重建整园 .blend 导出 Gazebo 查看 GLB（headless）.

用法（Blender 自带 Python，勿混进 aubo_py3.12 / numpy 1.26.4 环境）::

    _tools/blender-4.5.14-linux-x64/blender -b \
        src/sim/sim_worlds/reconstruction/output/field_no_grass_20261009/ \
        bagged_peach_orchard.blend \
        -P src/sim/sim_worlds/blender/export_field_to_gz.py -- \
        --out src/sim/sim_worlds/meshes/field_no_grass.glb [--keep-backdrop] [--decimate 0.25]

流程：删相机/灯 → （默认删 backdrop 远景行实例）→ collection 实例实体化 →
几何节点叶 apply（实例 realize）→ 逐对象烘焙变换（不 join，多节点结构
与 build_assets 树 GLB 同构）→ 可选 decimate → glTF GLB
（export_yup，与 build_assets.py 同参数；Gazebo world 引用
``model://sim_description/meshes/field_no_grass.glb``——sim_description
安装树内的副本，见 worlds/field_no_grass.sdf）。

注意：这不再是"一次性查看桥接"——P2/P4b 起该世界带地面碰撞与
Sensors(ogre2) 渲染，是 harvest.launch.py field 预设的自主采摘世界；
导航/站位 GT 由 generate_field 取 scene_manifest 生成
（maps/field_map、maps/field_stations）。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import bpy


def log(*args) -> None:
    print('EXPORT', *args, flush=True)


def parse_args() -> argparse.Namespace:
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    p = argparse.ArgumentParser()
    p.add_argument('--out', required=True, help='output .glb path')
    p.add_argument('--keep-backdrop', action='store_true',
                   help='keep the 14 backdrop row collection instances')
    p.add_argument('--decimate', type=float, default=0.0,
                   help='decimate ratio after join (0=off, e.g. 0.25)')
    return p.parse_args(argv)


def _find_upstream(node_tree, socket, node_type):
    """Depth-first search upstream of socket for a node of node_type."""
    for link in socket.links:
        node = link.from_node
        if node.type == node_type:
            return node
        for inp in node.inputs:
            if inp.type == 'RGBA':
                found = _find_upstream(node_tree, inp, node_type)
                if found:
                    return found
    return None


def _ramp_mid_color(socket):
    """Mid-element color of the first ValToRGB ramp upstream, else None."""
    for link in socket.links:
        node = link.from_node
        if node.type == 'VAL_TO_RGB':
            elements = node.color_ramp.elements
            return list(elements[len(elements) // 2].color)
        for inp in node.inputs:
            if inp.type == 'RGBA':
                color = _ramp_mid_color(inp)
                if color:
                    return color
    return None


def simplify_materials() -> None:
    """Make Base Color glTF-exportable.

    Reconstruction materials feed Base Color through Mix(noise, image) or
    Noise->ColorRamp chains; the glTF exporter drops them (all-white model
    in Gazebo). Rewire image textures straight into Base Color when found;
    otherwise fall back to the material's diffuse_color (set at creation
    in materials.py, sRGB viewport shade), converted to linear.
    """
    rewired, flattened = 0, 0
    for mat in bpy.data.materials:
        if not mat.use_nodes:
            continue
        bsdf = next((n for n in mat.node_tree.nodes
                     if n.type == 'BSDF_PRINCIPLED'), None)
        if bsdf is None:
            continue
        base = bsdf.inputs['Base Color']
        image = _find_upstream(mat.node_tree, base, 'TEX_IMAGE')
        if image is not None and image.image is not None:
            mat.node_tree.links.new(image.outputs['Color'], base)
            rewired += 1
        else:
            # Unlink the procedural chain first: a socket link overrides
            # default_value, so setting the factor alone exports nothing.
            # diffuse_color stays linear — no sRGB conversion here.
            for link in list(base.links):
                mat.node_tree.links.remove(link)
            color = mat.diffuse_color
            base.default_value = (*color[:3], 1.0)
            flattened += 1
        bsdf.inputs['Metallic'].default_value = 0.0
    log('materials rewired to image:', rewired, 'flattened to diffuse:',
        flattened)


def bake_paper_albedo() -> None:
    """paper* 材质 Base Color 直连 paper.png（UV 与高度贴图同源）.

    2026-10-14 P4b 实证：袋材质 Base Color 是程序化节点（无图片），导出时
    回退纯色 diffuse → gz 里袋子失去纸纹理/折痕，YOLO 对 gz 渲染零检出
    （Blender 渲染同袋 conf 0.8、gz conf<0.05 均无，对照实验在案）。把重建
    管线自己生成的纸袋 albedo（reconstruction/textures/paper.png）直连后，
    gz 材质与 Blender 验收图同分布。
    """
    tex_path = Path(__file__).resolve().parents[1] / \
        'reconstruction' / 'textures' / 'paper.png'
    if not tex_path.exists():
        log('paper.png 缺失，跳过 albedo 直连：', tex_path)
        return
    image = bpy.data.images.load(str(tex_path), check_existing=True)
    baked = 0
    for mat in bpy.data.materials:
        if not mat.name.startswith('Paper / pigment') or not mat.use_nodes:
            continue
        bsdf = next((n for n in mat.node_tree.nodes
                     if n.type == 'BSDF_PRINCIPLED'), None)
        if bsdf is None:
            continue
        tex = mat.node_tree.nodes.new('ShaderNodeTexImage')
        tex.image = image
        tex.label = 'paper_albedo_export'
        mat.node_tree.links.new(tex.outputs['Color'],
                                bsdf.inputs['Base Color'])
        baked += 1
    log('paper albedo baked into Base Color:', baked)


def main() -> None:
    args = parse_args()

    for obj in list(bpy.data.objects):
        if obj.type in {'CAMERA', 'LIGHT'}:
            bpy.data.objects.remove(obj, do_unlink=True)
    log('cameras/lights removed')

    bake_paper_albedo()
    simplify_materials()

    if not args.keep_backdrop:
        empties = [o for o in bpy.data.objects
                   if o.type == 'EMPTY' and o.instance_collection]
        for obj in empties:
            bpy.data.objects.remove(obj, do_unlink=True)
        log('backdrop instances removed:', len(empties))

    inst = [o for o in bpy.data.objects
            if o.type == 'EMPTY' and o.instance_collection]
    if inst:
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.duplicates_make_real(use_base_parent=True,
                                            use_hierarchy=False)
    log('collection instances realized:', len(inst))

    # Realize GN leaf instances the fast way: insert a Realize Instances
    # node before the group output (shared node groups touched once), then
    # materialize each object's evaluated geometry as a plain mesh. The plain
    # modifier_apply route spends tens of minutes creating per-leaf objects;
    # this stays in C++ parallel evaluation.
    gn_objs = [o for o in bpy.data.objects if o.type == 'MESH'
               and any(m.type == 'NODES' for m in o.modifiers)]
    node_groups = {m.node_group for o in gn_objs
                   for m in o.modifiers
                   if m.type == 'NODES' and m.node_group}
    for nt in node_groups:
        for out in [n for n in nt.nodes if n.type == 'GROUP_OUTPUT']:
            for link in list(out.inputs[0].links):
                # Detach the existing link first: an input socket accepts a
                # single connection, so wiring the realize node before the
                # removal raises "unable to place connection".
                from_socket = link.from_socket
                nt.links.remove(link)
                realize = nt.nodes.new('GeometryNodeRealizeInstances')
                nt.links.new(from_socket, realize.inputs[0])
                nt.links.new(realize.outputs[0], out.inputs[0])
        log('realize node inserted into', nt.name)

    depsgraph = bpy.context.evaluated_depsgraph_get()
    # Two phases: evaluate ALL GN objects first, delete originals after.
    # Removing an object mid-loop invalidates the depsgraph handles, which
    # silently yielded empty leaves for every tree but the first.
    realized = []
    for obj in gn_objs:
        ob_eval = obj.evaluated_get(depsgraph)
        mesh_eval = bpy.data.meshes.new_from_object(
            ob_eval, preserve_all_data_layers=True, depsgraph=depsgraph)
        new_ob = bpy.data.objects.new(obj.name + '_real', mesh_eval)
        new_ob.matrix_world = obj.matrix_world.copy()
        bpy.context.scene.collection.objects.link(new_ob)
        realized.append(new_ob)
    for obj in gn_objs:
        bpy.data.objects.remove(obj, do_unlink=True)
    log('gn objects realized:', len(realized), 'leaf verts per tree ~',
        max((len(o.data.vertices) for o in realized
             if 'leaves' in o.name), default=0))

    # Only meshes inside the view layer: GN prototype sources (e.g. leaf
    # proto 0..19) live in collections excluded from rendering, and
    # select_set on them raises "not in view layer". They must not ship in
    # the GLB anyway — realized leaves already replaced them.
    # None entries linger in view_layer.objects after object removal.
    bpy.context.view_layer.update()
    meshes = [o for o in bpy.context.view_layer.objects
              if o is not None and o.type == 'MESH']
    if not meshes:
        raise RuntimeError('no meshes left to export')

    # Keep meshes as separate objects (no join): smaller GLB than a joined
    # single mesh (measured on the 2026-10-09 scene: ~143 MB vs 220 MB; the
    # current full-leaf asset is ~1.6 GB either way) and structural parity
    # with build_assets tree GLBs.
    # Bake each object's world matrix into its vertices via Mesh.transform
    # (pure data op; per-object bpy.ops.transform_apply costs ~1 ms x 13k
    # objects and is too slow). Axis note: the export itself uses
    # export_yup=False — this gz-common 5.9/Assimp chain reads GLB vertices
    # as Z-up, so yup=True data renders as a 90-degree vertical wall.
    for obj in meshes:
        obj.data.transform(obj.matrix_world)
        obj.parent = None
        obj.location = (0.0, 0.0, 0.0)
        obj.rotation_euler = (0.0, 0.0, 0.0)
        obj.scale = (1.0, 1.0, 1.0)
    log('world transforms baked into vertices, meshes kept separate:',
        len(meshes))

    if args.decimate > 0:
        # Only decimate the big leaf meshes (~1.7 M verts each): applying a
        # decimate modifier per object via ops costs ~0.1 s each, and the
        # 13k small objects contribute almost nothing to the file size.
        big = [o for o in meshes if len(o.data.vertices) > 100_000]
        for obj in big:
            mod = obj.modifiers.new('gz_decimate', 'DECIMATE')
            mod.ratio = args.decimate
            bpy.context.view_layer.objects.active = obj
            bpy.ops.object.modifier_apply(modifier=mod.name)
        log('decimated', len(big), 'big meshes at ratio', args.decimate)

    log('verts total:', sum(len(o.data.vertices) for o in meshes))
    bpy.ops.object.select_all(action='DESELECT')
    for obj in meshes:
        obj.select_set(True)
    bpy.ops.export_scene.gltf(filepath=args.out, export_format='GLB',
                              use_selection=True, export_yup=False)
    log('exported (yup=False: vertices stay Blender Z-up, matching what '
        'this gz/Assimp build actually renders upright)', args.out)


main()
