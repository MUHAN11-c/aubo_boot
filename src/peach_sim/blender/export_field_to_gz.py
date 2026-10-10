r"""从重建整园 .blend 导出 Gazebo 查看 GLB（headless）.

用法（Blender 自带 Python，勿混进 aubo_py3.12 / numpy 1.26.4 环境）::

    _tools/blender-4.5.14-linux-x64/blender -b \
        src/peach_sim/reconstruction/output/field_no_grass_20261009/bagged_peach_orchard.blend \
        -P src/peach_sim/blender/export_field_to_gz.py -- \
        --out src/peach_sim/meshes/field_no_grass.glb [--keep-backdrop] [--decimate 0.25]

流程：删相机/灯 → （默认删 backdrop 远景行实例）→ collection 实例实体化 →
几何节点叶 apply（实例 realize）→ 逐对象烘焙变换（不 join，多节点结构
与 build_assets 树 GLB 同构）→ 可选 decimate → glTF GLB
（export_yup，与 build_assets.py 同参数；Gazebo world 以
``model://peach_sim/meshes/field_no_grass.glb`` 引用，见 worlds/field_no_grass.sdf）。

注意：这是"重建 GT → Gazebo 查看"的一次性桥接产物，供人工查看整园效果，
不替代 worlds/peach_orchard.sdf 的程序化锚定世界（那才是导航轮 GT）。
"""

from __future__ import annotations

import argparse
import sys
import time

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


def main() -> None:
    args = parse_args()

    for obj in list(bpy.data.objects):
        if obj.type in {'CAMERA', 'LIGHT'}:
            bpy.data.objects.remove(obj, do_unlink=True)
    log('cameras/lights removed')

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
    for obj in gn_objs:
        ob_eval = obj.evaluated_get(depsgraph)
        mesh_eval = bpy.data.meshes.new_from_object(
            ob_eval, preserve_all_data_layers=True, depsgraph=depsgraph)
        new_ob = bpy.data.objects.new(obj.name + '_real', mesh_eval)
        new_ob.matrix_world = obj.matrix_world.copy()
        bpy.context.scene.collection.objects.link(new_ob)
        bpy.data.objects.remove(obj, do_unlink=True)
    log('gn objects realized:', len(gn_objs))

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

    # Keep meshes as separate objects (no join). gz-common 5.9 loads glb
    # through Assimp: a single-node single-mesh glb rendered rotated 90 deg
    # (vertical ground wall) while multi-node glbs (build_assets trees)
    # load upright — match the proven structure. Bake each object's world
    # matrix into its vertices via Mesh.transform (pure data op; per-object
    # bpy.ops.transform_apply costs ~1 ms x 13k objects and is too slow).
    for obj in meshes:
        obj.data.transform(obj.matrix_world)
        obj.parent = None
        obj.location = (0.0, 0.0, 0.0)
        obj.rotation_euler = (0.0, 0.0, 0.0)
        obj.scale = (1.0, 1.0, 1.0)
    log('world transforms baked into vertices, meshes kept separate:',
        len(meshes))

    if args.decimate > 0:
        for obj in meshes:
            mod = obj.modifiers.new('gz_decimate', 'DECIMATE')
            mod.ratio = args.decimate
            bpy.context.view_layer.objects.active = obj
            bpy.ops.object.modifier_apply(modifier=mod.name)
        log('decimated every mesh at ratio', args.decimate)

    log('verts total:', sum(len(o.data.vertices) for o in meshes))
    bpy.ops.object.select_all(action='DESELECT')
    for obj in meshes:
        obj.select_set(True)
    bpy.ops.export_scene.gltf(filepath=args.out, export_format='GLB',
                              use_selection=True, export_yup=True)
    log('exported', args.out)


main()
