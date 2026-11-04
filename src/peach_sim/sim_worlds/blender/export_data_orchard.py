"""Export a small data-referenced orchard with separate harvestable rigid bags.

Run with Blender 4.5 on an existing reconstruction .blend, passing --out.
Tree topology/placement remain inferred; visible appearance and size priors
come from the existing PeachDataSet reconstruction, not a surveyed orchard.
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import bpy

from mathutils import Matrix, Vector


def digest(path):
    """Fingerprint the exact input/output rather than an asset revision label."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def export_meshes(objects, path, origin):
    """Bake evaluated world vertices into one explicit Z-up local frame."""
    bpy.ops.object.select_all(action='DESELECT')
    temporary = []
    graph = bpy.context.evaluated_depsgraph_get()
    for source in objects:
        data = bpy.data.meshes.new_from_object(
            source.evaluated_get(graph), preserve_all_data_layers=True, depsgraph=graph)
        data.transform(Matrix.Translation(-Vector(origin)) @ source.matrix_world)
        obj = bpy.data.objects.new('export_' + source.name, data)
        bpy.context.scene.collection.objects.link(obj)
        obj.select_set(True)
        temporary.append(obj)
    if not temporary:
        raise ValueError('Empty asset: ' + str(path))
    bpy.context.view_layer.objects.active = temporary[0]
    bpy.ops.export_scene.gltf(filepath=str(path), export_format='GLB',
                              use_selection=True, export_yup=False,
                              export_cameras=False, export_lights=False)
    for obj in temporary:
        data = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        bpy.data.meshes.remove(data)


def simplify_materials():
    """Retain image albedo or measured material colour in the glTF renderer."""
    for material in bpy.data.materials:
        if not material.use_nodes:
            continue
        bsdf = next((n for n in material.node_tree.nodes
                     if n.type == 'BSDF_PRINCIPLED'), None)
        if bsdf is None:
            continue
        base = bsdf.inputs['Base Color']
        for link in list(base.links):
            material.node_tree.links.remove(link)
        base.default_value = material.diffuse_color
        bsdf.inputs['Metallic'].default_value = 0.0
        # The existing packed paper albedo is authored in this same pipeline.
        if material.name.startswith('Paper / pigment'):
            texture = Path(__file__).resolve().parents[1] / 'reconstruction/textures/paper.png'
            if not texture.is_file():
                raise FileNotFoundError(texture)
            image = bpy.data.images.load(str(texture), check_existing=True)
            node = material.node_tree.nodes.new('ShaderNodeTexImage')
            node.image = image
            material.node_tree.links.new(node.outputs['Color'], base)


def wood_segments(obj, offset):
    """Derive conservative cylinder segments from the actual woody mesh rings."""
    sides = int(obj.get('tube_sides', 0))
    if sides < 3 or len(obj.data.vertices) % sides:
        return []
    rings = []
    for first in range(0, len(obj.data.vertices), sides):
        vertices = [obj.matrix_world @ v.co
                    for v in obj.data.vertices[first:first + sides]]
        center = sum(vertices, Vector()) / sides
        rings.append((center, max((v - center).length for v in vertices)))
    segments = []
    for i, ((a, ra), (b, rb)) in enumerate(zip(rings, rings[1:])):
        direction = b - a
        if direction.length < 1e-5:
            continue
        rotation = Vector((0, 0, 1)).rotation_difference(direction.normalized())
        center = (a + b) / 2 + offset
        segments.append({
            'id': obj.name.replace('/', '_') + f'_{i}', 'shape': 'cylinder',
            'position': list(center), 'orientation': list(rotation.to_euler()),
            'radius': max(ra, rb), 'length': direction.length,
            'endpoints': [list(a + offset), list(b + offset)],
            'source_object': obj.name})
    return segments


def main():
    """Export two actual source trees without moving or rescaling their fruit."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--trees', type=int, nargs='+', default=[0, 1])
    parser.add_argument('--leaf-ratio', type=float, default=0.20)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    if not 0 < args.leaf_ratio <= 1 or not args.trees or len(set(args.trees)) != len(args.trees):
        raise ValueError('Invalid trees or leaf ratio')
    out = args.out.resolve()
    project = Path(__file__).resolve().parents[2]
    if not out.is_relative_to(project):
        raise ValueError('Assets must stay inside peach_sim')
    out.mkdir(parents=True, exist_ok=True)
    blend = Path(bpy.data.filepath)
    manifest_path = blend.with_name('scene_manifest.json')
    source = json.loads(manifest_path.read_text())
    prefixes = tuple(f'Tree{i:02d}/' for i in args.trees)
    selected = [o for o in bpy.context.scene.objects
                if o.type == 'MESH' and o.name.startswith(prefixes)]
    if not selected:
        raise ValueError('No selected tree meshes')
    # Remove unselected orchard/backdrop objects before evaluating foliage.
    # Keep prototype collections: selected GN leaf clouds reference them.
    for obj in list(bpy.context.scene.objects):
        if obj not in selected and obj.instance_collection:
            bpy.data.objects.remove(obj, do_unlink=True)
        elif obj.type == 'MESH' and obj.name.startswith('Tree') and obj not in selected:
            bpy.data.objects.remove(obj, do_unlink=True)
    groups = {m.node_group for o in selected for m in o.modifiers
              if m.type == 'NODES' and m.node_group}
    for group in groups:
        for output in [n for n in group.nodes if n.type == 'GROUP_OUTPUT']:
            for link in list(output.inputs[0].links):
                socket = link.from_socket
                group.links.remove(link)
                realize = group.nodes.new('GeometryNodeRealizeInstances')
                group.links.new(socket, realize.inputs[0])
                group.links.new(realize.outputs[0], output.inputs[0])
    simplify_materials()
    offset = Vector((5.25, -2.3, 0.0))
    targets, trees, obstacles = [], [], []
    for number in args.trees:
        prefix = f'Tree{number:02d}/'
        members = [o for o in selected if o.name.startswith(prefix)]
        target_records = [t for t in source['targets'] if t['name'].startswith(prefix)]
        bag_prefixes = tuple(t['name'] for t in target_records)
        vegetation = [o for o in members
                      if not any(o.name == p or o.name.startswith(p + '/')
                                 for p in bag_prefixes)]
        # Evaluate GN once per tree and decimate only foliage, retaining all
        # woody rings and all bag geometry at their original resolution.
        for obj in vegetation:
            if not any(m.type == 'NODES' for m in obj.modifiers):
                continue
            bpy.context.view_layer.objects.active = obj
            for modifier in list(obj.modifiers):
                bpy.ops.object.modifier_apply(modifier=modifier.name)
            modifier = obj.modifiers.new('leaf_export_decimate', 'DECIMATE')
            modifier.ratio = args.leaf_ratio
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        tree_file = out / f'tree_{number:02d}.glb'
        export_meshes(vegetation, tree_file, -offset)
        tree_obstacles = [s for o in vegetation for s in wood_segments(o, offset)]
        obstacles.extend(tree_obstacles)
        trees.append({'id': f'tree_{number:02d}', 'mesh': tree_file.name,
                      'sha256': digest(tree_file), 'collision_segments': len(tree_obstacles),
                      'source': prefix, 'vertices': sum(len(o.data.vertices) for o in vegetation)})
        for record in target_records:
            prefix_bag = record['name']
            bag = [o for o in members
                   if o.name == prefix_bag or o.name.startswith(prefix_bag + '/')]
            bottom = Vector(record['bottom_world'])
            vertices = [o.matrix_world @ v.co - bottom for o in bag for v in o.data.vertices]
            if not vertices:
                raise ValueError('Empty bag ' + prefix_bag)
            low = Vector(tuple(min(v[i] for v in vertices) for i in range(3)))
            high = Vector(tuple(max(v[i] for v in vertices) for i in range(3)))
            key = f'peach_{number:02d}_{record["id"]:03d}'
            file = out / (key + '.glb')
            export_meshes(bag, file, bottom)
            center = (low + high) / 2
            item = {
                'id': key, 'tree': f'tree_{number:02d}', 'source_target': record['name'],
                'bag_bottom': list(bottom + offset), 'entry': list(bottom + offset),
                'bag_neck': list(Vector(record['neck']) + offset),
                'axis': record['axis_world'], 'fruit_center': list(
                    Vector(record['center_world']) + offset),
                'body_diameter': record['width_m'], 'body_thickness': high.y - low.y,
                'bottom_to_neck': (Vector(record['neck']) - bottom).length,
                'fruit_diameter': 2 * record['fruit_radius_m'],
                'mass_kg': record['fruit_mass_kg'], 'visual_mesh': file.name,
                'visual_sha256': digest(file), 'visual_pose': [0, 0, 0, 0, 0, 0],
                'visual_scale': [1, 1, 1], 'collision_center_local': list(center),
                'collision_center': list(bottom + offset + center),
                'collision_size': list(high - low),
                'fruit_ellipsoid_size': [2 * record['fruit_radius_m']] * 3,
                'paper_form': record['paper_form'], 'occlusion': record['occlusion'],
                'geometry_source': 'PeachDataSet constrained v11 reconstruction',
                'placement_status': 'inferred source placement, translation only'}
            targets.append(item)
    evidence = project / 'sim_worlds/reconstruction/evidence'
    report = {
        'schema': 'sim_worlds/data_orchard/v1', 'world_name': 'peach_orchard',
        'frame': 'map', 'trees': trees, 'targets': targets, 'obstacles': obstacles,
        'source_blend': str(blend.relative_to(project)), 'source_blend_sha256': digest(blend),
        'source_manifest_sha256': digest(manifest_path),
        'source_revision': source['modeling_revision'],
        'exporter_sha256': digest(__file__), 'translation_m': list(offset),
        'leaf_decimation_ratio': args.leaf_ratio,
        'dataset_url': 'https://github.com/tsing-luo/Multi-class-peach-RGB-D-dataset',
        'prior_sha256': digest(evidence / 'priors.json'),
        'data_audit_sha256': digest(evidence / 'real_feature_audit.json'),
        'limits': [
            'Tree topology, hidden surfaces and orchard layout are inferred.',
            'Size priors use approximate FOV intrinsics, not calibrated measurement.',
            'Rigid bags and simplified cylinders do not simulate flexible branches or cutting.',
            'All source bags retained, including oversized, occluded and unreachable fruit.']}
    (out / 'assets.json').write_text(json.dumps(report, indent=2))
    print('DATA_ORCHARD', json.dumps({'trees': len(trees), 'targets': len(targets),
                                     'collision_segments': len(obstacles)}), flush=True)


if __name__ == '__main__':
    main()
