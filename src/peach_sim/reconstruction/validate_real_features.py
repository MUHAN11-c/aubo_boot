"""
Audit saved orchard meshes, without rendering or starting ROS.

blender -b --python-exit-code 1 -P validate_real_features.py -- scene.blend
Writes real_feature_validation.json beside the input blend. Geometric
consistency tolerances are numerical checks, not measured botanical priors.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re
import sys

import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

CONNECT_TOL = 1e-4
RADIUS_TOL = 1e-5


def summary(values):
    values = sorted(float(v) for v in values)
    if not values:
        return {'n': 0}

    def q(f):
        p = f * (len(values) - 1)
        lo = int(p)
        hi = min(lo + 1, len(values) - 1)
        return values[lo] * (1 - p + lo) + values[hi] * (p - lo)
    return {'n': len(values), 'min': values[0], 'p10': q(.1),
            'p50': q(.5), 'p90': q(.9), 'max': values[-1]}


def tube_axis(obj):
    """Recover ring centers/radii from the actual ordered tube vertices."""
    sides = int(obj.get('tube_sides', 0))
    if not sides:
        caps = [len(p.vertices) for p in obj.data.polygons if len(p.vertices) > 4]
        if caps:
            sides = caps[-1]
    if sides < 5 or len(obj.data.vertices) % sides:
        raise ValueError(f'{obj.name}: missing or invalid tube rings')
    vertices = [obj.matrix_world @ v.co for v in obj.data.vertices]
    centers, radii = [], []
    for i in range(0, len(vertices), sides):
        ring = vertices[i:i + sides]
        center = sum(ring, Vector()) / sides
        centers.append(center)
        radii.append(sum((v - center).length for v in ring) / sides)
    if len(centers) < 2:
        raise ValueError(f'{obj.name}: fewer than two tube rings')
    return {'name': obj.name, 'centers': centers, 'radii': radii, 'object': obj}


def point_segment(point, a, b):
    delta = b - a
    t = (max(0., min(1., (point - a).dot(delta) / delta.length_squared))
         if delta.length_squared else 0.)
    return (point - a.lerp(b, t)).length, t


def nearest(point, axes, exclude=None):
    candidates = []
    for axis in axes:
        name = axis['name']
        if exclude and (name == exclude or name.startswith(exclude + '/')):
            continue
        for i, (a, b) in enumerate(zip(axis['centers'], axis['centers'][1:])):
            distance, t = point_segment(point, a, b)
            radius = axis['radii'][i] * (1 - t) + axis['radii'][i + 1] * t
            candidates.append((distance, radius, name, i, t))
    if not candidates:
        return None
    connected = [c for c in candidates if c[0] <= CONNECT_TOL]
    # Shared roots can meet several children. Prefer the thickest physical
    # support, rather than whichever object happens to come first in bpy.
    return max(connected, key=lambda c: c[1]) if connected else min(candidates)


def mesh_bvh(obj):
    return BVHTree.FromPolygons(
        [obj.matrix_world @ v.co for v in obj.data.vertices],
        [tuple(p.vertices) for p in obj.data.polygons])


def audit():
    errors = []
    objects = list(bpy.context.scene.objects)
    groups = {}
    for obj in objects:
        match = re.match(r'^(Tree\d+|Probe|Audit)/', obj.name)
        if match:
            groups.setdefault(match[1], []).append(obj)
    if not groups:
        errors.append('No orchard tree meshes found')
    report = {'input_blend': bpy.data.filepath, 'blender_version': bpy.app.version_string,
              'measurement_basis': ('saved mesh ring centers, world-space vertices '
                                    'and evaluated leaf instances'),
              'numerical_tolerances_m': {'connection': CONNECT_TOL, 'radius': RADIUS_TOL},
              'trees': {}, 'bags': [], 'errors': errors}
    all_axes = {}
    depsgraph = bpy.context.evaluated_depsgraph_get()
    for tree_name, members in sorted(groups.items()):
        axes = []
        for obj in members:
            if obj.type != 'MESH' or any(s in obj.name for s in ('/bag', '/leaves')):
                continue
            try:
                axes.append(tube_axis(obj))
            except ValueError as exc:
                errors.append(str(exc))
        all_axes[tree_name] = axes
        branches = []
        trunks = [a for a in axes if a['name'].endswith('/trunk')]
        if len(trunks) != 1:
            errors.append(f'{tree_name}: expected exactly one trunk, found {len(trunks)}')
        for axis in axes:
            if axis in trunks:
                continue
            parent = nearest(axis['centers'][0], axes, axis['name'])
            item = {'name': axis['name'], 'root_radius_m': axis['radii'][0]}
            if parent is None:
                errors.append(f"{axis['name']}: no possible parent")
            else:
                distance, radius, name, segment, fraction = parent
                ratio = axis['radii'][0] / radius
                item.update(parent=name, parent_segment=segment, parent_segment_fraction=fraction,
                            root_centerline_error_m=distance, parent_radius_m=radius,
                            root_parent_radius_ratio=ratio)
                if distance > CONNECT_TOL:
                    errors.append(f"{axis['name']}: root floats {distance:.6g} m from parent")
                if axis['radii'][0] > radius + RADIUS_TOL:
                    errors.append(f"{axis['name']}: root radius exceeds parent ({ratio:.6g})")
            for prop in ('root_radius_m', 'parent_radius_m'):
                if prop in axis['object']:
                    item['declared_' + prop] = float(axis['object'][prop])
            branches.append(item)
        parents = {item['name']: item.get('parent') for item in branches}
        trunk_names = {axis['name'] for axis in trunks}
        for child in parents:
            cursor, visited = child, set()
            while cursor in parents and cursor not in visited:
                visited.add(cursor)
                cursor = parents[cursor]
            if cursor not in trunk_names:
                errors.append(f'{child}: parent graph does not reach trunk')
        scaffold_angles = []
        for axis in axes:
            if '/scaffold' in axis['name']:
                direction = axis['centers'][-1] - axis['centers'][0]
                scaffold_angles.append(math.degrees(math.atan2(direction.z, direction.xy.length)))
        leaf_lengths, leaf_widths, roots, directions, heights = [], [], [], [], []
        prototype_counts, materials, shape_counts = Counter(), Counter(), Counter()
        for obj in members:
            if obj.type == 'MESH' and '/leaves' not in obj.name:
                heights.extend((obj.matrix_world @ v.co).z for v in obj.data.vertices)
        for instance in depsgraph.object_instances:
            if not (instance.is_instance and instance.parent
                    and instance.parent.original.name.startswith(tree_name + '/leaves')):
                continue
            proto = instance.object.original
            matrix = instance.matrix_world
            verts = [matrix @ v.co for v in proto.data.vertices]
            heights.extend(v.z for v in verts)
            # Local +X is the blade axis; lengths measured by the transformed
            # prototype coordinates, not leaf_scale bookkeeping.
            local_x = [v.co.x for v in proto.data.vertices]
            local_y = [v.co.y for v in proto.data.vertices]
            leaf_lengths.append((max(local_x) - min(local_x)) *
                                (matrix.to_3x3() @ Vector((1, 0, 0))).length)
            leaf_widths.append((max(local_y) - min(local_y)) *
                               (matrix.to_3x3() @ Vector((0, 1, 0))).length)
            direction = (matrix.to_3x3() @ Vector((1, 0, 0))).normalized()
            directions.append(list(direction))
            parent = nearest(matrix.translation, axes)
            roots.append(parent[0] if parent else 1e30)
            prototype_counts[proto.name] += 1
            shape_counts[proto.name.rsplit('_', 1)[-1]] += 1
            for mat in proto.data.materials:
                materials[mat.name] += 1
        if not leaf_lengths:
            errors.append(f'{tree_name}: no evaluated leaf instances')
        if roots and max(roots) > CONNECT_TOL:
            errors.append(f'{tree_name}: leaf roots float off branch axes')
        base_z = trunks[0]['centers'][0].z if trunks else min(heights, default=0)
        trunk_height = trunks[0]['centers'][-1].z - base_z if trunks else None
        tree_height = max(heights, default=base_z) - base_z
        if trunk_height is not None and not .4 - CONNECT_TOL <= trunk_height <= .5 + CONNECT_TOL:
            errors.append(f'{tree_name}: trunk outside recorded 0.4–0.5 m prior')
        if tree_height > 2.5 + CONNECT_TOL:
            errors.append(f'{tree_name}: mesh crown exceeds recorded 2.5 m prior')
        octants = Counter(''.join('+' if v >= 0 else '-' for v in d) for d in directions)
        report['trees'][tree_name] = {
            'branches': branches,
            'trunk_height_m': trunk_height, 'mesh_tree_height_m': tree_height,
            'scaffold_count': sum('/scaffold' in a['name'] for a in axes),
            'scaffold_elevation_deg': summary(scaffold_angles),
            'leaves': {'count': len(leaf_lengths), 'length_m': summary(leaf_lengths),
                       'width_m': summary(leaf_widths), 'root_axis_error_m': summary(roots),
                       'prototypes': dict(prototype_counts), 'materials': dict(materials),
                       'shapes': dict(shape_counts),
                       'axis_z': summary(d[2] for d in directions),
                       'axis_octants': dict(octants)}}
    for bag in objects:
        if bag.type != 'MESH' or not bag.name.endswith('/bag'):
            continue
        fruit = bpy.data.objects.get(bag.name + '/enclosed peach')
        peduncle = bpy.data.objects.get(bag.name + '/fruit peduncle')
        item = {'name': bag.name, 'paper_form': bag.get('paper_form', 'unrecorded'),
                'fruit_count': int(fruit is not None),
                'interior_status': 'inferred; opaque paper does not measure interior fruit'}
        coordinates = [v.co for v in bag.data.vertices]
        width = max(p.x for p in coordinates) - min(p.x for p in coordinates)
        depth = max(p.y for p in coordinates) - min(p.y for p in coordinates)
        height = max(p.z for p in coordinates) - min(p.z for p in coordinates)
        rings = {}
        for coordinate in coordinates:
            rings.setdefault(round(coordinate.z, 7), []).append(coordinate)
        equatorial_ring = max(rings.values(), key=lambda ring:
                              max(p.x for p in ring) - min(p.x for p in ring))
        area = abs(sum(a.x * b.y - b.x * a.y for a, b in
                       zip(equatorial_ring, equatorial_ring[1:] + equatorial_ring[:1]))) / 2
        perimeter = sum((a.xy - b.xy).length for a, b in
                        zip(equatorial_ring, equatorial_ring[1:] + equatorial_ring[:1]))
        item.update(mesh_local_width_m=width, mesh_local_depth_m=depth,
                    mesh_local_height_m=height, depth_width_ratio=depth / width,
                    widest_ring_circularity=4 * math.pi * area / perimeter ** 2)
        if fruit is None or peduncle is None:
            errors.append(f'{bag.name}: missing fruit or internal peduncle')
        else:
            axis = tube_axis(peduncle)
            endpoints = (axis['centers'][0], axis['centers'][-1])
            fruit_distance = mesh_bvh(fruit).find_nearest(endpoints[0])[3]
            neck_distance = (endpoints[-1] - Vector(bag['attachment_world'])).length
            item.update(peduncle_endpoints_world=[list(p) for p in endpoints],
                        peduncle_fruit_surface_error_m=fruit_distance,
                        peduncle_neck_error_m=neck_distance)
            if (fruit_distance is None or fruit_distance > CONNECT_TOL
                    or neck_distance > CONNECT_TOL):
                errors.append(f'{bag.name}: peduncle does not connect fruit surface and neck')
            equator = [v.co.xy.length for v in fruit.data.vertices if abs(v.co.z) < CONNECT_TOL]
            shrink = 1 - min(equator) / max(equator) if equator else 0
            item['equatorial_suture_relative_depth'] = shrink
            if shrink < .001:
                errors.append(f'{bag.name}: missing measurable suture groove')
        report['bags'].append(item)
    report['bag_form_counts'] = dict(Counter(b['paper_form'] for b in report['bags']))
    report['bag_mesh_statistics'] = {
        key: summary(b[key] for b in report['bags'])
        for key in ('mesh_local_width_m', 'mesh_local_depth_m', 'mesh_local_height_m',
                    'depth_width_ratio', 'widest_ring_circularity')}
    report['reference_bags'] = [
        {'name': obj.name, 'paper_form': obj.get('paper_form', 'unrecorded'),
         'source': obj.get('source', 'unrecorded'),
         'interior_status': obj.get('interior_status', 'unobserved')}
        for obj in objects if obj.type == 'MESH' and obj.name.startswith('Reference/bag_')
        and '/' not in obj.name[len('Reference/'):]]
    report['real_evidence_boundary'] = {
        'recorded_sources': ['PeachDataSet RGB-D visible bag profiles and foliage support',
                             'Peach_nobag approximate diameter/aspect quantiles',
                             'orchard_standard trunk height, tree height, scaffold elevation',
                             'field_20260909 bag axis tilt'],
        'unmeasured_parameters': ['hidden branch layout and diameters',
                                  'leaf phyllotaxy and petiole dimensions',
                                  'serration and curvature distribution',
                                  'branch collars, buds and pruning scars',
                                  'paper backside, slack, thickness, folds and tie construction',
                                  'enclosed fruit surface, peduncle and mass density'],
        'claim_limit': ('Geometry consistency is verified here; botanical realism '
                        'of unmeasured parameters is not established.')}
    report['passed'] = not errors
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('blend', nargs='?', default=bpy.data.filepath)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else [])
    if not args.blend:
        raise ValueError('Pass an existing .blend file')
    path = Path(args.blend).resolve()
    if Path(bpy.data.filepath or '.').resolve() != path:
        bpy.ops.wm.open_mainfile(filepath=str(path))
    report = audit()
    output = path.parent / 'real_feature_validation.json'
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
    print(f"REAL_FEATURE_VALIDATION {'PASS' if report['passed'] else 'FAIL'} {output}", flush=True)
    if not report['passed']:
        for error in report['errors']:
            print(error, flush=True)
        raise RuntimeError(f"{len(report['errors'])} real-feature geometry checks failed")


if __name__ == '__main__':
    main()
