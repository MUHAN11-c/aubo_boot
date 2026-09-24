"""Validate saved Blender geometry: closed bags, fruit containment, finite meshes."""
from pathlib import Path
import json
import math
import bpy
import bmesh
from mathutils.bvhtree import BVHTree
from mathutils import Vector
HERE = Path(__file__).resolve().parent
report = {'bags': [], 'errors': []}
for obj in bpy.data.objects:
    if obj.type != 'MESH' or 'geometry' not in obj:
        continue
    if any(not math.isfinite(c) for v in obj.data.vertices for c in v.co):
        report['errors'].append(obj.name+': nonfinite vertex')
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bad = sum(not e.is_manifold for e in bm.edges)
    bm.free()
    item = {
        'name': obj.name,
        'vertices': len(
            obj.data.vertices),
        'nonmanifold_edges': bad}
    if bad:
        report['errors'].append(obj.name + ': open envelope')
    tree = BVHTree.FromPolygons([v.co for v in obj.data.vertices], [
                                p.vertices[:] for p in obj.data.polygons])
    fruits = [
        p for p in bpy.data.objects if p.name in [
            obj.name +
            '/hidden peach',
            obj.name +
            '/enclosed peach']]
    for fruit in fruits:
        c = obj.matrix_world.inverted() @ fruit.location
        point, normal, index, distance = tree.find_nearest(c)
        radius = max(fruit.dimensions) / 2
        inside = (c - point).dot(normal) < 0
        item['fruit_clearance_m'] = distance - radius
        item['fruit_center_inside'] = inside
        if not inside or distance < radius - .0001:
            report['errors'].append(obj.name + ': fruit intersects paper')
    report['bags'].append(item)
report['mesh_objects'] = sum(o.type == 'MESH' for o in bpy.data.objects)
report['vertices'] = sum(len(o.data.vertices)
                         for o in bpy.data.objects if o.type == 'MESH')
(HERE / 'output/geometry_validation.json').write_text(json.dumps(report, indent=2))
print(json.dumps({'bags': len(report['bags']),
                  'errors': report['errors'],
                  'vertices': report['vertices']}),
      flush=True)
if report['errors']:
    raise RuntimeError('Geometry gate failed')
