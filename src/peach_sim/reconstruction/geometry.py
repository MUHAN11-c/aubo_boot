"""Blender mesh construction; connected branches and instanced leaf blades."""
import array
import math
import random
import bpy
from mathutils import Quaternion, Vector

from distributions import (FRUIT_SEAT_M, fruit_center_z,  # noqa: E402,F401,I100
                           wrap_half_at, wrap_radius)

# Shared leaf meshes. Instances scale X/Z with length and Y with width.
PROTO_LENGTH = .10
PROTO_WIDTH = .022
_SHAPES = ((-.12, .00, .10), (.06, .20, .16), (-.05, -.18, .08), (.10, -.22, .18))
_LEAF_GROUP = None


def mesh(name, verts, faces, mat, uvs=None, smooth=True):
    data = bpy.data.meshes.new(name)
    data.from_pydata(verts, [], faces)
    data.update()
    if data.validate():
        raise ValueError('Invalid topology: ' + name)
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    if mat:
        data.materials.append(mat)
    if smooth:
        for p in data.polygons:
            p.use_smooth = True
    if uvs:
        layer = data.uv_layers.new()
        for p in data.polygons:
            for li in p.loop_indices:
                layer.data[li].uv = uvs[data.loops[li].vertex_index]
    return obj


def tube(name, points, radii, mat, sides=9):
    verts = []
    faces = []
    points = [Vector(p) for p in points]
    for i, (p, r) in enumerate(zip(points, radii)):
        axis = (points[min(i + 1, len(points) - 1)] -
                points[max(i - 1, 0)]).normalized()
        q = axis.to_track_quat('Z', 'Y')
        for j in range(sides):
            a = j * math.tau / sides
            v = p + q @ Vector((r * math.cos(a), r * math.sin(a), 0))
            verts.append(v)
    for i in range(len(points) - 1):
        for j in range(sides):
            a = i * sides + j
            b = i * sides + (j + 1) % sides
            faces.append((a, b, b + sides, a + sides))
    faces.extend([tuple(reversed(range(sides))), tuple(
        range(len(verts) - sides, len(verts)))])
    obj = mesh(name, verts, faces, mat)
    obj['tube_sides'] = sides
    return obj


def blade_vertices(length, width, curl, twist, droop):
    """One blade in local space: petiole at the origin, tip along +X."""
    verts = []
    faces = []
    uv = []
    segments = 40
    for i in range(segments + 1):
        t = i / segments
        # Narrow attached petiole, then a lanceolate, finely serrated blade.
        blade_t = max(0., (t - .08) / .92)
        w = (.00055 if t < .08 else width / 2 * max(
            .005, math.sin(math.pi * blade_t) ** .92)
            * (1 + .025 * (-1 if i % 2 else 1)))
        for j in range(5):
            s = (j - 2) / 2
            verts.append((length * t, w * s, length * curl * t * t + s * s
                          * width * .035 * math.sin(math.pi * t)
                          + twist * s * w * math.sin(math.pi * blade_t)
                          - droop * t * t))
            uv.append((t, (s + 1) / 2))
    for i in range(segments):
        for j in range(4):
            k = i * 5 + j
            faces.append((k, k + 1, k + 6, k + 5))
    return verts, faces, uv


def _prototype_collection(mats):
    name = 'Leaf prototypes'
    if name in bpy.data.collections:
        return bpy.data.collections[name]
    col = bpy.data.collections.new(name)
    for material in range(5):
        for shape, (curl, twist, droop_k) in enumerate(_SHAPES):
            verts, faces, uv = blade_vertices(
                PROTO_LENGTH, PROTO_WIDTH, curl, twist, PROTO_LENGTH * droop_k)
            # Paired basal glands are a botanical feature; size is inferred.
            for side in (-1, 1):
                center = Vector((PROTO_LENGTH * .085, side * .0008, .00025))
                offset = len(verts)
                directions = ((1, 0, 0), (-1, 0, 0), (0, 1, 0),
                              (0, -1, 0), (0, 0, 1), (0, 0, -1))
                verts.extend(tuple(center + Vector(d) * .0006) for d in directions)
                uv.extend([(.085, .5)] * 6)
                faces.extend(tuple(offset + j for j in f) for f in
                             ((0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4),
                              (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)))
            obj = mesh(f'leaf proto {material}_{shape}', verts, faces,
                       mats[f'leaf{material}'], uv)
            for owner in list(obj.users_collection):
                owner.objects.unlink(obj)
            col.objects.link(obj)
    return col


def _leaf_group(collection):
    """Point cloud in, one shared-mesh instance per point."""
    global _LEAF_GROUP
    if _LEAF_GROUP is not None:
        return _LEAF_GROUP
    ng = bpy.data.node_groups.new('Peach leaf instances', 'GeometryNodeTree')
    ng.interface.new_socket(
        name='Geometry', in_out='INPUT', socket_type='NodeSocketGeometry')
    ng.interface.new_socket(
        name='Geometry', in_out='OUTPUT', socket_type='NodeSocketGeometry')
    nodes, links = ng.nodes, ng.links
    inp = nodes.new('NodeGroupInput')
    out = nodes.new('NodeGroupOutput')
    to_points = nodes.new('GeometryNodeMeshToPoints')
    coll = nodes.new('GeometryNodeCollectionInfo')
    coll.inputs['Separate Children'].default_value = True
    coll.inputs['Reset Children'].default_value = True
    coll.inputs['Collection'].default_value = collection
    inst = nodes.new('GeometryNodeInstanceOnPoints')
    inst.inputs['Pick Instance'].default_value = True
    quat = nodes.new('GeometryNodeInputNamedAttribute')
    quat.data_type = 'QUATERNION'
    quat.inputs['Name'].default_value = 'leaf_quat'
    index = nodes.new('GeometryNodeInputNamedAttribute')
    index.data_type = 'INT'
    index.inputs['Name'].default_value = 'leaf_index'
    scale = nodes.new('GeometryNodeInputNamedAttribute')
    scale.data_type = 'FLOAT_VECTOR'
    scale.inputs['Name'].default_value = 'leaf_scale'
    links.new(inp.outputs[0], to_points.inputs['Mesh'])
    links.new(to_points.outputs['Points'], inst.inputs['Points'])
    links.new(coll.outputs['Instances'], inst.inputs['Instance'])
    links.new(quat.outputs['Attribute'], inst.inputs['Rotation'])
    links.new(index.outputs['Attribute'], inst.inputs['Instance Index'])
    links.new(scale.outputs['Attribute'], inst.inputs['Scale'])
    links.new(inst.outputs['Instances'], out.inputs[0])
    _LEAF_GROUP = ng
    return ng


class Leaves:
    """Per-tree point cloud instancing 20 shared blades (5 materials × 4 shapes)."""

    def __init__(self, mats, seed=31):
        self.mats = mats
        self.rng = random.Random(seed)
        self.records = []

    def add(self, root, tip, width=None, normal=None):
        root = Vector(root)
        tip = Vector(tip)
        axis = tip - root
        length = axis.length
        if length < .005:
            return
        width = width or length * self.rng.uniform(.18, .27)
        quat = axis.to_track_quat('X', 'Z') @ Quaternion(
            (1, 0, 0), self.rng.uniform(-.85, .85))
        if normal is not None:
            direction = axis.normalized()
            projected = Vector(normal) - direction * Vector(normal).dot(direction)
            if projected.length < 1e-6:
                raise ValueError('Leaf normal must not be parallel to its axis')
            quat = (quat @ Vector((0, 0, -1))).rotation_difference(
                projected.normalized()) @ quat
        index = self.rng.randrange(5) * 4 + self.rng.randrange(4)
        scale = (length / PROTO_LENGTH, width / PROTO_WIDTH, length / PROTO_LENGTH)
        self.records.append((index, root, quat, scale))

    def finish(self, prefix):
        if not self.records:
            return []
        me = bpy.data.meshes.new(f'{prefix} leaf points')
        me.from_pydata([tuple(rec[1]) for rec in self.records], [], [])
        me.attributes.new('leaf_quat', 'QUATERNION', 'POINT')
        me.attributes.new('leaf_index', 'INT', 'POINT')
        me.attributes.new('leaf_scale', 'FLOAT_VECTOR', 'POINT')
        qflat, iflat, sflat = [], [], []
        for index, _loc, quat, scale in self.records:
            qflat.extend((quat.w, quat.x, quat.y, quat.z))
            iflat.append(index)
            sflat.extend(scale)
        # Adding attributes can invalidate earlier RNA attribute references.
        # Resolve each attribute only after all layers have been created.
        me.attributes['leaf_quat'].data.foreach_set('value', array.array('f', qflat))
        me.attributes['leaf_index'].data.foreach_set('value', array.array('i', iflat))
        me.attributes['leaf_scale'].data.foreach_set('vector', array.array('f', sflat))
        me.update()
        obj = bpy.data.objects.new(f'{prefix}/leaves', me)
        bpy.context.collection.objects.link(obj)
        mod = obj.modifiers.new('leaves', 'NODES')
        mod.node_group = _leaf_group(_prototype_collection(self.mats))
        return [obj]


def paper_bag(name, rings, mat, seed=1):
    """Closed paper envelope. Rings are (z, center_x, half_width, front_y, thickness)."""
    rng = random.Random(seed)
    phase = rng.uniform(0, 6.28)
    verts = []
    faces = []
    uv = []
    n = 64
    filled = len(rings[0]) > 6
    folds = [(rng.uniform(-1.5, 1.5), rng.uniform(-1, 1),
              rng.uniform(.06, .16), rng.uniform(-.0008, .0008)) for _ in range(18)]
    for i, ring in enumerate(rings):
        z, cx, w, front, thick = ring[:5]
        # 5-tuples stay flat panels (the outline probe). A 6th value rounds
        # the cheek over a fruit; the neck passes 2.
        power = ring[5] if len(ring) > 5 else 20.
        t = i / (len(rings) - 1)
        for j in range(n):
            a = math.tau * j / n
            co = math.cos(a)
            si = math.sin(a)
            # High-order superellipse: broad planar paper faces with a small
            # corner radius, rather than a continuously inflated cloth pouch.
            scale = (abs(co)**power + abs(si)**power)**(-1 / power)
            x = co * scale * w
            y = si * scale * thick * .5
            if power < 4 and not filled:
                x *= 1 + (0. if seed % 3 == 0 else .10 if seed % 3 == 1 else .06)
            panel = x / max(w, .001)
            envelope = math.sin(math.pi * t)
            face = min(1., abs(si) / max(abs(co), 1e-8))
            if filled:
                # Sparse physical creases on broad paper; no inflated ribs.
                offset = sum(amplitude * 2.4 * max(0., 1 - abs(
                    panel - center - slope * (t - .5)) / spread)
                    for slope, center, spread, amplitude in folds[:5])
                fold_slope = 1.35 if seed % 2 else -1.35
                line_distance = abs(panel - .18 * math.sin(phase) - fold_slope * (t - .45))
                main_fold = max(0., 1 - line_distance / .22)
                offset = max(-.0014, min(.0045, offset + .0040 * main_fold))
                offset *= face * envelope
                gathers = .0016 * max(0., (t - .72) / .28) * math.sin(
                    7 * a + phase) * math.sin(math.pi * t)
                y += math.copysign(1, si) * offset + gathers * si
            elif power < 4:
                # Outward-only folds preserve the fruit clearance envelope.
                fold = .0012 * max(0., math.cos(7 * a + phase)) ** 6
                fold *= math.sin(math.pi * t) ** 2
                x += fold * co
                y += fold * si
                # Gathered neck above the fruit.
                gathers = .0016 * max(0., t - .78)**2 * math.sin(
                    8 * a + phase)
                crease = .0012 * max(0., t - .86) * math.sin(6 * a + phase)
                y += (gathers + crease) * si
            else:
                offset = 0.
                for slope, center, spread, amplitude in folds:
                    distance = abs(panel - (center + slope * (t - .5)))
                    offset += amplitude * max(0., 1 - distance / spread)
                offset = max(-min(.0012, thick * .06),
                             min(min(.0012, thick * .06), offset))
                offset *= face * envelope
                seam = .0007 * math.exp(-((t - .035) / .018)**2)
                seam *= face * envelope
                side = .0005 * max(0., 1 - abs(abs(panel) - .94) / .06) * envelope
                gathers = .0012 * t**8 * math.sin(9 * a + phase) * envelope
                rib = min(.0016, thick * .05) * math.sin(math.pi * t) * max(
                    0., math.sin(5 * a + phase))
                neck = max(0., (t - .78) / .22)
                crease = min(.0025, thick * .15) * neck * math.sin(8 * a + phase)
                y += math.copysign(1, si) * (offset + seam + side + rib) + (
                    gathers + crease) * si
            zz = z
            if filled:
                zz += .0030 * co * math.sin(phase) * math.exp(-(t / .12) ** 2)
                # Curl the free lower paper, fading before the fruit contact.
                y += .004 * math.sin(phase) * max(0., 1 - t / .25) ** 2
                zz += .0007 * math.sin(5 * a + phase) * max(0., (t - .94) / .06)
            verts.append((cx + x, front + thick * .5 + y, zz))
            uv.append((j / n, t))
    for i in range(len(rings) - 1):
        for j in range(n):
            a = i * n + j
            b = i * n + (j + 1) % n
            if (i + j) % 2:
                faces.extend(((a, b, a + n), (b, b + n, a + n)))
            else:
                faces.extend(((a, b, b + n), (a, b + n, a + n)))
    faces.extend([tuple(reversed(range(n))), tuple(
        range(len(verts) - n, len(verts)))])
    obj = mesh(name, verts, faces, mat, uv)
    obj['geometry'] = 'closed paper shell; observed folds with inferred construction'
    obj['paper_form'] = (rings[0][6] if len(rings[0]) > 6
                         else 'reference_visible_panel')
    obj['hidden_surface_status'] = 'thickness, creases and seam construction inferred'
    obj['single_crease_amplitude_bound_m'] = .0040 if filled else .0008
    obj['combined_crease_amplitude_bound_m'] = .0045 if filled else .0012
    return obj


FRUIT_SEAT_CLEARANCE_M = FRUIT_SEAT_M


def _profile_width(t, controls):
    for (a, wa), (b, wb) in zip(controls, controls[1:]):
        if a <= t <= b:
            return wa + (wb - wa) * (t - a) / (b - a)
    return controls[-1][1]


def _empty_bag_rings(width, height, thickness, seed):
    """Flat-panel envelope used when no fruit drives the silhouette."""
    rng = random.Random(seed)
    offset = rng.uniform(-.08, .08) * width
    controls = [(0, .90), (.06, 1.0), (.30, 1.0), (.55, .92),
                (.74, .62), (.88, .22), (.94, .09), (1, .16)]
    rings = []
    for i in range(33):
        t = i / 32
        half_w = width * _profile_width(t, controls) / 2
        if t < .94:
            thick = thickness * (.2 + .8 * math.sin(
                math.pi * min(t / .94, 1)) ** .55)
        else:
            thick = thickness * .2
        rings.append((t * height, offset * (1 - t), half_w,
                      -thick / 2, thick, 20.))
    return rings


def bag_rings(width, height, thickness=.055, seed=1, fruit_diameter=None):
    """Build a loose folded paper envelope with a fruit-clearance constraint."""
    if not fruit_diameter:
        return _empty_bag_rings(width, height, thickness, seed)
    fruit_r = fruit_diameter / 2
    clearance_r = fruit_r + .006
    forms = ('folded_gusset', 'broad_panel', 'creased_panel')
    # A two-panel envelope: thin sealed hem and sides, local fruit bulge.
    # The flat paper silhouette stays broad below the fruit; the front/back
    # depth follows its support only locally instead of forming a cuboid.
    width_controls = [(0, .94), (.10, 1.), (.35, .96), (.55, .89),
                      (.72, .68), (.86, .34), (.95, .095), (1, .15)]
    depth_controls = [(0, .0012), (.08, .0020), (.22, .009),
                      (.50, .020), (.72, .014), (.88, .005), (1, .003)]
    rings = []
    for i in range(65):
        u = i / 64
        z = u * height
        dz = z - fruit_center_z(fruit_diameter)
        support = math.sqrt(max(0., clearance_r ** 2 - dz ** 2))
        center_z = fruit_center_z(fruit_diameter)
        tangent_z = (center_z ** 2 - clearance_r ** 2) / center_z
        if z < tangent_z:
            support = z * clearance_r / math.sqrt(
                center_z ** 2 - clearance_r ** 2)
        cx = width * .035 * math.sin(seed * 1.73 + .6) * (1 - u)
        half_width = max(width * .5 * _profile_width(u, width_controls),
                         support + abs(cx))
        depth_half = max(_profile_width(u, depth_controls), support)
        # Elliptical lens cross-sections meet at thin side folds. High-order
        # superellipses previously made thick vertical walls and a box floor.
        power = 2.0
        rings.append((z, cx, half_width, -depth_half, depth_half * 2,
                      power, forms[seed % 3]))
    return rings
