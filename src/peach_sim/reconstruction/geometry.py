"""Blender mesh construction; connected branches and instanced leaf blades."""
import array
import math
import random
import bpy
from mathutils import Quaternion, Vector

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
    return mesh(name, verts, faces, mat)


def blade_vertices(length, width, curl, twist, droop):
    """One blade in local space: petiole at the origin, tip along +X."""
    verts = []
    faces = []
    uv = []
    segments = 24
    for i in range(segments + 1):
        t = i / segments
        # Narrow attached petiole, then a lanceolate, finely serrated blade.
        blade_t = max(0., (t - .08) / .92)
        w = (.00055 if t < .08 else width / 2 * max(
            .005, math.sin(math.pi * blade_t) ** .92)
            * (1 + .018 * (-1 if i % 2 else 1)))
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

    def add(self, root, tip, width=None):
        root = Vector(root)
        tip = Vector(tip)
        axis = tip - root
        length = axis.length
        if length < .005:
            return
        width = width or length * self.rng.uniform(.18, .27)
        quat = axis.to_track_quat('X', 'Z') @ Quaternion(
            (1, 0, 0), self.rng.uniform(-.85, .85))
        index = self.rng.randrange(5) * 4 + self.rng.randrange(4)
        scale = (length / PROTO_LENGTH, width / PROTO_WIDTH, length / PROTO_LENGTH)
        self.records.append((index, root, quat, scale))

    def finish(self, prefix):
        if not self.records:
            return []
        me = bpy.data.meshes.new(f'{prefix} leaf points')
        me.from_pydata([tuple(rec[1]) for rec in self.records], [], [])
        quats = me.attributes.new('leaf_quat', 'QUATERNION', 'POINT')
        indexes = me.attributes.new('leaf_index', 'INT', 'POINT')
        scales = me.attributes.new('leaf_scale', 'FLOAT_VECTOR', 'POINT')
        qflat, iflat, sflat = [], [], []
        for index, _loc, quat, scale in self.records:
            qflat.extend((quat.w, quat.x, quat.y, quat.z))
            iflat.append(index)
            sflat.extend(scale)
        quats.data.foreach_set('value', array.array('f', qflat))
        indexes.data.foreach_set('value', array.array('i', iflat))
        scales.data.foreach_set('vector', array.array('f', sflat))
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
            panel = x / max(w, .001)
            envelope = math.sin(math.pi * t)
            face = min(1., abs(si) / max(abs(co), 1e-8))
            offset = 0.
            for slope, center, spread, amplitude in folds:
                distance = abs(panel - (center + slope * (t - .5)))
                offset += amplitude * max(0., 1 - distance / spread)
            # Local angular folds are sub-millimetre; no global sine-wave warp.
            offset = max(-min(.0012, thick * .06),
                         min(min(.0012, thick * .06), offset))
            offset *= face * envelope
            # Thin folded bottom seam and side fold, integrated into the closed
            # mesh so their pixels inherit the same object/instance index.
            seam = .0007 * math.exp(-((t - .035) / .018)**2)
            seam *= face * envelope
            side = .0005 * max(0., 1 - abs(abs(panel) - .94) / .06) * envelope
            gathers = .0012 * t**8 * math.sin(9 * a + phase) * envelope
            # Outward ribs, zero on the bottom ring. Neck creases (t>0.78)
            # radiate from the tie and stay outside the fruit cheek.
            rib = min(.0016, thick * .05) * math.sin(math.pi * t) * max(
                0., math.sin(5 * a + phase))
            neck = max(0., (t - .78) / .22)
            crease = min(.0025, thick * .15) * neck * math.sin(8 * a + phase)
            y += math.copysign(1, si) * (offset + seam + side + rib) + (
                gathers + crease) * si
            zz = z
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
    obj['geometry'] = 'closed flat paper panels; thin integrated seams and inferred microfolds'
    obj['hidden_surface_status'] = 'thickness, creases and seam construction inferred'
    obj['single_crease_amplitude_bound_m'] = .0008
    obj['combined_crease_amplitude_bound_m'] = .0012
    return obj


# Fruit centre above the bag bottom, so a little paper remains underneath.
FRUIT_SEAT_CLEARANCE_M = .010


def fruit_center_z(diameter):
    return diameter / 2 + FRUIT_SEAT_CLEARANCE_M


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
    """Teardrop paper around a fruit: full cheek, gathered neck, flared lip.

    The 6th ring value is the superellipse power (2.3 = round cheek).
    Without a fruit this falls back to a flat-panel envelope.
    """
    if not fruit_diameter:
        return _empty_bag_rings(width, height, thickness, seed)
    rng = random.Random(seed)
    offset = rng.uniform(-.04, .04) * width
    fruit_r = fruit_diameter / 2
    z_c = fruit_center_z(fruit_diameter)
    cheek = width / 2
    sigma = max(fruit_r * .95, .025)
    neck_half = .008
    rings = []
    steps = 40
    for i in range(steps + 1):
        t = i / steps
        z = t * height
        if t > .92:
            half = neck_half + (t - .92) / .08 * .012
            power = 2.
        else:
            bulge = math.exp(-((z - z_c) / sigma) ** 2)
            half = neck_half + (cheek - neck_half) * bulge
            power = 2.15
        dz = z - z_c
        section = math.sqrt(max(0., fruit_r ** 2 - dz ** 2)) if abs(dz) < fruit_r else 0.
        half = max(half, section + .006, .004)
        # Round cross-section: a flat slab reads as a cushion.
        thick = half * 2
        rings.append((z, offset * (1 - t), half, -thick / 2, thick, power))
    return rings
