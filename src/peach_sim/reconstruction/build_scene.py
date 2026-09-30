"""Build a new data-referenced orchard in Blender 4.5; never loads legacy assets.

blender -b -t 8 -P build_scene.py -- --view reference --samples 32
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import sys

import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from mathutils.geometry import intersect_point_line
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from distributions import (sample_bag_for_fruit, sample_mature_fruit,  # noqa: E402,I100
                           sample_percentile)
from geometry import (bag_rings, fruit_center_z, Leaves, mesh,  # noqa: E402
                      paper_bag, tube)
import lighting as lighting_mod  # noqa: E402
from lighting_apply import apply_lighting  # noqa: E402
from materials import create  # noqa: E402
import occlusion  # noqa: E402
from render_evidence import record_render  # noqa: E402

RNG = random.Random(240924)
TARGETS = []
CONNECTIONS = []
RENDER_BACKEND = 'CPU'

# Controlled-occlusion viewing side: the alley/camera side is -Y in this
# scene layout; foreground leaves and blocking branches are placed between
# bags and this direction (recorded in the manifest).
VIEW_DIR = (0.0, -1.0, 0.0)
CORRIDOR_PROB = .15

_PRIORS = json.loads((HERE / 'evidence' / 'priors.json').read_text())
BAG_STATS = _PRIORS['splits']['Peach_bag']['classes']['0']
# VOC labels encode occlusion, not maturity. Use non-occluded bare fruit.
# Its upper size half is an explicit mature-fruit modelling assumption.
NOBAG_STATS = _PRIORS['splits']['Peach_nobag']['classes']['0']
TILT_STATS = _PRIORS['field_20260909']['axis_tilt_from_up_deg']

SCALES = {
    'validation': {
        'origins': [(-.3, 2, 0), (-2, 2, 0), (1.3, 3, 0), (-2, 5.5, 0),
                    (1.3, 6.5, 0), (-5.3, 2, 0), (-5.3, 5.5, 0),
                    (4.6, 3, 0), (4.6, 6.5, 0)],
        'grass': {'x_range': (-7, 7), 'y_range': (-5, 13), 'path_x': (1.8,)},
        'crown': 1.0, 'backdrop': []},
    'field': {
        'origins': [(x, 1.5 + j * 3.0, 0)
                    for x in (-5.5, -.5, 4.5) for j in range(8)],
        'grass': {'x_range': (-7.5, 7.5), 'y_range': (-1, 24.5),
                  'path_x': (-3.0, 2.0)},
        'crown': 1.5,
        # Block repeats continue the 5 m row / 3 m tree pitch to the horizon.
        'backdrop': [(dx, dy, 0) for dx in (-30, -15, 0, 15, 30)
                     for dy in (0, 24, 48) if (dx, dy) != (0, 0)]},
}
"""validation = 现行 ~10 树验证场（矩阵用）；field = 3 行×8 株锚定场
（成熟冠幅 + 远景行列实例，仅整园图 + 完整 manifest，为导航轮铺路，不进矩阵）。"""


def backdrop(names, offsets):
    """Instance the built block (without bags) as distant rows; visual only.

    Instances inherit pass_index, so target objects stay out of the block
    to keep IndexOB a unique-target GT.
    """
    block = bpy.data.collections.new('Orchard block (instanced)')
    bpy.context.scene.collection.children.link(block)
    for obj in [o for o in bpy.context.scene.objects
                if o.name in names and o.pass_index == 0]:
        for owner in list(obj.users_collection):
            owner.objects.unlink(obj)
        block.objects.link(obj)
    for i, offset in enumerate(offsets):
        inst = bpy.data.objects.new(f'Backdrop rows {i:02d}', None)
        inst.instance_type = 'COLLECTION'
        inst.instance_collection = block
        inst.location = offset
        bpy.context.scene.collection.objects.link(inst)


def sphere(name, center, radius, mat):
    bpy.ops.mesh.primitive_uv_sphere_add(
        segments=24,
        ring_count=12,
        radius=radius,
        location=center)
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(mat)
    for p in obj.data.polygons:
        p.use_smooth = True
    return obj


def peach(name, center, radius, aspect, rotation, mat, seed):
    """Oblate fruit with a suture groove. The groove cuts inward, so the
    mesh stays inside the equatorial bounding sphere the bag was sized for.
    """
    obj = sphere(name, center, radius, mat)
    obj.rotation_euler = rotation
    phase = random.Random(seed).uniform(0, math.tau)
    for v in obj.data.vertices:
        co = v.co
        co.z *= aspect
        # Inward stem cavity preserves the conservative bounding sphere.
        if co.z > radius * aspect * .75:
            cavity = .055 * radius * math.exp(-((co.xy.length / (radius * .20)) ** 2))
            co.z -= cavity
        delta = math.atan2(math.sin(math.atan2(co.y, co.x) - phase),
                           math.cos(math.atan2(co.y, co.x) - phase))
        groove = math.exp(-(delta / .22) ** 2)
        co.x *= (1 - .045 * groove)
        co.y *= (1 - .045 * groove)
        v.co = co
    obj.data.update()
    return obj


def attach(name, points, radii, mat, parent_point=None, parent_radius=None):
    if parent_point is not None:
        error = (Vector(points[0]) - Vector(parent_point)).length
        if error > 1e-6:
            raise ValueError(f'Disconnected branch {name}: {error}')
        CONNECTIONS.append({'name': name,
                            'root': list(points[0]),
                            'parent_point': list(parent_point),
                            'error_m': error})
    if parent_radius is not None and radii[0] > parent_radius:
        raise ValueError(f'{name}: child radius exceeds supporting wood')
    obj = tube(name, points, radii, mat)
    obj['root_radius_m'] = radii[0]
    if parent_radius is not None:
        obj['parent_radius_m'] = parent_radius
    return obj


def add_bag(name, neck, width, height, mats, fruit, seed=1, tilt=.1,
            angle=0, tilt_from_up_deg=None):
    """Paper sized around a mature fruit. The fruit is not shrunk to fit."""
    radius = fruit['diameter_m'] / 2
    rings = bag_rings(width, height, .055, seed, fruit['diameter_m'])
    obj = paper_bag(name, rings, mats[f'paper{seed % 3}'], seed)
    obj['geometry'] = 'loose folded paper envelope over unchanged fruit; gathered neck'
    # End of stem = gathered neck; bag hangs below, with restrained natural
    # tilt.
    obj.rotation_euler = (tilt, tilt * .2, angle)
    rot = obj.rotation_euler.to_matrix()
    upper = [r for r in rings if r[0] > height * .75]
    cinch = min(upper, key=lambda r: r[2])
    local_neck = Vector((cinch[1], 0, cinch[0]))
    obj.location = Vector(neck) - rot @ local_neck
    obj.pass_index = len(TARGETS) + 1
    obj['attachment_world'] = list(neck)
    local_center = Vector((0, 0, fruit_center_z(fruit['diameter_m'])))
    target = {'id': obj.pass_index,
              'name': name,
              'neck': list(neck),
              'width_m': (max(v.co.x for v in obj.data.vertices)
                          - min(v.co.x for v in obj.data.vertices)),
              'height_m': height,
              'paper_form': obj['paper_form'],
              'source': 'paper wrap over mature fruit; placement inferred'}
    target['interior_status'] = fruit['source']
    # World frame recorded for occlusion dressing and viewpoint planning.
    # Centre is the fruit, which is what a picker aims at.
    target['center_world'] = list(obj.location + rot @ local_center)
    target['bottom_world'] = list(obj.location)
    target['axis_world'] = list(
        (rot @ Vector((0, 0, 1))).normalized())
    if tilt_from_up_deg is not None:
        target['tilt_from_up_deg'] = round(tilt_from_up_deg, 2)
    TARGETS.append(target)
    bvh = BVHTree.FromPolygons([v.co for v in obj.data.vertices],
                               [p.vertices[:] for p in obj.data.polygons])
    nearest = bvh.find_nearest(local_center)
    if nearest[3] is None or nearest[3] < radius + .0005:
        raise ValueError(f'{name}: mature fruit does not clear paper '
                         f'(nearest surface {nearest[3]} m, radius '
                         f'{radius:.4f} m)')
    fruit_obj = peach(name + '/enclosed peach',
                      obj.location + rot @ local_center,
                      radius, fruit['aspect'], obj.rotation_euler,
                      mats['fruit'], seed)
    target['fruit_radius_m'] = radius
    target['fruit_aspect'] = round(fruit['aspect'], 3)
    target['fruit_mass_kg'] = round(fruit['mass_kg'], 4)
    target['fruit_diameter_source_percentile'] = fruit[
        'diameter_source_percentile']
    target['fruit_clearance_m'] = nearest[3] - radius
    fruit_obj.pass_index = obj.pass_index
    # Attach to the actual inward stem cavity, not the bounding sphere top.
    pole = max((v.co for v in fruit_obj.data.vertices if v.co.xy.length < 1e-6),
               key=lambda co: co.z)
    fruit_top = fruit_obj.location + rot @ pole
    tube(name + '/fruit peduncle', [fruit_top, Vector(neck)],
         [.0012, .0015], mats['twig'])
    target['peduncle_endpoints_world'] = [list(fruit_top), list(neck)]
    target['peduncle_status'] = 'inferred internal attachment, not observed through paper'
    wirepoints = []
    for j in range(45):
        a = j / 44 * math.tau * 1.4
        co, si = math.cos(a), math.sin(a)
        contour = (abs(co) ** cinch[5] + abs(si) ** cinch[5]) ** (-1 / cinch[5])
        wirepoints.append(Vector(neck) + rot @ Vector((
            co * contour * (cinch[2] + .001),
            si * contour * (cinch[4] * .5 + .001), .0006 * j / 44)))
    tube(
        name +
        '/tie',
        wirepoints,
        [.00065] *
        len(wirepoints),
        mats['wire'],
        5)
    return obj


def dress_occlusion(name, anchor, neck, target, mats, leaves, rng, parent_radius):
    """Controlled leaf occlusion + branch interference for one bag (M2).

    Plans come from occlusion.assign_levels; foreground leaves physically
    hang off the fruit stem (result shoots carry leaves near fruit), and
    interfering branches grow from the same anchor so the connection graph
    stays exact.
    """
    plan = occlusion.assign_levels(rng, 1)[0]
    corridor = rng.random() < CORRIDOR_PROB
    frame = occlusion.BagFrame(
        neck=Vector(neck),
        center=Vector(target['center_world']),
        bottom=Vector(target['bottom_world']),
        face_dir=VIEW_DIR,
        width=target['width_m'],
        height=target['height_m'])
    slots = occlusion.foreground_leaf_slots(frame, plan, rng, VIEW_DIR)
    for k, (root, tip) in enumerate(slots):
        root = Vector(root)
        attach(f'{name}/occluder stem{k}',
               [neck, neck.lerp(root, .5) + Vector((0, .004, .005)), root],
               [.0006, .00045, .0003], mats['twig'], neck, parent_radius=.0009)
        leaves.add(root, Vector(tip), width=rng.uniform(.031, .045))
    if plan.branch_front:
        p0, p1, radius = occlusion.front_branch_segment(frame, rng)
        radius = min(radius, parent_radius * .75)
        attach(f'{name}/blocking branch',
               [anchor, Vector(p0), Vector(p1)],
               [radius, radius * .75, radius * .45], mats['twig'], anchor,
               parent_radius=parent_radius)
    if corridor:
        approach = (Vector(target['bottom_world']) -
                    Vector(neck)).normalized()
        p0, p1, radius = occlusion.corridor_branch_segment(
            frame, rng, approach)
        radius = min(radius, parent_radius * .75)
        attach(f'{name}/corridor branch',
               [anchor, Vector(p0), Vector(p1)],
               [radius, radius * .8, radius * .5], mats['twig'], anchor,
               parent_radius=parent_radius)
    target['occlusion'] = {
        'level': plan.level,
        'nominal_coverage': plan.nominal_coverage,
        'leaves': len(slots),
        'branch_front': plan.branch_front,
        'branch_corridor': corridor,
    }


def shoot(name, start, direction, length, mats, leaves, seed, bags=False,
          parent_radius=.006):
    rng = random.Random(seed)
    start = Vector(start)
    direction = Vector(direction).normalized()
    pts = [start + direction * (length * t) + Vector((.022 * math.sin(
        t * math.pi) * rng.uniform(-1, 1), 0, -.05 * t * t)) for t in [0, .25, .5, .75, 1]]
    base_radius = min(.004, parent_radius * .65)
    radii = [base_radius * f for f in (1, .85, .65, .45, .175)]
    obj = attach(name, pts, radii, mats['young_twig'], start, parent_radius)
    obj['wood_age'] = 'one-year fruiting shoot / inferred'
    frame = direction.to_track_quat('Z', 'Y')
    count = max(4, int(length / .018))
    for i in range(1, count):
        # Bounded jitter cannot reorder alternate nodes along the shoot.
        t = (i + rng.uniform(-.28, .28)) / count
        seg = min(3, int(t * 4))
        u = t * 4 - seg
        p = pts[seg].lerp(pts[seg + 1], u)
        a = i * 2.399 + seed
        d = frame @ Vector((math.cos(a), math.sin(a), rng.uniform(.1, .65))).normalized()
        end = p + d * rng.uniform(.09, .165)
        leaves.add(p, end)
    if bags:
        t = .48
        seg = 1
        u = .92
        anchor = pts[seg].lerp(pts[seg + 1], u)
        neck = anchor + Vector((0, 0, -.012))
        attach(name + '/fruit stem', [anchor, neck],
               [min(.0015, radii[seg] * .7), .0009], mats['twig'], anchor,
               parent_radius=radii[seg])
        fruit = sample_mature_fruit(NOBAG_STATS, rng)
        sample = sample_bag_for_fruit(BAG_STATS, fruit, rng)
        tilt_deg, tilt_u = sample_percentile(TILT_STATS, rng.random())
        add_bag(name + '/bag', neck, sample['width_m'], sample['height_m'],
                mats, fruit, seed, math.radians(tilt_deg),
                rng.uniform(-math.pi, math.pi), tilt_from_up_deg=tilt_deg)
        TARGETS[-1]['dimension_source'] = sample['source']
        TARGETS[-1]['dimension_fit'] = sample['dimension_fit']
        TARGETS[-1]['observed_paper_size_m'] = [
            sample['observed_width_m'], sample['observed_height_m']]
        TARGETS[-1]['width_source_percentile'] = sample[
            'width_source_percentile']
        TARGETS[-1]['aspect_source_percentile'] = sample[
            'aspect_source_percentile']
        TARGETS[-1]['tilt_source_percentile'] = round(tilt_u, 4)
        dress_occlusion(name, anchor, neck, TARGETS[-1], mats, leaves, rng,
                        radii[seg] * (1 - u) + radii[seg + 1] * u)


def tree(name, origin, mats, seed, crown=1.0):
    """Open-center tree with per-tree structural randomisation.

    Anchored to the orchard standard in evidence/priors.json (三主枝自然
    开心形, trunk 0.40-0.50 m grown range, tree <=2.5 m, scaffold elevation
    40-70°). Trunk height/lean, scaffold count (3 standard, occasionally 4),
    length, polar angle, crown volume and side-wood density all vary per
    tree — the pre-2026-09-29 version hardcoded one clone per row and only
    jittered azimuth.

    ``crown`` scales scaffold/side-wood reach and crown-shoot count; 1.0
    keeps the validation-scene tree unchanged. Field scale uses a mature
    crown so neighbouring canopies meet at the 3 m in-row spacing, as in
    the reference photos.
    """
    rng = random.Random(seed)
    o = Vector(origin)
    leaves = Leaves(mats, seed)
    trunk_h = rng.uniform(*_PRIORS['orchard_standard']['trunk_height_m'])
    trunk = [o,
             o + Vector((rng.uniform(-.035, .035), rng.uniform(-.035, .035),
                         trunk_h * .5)),
             o + Vector((rng.uniform(-.06, .06), rng.uniform(-.06, .06),
                         trunk_h))]
    base_r = rng.uniform(.084, .100)
    tube(name + '/trunk', trunk,
         [base_r * 1.18, base_r * .88, base_r * .60], mats['bark'], 14)
    arms = rng.choice((3, 3, 3, 4))
    base_az = rng.uniform(0, math.tau)
    anchors = []
    for arm in range(arms):
        a = base_az + arm * math.tau / arms + rng.uniform(-.28, .28)
        polar = math.radians(rng.uniform(42, 58))
        length = rng.uniform(.80, 1.15) * crown
        horiz = math.cos(polar) * length
        rad = Vector((math.cos(a) * horiz, math.sin(a) * horiz,
                      math.sin(polar) * length))
        # Stagger scaffolds on the actual upper trunk centerline.
        root_t = .40 / trunk_h + (1 - .40 / trunk_h) * arm / max(arms - 1, 1)
        top = trunk[1].lerp(trunk[2], (root_t - .5) * 2)
        bend = top + rad * .5 + Vector((0, 0, rng.uniform(.00, .09)))
        tip = top + rad
        tip.z = min(tip.z, 2.35)
        pts = [top, bend, tip]
        parent_u = (root_t - .5) * 2
        parent_r = base_r * (.88 * (1 - parent_u) + .60 * parent_u)
        # Crown reach does not authorize wood thicker than its supporting trunk.
        scale = min(length / 1.0, parent_r * .92 / .046)
        attach(name + f'/scaffold{arm}', pts,
               [.046 * scale, .027 * scale, .013 * scale], mats['bark'], top,
               parent_radius=parent_r)
        anchors.append((pts, a, [.046 * scale, .027 * scale, .013 * scale]))
        n_seg = len(pts) - 1
        n_side = round(7 * crown)
        for k in range(2, 2 + n_side):
            t = .2 + .6 * (k - 2) / (n_side - 1)
            seg = min(int(t * n_seg), n_seg - 1)
            u = t * n_seg - seg
            root = pts[seg].lerp(pts[seg + 1], u)
            side = (-1 if k % 2 else 1)
            b = a + side * rng.uniform(.45, 1.1)
            length_s = rng.uniform(.40, .74) * (1 + (crown - 1) * .6)
            vec = Vector((math.cos(b), math.sin(
                b), rng.uniform(.35, .75))).normalized()
            length_s = max(.15, min(length_s, (2.2 - root.z) / max(vec.z, .1)))
            end = root + vec * length_s
            mid = root.lerp(end, .55) + Vector((0, 0, .035))
            scaffold_r = ([.046, .027, .013][seg] * (1 - u)
                          + [.046, .027, .013][seg + 1] * u) * scale
            secondary_r = min(.012, scaffold_r * .65)
            secondary_radii = [secondary_r * f for f in (1, .58, .17)]
            attach(name + f'/secondary{arm}_{k}',
                   [root, mid, end], secondary_radii, mats['twig'], root, scaffold_r)
            anchors.append(([root, mid, end], b, secondary_radii))
            n_shoot = round(7 * crown)
            for j in range(n_shoot):
                s = .19 + .72 * j / (n_shoot - 1)
                r = root.lerp(mid,
                              s / .55) if s <= .55 else mid.lerp(end,
                                                                 (s - .55) / .45)
                d = Vector((math.cos(b + (-1)**j * .85),
                           math.sin(b + (-1)**j * .85), rng.uniform(-.12, .65)))
                shoot(name +
                      f'/shoot{arm}_{k}_{j}', r, d, rng.uniform(.21, .39), mats, leaves, seed *
                      1000 +
                      arm *
                      100 +
                      k *
                      10 +
                      j, bags=(j == 1 and k %
                               2 == 0), parent_radius=(
                          secondary_radii[0] * (1 - s / .55)
                          + secondary_radii[1] * s / .55 if s <= .55 else
                          secondary_radii[1] * (1 - (s - .55) / .45)
                          + secondary_radii[2] * (s - .55) / .45))
    # Leaf-bearing extension shoots, rooted on the actual bent parent axis.
    # Do not bridge the endpoints of a curved branch: that floats roots in air.
    # Keep the vase centre open by favouring outward directions.
    for c in range(round(rng.randint(20, 32) * crown * crown)):
        pts_list, az, parent_radii = rng.choice(anchors)
        position = rng.uniform(.3, .95) * (len(pts_list) - 1)
        segment = min(int(position), len(pts_list) - 2)
        base = pts_list[segment].lerp(
            pts_list[segment + 1], position - segment)
        azimuth = az + rng.uniform(-.65, .65)
        d = Vector((math.cos(azimuth), math.sin(azimuth),
                    rng.uniform(.7, 1.7))).normalized()
        length = min(rng.uniform(.38, .65), (2.32 - base.z) / max(d.z, .1))
        if length < .12:
            continue
        shoot(name + f'/crown{c}', base, d, length,
              mats, leaves, seed * 1000 + 777 + c, bags=False,
              parent_radius=parent_radii[segment] * (1 - (position - segment))
              + parent_radii[segment + 1] * (position - segment))
    leaves.finish(name)


def pixel_point(u, v, depth):
    return Vector(((u - 640) * depth / 640, depth,
                  1.6 + (360 - v) * depth / 640))


def local_reference(mats, report):
    """Visible bag profile measured; branch centerlines manually traced in RGB-D 1200."""
    data = report['frames']['1200']
    depth = np.load(HERE / 'evidence/1200_depth.npy')

    def trace_point(u, v):
        samples = depth[max(0, v - 5):v + 6, max(0, u - 5):u + 6]
        valid = samples[samples > 0]
        if len(valid) < 5:
            raise ValueError(f'No depth for branch trace {u},{v}')
        return pixel_point(u, v, float(np.median(valid)) * .001)
    # RGB trace marks and supporting depths are approximate. Occluded continuations
    # and the 45 mm rear offset of branch paths are explicit geometric inference.
    trunk_uv = [(536, 714), (552, 618), (553, 508), (566, 402),
                (588, 296), (594, 193), (617, 80), (632, 4)]
    trunk = [trace_point(*p) for p in trunk_uv]
    tube('Reference/visible trunk', trunk,
         [.018, .019, .02, .021, .021, .018, .016, .014], mats['bark'], 14)
    seen = set()
    necks = []
    for item in data['objects']:
        key = tuple(item['box'])
        if key in seen:
            continue
        seen.add(key)
        if item['valid_fraction'] < .7:
            continue
        profile=np.array([[np.nan if v is None else v for v in row] for row in item['profile']],dtype=float)[::-1]
        center=item['depth_m'];width=item['visible_width_m'];thickness=min(.065,width*.50)
        valid=np.isfinite(profile[:,3])
        if not valid.any():raise ValueError('No row-depth support for bag')
        row_depth=np.interp(np.arange(len(profile)),np.where(valid)[0],profile[valid,3])
        row_depth=np.clip(row_depth,*item['depth_p10_p90_m'])
        row_depth=np.convolve(np.pad(row_depth,2,mode='edge'),np.ones(5)/5,mode='valid')
        rings=[]
        for (row,left,right,_),front in zip(profile,row_depth):
            w=max(.001,(right-left)*front/1280)
            x=((right+left)/2-640)*front/640
            z=1.6+(360-row)*front/640
            rings.append((z,x,w,float(front),thickness))
        # Complete beyond the image crop, never identify the crop edge as a neck.
        extension={'top_m':0.,'bottom_m':0.}
        if item['box'][1]<=1:
            z,x,w,front,thick=rings[-1];h=max(.028,width*.28)
            rings.extend([(z+h*.45,x,w*.42,front,thick*.4),(z+h,x,w*.16,front,thick*.22)])
            extension['top_m']=h
        if item['box'][3]>=719:
            z,x,w,front,thick=rings[0];h=max(.045,width*.4)
            rings=[(z-h,x,w*.88,front,thick*.35),(z-h*.4,x,w,front,thick*.7)]+rings
            extension['bottom_m']=h
        obj = paper_bag(
            f'Reference/bag_{item["id"]}', rings, mats[f'paper{item["id"] % 3}'], item['id'] + 20)
        obj.pass_index = len(TARGETS) + 1
        neck = Vector((rings[-1][1], rings[-1][3] + rings[-1][4] * .5, rings[-1][0]))
        obj['source'] = 'PeachDataSet/Peach_bag/1200; SAM + valid depth; backside inferred'
        TARGETS.append({'id': obj.pass_index, 'name': obj.name, 'neck': list(neck), 'source_frame': '1200', 'source_instance': item['id'],
                        'visible_width_m': width, 'visible_height_m': item['visible_height_m'], 'depth_m': center, 'valid_fraction': item['valid_fraction'], 'reference_box': item['box'], 'truncated': item['truncated'],'inferred_crop_extension':extension,'row_depth_support':int(valid.sum())})
        necks.append(neck)
        # Opaque reference bags do not reveal the fruit: preserve unknown interior.
        # Explicit fruit geometry is limited to the inferred orchard instances.
        obj['interior_status']='unobserved; no invented fruit surface'
        angles = [j * math.tau / 32 for j in range(33)]
        wire = [
            neck + Vector((.006 * math.cos(a), .004 * math.sin(a), .0005)) for a in angles]
        tube(obj.name + '/tie', wire, [.0006] * len(wire), mats['wire'], 5)
    leaves = Leaves(mats, 240)
    # Branches traced to actual visible junctions, then connected to bag necks.
    paths = [[(553, 508), (629, 480), (749, 464), (903, 439), (1064, 407)],
             [(566, 402), (530, 359), (468, 339), (363, 335), (292, 291)],
             [(588, 296), (683, 260), (776, 218)],
             [(552, 618), (629, 599), (713, 556)],
             [(594, 193), (538, 157), (414, 157), (306, 172)],
             [(617, 80), (760, 54), (940, 30)]]
    branch_points = list(trunk)
    branch_segments=list(zip(trunk,trunk[1:]))
    def nearest_branch(point):
        candidates=[]
        for a,b in branch_segments:
            _,factor=intersect_point_line(point,a,b)
            q=a.lerp(b,max(0,min(1,factor)))
            candidates.append(q)
        return min(candidates,key=lambda q:(q-point).length)
    for i, path in enumerate(paths):
        pts = [trace_point(*p) for p in path]
        # Occluded supporting wood runs behind the visible paper, not through
        # it.
        pts = [p + Vector((0, .045, 0)) for p in pts]
        parent = min(trunk, key=lambda p: (p - pts[0]).length)
        pts[0] = parent
        attach(f'Reference/traced branch{i}',
               pts,
               [.008 * (1 - j / (len(pts) + .5)) for j in range(len(pts))],
               mats['twig'],
               parent)
        branch_points += pts
        branch_segments.extend(zip(pts,pts[1:]))
    for i, neck in enumerate(necks):
        parent = nearest_branch(neck)
        mid = parent.lerp(neck, .6) + Vector((0, .018, .012))
        attach(f'Reference/inferred attachment{i}', [
               parent, mid, neck], [.003, .0022, .0013], mats['twig'], parent)
        # Shoots continue beyond fruit attachment rather than ending at the
        # bag.
        d = (neck - parent).normalized() + Vector((0, .1, .35))
        attach(f'Reference/fruit shoot{i}', [neck, neck +
               d * .07], [.0014, .0004], mats['twig'], neck)
    # Visible mask outlines are constrained by the source image. Per-leaf
    # curved depth completion and hidden attachments remain explicit inference.
    observed = np.load(HERE / 'evidence/observed_foliage.npz')
    surface = mesh('Reference/observed foliage surface', observed['vertices'].tolist(),
                   observed['faces'].tolist(), mats['leaf0'], observed['uv'].tolist())
    for index in range(1, 5):
        surface.data.materials.append(mats[f'leaf{index}'])
    for polygon, index in zip(surface.data.polygons, observed['materials']):
        polygon.material_index = int(index)
    surface['source'] = '1200 estimated leaf masks and median valid depth; no photo texture'
    surface['surface_status'] = 'visible silhouettes; curved sheets and hidden backs inferred'
    surface['source_triangles'] = len(observed['faces'])
    measured = json.loads((HERE / 'evidence/observed_foliage.json').read_text())
    surface['patch_count'] = len(measured['patches'])
    for index, patch in enumerate(measured['patches']):
        ends = [Vector(p) for p in patch['endpoints']]
        root = min(ends, key=lambda p: (nearest_branch(p) - p).length)
        parent = nearest_branch(root)
        midpoint = parent.lerp(root, .65)
        attach(f'Reference/inferred leaf support{index}', [parent, midpoint, root],
               [.0015, .001, .0004], mats['twig'], parent)
    # Continue the partial foreground trunk to soil only for the orchard
    # overview.
    lower = trunk[0]
    tube('Reference/inferred lower trunk', [(lower.x - .06, lower.y + .04, 0),
         (lower.x - .03, lower.y + .02, .65), lower], [.055, .035, .018], mats['bark'], 14)
    return trunk


def ground(mats, x_range=(-7, 7), y_range=(-5, 13), path_x=(1.8,)):
    bpy.ops.mesh.primitive_plane_add(size=200)
    bpy.context.object.name = 'Orchard ground'
    bpy.context.object.data.materials.append(mats['soil'])
    # Fixed local seed keeps vegetation independent of tree construction order.
    rng = random.Random(240929)
    verts, faces = [], []
    area = (x_range[1] - x_range[0]) * (y_range[1] - y_range[0])
    for _ in range(round(area * 75)):
        x, y = rng.uniform(*x_range), rng.uniform(*y_range)
        cover = (.5 + .25 * math.sin(.8 * x + .31 * y)
                 + .25 * math.sin(1.4 * y - .26 * x))
        if rng.random() > .18 + .82 * cover:
            continue
        mown = min(abs(x - p) for p in path_x) < .65
        for _ in range(rng.randint(4, 7)):
            angle = rng.uniform(0, math.tau)
            height = rng.uniform(.035, .14) * (.25 if mown else 1)
            width = rng.uniform(.0015, .0035)
            root = Vector((x + rng.uniform(-.025, .025),
                           y + rng.uniform(-.025, .025), .001))
            side = Vector((math.cos(angle), math.sin(angle), 0))
            bend = Vector((-math.sin(angle), math.cos(angle), 0))
            start = len(verts)
            for level in range(4):
                t = level / 3
                center = root + bend * (height * .65 * t * t)
                center.z += height * (t - .22 * t * t)
                half = width * (1 - t) + .00008
                verts.extend((center - side * half, center + side * half))
            for level in range(3):
                i = start + level * 2
                faces.append((i, i + 1, i + 3, i + 2))
    obj = mesh('Groundcover / curved tufts', verts, faces, mats['grass'])
    obj['source'] = 'inferred mown alley and patchy grass; not surveyed vegetation'
    # Sparse curled litter and low soil crumbs supply contact shadows without
    # altering the measured world-Z baseline or inventing navigable slopes.
    verts, faces = [], []
    for _ in range(round(area * 3)):
        x, y = rng.uniform(*x_range), rng.uniform(*y_range)
        angle = rng.uniform(0, math.tau)
        length, width = rng.uniform(.035, .09), rng.uniform(.008, .022)
        axis = Vector((math.cos(angle), math.sin(angle), 0))
        side = Vector((-math.sin(angle), math.cos(angle), 0))
        root = Vector((x, y, .002))
        start = len(verts)
        for i in range(7):
            t = i / 6
            half = width * .5 * max(.01, math.sin(math.pi * t))
            center = root + axis * (length * t)
            center.z += .006 * t * t
            verts.extend((center - side * half, center + Vector((0, 0, .002)),
                          center + side * half))
        for i in range(6):
            for j in range(2):
                k = start + i * 3 + j
                faces.append((k, k + 1, k + 4, k + 3))
    mesh('Groundcover / curled fallen leaves', verts, faces, mats['litter'])
    verts, faces = [], []
    for _ in range(round(area * 5)):
        x, y = rng.uniform(*x_range), rng.uniform(*y_range)
        r = rng.uniform(.006, .023)
        start = len(verts)
        for j in range(5):
            angle = math.tau * j / 5
            verts.append((x + r * math.cos(angle), y + r * math.sin(angle), .001))
        verts.append((x + r * .2, y, r * rng.uniform(.3, .7)))
        for j in range(5):
            faces.append((start + j, start + (j + 1) % 5, start + 5))
        faces.append(tuple(start + j for j in reversed(range(5))))
    mesh('Groundcover / soil crumbs', verts, faces, mats['soil'], smooth=False)


def camera(name, location, target, lens):
    data = bpy.data.cameras.new(name)
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    obj.location = location
    obj.rotation_euler = (
        Vector(target) -
        obj.location).to_track_quat(
        '-Z',
        'Y').to_euler()
    data.type = 'PERSP'
    data.lens = lens
    data.sensor_width = 36
    data.clip_start = .015
    data.clip_end = 200
    return obj


def bag_detail_camera(lighting_name):
    """Choose a readable bag face without deleting surrounding vegetation."""
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    preset = lighting_mod.get(lighting_name)
    az = math.radians(preset.sun_azimuth_deg)
    el = math.radians(preset.sun_elevation_deg)
    sun = Vector((math.sin(az) * math.cos(el),
                  math.cos(az) * math.cos(el), math.sin(el)))
    best = None
    for target in [t for t in TARGETS if t.get('fruit_radius_m')][:48]:
        bag = bpy.data.objects[target['name']]
        rotation = bag.rotation_euler.to_matrix()
        body_shift = target['height_m'] * .45 - fruit_center_z(2 * target['fruit_radius_m'])
        center = Vector(target['center_world']) + rotation @ Vector((0, 0, body_shift))
        for side, face_side in ((-.55, -1), (0., -1), (.55, -1),
                                (-.55, 1), (0., 1), (.55, 1)):
            direction = (rotation @ Vector((side, face_side, .22))).normalized()
            eye = center + direction * .60
            right = direction.cross(Vector((0, 0, 1))).normalized()
            up = rotation @ Vector((0, 0, 1))
            visible = 0
            lit = 0
            for x in (-1, 0, 1):
                for z in (-1, 0, 1):
                    point = (center + right * x * target['width_m'] * .34
                             + up * z * target['height_m'] * .28)
                    ray = point - eye
                    hit, point_hit, normal, _i, obj, _m = bpy.context.scene.ray_cast(
                        depsgraph, eye, ray.normalized(), distance=ray.length)
                    if hit and obj.original.name == bag.name:
                        visible += 1
                        if normal.dot(sun) > .15:
                            shadow = bpy.context.scene.ray_cast(
                                depsgraph, point_hit + sun * .002, sun, distance=20)[0]
                            lit += int(not shadow)
            neighbours = sum((Vector(t['center_world']) - center).length < .28
                             for t in TARGETS if t.get('fruit_radius_m'))
            score = visible * 100 + lit * 3 + min(neighbours, 4)
            if best is None or score > best[0]:
                best = (score, eye, center, target['id'], visible / 9, lit / 9)
    cam = camera('Bag-priority close-up / inferred tree', best[1], best[2], 42)
    cam['target_id'] = best[3]
    cam['visible_probe_fraction'] = best[4]
    cam['sunlit_probe_fraction'] = best[5]
    return cam


def _enable_gpu():
    """Prefer OptiX/CUDA over CPU rendering; return the compute backend used."""
    try:
        prefs = bpy.context.preferences.addons['cycles'].preferences
        for backend in ('OPTIX', 'CUDA', 'HIP'):
            try:
                prefs.compute_device_type = backend
                prefs.get_devices()
            except Exception:
                continue
            gpus = [d for d in prefs.devices if d.type != 'CPU']
            if not gpus:
                continue
            # One GPU: dual OptiX BVH on this orchard has hung the driver.
            chosen = gpus[0]
            for d in prefs.devices:
                d.use = d == chosen
            print('CYCLES DEVICE', backend, chosen.name, flush=True)
            return backend
    except Exception as exc:  # GPU optional: fall back silently to CPU.
        print('CYCLES DEVICE fallback CPU:', exc, flush=True)
    print('CYCLES DEVICE CPU', flush=True)
    return 'CPU'


def setup_render(out, samples, lighting_name='noon'):
    preset = lighting_mod.get(lighting_name)
    s = bpy.context.scene
    s.render.engine = 'CYCLES'
    backend = _enable_gpu()
    global RENDER_BACKEND
    RENDER_BACKEND = backend
    s.cycles.device = 'GPU' if backend != 'CPU' else 'CPU'
    s.cycles.samples = samples
    s.cycles.use_denoising = True
    s.cycles.max_bounces = 7
    s.cycles.transparent_max_bounces = 5
    s.render.resolution_x = 1280
    s.render.resolution_y = 720
    s.render.resolution_percentage = 100
    s.render.image_settings.file_format = 'PNG'
    s.render.image_settings.color_mode = 'RGB'
    s.view_settings.view_transform = 'AgX'
    s.view_settings.look = 'AgX - Medium High Contrast'
    s.world.use_nodes = True
    apply_lighting(s, preset)
    # Full floating-point world position is unambiguous for optical Z
    # conversion.
    layer = s.view_layers[0]
    layer.use_pass_z = True
    layer.use_pass_object_index = True
    layer.use_pass_position = True
    s.use_nodes = True
    nodes = s.node_tree.nodes
    nodes.clear()
    r = nodes.new('CompositorNodeRLayers')
    com = nodes.new('CompositorNodeComposite')
    s.node_tree.links.new(r.outputs['Image'], com.inputs[0])
    f = nodes.new('CompositorNodeOutputFile')
    f.base_path = str(out)
    f.format.file_format = 'OPEN_EXR'
    f.format.color_depth = '32'
    f.format.exr_codec = 'ZIP'
    f.file_slots.clear()
    for key in ['Depth', 'IndexOB', 'Position']:
        f.file_slots.new(key)
        if key == 'Position':
            sep = nodes.new('CompositorNodeSeparateXYZ')
            comb = nodes.new('CompositorNodeCombRGBA')
            s.node_tree.links.new(r.outputs[key], sep.inputs[0])
            for xyz, rgba in zip(['X', 'Y', 'Z'], ['R', 'G', 'B']):
                s.node_tree.links.new(sep.outputs[xyz], comb.inputs[rgba])
            s.node_tree.links.new(comb.outputs[0], f.inputs[key])
        else:
            s.node_tree.links.new(r.outputs[key], f.inputs[key])
    return f


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        '--view',
        choices=[
            'reference',
            'detail',
            'orchard',
            'all'],
        default='all')
    p.add_argument('--samples', type=int, default=32)
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--build-only', action='store_true')
    p.add_argument('--lighting', default='noon',
                   choices=sorted(lighting_mod.PRESETS),
                   help='outdoor lighting preset (lighting.py)')
    p.add_argument('--scale', default='validation',
                   choices=sorted(SCALES),
                   help='validation = matrix scene; field = anchor render only')
    p.add_argument('--out', default='',
                   help='alternate output dir (e.g. field anchor renders); '
                        'default output/ holds the validation-scene contract')
    args = p.parse_args(sys.argv[sys.argv.index(
        '--') + 1:] if '--' in sys.argv else [])
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    out = Path(args.out) if args.out else HERE / 'output'
    out.mkdir(parents=True, exist_ok=True)
    mats = create()
    report = json.loads(
        (HERE / 'evidence/reference_measurements.json').read_text())
    if args.scale == 'validation':
        local_reference(mats, report)
    # Whole orchard is an explicit extrapolation, not claimed as surveyed
    # geometry.
    scale = SCALES[args.scale]
    before = {o.name for o in bpy.data.objects}
    for i, origin in enumerate(scale['origins']):
        tree(f'Tree{i:02d}', origin, mats, 80 + i, scale['crown'])
    ground(mats, **scale['grass'])
    if scale['backdrop']:
        backdrop({o.name for o in bpy.data.objects} - before - {'Orchard ground'},
                 scale['backdrop'])
    cams = {
        'reference': camera('Reference 1200 / approximate 90deg',
                            (0, 0, 1.6), (0, 1, 1.6), 18),
        'detail': (bag_detail_camera(args.lighting) if args.scale == 'validation' else
                   camera('Field unused close-up',
                          (-.92, .72, 1.22), (-.40, 1.44, 1.05), 40)),
        'orchard': camera('Orchard / aisle overview',
                          (5, -6, 2.25), (-.7, 3.7, 1.12), 36)}
    if args.scale == 'field':
        overview = cams['orchard']
        overview.location = (12, -11, 9)
        overview.rotation_euler = (Vector((0, 10, .8)) - overview.location).to_track_quat(
            '-Z', 'Y').to_euler()
        overview.data.lens = 32
        # Robot-height stop in the x=-3.0 alley facing the middle row,
        # dataset intrinsics (18 mm on 36 mm = 90° HFOV).
        cams['aisle'] = camera('Field / alley stop', (-3.0, 10.5, 1.4),
                               (-.5, 10.9, 1.35), 18)
    f = setup_render(out, args.samples, args.lighting)
    s = bpy.context.scene
    s.camera = cams['reference']
    s.render.resolution_x = args.width
    s.render.resolution_y = round(args.width * 9 / 16)
    s['data_source'] = 'PeachDataSet 1200 RGB-D + selected samples; reference intrinsics approximate; other trees inferred'
    s['units'] = 'metres'
    s.unit_settings.system = 'METRIC'
    # Save manifest before rendering for crash-safe evidence.
    manifest = {
        'modeling_revision': '2026-09-30-reference-scene-v11',
        'source_sha256': {
            name: hashlib.sha256((HERE / name).read_bytes()).hexdigest()
            for name in ('build_scene.py', 'geometry.py', 'materials.py',
                         'lighting.py', 'lighting_apply.py', 'occlusion.py',
                         'distributions.py', 'make_textures.py', 'textures/paper_height.png',
                         'evidence/priors.json',
                         'evidence/reference_measurements.json',
                         'measure_reference_leaves.py', 'observed_foliage.py',
                         'evidence/reference_leaf_prompts.json',
                         'evidence/reference_leaf_masks.npz',
                         'evidence/reference_leaf_measurement.json',
                         'evidence/observed_foliage.npz',
                         'evidence/observed_foliage.json')},
        'render_settings': {'samples': args.samples, 'width': args.width,
                            'height': s.render.resolution_y,
                            'exposure_ev': s.view_settings.exposure,
                            'view_transform': s.view_settings.view_transform,
                            'look': s.view_settings.look},
        'targets': TARGETS,
        'connections': CONNECTIONS,
        'reference_intrinsics': report['intrinsics'],
        'render_backend': RENDER_BACKEND,
        'lighting': lighting_mod.manifest_entry(args.lighting),
        'scale': args.scale,
        'reference_patch_included': args.scale == 'validation',
        'crown_scale': scale['crown'],
        'backdrop_offsets_m': [list(o) for o in scale['backdrop']],
        'occlusion_view_dir': list(VIEW_DIR),
        'cameras': {},
        'notes': [
            ('Only bagged peaches are presented; enclosed fruits support geometry, '
             'not naked-fruit display.'),
            'Paper dimensions sample Peach_bag class 0 quantiles within fruit-supported bounds.',
            'Paper folds and hidden back surfaces are inferred; fruit is not shrunk to fit.',
            ('Reference interiors are unobserved; missing modeled interiors do not '
             'mean empty bags.'),
            'Bag-priority detail uses visible-ray probes; canopy geometry is unchanged.',
            'Tree structure, leaf prototypes and internal fruit semantics retain v8 corrections.']}
    bpy.context.view_layer.update()
    for key, cam in cams.items():
        manifest['cameras'][key] = {
            'object_name': cam.name,
            'matrix_world': [
                list(row) for row in cam.matrix_world],
            'location': list(
                cam.location),
            'rotation_euler': list(
                cam.rotation_euler),
            'target_id': cam.get('target_id'),
            'visible_probe_fraction': cam.get('visible_probe_fraction'),
            'sunlit_probe_fraction': cam.get('sunlit_probe_fraction'),
            'lens_mm': cam.data.lens,
            'sensor_width_mm': 36}
    (out / 'scene_manifest.json').write_text(json.dumps(manifest, indent=2))
    # Geometry-level checks before saving.
    assert all(c['error_m'] < 1e-6 for c in CONNECTIONS)
    assert len({t['id'] for t in TARGETS}) == len(TARGETS)
    s.camera = cams['orchard'] if args.scale == 'field' else cams['reference']
    bpy.ops.wm.save_as_mainfile(filepath=str(
        out / 'bagged_peach_orchard.blend'))
    manifest['source_blend_sha256'] = hashlib.sha256(
        (out / 'bagged_peach_orchard.blend').read_bytes()).hexdigest()
    (out / 'scene_manifest.json').write_text(json.dumps(manifest, indent=2))
    if args.build_only:
        return
    views = list(cams) if args.view == 'all' else [args.view]
    if args.scale == 'field':
        # Field scale is the navigation-round anchor: overview + alley stop.
        views = ['orchard', 'aisle']
        print('field scale: rendering orchard + aisle anchor views', flush=True)
    for view in views:
        s.camera = cams[view]
        s.render.filepath = str(out / f'{view}.png')
        for slot, key in zip(f.file_slots, ['Depth', 'IndexOB', 'Position']):
            slot.path = f'{view}_{key}_'
        bpy.ops.render.render(write_still=True)
        record_render(out, view, manifest)
    print(
        'REBUILD COMPLETE',
        len(TARGETS),
        'bags',
        len(CONNECTIONS),
        'connected branches',
        flush=True)


if __name__ == '__main__':
    main()
