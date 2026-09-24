"""Build a new data-referenced orchard in Blender 4.5; never loads legacy assets.

blender -b -t 8 -P build_scene.py -- --view reference --samples 32
"""
from pathlib import Path
import argparse
import json
import math
import random
import sys
import bpy
import numpy as np
from mathutils import Vector
from mathutils.geometry import intersect_point_line

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from materials import create  # noqa: E402
from geometry import Leaves, bag_rings, mesh, paper_bag, tube  # noqa: E402

RNG = random.Random(240924)
TARGETS = []
BAG_SHAPES = []
CONNECTIONS = []


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


def attach(name, points, radii, mat, parent_point=None):
    if parent_point is not None:
        error = (Vector(points[0]) - Vector(parent_point)).length
        if error > 1e-6:
            raise ValueError(f'Disconnected branch {name}: {error}')
        CONNECTIONS.append({'name': name,
                            'root': list(points[0]),
                            'parent_point': list(parent_point),
                            'error_m': error})
    return tube(name, points, radii, mat)


def add_bag(name, neck, width, height, mats, seed=1, tilt=.1, angle=0):
    rings = bag_rings(width, height, min(width * .53, .067), seed)
    obj = paper_bag(name, rings, mats[f'paper{seed % 3}'], seed)
    # End of stem = gathered neck; bag hangs below, with restrained natural
    # tilt.
    obj.rotation_euler = (tilt, tilt * .2, angle)
    rot = obj.rotation_euler.to_matrix()
    local_neck = Vector((rings[30][1], 0, height * .94))
    obj.location = Vector(neck) - rot @ local_neck
    obj.pass_index = len(TARGETS) + 1
    obj['attachment_world'] = list(neck)
    TARGETS.append({'id': obj.pass_index,
                    'name': name,
                    'neck': list(neck),
                    'width_m': width,
                    'height_m': height,
                    'source': 'distribution of selected RGB-D samples; placement inferred'})
    # Hidden fruit stays comfortably inside; not used as external geometry
    # evidence.
    fruit = sphere(name + '/enclosed peach',
                   obj.location + rot @ Vector((0,
                                                0,
                                                height * .40)),
                   min(width * .19,
                       height * .18),
                   mats['fruit'])
    fruit.pass_index = obj.pass_index
    wirepoints = []
    for j in range(45):
        a = j / 44 * math.tau * 1.4
        wirepoints.append(Vector(neck) + rot @ Vector((width * .062 *
                          math.cos(a), width * .043 * math.sin(a), .001 * j / 44)))
    tube(
        name +
        '/tie',
        wirepoints,
        [.00065] *
        len(wirepoints),
        mats['wire'],
        5)
    return obj


def shoot(name, start, direction, length, mats, leaves, seed, bags=False):
    rng = random.Random(seed)
    start = Vector(start)
    direction = Vector(direction).normalized()
    pts = [start + direction * (length * t) + Vector((.022 * math.sin(
        t * math.pi) * rng.uniform(-1, 1), 0, -.05 * t * t)) for t in [0, .25, .5, .75, 1]]
    attach(name, pts, [.004, .0034, .0026, .0018, .0007], mats['twig'], start)
    count = max(4, int(length / .018))
    for i in range(1, count):
        t = i / count
        seg = min(3, int(t * 4))
        u = t * 4 - seg
        p = pts[seg].lerp(pts[seg + 1], u)
        a = i * 2.399 + seed
        d = Vector((math.cos(a), math.sin(a), rng.uniform(-.45, .3)))
        end = p + d * rng.uniform(.07, .135)
        leaves.add(p, end)
    if bags:
        t = .48
        seg = 1
        u = .92
        anchor = pts[seg].lerp(pts[seg + 1], u)
        neck = anchor + Vector((0, 0, -.012))
        attach(name + '/fruit stem', [anchor, neck],
               [.0018, .0015], mats['twig'], anchor)
        sample=rng.choice(BAG_SHAPES)
        add_bag(name + '/bag', neck, sample['width'], sample['height'],
                mats, seed, rng.uniform(-.22, .22), rng.uniform(-.8, .8))
        TARGETS[-1]['dimension_source']=sample['source']


def tree(name, origin, mats, seed):
    rng = random.Random(seed)
    o = Vector(origin)
    leaves = Leaves(mats, seed)
    trunk = [o, o + Vector((.015, .02, .3)), o + Vector((-.025, .018, .63))]
    tube(name + '/trunk', trunk, [.092, .073, .052], mats['bark'], 14)
    for arm in range(4):
        a = arm * math.tau / 4 + rng.uniform(-.25, .25)
        rad = Vector((math.cos(a), math.sin(a), 0))
        start = trunk[-1]
        pts = [start, start + rad * .28 + Vector((0, 0, .30)), start + rad * .63 + Vector(
            (0, 0, .67)), start + rad * .99 + Vector((0, 0, .80)), start + rad * 1.18 + Vector((0, 0, .96))]
        attach(name + f'/scaffold{arm}', pts,
               [.045, .035, .023, .012, .006], mats['bark'], start)
        for k in range(2, 9):
            t = k / 10
            seg = int(t * 4)
            u = t * 4 - seg
            root = pts[seg].lerp(pts[seg + 1], u)
            side = (-1 if k % 2 else 1)
            b = a + side * rng.uniform(.45, 1.1)
            length = rng.uniform(.40, .74)
            vec = Vector((math.cos(b), math.sin(
                b), rng.uniform(.35, .75))).normalized()
            end = root + vec * length
            mid = root.lerp(end, .55) + Vector((0, 0, .035))
            attach(name + f'/secondary{arm}_{k}',
                   [root, mid, end], [.012, .007, .002], mats['twig'], root)
            for j in range(7):
                s = .19 + j * .12
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
                               2 == 0))
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
            w=max(.001,(right-left)*(front+thickness*.5)/1280)
            x=((right+left)/2-640)*(front+thickness*.5)/640
            z=1.6+(360-row)*front/640
            rings.append((z,x,w,float(front),thickness))
        # Complete beyond the image crop, never identify the crop edge as a neck.
        extension={'top_m':0.,'bottom_m':0.}
        if item['box'][1]<=1:
            z,x,w,front,thick=rings[-1];h=max(.035,width*.42)
            rings.extend([(z+h*.5,x,w*.6,front,thick*.65),(z+h,x,w*.1,front,thick*.2)])
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
    # Additional observed foliage: front/back support follows valid depth, but leaf
    # orientations and hidden petioles remain inferred. All attach to branch
    # graph.
    support = json.loads((HERE / 'evidence/foliage_support.json').read_text())
    for i, item in enumerate(support):
        center = pixel_point(item['u'], item['v'], item['depth_m'])
        a = item['angle_rad']
        length = item['length_m']
        direction = Vector((math.sin(a), RNG.uniform(-.25, .25), -math.cos(a)))
        root = center - direction * length * .5
        tip = center + direction * length * .5
        parent=nearest_branch(root)
        if (parent-root).length>.20:
            continue
        if (parent-root).length>.025:
            # A shared woody shoot, not a separate long petiole for every leaf.
            end=parent.lerp(root,.90)
            midpoint=parent.lerp(end,.5)+Vector((0,.005,.006))
            attach(f'Reference/inferred shared shoot{i}',[parent,midpoint,end],[.0022,.0016,.0009],mats['twig'],parent)
            branch_segments.extend([(parent,midpoint),(midpoint,end)])
            parent=end
        # Stem skeleton hidden by foliage is inferred, but always physically
        # connected.
        attach(f'Reference/inferred petiole{i}', [parent, parent.lerp(
            root, .6), root], [.0018, .0011, .0005], mats['twig'], parent)
        leaves.add(root, tip, length * RNG.uniform(.22, .32))
    # Continue the partial foreground trunk to soil only for the orchard
    # overview.
    lower = trunk[0]
    tube('Reference/inferred lower trunk', [(lower.x - .06, lower.y + .04, 0),
         (lower.x - .03, lower.y + .02, .65), lower], [.055, .035, .018], mats['bark'], 14)
    leaves.finish('Reference')
    return trunk


def ground(mats):
    bpy.ops.mesh.primitive_plane_add(size=200)
    bpy.context.object.name = 'Orchard ground'
    bpy.context.object.data.materials.append(mats['soil'])
    verts = []
    faces = []
    for i in range(150000):
        x = RNG.uniform(-7, 7)
        y = RNG.uniform(-5, 13)
        # Mown path and taller groundcover under tree rows.
        height = RNG.uniform(.025, .10) * (.35 if abs(x - 1.8) < .55 else 1)
        a = RNG.uniform(0, math.tau)
        w = .005
        idx = len(verts)
        verts.extend([(x - w * math.cos(a), y - w * math.sin(a), .002), (x + w * math.cos(a),
                     y + w * math.sin(a), .002), (x + height * .3 * math.sin(a), y, height)])
        faces.append((idx, idx + 1, idx + 2))
    mesh('Groundcover / blades', verts, faces, mats['grass'])


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


def setup_render(out, samples):
    s = bpy.context.scene
    s.render.engine = 'CYCLES'
    s.cycles.device = 'CPU'
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
    s.view_settings.exposure = -.55
    s.world.use_nodes = True
    n = s.world.node_tree.nodes
    l = s.world.node_tree.links
    sky = n.new('ShaderNodeTexSky')
    sky.sky_type = 'NISHITA'
    sky.sun_elevation = math.radians(38)
    sky.sun_rotation = math.radians(125)
    sky.sun_disc = True
    sky.sun_size = math.radians(2)
    l.new(sky.outputs[0], n.get('Background').inputs[0])
    n.get('Background').inputs[1].default_value = .11
    light = bpy.data.lights.new('Sun through canopy', 'SUN')
    light.energy = .75
    light.angle = .06
    obj = bpy.data.objects.new('Sun through canopy', light)
    bpy.context.collection.objects.link(obj)
    obj.rotation_euler = (.5, -.6, -.6)
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
    args = p.parse_args(sys.argv[sys.argv.index(
        '--') + 1:] if '--' in sys.argv else [])
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    out = HERE / 'output'
    out.mkdir(exist_ok=True)
    mats = create()
    report = json.loads(
        (HERE / 'evidence/reference_measurements.json').read_text())
    for fid,frame in report['frames'].items():
        seen=set()
        for item in frame['objects']:
            box=tuple(item['box'])
            if box in seen:continue
            seen.add(box)
            w,h=item['visible_width_m'],item['visible_height_m']
            if not item['truncated'] and item['valid_fraction']>=.8 and .055<w<.19 and .065<h<.24:
                BAG_SHAPES.append({'width':w,'height':h,'source':f'Peach_bag/{fid} instance {item["id"]}; visible extents, FOV approximation'})
    if not BAG_SHAPES:raise ValueError('No measured bag dimensions')
    local_reference(mats, report)
    # Whole orchard is an explicit extrapolation, not claimed as surveyed
    # geometry.
    for i, origin in enumerate([(-.3, 2, 0), (-2, 2, 0), (1.3, 3, 0), (-2, 5.5, 0),
                               (1.3, 6.5, 0), (-5.3, 2, 0), (-5.3, 5.5, 0), (4.6, 3, 0), (4.6, 6.5, 0)]):
        tree(f'Tree{i:02d}', origin, mats, 80 + i)
    ground(mats)
    cams = {'reference': camera('Reference 1200 / approximate 90deg', (0, 0, 1.6), (0, 1, 1.6), 18),
            'detail': camera('Fruit branch / oblique', (-.36, -.12, 1.66), (-.02, .46, 1.58), 27),
            'orchard': camera('Orchard / aisle overview', (5, -6, 2.25), (-.7, 3.7, 1.12), 36)}
    f = setup_render(out, args.samples)
    s = bpy.context.scene
    s.camera = cams['reference']
    s.render.resolution_x = args.width
    s.render.resolution_y = round(args.width * 9 / 16)
    s['data_source'] = 'PeachDataSet 1200 RGB-D + selected samples; reference intrinsics approximate; other trees inferred'
    s['units'] = 'metres'
    s.unit_settings.system = 'METRIC'
    # Save manifest before rendering for crash-safe evidence.
    manifest = {
        'targets': TARGETS,
        'connections': CONNECTIONS,
        'reference_intrinsics': report['intrinsics'],
        'cameras': {},
        'notes': [
            'Reference visible geometry is approximate RGB-D reconstruction.',
            'Hidden branches, leaf orientations, bag thickness and whole orchard are inferred.',
            'No legacy geometry, textures or distributions imported.']}
    bpy.context.view_layer.update()
    for key, cam in cams.items():
        manifest['cameras'][key] = {
            'matrix_world': [
                list(row) for row in cam.matrix_world],
            'location': list(
                cam.location),
            'rotation_euler': list(
                cam.rotation_euler),
            'lens_mm': cam.data.lens,
            'sensor_width_mm': 36}
    (out / 'scene_manifest.json').write_text(json.dumps(manifest, indent=2))
    # Geometry-level checks before saving.
    assert all(c['error_m'] < 1e-6 for c in CONNECTIONS)
    assert len({t['id'] for t in TARGETS}) == len(TARGETS)
    s.camera = cams['reference']
    bpy.ops.wm.save_as_mainfile(filepath=str(
        out / 'bagged_peach_orchard.blend'))
    if args.build_only:
        return
    for view in (cams if args.view == 'all' else [args.view]):
        s.camera = cams[view]
        s.render.filepath = str(out / f'{view}.png')
        for slot, key in zip(f.file_slots, ['Depth', 'IndexOB', 'Position']):
            slot.path = f'{view}_{key}_'
        bpy.ops.render.render(write_still=True)
    print(
        'REBUILD COMPLETE',
        len(TARGETS),
        'bags',
        len(CONNECTIONS),
        'connected branches',
        flush=True)


if __name__ == '__main__':
    main()
