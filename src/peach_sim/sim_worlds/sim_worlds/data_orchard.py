"""Generate visual/physical/navigation resources from one exported orchard manifest."""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw
import yaml

from .cli import default_share
from .demo import _node, fruit_sdf, world_base


def _cylinder(link, item):
    collision = _node(link, 'collision', name=item['id'])
    pose = (*item['position'], *item['orientation'])
    _node(collision, 'pose', ' '.join(map(str, pose)))
    cylinder = _node(_node(collision, 'geometry'), 'cylinder')
    _node(cylinder, 'radius', item['radius'])
    _node(cylinder, 'length', item['length'])


def _map(obstacles, out):
    """Project woody geometry in the chassis/laser height band into one occupancy map."""
    resolution, origin = .025, (-6., -6.)
    image = Image.new('L', (480, 480), 254)
    draw = ImageDraw.Draw(image)

    def pixel(x, y):
        return ((x - origin[0]) / resolution, (6 - y) / resolution)

    for item in obstacles:
        a, b = item['endpoints']
        radius = item['radius']
        if max(a[2], b[2]) + radius < .10 or min(a[2], b[2]) - radius > .85:
            continue
        # Clip inclined axes to the same physical height band; upper canopy
        # branches remain 3D arm obstacles without blocking the whole alley.
        if abs(b[2] - a[2]) > 1e-6:
            limits = sorted(((.10 - radius - a[2]) / (b[2] - a[2]),
                             (.85 + radius - a[2]) / (b[2] - a[2])))
            low, high = max(0., limits[0]), min(1., limits[1])
            points = [[a[i] + t * (b[i] - a[i]) for i in range(2)] for t in (low, high)]
        else:
            points = [a[:2], b[:2]]
        width = max(1, math.ceil(2 * radius / resolution))
        draw.line([pixel(*p) for p in points], fill=0, width=width)
        for x, y in points:
            draw.ellipse((*pixel(x - radius, y + radius),
                          *pixel(x + radius, y - radius)), fill=0)
    draw.rectangle((0, 0, 479, 479), outline=0, width=3)
    image.save(out / 'map.pgm')
    path = out / 'map.yaml'
    path.write_text(yaml.safe_dump({
        'image': 'map.pgm', 'mode': 'trinary', 'resolution': resolution,
        'origin': [*origin, 0.], 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}))
    return path


def prepare_data_orchard(output_directory, asset_directory=None):
    """Keep source bag size/axis, collision segments and tree visuals in the same frame."""
    out = Path(output_directory).resolve()
    out.mkdir(parents=True, exist_ok=True)
    assets = Path(asset_directory or default_share() / 'worlds/data_orchard').resolve()
    report = json.loads((assets / 'assets.json').read_text())
    manifest = deepcopy(report)
    for item in manifest['trees'] + manifest['targets']:
        key = 'mesh' if 'mesh' in item else 'visual_mesh'
        path = (assets / item[key]).resolve()
        if not path.is_relative_to(assets):
            raise ValueError('Asset path escapes its directory')
        expected = item.get('sha256', item.get('visual_sha256'))
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Asset hash changed: ' + str(path))
        item[key] = str(path)
    sdf, world = world_base()
    stations = []
    for tree in manifest['trees']:
        model = _node(world, 'model', name=tree['id'])
        _node(model, 'static', 'true')
        link = _node(model, 'link', name='vegetation')
        mesh = _node(_node(_node(link, 'visual', name='tree'), 'geometry'), 'mesh')
        _node(mesh, 'uri', tree['mesh'])
        woody = [o for o in manifest['obstacles']
                 if o['source_object'].startswith(tree['source'])]
        for item in woody:
            _cylinder(link, item)
        trunk = next(o for o in woody if o['source_object'].endswith('/trunk'))
        x, y, _ = trunk['endpoints'][0]
        # Tree-level survey stops do not encode which fruit should be selected.
        stations.append({'id': 'survey_' + tree['id'], 'tree_id': tree['id'],
                         'nav_pose': [x + 1.10, y + .40, -math.pi / 2]})
    for target in manifest['targets']:
        model = ET.fromstring(fruit_sdf(target)).find('model')
        _node(model, 'pose', ' '.join(map(str, (*target['bag_bottom'], 0, 0, 0))))
        world.append(model)
    stations.sort(key=lambda s: s['nav_pose'][1])
    spawn = {'x': 2.0, 'y': -2.5, 'yaw': -math.pi / 2}
    manifest_path = out / 'orchard.manifest.yaml'
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False))
    stations_path = out / 'stations.yaml'
    stations_path.write_text(yaml.safe_dump({'spawn': spawn, 'stations': stations}))
    world_path = out / 'data_orchard.sdf'
    world_path.write_text(ET.tostring(sdf, encoding='unicode'))
    map_path = _map(manifest['obstacles'], out)
    return {'world': world_path, 'manifest': manifest_path, 'map': map_path,
            'stations': stations_path, 'scene_params': default_share() / 'config/orchard.yaml'}
