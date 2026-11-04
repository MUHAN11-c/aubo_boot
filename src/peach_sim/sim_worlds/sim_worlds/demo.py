"""Generate a small, self-contained harvesting world and native grasp plugins."""

from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw
import yaml

from .cli import default_share
from .mesh_geometry import bag_geometry, BAG_MESH_URI

WORLD = 'peach_orchard'


def _node(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def _box(parent, name, xyz, size, color, collision=True):
    visual = _node(parent, 'visual', name=name)
    _node(visual, 'pose', ' '.join(map(str, (*xyz, 0, 0, 0))))
    _node(_node(_node(visual, 'geometry'), 'box'), 'size', ' '.join(map(str, size)))
    material = _node(visual, 'material')
    for key in ('ambient', 'diffuse'):
        _node(material, key, ' '.join(map(str, (*color, 1))))
    if collision:
        item = deepcopy(visual)
        item.tag = 'collision'
        item.remove(item.find('material'))
        parent.append(item)


def _pose_publisher(model):
    """Publish named model poses; SceneBroadcaster poses lack bridge frame metadata."""
    plugin = _node(model, 'plugin', name='gz::sim::systems::PosePublisher',
                   filename='gz-sim-pose-publisher-system')
    for tag, value in {'publish_model_pose': 'true', 'publish_link_pose': 'false',
                       'publish_nested_model_pose': 'false', 'use_pose_vector_msg': 'true',
                       'update_frequency': '30', 'topic': '/sim_harvest/poses'}.items():
        _node(plugin, tag, value)


def fruit_sdf(target, dynamic=False):
    """Return a rigid bag proxy; the cut creates a gravity-driven copy."""
    sdf = ET.Element('sdf', version='1.9')
    model = _node(sdf, 'model', name=target['id'] + ('_picked' if dynamic else ''))
    _node(model, 'static', 'false' if dynamic else 'true')
    if dynamic:
        _pose_publisher(model)
    link = _node(model, 'link', name='bag')
    geometry = ({'collision_center': target['collision_center_local'],
                 'collision_size': target['collision_size'],
                 'visual_pose': target['visual_pose'], 'scale': target['visual_scale']}
                if target.get('visual_mesh') else bag_geometry(
                    target['body_diameter'], target['body_thickness'], target['bottom_to_neck']))
    center, size = geometry['collision_center'], geometry['collision_size']
    inertial = _node(link, 'inertial')
    _node(inertial, 'pose', ' '.join(map(str, (*center, 0, 0, 0))))
    mass = float(target.get('mass_kg', 0.12))
    _node(inertial, 'mass', mass)
    inertia = _node(inertial, 'inertia')
    x, y, z = size
    for key, value in [('ixx', mass * (y*y + z*z) / 12),
                       ('iyy', mass * (x*x + z*z) / 12),
                       ('izz', mass * (x*x + y*y) / 12),
                       ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        _node(inertia, key, value)
    _box(link, 'paper_bag', center, size,
         (0.66, 0.44, 0.30), collision=dynamic or bool(target.get('visual_mesh')))
    visual = link.find("visual[@name='paper_bag']")
    if target.get('visual_mesh'):
        visual.remove(visual.find('material'))
    visual.find('pose').text = ' '.join(map(str, geometry['visual_pose']))
    visual.find('geometry').clear()
    mesh = _node(visual.find('geometry'), 'mesh')
    _node(mesh, 'uri', target.get('visual_mesh', BAG_MESH_URI))
    _node(mesh, 'scale', ' '.join(map(str, geometry['scale'])))
    if link.find('collision') is not None:
        surface = _node(link.find('collision'), 'surface')
        _node(_node(surface, 'contact'), 'collide_bitmask', '2')
    return ET.tostring(sdf, encoding='unicode')


def world_base():
    """Create the common simulator systems and finite soil plane."""
    sdf = ET.Element('sdf', version='1.9')
    world = _node(sdf, 'world', name=WORLD)
    physics = _node(world, 'physics', name='default', type='ignored')
    _node(physics, 'max_step_size', '0.002')
    _node(physics, 'real_time_factor', '1.0')
    for system in ('Physics', 'UserCommands', 'SceneBroadcaster', 'Sensors', 'Contact'):
        filename = {'Physics': 'physics', 'UserCommands': 'user-commands',
                    'SceneBroadcaster': 'scene-broadcaster', 'Sensors': 'sensors',
                    'Contact': 'contact'}[system]
        plugin = _node(world, 'plugin', name=f'gz::sim::systems::{system}',
                       filename=f'gz-sim-{filename}-system')
        if system == 'Sensors':
            _node(plugin, 'render_engine', 'ogre2')
    scene = _node(world, 'scene')
    _node(scene, 'ambient', '0.65 0.65 0.65 1')
    _node(scene, 'background', '0.65 0.78 0.9 1')
    light = _node(world, 'light', name='sun', type='directional')
    _node(light, 'direction', '-0.4 0.2 -0.9')
    _node(light, 'diffuse', '0.9 0.9 0.9 1')
    _node(light, 'cast_shadows', 'true')
    ground = _node(world, 'model', name='ground')
    _node(ground, 'static', 'true')
    _box(_node(ground, 'link', name='ground'), 'soil', (0, 0, -0.05),
         (12, 12, 0.1), (0.38, 0.42, 0.30))
    return sdf, world


def prepare_demo(output_directory):
    """Write world, map, targets and stations from one layout description."""
    out = Path(output_directory).resolve()
    out.mkdir(parents=True, exist_ok=True)
    sdf, world = world_base()
    targets, stations, obstacles = [], [], []
    for index, y in enumerate((-0.8, 1.4)):
        tree = _node(world, 'model', name=f'tree_{index}')
        _node(tree, 'static', 'true')
        link = _node(tree, 'link', name='trunk')
        _box(link, 'trunk', (-0.25, y, 0.70), (0.14, 0.14, 1.4), (0.3, 0.19, 0.1))
        _box(link, 'branch', (-0.05, y, 1.18), (0.45, 0.04, 0.04),
             (0.38, 0.24, 0.12), collision=False)
        _box(link, 'leaves', (-0.35, y + 0.2, 1.5), (0.6, 0.45, 0.35),
             (0.20, 0.40, 0.16), collision=False)
        obstacles.append({'id': f'tree_{index}', 'position': [-0.25, y, 0.70],
                          'size': [0.14, 0.14, 1.4]})
        target = {'id': f'peach_demo_{index}', 'tree': f'tree_{index}',
                  'bag_bottom': [0.10, y, 1.06], 'entry': [0.10, y, 1.06],
                  'bag_neck': [0.10, y, 1.15], 'axis': [0.0, 0.0, 1.0],
                  'fruit_center': [0.10, y, 1.10], 'body_diameter': 0.075,
                  'body_thickness': 0.06, 'bottom_to_neck': 0.09,
                  'fruit_diameter': 0.055, 'sdf_pose': [0.10, y, 1.06, 0, 0, 0],
                  'reachable': True}
        geometry = bag_geometry(target['body_diameter'], target['body_thickness'],
                                target['bottom_to_neck'])
        for key in ('fruit_center', 'collision_center'):
            target[key] = [a + b for a, b in zip(target['bag_bottom'], geometry[key])]
        target['collision_center_local'] = list(geometry['collision_center'])
        target['collision_size'] = list(geometry['collision_size'])
        target['fruit_ellipsoid_size'] = list(geometry['fruit_ellipsoid_size'])
        # The embedded fruit is an ellipsoid after nonuniform bag scaling.
        target['fruit_diameter'] = min(geometry['fruit_ellipsoid_size'])
        model = ET.fromstring(fruit_sdf(target)).find('model')
        _node(model, 'pose', f'0.10 {y} 1.06 0 0 0')
        world.append(model)
        targets.append(target)
        stations.append({'id': f'station_{index}', 'target_id': target['id'],
                         'nav_pose': [0.70, y, -math.pi / 2]})
    # Asymmetric, low landmarks give AMCL an observable scan/map match.
    for index, (x, y, sx, sy) in enumerate(((-2, -2, .3, .3), (3, 2, .3, 1),
                                           (-2, 3, .6, .3), (3, -3, .4, .4))):
        model = _node(world, 'model', name=f'landmark_{index}')
        _node(model, 'static', 'true')
        _box(_node(model, 'link', name='body'), 'post', (x, y, 0.5),
             (sx, sy, 1), (0.55, 0.55, 0.5))
        obstacles.append({'id': f'landmark_{index}', 'position': [x, y, 0.5],
                          'size': [sx, sy, 1.0]})
    world_path = out / 'demo.sdf'
    world_path.write_text(ET.tostring(sdf, encoding='unicode'), encoding='utf-8')
    manifest = {'schema': 'sim_worlds/harvest_demo/v1', 'world_name': WORLD,
                'frame': 'map', 'targets': targets, 'obstacles': obstacles}
    manifest_path = out / 'demo.manifest.yaml'
    manifest_path.write_text(yaml.safe_dump(manifest), encoding='utf-8')
    station_path = out / 'stations.yaml'
    station_path.write_text(yaml.safe_dump({
        'spawn': {'x': 1.7, 'y': -2.3, 'yaw': -math.pi / 2},
        'stations': stations}), encoding='utf-8')
    image = Image.new('L', (240, 240), 254)
    draw = ImageDraw.Draw(image)
    for item in obstacles:
        x, y, _ = item['position']
        sx, sy, _ = item['size']
        draw.rectangle(((x - sx / 2 + 6) / .05, (6 - y - sy / 2) / .05,
                        (x + sx / 2 + 6) / .05, (6 - y + sy / 2) / .05), fill=0)
    # Mark the finite ground edge as non-traversable, as in the orchard map.
    draw.rectangle((0, 0, image.width - 1, image.height - 1), outline=0, width=4)
    image.save(out / 'map.pgm')
    map_path = out / 'map.yaml'
    map_path.write_text(yaml.safe_dump({
        'image': 'map.pgm', 'mode': 'trinary', 'resolution': 0.05,
        'origin': [-6.0, -6.0, 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196}), encoding='utf-8')
    return {'world': world_path, 'manifest': manifest_path,
            'map': map_path, 'stations': station_path,
            'scene_params': default_share() / 'config' / 'orchard.yaml'}


def add_harvest_plugins(robot_xml, manifest):
    """Add native detachable joints; absent dynamic fruit initially stays absent."""
    robot = ET.fromstring(robot_xml)
    _pose_publisher(_node(robot, 'gazebo'))
    from ament_index_python.packages import get_package_share_directory
    srdf = Path(get_package_share_directory('aubo_e5_moveit_config')) / 'config/aubo_e5.srdf'
    state = ET.parse(srdf).find(".//group_state[@name='harvest_stow']")
    if state is None:
        raise ValueError('SRDF has no harvest_stow')
    positions = {j.get('name'): j.get('value') for j in state.findall('joint')}
    for joint in robot.findall('.//ros2_control/joint'):
        position = joint.find("state_interface[@name='position']")
        if position is not None:
            _node(position, 'param', positions[joint.get('name')], name='initial_value')
    for target in manifest.get('targets', []):
        key = target['id']
        plugin = _node(_node(robot, 'gazebo'), 'plugin',
                       filename='gz-sim-detachable-joint-system',
                       name='gz::sim::systems::DetachableJoint')
        for tag, value in {
                'parent_link': 'wrist3_Link', 'child_model': key + '_picked',
                'child_link': 'bag', 'suppress_child_warning': 'true',
                'attach_topic': f'/sim_harvest/{key}/attach',
                'detach_topic': f'/sim_harvest/{key}/detach',
                'output_topic': f'/sim_harvest/{key}/state'}.items():
            _node(plugin, tag, value)
    return ET.tostring(robot, encoding='unicode')
