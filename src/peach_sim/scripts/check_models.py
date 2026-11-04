#!/usr/bin/env python3
"""Inspect source robot models offline; write evidence without starting ROS nodes."""

import argparse
from functools import lru_cache
import hashlib
from importlib import import_module
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sim_worlds'))
demo = import_module('sim_worlds.demo')
parameters = import_module('sim_worlds.params')

PROFILES = ('shear_v1', 'bite_shear_v1', 'adaptive_shear_v1')
JOINTS = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
          'wrist1_joint', 'wrist2_joint', 'wrist3_joint')


def transform(origin):
    """Read a URDF origin in metres and fixed-axis XYZ radians."""
    result = np.eye(4)
    if origin is not None:
        result[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        result[:3, :3] = Rotation.from_euler(
            'xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return result


def resource(uri):
    """Resolve package resources without downloading or copying dependencies."""
    if uri.startswith(('package://', 'model://')):
        package, relative = uri.split('://', 1)[1].split('/', 1)
        share = ROOT / package if (ROOT / package).is_dir() else Path(
            get_package_share_directory(package))
        return share / relative
    return Path(uri.removeprefix('file://'))


@lru_cache(maxsize=64)
def mesh_vertices(filename):
    """Load mesh coordinates with scene-node transforms applied."""
    scene = trimesh.load(filename, force='scene', process=False)
    return np.asarray(scene.to_geometry().vertices)


def vertices(geometry):
    """Sample exact mesh vertices or a conservative primitive bounding box."""
    mesh = geometry.find('mesh')
    if mesh is not None:
        points = mesh_vertices(str(resource(mesh.get('filename'))))
        return points * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
    box, cylinder, sphere = (geometry.find(tag) for tag in ('box', 'cylinder', 'sphere'))
    if box is not None:
        size = np.fromstring(box.get('size'), sep=' ')
    elif cylinder is not None:
        size = np.array([2 * float(cylinder.get('radius'))] * 2
                        + [float(cylinder.get('length'))])
    elif sphere is not None:
        size = np.full(3, 2 * float(sphere.get('radius')))
    else:
        raise ValueError('Unsupported collision geometry')
    return trimesh.bounds.corners(np.array([-size / 2, size / 2]))


def signature(element):
    """Compare numeric attributes semantically, ignoring XML whitespace."""
    if element is None:
        return None
    attributes = {}
    for key, value in element.attrib.items():
        try:
            attributes[key] = tuple(float(part) for part in value.split())
        except ValueError:
            attributes[key] = value
    return element.tag, attributes, tuple(signature(child) for child in element)


def inspect(profile, params, out):
    """Expand, check and convert one tool profile using the installed ROS tools."""
    problems = []
    robot = ET.fromstring(demo.add_harvest_plugins(xacro.process_file(
        str(ROOT / 'sim_description/urdf/harvester_unified.urdf.xacro'),
        mappings=parameters.xacro_mappings(params, profile)).toxml(), {}))
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = robot.findall('joint')
    children = [joint.find('child').get('link') for joint in joints]
    roots = sorted(set(links) - set(children))
    if roots != ['platform_base_link'] or len(children) != len(set(children)):
        problems.append('Robot must have one platform root and one parent per child')
    if len(links) != len(robot.findall('link')):
        problems.append('Duplicate link names')
    moving = [joint.get('name') for joint in joints if joint.get('type') != 'fixed']
    if moving != list(JOINTS):
        problems.append(f'Unexpected moving joints/order: {moving}')
    frames = {'platform_base_link': np.eye(4)}
    pending = list(joints)
    while pending:
        remaining = []
        for joint in pending:
            parent, child = (joint.find(tag).get('link') for tag in ('parent', 'child'))
            if parent not in links or child not in links:
                raise ValueError(f'Unknown link in {joint.get("name")}')
            if parent not in frames:
                remaining.append(joint)
                continue
            local = transform(joint.find('origin'))
            if joint.get('type') != 'fixed':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                if not np.isclose(np.linalg.norm(axis), 1):
                    problems.append(f'Non-unit axis: {joint.get("name")}')
                state = robot.find(f"ros2_control/joint[@name='{joint.get('name')}']/"
                                   "state_interface[@name='position']/param")
                angle = float(state.text)
                limit = joint.find('limit')
                if not float(limit.get('lower')) <= angle <= float(limit.get('upper')):
                    problems.append(f'Stow outside limit: {joint.get("name")}')
                motion = np.eye(4)
                motion[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
                local = local @ motion
            frames[child] = frames[parent] @ local
        if len(remaining) == len(pending):
            raise ValueError('Disconnected or cyclic robot tree')
        pending = remaining
    source = ET.parse(resource('package://aubo_description/urdf/aubo_e5.urdf')).getroot()
    for original in source.findall('joint'):
        if original.get('name') != 'world_joint' and signature(original) != signature(
                robot.find(f"joint[@name='{original.get('name')}']")):
            problems.append(f'Arm joint source mismatch: {original.get("name")}')
    for original in source.findall('link'):
        local = links.get(original.get('name'))
        if local is not None and signature(original.find('inertial')) != signature(
                local.find('inertial')):
            problems.append(f'Arm inertia source mismatch: {original.get("name")}')
    envelopes = {}
    total_mass = 0.0
    for name, link in links.items():
        for mesh in link.findall('.//mesh'):
            if not resource(mesh.get('filename')).is_file():
                problems.append(f'Missing resource: {mesh.get("filename")}')
        inertial = link.find('inertial')
        if inertial is not None:
            mass = float(inertial.find('mass').get('value'))
            total_mass += mass
            v = {key: float(value) for key, value in inertial.find('inertia').attrib.items()}
            tensor = np.array([[v['ixx'], v['ixy'], v['ixz']],
                               [v['ixy'], v['iyy'], v['iyz']],
                               [v['ixz'], v['iyz'], v['izz']]])
            eigenvalues = np.linalg.eigvalsh(tensor)
            if (mass <= 0 or not np.isfinite(tensor).all() or eigenvalues[0] <= 0
                    or eigenvalues[2] > sum(eigenvalues[:2]) + 1e-12):
                problems.append(f'Nonphysical mass/inertia: {name}')
        clouds = [trimesh.transform_points(
            vertices(c.find('geometry')), frames[name] @ transform(c.find('origin')))
                  for c in link.findall('collision')]
        if clouds:
            points = np.vstack(clouds)
            if not np.isfinite(points).all():
                problems.append(f'Nonfinite collision geometry: {name}')
            envelopes[name] = np.array([points.min(axis=0), points.max(axis=0)]).tolist()
    all_bounds = np.array(list(envelopes.values()))
    bounds = np.array([all_bounds[:, 0].min(axis=0), all_bounds[:, 1].max(axis=0)])
    nav = yaml.safe_load((ROOT / 'sim_bringup/config/nav2_params.yaml').read_text())
    for costmap in ('local_costmap', 'global_costmap'):
        footprint = np.array(json.loads(nav[costmap][costmap]['ros__parameters']['footprint']))
        # The configured footprint is an axis-aligned rectangle; test its full envelope.
        if (np.any(bounds[0, :2] < footprint.min(axis=0))
                or np.any(bounds[1, :2] > footprint.max(axis=0))):
            problems.append(f'{costmap} footprint excludes stowed collision geometry')
    urdf = out / f'{profile}.urdf'
    urdf.write_text(ET.tostring(robot, encoding='unicode'))
    semantic = ET.parse(resource('package://aubo_e5_moveit_config/config/aubo_e5.srdf')).getroot()
    for item in list(semantic.findall('disable_collisions')):
        if item.get('link1') not in links or item.get('link2') not in links:
            semantic.remove(item)
    semantic.extend(ET.parse(ROOT / 'sim_description/config/assembly.srdf').getroot())
    urdf.with_suffix('.srdf').write_text(ET.tostring(semantic, encoding='unicode'))
    conversion = subprocess.run(['gz', 'sdf', '-p', str(urdf)], capture_output=True,
                                text=True, timeout=30, check=True)
    (out / f'{profile}.sdf').write_text(conversion.stdout)
    (out / f'{profile}.conversion.log').write_text(conversion.stderr)
    sdf = ET.fromstring(conversion.stdout)
    sensors = [(link.get('name'), sensor.get('name')) for link in sdf.findall('model/link')
               for sensor in link.findall('sensor')]
    if sorted(sensors) != [('platform_base_link', 'orchard_scan'), ('wrist3_Link', 'wrist_rgbd')]:
        problems.append(f'Sensors lost or reparented: {sensors}')
    return {'problems': problems, 'links': len(links), 'joints': len(joints),
            'mass_kg': total_mass, 'stow_collision_bounds_m': bounds.tolist(),
            'link_collision_bounds_m': envelopes, 'sdf_sensors': sensors}


def inspect_scene(out):
    """Compare actual mesh coordinates with semantic anchors and emitted SDF."""
    assets = import_module('sim_worlds.mesh_geometry')
    mesh = resource(assets.BAG_MESH_URI)
    problems = []
    if hashlib.sha256(mesh.read_bytes()).hexdigest() != assets.BAG_MESH_SHA256:
        problems.append('Bag asset changed: remeasure its bounds and semantic anchors')
    cloud = mesh_vertices(str(mesh))
    if not (np.allclose(cloud.min(axis=0), assets.BAG_RAW_MIN, atol=1e-9)
            and np.allclose(cloud.max(axis=0), assets.BAG_RAW_MAX, atol=1e-9)):
        problems.append('Bag embedded geometry disagrees with measured metadata')
    files = demo.prepare_demo(out / 'demo')
    manifest = yaml.safe_load(files['manifest'].read_text())
    world = ET.parse(files['world']).getroot().find('world')
    for target in manifest['targets']:
        geometry = assets.bag_geometry(target['body_diameter'], target['body_thickness'],
                                       target['bottom_to_neck'])
        pose = geometry['visual_pose']
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_euler('xyz', pose[3:]).as_matrix()
        matrix[:3, 3] = pose[:3]
        transformed = trimesh.transform_points(cloud * geometry['scale'], matrix)
        for key, actual in [('min', transformed.min(axis=0)),
                            ('max', transformed.max(axis=0))]:
            if not np.allclose(actual, geometry['visual_bounds'][key], atol=1e-9):
                problems.append(f'Visual envelope mismatch: {target["id"]}/{key}')
        neck = matrix @ np.array([0, assets.BAG_NECK_AXIS * geometry['scale'][1], 0, 1])
        if not np.allclose(neck[:3], geometry['neck_center'], atol=1e-9):
            problems.append(f'Neck anchor mismatch: {target["id"]}')
        model = world.find(f"model[@name='{target['id']}']")
        if model is None or len(model.findall('link/visual')) != 1:
            problems.append(f'Missing/duplicated bag visual: {target["id"]}')
        dynamic = ET.fromstring(demo.fruit_sdf(target, dynamic=True)).find('model/link')
        center = np.fromstring(dynamic.findtext('inertial/pose'), sep=' ')[:3]
        if not np.allclose(center, geometry['collision_center']):
            problems.append(f'Rigid proxy COM mismatch: {target["id"]}')
        box = np.fromstring(dynamic.findtext('collision/geometry/box/size'), sep=' ')
        if not np.allclose(box, target['collision_size']):
            problems.append(f'Planner/Gazebo proxy mismatch: {target["id"]}')
    return {'problems': problems, 'targets_checked': len(manifest['targets']),
            'bag_asset_sha256': assets.BAG_MESH_SHA256,
            'bag_geometry_m': assets.bag_geometry(.075, .06, .09)}


def main():
    """Store reproducible source-model evidence entirely within this project."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / '.runtime/model-audit/current')
    args = parser.parse_args()
    out = args.output.resolve()
    if not out.is_relative_to(ROOT):
        parser.error('output must remain inside peach_sim')
    out.mkdir(parents=True, exist_ok=True)
    params, errors = parameters.params_from_dict(yaml.safe_load(
        (ROOT / 'sim_worlds/config/orchard.yaml').read_text()))
    if errors:
        raise ValueError(errors)
    result = {profile: inspect(profile, params, out) for profile in PROFILES}
    result['scene'] = inspect_scene(out)
    (out / 'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    for profile, report in result.items():
        print(profile, json.dumps({k: v for k, v in report.items()
                                   if k != 'link_collision_bounds_m'}, ensure_ascii=False))
    return int(any(report['problems'] for report in result.values()))


if __name__ == '__main__':
    raise SystemExit(main())
