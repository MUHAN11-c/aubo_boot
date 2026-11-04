"""Launch one isolated Gazebo robot and activate controllers after successful spawn."""

from __future__ import annotations

import os
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from sim_worlds.params import params_from_dict, xacro_mappings
from sim_worlds.scene import PLATFORM_MODEL, work_pose
import xacro
import yaml


def _runtime_default():
    """Locate the project-local runtime directory from source or local install."""
    for directory in Path(__file__).resolve().parents:
        if (directory / 'sim_description' / 'package.xml').is_file():
            return str(directory / '.runtime' / 'run')
    raise RuntimeError('Set runtime_dir explicitly for a relocated installation')


def _checked_exit(next_actions, label):
    """Continue the launch only when the prerequisite process exits successfully."""
    def callback(event, context):
        del context
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason=f'{label} failed with exit code {event.returncode}'))]
        return next_actions
    return callback


def _setup(context):
    """Expand one robot model, bridge its simulator interfaces, then spawn it."""
    def value(key):
        return LaunchConfiguration(key).perform(context)

    if value('use_sim_time').lower() != 'true':
        raise RuntimeError('Gazebo requires use_sim_time:=true throughout the graph')
    runtime = Path(value('runtime_dir')).expanduser().resolve()
    runtime.mkdir(parents=True, exist_ok=True)
    desc = Path(get_package_share_directory('sim_description'))
    aubo = Path(get_package_share_directory('aubo_description'))
    bringup = Path(get_package_share_directory('sim_bringup'))
    scene_path = Path(value('scene_params')).resolve()
    world = Path(value('world')).resolve()
    if not world.is_file():
        raise RuntimeError(f'World file does not exist: {world}')
    raw = yaml.safe_load(scene_path.read_text(encoding='utf-8'))
    params, problems = params_from_dict(raw if isinstance(raw, dict) else {})
    if problems:
        raise RuntimeError(f'Invalid scene configuration: {problems}')
    tool_profile = value('tool_profile')
    robot_xml = xacro.process_file(
        str(desc / 'urdf' / 'harvester_unified.urdf.xacro'),
        mappings=xacro_mappings(params, tool_profile)).toxml()
    manifest = {}
    if value('manifest_file'):
        manifest = yaml.safe_load(Path(value('manifest_file')).read_text(encoding='utf-8'))
    from sim_worlds.demo import add_harvest_plugins
    robot_xml = add_harvest_plugins(robot_xml, manifest)
    robot_file = runtime / 'robot.urdf'
    robot_file.write_text(robot_xml, encoding='utf-8')
    # The fixed shear mesh has no actuated jaws. Native fixed attachment models
    # retention; permit release through that mesh while keeping basket collisions.
    converted = subprocess.run(['gz', 'sdf', '-p', str(robot_file)],
                               check=True, capture_output=True, text=True, timeout=30)
    sdf = ET.fromstring(converted.stdout)
    for link in sdf.findall('model/link'):
        basket = [c.get('name') for c in link.findall('collision')
                  if 'harvest_basket' in c.get('name', '')]
        if basket:
            sensor = ET.SubElement(link, 'sensor', name='basket_contacts', type='contact')
            contact = ET.SubElement(sensor, 'contact')
            for name in basket:
                ET.SubElement(contact, 'collision').text = name
            ET.SubElement(contact, 'topic').text = '/sim_harvest/basket_contacts'
    # Contact-sensor <collision> entries contain names, not geometry. Never
    # append surfaces to those references: that makes the model fail DOM parsing.
    for collision in sdf.findall('model/link/collision'):
        name = collision.get('name', '')
        mask = '1' if any(part in name for part in (
            'tool_body_link', 'sleeve_mouth', 'cutting_plane')) else '3'
        surface = collision.find('surface')
        if surface is None:
            surface = ET.SubElement(collision, 'surface')
        contact = surface.find('contact')
        if contact is None:
            contact = ET.SubElement(surface, 'contact')
        bitmask = contact.find('collide_bitmask')
        if bitmask is None:
            bitmask = ET.SubElement(contact, 'collide_bitmask')
        bitmask.text = mask
    robot_sdf = runtime / 'robot.sdf'
    robot_sdf.write_text(ET.tostring(sdf, encoding='unicode'), encoding='utf-8')

    bridges = yaml.safe_load((bringup / 'config' / 'sim_bridge.yaml').read_text())
    for target in manifest.get('targets', []):
        prefix = f"/sim_harvest/{target['id']}"
        for operation in ('attach', 'detach'):
            bridges.append({
                'ros_topic_name': f'{prefix}/{operation}',
                'gz_topic_name': f'{prefix}/{operation}',
                'ros_type_name': 'std_msgs/msg/Empty', 'gz_type_name': 'gz.msgs.Empty',
                'direction': 'ROS_TO_GZ'})
        bridges.append({
            'ros_topic_name': f'{prefix}/state', 'gz_topic_name': f'{prefix}/state',
            'ros_type_name': 'std_msgs/msg/String', 'gz_type_name': 'gz.msgs.StringMsg',
            'direction': 'GZ_TO_ROS'})
    bridge_file = runtime / 'bridge.yaml'
    bridge_file.write_text(yaml.safe_dump(bridges, sort_keys=False), encoding='utf-8')

    pose = work_pose(params)
    if bool(value('spawn_x')) != bool(value('spawn_y')):
        raise RuntimeError('spawn_x and spawn_y must be supplied together')
    if value('spawn_x'):
        spawn = [float(value('spawn_x')), float(value('spawn_y')),
                 float(value('spawn_yaw') or 0.0)]
    else:
        x = pose.vehicle_center[0]
        if value('spawn') == 'corridor':
            x += params.rows.spacing / 2.0 - params.platform.row_gap
        spawn = [x, pose.vehicle_center[1], pose.vehicle_yaw]
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', name='spawn_harvester', output='screen',
        arguments=['-world', 'peach_orchard', '-name', PLATFORM_MODEL,
                   '-file', str(robot_sdf), '-x', str(spawn[0]), '-y', str(spawn[1]),
                   '-z', '0.0', '-Y', str(spawn[2])])
    jsb = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120',
                   '--service-call-timeout', '60', '--switch-timeout', '30'])
    jtc = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_trajectory_controller', '--controller-manager-timeout', '120',
                   '--service-call-timeout', '60', '--switch-timeout', '30'])
    after_controllers = [LogInfo(msg='Simulator joint controllers are active')]
    if value('moveit').lower() == 'true':
        after_controllers.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch' / 'moveit_sim.launch.py')),
            launch_arguments={
                'use_sim_time': 'true', 'scene_params': str(scene_path),
                'robot_description_file': str(robot_file), 'tool_profile': tool_profile,
                'rviz': value('rviz'), 'rviz_config': value('rviz_config'),
                'runtime_dir': str(runtime),
            }.items()))
    elif value('rviz').lower() == 'true':
        after_controllers.append(Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', value('rviz_config')] if value('rviz_config') else [],
            parameters=[{'use_sim_time': True, 'robot_description': robot_xml}]))
    gz_args = ['-r', '-v', '2']
    if value('gui').lower() != 'true':
        gz_args.append('-s')
    gz_args.append(str(world))
    resource_dirs = [str(aubo.parent), str(desc.parent)]
    resource_dirs.extend(filter(
        None, os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(os.pathsep)))
    return [
        SetParameter('use_sim_time', True),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.pathsep.join(dict.fromkeys(resource_dirs))),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot, on_exit=_checked_exit([jsb], 'Robot spawn'))),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb, on_exit=_checked_exit([jtc], 'Joint-state broadcaster'))),
        RegisterEventHandler(OnProcessExit(
            target_action=jtc, on_exit=_checked_exit(after_controllers, 'Trajectory controller'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')),
            launch_arguments={'gz_args': shlex.join(gz_args), 'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_xml, 'use_sim_time': True}],
             on_exit=EmitEvent(event=Shutdown(reason='robot_state_publisher exited'))),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sim_bridge',
             output='screen', parameters=[{'config_file': str(bridge_file)}],
             on_exit=EmitEvent(event=Shutdown(reason='Gazebo bridge exited'))),
        spawn_robot,
    ]


def generate_launch_description():
    """Declare the standalone simulator interface."""
    worlds = Path(get_package_share_directory('sim_worlds'))
    args = {
        'use_sim_time': 'true', 'runtime_dir': _runtime_default(),
        'scene_params': str(worlds / 'config' / 'orchard.yaml'),
        'world': str(worlds / 'worlds' / 'peach_orchard.sdf'), 'manifest_file': '',
        'gui': 'false', 'moveit': 'true', 'rviz': 'false', 'rviz_config': '',
        'spawn': 'work', 'spawn_x': '', 'spawn_y': '', 'spawn_yaw': '',
        'tool_profile': 'bite_shear_v1',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=default) for key, default in args.items()],
        OpaqueFunction(function=_setup),
    ])
