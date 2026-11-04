"""Compose the navigation, manipulation, perception and visualization simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from sim_worlds.params import params_from_dict
from sim_worlds.scene import work_pose
from sim_worlds.tour_core import stations as orchard_stations
import yaml


def _runtime_default():
    """Keep generated maps, models and launch configuration inside this project."""
    for directory in Path(__file__).resolve().parents:
        if (directory / 'sim_description' / 'package.xml').is_file():
            return str(directory / '.runtime' / 'run')
    raise RuntimeError('Set runtime_dir explicitly for a relocated installation')


def _include(share, filename, arguments):
    """Include a package launch file with explicit arguments."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / filename)),
        launch_arguments=arguments.items())


def _orchard_resources(worlds, runtime):
    """Build a task station list matching the procedural world target IDs."""
    scene_params = worlds / 'config' / 'orchard.yaml'
    params, problems = params_from_dict(yaml.safe_load(scene_params.read_text()))
    if problems:
        raise RuntimeError(f'Invalid orchard parameters: {problems}')
    manifest_file = worlds / 'worlds' / 'peach_orchard.manifest.yaml'
    manifest = yaml.safe_load(manifest_file.read_text())
    targets_by_tree = {}
    for target in manifest['targets']:
        targets_by_tree.setdefault(target['tree'], []).append(target)
    waypoints = []
    for station in orchard_stations(params):
        if station.index < 0:
            continue
        for target in targets_by_tree.get(station.tree_id, []):
            waypoints.append({
                'id': station.tree_id, 'target_id': target['id'],
                'nav_pose': [station.x, station.y, station.yaw]})
    work = work_pose(params)
    spawn = {'x': work.vehicle_center[0] + params.rows.spacing / 2.0 - params.platform.row_gap,
             'y': work.vehicle_center[1], 'yaw': work.vehicle_yaw}
    stations_file = runtime / 'orchard_stations.yaml'
    stations_file.write_text(yaml.safe_dump({'spawn': spawn, 'stations': waypoints}))
    return {
        'world': worlds / 'worlds' / 'peach_orchard.sdf', 'manifest': manifest_file,
        'map': worlds / 'maps' / 'orchard_map.yaml', 'stations': stations_file,
        'scene_params': scene_params,
    }


def _setup(context):
    """Choose one coherent scene preset and compose the standard ROS stacks."""
    def value(key):
        return LaunchConfiguration(key).perform(context)

    if value('use_sim_time').lower() != 'true':
        raise RuntimeError('The complete simulation requires use_sim_time:=true')
    bringup = Path(get_package_share_directory('sim_bringup'))
    worlds = Path(get_package_share_directory('sim_worlds'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    runtime = Path(value('runtime_dir')).expanduser().resolve()
    runtime.mkdir(parents=True, exist_ok=True)
    scene = value('scene')
    if value('field'):
        scene = 'field' if value('field').lower() == 'true' else 'orchard'
    if scene == 'demo':
        from sim_worlds.demo import prepare_demo
        resources = prepare_demo(runtime / 'scene')
    elif scene == 'data_orchard':
        from sim_worlds.data_orchard import prepare_data_orchard
        resources = prepare_data_orchard(runtime / 'scene')
    elif scene == 'orchard':
        resources = _orchard_resources(worlds, runtime)
    elif scene == 'field':
        resources = {
            'world': worlds / 'worlds' / 'field_no_grass.sdf',
            'map': worlds / 'maps' / 'field_map.yaml',
            'stations': worlds / 'maps' / 'field_stations.yaml',
            'scene_params': worlds / 'config' / 'orchard.yaml', 'manifest': '',
        }
    else:
        raise RuntimeError('scene must be data_orchard, demo, orchard or field')
    for key, arg in [('world', 'world'), ('map', 'map'), ('manifest', 'manifest_file'),
                     ('stations', 'stations_file'), ('scene_params', 'scene_params')]:
        if value(arg):
            resources[key] = Path(value(arg)).expanduser().resolve()
    for key in ('world', 'map', 'stations', 'scene_params'):
        if not Path(resources[key]).is_file():
            raise RuntimeError(f'Missing {scene} {key} resource: {resources[key]}')
    station_data = yaml.safe_load(Path(resources['stations']).read_text(encoding='utf-8'))
    spawn = station_data['spawn']
    nav_file = value('nav2_params') or str(bringup / 'config' / (
        'nav2_params_field.yaml' if scene == 'field' else 'nav2_params.yaml'))
    nav_config = yaml.safe_load(Path(nav_file).read_text(encoding='utf-8'))
    nav_config['amcl']['ros__parameters']['initial_pose'] = {
        'x': float(spawn['x']), 'y': float(spawn['y']),
        'z': 0.0, 'yaw': float(spawn.get('yaw', 0.0))}
    if scene in ('demo', 'data_orchard'):
        checker = nav_config['controller_server']['ros__parameters']['general_goal_checker']
        checker.update(xy_goal_tolerance=0.06, yaw_goal_tolerance=0.08)
        nav_config['controller_server']['ros__parameters']['FollowPath']['visualize'] = False
    for node in ('controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor'):
        nav_config[node]['ros__parameters']['enable_stamped_cmd_vel'] = False
    nav_config['collision_monitor']['ros__parameters']['cmd_vel_in_topic'] = 'cmd_vel_interlocked'
    runtime_nav = runtime / 'nav2.yaml'
    runtime_nav.write_text(yaml.safe_dump(nav_config, sort_keys=False), encoding='utf-8')
    common = {'use_sim_time': 'true', 'runtime_dir': str(runtime)}
    sim_args = {
        **common, 'world': str(resources['world']), 'scene_params': str(resources['scene_params']),
        'manifest_file': str(resources['manifest']), 'tool_profile': value('tool_profile'),
        'spawn_x': str(spawn['x']), 'spawn_y': str(spawn['y']),
        'spawn_yaw': str(spawn.get('yaw', 0.0)), 'gui': value('gui'),
        'moveit': value('moveit'), 'rviz': value('rviz'),
        'rviz_config': str(bringup / 'config' / 'sim_harvest.rviz'),
    }
    nodes = [
        SetEnvironmentVariable('PEACH_SIM_RUN_DIRECTORY', str(runtime)),
        LogInfo(msg=f'Joint simulation scene={scene}; runtime={runtime}'),
        Node(package='sim_harvest', executable='base_guard', output='screen',
             parameters=[{'use_sim_time': True, 'require_arm_lock': bool(resources['manifest'])}],
             on_exit=EmitEvent(event=Shutdown(reason='Base interlock exited'))),
        _include(bringup, 'sim.launch.py', sim_args),
        _include(nav2, 'bringup_launch.py', {
            'slam': str(value('slam').lower() == 'true'), 'map': str(resources['map']),
            'params_file': str(runtime_nav), 'use_sim_time': 'true',
            'autostart': 'true', 'use_composition': 'False', 'use_respawn': 'False',
        }),
    ]
    if value('perception') == 'vision':
        nodes.append(_include(bringup, 'perception.launch.py', {
            **common, 'tool_profile': value('tool_profile')}))
    elif value('perception') != 'ground_truth':
        raise RuntimeError('perception must be ground_truth or vision')
    if resources['manifest']:
        if value('moveit').lower() != 'true':
            raise RuntimeError('Harvest manipulation requires moveit:=true')
        task_params = {
            'use_sim_time': True, 'manifest_file': str(resources['manifest']),
            'stations_file': str(resources['stations']), 'run_directory': str(runtime),
            'robot_description_file': str(runtime / 'robot.urdf'),
            'perception_mode': value('perception'),
            'tool_profile': value('tool_profile'),
        }
        nodes.extend([
            Node(package='sim_harvest', executable='grasp_node', output='screen',
                 parameters=[task_params],
                 on_exit=EmitEvent(event=Shutdown(reason='Simulator grasp server exited'))),
            Node(package='sim_harvest', executable='mission_node', output='screen',
                 parameters=[task_params, {
                     'autostart': value('autostart').lower() == 'true',
                     'max_targets': int(value('max_targets')),
                     'perception_mode': value('perception'),
                 }], on_exit=EmitEvent(event=Shutdown(reason='Simulator mission server exited'))),
        ])
    else:
        if value('autostart').lower() == 'true':
            raise RuntimeError('field preview has no independently detachable target manifest')
        nodes.append(LogInfo(msg=(
            'Field scene is navigation/visualization only: its whole-orchard GLB has no '
            'independently detachable fruit models. Use scene:=demo for physical pickup.')))
    return nodes


def generate_launch_description():
    """Declare the complete simulation deployment configuration."""
    args = {
        'scene': 'data_orchard', 'field': '', 'use_sim_time': 'true',
        'runtime_dir': _runtime_default(), 'world': '', 'map': '',
        'manifest_file': '', 'stations_file': '', 'scene_params': '', 'nav2_params': '',
        'slam': 'false', 'gui': 'false', 'moveit': 'true', 'rviz': 'true',
        'tool_profile': 'adaptive_shear_v1', 'perception': 'vision',
        'autostart': 'false', 'max_targets': '1',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=default) for key, default in args.items()],
        OpaqueFunction(function=_setup),
    ])
