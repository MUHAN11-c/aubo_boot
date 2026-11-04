"""Launch MoveIt using exactly the URDF spawned in Gazebo and a simulator policy."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from sim_worlds.params import params_from_dict, xacro_mappings
import xacro
import yaml


def _setup(context):
    """Load shared model/SRDF and keep simulator execution settings local."""
    def value(key):
        return LaunchConfiguration(key).perform(context)

    if value('use_sim_time').lower() != 'true':
        raise RuntimeError('The Gazebo planning frontend requires simulated time')
    bringup = Path(get_package_share_directory('sim_bringup'))
    description = Path(get_package_share_directory('sim_description'))
    model_file = value('robot_description_file')
    if model_file:
        robot_xml = Path(model_file).read_text(encoding='utf-8')
    else:
        data = yaml.safe_load(Path(value('scene_params')).read_text(encoding='utf-8'))
        params, problems = params_from_dict(data)
        if problems:
            raise RuntimeError(f'Invalid scene parameters: {problems}')
        robot_xml = xacro.process_file(
            str(description / 'urdf' / 'harvester_unified.urdf.xacro'),
            mappings=xacro_mappings(params, value('tool_profile'))).toxml()
    builder = MoveItConfigsBuilder('aubo_e5', package_name='aubo_e5_moveit_config')
    moveit = (builder.robot_description_semantic().robot_description_kinematics()
              .joint_limits().planning_pipelines(
                  pipelines=['ompl', 'pilz_industrial_motion_planner'],
                  default_planning_pipeline='ompl')
              .trajectory_execution(
                  file_path=str(bringup / 'config' / 'controllers_sim.yaml'),
                  moveit_manage_controllers=False)
              .planning_scene_monitor(
                  publish_planning_scene=True, publish_geometry_updates=True,
                  publish_state_updates=True, publish_transforms_updates=True,
                  publish_robot_description=True, publish_robot_description_semantic=True)
              .to_moveit_configs())
    config = moveit.to_dict()
    semantic = ET.fromstring(config['robot_description_semantic'])
    known_links = {item.get('name') for item in ET.fromstring(robot_xml).findall('link')}
    for item in list(semantic.findall('disable_collisions')):
        if item.get('link1') not in known_links or item.get('link2') not in known_links:
            semantic.remove(item)
    assembly = Path(get_package_share_directory('sim_description')) / 'config/assembly.srdf'
    semantic.extend(ET.parse(assembly).getroot().findall('disable_collisions'))
    config['robot_description_semantic'] = ET.tostring(semantic, encoding='unicode')
    config['ompl'].setdefault('totg', {})['path_tolerance'] = 0.0005
    config['ompl']['manipulator_e5']['longest_valid_segment_fraction'] = 0.0003
    config['ompl']['response_adapters'].append(
        'default_planning_response_adapters/DisplayMotionPath')
    config['robot_description'] = ParameterValue(robot_xml, value_type=str)
    config.pop('sensors', None)
    config['octomap_frame'] = 'platform_base_link'
    config.update({
        'use_sim_time': True,
        'capabilities': ('pilz_industrial_motion_planner/MoveGroupSequenceAction '
                         'pilz_industrial_motion_planner/MoveGroupSequenceService'),
    })
    config['robot_description_kinematics']['manipulator_e5']['kinematics_solver_timeout'] = 0.1
    nodes = [Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[config],
        on_exit=EmitEvent(event=Shutdown(reason='MoveIt planning server exited')))]
    if value('rviz').lower() == 'true':
        rviz_config = value('rviz_config') or str(bringup / 'config' / 'sim_harvest.rviz')
        nodes.append(Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', rviz_config], parameters=[config]))
    return nodes


def generate_launch_description():
    """Declare standalone MoveIt settings; full bringup passes robot_description_file."""
    worlds = Path(get_package_share_directory('sim_worlds'))
    args = {
        'use_sim_time': 'true', 'robot_description_file': '',
        'scene_params': str(worlds / 'config' / 'orchard.yaml'),
        'tool_profile': 'bite_shear_v1', 'rviz': 'false', 'rviz_config': '',
        'runtime_dir': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=default) for key, default in args.items()],
        OpaqueFunction(function=_setup),
    ])
