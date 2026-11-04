"""Peach scene-obstacle snapshot node (plain Node, not lifecycle-managed)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('peach_scene_obstacles'), 'config', 'scene_obstacles.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='peach_scene_obstacles',
            executable='peach_scene_obstacles',
            name='peach_scene_obstacles',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                ParameterFile(params, allow_substs=True),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
