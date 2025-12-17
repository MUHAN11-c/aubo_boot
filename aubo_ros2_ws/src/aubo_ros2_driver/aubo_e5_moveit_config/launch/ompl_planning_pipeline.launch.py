from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    start_state_max_bounds_error_arg = DeclareLaunchArgument(
        'start_state_max_bounds_error',
        default_value='0.1',
        description='Start state max bounds error'
    )

    # Get paths
    ompl_planning_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'ompl_planning.yaml'
    ])

    # In ROS2 MoveIt2, OMPL planning parameters are loaded by move_group node
    # The planning_plugin, request_adapters, and ompl_planning.yaml
    # are configured in move_group.launch.py
    return LaunchDescription([
        start_state_max_bounds_error_arg,
        # Note: OMPL parameters are passed to move_group node
    ])

