from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    moveit_controller_manager_arg = DeclareLaunchArgument(
        'moveit_controller_manager',
        default_value='moveit_simple_controller_manager/MoveItSimpleControllerManager',
        description='The param that trajectory_execution_manager needs to find the controller plugin'
    )

    # Get paths
    controllers_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'controllers.yaml'
    ])

    # Controller manager node (parameters will be loaded by move_group)
    # In ROS2, these parameters are typically passed to move_group node
    return LaunchDescription([
        moveit_controller_manager_arg,
        # Note: The moveit_controller_manager parameter and controller_list
        # are loaded by the move_group node in move_group.launch.py
    ])

