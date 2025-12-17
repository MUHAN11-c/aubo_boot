from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get paths
    fake_controllers_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'fake_controllers.yaml'
    ])

    # In ROS2, the moveit_controller_manager parameter and fake_controllers
    # are loaded by the move_group node
    return LaunchDescription([
        # Note: Parameters are passed to move_group node
    ])

