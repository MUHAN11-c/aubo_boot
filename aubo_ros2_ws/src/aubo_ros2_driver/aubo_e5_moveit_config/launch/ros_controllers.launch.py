from launch import LaunchDescription
from launch.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get paths
    ros_controllers_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'ros_controllers.yaml'
    ])

    # Controller spawner
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        respawn=False,
        output='screen',
        namespace='aubo_e5',
        arguments=['manipulator_e5_controller']
    )

    # Robot state publisher for controllers
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        respawn=False,
        output='screen',
        remappings=[('/joint_states', 'aubo_e5/joint_states')]
    )

    return LaunchDescription([
        controller_spawner,
        robot_state_publisher,
    ])

