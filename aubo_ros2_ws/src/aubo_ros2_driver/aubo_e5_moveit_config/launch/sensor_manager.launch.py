from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    moveit_sensor_manager_arg = DeclareLaunchArgument(
        'moveit_sensor_manager',
        default_value='aubo_e5',
        description='Load the robot specific sensor manager'
    )

    # Get paths
    sensors_3d_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'sensors_3d.yaml'
    ])

    # Sensor manager launch
    sensor_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                PythonExpression([
                    "'", LaunchConfiguration('moveit_sensor_manager'), "_moveit_sensor_manager.launch.py'"
                ])
            ])
        ])
    )

    return LaunchDescription([
        moveit_sensor_manager_arg,
        sensor_manager_launch,
    ])

