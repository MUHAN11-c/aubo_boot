from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    moveit_warehouse_database_path_arg = DeclareLaunchArgument(
        'moveit_warehouse_database_path',
        description='The path to the database must be specified'
    )

    # Warehouse settings
    warehouse_settings = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                'warehouse_settings.launch.py'
            ])
        ])
    )

    # MongoDB wrapper
    mongo_wrapper = Node(
        package='warehouse_ros_mongo',
        executable='mongo_wrapper_ros.py',
        name='mongo_wrapper_ros',
        output='screen',
        parameters=[{
            'overwrite': False,
            'database_path': LaunchConfiguration('moveit_warehouse_database_path')
        }]
    )

    return LaunchDescription([
        moveit_warehouse_database_path_arg,
        warehouse_settings,
        mongo_wrapper,
    ])

