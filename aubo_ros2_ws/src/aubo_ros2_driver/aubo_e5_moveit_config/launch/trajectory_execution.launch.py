from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    moveit_manage_controllers_arg = DeclareLaunchArgument(
        'moveit_manage_controllers',
        default_value='true',
        description='Flag indicating whether MoveIt! is allowed to load/unload or switch controllers'
    )
    
    moveit_controller_manager_arg = DeclareLaunchArgument(
        'moveit_controller_manager',
        default_value='aubo_e5',
        description='The robot specific controller manager'
    )

    # Controller manager launch
    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                PythonExpression([
                    "'fake_moveit_controller_manager.launch.py' if '", 
                    LaunchConfiguration('moveit_controller_manager'), 
                    "' == 'fake' else 'aubo_e5_moveit_controller_manager.launch.py'"
                ])
            ])
        ])
    )

    return LaunchDescription([
        moveit_manage_controllers_arg,
        moveit_controller_manager_arg,
        controller_manager,
    ])

