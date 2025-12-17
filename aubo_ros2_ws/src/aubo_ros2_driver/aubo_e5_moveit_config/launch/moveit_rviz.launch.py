from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Debug flag'
    )
    
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='false',
        description='Use RViz config file'
    )

    # Get paths
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'launch',
        'moveit.rviz'
    ])
    
    kinematics_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'kinematics.yaml'
    ])

    # RViz node with config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{
            'robot_description_kinematics': kinematics_path
        }],
        condition=IfCondition(LaunchConfiguration('config'))
    )

    # RViz node without config (when config is false)
    rviz_node_no_config = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'robot_description_kinematics': kinematics_path
        }],
        condition=UnlessCondition(LaunchConfiguration('config'))
    )

    return LaunchDescription([
        debug_arg,
        config_arg,
        rviz_node,
        rviz_node_no_config,
    ])
