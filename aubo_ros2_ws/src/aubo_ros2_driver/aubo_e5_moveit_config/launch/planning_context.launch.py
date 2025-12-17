from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare arguments
    load_robot_description_arg = DeclareLaunchArgument(
        'load_robot_description',
        default_value='false',
        description='By default we do not overwrite the URDF. Change the following to true to change the default behavior'
    )
    
    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value='robot_description',
        description='The name of the parameter under which the URDF is loaded'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='None',
        description='Robot IP address (required if sim=false)'
    )

    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('aubo_description_ros2'),
                'urdf',
                'aubo_e5.xacro'
            ]),
            ' robot_ip:=',
            LaunchConfiguration('robot_ip')
        ]),
        value_type=str
    )

    # Robot description semantic path
    robot_description_semantic_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'aubo_e5.srdf'
    ])

    # Joint limits path
    joint_limits_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'joint_limits.yaml'
    ])

    # Kinematics path
    kinematics_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'kinematics.yaml'
    ])

    # Note: In ROS2 MoveIt2, the semantic, joint_limits, and kinematics parameters
    # are typically loaded by the move_group node via parameter files.
    # These will be passed to move_group in move_group.launch.py
    # The robot_description parameter will be set by nodes that need it (e.g., robot_state_publisher)

    return LaunchDescription([
        load_robot_description_arg,
        robot_description_arg,
        robot_ip_arg,
    ])

