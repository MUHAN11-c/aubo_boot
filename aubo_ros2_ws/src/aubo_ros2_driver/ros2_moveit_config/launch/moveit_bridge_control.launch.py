#!/usr/bin/env python3

"""
Launch file for MoveIt2 with ROS 1 bridge control.
This launch file starts MoveIt2, RViz2, and the bridge to control ROS 1 robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='127.0.0.1',
        description='Robot IP address'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    use_bridge_arg = DeclareLaunchArgument(
        'use_bridge',
        default_value='false',
        description='Launch ros1_bridge'
    )
    
    # Get package paths
    ros2_moveit_config_pkg = FindPackageShare('ros2_moveit_config')
    aubo_description = FindPackageShare('aubo_description')
    
    # Robot description - using xacro command
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            aubo_description,
            'urdf',
            'aubo_e5.xacro'
        ])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot description semantic - read SRDF file
    robot_description_semantic_path = PathJoinSubstitution([
        ros2_moveit_config_pkg,
        'config',
        'aubo_e5.srdf'
    ])
    
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_path
    }
    
    # Planning pipeline
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'response_adapters': 'default_planner_response_adapters/AddTimeOptimalParameterization'
        }
    }
    
    # Trajectory execution - using simple controller manager
    # Note: Actual execution is done via demo_driver services through bridge
    trajectory_execution = {
        'moveit_manage_controllers': False,  # We manage execution via services
        'trajectory_execution': {
            'allowed_execution_duration_scaling': 4.0,
            'allowed_goal_duration_margin': 0.5,
            'allowed_start_tolerance': 0.01
        },
        'moveit_controller_manager': 'moveit_simple_controller_manager::MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_list': [
                {
                    'name': 'aubo_e5_controller',
                    'type': 'FollowJointTrajectory',
                    'joints': [
                        'shoulder_joint',
                        'upperArm_joint',
                        'foreArm_joint',
                        'wrist1_joint',
                        'wrist2_joint',
                        'wrist3_joint'
                    ],
                    'action_ns': 'follow_joint_trajectory'
                }
            ]
        }
    }
    
    # Planning scene monitor
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }
    
    # Start MoveIt2 move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
            PathJoinSubstitution([
                ros2_moveit_config_pkg,
                'config',
                'joint_limits.yaml'
            ]),
            PathJoinSubstitution([
                ros2_moveit_config_pkg,
                'config',
                'kinematics.yaml'
            ]),
            PathJoinSubstitution([
                ros2_moveit_config_pkg,
                'config',
                'ompl_planning.yaml'
            ])
        ]
    )
    
    # RViz2
    rviz_config_file = PathJoinSubstitution([
        ros2_moveit_config_pkg,
        'config',
        'moveit.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # ros1_bridge
    bridge_node = Node(
        package='ros1_bridge',
        executable='dynamic_bridge',
        name='ros1_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_bridge'))
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        use_rviz_arg,
        use_bridge_arg,
        move_group_node,
        rviz_node,
        bridge_node,
        robot_state_publisher_node
    ])
