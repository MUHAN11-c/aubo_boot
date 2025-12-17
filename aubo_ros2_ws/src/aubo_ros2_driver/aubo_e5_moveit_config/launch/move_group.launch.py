from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetParameter
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Planning context
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                'planning_context.launch.py'
            ])
        ])
    )

    # Declare arguments
    pipeline_arg = DeclareLaunchArgument(
        'pipeline',
        default_value='ompl',
        description='Planning pipeline'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='GDB Debug Option'
    )
    
    info_arg = DeclareLaunchArgument(
        'info',
        default_value='',
        description='Verbose Mode Option'
    )
    
    allow_trajectory_execution_arg = DeclareLaunchArgument(
        'allow_trajectory_execution',
        default_value='true',
        description='move_group settings'
    )
    
    fake_execution_arg = DeclareLaunchArgument(
        'fake_execution',
        default_value='false',
        description='move_group settings'
    )
    
    max_safe_path_cost_arg = DeclareLaunchArgument(
        'max_safe_path_cost',
        default_value='1',
        description='move_group settings'
    )
    
    jiggle_fraction_arg = DeclareLaunchArgument(
        'jiggle_fraction',
        default_value='0.05',
        description='move_group settings'
    )
    
    publish_monitored_planning_scene_arg = DeclareLaunchArgument(
        'publish_monitored_planning_scene',
        default_value='true',
        description='move_group settings'
    )
    
    capabilities_arg = DeclareLaunchArgument(
        'capabilities',
        default_value='',
        description='load these non-default MoveGroup capabilities (space seperated)'
    )
    
    disable_capabilities_arg = DeclareLaunchArgument(
        'disable_capabilities',
        default_value='',
        description='inhibit these default MoveGroup capabilities (space seperated)'
    )

    # Planning pipeline
    planning_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                'planning_pipeline.launch.py'
            ])
        ]),
        launch_arguments={
            'pipeline': LaunchConfiguration('pipeline')
        }.items()
    )

    # Trajectory execution
    trajectory_execution = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                'trajectory_execution.launch.py'
            ])
        ]),
        launch_arguments={
            'moveit_manage_controllers': 'true',
            'moveit_controller_manager': PythonExpression([
                "'fake' if '", LaunchConfiguration('fake_execution'), "' == 'true' else 'aubo_e5'"
            ])
        }.items(),
        condition=IfCondition(LaunchConfiguration('allow_trajectory_execution'))
    )

    # Sensors
    sensor_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                'sensor_manager.launch.py'
            ])
        ]),
        launch_arguments={
            'moveit_sensor_manager': 'aubo_e5'
        }.items(),
        condition=IfCondition(LaunchConfiguration('allow_trajectory_execution'))
    )

    # Get paths
    robot_description_semantic_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'aubo_e5.srdf'
    ])
    
    joint_limits_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'joint_limits.yaml'
    ])
    
    kinematics_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'kinematics.yaml'
    ])

    # Move group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            {
                'robot_description_semantic': robot_description_semantic_path,
                'robot_description_planning.joint_limits_file': joint_limits_path,
                'robot_description_kinematics': kinematics_path,
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                'response_adapters': 'default_planner_response_adapters/AddTimeOptimalParameterization default_planner_response_adapters/ValidateSolution',
                'allow_trajectory_execution': LaunchConfiguration('allow_trajectory_execution'),
                'max_safe_path_cost': LaunchConfiguration('max_safe_path_cost'),
                'jiggle_fraction': LaunchConfiguration('jiggle_fraction'),
                'capabilities': LaunchConfiguration('capabilities'),
                'disable_capabilities': LaunchConfiguration('disable_capabilities'),
                'planning_scene_monitor.publish_planning_scene': LaunchConfiguration('publish_monitored_planning_scene'),
                'planning_scene_monitor.publish_geometry_updates': LaunchConfiguration('publish_monitored_planning_scene'),
                'planning_scene_monitor.publish_state_updates': LaunchConfiguration('publish_monitored_planning_scene'),
                'planning_scene_monitor.publish_transforms_updates': LaunchConfiguration('publish_monitored_planning_scene'),
            }
        ],
        prefix=PythonExpression([
            "'gdb -ex run --args' if '", LaunchConfiguration('debug'), "' == 'true' else ''"
        ]),
        arguments=PythonExpression([
            "'--debug' if '", LaunchConfiguration('info'), "' == 'true' or '", LaunchConfiguration('debug'), "' == 'true' else ''"
        ])
    )

    return LaunchDescription([
        planning_context,
        pipeline_arg,
        debug_arg,
        info_arg,
        allow_trajectory_execution_arg,
        fake_execution_arg,
        max_safe_path_cost_arg,
        jiggle_fraction_arg,
        publish_monitored_planning_scene_arg,
        capabilities_arg,
        disable_capabilities_arg,
        planning_pipeline,
        trajectory_execution,
        sensor_manager,
        move_group_node,
    ])

