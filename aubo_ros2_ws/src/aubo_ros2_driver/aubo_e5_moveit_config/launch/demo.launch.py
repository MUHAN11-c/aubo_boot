from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    pipeline_arg = DeclareLaunchArgument(
        'pipeline',
        default_value='lerp',
        description='specify the planning pipeline'
    )
    
    db_arg = DeclareLaunchArgument(
        'db',
        default_value='false',
        description='By default, we do not start a database (it can be large)'
    )
    
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=[PathJoinSubstitution([
            FindPackageShare('aubo_e5_moveit_config'),
            'default_warehouse_mongo_db'
        ])],
        description='Allow user to specify database location'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='By default, we are not in debug mode'
    )
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='By default, hide joint_state_publisher\'s GUI'
    )

    # Planning context
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                'planning_context.launch.py'
            ])
        ]),
        launch_arguments={
            'load_robot_description': 'true'
        }.items()
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_gui': LaunchConfiguration('use_gui'),
            'source_list': ['move_group/fake_controller_joint_states']
        }]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        respawn=True,
        output='screen'
    )

    # Move group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'allow_trajectory_execution': 'true',
            'fake_execution': 'true',
            'info': 'true',
            'debug': LaunchConfiguration('debug'),
            'pipeline': LaunchConfiguration('pipeline')
        }.items()
    )

    # RViz
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                'moveit_rviz.launch.py'
            ])
        ]),
        launch_arguments={
            'config': 'true',
            'debug': LaunchConfiguration('debug')
        }.items()
    )

    # Warehouse database (conditional)
    warehouse_db = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                'default_warehouse_db.launch.py'
            ])
        ]),
        launch_arguments={
            'moveit_warehouse_database_path': LaunchConfiguration('db_path')
        }.items(),
        condition=IfCondition(LaunchConfiguration('db'))
    )

    return LaunchDescription([
        pipeline_arg,
        db_arg,
        db_path_arg,
        debug_arg,
        use_gui_arg,
        planning_context,
        joint_state_publisher,
        robot_state_publisher,
        move_group,
        moveit_rviz,
        warehouse_db
    ])

