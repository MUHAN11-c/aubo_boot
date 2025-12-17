from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    db_arg = DeclareLaunchArgument(
        'db',
        default_value='false',
        description='By default, we do not start a database (it can be large)'
    )
    
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('aubo_e5_moveit_config'),
            'default_warehouse_mongo_db'
        ]),
        description='Allow user to specify database location'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='By default, we are not in debug mode'
    )
    
    load_robot_description_arg = DeclareLaunchArgument(
        'load_robot_description',
        default_value='false',
        description='By default, we won\'t load or override the robot_description'
    )
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='By default, hide joint_state_publisher\'s GUI'
    )
    
    gazebo_gui_arg = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='true',
        description='Gazebo specific options'
    )
    
    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Gazebo specific options'
    )
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('aubo_description_ros2'),
            'urdf',
            'aubo_e5.xacro'
        ]),
        description='By default, use the urdf location provided from the package'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'paused': LaunchConfiguration('paused'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'urdf_path': LaunchConfiguration('urdf_path')
        }.items()
    )

    # Joint state publisher (without GUI)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['move_group/fake_controller_joint_states', '/joint_states']
        }],
        condition=UnlessCondition(LaunchConfiguration('use_gui'))
    )

    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'source_list': ['move_group/fake_controller_joint_states', '/joint_states']
        }],
        condition=IfCondition(LaunchConfiguration('use_gui'))
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
            'fake_execution': 'false',
            'info': 'true',
            'debug': LaunchConfiguration('debug')
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
        db_arg,
        db_path_arg,
        debug_arg,
        load_robot_description_arg,
        use_gui_arg,
        gazebo_gui_arg,
        paused_arg,
        urdf_path_arg,
        gazebo,
        joint_state_publisher,
        joint_state_publisher_gui,
        robot_state_publisher,
        move_group,
        moveit_rviz,
        warehouse_db,
    ])

