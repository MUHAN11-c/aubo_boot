from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start Gazebo paused'
    )
    
    gazebo_gui_arg = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='true',
        description='Set to "false" to run headless'
    )
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('aubo_description_ros2'),
            'urdf',
            'aubo_e5.xacro'
        ]),
        description='URDF/xacro file path'
    )

    # Get robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_path')]),
        value_type=str
    )

    # Gazebo world
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': 'worlds/empty.world',
            'paused': LaunchConfiguration('paused'),
            'gui': LaunchConfiguration('gazebo_gui')
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_gazebo_model',
        output='screen',
        arguments=[
            '-urdf',
            '-param', 'robot_description',
            '-model', 'robot',
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ]
    )

    # ROS controllers
    ros_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                'ros_controllers.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        paused_arg,
        gazebo_gui_arg,
        urdf_path_arg,
        gazebo_world,
        robot_state_publisher,
        spawn_entity,
        ros_controllers,
    ])

