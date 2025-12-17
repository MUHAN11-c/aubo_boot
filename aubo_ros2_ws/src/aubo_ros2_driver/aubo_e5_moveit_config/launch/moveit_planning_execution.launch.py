from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='The "sim" argument controls whether we connect to a Simulated or Real robot'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='127.0.0.1',
        description='Robot IP address (required if sim=false)'
    )

    # Load joint names
    joint_names_path = PathJoinSubstitution([
        FindPackageShare('aubo_e5_moveit_config'),
        'config',
        'joint_names.yaml'
    ])

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
            'load_robot_description': 'true',
            'robot_ip': LaunchConfiguration('robot_ip')
        }.items()
    )

    # Industrial robot simulator
    aubo_robot_simulator = Node(
        package='aubo_controller',
        executable='aubo_robot_simulator',
        name='aubo_robot_simulator'
    )

    # Joint trajectory action
    aubo_joint_trajectory_action = Node(
        package='aubo_controller',
        executable='aubo_joint_trajectory_action',
        name='aubo_joint_trajectory_action'
    )

    # Aubo driver
    aubo_driver = Node(
        package='aubo_driver',
        executable='aubo_driver',
        name='aubo_driver',
        parameters=[{
            '/server_host': LaunchConfiguration('robot_ip')
        }]
    )

    # Get robot description for robot_state_publisher
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

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content
        }]
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
            'publish_monitored_planning_scene': 'true'
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
            'config': 'true'
        }.items()
    )

    return LaunchDescription([
        sim_arg,
        robot_ip_arg,
        planning_context,
        aubo_robot_simulator,
        aubo_joint_trajectory_action,
        aubo_driver,
        robot_state_publisher,
        move_group,
        moveit_rviz,
    ])

