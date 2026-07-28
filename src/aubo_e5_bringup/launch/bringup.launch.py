import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_nodes(context):
    mode = LaunchConfiguration('hardware_mode').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    if mode not in ('mock', 'sim', 'real'):
        raise RuntimeError("hardware_mode must be one of: mock | sim | real")
    # 已取消 RT 内核/SCHED_FIFO 预检：普通内核直接运行 real 模式。

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('aubo_description'), 'urdf', 'aubo_e5.urdf.xacro']),
        ' hardware_mode:=', mode,
        ' robot_ip:=', robot_ip,
    ])

    controllers_yaml = os.path.join(
        get_package_share_directory('aubo_e5_bringup'), 'config', 'controllers.yaml')

    # cwd must contain ./config/auborobot.conf for the legacy AUBO SDK (real mode)
    sdk_cwd = get_package_share_directory('aubo_e5_hardware')

    # URDF 文本会被 launch_ros 误当 YAML 解析，显式声明为字符串
    robot_description_str = ParameterValue(robot_description, value_type=str)

    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description_str}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': robot_description_str}, controllers_yaml],
             cwd=sdk_cwd, output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                        '--controller-manager-timeout', '10'],
             output='screen'),
    ]

    if mode == 'mock':
        nodes.append(Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '10'],
            output='screen'))
    else:
        nodes += [
            Node(package='controller_manager', executable='spawner',
                 arguments=['aubo_io_controller', '--controller-manager', '/controller_manager',
                            '--controller-manager-timeout', '10'],
                 output='screen'),
            Node(package='controller_manager', executable='spawner',
                 arguments=['aubo_passthrough_trajectory_controller',
                            '--controller-manager', '/controller_manager',
                            '--controller-manager-timeout', '10'],
                 output='screen'),
        ]
        if mode == 'real':
            nodes.append(Node(
                package='aubo_dashboard', executable='aubo_dashboard_node',
                parameters=[{'robot_ip': robot_ip}],
                cwd=get_package_share_directory('aubo_dashboard'), output='screen'))

    # MoveIt 控制器映射按模式选择：mock 走标准 JTC（官方 ros2_control 链路），
    # sim/real 走 passthrough 控制器。放进 OpaqueFunction 才能按 mode 取值。
    moveit_enabled = LaunchConfiguration('moveit_enabled').perform(context).lower() == 'true'
    if moveit_enabled:
        controllers_file = 'controllers_mock.yaml' if mode == 'mock' else 'controllers.yaml'
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('aubo_e5_moveit_config'),
                'launch', 'moveit.launch.py')),
            launch_arguments={
                'standalone_state_publishers': 'false',
                'controllers_file': controllers_file,
            }.items()))
    return nodes


def generate_launch_description():
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('percipio_camera'),
            'launch', 'percipio_camera.launch.py')),
        condition=IfCondition(LaunchConfiguration('camera_enabled')))
    extrinsics = Node(
        package='aubo_hand_eye_calibration',
        executable='extrinsics_publisher',
        name='hand_eye_extrinsics_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('camera_enabled')))
    calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('aubo_hand_eye_calibration'),
            'launch', 'hand_eye_calibration.launch.py')),
        launch_arguments={
            'camera_enabled': 'false',
            'extrinsics_enabled': 'false',
            'web_enabled': LaunchConfiguration('hand_eye_web_enabled'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('hand_eye_enabled')))
    named_pose = Node(
        package='aubo_e5_bringup', executable='named_pose_controller.py',
        name='aubo_named_pose_controller', output='screen',
        condition=IfCondition(LaunchConfiguration('named_pose_enabled')))
    return LaunchDescription([
        DeclareLaunchArgument('hardware_mode', default_value='mock',
                              description='mock | sim | real'),
        DeclareLaunchArgument('robot_ip', default_value='169.254.10.98'),
        DeclareLaunchArgument('moveit_enabled', default_value='true',
                              description='默认启动 MoveIt move_group + rviz2；'
                                          '硬件底层调试时可用 moveit_enabled:=false 关闭'),
        DeclareLaunchArgument('named_pose_enabled', default_value='false'),
        DeclareLaunchArgument('camera_enabled', default_value='false'),
        DeclareLaunchArgument('hand_eye_enabled', default_value='false'),
        DeclareLaunchArgument('hand_eye_web_enabled', default_value='true'),
        OpaqueFunction(function=launch_nodes),
        named_pose,
        camera_launch,
        extrinsics,
        calibration_launch,
    ])
