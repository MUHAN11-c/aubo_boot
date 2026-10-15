"""官方示例演示栈 — 官方组件为主、本包 fixture 只补合成传感器流.

官方优先原则（2026-10-15 用户裁定）：官方生态能提供的数据面一律用官方组件，
viz_fixture 退守官方无法提供的合成流（无相机/激光硬件与仿真时）：

  robot_description / TF    官方 robot_state_publisher + MoveIt2 官方 panda URDF
                            （moveit_resources_panda_description，官方 ros2 分支）
  joint_states → /tf 动态   官方 `ros2 topic pub`（web 亦可经 clientPublish 反向驱动）
  /map                       官方 nav2_map_server + 官方地图 tb3_sandbox
  /diagnostics               官方 nav2_lifecycle_manager（Nav2 Health）
  /rosout                    各官方节点自带
  传感器合成流               viz_fixture（Image/PointCloud2/LaserScan/Odometry/…）
  通信矩阵被测端             comm_fixture（example_interfaces 官方类型）

启动（需 source 工作区 install）：
  ros2 launch ros_web_link official_demo.launch.py
  # 浏览器 http://127.0.0.1:8095/console.html —— 官方 panda + 官方地图
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    gateway_port = LaunchConfiguration('gateway_port')
    map_yaml = LaunchConfiguration('map_yaml')
    fixture = LaunchConfiguration('fixture')

    # 官方 panda URDF（官方 xacro 展开；file:// mesh 由 web 端重写 package:// 走网关路由）
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('moveit_resources_panda_description'),
                'urdf', 'panda.urdf.xacro']),
        ]),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}])

    # 官方 joint_states 驱动（1Hz 存活帧；web 经 clientPublish 可覆盖驱动官方 RSP 出动态 TF）
    joint_pub = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-r', '1', '/joint_states',
             'sensor_msgs/msg/JointState',
             '{name: [panda_joint1, panda_joint2, panda_joint3, panda_joint4,'
             ' panda_joint5, panda_joint6, panda_joint7],'
             ' position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]}'],
        output='screen')

    # 官方 Nav2 map_server（官方地图）+ lifecycle_manager 拉起（自动发现偶发不推进，
    # TimerAction 延迟启动给发现留时间；仍卡住时官方 CLI 兜底：
    # ros2 lifecycle set /map_server configure && ... activate）
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}])
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager', output='screen',
        parameters=[{'use_sim_time': False, 'node_names': ['map_server'],
                     'autostart': True}])

    transport = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_web_link'), 'launch', 'transport.launch.xml'])),
        launch_arguments={
            'port': port,
            'gateway_port': gateway_port,
            'fixture': fixture,
            'viz_fixture': 'true',
            # 官方 RSP 已提供 /robot_description——fixture 演示 URDF 让位（防 latched 双源）
            'viz_publish_urdf': 'false',
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('gateway_port', default_value='8095'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'maps', 'tb3_sandbox.yaml'])),
        DeclareLaunchArgument('fixture', default_value='true',
                              description='comm_fixture 通信矩阵被测端（自检 9 项用）'),
        transport,
        rsp,
        joint_pub,
        TimerAction(period=3.0, actions=[map_server, lifecycle]),
    ])
