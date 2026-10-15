"""官方示例真实仿真数据面演示栈 — turtlebot3 官方仿真 + nav2 官方栈 + ros2 官方示例.

2026-10-15 用户裁定：viz_fixture 合成流删除；验证数据面全部来自 GitHub 热门官方
仓库（克隆进本项目：src/turtlebot3_msgs、src/turtlebot3_simulations、
src/ros2_examples、external/navigation2）与 ros2/moveit2/nav2 官方示例，
本地自写项目不参与验证。

数据源（全部官方）：
  turtlebot3_gazebo（ROBOTIS-GIT/turtlebot3_simulations jazzy 分支，已克隆构建）
    /joint_states /odom /tf /imu /scan /camera/image_raw /camera/camera_info /clock
  nav2_bringup（官方 apt；navigation2 jazzy 浅克隆于 external/ 做基线）
    /map /amcl_pose /particle_cloud /plan /diagnostics（localization + navigation）
  examples_rclpy_pointcloud_publisher（ros2/examples 官方点云示例）
    /test_cloud  PointCloud2
  examples_rclpy_minimal_publisher（ros2/examples 官方话题示例）
    /topic       std_msgs/String
  image_transport republish（官方）
    /camera/image_raw/compressed  CompressedImage

组成：gzserver（官方 turtlebot3_world，-r -s 无头）+ gzclient（gui:=true 打开
图形界面，界面正常打开；on_exit_shutdown 不挂——客户端崩溃不连坐仿真）+
官方 RSP/spawn 子 launch + 官方 nav2（AMCL 待 initialpose 激活）+ 官方示例节点
+ 传输栈（foxglove 桥 + 网关）。

启动（需 source 工作区 install；TB3 世界 ~20s，nav2 就绪 ~30s）：
  ros2 launch ros_web_link sim_demo.launch.py
  # 浏览器 http://127.0.0.1:8095/console.html —— 官方仿真真实数据
  # AMCL/规划闭环：console 发布 /initialpose 后 /amcl_pose /particle_cloud 出流
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import (FrontendLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration('port')
    gateway_port = LaunchConfiguration('gateway_port')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    fixture = LaunchConfiguration('fixture')
    gz_partition = LaunchConfiguration('gz_partition')
    nav2 = LaunchConfiguration('nav2')
    examples = LaunchConfiguration('examples')
    transport = LaunchConfiguration('transport')

    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    world = os.path.join(tb3_gazebo_share, 'worlds', 'turtlebot3_world.world')

    # 官方 turtlebot3_world：gzserver 无头 + 物理世界
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true',
        }.items())

    # gz 图形客户端（gui:=true 正常打开界面；客户端崩溃不连坐仿真）
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': '-g -v2 '}.items(),
        condition=IfCondition(gui))

    # 官方 RSP + spawn 子 launch（robot_description / joint_states / 桥）
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tb3_gazebo_share, 'launch', 'robot_state_publisher.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items())
    # 出生位姿必须显式传：spawn 子 launch 默认 (0,0) 是 turtlebot3_world 中心柱，
    # 机器人会卡在障碍物里（DiffDrive 指令全部无效，vanilla 入口靠自身传 -2/-0.5）
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tb3_gazebo_share, 'launch', 'spawn_turtlebot3.launch.py'])),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
        }.items())

    # 官方 nav2：AMCL 定位（tb3_sandbox 官方地图）+ 规划/控制全栈
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [nav2_bringup_share, 'launch', 'localization_launch.py'])),
        launch_arguments={
            'map': PathJoinSubstitution(
                [nav2_bringup_share, 'maps', 'tb3_sandbox.yaml']),
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(nav2))
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [nav2_bringup_share, 'launch', 'navigation_launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(nav2))

    # ros2 官方示例节点（ros2/examples，克隆于 src/ros2_examples）
    examples_on = IfCondition(examples)
    pointcloud = Node(
        package='examples_rclpy_pointcloud_publisher',
        executable='pointcloud_publisher', name='official_pointcloud',
        output='screen', condition=examples_on)
    talker = Node(
        package='examples_rclpy_minimal_publisher',
        executable='publisher_member_function', name='official_talker',
        output='screen', condition=examples_on)

    # 官方 image_transport republish：raw → compressedJPEG
    republish = Node(
        package='image_transport', executable='republish', name='image_republish',
        arguments=['raw', 'compressed'],
        remappings=[('in', '/camera/image_raw'),
                    ('out', '/camera/image_raw')],
        output='screen', condition=examples_on)

    transport = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_web_link'), 'launch', 'transport.launch.xml'])),
        launch_arguments={
            'port': port, 'gateway_port': gateway_port, 'fixture': fixture,
        }.items(),
        condition=IfCondition(transport))

    nav2_rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=IfCondition(rviz),
        arguments=['-d', PathJoinSubstitution(
            [nav2_bringup_share, 'rviz', 'nav2_default_view.rviz'])],
        parameters=[{'use_sim_time': True}],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('gateway_port', default_value='8095'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='gz 图形客户端（界面正常打开）'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('fixture', default_value='false',
                              description='comm_fixture 通信矩阵被测端'),
        DeclareLaunchArgument('gz_partition', default_value='',
                              description='gz-transport 分区（多仿真共存时必须互异，'
                              '空=默认分区；本机同时跑其他 gz 仿真时务必设置）'),
        DeclareLaunchArgument('nav2', default_value='true',
                              description='官方 nav2 定位/规划全栈（false=纯 TB3 仿真轻量模式）'),
        DeclareLaunchArgument('examples', default_value='true',
                              description='ros2/examples 官方示例节点（点云/talker/republish）'),
        DeclareLaunchArgument('transport', default_value='true',
                              description='foxglove 桥 + 网关传输栈'),
        DeclareLaunchArgument('x_pose', default_value='-2.0',
                              description='出生位姿（turtlebot3_world 官方点位）'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        SetEnvironmentVariable('GZ_PARTITION', gz_partition),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(tb3_gazebo_share, 'models')),
        gzserver,
        gzclient,
        spawn_tb3,
        rsp,
        nav2_localization,
        nav2_navigation,
        pointcloud,
        talker,
        republish,
        transport,
        nav2_rviz,
    ])
