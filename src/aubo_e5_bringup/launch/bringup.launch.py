# Copyright 2026, aubo_e5_ros2_ws authors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# bringup.launch.py —— AUBO E5 工作区唯一启动入口
#
# 通过 launch 参数 hardware_mode 切换三种运行模式（mock/sim/real，详见 AGENTS.md
# 第 5 节）：
#   mock：mock_components/GenericSystem + 标准 joint_trajectory_controller
#         （ros2_control 回归链路）
#   sim ：aubo_e5_hardware/AuboE5SimHardware + passthrough 控制器
#         （无真机全链路闭环模拟，开发验证首选）
#   real：aubo_e5_hardware/AuboE5Hardware + passthrough 控制器 + dashboard（真机）
#
# 文件分两段：
#   1) launch_nodes()      —— OpaqueFunction 回调。launch 参数在声明阶段只有
#      "替换对象"而没有具体值，只有进入 OpaqueFunction 拿到 context 后才能
#      .perform(context) 求值，因此所有"按 mode 分支"的逻辑必须放在这里。
#   2) generate_launch_description() —— 声明全部 launch 参数，以及各自独立
#      开关的可选功能块（相机 / 手眼外参 TF / 手眼标定流程）。
#      IfCondition 在 launch 执行期惰性求值，不需要 mode 的确定值，
#      所以这部分可以留在声明式主流程里。

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
    """按 hardware_mode 分支组装核心节点列表并返回（OpaqueFunction 回调）.

    在这里 LaunchConfiguration 才能被 .perform(context) 解析成真实字符串，
    因此 hardware_mode / robot_ip / moveit_enabled 的分支逻辑都集中在本函数。
    """
    mode = LaunchConfiguration('hardware_mode').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    if mode not in ('mock', 'sim', 'real'):
        # 早失败：拼错 mode 时直接抛错，避免带着错误配置起一半节点
        raise RuntimeError('hardware_mode must be one of: mock | sim | real')
    # 已取消 RT 内核/SCHED_FIFO 预检：普通内核直接运行 real 模式。

    # 用 xacro 命令现场展开 URDF，并把 hardware_mode / robot_ip 透传进去——
    # xacro 内部据此选择硬件插件（mock/sim/real）并填充 <param> robot_ip，
    # 即"一份 URDF 模板、三种硬件后端"的实现方式。
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('aubo_description'), 'urdf', 'aubo_e5.urdf.xacro']),
        ' hardware_mode:=', mode,
        ' robot_ip:=', robot_ip,
    ])

    # 控制器参数（goal 容差、blend、RIB 流控等），同时喂给 ros2_control_node
    controllers_yaml = os.path.join(
        get_package_share_directory('aubo_e5_bringup'), 'config', 'controllers.yaml')

    # cwd must contain ./config/auborobot.conf for the legacy AUBO SDK (real mode)
    # 旧 SDK 按"进程 CWD"读取 ./config/auborobot.conf 与 tracelog.properties，
    # 因此 ros2_control_node 的工作目录必须指到 aubo_e5_hardware 的 share 目录
    # （config 随包安装在那里）。mock/sim 模式不连 SDK，设了也无害。
    sdk_cwd = get_package_share_directory('aubo_e5_hardware')

    # URDF 文本会被 launch_ros 误当 YAML 解析，显式声明为字符串
    robot_description_str = ParameterValue(robot_description, value_type=str)

    nodes = [
        # 发布 robot_description + TF（joint_states → 各连杆坐标系）
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description_str}], output='screen'),
        # ros2_control 主节点：加载硬件插件与全部控制器，200Hz 控制循环
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': robot_description_str}, controllers_yaml],
             cwd=sdk_cwd, output='screen'),
        # 关节状态广播器：三种模式都需要（MoveIt/RViz 依赖 /joint_states）
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                        '--controller-manager-timeout', '10'],
             output='screen'),
    ]

    if mode == 'mock':
        # mock 走官方 ros2_control 回归链路：标准 JTC（FollowJointTrajectory
        # action server），与 MoveIt 的 controllers_mock.yaml 映射对应
        nodes.append(Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager',
                       '/controller_manager', '--controller-manager-timeout', '10'],
            output='screen'))
    else:
        # sim / real 走 passthrough 架构的两个自研控制器：
        #   aubo_io_controller            —— IO 状态发布 + set_io 服务 + RIB 状态
        #   aubo_passthrough_trajectory_controller —— FJT action server，
        #       一次性下发轨迹（蓝本语义，见 AGENTS.md 第 1 节）
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
            # 真机才有 dashboard：上电/断电/停止/FK/IK/负载等非运动类服务。
            # 与 SDK 的第二条连接（conn_status_），同样需要 SDK 配置目录作 CWD，
            # 这里用的是 aubo_dashboard 自己的 share 目录（也装了 SDK config）。
            nodes.append(Node(
                package='aubo_dashboard', executable='aubo_dashboard_node',
                parameters=[{'robot_ip': robot_ip}],
                cwd=get_package_share_directory('aubo_dashboard'), output='screen'))

    # MoveIt 由 aubo_e5_moveit_config 的唯一 launch（moveit.launch.py，整体启动
    # move_group + rviz2）提供；本文件只按模式选控制器映射并集成导入：mock 走
    # 标准 JTC（controllers_mock.yaml，官方 ros2_control 链路），sim/real 走
    # passthrough 控制器（controllers.yaml）。放进 OpaqueFunction 才能按 mode 取值。
    moveit_enabled = LaunchConfiguration('moveit_enabled').perform(context).lower() == 'true'
    if moveit_enabled:
        controllers_file = 'controllers_mock.yaml' if mode == 'mock' else 'controllers.yaml'
        # standalone_state_publishers=false：robot_state_publisher 已在上面启动，
        # moveit.launch.py 不要再起一份
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
    """声明全部 launch 参数，并组装可选功能块（各自独立开关，IfCondition 惰性求值）.

    默认：hardware_mode:=real，相机 / 手眼外参 TF / MoveIt 开启；
    手眼标定流程（采集+求解）默认关闭。
    """
    # ---- 可选功能块（每个都有独立 enabled 参数）----

    # Percipio 相机：include percipio_camera.launch.py（深度/点云等由其内部参数决定）
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('percipio_camera'),
            'launch', 'percipio_camera.launch.py')),
        condition=IfCondition(LaunchConfiguration('camera_enabled')))
    # 手眼外参静态 TF（wrist3_Link → camera_link），与相机开关独立
    extrinsics = Node(
        package='aubo_hand_eye_calibration',
        executable='extrinsics_publisher',
        name='hand_eye_extrinsics_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('extrinsics_enabled')))
    # 手眼标定流程（17 预定义位姿采集 + 求解 + Web）。
    # 标定 launch 内不再起相机/外参：由本文件的 camera_enabled /
    # extrinsics_enabled 统一管；这里固定 extrinsics_enabled:=false 避免双发 TF。
    calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('aubo_hand_eye_calibration'),
            'launch', 'hand_eye_calibration.launch.py')),
        launch_arguments={
            'extrinsics_enabled': 'false',
            'web_enabled': LaunchConfiguration('hand_eye_web_enabled'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('hand_eye_enabled')))
    return LaunchDescription([
        # ---- launch 参数（默认 = 日常真机 + 相机 + 外参）----
        DeclareLaunchArgument(
            'hardware_mode', default_value='real',
            choices=['mock', 'sim', 'real'],
            description='mock | sim | real（默认 real）'),
        DeclareLaunchArgument(
            'robot_ip', default_value='169.254.10.98',
            description='真机控制器 IP；mock/sim 不用'),
        DeclareLaunchArgument(
            'moveit_enabled', default_value='true',
            description='MoveIt move_group + rviz2；false 关闭'),
        DeclareLaunchArgument(
            'camera_enabled', default_value='true',
            description='启动 percipio_camera.launch.py；false 关闭'),
        DeclareLaunchArgument(
            'extrinsics_enabled', default_value='true',
            description='启动手眼外参 TF（extrinsics_publisher）；false 关闭'),
        DeclareLaunchArgument(
            'hand_eye_enabled', default_value='false',
            description='启动手眼标定流程（采集/求解/Web）；默认关'),
        DeclareLaunchArgument(
            'hand_eye_web_enabled', default_value='false',
            description='标定 Web 界面；仅 hand_eye_enabled:=true 时生效'),
        OpaqueFunction(function=launch_nodes),
        camera_launch,
        extrinsics,
        calibration_launch,
    ])
