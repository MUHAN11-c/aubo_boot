"""
果园自主导航采摘环境 launch（Gazebo Harmonic + Nav2，sim 组内）.

组成：
1. Include 本包 ``orchard_sim.launch.py mobile:=true``：可驾驶履带平台
   （gz DiffDrive 滚筒履带 + 2D 激光）+ RSP + spawn + 完整桥接；
2. Include ``nav2_bringup bringup_launch.py``：AMCL + map_server（默认吃
   ``sim_worlds/maps/orchard_map.yaml`` 确定性生成图）或 ``slam:=True`` 在线
   建图，MPPI 控制器 + NavFn 规划器，参数文件 ``config/nav2_params.yaml``；
3. 可选 RViz（``rviz:=True``，nav2 默认视图，2D Goal Point 发目标）；
4. 可选 gz GUI 附着窗（``gz_gui:=True``）：本机图形栈上 gz-gui 的 ogre2
   EGL 通道损坏（Mesa 更新×NVIDIA 驱动错配，`driver (null)` 空转白屏），
   有效组合 = Qt 强制 GLX + GUI 3D 引擎切 OGRE 1.x（GLX 原生）+
   ``config/gz_gui_ogre1.config``；368 模型首场加载约需 60–90 s。

巡航：另开终端 ``ros2 run sim_worlds orchard_tour --rows 2,3``（逐树停靠位
NavigateToPose，注意 --rows 用逗号分隔）。SLAM 校验：``slam:=True`` +
``ros2 run nav2_map_server map_saver_cli -f /tmp/slam_check`` 与生成图对照。
测完 Ctrl+C 停栈并 ``pgrep -af 'gz sim|ros2|rviz'`` 清残留（MUST）。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory('sim_bringup'))
    worlds_share = Path(get_package_share_directory('sim_worlds'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('nav2_params')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    gz_gui = LaunchConfiguration('gz_gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真时钟（全图统一，勿关）'),
        DeclareLaunchArgument(
            'slam', default_value='False',
            description='true=slam_toolbox 在线建图（校验模式）；'
                        'false=AMCL + 静态地图'),
        DeclareLaunchArgument(
            'map', default_value=str(worlds_share / 'maps' / 'orchard_map.yaml'),
            description='静态地图（generate_orchard --map 确定性产物）'),
        DeclareLaunchArgument(
            'nav2_params', default_value=str(bringup_share / 'config' /
                                             'nav2_params.yaml'),
            description='Nav2 参数（帧 platform_base_link、果园限速）'),
        DeclareLaunchArgument(
            'gui', default_value='false', description='gz 仿真器自带 GUI'
            '（本机图形栈上不可用，见包 README；可视化请用 rviz/gz_gui）'),
        DeclareLaunchArgument(
            'rviz', default_value='True', description='起 RViz（nav2 视图）'),
        DeclareLaunchArgument(
            'gz_gui', default_value='False',
            description='起 gz GUI 附着窗（QT 强制 GLX + OGRE 1.x 引擎，'
                        '本机验证可用的组合；首场加载 60–90 s）'),

        # gz-gui 在本机走 GLX：Qt 层强制 xcb_glx（EGL-X11 通道已损坏）
        SetEnvironmentVariable('QT_XCB_GL_INTEGRATION', 'xcb_glx'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / 'launch' / 'orchard_sim.launch.py')),
            launch_arguments={
                'mobile': 'true',
                'gui': gui,
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / 'launch' / 'bringup_launch.py')),
            launch_arguments={
                'slam': slam,
                'map': map_yaml,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items()),
        ExecuteProcess(
            cmd=['gz', 'sim', '-g', '--gui-config',
                 str(bringup_share / 'config' / 'gz_gui_ogre1.config')],
            output='screen',
            condition=IfCondition(gz_gui)),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', str(nav2_share / 'rviz' /
                                 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]),
    ])
