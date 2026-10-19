"""ros_web_link 独立监控入口 — 附着到**已在运行**的 ROS 2 图（仿真/真机皆可）.

与 ``sim_demo.launch.py``（自带 TB3 官方数据面）和 ``transport.launch.xml``
（纯传输 + 可选 fixture）的区别：本 launch 不拥有任何仿真/演示节点，只起
Foxglove WebSocket 桥 + 网关。典型用法是先把完整仿真项目正常跑起来，再在
另一个终端执行：

    ros2 launch ros_web_link monitor.launch.py

浏览器打开 ``http://127.0.0.1:8095/``（自动跳转诊断台），连接后 Displays
会按图上实际存在的话题自动适配（果园仿真的 ``/camera/color/image_raw``、
``/camera/depth_registered/points``、``/peach/**`` 标记等；官方 TB3 栈的
``/test_cloud``、压缩图等同样自动识别）。

设计边界（对齐 rosbridge_suite 的 server/transport 分包）：
- 本包只消费 ROS 图，不发布运动指令、不改变仿真状态；
- 桥与网关随本 launch 生命周期起停；被测仿真保持原样；
- 已有桥在跑时传 ``start_bridge:=false`` 复用（``bridge_host/bridge_port``）。
"""
from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _default_python() -> str:
    """网关解释器：优先 RWL_PYTHON，其次工作区 venv，最后 python3.

    网关依赖 fastapi/starlette 的兼容版本组合；系统 Python 常见版本冲突
    （Router.__init__() got an unexpected keyword argument 'on_startup'）。
    工作区布局 ``<ws>/install/<pkg>`` + ``<ws>/aubo_py3.12`` 时自动选 venv，
    让 ``ros2 launch ros_web_link monitor.launch.py`` 开箱即用。
    """
    if os.environ.get('RWL_PYTHON'):
        return os.environ['RWL_PYTHON']
    try:
        from ament_index_python.packages import get_package_prefix
        ws = Path(get_package_prefix('ros_web_link')).parent.parent
        candidate = ws / 'aubo_py3.12' / 'bin' / 'python'
        if candidate.is_file():
            return str(candidate)
    except Exception:  # noqa: BLE001 —— 非工作区安装时退回 PATH
        pass
    return 'python3'


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration('bridge_port')
    gateway_port = LaunchConfiguration('gateway_port')
    bind = LaunchConfiguration('bind')
    bridge_host = LaunchConfiguration('bridge_host')
    start_bridge = LaunchConfiguration('start_bridge')
    python = LaunchConfiguration('python')

    bridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ivg_foxglove_bridge'), 'launch',
             'ivg_foxglove_bridge.launch.xml'])),
        launch_arguments={'port': port}.items(),
        condition=IfCondition(start_bridge))

    gateway = ExecuteProcess(
        cmd=[python, '-m', 'ros_web_link.cli', gateway_port,
             '--bind', bind,
             '--foxglove-bridge-host', bridge_host,
             '--foxglove-bridge-port', port],
        output='screen',
        respawn=True,
        respawn_delay=2.0)

    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_port', default_value='18766',
            description='Foxglove WS 桥端口（默认 18766；8765 常被 preview_server 占用）'),
        DeclareLaunchArgument(
            'gateway_port', default_value='8095',
            description='网关 HTTP/WS 端口（8090 归 peach_observability，勿占）'),
        DeclareLaunchArgument(
            'bind', default_value='0.0.0.0',
            description='网关监听地址'),
        DeclareLaunchArgument(
            'bridge_host', default_value='127.0.0.1',
            description='网关连接的上游桥地址（start_bridge:=false 时指定外部桥）'),
        DeclareLaunchArgument(
            'start_bridge', default_value='true',
            description='true=随本 launch 起桥；false=复用已在运行的桥'),
        DeclareLaunchArgument(
            'python', default_value=_default_python(),
            description='网关解释器（默认自动选工作区 aubo_py3.12 venv；'
                        '可用 RWL_PYTHON 覆盖）'),
        LogInfo(msg=[
            'ros_web_link monitor 已启动 → http://127.0.0.1:', gateway_port,
            '/  (诊断台 /console.html；桥 ', bridge_host, ':', port, ')']),
        bridge,
        gateway,
    ])
