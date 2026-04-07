"""
在 web_dashboard.launch 基础上增加 FastAPI 网关进程：
  SetEnvironmentVariable IVG_GATEWAY_ROSBRIDGE_PORT → 与 rosbridge 端口一致，供 ivg_gateway 连接上游；
  uvicorn ivg_gateway.main:app → 浏览器经 ws://主机:gateway_port/ws/ros?token= 代理到 rosbridge。

注意：需已 pip 安装 gateway/requirements.txt，且 ros2 launch 前 source 工作空间。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aubo_ros2_web_dashboard')
    web_dir = os.path.join(pkg_share, 'web', 'ros2_web_bridge_demo')
    rosbridge_xml = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml',
    )

    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    gateway_host = LaunchConfiguration('gateway_host')  # uvicorn 绑定
    gateway_port = LaunchConfiguration('gateway_port')  # 与前端网关端口、登录 URL 一致

    return LaunchDescription(
        [
            DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
            DeclareLaunchArgument('web_port', default_value='8090'),
            DeclareLaunchArgument('rosbridge_port', default_value='9090'),
            DeclareLaunchArgument('gateway_host', default_value='0.0.0.0'),
            DeclareLaunchArgument('gateway_port', default_value='8765'),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(rosbridge_xml),
                launch_arguments={'port': rosbridge_port}.items(),
            ),
            Node(
                package='tf2_web_republisher',
                executable='tf2_web_republisher_node',
                name='tf2_web_republisher',
                output='screen',
            ),
            # 将 launch 的 rosbridge 端口注入网关进程（pydantic 读 IVG_GATEWAY_ROSBRIDGE_PORT）
            SetEnvironmentVariable(name='IVG_GATEWAY_ROSBRIDGE_PORT', value=rosbridge_port),
            ExecuteProcess(
                cmd=[
                    'python3',
                    '-m',
                    'uvicorn',
                    'ivg_gateway.main:app',
                    '--host',
                    gateway_host,
                    '--port',
                    gateway_port,
                ],
                output='screen',
            ),
            ExecuteProcess(
                cmd=[
                    'python3',
                    '-m',
                    'http.server',
                    web_port,
                    '--bind',
                    web_host,
                    '--directory',
                    web_dir,
                ],
                output='screen',
            ),
        ]
    )
