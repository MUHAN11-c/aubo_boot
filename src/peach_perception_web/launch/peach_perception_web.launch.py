# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/BSD-3-Clause
"""启动桃子感知与重建只读 Web 控制台."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_node(context):
    """仅在用户显式给出 host/port 时覆盖 YAML."""
    parameters = [LaunchConfiguration('web_params_file')]
    overrides = {}
    host = LaunchConfiguration('host').perform(context)
    port = int(LaunchConfiguration('port').perform(context))
    if host:
        overrides['host'] = host
    if port:
        overrides['port'] = port
    if overrides:
        parameters.append(overrides)
    return [
        Node(
            package='peach_perception_web',
            executable='peach_perception_web',
            name='peach_perception_web',
            parameters=parameters,
            output='screen',
        )
    ]


def generate_launch_description():
    """生成默认加载 config/web.yaml 的 launch 描述."""
    share = get_package_share_directory('peach_perception_web')
    config = os.path.join(share, 'config', 'web.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'web_params_file', default_value=config,
            description='Web 网关参数文件；默认包内 config/web.yaml'),
        DeclareLaunchArgument(
            'host', default_value='',
            description='覆盖监听地址；空串使用 YAML'),
        DeclareLaunchArgument(
            'port', default_value='0',
            description='覆盖监听端口；0 使用 YAML'),
        OpaqueFunction(function=_launch_node),
    ])
