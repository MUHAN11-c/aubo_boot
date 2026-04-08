"""
启动「直连」Web 栈（无 FastAPI 网关），通信栈为 RobotWebTools **rosbridge_suite**：

  IncludeLaunchDescription(rosbridge_server/rosbridge_websocket_launch.xml)
    → WebSocket JSON API（rosbridge + rosapi），浏览器 ws://主机:rosbridge_port；
  tf2_web_republisher → 网页 TF；
  ThreadingHTTPServer → 托管 web/public 静态页（文档根；上游 RobotWebTools 会议 demo 演化，非 Node ros2-web-bridge）；
  web_video_server（可选）→ HTTP MJPEG/snapshot，/stream?topic=，减轻 sensor_msgs/Image 走 rosbridge。

参数：默认打开「服务 / 动作在新线程执行」，并提高 max_message_size，减轻大 JSON（点云等）被拒。
参见 https://github.com/RobotWebTools/rosbridge_suite
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 已安装 share 目录（含 web 资源）
    pkg_share = get_package_share_directory('aubo_ros2_web_dashboard')
    web_dir = os.path.join(pkg_share, 'web', 'public')
    rosbridge_xml = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml',
    )

    web_host = LaunchConfiguration('web_host')  # 静态页绑定地址
    web_port = LaunchConfiguration('web_port')  # 静态页端口，如 8090
    rosbridge_port = LaunchConfiguration('rosbridge_port')  # 与前端 ?rosbridge_port= 一致
    rosbridge_max_message_size = LaunchConfiguration('rosbridge_max_message_size')
    rosbridge_call_services_in_new_thread = LaunchConfiguration('rosbridge_call_services_in_new_thread')
    rosbridge_send_action_goals_in_new_thread = LaunchConfiguration(
        'rosbridge_send_action_goals_in_new_thread'
    )
    include_web_video = LaunchConfiguration('include_web_video_server')
    web_video_port = LaunchConfiguration('web_video_port')

    return LaunchDescription(
        [
            DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
            DeclareLaunchArgument('web_port', default_value='8090'),
            DeclareLaunchArgument('rosbridge_port', default_value='9090'),
            DeclareLaunchArgument(
                'rosbridge_max_message_size',
                default_value='20000000',
                description='rosbridge max JSON frame size (bytes); raise for large PointCloud2 via rosbridge',
            ),
            DeclareLaunchArgument(
                'rosbridge_call_services_in_new_thread',
                default_value='true',
                description='rosbridge: avoid blocking websocket on long service calls',
            ),
            DeclareLaunchArgument(
                'rosbridge_send_action_goals_in_new_thread',
                default_value='true',
                description='rosbridge: avoid blocking websocket on action goals',
            ),
            DeclareLaunchArgument(
                'include_web_video_server',
                default_value='true',
                description='若 true 则启动 web_video_server（需已安装 ros-humble-web-video-server）',
            ),
            DeclareLaunchArgument(
                'web_video_port',
                default_value='8089',
                description='web_video_server HTTP 端口（默认 8089，避免与 IVG 手眼 Web 常用 8080 冲突）',
            ),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(rosbridge_xml),
                launch_arguments={
                    'port': rosbridge_port,
                    'max_message_size': rosbridge_max_message_size,
                    'call_services_in_new_thread': rosbridge_call_services_in_new_thread,
                    'send_action_goals_in_new_thread': rosbridge_send_action_goals_in_new_thread,
                }.items(),
            ),
            Node(
                package='tf2_web_republisher',
                executable='tf2_web_republisher_node',
                name='tf2_web_republisher',
                output='screen',
            ),
            Node(
                package='web_video_server',
                executable='web_video_server',
                name='web_video_server',
                output='screen',
                condition=IfCondition(include_web_video),
                parameters=[
                    {
                        'port': ParameterValue(web_video_port, value_type=int),
                        'address': '0.0.0.0',
                        'server_threads': 4,
                        'ros_threads': 2,
                    }
                ],
            ),
            ExecuteProcess(
                cmd=[
                    'python3',
                    '-m',
                    'aubo_ros2_web_dashboard.threaded_static_server',
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
