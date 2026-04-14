"""
启动 IVG Web 栈，通信栈为 RobotWebTools **rosbridge_suite**（默认 **统一代理**，非「浏览器直连 rosbridge」）：

  IncludeLaunchDescription(rosbridge_server/rosbridge_websocket_launch.xml)
    → 本机 Tornado WebSocket（rosbridge + rosapi）；浏览器侧默认经 FastAPI **同源** ``/ws/rosbridge`` 转发到 ``rosbridge_host:rosbridge_port``（与 ``IVG_ROSBRIDGE_*`` 一致）。
  tf2_web_republisher → 网页 TF。
  web_video_server（可选）→ 本机 MJPEG/snapshot；浏览器侧默认经 **同源** ``/api/ivg/proxy/web-video/…`` 转发（``IVG_WEB_VIDEO_*`` 指网关访问上游的地址）。
  FastAPI + Uvicorn 子进程：``--directory`` 指向已安装的 ``share/.../web/public``，提供静态页、``GET /health``、``GET /api/ivg/runtime-config``。

参数：默认打开「服务 / 动作在新线程执行」，并提高 max_message_size。
参见 https://github.com/RobotWebTools/rosbridge_suite
"""

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory  # 获取包的共享目录
from launch import LaunchDescription #启动文件
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable #启动动作
from launch.conditions import IfCondition  # 启动条件
from launch.launch_description_sources import FrontendLaunchDescriptionSource #启动描述源
from launch.substitutions import LaunchConfiguration #启动配置
from launch_ros.actions import Node #启动节点
from launch_ros.parameter_descriptions import ParameterValue #启动参数描述


def generate_launch_description():
    # image_transport 的 Raw 插件在 libimage_transport_plugins.so（位于 /opt/ros/.../lib，而非仅 x86_64-linux-gnu 子目录）。
    _img_transport_lib = os.path.join(get_package_prefix('image_transport'), 'lib')
    # 已安装 share 目录（含 web 资源）
    pkg_share = get_package_share_directory('aubo_ros2_web_dashboard')
    web_dir = os.path.join(pkg_share, 'web', 'public')
    # rosbridge 订阅 demo_interface/msg/RobotStatus 时需 dlopen libdemo_interface__rosidl_typesupport_c.so；
    # 若仅 source 了 overlay 但子进程未继承完整 LD_LIBRARY_PATH，会报
    # ``Could not import 'rosidl_typesupport_c' for package 'demo_interface'``，前端末端位姿永远无 JSON。
    _demo_interface_lib = os.path.join(get_package_prefix('demo_interface'), 'lib')
    _ros_base_lib = os.path.join(get_package_prefix('rclpy'), 'lib')
    _rosbridge_ld = os.pathsep.join([_demo_interface_lib, _ros_base_lib])
    if os.environ.get('LD_LIBRARY_PATH'):
        _rosbridge_ld = _rosbridge_ld + os.pathsep + os.environ['LD_LIBRARY_PATH']
    # web_video Node 使用 additional_env 覆盖 LD_LIBRARY_PATH，需显式带上 image_transport 与上述路径。
    _web_video_ld = _img_transport_lib + os.pathsep + _rosbridge_ld
    rosbridge_xml = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml',
    )

    web_host = LaunchConfiguration('web_host')  # FastAPI 静态网关绑定地址
    web_port = LaunchConfiguration('web_port')  # 静态网关端口，如 8090
    rosbridge_host = LaunchConfiguration('rosbridge_host')  # 网关转发 rosbridge 时的上游地址
    rosbridge_port = LaunchConfiguration('rosbridge_port')  # 与前端 ?rosbridge_port= 一致
    rosbridge_max_message_size = LaunchConfiguration('rosbridge_max_message_size')
    rosbridge_call_services_in_new_thread = LaunchConfiguration('rosbridge_call_services_in_new_thread')
    rosbridge_send_action_goals_in_new_thread = LaunchConfiguration(
        'rosbridge_send_action_goals_in_new_thread'
    )
    include_web_video = LaunchConfiguration('include_web_video_server')
    web_video_host = LaunchConfiguration('web_video_host')
    web_video_port = LaunchConfiguration('web_video_port')

    return LaunchDescription(
        [
            DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
            DeclareLaunchArgument('web_port', default_value='8090'),
            DeclareLaunchArgument(
                'rosbridge_host',
                default_value='127.0.0.1',
                description='IVG_ROSBRIDGE_HOST：FastAPI 统一代理转发 rosbridge 时的上游主机',
            ),
            DeclareLaunchArgument('rosbridge_port', default_value='9090'),
            DeclareLaunchArgument(
                'rosbridge_max_message_size',
                default_value='67108864',
                description='rosbridge max JSON frame size (bytes); 大点云 JSON/CBOR 单帧；默认 64MiB',
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
                'web_video_host',
                default_value='127.0.0.1',
                description='IVG_WEB_VIDEO_HOST：网关转发 web_video 时的上游主机',
            ),
            DeclareLaunchArgument(
                'web_video_port',
                default_value='8089',
                description='web_video_server HTTP 端口（默认 8089，避免与 IVG 手眼 Web 常用 8080 冲突）',
            ),
            SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=_rosbridge_ld),
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
            # 需已安装 image_transport（及常见 plugins）；否则日志会出现
            # ``Unable to load plugin for transport 'image_transport/raw_sub'``，快照/MJPEG异常。
            Node(
                package='web_video_server',
                executable='web_video_server',
                name='web_video_server',
                output='screen',
                condition=IfCondition(include_web_video),
                additional_env={'LD_LIBRARY_PATH': _web_video_ld},
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
                    'aubo_ros2_web_dashboard.fastapi_static_gateway',
                    web_port,
                    '--bind',
                    web_host,
                    '--directory',
                    web_dir,
                ],
                additional_env={
                    # 与 launch 参数一致，供 /api/ivg/runtime-config 与上游代理使用
                    'IVG_ROSBRIDGE_HOST': rosbridge_host,
                    'IVG_ROSBRIDGE_PORT': rosbridge_port,
                    'IVG_WEB_VIDEO_HOST': web_video_host,
                    'IVG_WEB_VIDEO_PORT': web_video_port,
                },
                output='screen',
            ),
        ]
    )
