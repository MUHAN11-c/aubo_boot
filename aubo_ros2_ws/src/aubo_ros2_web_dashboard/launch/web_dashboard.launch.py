"""IVG Web Dashboard — ROS 2 启动文件。

启动流程:
  1. rosbridge (Tornado WebSocket) ← ROS 消息总线桥
  2. tf2_web_republisher           ← TF 坐标 Web 发布
  3. web_video_server              ← 摄像头 MJPEG/快照 HTTP
  4. FastAPI 网关 (uvicorn)        ← 统一入口：代理 + 静态文件

参考: MoveIt2 demo.launch.py / rosbridge 官方 launch 模式

配置流: config/defaults.yaml → launch args 覆盖 → CLI 参数 → 网关进程
"""
import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# 确保能 import 本包的 config 模块
_pkg_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
if _pkg_root not in sys.path:
    sys.path.insert(0, _pkg_root)

from aubo_ros2_web_dashboard import config as cfg  # noqa: E402


def generate_launch_description():
    ld = LaunchDescription()

    # ── 路径解析 ────────────────────────────────────────────────────────
    # Vue 3 构建产物 (web/dist)
    pkg_share_str = get_package_share_directory("aubo_ros2_web_dashboard")
    web_dir = os.path.join(pkg_share_str, "web", "dist")

    # rosbridge launch 文件路径
    rosbridge_launch = PathJoinSubstitution([
        FindPackageShare("rosbridge_server"),
        "launch", "rosbridge_websocket_launch.xml",
    ])

    # ── 动态库路径 ──────────────────────────────────────────────────────
    img_lib = os.path.join(get_package_prefix("image_transport"), "lib")
    demo_lib = os.path.join(get_package_prefix("ivg_interfaces"), "lib")
    ros_lib = os.path.join(get_package_prefix("rclpy"), "lib")
    rosbridge_ld = os.pathsep.join([demo_lib, ros_lib])
    if os.environ.get("LD_LIBRARY_PATH"):
        rosbridge_ld = rosbridge_ld + os.pathsep + os.environ["LD_LIBRARY_PATH"]
    web_video_ld = os.pathsep.join([img_lib, rosbridge_ld])

    # ── 声明启动参数（默认值全部来自 YAML）─────────────────────────────
    ld.add_action(DeclareLaunchArgument("web_host",
        default_value=cfg.gateway_bind(),
        description="网关监听地址"))
    ld.add_action(DeclareLaunchArgument("web_port",
        default_value=str(cfg.gateway_port()),
        description="网关监听端口"))
    ld.add_action(DeclareLaunchArgument("rosbridge_host",
        default_value=cfg.rosbridge_host(),
        description="rosbridge 上游主机"))
    ld.add_action(DeclareLaunchArgument("rosbridge_port",
        default_value=str(cfg.rosbridge_port()),
        description="rosbridge 上游端口"))
    ld.add_action(DeclareLaunchArgument("rosbridge_max_msg_size",
        default_value=str(cfg.rosbridge_max_message_bytes()),
        description="rosbridge 最大消息字节"))
    ld.add_action(DeclareLaunchArgument("rosbridge_services_new_thread",
        default_value="true",
        description="服务调用使用新线程"))
    ld.add_action(DeclareLaunchArgument("rosbridge_actions_new_thread",
        default_value="true",
        description="Action 目标使用新线程"))
    ld.add_action(DeclareLaunchArgument("include_web_video_server",
        default_value="true",
        description="是否启动 web_video_server"))
    ld.add_action(DeclareLaunchArgument("web_video_host",
        default_value=cfg.web_video_host(),
        description="web_video 上游主机"))
    ld.add_action(DeclareLaunchArgument("web_video_port",
        default_value=str(cfg.web_video_port()),
        description="web_video 上游端口"))
    # 收集 LaunchConfiguration 引用
    lc = {
        "web_host":              LaunchConfiguration("web_host"),
        "web_port":              LaunchConfiguration("web_port"),
        "rosbridge_host":        LaunchConfiguration("rosbridge_host"),
        "rosbridge_port":        LaunchConfiguration("rosbridge_port"),
        "rosbridge_max_msg_size": LaunchConfiguration("rosbridge_max_msg_size"),
        "rosbridge_services_new_thread": LaunchConfiguration("rosbridge_services_new_thread"),
        "rosbridge_actions_new_thread":  LaunchConfiguration("rosbridge_actions_new_thread"),
        "include_web_video_server":      LaunchConfiguration("include_web_video_server"),
        "web_video_host":        LaunchConfiguration("web_video_host"),
        "web_video_port":        LaunchConfiguration("web_video_port"),
    }

    # ── LD_LIBRARY_PATH ──────────────────────────────────────────────────
    ld.add_action(SetEnvironmentVariable("LD_LIBRARY_PATH", rosbridge_ld))

    # ── 1. rosbridge (Tornado WebSocket) ─────────────────────────────────
    ld.add_action(IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(rosbridge_launch),
        launch_arguments={
            "port": lc["rosbridge_port"],
            "max_message_size": lc["rosbridge_max_msg_size"],
            "call_services_in_new_thread": lc["rosbridge_services_new_thread"],
            "send_action_goals_in_new_thread": lc["rosbridge_actions_new_thread"],
        }.items(),
    ))

    # ── 2. TF Web 重发布 ────────────────────────────────────────────────
    ld.add_action(Node(
        package="tf2_web_republisher",
        executable="tf2_web_republisher_node",
        name="tf2_web_republisher",
        output="screen",
    ))

    # ── 3. web_video_server ─────────────────────────────────────────────
    ld.add_action(Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        condition=IfCondition(lc["include_web_video_server"]),
        additional_env={"LD_LIBRARY_PATH": web_video_ld},
        parameters=[{
            "port": ParameterValue(lc["web_video_port"], value_type=int),
            "address": cfg.web_video_listen_address(),
            "server_threads": cfg.web_video_server_threads(),
            "ros_threads": cfg.web_video_ros_threads(),
        }],
    ))

    # ── 4. FastAPI 网关 — 所有配置通过 CLI 参数传递 ────────────────────
    ld.add_action(ExecuteProcess(
        cmd=[
            "python3", "-m",
            "aubo_ros2_web_dashboard.fastapi_static_gateway",
            lc["web_port"],
            "--bind", lc["web_host"],
            "--directory", web_dir,
            "--rosbridge-host", lc["rosbridge_host"],
            "--rosbridge-port", lc["rosbridge_port"],
            "--web-video-host", lc["web_video_host"],
            "--web-video-port", lc["web_video_port"],
        ],
        output="screen",
        respawn=True,
        respawn_delay=5.0,
    ))

    return ld
