"""
IVG Web Dashboard — ROS 2 启动文件（2026-10-10 阶段 3 重写）.

启动流程:
  1. ivg_foxglove_bridge (vendored, MIT)   ← 唯一 ROS WebSocket 传输面（CDR, 8765）
  2. FastAPI 网关 (uvicorn)                ← 统一入口：代理 + 静态文件（端口 8095）

注（阶段 3, 2026-10-10）: rosbridge / tf2_web_republisher / rosapi 已整体移除，
浏览器端控制面与数据面全部经 vendored foxglove_bridge（见 VENDOR_PROVENANCE 与
ivg_foxglove_bridge 包）。本栈不进 peach harvest_system/lifecycle；8090 归 peach_observability。
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# 确保能 import 本包的 config 模块
_pkg_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
if _pkg_root not in sys.path:
    sys.path.insert(0, _pkg_root)

from aubo_ros2_web_dashboard import config as cfg  # noqa: E402


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share_str = get_package_share_directory("aubo_ros2_web_dashboard")
    web_dir = os.path.join(pkg_share_str, "web", "public")

    bridge_launch = PathJoinSubstitution([
        FindPackageShare("ivg_foxglove_bridge"),
        "launch", "ivg_foxglove_bridge.launch.xml",
    ])

    ld.add_action(DeclareLaunchArgument(
        "web_host", default_value=cfg.gateway_bind(),
        description="网关监听地址"))
    ld.add_action(DeclareLaunchArgument(
        "web_port", default_value=str(cfg.gateway_port()),
        description="网关监听端口（默认 8095；8090 归 peach_observability）"))
    ld.add_action(DeclareLaunchArgument(
        "foxglove_bridge_port", default_value=str(cfg.foxglove_bridge_port()),
        description="foxglove_bridge 上游端口（vendored ivg_foxglove_bridge）"))

    lc = {
        "web_host": LaunchConfiguration("web_host"),
        "web_port": LaunchConfiguration("web_port"),
        "foxglove_bridge_port": LaunchConfiguration("foxglove_bridge_port"),
    }

    # 1. vendored foxglove_bridge（唯一 ROS WebSocket 传输面；launch 内置 respawn 兜底）
    ld.add_action(IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(bridge_launch),
        launch_arguments={"port": lc["foxglove_bridge_port"]}.items(),
    ))

    # 2. FastAPI 网关 — 所有配置通过 CLI 参数传递
    ld.add_action(ExecuteProcess(
        cmd=[
            "python3", "-m",
            "aubo_ros2_web_dashboard.fastapi_static_gateway",
            lc["web_port"],
            "--bind", lc["web_host"],
            "--directory", web_dir,
            "--foxglove-bridge-port", lc["foxglove_bridge_port"],
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    ))

    return ld
