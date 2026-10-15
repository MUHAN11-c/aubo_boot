"""
aubo_ros2_web_dashboard 组合启动 — UI 面板 + ros_web_link 通信层.

启动流程:
  1. ivg_foxglove_bridge（vendored, MIT）  ← 唯一 ROS WebSocket 传输面（CDR, 8765）
  2. ros_web_link 网关 + UI 静态/BFF       ← 组合式：create_app(js_roots, html_root, sub_routers)
     通信层 JS 自动包含（ros_web_link 包 share）
     UI 层 HTML/CSS/panel JS 从本包 share 挂载
     BFF 子路由（runtime/settings/tool-geometries）从 gateway_routes 注入

独立传输模式（无 UI，等价 rosbridge 单独起）：
  ros2 launch ros_web_link transport.launch.xml
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_pkg_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
if _pkg_root not in sys.path:
    sys.path.insert(0, _pkg_root)


def generate_launch_description():
    ld = LaunchDescription()

    ui_share = get_package_share_directory("aubo_ros2_web_dashboard")
    web_dir = os.path.join(ui_share, "web", "public")
    js_dir = os.path.join(web_dir, "js")

    bridge_launch = PathJoinSubstitution([
        FindPackageShare("ivg_foxglove_bridge"),
        "launch", "ivg_foxglove_bridge.launch.xml",
    ])

    ld.add_action(DeclareLaunchArgument(
        "web_host", default_value="0.0.0.0",
        description="网关监听地址"))
    ld.add_action(DeclareLaunchArgument(
        "web_port", default_value="8095",
        description="网关监听端口（8090 归 peach_observability）"))
    ld.add_action(DeclareLaunchArgument(
        "foxglove_bridge_port", default_value="8765",
        description="foxglove_bridge 上游端口"))

    lc = {
        "web_host": LaunchConfiguration("web_host"),
        "web_port": LaunchConfiguration("web_port"),
        "foxglove_bridge_port": LaunchConfiguration("foxglove_bridge_port"),
    }

    # 1. vendored foxglove_bridge
    ld.add_action(IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(bridge_launch),
        launch_arguments={"port": lc["foxglove_bridge_port"]}.items(),
    ))

    # 2. 组合网关（ros_web_link 通信层 + UI 静态/BFF）
    #    用 Python 入口脚本：导入 ros_web_link.create_app + 注入 BFF + 挂载 UI 静态
    gateway_cmd = (
        f"python3 -c \""
        f"import sys; sys.path.insert(0, '{ui_share}/..'); "
        f"from aubo_ros2_web_dashboard.launch_gateway import run; "
        f"run(port={8095}, bind='{lc['web_host']}', "
        f"html_root='{web_dir}', js_roots=['{js_dir}'])"
        f"\""
    )
    # 简化：直接用 console_script（CLI 支持 --directory/--js-directory）
    ld.add_action(ExecuteProcess(
        cmd=[
            "python3", "-m", "aubo_ros2_web_dashboard.launch_gateway",
            lc["web_port"],
            "--bind", lc["web_host"],
            "--foxglove-bridge-port", lc["foxglove_bridge_port"],
            "--directory", web_dir,
            "--js-directory", js_dir,
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    ))

    return ld
