"""
统一启动：rosapi、rosbridge WebSocket、可选 tf2_web_republisher、FastAPI 网关进程。

可选 Include VPE Web（visual_pose_estimation_web），便于单机一键调试。
网关子进程通过环境变量接收 rosbridge 端口、VPE 上游 URL 等（见 _setup 内 env 字典）。
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):  # noqa: ARG001
    """根据 launch 参数拼装 Node/ExecuteProcess 列表，并注入 web_dashboard 所需环境变量。"""
    web_host = context.perform_substitution(LaunchConfiguration("web_host"))
    web_port = context.perform_substitution(LaunchConfiguration("web_port"))
    rosbridge_port = context.perform_substitution(LaunchConfiguration("rosbridge_port"))
    public_host = context.perform_substitution(LaunchConfiguration("public_host"))
    use_sim = context.perform_substitution(LaunchConfiguration("use_sim_time"))
    fixed_frame = context.perform_substitution(LaunchConfiguration("fixed_frame"))
    start_tf2 = context.perform_substitution(LaunchConfiguration("start_tf2_web_republisher"))
    vpe_upstream = context.perform_substitution(LaunchConfiguration("vpe_upstream"))

    try:
        rbp = int(rosbridge_port)
    except ValueError:
        rbp = 9090

    env = os.environ.copy()
    # 与 vpe_proxy._upstream_base()、gateway_ready 使用的变量一致
    env["AUBO_VPE_UPSTREAM"] = vpe_upstream.strip() or "http://127.0.0.1:8088"
    env["AUBO_WEB_ROSBRIDGE_PORT"] = str(rbp)
    env["AUBO_WEB_USE_SIM_TIME"] = use_sim
    env["AUBO_WEB_FIXED_FRAME"] = fixed_frame
    if public_host.strip():
        env["AUBO_WEB_PUBLIC_HOST"] = public_host.strip()

    # rosapi 为 rosbridge 提供话题/服务类型查询等能力，多数 RWT 客户端需要
    actions = [
        Node(
            package="rosapi",
            executable="rosapi_node",
            name="rosapi",
            output="screen",
        ),
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[{"port": rbp, "delay_between_messages": 0.0}],
        ),
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "aubo_ros2_web_dashboard",
                "web_dashboard",
                "--host",
                web_host,
                "--port",
                web_port,
            ],
            output="screen",
            env=env,  # type: ignore[arg-type]
        ),
    ]

    # tf2_web 供浏览器端 ros3d 等订阅 TF（可选，按需关闭以减负）
    if start_tf2.lower() in ("1", "true", "yes"):
        actions.insert(
            1,
            Node(
                package="tf2_web_republisher",
                # Humble 及当前二进制名为 tf2_web_republisher_node（非旧名 tf2_web_republisher）
                executable="tf2_web_republisher_node",
                name="tf2_web_republisher",
                output="screen",
            ),
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """声明参数、条件启动 VPE Web，再通过 OpaqueFunction 展开具体进程列表。"""
    vpe_web_launch = PathJoinSubstitution(
        [FindPackageShare("visual_pose_estimation_python"), "launch", "visual_pose_estimation_web.launch.py"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("web_host", default_value="0.0.0.0", description="网关 FastAPI 绑定地址"),
            DeclareLaunchArgument("web_port", default_value="8090", description="网关 HTTP 端口（统一入口）"),
            DeclareLaunchArgument(
                "vpe_upstream",
                default_value="http://127.0.0.1:8088",
                description="视觉位姿 FastAPI 反向代理上游（须与 VPE Web 地址一致）",
            ),
            DeclareLaunchArgument(
                "launch_vpe_web",
                default_value="false",
                description="若为 true，同时启动 visual_pose_estimation_web（默认 127.0.0.1:8088）",
            ),
            DeclareLaunchArgument("rosbridge_port", default_value="9090", description="rosbridge WebSocket port"),
            DeclareLaunchArgument(
                "public_host",
                default_value="",
                description="可选；若设置，前端可用该主机名构造 WebSocket（替代 location.hostname）",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="写入 /api/config，供前端提示"),
            DeclareLaunchArgument("fixed_frame", default_value="base_link", description="默认 fixed frame 提示（/api/config）"),
            DeclareLaunchArgument(
                "start_tf2_web_republisher",
                default_value="true",
                description="是否启动 tf2_web_republisher（浏览器 TF 可视化常用）",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([vpe_web_launch]),
                launch_arguments={
                    "host": "127.0.0.1",
                    "port": "8088",
                    "reload": "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("launch_vpe_web")),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
