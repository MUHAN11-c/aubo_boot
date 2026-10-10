#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _web_node(context, *_args, **_kwargs):
    host = LaunchConfiguration("host").perform(context)
    port = LaunchConfiguration("port").perform(context)
    reload_raw = LaunchConfiguration("reload").perform(context).strip().lower()
    reload_on = reload_raw in ("true", "1", "yes")
    arguments = ["--host", host, "--port", port]
    if reload_on:
        arguments.append("--reload")
    return [
        Node(
            package="ivg_pose_estimation",
            executable="ivg_pose_estimation_web",
            arguments=arguments,
            output="screen",
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    host_arg = DeclareLaunchArgument(
        "host",
        default_value="127.0.0.1",
        description="FastAPI bind host",
    )

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8089",
        description="FastAPI bind port（8089：与 aubo_hand_eye_calibration 网关 8088 错开）",
    )

    reload_arg = DeclareLaunchArgument(
        "reload",
        default_value="false",
        description="If true, pass --reload to uvicorn (dev: auto-restart on code changes)",
    )

    return LaunchDescription(
        [host_arg, port_arg, reload_arg, OpaqueFunction(function=_web_node)]
    )
