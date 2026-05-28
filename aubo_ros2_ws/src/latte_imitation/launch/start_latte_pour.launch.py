"""启动 latte_imitation 拉花轨迹预览/回放节点喵~

用法:
  ros2 launch latte_imitation start_latte_pour.launch.py

或带参数:
  ros2 launch latte_imitation start_latte_pour.launch.py \
    episode_idx:=0 arm:=right mode:=preview \
    roll_deg:=0 pitch_deg:=0 yaw_deg:=90
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        # ── Episode / 手臂 / 速度 / 模式 ──
        DeclareLaunchArgument("episode_idx", default_value="0",
                              description="Episode 编号 (0-39)"),
        DeclareLaunchArgument("arm", default_value="right",
                              description="'left'=持杯臂 / 'right'=拉花臂"),
        DeclareLaunchArgument("speed_scale", default_value="1.0",
                              description="播放速度倍率"),
        DeclareLaunchArgument("mode", default_value="preview",
                              description="'preview'=RViz2预览 / 'debug'=仅发布 / 'action'=完整管线"),

        # ── 3 轴 RPY 可调 (Isaac Teleop 语义) ──
        DeclareLaunchArgument("roll_deg", default_value="0.0",
                              description="绕 X 轴旋转角度 (度)"),
        DeclareLaunchArgument("pitch_deg", default_value="0.0",
                              description="绕 Y 轴旋转角度 (度)"),
        DeclareLaunchArgument("yaw_deg", default_value="0.0",
                              description="绕 Z 轴旋转角度 (度)"),

        # ── Pipeline 参数 ──
        DeclareLaunchArgument("planning_group", default_value="manipulator"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("ee_link", default_value="tool_tcp"),
        DeclareLaunchArgument("cartesian_max_step", default_value="0.01"),
        DeclareLaunchArgument("cartesian_jump_threshold", default_value="0.0"),
        DeclareLaunchArgument("fraction_acceptable", default_value="0.95"),
        DeclareLaunchArgument("fraction_min_executable", default_value="0.50"),
        DeclareLaunchArgument("waypoint_sample_step", default_value="4"),
        DeclareLaunchArgument("service_timeout", default_value="15.0"),
        DeclareLaunchArgument("cartesian_timeout", default_value="60.0"),
        DeclareLaunchArgument("execution_timeout", default_value="120.0"),
        DeclareLaunchArgument("tf_retry_count", default_value="20"),
        DeclareLaunchArgument("tf_retry_interval", default_value="0.03"),

        # ── C++ Cartesian Planner (MoveIt2 plan+execute) ──
        Node(
            package="latte_cartesian_planner",
            executable="cartesian_planner_node",
            name="latte_cartesian_planner",
            output="screen",
            parameters=[{
                "planning_group": LaunchConfiguration("planning_group"),
                "base_frame": LaunchConfiguration("base_frame"),
            }],
        ),

        # ── LatteImitationNode ──
        Node(
            package="latte_imitation",
            executable="latte_imitation_node",
            name="latte_imitation",
            output="screen",
            parameters=[{
                "episode_idx": LaunchConfiguration("episode_idx"),
                "arm": LaunchConfiguration("arm"),
                "speed_scale": LaunchConfiguration("speed_scale"),
                "mode": LaunchConfiguration("mode"),
                "planning_group": LaunchConfiguration("planning_group"),
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_link": LaunchConfiguration("ee_link"),
                "cartesian_max_step": LaunchConfiguration("cartesian_max_step"),
                "cartesian_jump_threshold": LaunchConfiguration("cartesian_jump_threshold"),
                "fraction_acceptable": LaunchConfiguration("fraction_acceptable"),
                "fraction_min_executable": LaunchConfiguration("fraction_min_executable"),
                "waypoint_sample_step": LaunchConfiguration("waypoint_sample_step"),
                "service_timeout": LaunchConfiguration("service_timeout"),
                "cartesian_timeout": LaunchConfiguration("cartesian_timeout"),
                "execution_timeout": LaunchConfiguration("execution_timeout"),
                "tf_retry_count": LaunchConfiguration("tf_retry_count"),
                "tf_retry_interval": LaunchConfiguration("tf_retry_interval"),
            }],
        ),
    ])
