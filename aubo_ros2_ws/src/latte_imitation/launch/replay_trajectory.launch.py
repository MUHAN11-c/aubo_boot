"""发布笛卡尔末端轨迹（debug）或规划并执行（action）。"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    episode_idx = LaunchConfiguration("episode_idx", default="0")
    speed_scale = LaunchConfiguration("speed_scale", default="1.0")
    mode = LaunchConfiguration("mode", default="debug")
    arm = LaunchConfiguration("arm", default="right")

    return LaunchDescription([
        DeclareLaunchArgument("episode_idx", default_value="0",
                              description="Episode 编号 (0-39)"),
        DeclareLaunchArgument("arm", default_value="right",
                              description="'left'=左臂持杯, 'right'=右臂拉花"),
        DeclareLaunchArgument("speed_scale", default_value="1.0",
                              description="播放速度倍率"),
        DeclareLaunchArgument("mode", default_value="debug",
                              description="'debug'=发布 PoseStamped/Path; "
                                          "'action'=MoveIt2 CartesianPath→Execute"),
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
        Node(
            package="latte_imitation",
            executable="latte_imitation_node",
            name="latte_imitation",
            output="screen",
            parameters=[{
                "episode_idx": episode_idx,
                "arm": arm,
                "speed_scale": speed_scale,
                "mode": mode,
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
