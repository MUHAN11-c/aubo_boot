"""发布笛卡尔末端轨迹（debug）或 IK→JointTrajectory 执行（action）。"""

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
            }],
        ),
    ])
