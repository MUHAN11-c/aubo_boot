"""发布拉花末端笛卡尔轨迹（PoseStamped + Path）。"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    episode_idx = LaunchConfiguration("episode_idx", default="0")
    speed_scale = LaunchConfiguration("speed_scale", default="1.0")
    local_parquet = LaunchConfiguration("local_parquet", default="")

    return LaunchDescription([
        DeclareLaunchArgument("episode_idx", default_value="0"),
        DeclareLaunchArgument("speed_scale", default_value="1.0"),
        DeclareLaunchArgument("local_parquet", default_value=""),
        Node(
            package="latte_imitation",
            executable="latte_imitation_node",
            name="latte_imitation",
            output="screen",
            parameters=[{
                "episode_idx": episode_idx,
                "speed_scale": speed_scale,
                "local_parquet": local_parquet,
            }],
        ),
    ])
