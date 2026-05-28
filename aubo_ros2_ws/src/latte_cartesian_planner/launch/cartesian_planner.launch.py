from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("planning_group", default_value="manipulator"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
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
    ])
