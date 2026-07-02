"""生产默认：同时启动 gripper_swap_worker + scene_attach_worker（ACO + world 清理）喵~

启动时可指定初始工具参数:
  ros2 launch tool_changer gripper_swap_worker.launch.py initial_tool_id:=gripper0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    initial_tool_id_arg = DeclareLaunchArgument(
        "initial_tool_id", default_value="",
        description="启动时预设的末端工具 ID（如 gripper0 / gripper2），空字符串 = 无工具状态")

    return LaunchDescription([
        initial_tool_id_arg,
        Node(
            package="tool_changer",
            executable="gripper_swap_worker_node",
            name="gripper_swap_worker",
            output="screen",
            parameters=[{
                "initial_tool_id": LaunchConfiguration("initial_tool_id"),
            }],
        ),
        Node(
            package="tool_changer",
            executable="scene_attach_worker_node",
            name="scene_attach_worker",
            output="screen",
        ),
    ])
