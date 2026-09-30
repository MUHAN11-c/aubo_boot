"""
启动 gripper_swap_worker + scene_attach_worker.

适配自 aubo_boot（2026-09-30）：io_simulated 参数；gripper_swap_worker 构造
MGI，必须随发 MoveIt 参数（robot_description/SRDF/kinematics），缺参 10s
超时 FATAL（移植首版踩过）。启动时可指定初始工具:

  ros2 launch tool_changer gripper_swap_worker.launch.py \
      initial_tool_id:=gripper0 io_simulated:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('aubo_e5', package_name='aubo_e5_moveit_config')
        .robot_description(mappings={'tool_profile': LaunchConfiguration('tool_profile')})
        .robot_description_semantic(file_path='config/aubo_e5.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'initial_tool_id', default_value='',
            description='启动时预设的末端工具 ID（如 gripper0 / gripper2），空 = 无工具'),
        DeclareLaunchArgument(
            'io_simulated', default_value='false',
            description='mock 栈无 aubo_io_controller 时置 true（IO 旁路）'),
        DeclareLaunchArgument(
            'attach_frame', default_value='quick_changer_link',
            description='ACO 附着帧（本仓 xacro 帧名）'),
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_shear_v1',
            description='工具档案（须与 RSP/move_group 同值）'),
        Node(
            package='tool_changer',
            executable='gripper_swap_worker_node',
            name='gripper_swap_worker',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {
                    'initial_tool_id': LaunchConfiguration('initial_tool_id'),
                    'io_simulated': LaunchConfiguration('io_simulated'),
                },
            ],
        ),
        Node(
            package='tool_changer',
            executable='scene_attach_worker_node',
            name='scene_attach_worker',
            output='screen',
            parameters=[{'attach_frame': LaunchConfiguration('attach_frame')}],
        ),
    ])
