"""
启动咖啡拉花工作流节点 + 拉花 IO 桥.

依赖外部已起 move_group 与控制器栈（如 aubo_e5_bringup + moveit）。
适配自 aubo_boot latte_workflow.launch.py（2026-09-30）：
  - MoveIt 配置指向本仓 aubo_e5_moveit_config（aubo_e5.srdf / kinematics.yaml）
  - urdf 由 robot_state_publisher 的 robot_description 参数透传，不重复加载
  - 不再内置 subprocess 轮询 /joint_states（由外部栈保证就绪）

用法:
  ros2 launch latte_backend latte_workflow.launch.py io_simulated:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    tool_profile = LaunchConfiguration('tool_profile')
    io_simulated = LaunchConfiguration('io_simulated')

    moveit_config = (
        MoveItConfigsBuilder('aubo_e5', package_name='aubo_e5_moveit_config')
        .robot_description(mappings={'tool_profile': tool_profile})
        .robot_description_semantic(file_path='config/aubo_e5.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .to_moveit_configs()
    )

    workflow_node = Node(
        package='latte_backend',
        executable='latte_workflow_node',
        name='latte_workflow_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                'io_simulated': io_simulated,
                'lwf_execute_latte': True,
            },
        ],
    )

    io_node = Node(
        package='latte_backend',
        executable='latte_io_node',
        name='latte_io_node',
        output='screen',
        parameters=[{'io_simulated': io_simulated}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_shear_v1',
            description='工具档案（须与 RSP/move_group 同值；本仓 xacro 仅 '
                        'shear_v1/bite_shear_v1/adaptive_shear_v1 三档）'),
        DeclareLaunchArgument(
            'io_simulated', default_value='false',
            description='mock 栈无 aubo_io_controller 时置 true（IO 旁路）'),
        workflow_node,
        io_node,
    ])
