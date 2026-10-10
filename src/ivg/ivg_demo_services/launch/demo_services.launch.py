"""
启动 IVG 演示栈服务（system_monitor + move_service + grasp_trigger）.

不进 harvest_system/lifecycle；须外部已起 move_group 与控制器栈。
MoveIt 配置（robot_description/SRDF/kinematics）必须随节点下发——
MGI 构造从节点参数加载 RDF，缺参 10s 超时后 FATAL（移植首版踩过）。

用法:
  ros2 launch ivg_demo_services demo_services.launch.py io_simulated:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    io_simulated = LaunchConfiguration('io_simulated')
    planning_group = LaunchConfiguration('planning_group')
    home_target = LaunchConfiguration('home_target')

    moveit_config = (
        MoveItConfigsBuilder('aubo_e5', package_name='aubo_e5_moveit_config')
        .robot_description(mappings={'tool_profile': LaunchConfiguration('tool_profile')})
        .robot_description_semantic(file_path='config/aubo_e5.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .to_moveit_configs()
    )

    common_moveit_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'io_simulated', default_value='false',
            description='mock 栈无 aubo_io_controller 时置 true（IO 旁路成功）'),
        DeclareLaunchArgument(
            'planning_group', default_value='manipulator_e5'),
        DeclareLaunchArgument(
            'home_target', default_value='camera_pose'),
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_shear_v1',
            description='工具档案（须与 RSP/move_group 同值）'),
        Node(
            package='ivg_demo_services', executable='system_monitor_node',
            name='system_monitor_node', output='screen'),
        Node(
            package='ivg_demo_services', executable='move_service_node',
            name='move_service_node', output='screen',
            parameters=common_moveit_params + [
                {'planning_group': planning_group,
                 'home_target': home_target,
                 'io_simulated': io_simulated},
            ]),
        Node(
            package='ivg_demo_services', executable='grasp_trigger_node',
            name='grasp_trigger_node', output='screen',
            parameters=common_moveit_params + [
                {'planning_group': planning_group,
                 'home_target': home_target,
                 'io_simulated': io_simulated},
            ]),
    ])
