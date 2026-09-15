"""
imu_follow servo 集成 launch：起 moveit_servo + 跟随节点（mock 主入口）.

bringup/move_group 由整栈或 bringup.launch.py 另行提供（前置）。servo_node
以 aubo_e5_moveit_config 的 MoveItConfigsBuilder 装配模型/运动学/限位，
参数取本包 config/moveit_servo.yaml；跟随节点参数取 config/imu_follow.yaml
且 backend 固定 servo、twist 输入指向 /moveit_servo/delta_twist_cmds。
tool_profile 透传进 robot_description（与整栈同档案，防 servo 模型分叉）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_configs(tool_profile: str):
    """与 aubo_e5_moveit_config/moveit.launch.py 同链的模型装配（带工具档案）."""
    return (
        MoveItConfigsBuilder(
            'aubo_e5', package_name='aubo_e5_moveit_config')
        .robot_description(mappings={'tool_profile': tool_profile})
        .to_moveit_configs())


def launch_setup(context):
    """tool_profile 须 perform 后再拼 Builder（xacro 映射构建期展开）."""
    tool_profile = LaunchConfiguration('tool_profile').perform(context)
    mc = _moveit_configs(tool_profile)
    return [
        Node(
            package='moveit_servo',
            executable='servo_node',
            name='moveit_servo',
            output='screen',
            parameters=[
                ParameterFile(LaunchConfiguration('servo_params_file'),
                              allow_substs=True),
                mc.robot_description,
                mc.robot_description_semantic,
                mc.robot_description_kinematics,
                mc.joint_limits,
            ],
        ),
        Node(
            package='imu_follow',
            executable='imu_follow_node',
            name='imu_follow',
            output='screen',
            emulate_tty=True,
            parameters=[
                ParameterFile(
                    LaunchConfiguration('imu_follow_params_file'),
                    allow_substs=True),
                {'motion.backend': 'servo',
                 'topics.imu_topic': LaunchConfiguration('imu_topic'),
                 'motion.enabled': ParameterValue(
                     LaunchConfiguration('motion_enabled'), value_type=bool)},
            ],
        ),
    ]


def generate_launch_description():
    """servo_node + imu_follow；motion_enabled 覆盖运动门（默认 false）."""
    share = FindPackageShare('imu_follow')
    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_follow_params_file',
            default_value=PathJoinSubstitution([share, 'config', 'imu_follow.yaml']),
            description='跟随节点参数文件；默认包内 config/imu_follow.yaml'),
        DeclareLaunchArgument(
            'servo_params_file',
            default_value=PathJoinSubstitution([share, 'config', 'moveit_servo.yaml']),
            description='moveit_servo 参数文件；默认包内 config/moveit_servo.yaml'),
        DeclareLaunchArgument(
            'motion_enabled', default_value='false',
            description='true=向 servo twist 输入发运动指令；真机须另行人工授权'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu/data',
            description='IMU 输入话题；无真 IMU 的受控演示可指假流话题'),
        DeclareLaunchArgument(
            'tool_profile', default_value='adaptive_cylinder_v1',
            choices=['hollow_cylinder_v1', 'adaptive_cylinder_v1'],
            description='末端工具档案（servo 模型 TCP 随档案；须与整栈同值）'),
        OpaqueFunction(function=launch_setup),
    ])
