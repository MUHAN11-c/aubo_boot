"""imu_follow 独立 launch：只起跟随节点（bringup/move_group 由整栈另行提供）."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """装载 config/imu_follow.yaml；motion_enabled 覆盖运动门（默认 false）."""
    share = FindPackageShare('imu_follow')
    default_params = PathJoinSubstitution([share, 'config', 'imu_follow.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_follow_params_file', default_value=default_params,
            description='参数文件；默认包内 config/imu_follow.yaml'),
        DeclareLaunchArgument(
            'motion_enabled', default_value='false',
            description='true=向 execution.follow_joint_trajectory_action '
                        '流式下发 FJT；真机须另行人工授权'),
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
                {'motion.enabled': ParameterValue(
                    LaunchConfiguration('motion_enabled'), value_type=bool)},
            ],
        ),
    ])
