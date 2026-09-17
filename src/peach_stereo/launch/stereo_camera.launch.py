# peach_stereo 启动：主机侧单图案立体深度相机（percipio_camera 的替代前端，话题同构）
# 与 percipio_camera.launch.py 互斥（相机连接独占）；感知/重建节点零改动接入。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    device_ip = LaunchConfiguration('device_ip')
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_ip', default_value='169.254.10.110',
            description='PS800-E1 相机 IP'),
        Node(
            package='peach_stereo',
            executable='stereo_camera_node',
            name='peach_stereo_camera_node',
            namespace='camera',
            output='screen',
            parameters=[
                ParameterFile(PathJoinSubstitution([
                    FindPackageShare('peach_stereo'), 'config', 'stereo_camera.yaml'])),
                {'device_ip': device_ip},
            ],
        ),
    ])
