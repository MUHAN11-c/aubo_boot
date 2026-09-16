"""Thin forwarder: harvest_system entry now lives in peach_bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_FORWARD = (
    ('hardware_mode', 'mock'),
    ('robot_ip', '169.254.10.98'),
    ('tool_profile', 'adaptive_cylinder_v1'),
    ('moveit_enabled', 'true'),
    ('camera_enabled', 'false'),
    ('extrinsics_enabled', 'true'),
    ('hand_eye_enabled', 'false'),
    ('hand_eye_web_enabled', 'false'),
    ('imu_enabled', 'true'),
)


def generate_launch_description():
    """转发到 peach_bringup/harvest_system.launch.py；不自动 RunHarvest."""
    declared = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in _FORWARD]
    forwarded = {name: LaunchConfiguration(name) for name, _ in _FORWARD}
    return LaunchDescription(declared + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('peach_bringup'),
                    'launch', 'harvest_system.launch.py'])),
            launch_arguments=forwarded.items()),
    ])
