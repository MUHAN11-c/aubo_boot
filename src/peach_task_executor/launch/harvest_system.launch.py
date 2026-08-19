# Copyright 2026 aubo_e5_ros2_ws authors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""启动桃子采摘完整业务栈."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, launch_arguments=None):
    """按包 share 目录包含一个 Python launch."""
    path = os.path.join(
        get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    """构造从 ros2_control 到 Web 的完整采摘系统入口."""
    hardware_mode = LaunchConfiguration('hardware_mode')
    robot_ip = LaunchConfiguration('robot_ip')
    moveit_enabled = LaunchConfiguration('moveit_enabled')
    camera_enabled = LaunchConfiguration('camera_enabled')
    extrinsics_enabled = LaunchConfiguration('extrinsics_enabled')
    hand_eye_enabled = LaunchConfiguration('hand_eye_enabled')
    hand_eye_web_enabled = LaunchConfiguration('hand_eye_web_enabled')
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_mode', default_value='sim',
            choices=['mock', 'sim', 'real'],
            description='硬件模式；默认 sim，真机须显式 hardware_mode:=real'),
        DeclareLaunchArgument(
            'robot_ip', default_value='169.254.10.98',
            description='AUBO 控制器 IP；sim/mock 模式不使用'),
        DeclareLaunchArgument(
            'moveit_enabled', default_value='true',
            description='启动 MoveIt move_group 和 RViz2'),
        DeclareLaunchArgument(
            'camera_enabled', default_value='false',
            description='启动 Percipio 相机；真机有设备时显式设为 true'),
        DeclareLaunchArgument(
            'extrinsics_enabled', default_value='true',
            description='启动手眼外参静态 TF'),
        DeclareLaunchArgument(
            'hand_eye_enabled', default_value='false',
            description='启动手眼标定采集流程'),
        DeclareLaunchArgument(
            'hand_eye_web_enabled', default_value='false',
            description='启动手眼标定 Web；仅 hand_eye_enabled 生效'),
        _include(
            'aubo_e5_bringup', 'bringup.launch.py', {
                'hardware_mode': hardware_mode,
                'robot_ip': robot_ip,
                'moveit_enabled': moveit_enabled,
                'camera_enabled': camera_enabled,
                'extrinsics_enabled': extrinsics_enabled,
                'hand_eye_enabled': hand_eye_enabled,
                'hand_eye_web_enabled': hand_eye_web_enabled,
            }),
        _include('peach_scene_perception', 'peach_pose.launch.py'),
        _include('peach_target_reconstruction', 'reconstruction.launch.py'),
        _include('peach_manipulation_skills', 'approach_grasp.launch.py'),
        _include('peach_observability', 'observability.launch.py'),
        _include('peach_task_executor', 'executor.launch.py'),
    ])
