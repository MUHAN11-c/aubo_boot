"""
内参标定: vendored camera_calibration 的交互式棋盘格标定器.

需先启动相机前端 (percipio 或 stereo) 且有 X 显示 (OpenCV GUI)。
棋盘格参数与手眼标定共用同一块板: 默认值运行时取自包内
config/calibration.yaml (单源, 换板只改那一处), launch 参数可覆盖。

流程: GUI 中采满进度条 -> CALIBRATE -> SAVE (写出 /tmp/calibrationdata.tar.gz)
-> ros2 run aubo_hand_eye_calibration apply_intrinsics /tmp/calibrationdata.tar.gz
COMMIT 按钮不可用: 两相机前端均无 set_camera_info 服务。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _calibration_params():
    """读手眼标定参数文件 (config/calibration.yaml) 的板参数单源默认值."""
    share = get_package_share_directory('aubo_hand_eye_calibration')
    path = os.path.join(share, 'config', 'calibration.yaml')
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)['/**']['ros__parameters']


def generate_launch_description():
    params = _calibration_params()
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/color/image_raw',
            description='彩色原始图像话题 (相机前端需已启动)'),
        DeclareLaunchArgument(
            'camera_name', default_value='camera_color',
            description='写入标定文件的相机名'),
        DeclareLaunchArgument(
            'board_columns', default_value=str(params['board_columns']),
            description='棋盘格内角点列数 (默认取手眼标定 calibration.yaml)'),
        DeclareLaunchArgument(
            'board_rows', default_value=str(params['board_rows']),
            description='棋盘格内角点行数 (默认取手眼标定 calibration.yaml)'),
        DeclareLaunchArgument(
            'board_square_size_m',
            default_value=str(params['board_square_size_m']),
            description='棋盘格格宽 (m, 需实测; 默认取手眼标定 calibration.yaml)'),
        DeclareLaunchArgument(
            'k_coefficients', default_value='2',
            description='径向畸变系数个数 (2 => plumb_bob 5 系数)'),
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='cameracalibrator',
            output='screen',
            # percipio/peach_stereo 无 set_camera_info 服务: 跳过启动期检查,
            # 结果走 SAVE + apply_intrinsics 落盘 (COMMIT 不可用)
            arguments=[
                '--no-service-check',
                '--size', [
                    LaunchConfiguration('board_columns'), 'x',
                    LaunchConfiguration('board_rows')],
                '--square', LaunchConfiguration('board_square_size_m'),
                '--camera_name', LaunchConfiguration('camera_name'),
                '--k-coefficients', LaunchConfiguration('k_coefficients'),
            ],
            remappings=[
                ('image', LaunchConfiguration('image_topic')),
            ],
        ),
    ])
