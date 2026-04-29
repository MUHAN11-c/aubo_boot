"""
video_publisher + YOLO track 组合启动
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('vision_perception')

    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value=os.path.join(pkg_share, 'resource',
                                   'video_新手咖啡拉花练习顺序_0.mp4'),
        description='输入视频文件路径')
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', default_value='/camera/color/image_raw',
        description='视频发布的话题名')
    fps_arg = DeclareLaunchArgument(
        'fps', default_value='0.0', description='发布帧率 (0=原始帧率)')
    resize_arg = DeclareLaunchArgument(
        'resize_height', default_value='480', description='缩放高度')
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_share, 'model', 'yolo26n.pt'),
        description='YOLO 模型权重路径')
    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda:0',
        description='推理设备 (cuda:0 / cpu)')
    conf_arg = DeclareLaunchArgument(
        'conf_threshold', default_value='0.3',
        description='置信度阈值')

    video_pub = Node(
        package='vision_perception',
        executable='video_publisher_node',
        name='video_publisher_node',
        output='screen',
        parameters=[{
            'video_path': LaunchConfiguration('video_path'),
            'output_topic': LaunchConfiguration('output_topic'),
            'fps': LaunchConfiguration('fps'),
            'resize_height': LaunchConfiguration('resize_height'),
            'loop': True,
        }],
    )

    yolo_track = Node(
        package='vision_perception',
        executable='yolo_track_node',
        name='yolo_track_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('output_topic'),
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'display_result': True,
        }],
    )

    return LaunchDescription([
        video_path_arg, output_topic_arg, fps_arg, resize_arg,
        model_path_arg, device_arg, conf_arg,
        video_pub, yolo_track,
    ])
