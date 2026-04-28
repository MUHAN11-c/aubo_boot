"""
视频输入测试: video_publisher → MediaPipe Holistic + YOLO26 OBB + 实时显示
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('vision_perception')
    default_video = os.path.join(pkg_share, 'resource',
                                 'video_新手咖啡拉花练习顺序_0.mp4')

    video_path_arg = DeclareLaunchArgument(
        'video_path', default_value=default_video,
        description='输入视频文件路径')
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', default_value='/camera/color/image_raw',
        description='视频发布的话题名')
    fps_arg = DeclareLaunchArgument(
        'fps', default_value='15.0', description='发布帧率')
    resize_arg = DeclareLaunchArgument(
        'resize_height', default_value='480', description='缩放高度')

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

    holistic = Node(
        package='vision_perception',
        executable='mediapipe_holistic_node',
        name='mediapipe_holistic_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('output_topic'),
            'model_complexity': 1,
            'enable_face': False,
            'enable_hands': True,
            'enable_pose': True,
            'publish_markers': True,
            'publish_arm_data': True,
        }],
    )

    yolo_obb = Node(
        package='vision_perception',
        executable='yolo_obb_node',
        name='yolo_obb_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('output_topic'),
            'model_path': '/home/mu/yolo26n-obb.pt',
            'conf_threshold': 0.25,
            'device': 'cpu',
            'publish_markers': True,
        }],
    )

    holistic_view = Node(
        package='image_view',
        executable='image_view',
        name='holistic_view',
        output='screen',
        remappings=[('image', '/vision/holistic/image')],
        parameters=[{'window_name': 'MediaPipe Holistic (全身)', 'autosize': True}],
    )

    arm_view = Node(
        package='image_view',
        executable='image_view',
        name='arm_view',
        output='screen',
        remappings=[('image', '/vision/holistic/arm_view')],
        parameters=[{'window_name': 'Arm/Wrist 局部 + 角度', 'autosize': True}],
    )

    obb_view = Node(
        package='image_view',
        executable='image_view',
        name='obb_view',
        output='screen',
        remappings=[('image', '/vision/yolo_obb/image')],
        parameters=[{'window_name': 'YOLO26 OBB', 'autosize': True}],
    )

    return LaunchDescription([
        video_path_arg, output_topic_arg, fps_arg, resize_arg,
        video_pub, holistic, yolo_obb,
        holistic_view, arm_view, obb_view,
    ])
