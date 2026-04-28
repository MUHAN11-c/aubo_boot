"""
一键启动 vision_perception 全部两个节点: MediaPipe Holistic + YOLOv8 OBB
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        'input_topic', default_value='/camera/color/image_raw',
        description='输入图像话题'
    )

    holistic = Node(
        package='vision_perception',
        executable='mediapipe_holistic_node',
        name='mediapipe_holistic_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'model_complexity': 1,
            'enable_face': True,
            'enable_hands': True,
            'enable_pose': True,
            'publish_markers': True,
        }],
    )

    yolo_obb = Node(
        package='vision_perception',
        executable='yolo_obb_node',
        name='yolo_obb_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'model_path': '/home/mu/yolo26n-obb.pt',
            'conf_threshold': 0.25,
            'device': 'cuda:0',
            'publish_markers': True,
        }],
    )

    return LaunchDescription([
        input_topic_arg,
        holistic,
        yolo_obb,
    ])
