"""
启动 MediaPipe Holistic 节点
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
    model_complexity_arg = DeclareLaunchArgument(
        'model_complexity', default_value='1',
        description='模型复杂度 (0/1/2)'
    )
    min_detection_arg = DeclareLaunchArgument(
        'min_detection_confidence', default_value='0.5',
        description='最低检测置信度'
    )
    min_tracking_arg = DeclareLaunchArgument(
        'min_tracking_confidence', default_value='0.5',
        description='最低跟踪置信度'
    )

    node = Node(
        package='vision_perception',
        executable='mediapipe_holistic_node',
        name='mediapipe_holistic_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'model_complexity': LaunchConfiguration('model_complexity'),
            'min_detection_confidence': LaunchConfiguration('min_detection_confidence'),
            'min_tracking_confidence': LaunchConfiguration('min_tracking_confidence'),
            'enable_face': True,
            'enable_hands': True,
            'enable_pose': True,
            'publish_markers': True,
        }],
    )

    return LaunchDescription([
        input_topic_arg,
        model_complexity_arg,
        min_detection_arg,
        min_tracking_arg,
        node,
    ])
