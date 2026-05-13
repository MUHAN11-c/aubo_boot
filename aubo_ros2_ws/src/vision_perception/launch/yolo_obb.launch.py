"""
启动 YOLO26 OBB 节点
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
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='',
        description='YOLO26 OBB 模型权重路径（留空则使用包内默认 yolo26n-obb.pt）'
    )
    conf_arg = DeclareLaunchArgument(
        'conf_threshold', default_value='0.25',
        description='置信度阈值'
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda:0',
        description='推理设备 (cuda:0 / cpu)'
    )

    node = Node(
        package='vision_perception',
        executable='yolo_obb_node',
        name='yolo_obb_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'model_path': LaunchConfiguration('model_path'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'device': LaunchConfiguration('device'),
            'iou_threshold': 0.7,
            'publish_markers': True,
        }],
    )

    return LaunchDescription([
        input_topic_arg,
        model_path_arg,
        conf_arg,
        device_arg,
        node,
    ])
