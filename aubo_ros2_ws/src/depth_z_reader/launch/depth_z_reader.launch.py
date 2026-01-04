from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/depth/image_raw',
            description='深度图像话题名称'
        ),
        DeclareLaunchArgument(
            'pixel_x',
            default_value='-1',
            description='要读取的像素 x 坐标，-1 表示图像中心'
        ),
        DeclareLaunchArgument(
            'pixel_y',
            default_value='-1',
            description='要读取的像素 y 坐标，-1 表示图像中心'
        ),
        DeclareLaunchArgument(
            'depth_scale',
            default_value='0.00025',
            description='深度缩放因子（默认 0.00025，适用于 scale_unit=0.25 的相机，将原始值转换为米）。如果相机 scale_unit=1.0，使用 0.001；如果 scale_unit=0.25，使用 0.00025。查看相机启动日志中的 "Depth stream scale unit" 来确定正确的值。'
        ),
        DeclareLaunchArgument(
            'publish_center_depth',
            default_value='true',
            description='是否发布中心点深度值到话题'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='发布深度值的频率（Hz）'
        ),
        DeclareLaunchArgument(
            'search_valid_depth',
            default_value='true',
            description='如果中心点深度无效，是否在附近搜索有效深度值'
        ),
        DeclareLaunchArgument(
            'search_radius',
            default_value='50',
            description='搜索有效深度值的半径（像素）'
        ),

        # 深度 z 读取节点
        Node(
            package='depth_z_reader',
            executable='depth_z_reader_node',
            name='depth_z_reader_node',
            output='screen',
            parameters=[{
                'depth_image_topic': LaunchConfiguration('depth_image_topic'),
                'pixel_x': LaunchConfiguration('pixel_x'),
                'pixel_y': LaunchConfiguration('pixel_y'),
                'depth_scale': LaunchConfiguration('depth_scale'),
                'publish_center_depth': LaunchConfiguration('publish_center_depth'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'search_valid_depth': LaunchConfiguration('search_valid_depth'),
                'search_radius': LaunchConfiguration('search_radius'),
            }]
        ),
    ])

