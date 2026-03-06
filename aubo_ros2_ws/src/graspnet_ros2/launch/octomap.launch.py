"""可选：启动 octomap_server，将 /graspnet_pointcloud 转为 /octomap_full，供 RViz OccupancyGrid 显示。避障由 move_group 的 sensors_3d 完成，本 launch 仅用于可视化。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[
                {
                    'frame_id': 'camera_frame',
                    'base_frame_id': 'base_link',
                    'sensor_model': 'beam',
                    'resolution': 0.02,
                    'max_range': 2.0,
                    'color': False,
                }
            ],
            remappings=[('/cloud_in', '/graspnet_pointcloud')],
        ),
    ])
