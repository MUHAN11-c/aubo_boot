"""
可选：ROS 侧「瘦点云」桥接，供网页 3D 订阅。

  ros2 launch aubo_ros2_web_dashboard pointcloud_web_bridge.launch.py

默认：/camera/depth_registered/points -> /camera/depth_registered/points_web（max_points=32000）

在 topics_lab 中将「点云话题」填输出话题，或点顶栏「3D: 瘦点云」；RViz 仍可订阅原始高清话题。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
	return LaunchDescription(
		[
			DeclareLaunchArgument(
				'input_topic',
				default_value='/camera/depth_registered/points',
				description='完整 PointCloud2 源话题',
			),
			DeclareLaunchArgument(
				'output_topic',
				default_value='/camera/depth_registered/points_web',
				description='下采样后话题（供 rosbridge / 浏览器）',
			),
			DeclareLaunchArgument(
				'max_points',
				default_value='32000',
				description='单帧最多保留点数（均匀步长抽样，覆盖整幅）',
			),
			Node(
				package='aubo_ros2_web_dashboard',
				executable='ivg_pointcloud_web_throttle',
				name='ivg_pointcloud_web_throttle',
				output='screen',
				parameters=[
					{
						'input_topic': LaunchConfiguration('input_topic'),
						'output_topic': LaunchConfiguration('output_topic'),
						'max_points': ParameterValue(LaunchConfiguration('max_points'), value_type=int),
					}
				],
			),
		]
	)
