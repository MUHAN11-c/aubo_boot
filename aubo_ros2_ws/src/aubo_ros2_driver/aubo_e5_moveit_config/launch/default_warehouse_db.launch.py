from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    moveit_warehouse_database_path_arg = DeclareLaunchArgument(
        'moveit_warehouse_database_path',
        default_value='',
        description='Path to the MoveIt warehouse database'
    )

    # Warehouse database node
    # Note: In ROS2 MoveIt2, warehouse functionality may be different
    # This is a placeholder for warehouse database functionality
    warehouse_db_node = Node(
        package='moveit_ros_warehouse',
        executable='moveit_warehouse_services',
        name='moveit_warehouse_services',
        output='screen',
        parameters=[{
            'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection',
            'warehouse_host': 'localhost',
            'warehouse_port': 33829,
            'warehouse_exec': 'mongod',
            'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection',
            'warehouse_db': LaunchConfiguration('moveit_warehouse_database_path')
        }]
    )

    return LaunchDescription([
        moveit_warehouse_database_path_arg,
        warehouse_db_node,
    ])

