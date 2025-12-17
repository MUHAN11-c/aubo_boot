from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    moveit_warehouse_port_arg = DeclareLaunchArgument(
        'moveit_warehouse_port',
        default_value='33829',
        description='The default DB port for moveit (not default MongoDB port to avoid potential conflicts)'
    )
    
    moveit_warehouse_host_arg = DeclareLaunchArgument(
        'moveit_warehouse_host',
        default_value='localhost',
        description='The default DB host for moveit'
    )

    # Set warehouse parameters
    warehouse_port_param = SetParameter(
        name='warehouse_port',
        value=LaunchConfiguration('moveit_warehouse_port')
    )
    
    warehouse_host_param = SetParameter(
        name='warehouse_host',
        value=LaunchConfiguration('moveit_warehouse_host')
    )
    
    warehouse_exec_param = SetParameter(
        name='warehouse_exec',
        value='mongod'
    )
    
    warehouse_plugin_param = SetParameter(
        name='warehouse_plugin',
        value='warehouse_ros_mongo::MongoDatabaseConnection'
    )

    return LaunchDescription([
        moveit_warehouse_port_arg,
        moveit_warehouse_host_arg,
        warehouse_port_param,
        warehouse_host_param,
        warehouse_exec_param,
        warehouse_plugin_param,
    ])

