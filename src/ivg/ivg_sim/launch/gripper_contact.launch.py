"""Isolate original gripper opening, contact, lift and release in Gazebo."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ivg_sim.arm_description import arm_xacro_path, worlds_dir
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    urdf = xacro.process_file(str(arm_xacro_path().with_name(
        'gripper_contact_fixture.xacro'))).toxml()
    resources = str(Path(get_package_share_directory('ivg_description')).parent)
    nodes = [
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', str(worlds_dir() / 'gripper_contact.sdf')],
                       additional_env={'GZ_SIM_RESOURCE_PATH': resources}, output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/ivg_fixture/lift@std_msgs/msg/Float64]gz.msgs.Double'],
             output='screen'),
        TimerAction(period=1.0, actions=[Node(
            package='ros_gz_sim', executable='create',
            arguments=['-topic', 'robot_description', '-name', 'gripper_fixture',
                       '-world', 'ivg_gripper_test'], output='screen')]),
    ]
    for index, controller in enumerate(('joint_state_broadcaster',
                                       'gripper_effort_controller')):
        nodes.append(TimerAction(period=2.0 + index, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=[controller], output='screen')]))
    return LaunchDescription(nodes)
