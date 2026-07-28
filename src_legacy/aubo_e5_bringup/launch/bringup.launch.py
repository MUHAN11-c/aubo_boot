import os
import subprocess
import xml.etree.ElementTree as ET
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def launch_nodes(context):
    mock = LaunchConfiguration('use_mock_hardware').perform(context).lower() == 'true'
    real = LaunchConfiguration('enable_real_hardware').perform(context).lower() == 'true'
    allow_motion = LaunchConfiguration('allow_motion_commands').perform(context).lower() == 'true'
    host = LaunchConfiguration('server_host').perform(context)
    start_moveit = LaunchConfiguration('start_moveit').perform(context).lower() == 'true'
    if not mock and not real:
        raise RuntimeError('Set enable_real_hardware:=true when use_mock_hardware is false')
    if allow_motion and real and not start_moveit:
        raise RuntimeError('Real motion requires start_moveit:=true for collision validation')
    if allow_motion and real:
        preflight = os.path.join(
            get_package_prefix('aubo_e5_bringup'), 'lib',
            'aubo_e5_bringup', 'realtime_preflight.sh')
        subprocess.run([preflight], check=True)
    xacro_path = os.path.join(
        get_package_share_directory('aubo_description'),
        'urdf',
        'aubo_e5.urdf.xacro')
    root = ET.fromstring(xacro.process_file(xacro_path).toxml())
    joint_interfaces = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type") == "fixed":
            continue
        limit = joint.find("limit")
        lower = limit.attrib.get("lower", "-3.05")
        upper = limit.attrib.get("upper", "3.05")
        joint_interfaces.append(
            '<joint name="{}"><command_interface name="position">'
            '<param name="min">{}</param><param name="max">{}</param>'
            '</command_interface><state_interface name="position"/>'
            '<state_interface name="velocity"/></joint>'.format(
                joint.attrib['name'], lower, upper))
    ros2_control = ET.fromstring(
        '<ros2_control name="AuboE5System" type="system"><hardware>'
        '<plugin>{}</plugin><param name="enable_real_hardware">{}</param>'
        '<param name="allow_motion_commands">{}</param>'
        '<param name="state_timeout_seconds">{}</param>'
        '<param name="server_host">{}</param><param name="server_port">8899</param>'
        '<param name="tcp2can_batch_max">{}</param>'
        '</hardware>{}</ros2_control>'.format(
            'mock_components/GenericSystem' if mock else 'aubo_e5_hardware/AuboE5Hardware',
            str(real).lower(), str(allow_motion).lower(),
            LaunchConfiguration('state_timeout_seconds').perform(context), host,
            LaunchConfiguration('tcp2can_batch_max').perform(context),
            ''.join(joint_interfaces)))
    root.append(ros2_control)
    description = ET.tostring(root, encoding='unicode')
    params = [{'robot_description': description}]
    config = os.path.join(get_package_share_directory('aubo_e5_bringup'), 'config', 'controllers.yaml')
    nodes = [Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=params, output='screen'),
             Node(package='controller_manager', executable='ros2_control_node', parameters=params + [config], output='screen')]
    controller_names = ['joint_state_broadcaster']
    if allow_motion:
        controller_names.append('aubo_e5_arm_controller')
    if allow_motion:
        nodes.append(Node(
            package='aubo_e5_bringup', executable='named_pose_controller.py',
            name='aubo_named_pose_controller', output='screen'))
    for name in controller_names:
        nodes.append(Node(package='controller_manager', executable='spawner', arguments=[name, '--controller-manager', '/controller_manager'], output='screen'))
    return nodes

def generate_launch_description():
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('percipio_camera'),
            'launch', 'percipio_camera.launch.py')),
        condition=IfCondition(LaunchConfiguration('camera_enabled')))
    extrinsics = Node(
        package='aubo_hand_eye_calibration',
        executable='extrinsics_publisher',
        name='hand_eye_extrinsics_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('camera_enabled')))
    calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('aubo_hand_eye_calibration'),
            'launch', 'hand_eye_calibration.launch.py')),
        launch_arguments={
            'camera_enabled': 'false',
            'extrinsics_enabled': 'false',
            'web_enabled': LaunchConfiguration('hand_eye_web_enabled'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('hand_eye_enabled')))
    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('enable_real_hardware', default_value='false'),
        DeclareLaunchArgument('allow_motion_commands', default_value='false'),
        DeclareLaunchArgument('state_timeout_seconds', default_value='3.0'),
        DeclareLaunchArgument('tcp2can_batch_max', default_value='8'),
        DeclareLaunchArgument('start_moveit', default_value='false'),
        DeclareLaunchArgument('server_host', default_value='169.254.10.98'),
        DeclareLaunchArgument('camera_enabled', default_value='false'),
        DeclareLaunchArgument('hand_eye_enabled', default_value='false'),
        DeclareLaunchArgument('hand_eye_web_enabled', default_value='true'),
        OpaqueFunction(function=launch_nodes),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('aubo_e5_moveit_config'),
                'launch', 'moveit.launch.py')),
            launch_arguments={'standalone_state_publishers': 'false'}.items(),
            condition=IfCondition(LaunchConfiguration('start_moveit'))),
        camera_launch,
        extrinsics,
        calibration_launch,
    ])
