from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_grasp_worker(context, *args, **kwargs):
    """LaunchConfiguration 在列表里会变成 string，节点期望 double_array；在此解析为 float。"""
    gx = float(LaunchConfiguration('grasp_x').perform(context))
    gy = float(LaunchConfiguration('grasp_y').perform(context))
    gz = float(LaunchConfiguration('grasp_z').perform(context))
    qx = float(LaunchConfiguration('grasp_qx').perform(context))
    qy = float(LaunchConfiguration('grasp_qy').perform(context))
    qz = float(LaunchConfiguration('grasp_qz').perform(context))
    qw = float(LaunchConfiguration('grasp_qw').perform(context))

    execute_grasp_pose_worker_node = Node(
        package='demo_driver',
        executable='execute_grasp_pose_worker_node',
        name='execute_grasp_pose_worker',
        output='screen',
        parameters=[{
            'egp_grasp_position': [gx, gy, gz],
            'egp_grasp_orientation': [qx, qy, qz, qw],
            'egp_grasp_z_offset': 0.01,
            'egp_height_above': 0.1,
            'egp_joint_velocity_scaling': 1.0,
            'egp_joint_acceleration_scaling': 0.1,
            'egp_home_velocity_scaling': 0.7,
            'egp_home_acceleration_scaling': 0.45,
            'egp_gripper_io_index': 7,
            'egp_lift_offset': 0.2,
            'egp_place_mode': 'home_offset',
            'egp_place_offset_y': -0.2,
            'egp_place_offset_z': -0.15,
            'egp_cartesian_max_points': 50,
            'egp_place_pose': [0.4, 0.0, 0.45, 0.0, 1.0, 0.0, 0.0],
            'egp_place_joints': [1.210212, 0.129677, 1.925533, 0.225356, 1.571783, 1.209540],
        }],
    )
    return [execute_grasp_pose_worker_node]


def generate_launch_description():
    grasp_x_arg = DeclareLaunchArgument('grasp_x', default_value='0.41176897287368774', description='抓取位置 X 坐标 (m)')
    grasp_y_arg = DeclareLaunchArgument('grasp_y', default_value='0.18364354968070984', description='抓取位置 Y 坐标 (m)')
    grasp_z_arg = DeclareLaunchArgument('grasp_z', default_value='0.2553410828113556', description='抓取位置 Z 坐标 (m)')
    grasp_qx_arg = DeclareLaunchArgument('grasp_qx', default_value='0.706803', description='抓取姿态四元数 X')
    grasp_qy_arg = DeclareLaunchArgument('grasp_qy', default_value='0.707410', description='抓取姿态四元数 Y')
    grasp_qz_arg = DeclareLaunchArgument('grasp_qz', default_value='0.000126', description='抓取姿态四元数 Z')
    grasp_qw_arg = DeclareLaunchArgument('grasp_qw', default_value='-0.000515', description='抓取姿态四元数 W')

    return LaunchDescription([
        grasp_x_arg,
        grasp_y_arg,
        grasp_z_arg,
        grasp_qx_arg,
        grasp_qy_arg,
        grasp_qz_arg,
        grasp_qw_arg,
        OpaqueFunction(function=_launch_grasp_worker),
    ])
