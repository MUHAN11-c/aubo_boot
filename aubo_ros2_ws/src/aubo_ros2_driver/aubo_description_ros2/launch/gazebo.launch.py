from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='aubo_i5',
        description='Robot model name (e.g., aubo_i5, aubo_e5, etc.)'
    )

    # Get URDF/xacro file path - try xacro first, then urdf
    urdf_xacro_path = PathJoinSubstitution([
        FindPackageShare('aubo_description_ros2'),
        'urdf',
        LaunchConfiguration('model') + '.xacro'
    ])
    
    urdf_path = PathJoinSubstitution([
        FindPackageShare('aubo_description_ros2'),
        'urdf',
        LaunchConfiguration('model') + '.urdf'
    ])

    # Robot description - try xacro first, fallback to urdf
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_xacro_path]),
        value_type=str
    )

    # Gazebo world
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Static transform publisher (base_link to base_footprint)
    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    # Spawn model in Gazebo
    # Note: spawn_entity needs the processed URDF, so we use robot_description parameter
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-topic', '/robot_description',
            '-entity', LaunchConfiguration('model')
        ],
        output='screen'
    )

    # Fake joint calibration (ROS2 equivalent)
    # Note: In ROS2, this is typically handled differently
    # You may need to use a separate node or service call
    # For now, we'll comment this out as it's not directly equivalent
    # fake_joint_calibration = Node(
    #     package='ros2topic',
    #     executable='ros2topic',
    #     name='fake_joint_calibration',
    #     arguments=['pub', '/calibrated', 'std_msgs/msg/Bool', '{data: true}']
    # )

    return LaunchDescription([
        model_arg,
        gazebo_world,
        tf_footprint_base,
        robot_state_publisher,
        spawn_model,
    ])

