from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui'
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

    # Joint state publisher (without GUI)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
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

    # RViz (optional - can be started separately)
    # rviz_config_path = PathJoinSubstitution([
    #     FindPackageShare('aubo_description_ros2'),
    #     'rviz',
    #     'display.rviz'
    # ])

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_path],
    #     output='screen'
    # )

    return LaunchDescription([
        model_arg,
        gui_arg,
        joint_state_publisher,
        joint_state_publisher_gui,
        robot_state_publisher,
        # rviz,  # Uncomment if you have a rviz config file
    ])

