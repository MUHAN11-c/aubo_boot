from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("aubo_i5", package_name="aubo_moveit_config")
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    # 等价于 generate_demo_launch 的常用开关
    ld.add_action(DeclareBooleanLaunchArg("db", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("warehouse_port", default_value="33829"))
    ld.add_action(DeclareLaunchArgument("warehouse_host", default_value="localhost"))
    ld.add_action(
        DeclareLaunchArgument(
            "warehouse_plugin",
            default_value="warehouse_ros_mongo::MongoDatabaseConnection",
        )
    )

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": LaunchConfiguration("monitor_dynamics"),
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_configuration],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control + 控制器加载（与官方 demo.launch.py 结构一致）
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            output="screen",
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )
    )

    controllers_yaml = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    )
    for controller_name in controllers_yaml.get("controller_names", []):
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller_name, "-c", "/controller_manager"],
                output="screen",
            )
        )

    # warehouse_db（原 warehouse_db.launch.py 展开）
    warehouse_ros_config = {
        "warehouse_plugin": LaunchConfiguration("warehouse_plugin"),
        "warehouse_host": LaunchConfiguration("warehouse_host"),
        "warehouse_port": ParameterValue(
            LaunchConfiguration("warehouse_port"), value_type=int
        ),
    }
    warehouse_db_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[warehouse_ros_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("db")),
    )

    ld.add_action(static_tf_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(move_group_node)
    ld.add_action(rviz_node)
    ld.add_action(warehouse_db_node)

    return ld
