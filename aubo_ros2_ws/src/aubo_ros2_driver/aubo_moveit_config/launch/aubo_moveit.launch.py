from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    # Command-line arguments
    robot_xacro_file = LaunchConfiguration("robot_xacro_file")
    support_package = LaunchConfiguration("support_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    aubo_type = LaunchConfiguration("aubo_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    # Planning context
    # 根据use_ros2_control参数选择使用ros2_control版本的xacro
    use_ros2_control_val = use_ros2_control.perform(context)
    if use_ros2_control_val == "true":
        robot_xacro_file_to_use = "aubo_ros2.xacro"
    else:
        robot_xacro_file_to_use = robot_xacro_file.perform(context)
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(support_package), "urdf/xacro/inc/", robot_xacro_file_to_use]
            ),
            " ",
            "aubo_type:=",
             aubo_type,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", moveit_config_file]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content.perform(
            context
        )
    }

    kinematics_yaml = load_yaml(
        "aubo_moveit_config", "config/kinematics.yaml"
    )

    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "aubo_moveit_config", "config/joint_limits.yaml"
        )
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
            "sample_duration": 0.005,
        }
    }
    ompl_planning_yaml = load_yaml(
        "aubo_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    # MoveIt 总是使用 moveit_controllers.yaml 配置文件
    # ros2_controllers.yaml 是给 controller_manager 节点用的，不是给 MoveIt 用的
    moveit_simple_controllers_yaml = load_yaml(
        "aubo_moveit_config", "config/moveit_controllers.yaml"
    )
    if moveit_simple_controllers_yaml is None:
        raise RuntimeError(
            "moveit_controllers.yaml could not be loaded. "
            "Please ensure the file exists."
        )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        # MoveIt does not handle controller switching automatically
        "moveit_manage_controllers": False,
        # 增加超时缩放因子以支持速度缩放（velocity_scale_factor = 0.5 时，执行时间会翻倍）
        # 原始时间 1.355s * 缩放因子 0.5 = 2.71s，需要至少 2.71 / 1.355 = 2.0 倍的缩放
        "trajectory_execution.allowed_execution_duration_scaling": 3.0,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/move_group/planning_scene_monitor",
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
           }
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
        ],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("aubo_moveit_config"), "config"
    )
    rviz_config = os.path.join(rviz_base, "view_robot.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    nodes_to_start = []
    
    # 根据use_ros2_control参数决定使用哪种控制方式
    if use_ros2_control_val == "true":
        # ROS2 Control 控制器管理器节点（参考 moveit2_tutorials demo.launch.py）
        ros2_controllers_path = os.path.join(
            get_package_share_directory("aubo_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        )
        controller_manager_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, ros2_controllers_path],
            output="screen",
        )
        
        # 启动控制器（参考 moveit2_tutorials demo.launch.py 的方式）
        # 使用 TimerAction 延迟启动控制器，增加超时时间以确保controller_manager已启动
        joint_state_broadcaster_spawner = TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2 run controller_manager spawner.py joint_state_broadcaster --controller-manager-timeout 20"],
                    shell=True,
                    output="screen",
                )
            ]
        )

        trajectory_controller_spawner = TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2 run controller_manager spawner.py joint_trajectory_controller --controller-manager-timeout 20"],
                    shell=True,
                    output="screen",
                )
            ]
        )
        
        nodes_to_start = [
            controller_manager_node,
            joint_state_broadcaster_spawner,
            trajectory_controller_spawner,
        ]
    else:
        # 使用传统的joint_state_publisher（用于测试和演示）
        joint_state_publisher_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[robot_description],
            output="screen",
        )
        nodes_to_start = [joint_state_publisher_node]
    
    # 添加其他必需的节点
    nodes_to_start.extend([
        robot_state_pub_node,
        static_tf_node,
        move_group_node,
        rviz_node,
    ])
    
    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    # TODO(andyz): add other options
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_xacro_file",
            default_value="aubo.xacro",
            description="Xacro describing the robot.",
            choices=["aubo.xacro"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "support_package",
            default_value="aubo_description",
            description="Name of the support package",
            choices=["aubo_description"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="aubo_moveit_config",
            description="Name of the support package",
            choices=["aubo_moveit_config"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            description="Name of the SRDF file",
            default_value="aubo_i5.srdf",
            choices=["aubo_i5.srdf"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            description="Name of the aubo_type",
            default_value="aubo_e5_10",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Whether to use fake hardware (simulation) or real hardware",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="true",
            description="Whether to use ros2_control (requires ros-foxy-controller-manager package)",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
