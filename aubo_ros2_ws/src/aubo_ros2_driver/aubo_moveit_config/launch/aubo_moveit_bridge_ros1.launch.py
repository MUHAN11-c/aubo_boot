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
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
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

    # Planning context
    # 直接使用 ros2_control 版本的 xacro
    robot_xacro_file_to_use = "aubo_ros2.xacro"
    
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
    joint_names_yaml = {
        "joint_name" : load_yaml(
          "aubo_moveit_config", "config/joint_names.yaml"
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
        # 增加超时缩放因子以提供足够的执行时间余量
        # 设置为 3.0 以提供足够的余量
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
    # static_tf_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    # )

    # Publish TF
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
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
        #aubo ros2 tarjectory action
    aubo_trajectory_action_node = Node(
        package="aubo_ros2_trajectory_action",
        executable="aubo_ros2_trajectory_action",
        output="screen",
        parameters=[
            joint_names_yaml,
        ],
    )
    
    # Feedback bridge node: 将 aubo_msgs/msg/JointTrajectoryFeedback 转换为 
    # control_msgs/action/FollowJointTrajectory_Feedback
    # 同时桥接 joint_states 消息（从 ROS1 转发到 ROS2）
    feedback_bridge_node = Node(
        package="feedback_bridge",
        executable="feedback_bridge_node",
        name="feedback_bridge",
        output="screen",
        parameters=[{
            # Feedback 消息桥接参数
            "feedback_input_topic": "feedback_states",  # 从 ROS 1 桥接过来的话题
            "feedback_output_topic": "aubo/feedback_states",  # 发布给 ROS 2 节点的话题（与 aubo_ros2_trajectory_action 订阅的话题一致）
            # Joint States 消息桥接参数
            "joint_states_input_topic": "joint_states",  # 从 ROS 1 桥接过来的 joint_states 话题
            "joint_states_output_topic": "joint_states",  # 发布给 ROS 2 节点的 joint_states 话题
            "enable_joint_states_bridge": True,  # 启用 joint_states 桥接
        }],
    )
    
    # 注意：在桥接 ROS 1 的场景中，不需要启动 ROS 2 Control 的 joint_trajectory_controller
    # 因为 aubo_ros2_trajectory_action 会监听 /joint_trajectory_controller/follow_joint_trajectory
    # 并将轨迹发布到 /moveItController_cmd，由 ROS 1 端的 aubo_driver 执行
    # 如果同时启动 joint_trajectory_controller，会导致 action server 冲突
    # trajectory_controller_spawner = TimerAction(
    #     period=3.0,
    #     actions=[
    #         ExecuteProcess(
    #             cmd=["ros2 run controller_manager spawner.py joint_trajectory_controller --controller-manager-timeout 20"],
    #             shell=True,
    #             output="screen",
    #         )
    #     ]
    # )
    
    nodes_to_start = [
        # 注意：虽然不使用 ROS 2 Control 的 joint_trajectory_controller，
        # 但 MoveIt 可能仍需要 controller_manager 来管理控制器状态
        # 如果 MoveIt 报错找不到 controller_manager，可以取消下面的注释
        # controller_manager_node,
        # trajectory_controller_spawner,  # 不需要启动 joint_trajectory_controller
        aubo_trajectory_action_node,  # 这个节点会监听 /joint_trajectory_controller/follow_joint_trajectory
        feedback_bridge_node,
    ]
    
    # 添加其他必需的节点
    nodes_to_start.extend([
        robot_state_pub_node,
        # static_tf_node,
        move_group_node,
        rviz_node,
        # 注意：Demo Driver 服务节点已分离到独立的 launch 文件
        # 使用以下命令单独启动：
        # ros2 launch aubo_moveit_config demo_driver_services.launch.py
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
