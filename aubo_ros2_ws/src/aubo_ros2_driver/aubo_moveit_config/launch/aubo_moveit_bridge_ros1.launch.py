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

    # SRDF: load by reading file (xacro fails in ROS2 due to $(find ...) in .srdf.xacro)
    moveit_config_file_val = context.perform_substitution(moveit_config_file)
    moveit_config_package_val = context.perform_substitution(moveit_config_package)
    srdf_path = os.path.join(
        get_package_share_directory(moveit_config_package_val), "config", moveit_config_file_val
    )
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

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
            # 增大规划时间和尝试次数，避免复杂场景下 Planning request aborted（-2）
            "planning_time": 15.0,
            "max_planning_attempts": 10,
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
        # 允许时间 = 轨迹时长×scaling + margin；过大会延长异常暴露（无动作时等很久才 cancel），过小易误超时
        "trajectory_execution.allowed_execution_duration_scaling": 8.0,
        "trajectory_execution.allowed_goal_duration_margin": 3.0,
        # 放宽起始容差，避免 joint_states 与轨迹起点略偏时 MoveIt 立即 abort（0.2 rad≈11.5°，缓解 client_cancel）
        "trajectory_execution.allowed_start_tolerance": 0.2,
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

    # 点云/Octomap 传感器配置（参考 graspnet_demo.launch.py，使用 graspnet_ros2/config/sensors_3d.yaml）
    # 仅当 use_octomap:=true 时启用
    use_octomap = context.perform_substitution(LaunchConfiguration("use_octomap")).lower() == "true"
    octomap_config = {"octomap_frame": "camera_frame", "octomap_resolution": 0.01}
    octomap_updater_config = None
    if use_octomap:
        try:
            octomap_updater_config = load_yaml("graspnet_ros2", "config/sensors_3d.yaml")
        except Exception:
            pass

    move_group_parameters = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        ompl_planning_pipeline_config,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        joint_limits_yaml,
        {"publish_robot_description_semantic": True},
    ]
    if use_octomap and octomap_updater_config:
        move_group_parameters.extend([octomap_config, octomap_updater_config])

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_parameters,
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

    # 手眼静态 TF（参考 graspnet_demo.launch.py）：末端 -> camera_frame，供 Octomap/点云等使用
    hand_eye_yaml = context.perform_substitution(LaunchConfiguration("hand_eye_yaml_path"))
    ee_frame_id = context.perform_substitution(LaunchConfiguration("ee_frame_id"))
    hand_eye_static_tf_node = Node(
        package="graspnet_ros2",
        executable="hand_eye_static_tf_node",
        name="hand_eye_static_tf_node",
        output="screen",
        parameters=[{
            "hand_eye_yaml_path": hand_eye_yaml,
            "ee_frame_id": ee_frame_id,
            "child_frame_id": "camera_frame",
        }],
    )
    # 感知帧与 link 同名等价：camera_frame 与 camera_link 视为同一坐标系（0 变换，仅旋转对齐）
    camera_frame_to_camera_link_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_frame_to_camera_link",
        arguments=["0", "0", "0", "1.5708", "-1.5708", "0", "camera_frame", "camera_link"],
    )

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
    # control_msgs/action/FollowJointTrajectory_Feedback（仅在使用 ROS1 driver 时需要）
    # 使用 aubo_driver_ros2 全 ROS2 时不需要 feedback_bridge
    feedback_bridge_node = Node(
        package="feedback_bridge",
        executable="feedback_bridge_node",
        name="feedback_bridge",
        output="screen",
        parameters=[{
            "feedback_input_topic": "feedback_states",
            "feedback_output_topic": "aubo/feedback_states",
            "joint_states_input_topic": "joint_states_ros1",
            "joint_states_output_topic": "joint_states",
            "enable_joint_states_bridge": True,
        }],
    )

    # aubo_driver_ros2: 全 ROS2 驱动，直接发布 joint_states、aubo/feedback_states，无需 feedback_bridge
    use_aubo_driver_ros2 = context.perform_substitution(LaunchConfiguration("use_aubo_driver_ros2")).lower() == "true"
    aubo_driver_ros2_node = Node(
        package="aubo_driver_ros2",
        executable="aubo_driver_ros2",
        name="aubo_driver",
        output="screen",
        parameters=[{
            "server_host": LaunchConfiguration("aubo_driver_server_host"),
            "external_axis_number": 0,
        }],
    )
    
    # Move To Pose Service Server: 提供高级位姿控制服务
    move_to_pose_server_node = Node(
        package="demo_driver",
        executable="move_to_pose_server_node",
        name="move_to_pose_server_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            joint_limits_yaml,
            {
                "planning_group_name": "manipulator",
                "base_frame": "base_link",
            }
        ],
    )
    # ROS2 轨迹插值节点（替代 ROS1 aubo_robot_simulator）：订阅 joint_path_command，插值后发布 moveItController_cmd，由 ros1_bridge 桥接到 ROS1 aubo_driver
    joint_names_list = joint_names_yaml.get("joint_name", {}).get("controller_joint_names", [
        "shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"
    ])
    aubo_robot_simulator_ros2_node = Node(
        package="aubo_robot_simulator_ros2",
        executable="aubo_robot_simulator_node",
        name="aubo_robot_simulator",
        output="screen",
        parameters=[{
            "motion_update_rate": 200.0,
            "minimum_buffer_size": 2000,
            "joint_names": joint_names_list,
            "velocity_scale_factor": 1.0,
            "cartesian_resample_threshold": 0.095,
            "cartesian_min_segment_interval": 0.09,
        }],
    )
    # 注意：在桥接 ROS 1 的场景中，不需要启动 ROS 2 Control 的 joint_trajectory_controller
    # 因为 aubo_ros2_trajectory_action 会监听 /joint_trajectory_controller/follow_joint_trajectory
    # 并将轨迹发布到 joint_path_command；aubo_robot_simulator_ros2 插值后发布 moveItController_cmd，由 ROS 1 端 aubo_driver 执行
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
    
    # 移植前/后对比：simulator_in_ros2=false 时不启动 aubo_robot_simulator_ros2，轨迹经 bridge 到 ROS1 插值
    simulator_in_ros2 = context.perform_substitution(LaunchConfiguration("simulator_in_ros2")).lower() == "true"
    nodes_to_start = [aubo_trajectory_action_node, move_to_pose_server_node]
    if use_aubo_driver_ros2:
        nodes_to_start.insert(0, aubo_driver_ros2_node)
    else:
        nodes_to_start.insert(1, feedback_bridge_node)
    if simulator_in_ros2:
        nodes_to_start.insert(1, aubo_robot_simulator_ros2_node)
    
    # 添加其他必需的节点
    nodes_to_start.extend([
        robot_state_pub_node,
        hand_eye_static_tf_node,
        camera_frame_to_camera_link_node,
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
            description="Name of the SRDF file (plain .srdf, loaded directly)",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "simulator_in_ros2",
            default_value="true",
            description="If true, run trajectory interpolator in ROS2 (aubo_robot_simulator_ros2). If false, use ROS1 simulator (for before-migration comparison).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_aubo_driver_ros2",
            default_value="false",
            description="If true, run aubo_driver_ros2 (full ROS2 driver) and do not start feedback_bridge. If false, use ROS1 aubo_driver + feedback_bridge.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_driver_server_host",
            default_value="169.254.10.98",
            description="Robot controller IP for aubo_driver_ros2 when use_aubo_driver_ros2=true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_octomap",
            default_value="false",
            description="If true, enable Octomap/point cloud obstacle monitoring (sensors_3d.yaml). Default off.",
        )
    )
    # 手眼静态 TF（与 graspnet_demo 一致）
    try:
        hand_eye_default = os.path.join(
            get_package_share_directory("hand_eye_calibration"),
            "config", "hand_eye_calibration_best.yaml",
        )
    except Exception:
        hand_eye_default = ""
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand_eye_yaml_path",
            default_value=hand_eye_default,
            description="手眼标定 YAML 路径（用于发布末端 -> camera_frame 静态 TF）",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_frame_id",
            default_value="wrist3_Link",
            description="末端 link（手眼 TF 父坐标系）",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
