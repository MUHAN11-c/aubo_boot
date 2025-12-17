"""
AUBO 机器人 MoveIt 规划与执行启动文件
用于启动完整的 MoveIt 运动规划系统，包括规划、执行和可视化功能
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
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
    """
    加载 YAML 配置文件
    
    参数:
        package_name: ROS2 包名
        file_path: 相对于包共享目录的文件路径
    
    返回:
        解析后的 YAML 字典，如果文件不存在则返回 None
    """
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    """
    启动设置函数，配置并启动所有必要的节点
    
    参数:
        context: Launch 上下文对象，包含启动参数信息
    """
    # 命令行参数配置
    robot_xacro_file = LaunchConfiguration("robot_xacro_file")  # 机器人 XACRO 文件名
    support_package = LaunchConfiguration("support_package")  # 支持包名（如 aubo_description）
    moveit_config_package = LaunchConfiguration("moveit_config_package")  # MoveIt 配置包名
    moveit_config_file = LaunchConfiguration("moveit_config_file")  # SRDF 配置文件
    aubo_type = LaunchConfiguration("aubo_type")  # AUBO 机器人型号（如 aubo_e5）

    # 规划上下文配置
    # 生成机器人描述（URDF），通过 xacro 处理 XACRO 文件
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # 查找 xacro 可执行文件
            " ",
            PathJoinSubstitution(
                [FindPackageShare(support_package), "urdf/xacro/inc/", robot_xacro_file]
            ),  # XACRO 文件路径
            " ",
            "aubo_type:=",  # 传递机器人型号参数
             aubo_type,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 生成语义描述（SRDF），用于 MoveIt 规划
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", moveit_config_file]
            ),  # SRDF 文件路径
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content.perform(
            context
        )
    }

    # 加载运动学配置（逆运动学求解器参数）
    kinematics_yaml = load_yaml(
        "aubo_moveit_config", "config/kinematics.yaml"
    )

    # 加载关节限制配置（速度、加速度、位置限制等）
    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "aubo_moveit_config", "config/joint_limits.yaml"
        )
    }

    # 规划功能配置
    # OMPL 规划管道配置（OMPL 是 MoveIt 的默认运动规划库）
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",  # OMPL 规划器插件
            # 规划请求适配器链（按顺序执行）：
            # 1. UniformSampleFilter: 均匀采样轨迹点
            # 2. AddTimeOptimalParameterization: 添加时间最优参数化
            # 3. ResolveConstraintFrames: 解析约束坐标系
            # 4. FixWorkspaceBounds: 修复工作空间边界
            # 5. FixStartStateBounds: 修复起始状态边界
            # 6. FixStartStateCollision: 修复起始状态碰撞
            # 7. FixStartStatePathConstraints: 修复起始状态路径约束
            "request_adapters": """industrial_trajectory_filters/UniformSampleFilter default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,  # 起始状态最大边界误差（弧度）
            "sample_duration": 0.005,  # 采样周期（秒），对应 200Hz 控制频率
        }
    }
    # 加载 OMPL 规划器详细配置（包含各种规划算法参数）
    ompl_planning_yaml = load_yaml(
        "aubo_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # 轨迹执行功能配置
    # 加载控制器配置（定义如何与机器人控制器通信）
    moveit_simple_controllers_yaml = load_yaml(
        "aubo_moveit_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,  # 控制器配置
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",  # 控制器管理器插件
    }

    # 轨迹执行参数
    trajectory_execution = {
        # MoveIt 不会自动处理控制器切换，需要外部管理
        "moveit_manage_controllers": False,  # 是否由 MoveIt 管理控制器
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,  # 允许的执行时长缩放因子（1.2倍）
        "trajectory_execution.allowed_goal_duration_margin": 0.5,  # 允许的目标时长余量（秒）
        "trajectory_execution.allowed_start_tolerance": 0.01,  # 允许的起始位置容差（弧度）
    }

    # 规划场景监控参数
    # 用于监控和发布规划场景的变化（碰撞对象、附加对象等）
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,  # 发布规划场景
        "publish_geometry_updates": True,  # 发布几何更新
        "publish_state_updates": True,  # 发布状态更新
        "publish_transforms_updates": True,  # 发布变换更新
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",  # 监控器名称
            "robot_description": "robot_description",  # 机器人描述参数名
            "joint_state_topic": "/joint_states",  # 关节状态话题
            "attached_collision_object_topic": "/move_group/planning_scene_monitor",  # 附加碰撞对象话题
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",  # 发布规划场景话题
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",  # 监控的规划场景话题
            "wait_for_initial_state_timeout": 10.0,  # 等待初始状态的超时时间（秒）
           }
    }

    # 启动 move_group 节点/动作服务器
    # move_group 是 MoveIt 的核心节点，负责运动规划、执行和场景监控
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",  # 输出到屏幕
        parameters=[
            robot_description,  # 机器人描述（URDF）
            robot_description_semantic,  # 语义描述（SRDF）
            kinematics_yaml,  # 运动学配置
            ompl_planning_pipeline_config,  # OMPL 规划管道配置
            trajectory_execution,  # 轨迹执行配置
            moveit_controllers,  # 控制器配置
            planning_scene_monitor_parameters,  # 规划场景监控参数
            joint_limits_yaml,  # 关节限制配置
        ],
    )

    # RViz 可视化节点
    # 用于可视化机器人模型、规划场景和运动轨迹
    rviz_base = os.path.join(
        get_package_share_directory("aubo_moveit_config"), "config"
    )
    rviz_config = os.path.join(rviz_base, "view_robot.rviz")  # RViz 配置文件路径
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",  # 输出到日志
        arguments=["-d", rviz_config],  # 加载配置文件
        parameters=[
            robot_description,  # 机器人描述
            robot_description_semantic,  # 语义描述
            ompl_planning_pipeline_config,  # 规划配置
            kinematics_yaml,  # 运动学配置
            joint_limits_yaml,  # 关节限制
        ],
    )

    # 静态 TF 发布节点
    # 发布从 world 到 base_link 的静态坐标变换
    # 参数：x y z roll pitch yaw parent_frame child_frame
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # 机器人状态发布节点
    # 根据 URDF 和关节状态发布机器人各连杆的 TF 变换
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",  # 同时输出到屏幕和日志
        parameters=[robot_description],
    )
    
    # 关节状态发布 GUI 节点
    # 提供图形界面用于手动控制机器人关节（用于测试和演示）
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[robot_description]
        )

    # 返回要启动的节点列表
    # 注意：joint_state_publisher_node 未包含在列表中，如需使用可手动启动
    nodes_to_start = [move_group_node, rviz_node, static_tf_node, robot_state_pub_node]
    return nodes_to_start


def generate_launch_description():
    """
    生成启动描述
    
    定义所有可用的启动参数，并返回包含参数和启动函数的 LaunchDescription
    """
    declared_arguments = []

    # TODO(andyz): add other options
    # 声明启动参数：机器人 XACRO 文件
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_xacro_file",
            default_value="aubo.xacro",
            description="Xacro describing the robot.",  # 描述机器人的 XACRO 文件
            choices=["aubo.xacro"],
        )
    )
    # 声明启动参数：支持包名
    declared_arguments.append(
        DeclareLaunchArgument(
            "support_package",
            default_value="aubo_description",
            description="Name of the support package",  # 包含机器人描述文件的支持包名
            choices=["aubo_description"],
        )
    )
    # 声明启动参数：MoveIt 配置包名
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="aubo_moveit_config",
            description="Name of the support package",  # MoveIt 配置包名
            choices=["aubo_moveit_config"],
        )
    )
    # 声明启动参数：SRDF 配置文件
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            description="Name of the SRDF file",  # SRDF 语义描述文件名
            default_value="aubo_i5.srdf",
            choices=["aubo_i5.srdf"],
        )
    )
    # 声明启动参数：AUBO 机器人型号
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            description="Name of the aubo_type ",  # AUBO 机器人型号（如 aubo_e5, aubo_i5 等）
            default_value="aubo_e5",
            choices=["aubo_e5"],
        )
    )

    # 返回启动描述，包含所有声明的参数和启动设置函数
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
