from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """
    启动 demo_driver 服务节点
    
    这些服务节点需要等待 MoveIt2 和 robot_description 参数就绪后才能启动。
    使用 TimerAction 延迟 8 秒启动，确保 MoveIt2 已完全初始化。
    
    使用方法：
        ros2 launch aubo_moveit_config demo_driver_services.launch.py
    
    注意：此 launch 文件需要先启动 MoveIt2（例如通过 aubo_moveit_bridge_ros1.launch.py）
    """
    
    # Demo Driver 服务节点（延迟启动，等待 MoveIt2 就绪）
    # 使用 TimerAction 延迟 8 秒启动，确保 MoveIt2 和 robot_description 已加载
    move_to_pose_server_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="demo_driver",
                executable="move_to_pose_server_node",
                name="move_to_pose_server",
                output="screen",
            )
        ]
    )
    
    plan_trajectory_server_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="demo_driver",
                executable="plan_trajectory_server_node",
                name="plan_trajectory_server",
                output="screen",
            )
        ]
    )
    
    execute_trajectory_server_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="demo_driver",
                executable="execute_trajectory_server_node",
                name="execute_trajectory_server",
                output="screen",
            )
        ]
    )
    
    get_current_state_server_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="demo_driver",
                executable="get_current_state_server_node",
                name="get_current_state_server",
                output="screen",
            )
        ]
    )
    
    set_speed_factor_server_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="demo_driver",
                executable="set_speed_factor_server_node",
                name="set_speed_factor_server",
                output="screen",
            )
        ]
    )
    
    return LaunchDescription([
        move_to_pose_server_node,
        plan_trajectory_server_node,
        execute_trajectory_server_node,
        get_current_state_server_node,
        set_speed_factor_server_node,
    ])

