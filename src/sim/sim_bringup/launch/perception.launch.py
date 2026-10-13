"""
仿真感知栈（P3）：peach 感知快照（sim_perception）跑在统一机体仿真相机流上.

前置：``sim.launch.py`` 已起（统一机体 + 腕载 rgbd + 相机桥 + JSB /joint_states）。
组成：
1. 两个 LifecycleNode（节点名保持 peach 契约：``peach_scene_perception_node`` /
   ``peach_target_reconstruction_node``；参数 = sim_perception share 两份全量
   yaml + use_sim_time）；
2. ``nav2_lifecycle_manager``（名 ``peach_lifecycle_manager``，顺序：感知 → 重建，
   autostart；bond_timeout 默认 0，与现行真机栈同口径）。

无 supervisor / scene_obstacles（P4）：验收门 = 手动/launch_testing 直调
``~/begin_scene`` → ``/peach/perception/target_observations`` 非空 →
``~/build_target_model`` 出 refined_pose。闩锁桥（managed_nodes_activated）
属 supervisor 生态，本轮不起。测完 Ctrl+C 停栈并 pgrep 清残留（MUST）。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    perc_share = Path(get_package_share_directory('sim_perception'))
    use_sim_time = LaunchConfiguration('use_sim_time')

    perception = LifecycleNode(
        package='sim_perception',
        executable='sim_scene_perception_node',
        name='peach_scene_perception_node',
        namespace='',
        output='screen',
        parameters=[
            ParameterFile(
                str(perc_share / 'config' / 'scene_perception.yaml'),
                allow_substs=True),
            {'use_sim_time': use_sim_time},
        ],
    )
    reconstruction = LifecycleNode(
        package='sim_perception',
        executable='sim_target_reconstruction_node',
        name='peach_target_reconstruction_node',
        namespace='',
        output='screen',
        parameters=[
            ParameterFile(
                str(perc_share / 'config' / 'target_reconstruction.yaml'),
                allow_substs=True),
            {'use_sim_time': use_sim_time},
        ],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='peach_lifecycle_manager',
        output='screen',
        parameters=[{
            'node_names': [
                'peach_scene_perception_node',
                'peach_target_reconstruction_node',
            ],
            'autostart': True,
            'bond_timeout': 0.0,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真时钟（与 sim.launch.py 同源 /clock）'),
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        perception,
        reconstruction,
        lifecycle_manager,
    ])
