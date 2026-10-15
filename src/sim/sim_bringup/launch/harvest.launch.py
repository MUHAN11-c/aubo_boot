"""
自主导航采摘全栈（P4 演示入口）：统一机体 + Nav2 + MoveIt + 感知，一键起.

组成（复用既有入口，不复制逻辑）：
1. Include ``sim.launch.py``（统一机体：可驾驶履带 + gz_ros2_control 臂 +
   MoveIt + 腕载 rgbd 相机桥；``spawn:=corridor`` 走廊中心起步清障）；
2. Include ``perception.launch.py``（感知→重建 lifecycle，与 gz 同生命周期）；
3. Include ``nav2_bringup bringup_launch.py``（AMCL + 确定性生成图，
   ``nav2_params.yaml``，同 orchard_nav 口径）；
4. 可选 RViz（nav2 视图）。

采摘编排在独立终端跑（不在 launch 内自动发起，与真机意图纪律一致）：

```bash
ros2 launch sim_bringup harvest.launch.py
# 另开终端（等 1-2 分钟世界首载 + 控制器/AMCL 就绪）：
ros2 run sim_bringup orchard_harvest --rows 2 --max-stations 3
```

每站节拍：harvest_stow 行驶 → NavigateToPose 到站 → global_photo_pose →
begin_scene 锁集 → build_target_model → tcp 沿袋轴接近 0.12 m → 下一站。
测完 Ctrl+C 停栈并 pgrep 清残留（MUST）。
"""

from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory('sim_bringup'))
    worlds_share = Path(get_package_share_directory('sim_worlds'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    params_file = LaunchConfiguration('nav2_params')
    gui = LaunchConfiguration('gui')
    moveit = LaunchConfiguration('moveit')
    rviz = LaunchConfiguration('rviz')
    field = LaunchConfiguration('field')
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')

    field_world = str(worlds_share / 'worlds' / 'field_no_grass.sdf')
    field_map = str(worlds_share / 'maps' / 'field_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真时钟（全图统一，勿关）'),
        DeclareLaunchArgument(
            'field', default_value='true',
            description='true=重建果园（field_no_grass GLB + generate_field '
                        '地图/站位，AMCL 初始位姿同源）；false=程序化世界'
                        '（确定性回归用）'),
        DeclareLaunchArgument(
            'world', default_value=field_world,
            description='世界 SDF（field 预设默认重建果园）'),
        DeclareLaunchArgument(
            'map', default_value=field_map,
            description='静态地图（field 预设 = generate_field 产物）'),
        DeclareLaunchArgument(
            'slam', default_value='False',
            description='true=slam_toolbox 在线建图；false=AMCL + 静态图'),
        DeclareLaunchArgument(
            'nav2_params', default_value='',
            description='Nav2 参数（空 = field 用 nav2_params_field.yaml、'
                        '否则 nav2_params.yaml）'),
        DeclareLaunchArgument(
            'gui', default_value='false', description='gz 自带 GUI（默认无头）'),
        DeclareLaunchArgument(
            'moveit', default_value='true',
            description='include MoveIt（采摘接近动作需要）'),
        DeclareLaunchArgument(
            'rviz', default_value='False', description='RViz（sim_harvest 视图）'),

        # gz-gui 本机走 GLX（同 orchard_nav 注记）
        SetEnvironmentVariable('QT_XCB_GL_INTEGRATION', 'xcb_glx'),
        OpaqueFunction(function=_setup),
    ])


def _setup(context, *args, **kwargs):
    """按 field/程序化预设解析 world/map/params/spawn 后组装."""
    bringup_share = Path(get_package_share_directory('sim_bringup'))
    worlds_share = Path(get_package_share_directory('sim_worlds'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    gui = LaunchConfiguration('gui')
    moveit = LaunchConfiguration('moveit')
    rviz = LaunchConfiguration('rviz')
    field = LaunchConfiguration('field').perform(context).lower() in ('true', '1')
    world = LaunchConfiguration('world').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)
    params_raw = LaunchConfiguration('nav2_params').perform(context)
    nav2_share = Path(get_package_share_directory('nav2_bringup'))

    if params_raw:
        params_file = params_raw
    else:
        params_file = str(bringup_share / 'config' /
                          ('nav2_params_field.yaml' if field
                           else 'nav2_params.yaml'))
    sim_args = {'gui': gui, 'moveit': moveit,
                'use_sim_time': use_sim_time,
                # RViz 经 sim→moveit_sim 链起（带 robot_description_semantic，
                # moveit 显示层需要；直接在此起裸 RViz 会报 SRDF 解析失败）
                'rviz': rviz,
                'rviz_config': str(bringup_share / 'config' /
                                   'sim_harvest.rviz')}
    if field:
        # generate_field 站位同源：spawn 读 field_stations.yaml 的 spawn 段
        # （走廊中点，非首站——站位是作业位贴树，出生会压膨胀圈）。
        # AMCL initial_pose（nav2_params_field.yaml）须与此同值。
        stations = yaml.safe_load(
            (worlds_share / 'maps' / 'field_stations.yaml')
            .read_text(encoding='utf-8'))
        spawn = stations['spawn']
        sim_args.update({'spawn_x': str(spawn['x']),
                         'spawn_y': str(spawn['y']),
                         'spawn_yaw': str(spawn.get('yaw', 0.0))})
    else:
        sim_args['spawn'] = 'corridor'

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / 'launch' / 'sim.launch.py')),
            launch_arguments=sim_args.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / 'launch' / 'perception.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nav2_share / 'launch' / 'bringup_launch.py')),
            launch_arguments={
                'slam': slam,
                'map': map_yaml,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items()),
    ]
    return nodes
