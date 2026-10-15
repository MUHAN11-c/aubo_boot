"""导航配置一致性测试（零 ROS）：桥接清单 / Nav2 参数 / 地图 / 站位对齐."""

from pathlib import Path

import yaml

PKG_DIR = Path(__file__).resolve().parents[1]
BRIDGE = PKG_DIR / 'config' / 'orchard_bridge.yaml'
NAV2 = PKG_DIR / 'config' / 'nav2_params.yaml'
NAV2_FIELD = PKG_DIR / 'config' / 'nav2_params_field.yaml'
# 地图与场景参数在兄弟包 sim_worlds（src/sim/ 组内源码树相对定位）
WORLDS = PKG_DIR.parent / 'sim_worlds'
MAP = WORLDS / 'maps' / 'orchard_map.yaml'
CONFIG = WORLDS / 'config' / 'orchard.yaml'
FIELD_STATIONS = WORLDS / 'maps' / 'field_stations.yaml'

BASE_FRAME = 'platform_base_link'


def _load(path: Path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def test_bridge_topics_match_urdf_plugins():
    """桥接清单覆盖 Nav2 全链：clock/odom/tf/scan 入、cmd_vel 出、关节状态."""
    entries = _load(BRIDGE)
    by_ros = {item['ros_topic_name']: item for item in entries}
    assert set(by_ros) == {'clock', 'joint_states', 'odom', 'tf', 'scan',
                           'cmd_vel'}
    # /clock 必须单向 GZ→ROS（官方要求）
    assert by_ros['clock']['direction'] == 'GZ_TO_ROS'
    assert by_ros['clock']['gz_topic_name'] == '/clock'
    # cmd_vel 是 ROS→GZ（普通 Twist，与 nav2_bringup 1.3.13 默认一致）
    assert by_ros['cmd_vel']['direction'] == 'ROS_TO_GZ'
    assert by_ros['cmd_vel']['ros_type_name'] == 'geometry_msgs/msg/Twist'
    assert by_ros['odom']['gz_type_name'] == 'gz.msgs.Odometry'
    assert by_ros['tf']['gz_type_name'] == 'gz.msgs.Pose_V'
    assert by_ros['scan']['gz_type_name'] == 'gz.msgs.LaserScan'
    # gz 侧话题与 URDF VelocityControl/OdometryPublisher 插件 <topic> 对齐
    for name in ('odom', 'tf', 'scan', 'cmd_vel'):
        assert by_ros[name]['gz_topic_name'] == f'/{name}'


def test_nav2_frames_all_use_platform_base():
    """Nav2 全节点帧统一 platform_base_link（与 mobile URDF 根一致）."""
    doc = _load(NAV2)
    checks = [
        doc['amcl']['ros__parameters']['base_frame_id'],
        doc['bt_navigator']['ros__parameters']['robot_base_frame'],
        doc['behavior_server']['ros__parameters']['robot_base_frame'],
        doc['collision_monitor']['ros__parameters']['base_frame_id'],
        doc['local_costmap']['local_costmap']['ros__parameters'][
            'robot_base_frame'],
        doc['global_costmap']['global_costmap']['ros__parameters'][
            'robot_base_frame'],
        doc['docking_server']['ros__parameters']['base_frame'],
    ]
    assert set(checks) == {BASE_FRAME}
    # REP-105：odom/map 帧名与桥接插件 frame_id 对齐
    assert doc['amcl']['ros__parameters']['odom_frame_id'] == 'odom'
    assert doc['amcl']['ros__parameters']['global_frame_id'] == 'map'
    assert doc['bt_navigator']['ros__parameters']['global_frame'] == 'map'
    assert doc['bt_navigator']['ros__parameters']['odom_topic'] == '/odom'


def test_amcl_initial_pose_matches_manifest_spawn():
    """AMCL 初始位姿 = mobile 出生点（行间走廊中心），免手发初值."""
    doc = _load(NAV2)
    initial = doc['amcl']['ros__parameters']['initial_pose']
    raw = _load(CONFIG)
    platform = raw['platform']
    row_x = (platform['face_row_index'] - (5 - 1) / 2.0) * 4.0
    corridor_x = row_x + 4.0 / 2.0  # launch mobile 分支：走廊中心（半行距）
    assert abs(initial['x'] - corridor_x) < 1e-6
    assert abs(initial['y'] - platform['y']) < 1e-6
    assert abs(initial['yaw'] - (-platform['side'] * 3.141592653589793
                                 / 2.0)) < 1e-4


def test_field_amcl_initial_pose_matches_stations_spawn():
    """
    Field 预设：AMCL initial_pose = field_stations.yaml spawn（双写防漂移门）.

    harvest.launch 读 stations spawn 下发 gz 出生位姿，AMCL 初值则是
    nav2_params_field.yaml 手写——两处必须同值，否则定位起步就漂。
    """
    initial = _load(NAV2_FIELD)['amcl']['ros__parameters']['initial_pose']
    spawn = _load(FIELD_STATIONS)['spawn']
    assert abs(initial['x'] - spawn['x']) < 1e-6
    assert abs(initial['y'] - spawn['y']) < 1e-6
    assert abs(initial['yaw'] - spawn.get('yaw', 0.0)) < 1e-6


def test_map_yaml_matches_generator_convention():
    """地图 yaml 与生成器口径一致：分辨率 0.05、原点 = 地面左下角."""
    ground = _load(CONFIG)['ground']
    doc = _load(MAP)
    assert doc['resolution'] == 0.05
    assert doc['origin'] == [-ground['size_x'] / 2.0,
                             -ground['size_y'] / 2.0, 0.0]
    assert doc['mode'] == 'trinary'
    assert Path(MAP.parent / doc['image']).exists()


def test_nav2_speeds_within_track_plugin_limits():
    """Nav2 限速不超出底盘驱动口径（VelocityControl：vx≤0.8、wz≤1.2）."""
    doc = _load(NAV2)
    velocity = doc['velocity_smoother']['ros__parameters']
    assert velocity['max_velocity'][0] <= 0.8
    assert velocity['max_velocity'][2] <= 1.2
    follow = doc['controller_server']['ros__parameters']['FollowPath']
    assert follow['vx_max'] <= 0.8
    assert follow['wz_max'] <= 1.2
