"""两模式 URDF 结构测试：mobile 翻转与锚定（xacro 子进程展开，不跑 gz）."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

PKG_DIR = Path(__file__).resolve().parents[1]
XACRO = PKG_DIR / 'urdf' / 'harvester_robot.urdf.xacro'


def _safe_fromstring(text: str) -> ET.Element:
    """Xacro 产物不含 DTD/实体；出现即拒绝解析（防实体扩展）."""
    lowered = text.lower()
    if '<!doctype' in lowered or '<!entity' in lowered:
        raise ValueError('URDF 含 DTD/实体声明，拒绝解析')
    return ET.fromstring(text)


def _expand(mobile: str) -> ET.Element:
    out = subprocess.run(
        ['xacro', str(XACRO), f'mobile:={mobile}'],
        capture_output=True, text=True, check=True).stdout
    return _safe_fromstring(out)


def _links_joints(root: ET.Element):
    links = [item.get('name') for item in root.findall('link')]
    joints = {
        item.get('name'): (item.find('parent').get('link'),
                           item.find('child').get('link'))
        for item in root.findall('joint')}
    children = {child for _, child in joints.values()}
    roots = [name for name in links if name not in children]
    return links, joints, roots


def test_anchored_mode_keeps_world_root():
    """mobile:=false（默认）：world 为根、臂链在、无驱动插件与传感器."""
    root = _expand('false')
    links, joints, roots = _links_joints(root)
    assert roots == ['world']
    assert joints['platform_mount'] == ('world', 'platform_base_link')
    assert joints['left_track_mount'] == ('platform_base_link', 'left_track')
    assert 'shoulder_joint' in joints  # 臂链存在（锚定建模）
    assert 'lidar_mount' not in links
    filenames = {item.get('filename') for item in root.iter('plugin')}
    # 锚定模式无驱动插件（VelocityControl/OdometryPublisher 均随 mobile 分支）
    assert 'gz-sim-velocity-control-system' not in filenames
    assert 'gz-sim-odometry-publisher-system' not in filenames
    assert not list(root.iter('sensor'))


def test_mobile_mode_is_platform_only_drivable():
    """mobile:=true：纯平台（无臂链/world），根 platform_base_link 可驾驶."""
    root = _expand('true')
    links, joints, roots = _links_joints(root)
    assert roots == ['platform_base_link']
    assert 'world' not in links  # world_joint 的世界锚定不得存在
    assert 'platform_mount' not in joints
    assert 'shoulder_joint' not in joints  # 无臂链
    assert joints['lidar_mount_joint'] == ('platform_base_link', 'lidar_mount')

    plugins = list(root.iter('plugin'))
    # P4b 口径：VelocityControl 直接控制机体速度（对齐 AgileX planar_move），
    # 不模拟滚轮物理；履带 = 碰撞足迹 + 视觉箱体，挂接 fixed
    velocity = next(item for item in plugins
                    if item.get('filename') == 'gz-sim-velocity-control-system')
    assert velocity.find('topic').text == '/cmd_vel'
    odom = next(item for item in plugins
                if item.get('filename') == 'gz-sim-odometry-publisher-system')
    assert odom.find('odom_frame').text == 'odom'
    assert odom.find('robot_frame').text == 'platform_base_link'
    # 履带挂接 fixed：VelocityControl 不需要旋转关节（DiffDrive 才需要滚筒）
    joint_types = {item.get('name'): item.get('type')
                   for item in root.findall('joint')}
    assert joints['left_track_mount'] == ('platform_base_link', 'left_track')
    assert joints['right_track_mount'] == ('platform_base_link', 'right_track')
    assert joint_types['left_track_mount'] == 'fixed'
    assert joint_types['right_track_mount'] == 'fixed'

    sensor = next(root.iter('sensor'))
    assert sensor.get('type') == 'gpu_lidar'
    assert sensor.find('topic').text == '/scan'
    assert sensor.find('gz_frame_id').text == 'lidar_mount'
    # FOV 对称 ±95°（xacro 输出弧度）
    horizontal = sensor.find('lidar/scan/horizontal')
    min_angle = float(horizontal.find('min_angle').text)
    max_angle = float(horizontal.find('max_angle').text)
    assert abs(min_angle + max_angle) < 1e-6
    assert abs(max_angle - 95.0 * 3.141592653589793 / 180.0) < 1e-4
