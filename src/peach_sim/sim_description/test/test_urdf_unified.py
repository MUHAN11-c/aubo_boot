"""统一机体 URDF 结构测试（xacro 子进程展开，不跑 gz）."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

PKG_DIR = Path(__file__).resolve().parents[1]
XACRO = PKG_DIR / 'urdf' / 'harvester_unified.urdf.xacro'

ARM_JOINTS = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
              'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
CAMERA_FRAMES = ['camera_link', 'camera_color_frame',
                 'camera_color_optical_frame', 'camera_depth_frame',
                 'camera_depth_optical_frame']


def _safe_fromstring(text: str) -> ET.Element:
    """Xacro 产物不含 DTD/实体；出现即拒绝解析（防实体扩展）."""
    lowered = text.lower()
    if '<!doctype' in lowered or '<!entity' in lowered:
        raise ValueError('URDF 含 DTD/实体声明，拒绝解析')
    return ET.fromstring(text)


def _expand(extra: list[str] | None = None) -> ET.Element:
    out = subprocess.run(
        ['xacro', str(XACRO), *(extra or [])],
        capture_output=True, text=True, check=True).stdout
    return _safe_fromstring(out)


def _joints(root: ET.Element) -> dict[str, ET.Element]:
    """运动学关节 = robot 直接子级（ros2_control 块内也有 <joint>，勿混入）."""
    return {j.get('name'): j for j in root.findall('joint')}


def test_root_is_platform_without_world_anchor():
    """根 = platform_base_link；无 world 链接与世界锚定关节."""
    root = _expand()
    names = {link.get('name') for link in root.iter('link')}
    assert 'platform_base_link' in names
    assert 'world' not in names
    assert 'world_joint' not in _joints(root)
    # 平台不挂任何父关节（platform_mount 仅锚定模式存在）
    parents = {j.find('child').get('link') for j in _joints(root).values()}
    assert 'platform_base_link' not in parents


def test_arm_chain_and_turret_mount():
    """六关节臂链齐全；base_link 经转台挂车体顶面 0.54 m."""
    root = _expand()
    joints = _joints(root)
    for name in ARM_JOINTS:
        assert joints[name].get('type') == 'revolute', name
    mount = joints['arm_mount_joint']
    assert mount.get('type') == 'fixed'
    assert mount.find('parent').get('link') == 'platform_base_link'
    assert mount.find('child').get('link') == 'base_link'
    assert mount.find('origin').get('xyz').startswith('0 0 0.54')


def test_gz_ros2_control_block_and_controllers_file():
    """GazeboSimSystem + 六关节 position 命令；插件指向 gz_controllers.yaml."""
    root = _expand()
    blocks = list(root.iter('ros2_control'))
    assert len(blocks) == 1
    assert blocks[0].find('hardware/plugin').text == \
        'gz_ros2_control/GazeboSimSystem'
    rc_joints = {j.get('name'): j for j in blocks[0].iter('joint')}
    assert sorted(rc_joints) == sorted(ARM_JOINTS)
    for joint in rc_joints.values():
        cmds = [c.get('name') for c in joint.iter('command_interface')]
        assert cmds == ['position']
    plugins = list(root.iter('plugin'))
    gz_ctrl = [p for p in plugins
               if (p.get('filename') or '').endswith('gz_ros2_control-system.so')]
    assert len(gz_ctrl) == 1
    assert 'gz_controllers.yaml' in gz_ctrl[0].find('parameters').text


def test_drivetrain_and_lidar_survived():
    """
    履带 VelocityControl 驱动与前桅杆激光随 mobile 分支保留.

    P4b 修正口径：VelocityControl 直接控制机体速度（对齐 AgileX
    planar_move），履带挂接 fixed（碰撞足迹，无滚筒物理）。
    """
    root = _expand()
    plugins = list(root.iter('plugin'))
    names = [p.get('name') for p in plugins]
    assert 'gz::sim::systems::VelocityControl' in names
    odom = next(item for item in plugins
                if item.get('filename') == 'gz-sim-odometry-publisher-system')
    assert odom.find('robot_base_frame').text == 'platform_base_link'
    sensors = {s.get('name'): s.get('type') for s in root.iter('sensor')}
    assert sensors.get('orchard_scan') == 'gpu_lidar'
    joints = _joints(root)
    assert joints['left_track_mount'].get('type') == 'fixed'


def test_camera_frame_chain_matches_peach_contract():
    """相机帧链 = peach 感知契约（camera_link→{color,depth}→optical）."""
    root = _expand()
    names = {link.get('name') for link in root.iter('link')}
    for frame in CAMERA_FRAMES:
        assert frame in names, frame
    joints = _joints(root)
    optical = joints['camera_color_optical_joint']
    assert optical.find('parent').get('link') == 'camera_color_frame'
    # REP-103 光学旋转（x 右 y 下 z 前）
    rpy = [float(v) for v in optical.find('origin').get('rpy').split()]
    assert abs(rpy[0] + 1.5707963) < 1e-5
    assert abs(rpy[2] + 1.5707963) < 1e-5


def test_wrist_rgbd_sensor_toggle():
    """腕载 rgbd sensor 默认在；wrist_camera:=false 可关（重测试减载）."""
    with_sensor = _expand()
    sensors = {s.get('name'): s for s in with_sensor.iter('sensor')}
    rgbd = sensors.get('wrist_rgbd')
    assert rgbd is not None and rgbd.get('type') == 'rgbd_camera'
    assert rgbd.find('gz_frame_id').text == 'camera_color_optical_frame'
    # 法兰和相机外壳对传感器透明。flags=0：所有相机都看不到。
    mask = rgbd.find('./camera/visibility_mask')
    assert mask is not None and mask.text == '268435454'
    near = rgbd.find('./camera/clip/near')
    assert near is not None and near.text == '0.002'
    flags = [
        node.text for node in with_sensor.iter('visibility_flags')]
    assert flags.count('0') >= 2
    without = _expand(['wrist_camera:=false'])
    assert not list(without.iter('sensor')) or \
        {s.get('name') for s in without.iter('sensor')} == {'orchard_scan'}


def test_tool_profile_variants_expand():
    """三种 peach 末端各自可展开（真机 CAD 引用 aubo_description）."""
    for profile in ('shear_v1', 'bite_shear_v1', 'adaptive_shear_v1'):
        root = _expand([f'tool_profile:={profile}'])
        text = ET.tostring(root, encoding='unicode')
        assert f'tools/{profile}' in text, profile
