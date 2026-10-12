"""ivg_sim 机械臂 URDF 构建辅助.

职责：
- build_arm_urdf()：xacro 展开 ``urdf/aubo_e5_ivg_sim.xacro``（源码树优先，
  symlink-install 不落 urdf data_files，与 launch 的世界解析同策略）；
- strip_world()：剥除厂商 URDF 的 ``world`` link + ``world_joint``（identity）。
  保留它们会让 gz 把整模型当静态焊接（关节不可动），且 RSP 会发布
  identity world→base_link 与真实摆放 TF 冲突（叠 child frame 红线）。
- 常量：SPAWN_XYZ = 机械臂基座在 cell ``world`` 帧的摆放（桌面中心系），
  launch 的 gz spawn 位姿与静态 TF world→base_link 共用此单源。

纯函数零 ROS 依赖，pytest 直测。
"""

import re
from pathlib import Path
import xml.etree.ElementTree as ET

# 机械臂基座摆放（world 帧，桌面中心原点）。x=-0.58 + 基座宽 0.24：
# 前缘 -0.46 距桌沿（-0.45）1cm 不相交（MoveIt 永碰撞态红线），同时
# 最远伸手 0.68 m 的前倾矩 ~223 Nm < 基座回复矩 353 Nm（1.6×；
# -0.62/0.18 组合曾在全展抓取时整体前翻，2026-09-30 实测）。
SPAWN_XYZ = (-0.58, 0.0, 0.75)

ARM_JOINTS = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)

# Original CAD subassemblies, diagonal outward axes. Positions in metres;
# commands are forces, not positions, so contact can stop an individual jaw.
FINGER_JOINTS = tuple(f'gripper2_{n}_joint' for n in ('pp', 'pn', 'np', 'nn'))
FINGER_OPEN_POSE = (0.055,) * 4
FINGER_GRASP_POSE = (0.0,) * 4
FINGER_OPEN_FORCE = (4.0,) * 4
FINGER_GRASP_FORCE = (-12.0,) * 4
GRASP_LINK = 'grasp_tcp'

# 属性顺序无关（minidom 序列化可能重排属性：name 不一定紧跟标签名）。
# barista 版：剥 world link + fixed_base（原 aubo 版为 world_joint）
_WORLD_LINK_RE = re.compile(r'<link\b[^>]*\bname="world"[^>]*/>\s*')
_WORLD_JOINT_RE = re.compile(
    r'<joint\b[^>]*\bname="(?:world_joint|fixed_base)"[^>]*>.*?</joint>\s*',
    re.DOTALL)


def strip_world(urdf_xml: str) -> str:
    """剥除 world link 与其固定锚 joint（world_joint/fixed_base；幂等）."""
    out = _WORLD_LINK_RE.sub('', urdf_xml, count=1)
    out = _WORLD_JOINT_RE.sub('', out)
    return out


def _package_root() -> Path:
    return Path(__file__).resolve().parent.parent


def arm_xacro_path() -> Path:
    source = _package_root() / 'urdf' / 'aubo_e5_ivg_sim.xacro'
    if source.exists():
        return source
    raise FileNotFoundError(
        f'未找到机械臂 xacro（源码树优先策略）：{source}')


def srdf_path() -> Path:
    """ivg_sim SRDF（源码树优先；.srdf 不进 data_files glob）."""
    source = _package_root() / 'config' / 'ivg_sim.srdf'
    if source.exists():
        return source
    raise FileNotFoundError(f'未找到 SRDF：{source}')


def worlds_dir() -> Path:
    """GT manifest/世界目录（源码树；与 launch 世界解析同策略）."""
    return _package_root() / 'worlds'


def collision_center_z(model_name: str) -> float:
    """Return the local box collision center; reject unmodelled geometry."""
    if not re.fullmatch(r'[a-zA-Z0-9_]+', model_name):
        raise ValueError('Invalid Gazebo model name')
    model = ET.parse(_package_root() / 'models' / model_name / 'model.sdf')
    collision = model.find('model/link/collision')
    if collision is None or collision.find('geometry/box') is None:
        raise ValueError(f'{model_name}: grasp height requires a box collision model')
    link_pose = model.findtext('model/link/pose', '0 0 0 0 0 0').split()
    collision_pose = collision.findtext('pose', '0 0 0 0 0 0').split()
    if any(abs(float(v)) > 1e-8 for v in link_pose[3:5]):
        raise ValueError(f'{model_name}: rotated link requires a measured grip pose')
    return float(link_pose[2]) + float(collision_pose[2])


def build_arm_urdf(gripper='gripper2') -> str:
    """xacro 展开并剥除 world link/joint，返回 robot_description 字符串."""
    import xacro

    if gripper != 'gripper2':
        raise ValueError('Gazebo actuation is defined only for original gripper2')
    doc = xacro.process_file(str(arm_xacro_path()), mappings={'gripper': gripper})
    return strip_world(doc.toprettyxml(indent='  '))
