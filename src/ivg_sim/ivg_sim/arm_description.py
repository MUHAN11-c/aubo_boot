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

# 机械臂基座摆放（world 帧，桌面中心原点）。x=-0.58 + 基座宽 0.24：
# 前缘 -0.46 距桌沿（-0.45）1cm 不相交（MoveIt 永碰撞态红线），同时
# 最远伸手 0.68 m 的前倾矩 ~223 Nm < 基座回复矩 353 Nm（1.6×；
# -0.62/0.18 组合曾在全展抓取时整体前翻，2026-09-30 实测）。
SPAWN_XYZ = (-0.58, 0.0, 0.75)

ARM_JOINTS = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)

# 四指夹爪（gripper2 剖分，v3 原装链挂接）：同一指令值同步开合，
# 0=闭态原位；行程 0.055
FINGER_JOINTS = (
    'finger_xp_joint', 'finger_xn_joint', 'finger_yp_joint',
    'finger_yn_joint',
)
FINGER_OPEN = 0.05    # 张开 50mm：开口 119×138mm（> 最大对象 100mm+余量）
FINGER_CLOSED = 0.0   # 闭态原位（effort 限力 40N/指，停在对象表面）

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


def build_arm_urdf() -> str:
    """xacro 展开并剥除 world link/joint，返回 robot_description 字符串."""
    import xacro

    doc = xacro.process_file(str(arm_xacro_path()))
    return strip_world(doc.toprettyxml(indent='  '))
