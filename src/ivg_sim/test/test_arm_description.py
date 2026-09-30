"""arm_description 纯核单测：world 剥除幂等性 + xacro 展开（有 ROS 环境）。"""

import pytest

from ivg_sim.arm_description import (
    ARM_JOINTS,
    FINGER_OPEN,
    SPAWN_XYZ,
    strip_world,
)


def test_strip_world_removes_link_and_joint():
    urdf = ('<robot name="aubo_e5">'
            '<link name="world" />'
            '<link name="base_link" />'
            '<joint name="fixed_base" type="fixed">'
            '<parent link="world" /><child link="base_link" />'
            '<origin xyz="0 0 0" rpy="0 0 0" /></joint>'
            '</robot>')
    out = strip_world(urdf)
    assert 'fixed_base' not in out
    assert '<link name="world"' not in out
    assert 'base_link' in out


def test_strip_world_idempotent():
    urdf = ('<robot name="aubo_e5">'
            '<link name="world" /><link name="base_link" />'
            '<joint name="fixed_base" type="fixed">'
            '<parent link="world" /><child link="base_link" /></joint>'
            '</robot>')
    assert strip_world(strip_world(urdf)) == strip_world(urdf)


def test_strip_world_noop_without_world():
    urdf = '<robot name="aubo_e5"><link name="base_link" /></robot>'
    assert strip_world(urdf) == urdf


def test_constants():
    assert len(ARM_JOINTS) == 6
    assert 0.0 < FINGER_OPEN <= 0.06  # 四指行程 0.055 内，开口 119×138mm
    assert len(SPAWN_XYZ) == 3
    assert SPAWN_XYZ[0] < -0.4  # 基座在桌沿外（桌半宽 0.45）


@pytest.mark.skipif(
    pytest.importorskip('xacro', reason='xacro 需要 ROS 环境'),
    condition=False,
    reason='无 ROS 环境',
)
def test_build_arm_urdf_contains_gz_pieces():
    from ivg_sim.arm_description import build_arm_urdf

    urdf = build_arm_urdf()
    # 按元素形态断言（xacro 注释里含旧关节名，裸子串会误伤）
    import re
    assert not re.search(r'<link\b[^>]*\bname="world"', urdf)
    assert not re.search(r'<joint\b[^>]*\bname="(?:world_joint|fixed_base)"',
                         urdf)
    assert 'gz_ros2_control/GazeboSimSystem' in urdf
    for j in ARM_JOINTS + tuple(
            f'finger_{s}_joint' for s in ('xp', 'xn', 'yp', 'yn')):
        assert f'name="{j}"' in urdf
    assert 'name="tcp"' in urdf
    assert 'name="g2_base_link"' in urdf
    assert 'g2_base.stl' in urdf
