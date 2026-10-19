"""TCP 接近姿态：+Z 与袋轴同向（零 ROS 计算，导入编排模块）."""

import math

from sim_bringup.orchard_harvest import tool_orientation


def _z_column(quaternion):
    """四元数对应旋转矩阵的第三列."""
    x, y, z, w = quaternion
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


def test_tool_z_follows_bag_axis():
    """接近方向 (1, 0, 0) 时 TCP +Z 与之重合."""
    quaternion = tool_orientation((2.0, 0.0, 0.0))
    zx, zy, zz = _z_column(quaternion)
    assert math.isclose(zx, 1.0, abs_tol=1e-9)
    assert math.isclose(zy, 0.0, abs_tol=1e-9)
    assert math.isclose(zz, 0.0, abs_tol=1e-9)
    norm = math.sqrt(sum(item * item for item in quaternion))
    assert math.isclose(norm, 1.0, abs_tol=1e-9)


def test_vertical_axis_is_unit_quaternion():
    """竖直袋轴不退化，四元数仍单位长."""
    quaternion = tool_orientation((0.0, 0.0, 1.0))
    norm = math.sqrt(sum(item * item for item in quaternion))
    zx, zy, zz = _z_column(quaternion)
    assert math.isclose(norm, 1.0, abs_tol=1e-9)
    assert math.isclose(zx, 0.0, abs_tol=1e-9)
    assert math.isclose(zy, 0.0, abs_tol=1e-9)
    assert math.isclose(zz, 1.0, abs_tol=1e-9)
