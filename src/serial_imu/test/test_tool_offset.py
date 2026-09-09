"""工具偏移纯核测试：零 ROS，只测 tool_offset 解算与清零自检门."""
import math

from serial_imu.tool_offset import (
    from_rpy_deg, normalize, qangle, qconj, qmul, qrot,
    ToolOffsetEstimator, zero_gate_ok)

TILT_DEG = 10.0


def _axis_quat(axis, deg):
    """轴角 → 四元数（w,x,y,z）."""
    rad = math.radians(deg) / 2.0
    s = math.sin(rad)
    return normalize((math.cos(rad), axis[0] * s, axis[1] * s, axis[2] * s))


def _assert_close_quat(a, b, tol_deg=0.01):
    assert qangle(a, b) < tol_deg, f'{a} vs {b}'


def test_zero_then_same_input_is_identity():
    est = ToolOffsetEstimator(from_rpy_deg([177.0, 0.0, 0.0]))
    q0 = _axis_quat((0.0, 0.0, 1.0), 30.0)
    qbn0 = _axis_quat((1.0, 0.0, 0.0), 15.0)
    assert est.zero(q0, qbn0)
    q_off, trans = est.update(q0, qbn0)
    _assert_close_quat(q_off, (1.0, 0.0, 0.0, 0.0))
    assert trans == (0.0, 0.0, 0.0)


def test_inverted_mount_rigid_arm_motion_is_identity():
    """倒装 mount（roll≈180°）下刚性臂运动必须解出恒等偏移（回归 2026-09-09 实测）."""
    # mount 共轭缺失会把臂运动误报为工具偏移。
    mount = from_rpy_deg([177.0, 0.0, 0.0])
    est = ToolOffsetEstimator(mount)
    q0 = _axis_quat((0.0, 1.0, 0.0), 40.0)  # 模组世界与臂基座无约束，任意
    qbn0 = _axis_quat((0.0, 0.0, 1.0), 25.0)
    assert est.zero(q0, qbn0)
    # 臂动 A，模组刚接 → Δ = M⁻¹·A·M（体坐标系下两者姿态变化等价）
    for a_deg in (5.0, -12.0, 30.0):
        a_move = _axis_quat((0.3, 0.4, 0.0), a_deg)
        qbn_t = qmul(qbn0, a_move)
        delta = qmul(qmul(qconj(mount), a_move), mount)
        q_imu_t = qmul(q0, delta)
        q_off, _ = est.update(q_imu_t, qbn_t)
        _assert_close_quat(q_off, (1.0, 0.0, 0.0, 0.0))


def test_deflection_axis_survives_inverted_mount():
    """臂不动、工具绕工具系 Y 前倾 10° → 必须解出同一旋转，不得镜像."""
    mount = from_rpy_deg([177.0, 0.0, 0.0])
    est = ToolOffsetEstimator(mount)
    q0 = _axis_quat((1.0, 0.0, 0.0), 70.0)
    qbn0 = _axis_quat((0.0, 0.0, 1.0), 10.0)
    assert est.zero(q0, qbn0)
    deflect = _axis_quat((0.0, 1.0, 0.0), TILT_DEG)
    # 臂静止（A=I）→ Δ = M⁻¹·D·M
    delta = qmul(qmul(qconj(mount), deflect), mount)
    q_off, _ = est.update(qmul(q0, delta), qbn0)
    _assert_close_quat(q_off, deflect)


def test_pivot_translation():
    """铰点在 −Z 深 0.05 m，绕 Y 倾 90° → 筒口平移 = R·(0,0,d)+(0,0,−d)."""
    mount = from_rpy_deg([0.0, 0.0, 0.0])
    est = ToolOffsetEstimator(mount, pivot_depth_m=0.05)
    assert est.zero((1.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    quarter = _axis_quat((0.0, 1.0, 0.0), 90.0)
    q_off, trans = est.update(qmul((1.0, 0.0, 0.0, 0.0),
                                   qmul(qconj(mount), quarter)), (1.0, 0.0, 0.0, 0.0))
    _assert_close_quat(q_off, quarter)
    tip = qrot(quarter, (0.0, 0.0, 0.05))
    assert abs(trans[0] - tip[0]) < 1e-9
    assert abs(trans[1] - tip[1]) < 1e-9
    assert abs(trans[2] - (tip[2] - 0.05)) < 1e-9


def test_unzeroed_returns_none_and_bad_quat_rejected():
    est = ToolOffsetEstimator(from_rpy_deg([0.0, 0.0, 0.0]))
    assert est.update((1.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)) is None
    assert not est.zero((0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    assert normalize((0.0, 0.0, 0.0, 0.0)) is None


def test_zero_gate_rejects_upright_assumption_for_flipped_probe():
    """模组倒装（实测 acc 沿 −Z）配「开口朝上零位」预期：mount 未标定时须拒绝."""
    acc_flipped = (-2.14, 0.5, -9.34)  # 2026-09-09 静置段实测均值
    g_tool = (0.0, 0.0, 1.0)
    assert not zero_gate_ok(acc_flipped, (1.0, 0.0, 0.0, 0.0), g_tool, 5.0)
    # mount 标定为倒装后，纯倒装（无俯仰）应通过：
    # 预期 M⁻¹·g = Rx(-177°)·(0,0,1) = (0, +0.052, -0.999)
    assert zero_gate_ok((0.0, 0.052, -0.999), from_rpy_deg([177.0, 0.0, 0.0]),
                        g_tool, 5.0)
    # 带 13° 俯仰（不在零位 / 贴歪）仍须拒绝
    assert not zero_gate_ok(acc_flipped, from_rpy_deg([177.0, 0.0, 0.0]),
                            g_tool, 5.0)
    assert not zero_gate_ok((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), g_tool, 5.0)


def test_qangle_and_from_rpy_roundtrip():
    """from_rpy(ZYX) == Rz⊗Ry⊗Rx 组合."""
    q = from_rpy_deg([30.0, -20.0, 10.0])
    expect = qmul(_axis_quat((0.0, 0.0, 1.0), 10.0),
                  qmul(_axis_quat((0.0, 1.0, 0.0), -20.0),
                       _axis_quat((1.0, 0.0, 0.0), 30.0)))
    _assert_close_quat(q, expect)
    assert qangle(q, q) < 1e-4
    assert abs(qangle((1.0, 0.0, 0.0, 0.0),
                      _axis_quat((0.0, 1.0, 0.0), 90.0)) - 90.0) < 1e-6
    assert qmul((1.0, 0.0, 0.0, 0.0), q) == q
    # 纯轴角特例与逆
    _assert_close_quat(from_rpy_deg([0.0, 0.0, 90.0]),
                       _axis_quat((0.0, 0.0, 1.0), 90.0))
    assert qangle(q, qconj(q)) > 1e-3
