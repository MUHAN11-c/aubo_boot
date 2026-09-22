"""零 ROS：姿态增量/符号映射/死区限幅/平滑/关节步长钳制/插入推进（对错以实机为准）."""
import math

from imu_follow.follow_core import (
    apply_deadband, clamp_joint_step, clamp_rotvec, delta_rotvec,
    insertion_position, insertion_step, map_signs, quat_conj, quat_mul,
    quat_normalize, quat_rotate, quat_to_rotvec, retraction_step,
    rotvec_to_quat, scale_vector, slerp_toward, target_orientation)


def _mag(v):
    return math.sqrt(sum(x * x for x in v))


def _close(a, b, tol=1e-9):
    return len(a) == len(b) and all(abs(x - y) <= tol for x, y in zip(a, b))


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def test_delta_identity_is_zero():
    q = rotvec_to_quat((0.11, -0.07, 0.23))
    assert _close(delta_rotvec(q, q), (0.0, 0.0, 0.0))


def test_delta_known_axis():
    q_now = rotvec_to_quat((0.2, 0.0, 0.0))
    v = delta_rotvec(IDENTITY, q_now)
    assert _close(v, (0.2, 0.0, 0.0), tol=1e-6)


def test_quat_conj_inverts():
    q = rotvec_to_quat((0.1, 0.2, -0.3))
    assert _close(quat_normalize(quat_mul(q, quat_conj(q))), IDENTITY)


def test_rotvec_roundtrip():
    v = (0.012, -0.034, 0.056)
    assert _close(quat_to_rotvec(rotvec_to_quat(v)), v, tol=1e-9)


def test_map_signs_flips_only_sign():
    v = (0.1, -0.2, 0.3)
    out = map_signs(v, (1.0, -1.0, 1.0))
    assert _close(out, (0.1, 0.2, 0.3))
    assert _mag(out) == _mag(v)


def test_deadband_table():
    assert _close(apply_deadband((0.01, 0.0, 0.0), 0.02), (0.0, 0.0, 0.0))
    assert _close(apply_deadband((0.03, 0.0, 0.0), 0.02), (0.03, 0.0, 0.0))


def test_clamp_keeps_direction_and_limit():
    assert _close(clamp_rotvec((0.9, 0.0, 0.0), 0.35), (0.35, 0.0, 0.0))
    v = (0.1, 0.1, 0.1)
    assert _mag(clamp_rotvec(v, 0.35)) - _mag(v) < 1e-12


def test_target_orientation_composes_reference():
    q_tcp = rotvec_to_quat((0.1, 0.2, 0.0))
    q_imu = rotvec_to_quat((0.0, 0.0, 0.12))
    got = target_orientation(q_tcp, IDENTITY, q_imu,
                             (1.0, 1.0, 1.0), 0.0, 3.0)
    want = quat_normalize(quat_mul(q_tcp, q_imu))
    assert _close(got, want, tol=1e-6)


def test_target_orientation_respects_cone():
    q_big = rotvec_to_quat((1.0, 0.0, 0.0))
    got = target_orientation(IDENTITY, IDENTITY, q_big,
                             (1.0, 1.0, 1.0), 0.0, 0.35)
    v = delta_rotvec(IDENTITY, got)
    assert abs(_mag(v) - 0.35) < 1e-6


def test_target_orientation_deadband_holds_reference():
    q_small = rotvec_to_quat((0.01, 0.0, 0.0))
    got = target_orientation(IDENTITY, IDENTITY, q_small,
                             (1.0, 1.0, 1.0), 0.02, 0.35)
    assert _close(got, IDENTITY, tol=1e-6)


def test_slerp_endpoints_and_short_arc():
    a = rotvec_to_quat((0.05, 0.0, 0.0))
    b = rotvec_to_quat((0.0, 0.2, 0.0))
    assert _close(slerp_toward(a, b, 0.0), a, tol=1e-6)
    assert _close(slerp_toward(a, b, 1.0), b, tol=1e-6)
    neg = tuple(-x for x in b)  # 双覆盖另一表示仍走短弧
    assert _close(slerp_toward(a, neg, 1.0), b, tol=1e-6)


def test_clamp_joint_step_table():
    assert _close(clamp_joint_step((0.3, -0.3), (0.0, 0.0), 0.05),
                  (0.05, -0.05))
    assert _close(clamp_joint_step((0.03, -0.01), (0.0, 0.0), 0.05),
                  (0.03, -0.01))


def test_quat_rotate_known_axes():
    q = rotvec_to_quat((math.pi / 2.0, 0.0, 0.0))  # 绕 X 转 90°
    assert _close(quat_rotate(q, (1.0, 0.0, 0.0)), (1.0, 0.0, 0.0), tol=1e-9)
    assert _close(quat_rotate(q, (0.0, 1.0, 0.0)), (0.0, 0.0, 1.0), tol=1e-9)
    assert _close(quat_rotate(q, (0.0, 0.0, 1.0)), (0.0, -1.0, 0.0), tol=1e-9)


def test_scale_vector():
    assert _close(scale_vector((1.0, -2.0, 0.5), 2.0), (2.0, -4.0, 1.0))


def test_insertion_step_integrates_and_caps():
    # 正常积分：0 + 0.01 m/s × 0.05 s（20 Hz 一拍）
    assert abs(insertion_step(0.0, 0.01, 0.05, 0.20) - 0.0005) < 1e-12
    # 累积积分
    travel = 0.0
    for _ in range(400):  # 20 s @ 20 Hz × 0.01 m/s = 0.20 m
        travel = insertion_step(travel, 0.01, 0.05, 0.20)
    assert abs(travel - 0.20) < 1e-9
    # 行程封顶：不越界
    assert insertion_step(0.195, 0.01, 1.0, 0.20) == 0.20
    # 负时长/负速度不回退（钳 0）
    assert insertion_step(0.1, 0.01, -1.0, 0.20) == 0.1
    assert insertion_step(0.1, -0.05, 1.0, 0.20) == 0.1


def test_retraction_step_decreases_and_floors():
    assert abs(retraction_step(0.10, 0.01, 0.05) - 0.0995) < 1e-12
    travel = 0.20
    for _ in range(400):
        travel = retraction_step(travel, 0.01, 0.05)
    assert travel == 0.0
    assert retraction_step(0.005, 0.01, 1.0) == 0.0
    assert retraction_step(0.1, 0.01, -1.0) == 0.1
    assert retraction_step(0.1, -0.05, 1.0) == 0.1


def test_insertion_position_moves_along_direction():
    ref = (1.0, 2.0, 3.0)
    # 零行程 = 参考点（未插入时目标即参考）
    assert _close(insertion_position(ref, (0.0, 0.0, 1.0), 0.0), ref)
    # 沿 +Z 推进 0.1 m
    assert _close(insertion_position(ref, (0.0, 0.0, 1.0), 0.1),
                  (1.0, 2.0, 3.1))
    # 斜向单位方向推进：位移模长 = 行程
    d = (0.0, 1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0))
    out = insertion_position(ref, d, 0.2)
    delta = tuple(out[i] - ref[i] for i in range(3))
    assert abs(_mag(delta) - 0.2) < 1e-9
