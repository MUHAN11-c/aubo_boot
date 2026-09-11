"""坐标系修正与对齐纯核：倒装 Rx(180°) 与 parent 误差清零."""
import math

from serial_imu.frame import (
    FramePipeline, from_rpy_deg, qangle, qconj, qmul, rpy_deg_from_quat)
from serial_imu.protocol import ImuSample


def _axis_quat(axis, deg):
    """轴角 → 四元数（w,x,y,z）."""
    rad = math.radians(deg) / 2.0
    s = math.sin(rad)
    return (math.cos(rad), axis[0] * s, axis[1] * s, axis[2] * s)


def _assert_close_quat(a, b, tol_deg=0.01):
    assert qangle(a, b) < tol_deg, f'{a} vs {b}'


def _sample(acc=(0.0, 0.0, -9.8), gyro=(0.1, 0.0, 0.0), mag=(1.0, 0.0, 0.0),
            q_wxyz=None, q_valid=True):
    if q_wxyz is None:
        q_wxyz = from_rpy_deg([180.0, 0.0, 0.0])
    w, x, y, z = q_wxyz
    return ImuSample(
        acc_x=acc[0], acc_y=acc[1], acc_z=acc[2],
        gyro_x=gyro[0], gyro_y=gyro[1], gyro_z=gyro[2],
        roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0,
        mag_level=0, temp_c=25.0,
        mag_x=mag[0], mag_y=mag[1], mag_z=mag[2],
        orientation_x=x, orientation_y=y, orientation_z=z, orientation_w=w,
        orientation_valid=q_valid,
    )


def test_raw_is_protocol_passthrough():
    """原始路不做倒装、不对齐."""
    pipe = FramePipeline([180.0, 0.0, 0.0])
    sample = _sample()
    raw = pipe.raw(sample)
    assert raw.acc == (0.0, 0.0, -9.8)
    assert raw.gyro == (0.1, 0.0, 0.0)
    _assert_close_quat(raw.q_wxyz, from_rpy_deg([180.0, 0.0, 0.0]))


def test_corrected_inverted_module_is_upright():
    """标准倒装 Rx(180°) 后，静止比力 +Z、姿态恒等（不是现场残差）."""
    pipe = FramePipeline([180.0, 0.0, 0.0])
    corr = pipe.corrected(_sample())
    assert abs(corr.acc[0]) < 1e-9
    assert abs(corr.acc[1]) < 1e-9
    assert abs(corr.acc[2] - 9.8) < 1e-9
    _assert_close_quat(corr.q_wxyz, (1.0, 0.0, 0.0, 0.0))
    assert abs(corr.gyro[0] - 0.1) < 1e-9
    assert abs(corr.mag[0] - 1.0) < 1e-9


def test_identity_frame_leaves_upright_unchanged():
    """物理正装 frame_rpy=[0,0,0] 时直立样本保持不变."""
    pipe = FramePipeline([0.0, 0.0, 0.0])
    corr = pipe.corrected(_sample(
        acc=(0.0, 0.0, 9.8), gyro=(0, 0, 0), mag=(0, 0, 0),
        q_wxyz=(1.0, 0.0, 0.0, 0.0)))
    assert abs(corr.acc[2] - 9.8) < 1e-9
    _assert_close_quat(corr.q_wxyz, (1.0, 0.0, 0.0, 0.0))


def test_capture_zeros_residual_onto_parent():
    """当前 IMU 相对 TCP 的差当误差：校正后姿态等于 parent."""
    pipe = FramePipeline([0.0, 0.0, 0.0])
    q_imu = from_rpy_deg([1.34, -2.41, -25.92])
    sample = _sample(acc=(0.40, 0.22, 9.58), q_wxyz=q_imu)
    framed = pipe.framed(sample)
    ok, msg = pipe.capture((1.0, 0.0, 0.0, 0.0), framed.q_wxyz)
    assert ok
    assert 'residual_rpy_deg' in msg
    corr = pipe.corrected(sample)
    _assert_close_quat(corr.q_wxyz, (1.0, 0.0, 0.0, 0.0))
    n = math.sqrt(sum(x * x for x in corr.acc))
    assert abs(corr.acc[2] / n - 1.0) < 0.01
    rpy = pipe.residual_rpy_deg()
    assert rpy is not None
    assert abs(rpy[2] - 25.92) < 0.2 or abs(rpy[2] + 25.92) < 0.2


def test_raw_stays_unaligned_after_capture():
    """对齐只作用于 corrected，raw 仍是模组体轴."""
    pipe = FramePipeline([0.0, 0.0, 0.0])
    q_imu = from_rpy_deg([0.0, 0.0, -30.0])
    sample = _sample(q_wxyz=q_imu)
    pipe.capture((1.0, 0.0, 0.0, 0.0), pipe.framed(sample).q_wxyz)
    raw = pipe.raw(sample)
    _assert_close_quat(raw.q_wxyz, q_imu)
    _assert_close_quat(pipe.corrected(sample).q_wxyz, (1.0, 0.0, 0.0, 0.0))


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
    _assert_close_quat(from_rpy_deg([0.0, 0.0, 90.0]),
                       _axis_quat((0.0, 0.0, 1.0), 90.0))
    assert qangle(q, qconj(q)) > 1e-3
    assert rpy_deg_from_quat((0.0, 0.0, 0.0, 0.0)) is None
