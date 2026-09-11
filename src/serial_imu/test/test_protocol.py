"""零 ROS：0xA4 切帧、四元数归一、sensor_msgs 协方差约定."""
import math
import struct

from serial_imu.protocol import (
    checksum_ok, covariance_diag, decode_full_payload, feed,
    FULL_PAYLOAD_LEN, FULL_START_REG, G)


def _payload(
        acc=(0, 0, 2048),
        gyro=(0, 0, 0),
        rpy=(0, 0, 0),
        mag_level=0,
        temp=2500,
        mag=(0, 0, 0),
        quat=(10000, 0, 0, 0)):
    """组 35 字节载荷；quat=(Q0,Q1,Q2,Q3) int16."""
    fields = list(acc) + list(gyro) + list(rpy)
    rest = list(mag) + list(quat)
    return struct.pack('<hhhhhhhhhBhhhhhhhh', *fields, mag_level, temp, *rest)


def _frame(payload: bytes) -> bytes:
    body = bytes([0xA4, 0x03, FULL_START_REG, FULL_PAYLOAD_LEN]) + payload
    return body + bytes([sum(body) & 0xFF])


def test_covariance_diag_sensor_msgs_rules():
    unused = covariance_diag(-1.0)
    assert unused[0] == -1.0
    assert unused[1:] == [0.0] * 8
    assert covariance_diag(0.0) == [0.0] * 9
    known = covariance_diag(0.04)
    assert known[0] == known[4] == known[8] == 0.04
    assert known[1] == known[2] == 0.0


def test_identity_quat_and_acc_scale():
    sample = decode_full_payload(_payload())
    assert sample is not None
    assert sample.orientation_valid
    assert sample.orientation_w == 1.0
    assert sample.orientation_x == sample.orientation_y == 0.0
    assert abs(sample.acc_z - G) < 1e-9
    assert sample.gyro_x == sample.gyro_y == sample.gyro_z == 0.0


def test_quat_normalized_and_invalid_rejected():
    sample = decode_full_payload(_payload(quat=(5000, 5000, 0, 0)))
    n = math.sqrt(sample.orientation_w ** 2 + sample.orientation_x ** 2)
    assert sample.orientation_valid
    assert abs(n - 1.0) < 1e-9
    assert abs(sample.orientation_w - sample.orientation_x) < 1e-9
    bad = decode_full_payload(_payload(quat=(0, 0, 0, 0)))
    assert not bad.orientation_valid
    assert bad.orientation_w == 1.0


def test_feed_checksum_and_junk_prefix():
    good = _frame(_payload())
    assert checksum_ok(good)
    buf = bytearray(b'\x00\xff') + good
    samples = feed(buf)
    assert len(samples) == 1
    assert samples[0].orientation_valid
    broken = bytearray(good[:-1] + bytes([(good[-1] + 1) & 0xFF]))
    assert feed(broken) == []
