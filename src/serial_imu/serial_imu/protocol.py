"""0xA4 寄存器 IMU 帧解析（无 ROS）."""
from __future__ import annotations

from dataclasses import dataclass
import math
import struct

G = 9.8
DEG2RAD = math.pi / 180.0
# 主动上报：起始寄存器 0x08、35 字节载荷 + 头 4 + 校验 1 = 40。
FULL_START_REG = 0x08
FULL_PAYLOAD_LEN = 0x23
MAX_FRAME = 256


@dataclass(frozen=True)
class ImuSample:
    """一帧解算结果；角速度 rad/s，加速度 m/s^2，四元数 ROS xyzw."""

    acc_x: float
    acc_y: float
    acc_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    mag_level: int
    temp_c: float
    mag_x: float
    mag_y: float
    mag_z: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float


def checksum_ok(frame: bytes) -> bool:
    """校验和为除末字节外所有字节之和的低 8 位."""
    if len(frame) < 5:
        return False
    return (sum(frame[:-1]) & 0xFF) == frame[-1]


def decode_full_payload(payload: bytes) -> ImuSample | None:
    """解 0x08 起 35 字节载荷；Q0 作 orientation.w（ROS xyzw）."""
    if len(payload) < FULL_PAYLOAD_LEN:
        return None
    u = struct.unpack_from('<hhhhhhhhhBhhhhhhhh', payload, 0)
    q0, q1, q2, q3 = (u[14] / 10000.0, u[15] / 10000.0,
                      u[16] / 10000.0, u[17] / 10000.0)
    return ImuSample(
        acc_x=u[0] / 2048.0 * G,
        acc_y=u[1] / 2048.0 * G,
        acc_z=u[2] / 2048.0 * G,
        gyro_x=u[3] / 16.4 * DEG2RAD,
        gyro_y=u[4] / 16.4 * DEG2RAD,
        gyro_z=u[5] / 16.4 * DEG2RAD,
        roll_deg=u[6] / 100.0,
        pitch_deg=u[7] / 100.0,
        yaw_deg=u[8] / 100.0,
        mag_level=int(u[9]),
        temp_c=u[10] / 100.0,
        mag_x=u[11] / 1000.0,
        mag_y=u[12] / 1000.0,
        mag_z=u[13] / 1000.0,
        orientation_x=q1,
        orientation_y=q2,
        orientation_z=q3,
        orientation_w=q0,
    )


def feed(buf: bytearray) -> list[ImuSample]:
    """从字节流切完整 A4 03 帧并解码；损坏字节丢弃."""
    samples: list[ImuSample] = []
    while True:
        start = buf.find(b'\xa4\x03')
        if start < 0:
            buf.clear()
            return samples
        if start:
            del buf[:start]
        if len(buf) < 5:
            return samples
        count = buf[3]
        need = 4 + count + 1
        if count == 0 or need > MAX_FRAME:
            del buf[0]
            continue
        if len(buf) < need:
            return samples
        frame = bytes(buf[:need])
        del buf[:need]
        if not checksum_ok(frame):
            continue
        if frame[2] != FULL_START_REG or count != FULL_PAYLOAD_LEN:
            continue
        sample = decode_full_payload(frame[4:-1])
        if sample is not None:
            samples.append(sample)
