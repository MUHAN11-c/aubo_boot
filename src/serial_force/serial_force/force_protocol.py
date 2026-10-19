"""0xA5 帧头 5 点力传感器帧解析（无 ROS）."""
from __future__ import annotations

from dataclasses import dataclass
import struct

# 咬合末端 5 点力传感器（IMS-C04A，量程 50g–2kg）：100Hz 主动上报，
# 17 字节定长帧 = A5 + 5×(通道号 + int16 小端 kgf×100) + 累加和。
FORCE_HEADER = 0xA5
FORCE_FRAME_LEN = 17
# 帧内通道号：采集板输入 0/1/2/4/7 → 输出重编号 1/2/3/5/7。
# 对应咬合末端圆盘点位见 config/serial_force.yaml channel_labels。
FORCE_CHANNELS = (1, 2, 3, 5, 7)
# 协议原生单位 kgf；ROS 接口按 REP-103 用 SI 牛顿。
KGF_TO_N = 9.80665


@dataclass(frozen=True)
class ForceSample:
    """一帧 5 通道力值（kgf），顺序同 FORCE_CHANNELS."""

    kgf: tuple[float, float, float, float, float]


def checksum_ok(frame: bytes) -> bool:
    """校验和 = 除末字节外所有字节之和的低 8 位."""
    if len(frame) < 2:
        return False
    return (sum(frame[:-1]) & 0xFF) == frame[-1]


def channels_ok(frame: bytes) -> bool:
    """5 个通道号须为 1/2/3/5/7；防载荷里的假 A5 误同步."""
    return all(
        frame[1 + 3 * i] == ch for i, ch in enumerate(FORCE_CHANNELS))


def decode_force_frame(frame: bytes) -> ForceSample | None:
    """解 17 字节帧；长度、通道号或校验不符返回 None."""
    if len(frame) != FORCE_FRAME_LEN:
        return None
    if not channels_ok(frame) or not checksum_ok(frame):
        return None
    vals = [struct.unpack_from('<h', frame, 2 + 3 * i)[0] / 100.0
            for i in range(len(FORCE_CHANNELS))]
    return ForceSample(kgf=(vals[0], vals[1], vals[2], vals[3], vals[4]))


def feed_force(buf: bytearray) -> list[ForceSample]:
    """从字节流切完整 A5 力帧并解码；损坏字节丢弃."""
    samples: list[ForceSample] = []
    while True:
        start = buf.find(b'\xa5')
        if start < 0:
            buf.clear()
            return samples
        if start:
            del buf[:start]
        if len(buf) < FORCE_FRAME_LEN:
            return samples
        frame = bytes(buf[:FORCE_FRAME_LEN])
        sample = decode_force_frame(frame)
        if sample is None:
            # 假帧头（载荷里也可能出现 A5）：跳一字节继续找。
            del buf[:1]
            continue
        del buf[:FORCE_FRAME_LEN]
        samples.append(sample)

