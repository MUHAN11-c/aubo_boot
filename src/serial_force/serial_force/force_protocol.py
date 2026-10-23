"""0xA5 帧头 5 点力传感器帧解析（无 ROS）.

2026-10-21 起为 32 字节 v3 帧（v2 基础上每通道增加原始 AD）：

    A5 | ch | gL gH | mm | aL aH | ... ×5 ... | sum

- 力：uint16 小端，单位克（1 LSB = 1 g）；
- 行程：uint8，×10（0.1 mm/LSB）；0x3C = 6.0 mm = 满量程（370 g）点；
- 原始 AD：uint16 小端，标定/诊断用，不直接换算力；
- 6.0 mm 同时被 STM32 用作断线哨兵；断线/饱和由 force_status 用 AD 区分，
  不再在协议层把 0x3C 置为无效。
- 校验：前 31 字节累加和的低 8 位。
"""
from __future__ import annotations

from dataclasses import dataclass
import struct

FORCE_HEADER = 0xA5
FORCE_FRAME_LEN = 32
FORCE_BLOCK_LEN = 6
# 帧内通道号：采集板输入 0/1/2/4/7 → 输出重编号 1/2/3/5/7。
# 对应咬合末端圆盘点位见 config/serial_force.yaml channel_labels。
FORCE_CHANNELS = (1, 2, 3, 5, 7)
# 协议原生单位克；ROS 接口按 REP-103 用 SI 牛顿。
G_TO_N = 9.80665 / 1000.0
# 行程 1 LSB = 0.1 mm；0x3C（60）是 6.0 mm 满量程/断线/超量程哨兵。
TRAVEL_LSB_MM = 0.1
TRAVEL_FULL = 0x3C
TRAVEL_SENTINEL = TRAVEL_FULL  # 兼容旧名；不再表示 invalid


@dataclass(frozen=True)
class ForceSample:
    """一帧 5 通道力（g）、行程（mm）与原始 AD，顺序同 FORCE_CHANNELS."""

    grams: tuple[int, int, int, int, int]
    travel_mm: tuple[float, float, float, float, float]
    ad: tuple[int, int, int, int, int]
    invalid: tuple[bool, bool, bool, bool, bool]

    @property
    def force_n(self) -> tuple[float, float, float, float, float]:
        """牛顿；6.0 mm/370 g 是合法满量程，不再返回 NaN."""
        return tuple(g * G_TO_N for g in self.grams)


def checksum_ok(frame: bytes) -> bool:
    """校验和 = 除末字节外所有字节之和的低 8 位."""
    if len(frame) < 2:
        return False
    return (sum(frame[:-1]) & 0xFF) == frame[-1]


def channels_ok(frame: bytes) -> bool:
    """5 个通道号须为 1/2/3/5/7；防载荷里的假 A5 误同步."""
    if len(frame) < FORCE_FRAME_LEN:
        return False
    return all(
        frame[1 + FORCE_BLOCK_LEN * i] == ch
        for i, ch in enumerate(FORCE_CHANNELS))


def decode_force_frame(frame: bytes) -> ForceSample | None:
    """解 32 字节帧；长度、通道号或校验不符返回 None."""
    if len(frame) != FORCE_FRAME_LEN:
        return None
    if not channels_ok(frame) or not checksum_ok(frame):
        return None
    grams: list[int] = []
    travel: list[float] = []
    ad: list[int] = []
    invalid: list[bool] = []
    for i in range(len(FORCE_CHANNELS)):
        off = 1 + FORCE_BLOCK_LEN * i
        grams.append(struct.unpack_from('<H', frame, off + 1)[0])
        raw = frame[off + 3]
        # 0x3C 是满量程 6.0 mm，也是断线哨兵；协议层不区分，交给 status。
        invalid.append(False)
        travel.append(raw * TRAVEL_LSB_MM)
        ad.append(struct.unpack_from('<H', frame, off + 4)[0])
    return ForceSample(
        grams=(grams[0], grams[1], grams[2], grams[3], grams[4]),
        travel_mm=(travel[0], travel[1], travel[2], travel[3], travel[4]),
        ad=(ad[0], ad[1], ad[2], ad[3], ad[4]),
        invalid=(invalid[0], invalid[1], invalid[2], invalid[3],
                 invalid[4]))


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
