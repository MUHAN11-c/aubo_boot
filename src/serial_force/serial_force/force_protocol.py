"""
0xA5 32 字节五点力帧解析（纯 Python，不依赖 ROS）.

帧格式（v3，STM32 100 Hz 主动上报）：

    A5 | ch | gL gH | mm | aL aH | ... ×5 ... | sum

- ``ch``：输出通道号，固定 1/2/3/5/7；
- ``g``：uint16 小端，单位克（1 LSB = 1 g）；
- ``mm``：uint8，单位 0.1 mm；``0x3C`` 既是 6.0 mm 满量程，也是断线哨兵，
  由 :mod:`serial_force.force_status` 用原始 AD 区分，协议层不置无效；
- ``ad``：uint16 小端，STM32 转发前的原始 AD，仅诊断/标定用；
- ``sum``：前 31 字节累加和的低 8 位。
"""
from __future__ import annotations

from dataclasses import dataclass
import struct

FORCE_HEADER = b'\xa5'
FORCE_FRAME_LEN = 32
FORCE_BLOCK_LEN = 6
# 采集板输入 0/1/2/4/7 -> 输出重编号 1/2/3/5/7；点位见 doc/geometry.md。
FORCE_CHANNELS = (1, 2, 3, 5, 7)
# 协议单位克；ROS 接口按 REP-103 用 SI 牛顿。
G_TO_N = 9.80665 / 1000.0
TRAVEL_LSB_MM = 0.1


@dataclass(frozen=True)
class ForceFrame:
    """一帧五点力：克、力等效行程、原始 AD，顺序同 FORCE_CHANNELS."""

    grams: tuple[int, ...]
    travel_mm: tuple[float, ...]
    ad: tuple[int, ...]

    @property
    def force_n(self) -> tuple[float, ...]:
        """五点力，SI 牛顿；6.0 mm/370 g 是合法满量程，不返回 NaN."""
        return tuple(g * G_TO_N for g in self.grams)


def _checksum_ok(frame: bytes) -> bool:
    """校验和 = 除末字节外所有字节之和的低 8 位."""
    return len(frame) >= 2 and (sum(frame[:-1]) & 0xFF) == frame[-1]


def _channels_ok(frame: bytes) -> bool:
    """五个通道号必须为 1/2/3/5/7，避免载荷中的假 A5 误同步."""
    return len(frame) >= FORCE_FRAME_LEN and all(
        frame[1 + FORCE_BLOCK_LEN * i] == ch
        for i, ch in enumerate(FORCE_CHANNELS))


def decode_force_frame(frame: bytes) -> ForceFrame | None:
    """解码一帧；长度、通道号或校验不符返回 None."""
    if len(frame) != FORCE_FRAME_LEN:
        return None
    if not _channels_ok(frame) or not _checksum_ok(frame):
        return None
    grams: list[int] = []
    travel: list[float] = []
    ad: list[int] = []
    for i in range(len(FORCE_CHANNELS)):
        offset = 1 + FORCE_BLOCK_LEN * i
        grams.append(struct.unpack_from('<H', frame, offset + 1)[0])
        travel.append(frame[offset + 3] * TRAVEL_LSB_MM)
        ad.append(struct.unpack_from('<H', frame, offset + 4)[0])
    return ForceFrame(tuple(grams), tuple(travel), tuple(ad))


def feed_force(buf: bytearray) -> list[ForceFrame]:
    """从字节流切完整 A5 帧；假帧头跳一字节重扫，尾部残帧留在 buf."""
    frames: list[ForceFrame] = []
    while True:
        start = buf.find(FORCE_HEADER)
        if start < 0:
            buf.clear()
            return frames
        if start:
            del buf[:start]
        if len(buf) < FORCE_FRAME_LEN:
            return frames
        frame = decode_force_frame(bytes(buf[:FORCE_FRAME_LEN]))
        if frame is None:
            del buf[:1]
            continue
        del buf[:FORCE_FRAME_LEN]
        frames.append(frame)
