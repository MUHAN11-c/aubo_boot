"""零 ROS：0xA5 力帧切帧、通道校验、kgf 缩放、A4/A5 同口分流."""
from serial_imu.force_protocol import (
    channels_ok, checksum_ok, decode_force_frame, feed_force, feed_mux,
    FORCE_CHANNELS, FORCE_FRAME_LEN)

# 厂商协议文档回归帧：全零与五点受压样例。
DOC_ZERO = bytes.fromhex(
    'A5 01 00 00 02 00 00 03 00 00 05 00 00 07 00 00 B7')
DOC_LOADED = bytes.fromhex(
    'A5 01 14 00 02 64 00 03 D2 00 05 37 00 07 C7 00 FF')


def _frame(vals=(0, 0, 0, 0, 0), channels=FORCE_CHANNELS):
    """按协议组 17 字节帧；vals 为 5 通道 int16×100 kgf."""
    body = bytearray([0xA5])
    for ch, v in zip(channels, vals):
        body.append(ch)
        body += v.to_bytes(2, 'little', signed=True)
    body.append(sum(body) & 0xFF)
    return bytes(body)


def test_doc_reference_frames():
    zero = decode_force_frame(DOC_ZERO)
    assert zero is not None
    assert zero.kgf == (0.0, 0.0, 0.0, 0.0, 0.0)
    loaded = decode_force_frame(DOC_LOADED)
    assert loaded is not None
    assert loaded.kgf == (0.20, 1.00, 2.10, 0.55, 1.99)


def test_frame_builder_roundtrip_and_len():
    frame = _frame(vals=(-50, 5, 12345, -32768, 32767))
    assert len(frame) == FORCE_FRAME_LEN
    sample = decode_force_frame(frame)
    assert sample is not None
    assert sample.kgf == (-0.50, 0.05, 123.45, -327.68, 327.67)


def test_validators():
    assert checksum_ok(DOC_ZERO)
    assert channels_ok(DOC_LOADED)
    assert not checksum_ok(DOC_ZERO[:-1])
    assert not channels_ok(_frame(channels=(1, 2, 3, 5, 6)))


def test_bad_channel_and_checksum_rejected():
    assert decode_force_frame(_frame(channels=(1, 2, 3, 5, 6))) is None
    broken = bytearray(DOC_LOADED)
    broken[-1] ^= 1
    assert decode_force_frame(bytes(broken)) is None
    assert decode_force_frame(DOC_ZERO[:-1]) is None


def test_feed_junk_prefix_and_back_to_back():
    buf = bytearray(b'\x00\xff') + DOC_ZERO + DOC_LOADED
    samples = feed_force(buf)
    assert [s.kgf for s in samples] == [
        (0.0, 0.0, 0.0, 0.0, 0.0), (0.20, 1.00, 2.10, 0.55, 1.99)]
    assert not buf


def test_feed_false_header_inside_payload():
    # 载荷里出现 0xA5（1.65 kgf）：不得把假帧头当起点丢帧。
    frame = _frame(vals=(0xA5, 0, 0, 0, 0))
    assert decode_force_frame(frame) is not None
    assert len(feed_force(bytearray(frame + frame))) == 2


def test_feed_skips_false_header_then_recovers():
    buf = bytearray(b'\xa5\x00\x00\x00') + DOC_LOADED
    samples = feed_force(buf)
    assert len(samples) == 1
    assert samples[0].kgf[0] == 0.20


def test_feed_partial_waits_for_more():
    buf = bytearray(DOC_ZERO[:-1])
    assert feed_force(buf) == []
    buf += DOC_ZERO[-1:]
    assert len(feed_force(buf)) == 1


def test_feed_mux_real_interleave():
    # 2026-09-29 /dev/imu 真实捕获开头：A5 全零力帧 + A4 IMU 帧（零载荷，
    # 校验 d2）交错。分流后两类各归各、缓冲清空。
    imu_frame = bytes.fromhex('a4030823') + bytes(35) + bytes.fromhex('d2')
    buf = bytearray(DOC_ZERO + imu_frame + DOC_LOADED + imu_frame)
    imu, force = feed_mux(buf)
    assert len(imu) == 2
    assert not imu[0].orientation_valid
    assert [f.kgf for f in force] == [
        (0.0, 0.0, 0.0, 0.0, 0.0), (0.20, 1.00, 2.10, 0.55, 1.99)]
    assert not buf


def test_feed_mux_junk_and_partial():
    imu_frame = bytes.fromhex('a4030823') + bytes(35) + bytes.fromhex('d2')
    buf = bytearray(b'\x00\xff') + DOC_ZERO + imu_frame
    imu, force = feed_mux(buf)
    assert len(imu) == 1 and len(force) == 1
    assert not buf
    # 半截 A4 帧留在缓冲等更多字节，不丢。
    buf = bytearray(DOC_ZERO + imu_frame[:20])
    imu, force = feed_mux(buf)
    assert len(force) == 1 and imu == []
    assert bytes(buf) == imu_frame[:20]
