"""零 ROS：0xA5 v3 32 字节力帧切帧、通道/校验、克→牛、行程/AD."""

from serial_force.force_protocol import (
    channels_ok, checksum_ok, decode_force_frame, feed_force,
    FORCE_CHANNELS, FORCE_FRAME_LEN, G_TO_N, TRAVEL_LSB_MM,
    TRAVEL_SENTINEL)

# 2026-10-21 现场实测帧（见 doc/force_frame_v3.md）。
DOC_REAL = bytes.fromhex(
    'A5 01 F3 00 25 B0 06 02 18 01 2A D2 05 03 BE 00 1D 19 05 '
    '05 36 01 2F DC 05 07 09 01 28 2C 06 43')
DOC_ZERO = bytes.fromhex(
    'A5 01 00 00 00 00 00 02 00 00 00 00 00 03 00 00 00 00 00 '
    '05 00 00 00 00 00 07 00 00 00 00 00 B7')
DOC_SENTINEL = bytes.fromhex(
    'A5 01 F3 00 25 B0 06 02 18 01 3C D2 05 03 BE 00 1D 19 05 '
    '05 36 01 2F DC 05 07 09 01 28 2C 06 55')


def _frame(grams=(0, 0, 0, 0, 0), travel=(0, 0, 0, 0, 0),
           ad=(0, 0, 0, 0, 0), channels=FORCE_CHANNELS):
    """按 v3 协议组 32 字节帧；grams/AD 为 uint16，travel 为 0.1 mm 原始字节."""
    body = bytearray([0xA5])
    for ch, g, t, a in zip(channels, grams, travel, ad):
        body.append(ch)
        body += int(g).to_bytes(2, 'little')
        body.append(t)
        body += int(a).to_bytes(2, 'little')
    body.append(sum(body) & 0xFF)
    return bytes(body)


def test_doc_reference_frames():
    zero = decode_force_frame(DOC_ZERO)
    assert zero is not None
    assert zero.grams == (0, 0, 0, 0, 0)
    assert zero.travel_mm == (0.0, 0.0, 0.0, 0.0, 0.0)
    assert zero.ad == (0, 0, 0, 0, 0)
    assert zero.invalid == (False, False, False, False, False)

    real = decode_force_frame(DOC_REAL)
    assert real is not None
    assert real.grams == (243, 280, 190, 310, 265)
    assert [round(v, 1) for v in real.travel_mm] == [3.7, 4.2, 2.9, 4.7, 4.0]
    assert real.ad == (1712, 1490, 1305, 1500, 1580)
    assert real.invalid == (False, False, False, False, False)
    assert abs(real.force_n[0] - 243 * G_TO_N) < 1e-12
    assert abs(real.force_n[-1] - 265 * G_TO_N) < 1e-12


def test_full_scale_6mm_is_not_invalid():
    # 0x3C 是 370 g/6.0 mm 满量程，也是断线哨兵；协议层不再把 6.0 判无效，
    # 断线/饱和由 force_status 用 AD 区分。
    sample = decode_force_frame(DOC_SENTINEL)
    assert sample is not None
    assert sample.invalid == (False, False, False, False, False)
    assert sample.travel_mm[1] == 6.0
    assert abs(sample.force_n[1] - 280 * G_TO_N) < 1e-12
    assert sample.grams[1] == 280
    assert sample.ad[1] == 1490
    assert sample.travel_mm[0] == 3.7


def test_frame_builder_roundtrip_and_len():
    frame = _frame(
        grams=(0, 1, 65535, 2000, 1999),
        travel=(0, 1, 30, TRAVEL_SENTINEL, 255),
        ad=(0, 1, 4095, 1712, 65535))
    assert len(frame) == FORCE_FRAME_LEN == 32
    sample = decode_force_frame(frame)
    assert sample is not None
    assert sample.grams == (0, 1, 65535, 2000, 1999)
    assert sample.travel_mm[0] == 0.0
    assert sample.travel_mm[1] == TRAVEL_LSB_MM
    assert sample.travel_mm[2] == 30 * TRAVEL_LSB_MM
    assert sample.travel_mm[3] == 6.0
    assert sample.travel_mm[4] == 255 * TRAVEL_LSB_MM
    assert sample.invalid == (False, False, False, False, False)
    assert sample.ad == (0, 1, 4095, 1712, 65535)


def test_validators():
    assert checksum_ok(DOC_ZERO)
    assert checksum_ok(DOC_REAL)
    assert channels_ok(DOC_REAL)
    assert not checksum_ok(DOC_ZERO[:-1])
    assert not channels_ok(_frame(channels=(1, 2, 3, 5, 6)))


def test_bad_channel_and_checksum_rejected():
    assert decode_force_frame(_frame(channels=(1, 2, 3, 5, 6))) is None
    broken = bytearray(DOC_REAL)
    broken[-1] ^= 1
    assert decode_force_frame(bytes(broken)) is None
    assert decode_force_frame(DOC_ZERO[:-1]) is None


def test_feed_junk_prefix_and_back_to_back():
    buf = bytearray(b'\x00\xff') + DOC_ZERO + DOC_REAL
    samples = feed_force(buf)
    assert [s.grams for s in samples] == [
        (0, 0, 0, 0, 0), (243, 280, 190, 310, 265)]
    assert not buf


def test_feed_false_header_inside_payload():
    # 载荷里出现 0xA5（165 g）：不得把假帧头当起点丢帧。
    frame = _frame(grams=(0xA5, 0, 0, 0, 0))
    assert decode_force_frame(frame) is not None
    assert len(feed_force(bytearray(frame + frame))) == 2


def test_feed_skips_false_header_then_recovers():
    buf = bytearray(b'\xa5\x00\x00\x00') + DOC_REAL
    samples = feed_force(buf)
    assert len(samples) == 1
    assert samples[0].grams[0] == 243


def test_feed_partial_waits_for_more():
    buf = bytearray(DOC_ZERO[:-1])
    assert feed_force(buf) == []
    buf += DOC_ZERO[-1:]
    assert len(feed_force(buf)) == 1
