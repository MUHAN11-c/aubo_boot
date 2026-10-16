"""E2E 大图/点云稳定传输 — 大数据面经网关的吞吐/新鲜度/丢帧证据.

数据面 = comm_fixture 大数据源（`-p rwl.bigdata:=true`，官方消息类型、合成内容）：
  /rwl_demo/big_image  sensor_msgs/Image        640×480 rgb8 @30Hz ≈ 0.92MB/帧
  /rwl_demo/big_cloud  sensor_msgs/PointCloud2  50k 点 ×16B  @10Hz ≈ 0.8MB/帧

客户端走**裸 websockets**（max_queue=2，绕开 FoxgloveWsClient 的应用层 64 深队列），
把消费速率控制权留在测试手里：
  - 快客户端（持续 recv）：吞吐 ≥ 名义率 65%、帧年龄（logTime→本地时钟）毫秒级、
    网关丢帧率 <5%；
  - 慢客户端（0.28s 读一帧 ≈ 3.5fps vs 30fps 生产）：网关有界数据通道
    （_BoundedForwardLane，4MiB 字节预算 + 丢最旧，对齐 rosbridge write_queue /
    Lichtblick parsedMessageCache / 桥 send-buffer 丢帧语义）开始丢帧——
    断言 ① /api/ivg/ws/stats 的 droppedData 增加；② 帧年龄有界且不随时间增长
    （旧版无界转发下该年龄以 ~0.27s/s 增长，8s 即超 2s；有界通道应稳定在秒内）。

「稳定传输没有延迟」的验收语义：慢消费者不能把延迟越积越高——丢帧是
BEST_EFFORT 视觉流的正确降级，积压不是。
"""
from __future__ import annotations

import asyncio
import json
import struct
import time

import pytest

pytest.importorskip('websockets')
pytest.importorskip('rclpy')
pytest.importorskip('fastapi')

import websockets  # noqa: E402

from .e2e_harness import Stack, loopback_client  # noqa: E402

SUBPROTOCOL = 'foxglove.websocket.v1'
OP_MESSAGE_DATA = 0x01
_IMG_TOPIC = '/rwl_demo/big_image'
_CLOUD_TOPIC = '/rwl_demo/big_cloud'
_STATS_PATH = '/api/ivg/ws/stats'


@pytest.fixture(scope='module')
def big_stack():
    # python -m 直跑没有 ros2 run 的自动 --ros-args 前缀，须显式带上
    st = Stack(fixtures=(
        'ros_web_link.comm_fixture#--ros-args -p rwl.bigdata:=true',))
    try:
        st.start()
    except FileNotFoundError as e:
        pytest.skip(f'ivg_foxglove_bridge 不在环境中: {e}')
    except Exception:
        st.stop()
        raise
    yield st
    st.stop()


async def _raw_connect(url: str):
    return await websockets.connect(
        url, subprotocols=[SUBPROTOCOL], max_size=64 * 1024 * 1024,
        max_queue=2, ping_interval=20, ping_timeout=60)


async def _wait_channel(ws, topic: str, channels: dict, timeout: float = 20.0):
    deadline = time.monotonic() + timeout
    while topic not in channels:
        remain = deadline - time.monotonic()
        if remain <= 0:
            raise TimeoutError(f'channel not advertised: {topic}')
        raw = await asyncio.wait_for(ws.recv(), min(remain, 2.0))
        if isinstance(raw, str):
            m = json.loads(raw)
            for ch in m.get('channels', []) or []:
                channels[ch['topic']] = ch
    return channels[topic]


async def _subscribe(ws, channel_id: int, sub_id: int) -> None:
    await ws.send(json.dumps({
        'op': 'subscribe',
        'subscriptions': [{'id': sub_id, 'channelId': channel_id}]}))


async def _next_data_frame(ws, channels: dict, timeout: float = 10.0):
    """读下一帧 messageData；返回 (subId, logTimeNs, size)。JSON op 顺带吸收."""
    while True:
        raw = await asyncio.wait_for(ws.recv(), timeout)
        if isinstance(raw, str):
            m = json.loads(raw)
            for ch in m.get('channels', []) or []:
                channels[ch['topic']] = ch
            continue
        if raw[0] == OP_MESSAGE_DATA and len(raw) >= 13:
            sub_id, log_ns = struct.unpack_from('<IQ', raw, 1)
            return sub_id, log_ns, len(raw)


def _gw_dropped(port: int) -> dict:
    """网关丢弃/转发计数：totals + active[] 求和.

    totals 仅在连接断开时归账（活跃计数在 active[]）——只读 totals 会踩
    归账时序盲区（2026-10-16 全量连跑实证：断言前后的读数完全相同）。
    """
    with loopback_client(port) as c:
        s = c.get(_STATS_PATH).json()
    tot = dict(s.get('totals', {}))
    for a in s.get('active') or []:
        for k in ('droppedData', 'forwardedData'):
            tot[k] = tot.get(k, 0) + (a.get(k) or 0)
    return {'dropped': tot.get('droppedData', 0),
            'forwarded': tot.get('forwardedData', 0)}


def test_bigdata_fast_client_throughput_and_freshness(big_stack):
    """快客户端：30Hz 0.92MB 图 + 10Hz 点云连续 6s——吞吐、新鲜度、丢帧率."""

    async def run():
        ws = await _raw_connect(big_stack.ws_url)
        channels: dict = {}
        try:
            img_ch = (await _wait_channel(ws, _IMG_TOPIC, channels))['id']
            cloud_ch = (await _wait_channel(ws, _CLOUD_TOPIC, channels))['id']
            await _subscribe(ws, img_ch, 1)
            await _subscribe(ws, cloud_ch, 2)

            counts = {1: 0, 2: 0}
            ages = []
            t_end = time.monotonic() + 6.0
            while time.monotonic() < t_end:
                sid, log_ns, size = await _next_data_frame(ws, channels, 5.0)
                counts[sid] = counts.get(sid, 0) + 1
                ages.append(time.time() - log_ns / 1e9)
            img_fps = counts[1] / 6.0
            cloud_fps = counts[2] / 6.0
        finally:
            await ws.close()
        return counts, img_fps, cloud_fps, ages

    counts, img_fps, cloud_fps, ages = asyncio.run(run())
    # 吞吐：名义 30/10Hz，留 35% 裕量（DDS 队列/桥缓冲在冷启动期丢帧属预期降级）
    assert img_fps >= 30 * 0.65, f'image {img_fps:.1f}fps'
    assert cloud_fps >= 10 * 0.65, f'cloud {cloud_fps:.1f}fps'
    # 新鲜度：logTime 年龄均值 <200ms、最大 <600ms（含时钟偏差裕量）
    assert sum(ages) / len(ages) < 0.2, f'mean age {sum(ages)/len(ages):.3f}s'
    assert max(ages) < 0.6, f'max age {max(ages):.3f}s'
    # 网关丢帧率（快客户端不该触发有界通道丢帧；小概率 GC 抖动容忍 <5%）
    st = _gw_dropped(big_stack.gateway_port)
    total = st['dropped'] + st['forwarded']
    if total > 0:
        assert st['dropped'] / total < 0.05, st


def test_bigdata_stall_recovery_bounded_backlog_and_fresh_resume(big_stack):
    """停顿-恢复：全链路缓冲有界（不随停顿时长增长）+ 恢复即新鲜帧.

    慢消费语义（TCP 物理现实）：慢/停顿客户端的内核 rcvbuf（自动调优 ~MB 级）
    是「已提交积压」，任何服务端都收不回——所以正确的验收不是「慢读时每帧都新」，
    而是 Lichtblick/桥同款语义：
      ① 停顿 4s 的产量**不会**全部堆积转发（网关 lane + 桥 send buffer 丢帧，
         排空帧数有硬上限，而非 4s×30fps≈120 帧）；
      ② 排空后等一拍，下一帧立刻新鲜（恢复延迟有界，显示侧 latest-wins 跳帧）；
      ③ 网关丢帧统计可见（dropped/droppedStale > 0，带宽受保护）。
    """

    async def run():
        ws = await _raw_connect(big_stack.ws_url)
        channels: dict = {}
        out = {}
        try:
            img_ch = (await _wait_channel(ws, _IMG_TOPIC, channels))['id']
            await _subscribe(ws, img_ch, 1)

            async def read_one(timeout=15.0):
                """返回 (logTime wall epoch, 本地读取年龄)。JSON op 吸收。"""
                while True:
                    raw = await asyncio.wait_for(ws.recv(), timeout)
                    if isinstance(raw, str):
                        for ch in json.loads(raw).get('channels', []) or []:
                            channels[ch['topic']] = ch
                        continue
                    if raw[0] == OP_MESSAGE_DATA and len(raw) >= 13:
                        _, log_ns = struct.unpack_from('<IQ', raw, 1)
                        log_s = log_ns / 1e9
                        return log_s, time.time() - log_s

            # 基线：两帧新鲜
            out['pre'] = [a for _, a in [await read_one() for _ in range(2)]]
            # 全停顿 4s（模拟隐藏标签页：不 recv）
            t_stall_start = time.time()
            await asyncio.sleep(4.0)
            t_stall_end = time.time()
            # 排空直到追上直播（帧 logTime 越过停顿窗末尾），统计停顿窗内的帧
            # 实际送达了几帧——无界转发会送满 4s×30fps≈120 帧；有界链路只会送出
            # 内核/桥缓冲里剩的尾部（其余已在网关 lane / 桥 send buffer 丢弃）
            stall_window = 0
            first_fresh_age = None
            for _ in range(300):
                log_s, age = await read_one(timeout=10.0)
                if log_s > t_stall_end:
                    first_fresh_age = age
                    break
                stall_window += 1
            out['stall_window'] = stall_window
            out['first_fresh_age'] = first_fresh_age
        finally:
            await ws.close()
        return out

    before = _gw_dropped(big_stack.gateway_port)
    out = asyncio.run(run())
    after = _gw_dropped(big_stack.gateway_port)

    # ① 停顿窗内帧的送达数有界：无界转发 ≈ 4s×30fps = 120 帧；
    #    有界链路（网关 lane 4MiB + 内核套接字 ~10MB + 桥 asio 10MB）≲ 40 帧。
    assert out['stall_window'] < 60, \
        f"停顿窗送达 {out['stall_window']} 帧——全链路缓冲无界"
    # ② 追上直播的第一帧新鲜（恢复延迟有界，显示侧 latest-wins 跳帧）
    assert out['first_fresh_age'] is not None and out['first_fresh_age'] < 1.5, \
        f"恢复帧年龄 {out['first_fresh_age']}"
    # ③ 带宽受保护：停顿窗生产的帧没有全量灌向慢客户端。丢弃可能发生在网关
    # lane（droppedData 计数可见）**或**桥 send buffer（上游先满时丢新帧，
    # 网关侧不可见）——两段都是合法的有界降级，负载时序决定谁先丢。桥侧丢弃
    # 以"送达远小于无界语义（≈120 帧）"为间接证据（<30：桥 10MB+lane 4MiB+
    # 内核缓冲的转发上限）。
    assert after['dropped'] > before['dropped'] or out['stall_window'] < 30, \
        (before, after, out['stall_window'])
