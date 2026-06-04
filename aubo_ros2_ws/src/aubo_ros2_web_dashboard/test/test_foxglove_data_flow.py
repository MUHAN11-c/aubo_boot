#!/usr/bin/env python3
"""验证 foxglove_bridge 是 Web 前端的数据主力 (CDR 二进制传输)

检测:
  1. foxglove CDR channels 总量 vs JSON channels
  2. 所有持续数据话题是否在 foxglove CDR 通道中
  3. CDR 消息流量统计
  4. rosbridge JSON 对比 (带宽对比)

用法:
  python3 test_foxglove_data_flow.py
"""

import asyncio
import json
import struct
import time
from collections import defaultdict

import websockets

FOXGLOVE_URL = "ws://127.0.0.1:8765"
SUBPROTOCOL = "foxglove.sdk.v1"
DURATION = 5.0  # 采集时长

GREEN = "\033[0;32m"; RED = "\033[0;31m"; YELLOW = "\033[1;33m"; BLUE = "\033[0;34m"; NC = "\033[0m"


# ── 所有前端需要的话题 (持续数据型) ──────────────────────────────────────────
EXPECTED_TOPICS = [
    "/joint_states",
    "/dynamic_joint_states",
    "/robot_status",
    "/tf",
    "/tf_static",
    "/tool_changer_status",
    "/aubo/mode",
    "/rosout",
    "/controller_state",
    "/state",
    "/robot_description",
    "/grasp_place_status",
]


async def main():
    print(f"\n{BLUE}═══ Foxglove 数据流验证 ═══{NC}")
    print(f"{BLUE}目标: {FOXGLOVE_URL}{NC}\n")

    # 连接
    ws = await websockets.connect(FOXGLOVE_URL, subprotocols=[SUBPROTOCOL])
    print(f"  {GREEN}✓{NC} 已连接 ({ws.subprotocol})")

    # 收集 metadata
    channels = []
    server_info = {}
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
        except asyncio.TimeoutError:
            continue
        if isinstance(raw, str):
            obj = json.loads(raw)
            if obj.get("op") == "serverInfo":
                server_info = obj
            elif obj.get("op") == "advertise" and "channels" in obj:
                channels.extend(obj["channels"])

    # ── 分析 ───────────────────────────────────────────────────────────────
    cdr_channels = [ch for ch in channels if ch.get("encoding") == "cdr"]
    json_channels = [ch for ch in channels if ch.get("encoding") == "json"]
    cdr_topics = {ch["topic"] for ch in cdr_channels}

    print(f"\n{BLUE}── Channel 编码分布 ──{NC}")
    print(f"  CDR (二进制):  {len(cdr_channels)} channels")
    print(f"  JSON (文本):   {len(json_channels)} channels  {YELLOW}(仅 /foxglove_bridge/sysinfo){NC}")

    # ── 预期话题检查 ───────────────────────────────────────────────────────
    print(f"\n{BLUE}── 前端话题覆盖检查 ──{NC}")
    all_ok = True
    for topic in EXPECTED_TOPICS:
        if topic in cdr_topics:
            ch = next(c for c in cdr_channels if c["topic"] == topic)
            print(f"  {GREEN}✓{NC} {topic}  →  CDR ({ch.get('schemaName','?')})")
        else:
            print(f"  {RED}✗{NC} {topic}  →  未在 foxglove CDR 中发现!")
            all_ok = False

    # ── CDR 数据流测量 ─────────────────────────────────────────────────────
    print(f"\n{BLUE}── CDR 数据流 ({DURATION}s 采集) ──{NC}")

    # 构建 topic→channelId 映射
    topic_to_ch = {ch["topic"]: ch for ch in cdr_channels}

    # 订阅所有预期话题
    sub_list = []
    sub_id_map = {}
    sub_counter = 1000
    for topic in EXPECTED_TOPICS:
        ch = topic_to_ch.get(topic)
        if ch:
            sub_counter += 1
            sub_list.append({"id": sub_counter, "channelId": ch["id"]})
            sub_id_map[sub_counter] = topic

    await ws.send(json.dumps({"op": "subscribe", "subscriptions": sub_list}))

    # 采集统计数据
    stats = defaultdict(lambda: {"count": 0, "bytes": 0, "first_ts": None, "last_ts": None})
    t0 = time.monotonic()
    deadline = t0 + DURATION

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=min(remaining, 1.0))
        except asyncio.TimeoutError:
            continue

        if not isinstance(raw, bytes) or len(raw) < 14:
            continue

        opcode = raw[0]
        if opcode != 0x01:  # MESSAGE_DATA
            continue

        sub_id = struct.unpack_from("<I", raw, 1)[0]
        ts_sec = struct.unpack_from("<I", raw, 5)[0]
        ts_nsec = struct.unpack_from("<I", raw, 9)[0]
        ts_ms = ts_sec * 1000 + ts_nsec // 1_000_000
        payload_size = len(raw) - 13

        topic = sub_id_map.get(sub_id)
        if not topic:
            continue

        s = stats[topic]
        s["count"] += 1
        s["bytes"] += payload_size
        if s["first_ts"] is None:
            s["first_ts"] = ts_ms
        s["last_ts"] = ts_ms

    await ws.close()

    # ── 输出统计 ───────────────────────────────────────────────────────────
    total_bytes = 0
    total_msgs = 0
    print(f"{'Topic':<30} {'消息数':>6} {'数据量':>10} {'频率':>8} {'状态'}")
    print("-" * 70)

    for topic in EXPECTED_TOPICS:
        s = stats.get(topic)
        if not s or s["count"] == 0:
            print(f"{topic:<30} {'—':>6} {'—':>10} {'—':>8}  {YELLOW}无数据{NC}")
            continue

        # 计算频率
        if s["first_ts"] and s["last_ts"] and s["first_ts"] != s["last_ts"]:
            duration_s = (s["last_ts"] - s["first_ts"]) / 1000.0
            freq = s["count"] / duration_s if duration_s > 0 else 0
        else:
            freq = 0

        total_bytes += s["bytes"]
        total_msgs += s["count"]

        # 格式化
        if s["bytes"] > 1024 * 1024:
            size_str = f"{s['bytes']/1024/1024:.1f} MB"
        elif s["bytes"] > 1024:
            size_str = f"{s['bytes']/1024:.1f} KB"
        else:
            size_str = f"{s['bytes']} B"

        freq_str = f"{freq:.0f} Hz" if freq >= 1 else f"{freq:.1f} Hz"
        status = f"{GREEN}CDR ✓{NC}"
        print(f"{topic:<30} {s['count']:>6} {size_str:>10} {freq_str:>8}  {status}")

    print("-" * 70)
    total_kbps = (total_bytes / DURATION) / 1024
    print(f"{'TOTAL':<30} {total_msgs:>6} {total_bytes/1024:>9.1f} KB {total_kbps:>7.1f} KB/s")

    # ── JSON 带宽对比估算 ──────────────────────────────────────────────────
    print(f"\n{BLUE}── CDR vs JSON 带宽估算 ──{NC}")
    # 基于 VALIDATION_REPORT.md §8.6 的实测比例
    savings_map = {
        "/joint_states": 0.093,       # 9.3%
        "/dynamic_joint_states": 0.201,  # 20.1%
        "/robot_status": 0.636,       # 63.6%
        "/tf": 0.590,                 # 59.0%
        "/tool_changer_status": 0.624,  # 62.4%
    }
    # 未在表中的话题估算 ~40% 节省
    total_cdr = 0
    total_json_est = 0
    for topic, s in stats.items():
        if s["count"] == 0:
            continue
        save = savings_map.get(topic, 0.40)
        total_cdr += s["bytes"]
        total_json_est += s["bytes"] / (1 - save)

    if total_cdr > 0:
        overall_save = (1 - total_cdr / total_json_est) * 100
        print(f"  CDR 总流量:    {total_cdr/1024:.1f} KB")
        print(f"  JSON 估算:     {total_json_est/1024:.1f} KB")
        print(f"  带宽节省:      {GREEN}{overall_save:.1f}%{NC}")

    # ── 总结 ───────────────────────────────────────────────────────────────
    print(f"\n{BLUE}── 结论 ──{NC}")
    cdr_count = sum(1 for t in EXPECTED_TOPICS if stats.get(t, {}).get("count", 0) > 0)
    total = len(EXPECTED_TOPICS)
    print(f"  foxglove CDR 覆盖: {cdr_count}/{total} 话题")
    print(f"  所有前端数据均通过 foxglove_bridge CDR 二进制传输" if all_ok else f"  部分话题未在 CDR 中发现!")


if __name__ == "__main__":
    asyncio.run(main())
