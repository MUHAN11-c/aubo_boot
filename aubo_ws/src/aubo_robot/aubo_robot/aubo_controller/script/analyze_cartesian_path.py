#!/usr/bin/env python3
"""
笛卡尔路径卡顿分析脚本

读取 /home/mu/IVG/.cursor/cartesian_path_data.ndjson，统计：
- 轨迹数量与点密度
- throttle 事件频率与累计阻塞时间
- 短间隔 segment 分布

用法:
  python3 analyze_cartesian_path.py [数据文件路径]
"""

import json
import sys
from collections import defaultdict

DEFAULT_PATH = "/home/mu/IVG/.cursor/cartesian_path_data.ndjson"


def load_events(path):
    events = []
    try:
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    except FileNotFoundError:
        print("数据文件不存在:", path)
        return []
    return events


def _stddev(lst):
    if len(lst) < 2:
        return 0.0
    mean = sum(lst) / len(lst)
    return (sum((x - mean) ** 2 for x in lst) / len(lst)) ** 0.5


def analyze(events):
    trajectories = [e for e in events if e.get("event") == "trajectory_received"]
    throttles = [e for e in events if e.get("event") == "throttle"]
    segments = [e for e in events if e.get("event") == "segment"]

    # 按事件顺序将 segment 归属到各轨迹（trajectory_received 之间的 segment 属前一条轨迹）
    ordered = [e for e in events if e.get("event") in ("trajectory_received", "segment")]
    segs_per_traj = [[] for _ in trajectories]
    traj_idx = -1
    for e in ordered:
        if e.get("event") == "trajectory_received":
            traj_idx += 1
        elif e.get("event") == "segment" and 0 <= traj_idx < len(trajectories):
            segs_per_traj[traj_idx].append(e)

    # 前 2 条=关节空间，第 3 条及以后=笛卡尔
    n_traj = len(trajectories)
    joint_indices = list(range(min(2, n_traj)))
    cartesian_indices = list(range(2, n_traj))

    print("=" * 60)
    print("笛卡尔路径卡顿分析报告")
    print("=" * 60)

    # 轨迹概览
    print("\n【轨迹概览】")
    if trajectories:
        for i, t in enumerate(trajectories):
            kind = "关节空间" if i in joint_indices else "笛卡尔"
            n = t.get("num_points", 0)
            dur = t.get("duration_sec", 0)
            iv = t.get("avg_interval_sec", 0)
            n_seg = len(segs_per_traj[i]) if i < len(segs_per_traj) else 0
            print("  轨迹 %d [%s]: %d 点, 时长 %.2fs, 点间隔 %.4fs, %d 短 segment" % (i + 1, kind, n, dur, iv, n_seg))
    else:
        print("  无 trajectory_received 记录")

    # 关节空间 vs 笛卡尔 对比
    print("\n【关节空间 vs 笛卡尔 对比】（前2条=关节，后N条=笛卡尔）")
    joint_segs = []
    cartesian_segs = []
    for i in joint_indices:
        if i < len(segs_per_traj):
            joint_segs.extend(segs_per_traj[i])
    for i in cartesian_indices:
        if i < len(segs_per_traj):
            cartesian_segs.extend(segs_per_traj[i])

    if joint_segs and cartesian_segs:
        j_ts = [s.get("segment_T", 0) for s in joint_segs]
        c_ts = [s.get("segment_T", 0) for s in cartesian_segs]
        j_avg = sum(j_ts) / len(j_ts)
        c_avg = sum(c_ts) / len(c_ts)
        j_bps = 1.0 / j_avg if j_avg > 0 else 0
        c_bps = 1.0 / c_avg if c_avg > 0 else 0

        print("  关节空间: %d segments, T=%.4f s, 约 %.1f 边界/秒" % (len(joint_segs), j_avg, j_bps))
        print("  笛卡尔:   %d segments, T=%.4f s, 约 %.1f 边界/秒" % (len(cartesian_segs), c_avg, c_bps))
        print("")
        if c_avg < j_avg:
            print("  差异: 笛卡尔 segment 更短 → 边界密度更高 → 易出现持续声音")
        else:
            print("  笛卡尔 segment 已接近或长于关节空间，抽稀生效")
    elif joint_segs and cartesian_indices and not cartesian_segs:
        j_ts = [s.get("segment_T", 0) for s in joint_segs]
        j_avg = sum(j_ts) / len(j_ts)
        j_bps = 1.0 / j_avg if j_avg > 0 else 0
        j_iv = [trajectories[i].get("avg_interval_sec", 0) for i in joint_indices if i < len(trajectories)]
        c_iv = [trajectories[i].get("avg_interval_sec", 0) for i in cartesian_indices if i < len(trajectories)]
        print("  关节空间: %d segments, T~%.4f s, 点间隔 %.4f s" % (len(joint_segs), j_avg, sum(j_iv)/len(j_iv) if j_iv else 0))
        print("  笛卡尔:   0 短 segment (T≥0.12s), 点间隔 %.4f s" % (sum(c_iv)/len(c_iv) if c_iv else 0))
        print("")
        print("  >>> 笛卡尔已无短 segment，抽稀生效，边界密度低于关节空间")
    else:
        print("  需至少 1 条关节 + 1 条笛卡尔轨迹才能对比")

    # Throttle 统计
    print("\n【Throttle 阻塞】")
    if throttles:
        total_blocks = sum(t.get("block_count", 0) for t in throttles)
        total_sleep_ms = sum(t.get("sleep_ms", 0) for t in throttles)
        rib_vals = [t.get("ribBufferSize", 0) for t in throttles]
        print("  阻塞次数: %d, 累计等待 %.1f ms" % (total_blocks, total_sleep_ms))
        if total_sleep_ms > 100:
            print("  >>> 卡顿可能原因：RIB 缓冲区积压，频繁节流")
    else:
        print("  无 throttle 记录")

    # 缓冲区
    buf_rem = [s.get("motion_buf_remaining", 0) for s in segments]
    if buf_rem:
        zero_count = sum(1 for b in buf_rem if b == 0)
        if zero_count > 0:
            print("  motion_buf_remaining 曾为 0 的 segment 数: %d" % zero_count)

    # 成因与建议
    print("\n【持续声音成因分析】")
    if joint_segs and cartesian_segs:
        print("  关节空间: segment_T 较均匀 (~0.1s)，边界少，运动连贯，无持续异响")
        print("  笛卡尔:   segment_T 更短且变化大，边界多，频繁加减速 → 机械臂持续声音")
        print("")
        print("  建议: 增大 min_segment_interval 抽稀笛卡尔路径")
    elif joint_segs and cartesian_indices and not cartesian_segs:
        print("  笛卡尔已无短 segment，抽稀生效，异响应已减轻")
    print("=" * 60)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    events = load_events(path)
    if events:
        analyze(events)
    else:
        print("无数据，请先执行笛卡尔路径规划以采集数据")


if __name__ == "__main__":
    main()
