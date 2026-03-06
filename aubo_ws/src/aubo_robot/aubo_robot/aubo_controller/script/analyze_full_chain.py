#!/usr/bin/env python3
"""
MoveIt2 → 机械臂 全链路分析

数据流: MoveIt2 → joint_path_command → aubo_robot_simulator → moveItController_cmd → aubo_driver → 机械臂
数据文件: /home/mu/IVG/.cursor/full_chain_motion.ndjson

事件:
- trajectory_received: 收到轨迹，含 point_times（MoveIt 时间参数化）
- segment_start: 开始处理一段（上一点到当前点）
- segment_done: 段处理完，含 num_steps（插值步数）
- throttle: RIB 缓冲满导致等待

用法: python3 analyze_full_chain.py [数据文件路径]
"""

import json
import sys
from collections import defaultdict

DEFAULT_PATH = "/home/mu/IVG/.cursor/full_chain_motion.ndjson"


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


def analyze(events):
    traj_ev = [e for e in events if e.get("event") == "trajectory_received"]
    seg_start = [e for e in events if e.get("event") == "segment_start"]
    seg_done = [e for e in events if e.get("event") == "segment_done"]
    throttles = [e for e in events if e.get("event") == "throttle"]

    print("=" * 70)
    print("MoveIt2 → 机械臂 全链路分析")
    print("=" * 70)

    # 1. 轨迹接收（MoveIt2 时间参数化）
    print("\n【1. 轨迹接收】joint_path_command ← MoveIt2/ROS2")
    if traj_ev:
        for i, e in enumerate(traj_ev):
            n = e.get("num_points", 0)
            dur = e.get("duration_sec", 0)
            pt = e.get("point_times", [])
            print("  轨迹 %d: %d 点, 时长 %.2fs" % (i + 1, n, dur))
            if len(pt) >= 2:
                intervals = [pt[j] - pt[j-1] for j in range(1, len(pt))]
                avg_iv = sum(intervals) / len(intervals)
                print("    点间隔: %.4f ~ %.4f s (均 %.4f)" % (min(intervals), max(intervals), avg_iv))
            elif len(pt) == 1:
                print("    单点轨迹")
    else:
        print("  无 trajectory_received")

    # 2. Segment 处理（仿真器插值）
    print("\n【2. Segment 处理】aubo_robot_simulator 5 次多项式插值")
    if seg_start and seg_done:
        if len(seg_start) != len(seg_done):
            print("  警告: segment_start(%d) != segment_done(%d)" % (len(seg_start), len(seg_done)))
        T_vals = [e.get("T", 0) for e in seg_done]
        steps = [e.get("num_steps", 0) for e in seg_done]
        print("  总 segment 数: %d" % len(seg_done))
        if T_vals:
            print("  segment T: %.4f ~ %.4f s" % (min(T_vals), max(T_vals)))
        if steps:
            print("  每段插值步数: %d ~ %d (200Hz 下 T*200≈步数)" % (min(steps), max(steps)))
            expected = [int(round(T * 200)) for T in T_vals]
            mismatch = sum(1 for a, b in zip(steps, expected) if abs(a - b) > 2)
            if mismatch:
                print("  步数与 T*200 不一致的段数: %d" % mismatch)
    else:
        print("  无 segment 记录")

    # 3. Throttle（RIB 缓冲）
    print("\n【3. Throttle】RIB 缓冲满导致仿真器等待")
    if throttles:
        total_block = sum(e.get("block_count", 0) for e in throttles)
        total_sleep = sum(e.get("sleep_ms", 0) for e in throttles)
        print("  阻塞次数: %d, 累计等待 %.1f ms" % (total_block, total_sleep))
    else:
        print("  无 throttle")

    # 4. 按轨迹划分 segment（用 trajectory_received 时间戳为界）
    print("\n【4. 关节空间 vs 笛卡尔】前 2 条=关节，后 N 条=笛卡尔")
    ordered = [e for e in events if e.get("event") in ("trajectory_received", "segment_start", "segment_done")]
    traj_boundaries = [i for i, e in enumerate(ordered) if e.get("event") == "trajectory_received"]
    segs_per_traj = []
    for k in range(len(traj_boundaries)):
        start = traj_boundaries[k]
        end = traj_boundaries[k + 1] if k + 1 < len(traj_boundaries) else len(ordered)
        segs = [e for e in ordered[start:end] if e.get("event") == "segment_done"]
        segs_per_traj.append(segs)
    if len(segs_per_traj) >= 3:
        j_segs = (segs_per_traj[0] + segs_per_traj[1]) if len(segs_per_traj) >= 2 else []
        c_segs = sum(segs_per_traj[2:], [])
        if j_segs and c_segs:
            j_T = [e.get("T", 0) for e in j_segs]
            c_T = [e.get("T", 0) for e in c_segs]
            j_steps = [e.get("num_steps", 0) for e in j_segs]
            c_steps = [e.get("num_steps", 0) for e in c_segs]
            print("  关节空间: %d 段, T=%.4f~%.4f s, 步数 %d~%d" % (len(j_segs), min(j_T), max(j_T), min(j_steps), max(j_steps)))
            print("  笛卡尔:   %d 段, T=%.4f~%.4f s, 步数 %d~%d" % (len(c_segs), min(c_T), max(c_T), min(c_steps), max(c_steps)))
            j_bps = 1.0 / (sum(j_T) / len(j_T)) if j_T else 0
            c_bps = 1.0 / (sum(c_T) / len(c_T)) if c_T else 0
            print("  边界频率: 关节 %.1f/s, 笛卡尔 %.1f/s" % (j_bps, c_bps))
    else:
        print("  轨迹数不足 4，无法对比")

    # 5. 成因归纳
    print("\n【5. 全链路成因归纳】")
    if traj_ev and len(segs_per_traj) >= 3:
        j_segs = (segs_per_traj[0] + segs_per_traj[1]) if len(segs_per_traj) >= 2 else []
        c_segs = sum(segs_per_traj[2:], [])
        if c_segs and j_segs:
            c_avg_T = sum(e.get("T", 0) for e in c_segs) / len(c_segs)
            j_avg_T = sum(e.get("T", 0) for e in j_segs) / len(j_segs)
            if c_avg_T < j_avg_T:
                print("  笛卡尔 segment 更短 (%.4f vs %.4f) → 边界更密 → 易异响" % (c_avg_T, j_avg_T))
            print("  链路: MoveIt2 时间参数化 → joint_path_command → 仿真器插值(200Hz) → moveItController_cmd → driver → 机械臂")
            print("  若异响仅在笛卡尔出现，可排查: MoveIt 笛卡尔采样、driver 滤波/插补、机械共振")
    print("=" * 70)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    events = load_events(path)
    if events:
        analyze(events)
    else:
        print("无数据。请先执行关节空间 + 笛卡尔路径规划以采集全链路数据。")


if __name__ == "__main__":
    main()
