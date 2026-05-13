#!/usr/bin/env python3
"""
离线分析 joint_states_motion_analyzer 生成的 CSV。

输入：
  1) 采样数据 CSV（默认）：
     /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/scripts/joint_states_motion_latest.csv
  2) 异常事件 CSV（默认）：
     /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/scripts/joint_states_motion_latest_events.csv

输出：
  文本报告（默认）：
     /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/scripts/joint_states_motion_analysis_latest.txt

示例：
  ros2 run demo_driver analyze_joint_states_motion_csv.py

  ros2 run demo_driver analyze_joint_states_motion_csv.py -- \
    --samples /tmp/joint_states_motion_latest.csv \
    --events /tmp/joint_states_motion_latest_events.csv \
    --output /tmp/joint_states_motion_analysis.txt
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass
from statistics import mean
from typing import Dict, List, Tuple


DEFAULT_DIR = os.path.dirname(os.path.realpath(__file__))
DEFAULT_SAMPLES = os.path.join(DEFAULT_DIR, "joint_states_motion_latest.csv")
DEFAULT_EVENTS = os.path.join(DEFAULT_DIR, "joint_states_motion_latest_events.csv")
DEFAULT_REPORT = os.path.join(DEFAULT_DIR, "joint_states_motion_analysis_latest.txt")


@dataclass
class SampleRow:
    t: float
    max_speed: float
    max_acc: float
    max_jerk: float
    is_moving: int


@dataclass
class EventRow:
    t: float
    event_type: str
    joint: str
    value: float
    threshold: float
    note: str


def percentile(values: List[float], p: float) -> float:
    if not values:
        return 0.0
    if p <= 0:
        return min(values)
    if p >= 100:
        return max(values)
    s = sorted(values)
    k = (len(s) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return s[int(k)]
    return s[f] + (s[c] - s[f]) * (k - f)


def read_samples(path: str) -> Tuple[List[SampleRow], List[str]]:
    rows: List[SampleRow] = []
    joint_names: List[str] = []
    if not os.path.exists(path):
        raise FileNotFoundError(f"采样文件不存在: {path}")

    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames or []
        joint_names = [h[:-4] for h in header if h.endswith("_pos")]
        for r in reader:
            rows.append(
                SampleRow(
                    t=float(r["t"]),
                    max_speed=float(r.get("max_speed", 0.0)),
                    max_acc=float(r.get("max_acc", 0.0)),
                    max_jerk=float(r.get("max_jerk", 0.0)),
                    is_moving=int(float(r.get("is_moving", 0))),
                )
            )
    return rows, joint_names


def read_events(path: str) -> List[EventRow]:
    rows: List[EventRow] = []
    if not os.path.exists(path):
        return rows
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(
                EventRow(
                    t=float(r["t"]),
                    event_type=r.get("event_type", ""),
                    joint=r.get("joint", "-"),
                    value=float(r.get("value", 0.0)),
                    threshold=float(r.get("threshold", 0.0)),
                    note=r.get("note", ""),
                )
            )
    return rows


def topk(rows: List[SampleRow], key: str, k: int = 5) -> List[SampleRow]:
    return sorted(rows, key=lambda x: getattr(x, key), reverse=True)[:k]


def summarize_events(events: List[EventRow]) -> Tuple[Dict[str, int], Dict[str, int]]:
    by_type: Dict[str, int] = {}
    by_joint: Dict[str, int] = {}
    for e in events:
        by_type[e.event_type] = by_type.get(e.event_type, 0) + 1
        by_joint[e.joint] = by_joint.get(e.joint, 0) + 1
    return by_type, by_joint


def build_report(
    sample_path: str,
    event_path: str,
    rows: List[SampleRow],
    joint_names: List[str],
    events: List[EventRow],
    stutter_speed_threshold: float,
) -> str:
    if not rows:
        return "无采样数据。"

    t0 = rows[0].t
    t1 = rows[-1].t
    duration = max(0.0, t1 - t0)

    speed = [r.max_speed for r in rows]
    acc = [r.max_acc for r in rows]
    jerk = [r.max_jerk for r in rows]
    moving_ratio = sum(r.is_moving for r in rows) / max(1, len(rows))
    low_speed_while_moving = [r for r in rows if r.is_moving == 1 and r.max_speed <= stutter_speed_threshold]
    stutter_ratio = len(low_speed_while_moving) / max(1, sum(r.is_moving for r in rows))

    lines: List[str] = []
    lines.append("=== Joint States 运动离线分析报告 ===")
    lines.append(f"采样文件: {sample_path}")
    lines.append(f"事件文件: {event_path} {'(存在)' if os.path.exists(event_path) else '(不存在)'}")
    lines.append("")
    lines.append("[基础信息]")
    lines.append(f"- 样本数: {len(rows)}")
    lines.append(f"- 关节数量: {len(joint_names)}")
    lines.append(f"- 时间范围: {t0:.6f} ~ {t1:.6f} (总时长 {duration:.3f}s)")
    lines.append(f"- moving 占比: {moving_ratio * 100:.2f}%")
    lines.append(f"- moving阶段低速占比(<= {stutter_speed_threshold:.4f}): {stutter_ratio * 100:.2f}%")
    lines.append("")

    lines.append("[总体统计]")
    lines.append(
        "- max_speed  : mean={:.4f}, p95={:.4f}, p99={:.4f}, max={:.4f}".format(
            mean(speed), percentile(speed, 95), percentile(speed, 99), max(speed)
        )
    )
    lines.append(
        "- max_acc    : mean={:.4f}, p95={:.4f}, p99={:.4f}, max={:.4f}".format(
            mean(acc), percentile(acc, 95), percentile(acc, 99), max(acc)
        )
    )
    lines.append(
        "- max_jerk   : mean={:.4f}, p95={:.4f}, p99={:.4f}, max={:.4f}".format(
            mean(jerk), percentile(jerk, 95), percentile(jerk, 99), max(jerk)
        )
    )
    lines.append("")

    lines.append("[峰值时刻 Top5]")
    for i, r in enumerate(topk(rows, "max_acc", 5), 1):
        lines.append(
            f"- acc#{i}: t={r.t:.6f}, max_speed={r.max_speed:.4f}, max_acc={r.max_acc:.4f}, "
            f"max_jerk={r.max_jerk:.4f}, moving={r.is_moving}"
        )
    for i, r in enumerate(topk(rows, "max_jerk", 5), 1):
        lines.append(
            f"- jerk#{i}: t={r.t:.6f}, max_speed={r.max_speed:.4f}, max_acc={r.max_acc:.4f}, "
            f"max_jerk={r.max_jerk:.4f}, moving={r.is_moving}"
        )
    lines.append("")

    lines.append("[异常事件统计]")
    if events:
        by_type, by_joint = summarize_events(events)
        lines.append(f"- 总事件数: {len(events)}")
        lines.append("- 按类型: " + ", ".join(f"{k}={v}" for k, v in sorted(by_type.items())))
        lines.append("- 按关节: " + ", ".join(f"{k}={v}" for k, v in sorted(by_joint.items())))
        lines.append("- 最近5条:")
        for e in events[-5:]:
            lines.append(
                f"  - t={e.t:.6f}, type={e.event_type}, joint={e.joint}, "
                f"value={e.value:.4f}, th={e.threshold:.4f}, note={e.note}"
            )
    else:
        lines.append("- 未检测到事件文件或事件为空。")
    lines.append("")

    lines.append("[诊断建议]")
    lines.append("- 若 max_jerk p99 明显偏高，优先检查轨迹时间参数化和控制周期一致性。")
    lines.append("- 若 sample_gap 频繁，优先检查通信链路、CPU 占用与实时线程调度。")
    lines.append("- 若 stutter 比例高，检查速度/加速度限幅是否过紧、控制器切换和阻塞调用。")
    lines.append("- 对比峰值时刻与上位机下发指令时间，定位是规划端还是执行端引起。")

    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="分析 joint_states_motion_analyzer 导出的 CSV")
    p.add_argument("--samples", default=DEFAULT_SAMPLES, help="采样 CSV 路径")
    p.add_argument("--events", default=DEFAULT_EVENTS, help="异常事件 CSV 路径")
    p.add_argument("--output", default=DEFAULT_REPORT, help="文本报告输出路径")
    p.add_argument(
        "--stutter-speed-threshold",
        type=float,
        default=0.03,
        help="统计 moving 阶段低速占比阈值（rad/s）",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()

    samples, joints = read_samples(args.samples)
    events = read_events(args.events)

    report = build_report(
        sample_path=args.samples,
        event_path=args.events,
        rows=samples,
        joint_names=joints,
        events=events,
        stutter_speed_threshold=args.stutter_speed_threshold,
    )

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(report)

    print(report)
    print(f"报告已保存: {args.output}")


if __name__ == "__main__":
    main()
