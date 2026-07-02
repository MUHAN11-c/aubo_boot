#!/usr/bin/env python3
"""
Parse move_group.log + rosbag2: compare command-side joint acceleration at segment starts.

Findings on ivg_session (see module doc in repo docs or run output):
- Joint->Cartesian handoffs tend to show higher median initial |acc| in /moveItController_cmd
  than Cartesian->Cartesian handoffs.
- Cartesian->Joint handoffs show lower initial |acc| in this session.

Requires: ROS 2 sourced (rclpy, message types), numpy, rosbag2_py.

Example:
  source /opt/ros/humble/setup.bash
  python3 analyze_joint_cartesian_junction_accel.py \\
    --bag /path/to/ivg_session \\
    --log /path/to/move_group_*.log \\
    --bag-t0-ns 1774600542445297133
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from trajectory_msgs.msg import JointTrajectoryPoint


def parse_ros_log_time(line: str) -> int | None:
    m = re.search(r"\[(\d+)\.(\d+)\]", line)
    if not m:
        return None
    sec, frac = m.group(1), m.group(2)
    frac = frac.ljust(9, "0")[:9]
    return int(sec) * 10**9 + int(frac)


def parse_segments_from_move_group_log(log_path: Path) -> list[tuple[int, int, str]]:
    events: list[tuple[int, str]] = []
    for line in log_path.read_text(errors="ignore").splitlines():
        t = parse_ros_log_time(line)
        if t is None:
            continue
        if "Planning request received for MoveGroup action" in line:
            events.append((t, "plan_joint"))
        elif "Computed Cartesian path with" in line:
            events.append((t, "plan_cart"))
        elif "Starting trajectory execution" in line:
            events.append((t, "exec_start"))
        elif "Completed trajectory execution with status SUCCEEDED" in line:
            events.append((t, "exec_done"))

    events.sort(key=lambda x: x[0])
    last_plan: str | None = None
    segments: list[tuple[int, int, str]] = []
    open_start: int | None = None
    open_type: str | None = None
    for t, ev in events:
        if ev == "plan_joint":
            last_plan = "joint"
        elif ev == "plan_cart":
            last_plan = "cartesian"
        elif ev == "exec_start":
            open_start = t
            open_type = last_plan or "unknown"
        elif ev == "exec_done" and open_start is not None:
            segments.append((open_start, t, open_type))
            open_start = None
            open_type = None
    return segments


def load_cmd_stream(bag_uri: str) -> tuple[np.ndarray, np.ndarray]:
    opts = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_uri, storage_id="sqlite3"), opts)
    cmd_t: list[int] = []
    cmd_q: list[np.ndarray] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != "/moveItController_cmd":
            continue
        msg = deserialize_message(data, JointTrajectoryPoint)
        cmd_t.append(t)
        cmd_q.append(np.array(msg.positions, dtype=float))
    return np.array(cmd_t, dtype=np.int64), np.vstack(cmd_q)


def mean_acc_start(
    cmd_t: np.ndarray,
    vel: np.ndarray,
    dt: np.ndarray,
    t_s: int,
    t_e: int,
    k: int = 25,
    max_dt: float = 0.03,
) -> float:
    i0 = np.searchsorted(cmd_t, t_s, side="left")
    i1 = np.searchsorted(cmd_t, t_e, side="right") - 1
    if i1 <= i0 + 2:
        return float("nan")
    acc_list: list[float] = []
    for j in range(i0, min(i0 + k, i1 - 2)):
        if dt[j] > max_dt or dt[j + 1] > max_dt:
            continue
        dtm = (dt[j] + dt[j + 1]) / 2.0
        a = (vel[j + 1] - vel[j]) / dtm
        acc_list.append(float(np.max(np.abs(a))))
    return float(np.median(acc_list)) if acc_list else float("nan")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag", required=True, help="rosbag2 directory (with metadata)")
    p.add_argument("--log", required=True, help="move_group log file path")
    p.add_argument("--bag-t0-ns", type=int, required=True, help="Bag start time (metadata starting_time)")
    p.add_argument("--duration-sec", type=float, default=0.0, help="0 = use 99999 (no clip)")
    args = p.parse_args()

    bag_t0 = int(args.bag_t0_ns)
    dur = float(args.duration_sec) if args.duration_sec > 0 else 1.0e6

    def bag_rel(t_ns: int) -> float:
        return (t_ns - bag_t0) / 1e9

    segments = parse_segments_from_move_group_log(Path(args.log))
    segments = [s for s in segments if bag_rel(s[0]) <= dur and bag_rel(s[1]) >= 0]

    cmd_t, cmd_q = load_cmd_stream(args.bag)
    dt = np.diff(cmd_t) / 1e9
    dq = np.diff(cmd_q, axis=0)
    vel = dq / dt[:, None]

    j_then_c: list[float] = []
    c_then_c: list[float] = []
    c_then_j: list[float] = []

    for i, (t_s, t_e, typ) in enumerate(segments):
        if i == 0:
            continue
        prev = segments[i - 1][2]
        acc0 = mean_acc_start(cmd_t, vel, dt, t_s, t_e)
        if prev == "joint" and typ == "cartesian":
            j_then_c.append(acc0)
        elif prev == "cartesian" and typ == "cartesian":
            c_then_c.append(acc0)
        elif prev == "cartesian" and typ == "joint":
            c_then_j.append(acc0)

    def med(x: list[float]) -> float:
        return float(np.median(x)) if x else float("nan")

    print("Segments overlapping bag (from log):", len(segments))
    print("NEW segment: median max|joint acc| in first ~25 dense cmd steps")
    print(f"  joint  -> cartesian: n={len(j_then_c):3d}  median={med(j_then_c):.3f} rad/s^2")
    print(f"  cartesian -> cartesian: n={len(c_then_c):3d}  median={med(c_then_c):.3f} rad/s^2")
    print(f"  cartesian -> joint:     n={len(c_then_j):3d}  median={med(c_then_j):.3f} rad/s^2")
    if j_then_c and c_then_c:
        print(f"Ratio (j->c)/(c->c) median accel: {med(j_then_c) / med(c_then_c):.2f}x")


if __name__ == "__main__":
    main()
