#!/usr/bin/env python3
"""
ROS2 通信测试脚本：检查关键话题的频率、带宽、延迟，用于排查瓶颈。

用法:
  # 系统已启动时运行，默认检测 10 秒
  ros2 run demo_driver test_ros2_communication.py

  # 指定检测时长和话题
  ros2 run demo_driver test_ros2_communication.py --duration 15
  ros2 run demo_driver test_ros2_communication.py --topics /joint_states /grasp_poses_base
  ros2 run demo_driver test_ros2_communication.py --duration 5 --output report.txt

  # 仅检查话题是否存在
  ros2 run demo_driver test_ros2_communication.py --check-only
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from typing import List, Optional, Tuple


# 默认检测的关键话题（aubo + graspnet 系统）
DEFAULT_TOPICS = [
    "/joint_states",
    "joint_path_command",
    "moveItController_cmd",
    "grasp_poses_base",
    "grasp_markers",
    "/camera/depth_registered/points",
    "grasp_place_status",
]


def run_cmd(cmd: List[str], timeout: float = 15.0) -> Tuple[int, str, str]:
    """运行命令，返回 (returncode, stdout, stderr)。"""
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "Timeout"
    except FileNotFoundError:
        return -1, "", "Command not found"


def _normalize_topic(t: str) -> str:
    """统一话题名格式便于比较。"""
    return t.strip().rstrip("/") or "/"


def check_topic_exists(topic: str) -> bool:
    """检查话题是否存在（有发布者或订阅者）。"""
    code, out, _ = run_cmd(["ros2", "topic", "list"], timeout=5.0)
    if code != 0:
        return False
    topics = [_normalize_topic(t) for t in out.splitlines() if t.strip()]
    norm = _normalize_topic(topic)
    return norm in topics or ("/" + norm) in topics or norm.lstrip("/") in [t.lstrip("/") for t in topics]


def get_topic_hz(topic: str, duration: float) -> Optional[float]:
    """运行 ros2 topic hz，返回平均频率 (Hz)，失败返回 None。"""
    # ros2 topic hz 会持续输出，用 timeout 截断
    cmd = ["ros2", "topic", "hz", topic]
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        proc.wait(timeout=duration + 2)
        out, err = proc.communicate(timeout=1)
        # 解析 "average rate: 100.123"
        for line in (out + err).splitlines():
            if "average rate:" in line.lower():
                parts = line.split(":")
                if len(parts) >= 2:
                    try:
                        return float(parts[1].strip().split()[0])
                    except (ValueError, IndexError):
                        pass
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        # 即使超时，hz 可能已输出统计
        pass
    except Exception:
        pass
    return None


def get_topic_hz_with_timeout(topic: str, duration: float) -> Optional[float]:
    """在子进程中运行 hz，duration 秒后终止并解析输出。"""
    cmd = ["timeout", str(int(duration) + 1), "ros2", "topic", "hz", topic]
    code, out, err = run_cmd(cmd, timeout=duration + 5)
    full = out + "\n" + err
    for line in full.splitlines():
        if "average rate:" in line.lower():
            parts = line.split(":")
            if len(parts) >= 2:
                try:
                    return float(parts[1].strip().split()[0])
                except (ValueError, IndexError):
                    pass
    return None


def get_topic_bw(topic: str, duration: float) -> Optional[float]:
    """运行 ros2 topic bw，返回带宽 (MB/s)，失败返回 None。"""
    cmd = ["timeout", str(int(duration) + 1), "ros2", "topic", "bw", topic]
    code, out, err = run_cmd(cmd, timeout=duration + 5)
    full = out + "\n" + err
    # ros2 topic bw 格式: "1.23 MB/s from N messages" 或 "123.45 KB/s" 或 "123 B/s"
    for line in full.splitlines():
        m = re.search(r"([\d.]+)\s*(B|KB|MB)/s", line, re.I)
        if m:
            val, unit = float(m.group(1)), m.group(2).upper()
            if unit == "MB":
                return val
            if unit == "KB":
                return val / 1024.0
            if unit == "B":
                return val / (1024.0 * 1024.0)
    return None


def get_topic_delay(topic: str, duration: float) -> Optional[float]:
    """运行 ros2 topic delay，返回延迟 (s)，失败返回 None。"""
    cmd = ["timeout", str(int(duration) + 1), "ros2", "topic", "delay", topic]
    code, out, err = run_cmd(cmd, timeout=duration + 5)
    full = out + "\n" + err
    for line in full.splitlines():
        if "average delay:" in line.lower() or "delay:" in line.lower():
            parts = line.split(":")
            if len(parts) >= 2:
                s = parts[1].strip().split()[0]
                try:
                    return float(s.replace("s", "").strip())
                except ValueError:
                    pass
    return None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ROS2 通信测试：检查话题频率、带宽、延迟",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="检测时长 (秒)，默认 10",
    )
    parser.add_argument(
        "--topics",
        nargs="+",
        default=DEFAULT_TOPICS,
        help="要检测的话题列表",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="仅检查话题是否存在，不测 hz/bw/delay",
    )
    parser.add_argument(
        "--no-hz",
        action="store_true",
        help="跳过频率检测",
    )
    parser.add_argument(
        "--no-bw",
        action="store_true",
        help="跳过带宽检测",
    )
    parser.add_argument(
        "--no-delay",
        action="store_true",
        help="跳过延迟检测",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default="",
        help="将报告写入文件",
    )
    args = parser.parse_args()

    lines: List[str] = []
    def log(s: str = "") -> None:
        print(s)
        lines.append(s)

    log("=" * 60)
    log("ROS2 通信测试")
    log("=" * 60)
    log(f"检测时长: {args.duration} 秒")
    log(f"话题: {args.topics}")
    log("")

    # 1. 检查话题存在性
    log("--- 话题存在性 ---")
    existing = []
    missing = []
    for topic in args.topics:
        if check_topic_exists(topic):
            existing.append(topic)
            log(f"  [OK] {topic}")
        else:
            missing.append(topic)
            log(f"  [--] {topic} (未找到)")
    log("")

    if args.check_only:
        log(f"存在: {len(existing)}/{len(args.topics)}")
        if args.output:
            with open(args.output, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
        return 0 if not missing else 1

    # 2. 对存在的话题测 hz / bw / delay
    topics_to_test = existing
    if not topics_to_test:
        log("无可用话题可测，请先启动系统。")
        if args.output:
            with open(args.output, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
        return 1

    log("--- 频率 (Hz) ---")
    for topic in topics_to_test:
        if args.no_hz:
            log(f"  {topic}: (跳过)")
            continue
        log(f"  检测 {topic} ...")
        hz = get_topic_hz_with_timeout(topic, args.duration)
        if hz is not None:
            log(f"  {topic}: {hz:.1f} Hz")
        else:
            log(f"  {topic}: (无数据或超时)")
    log("")

    log("--- 带宽 (MB/s) ---")
    for topic in topics_to_test:
        if args.no_bw:
            log(f"  {topic}: (跳过)")
            continue
        bw = get_topic_bw(topic, min(5.0, args.duration))
        if bw is not None:
            log(f"  {topic}: {bw:.2f} MB/s")
        else:
            log(f"  {topic}: (无数据或不可用)")
    log("")

    log("--- 延迟 (s) ---")
    for topic in topics_to_test:
        if args.no_delay:
            log(f"  {topic}: (跳过)")
            continue
        delay = get_topic_delay(topic, min(5.0, args.duration))
        if delay is not None:
            log(f"  {topic}: {delay:.3f} s")
        else:
            log(f"  {topic}: (无数据或不可用)")
    log("")

    log("=" * 60)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))
        log(f"报告已写入: {args.output}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
