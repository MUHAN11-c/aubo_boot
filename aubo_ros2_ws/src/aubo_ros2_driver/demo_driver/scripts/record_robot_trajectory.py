#!/usr/bin/env python3
"""轨迹录制喵~

用法: python3 record_robot_trajectory.py

  - 订阅 /robot_status, 记录全部数据
  - 控制台实时摘要 (每 0.5s)
  - 拉花段标记: ROS service ~/start_latte_record / ~/stop_latte_record
  - 退出时自动保存到 ~/robot_trajectories/<时间戳>/
  - 查看数据: 浏览器打开 Web Dashboard → 轨迹回放页面
"""

import atexit
import os
import signal
import sys
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from ivg_interfaces.msg import RobotStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


# ═══════════════════════════════════════════════════════════════════
# TrajectoryRecorder — 纯录制节点 (无 GUI)
# ═══════════════════════════════════════════════════════════════════

class TrajectoryRecorder(Node):
    def __init__(self, shutdown_evt: threading.Event, output_dir: str = ""):
        super().__init__("trajectory_recorder")
        self._shutdown = shutdown_evt
        self.sub = self.create_subscription(
            RobotStatus, "/robot_status", self._cb, 10
        )
        self._lock = threading.Lock()
        self._records: list[dict] = []
        self._start_time = time.time()

        # 拉花段: [(start_idx, end_idx), ...]  None=尚未停止
        self._latte_segments: list[list] = []
        self._latte_recording = False

        # ROS service
        self._srv_start = self.create_service(
            Trigger, "~/start_latte_record", self._on_start_latte)
        self._srv_stop = self.create_service(
            Trigger, "~/stop_latte_record", self._on_stop_latte)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._output_dir = output_dir or os.path.join(
            os.path.expanduser("~"), "robot_trajectories", ts
        )
        os.makedirs(self._output_dir, exist_ok=True)
        self.get_logger().info(f"输出目录: {self._output_dir}")
        self._saved = False

    # ── ROS service 回调 ───────────────────────────────────────────

    def _on_start_latte(self, req, rsp):
        with self._lock:
            if not self._latte_recording:
                self._latte_recording = True
                self._latte_segments.append([len(self._records), None])
                self.get_logger().info(
                    f"🏁 拉花 #{len(self._latte_segments)} 开始 "
                    f"(idx={self._latte_segments[-1][0]}) 喵~")
            rsp.success = True
            rsp.message = f"started segment {len(self._latte_segments)}"
        return rsp

    def _on_stop_latte(self, req, rsp):
        with self._lock:
            if self._latte_recording and self._latte_segments:
                seg = self._latte_segments[-1]
                seg[1] = len(self._records)
                self._latte_recording = False
                n = seg[1] - seg[0]
                self.get_logger().info(
                    f"🏁 拉花 #{len(self._latte_segments)} 停止 (共 {n} 点) 喵~")
            rsp.success = True
            rsp.message = "stopped"
        return rsp

    # ── 数据回调 ───────────────────────────────────────────────────

    def _cb(self, msg: RobotStatus):
        elapsed = time.time() - self._start_time
        rec = {
            "t": elapsed,
            "stamp_ns": msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec,
            "x": msg.cartesian_position_xyz.x,
            "y": msg.cartesian_position_xyz.y,
            "z": msg.cartesian_position_xyz.z,
            "roll": msg.cartesian_rpy.x,
            "pitch": msg.cartesian_rpy.y,
            "yaw": msg.cartesian_rpy.z,
            "j1": msg.joint_position_deg[0],
            "j2": msg.joint_position_deg[1],
            "j3": msg.joint_position_deg[2],
            "j4": msg.joint_position_deg[3],
            "j5": msg.joint_position_deg[4],
            "j6": msg.joint_position_deg[5],
        }
        with self._lock:
            self._records.append(rec)

    # ── 数据保存 ───────────────────────────────────────────────────

    def save_data(self):
        if self._saved:
            return
        self._saved = True
        with self._lock:
            full = list(self._records)
            segments = []
            for s, e in self._latte_segments:
                end = e if e is not None else len(full)
                if end - s >= 2:
                    segments.append((s, end))

        if len(full) == 0:
            self.get_logger().warn("没有录到数据喵~")
            return

        self._save_array(full, "full")
        for i, (s, e) in enumerate(segments):
            self._save_array(full[s:e], f"latte_{i+1}")

        self.get_logger().info(f"全部保存到: {self._output_dir} 喵~")

    def _save_array(self, records: list[dict], tag: str):
        n = len(records)
        if n == 0:
            return
        arr = {k: np.array([r[k] for r in records]) for k in records[0]}

        npz_path = os.path.join(self._output_dir, f"trajectory_{tag}.npz")
        np.savez(npz_path, **arr)
        self.get_logger().info(f"NPZ: {npz_path} ({n} 条)")

        csv_path = os.path.join(self._output_dir, f"trajectory_{tag}.csv")
        header = ",".join(arr.keys())
        data = np.column_stack([arr[k] for k in arr])
        np.savetxt(csv_path, data, delimiter=",", header=header, comments="")
        self.get_logger().info(f"CSV: {csv_path}")

        dur = arr["t"][-1] - arr["t"][0]
        self.get_logger().info(
            f"[{tag}] 时长:{dur:.1f}s 点位:{n} 频率:{n/dur:.1f}Hz")

    def auto_save(self):
        """定期自动保存完整数据"""
        with self._lock:
            if len(self._records) == 0:
                return
            full = list(self._records)
        try:
            arr = {k: np.array([r[k] for r in full]) for k in full[0]}
            np.savez(os.path.join(self._output_dir, "trajectory_full.npz"), **arr)
        except Exception:
            pass

    # ── 录制主循环 (纯控制台, 无 GUI) ──────────────────────────────

    def run_record(self):
        self.get_logger().info("录制中 — Ctrl+C 停止并保存喵~")
        last_autosave = time.time()
        was_latte = False
        while not self._shutdown.is_set():
            time.sleep(0.5)

            # ── 锁内快照数据 (记录引用是 append-only, 索引访问安全但需在锁内完成) ──
            with self._lock:
                n = len(self._records)
                if n == 0:
                    continue
                is_latte = self._latte_recording
                segments_info = [(s, e) for s, e in self._latte_segments]
                rec = self._records[-1]
                prev = self._records[-2] if n >= 2 else None
                seg_start_rec = None
                if is_latte and segments_info:
                    seg = segments_info[-1]
                    seg_start_rec = self._records[seg[0]]
                # 拉花段汇总用
                seg_summary = None
                if was_latte and segments_info:
                    seg = segments_info[-1]
                    end_idx = seg[1] if seg[1] is not None else n
                    if end_idx - seg[0] > 0:
                        seg_summary = (
                            seg[0], end_idx,
                            self._records[seg[0]],
                            self._records[end_idx - 1]
                        )
            # ── 锁外: 控制台 I/O + 计算 (不访问 self._records) ──

            if is_latte and seg_start_rec is not None:
                latte_idx = n - seg[0]
                latte_t = rec["t"] - seg_start_rec["t"]
                # 瞬时速度
                speed = 0.0
                if prev is not None:
                    dt = rec["t"] - prev["t"]
                    if dt > 0:
                        dx = rec["x"] - prev["x"]
                        dy = rec["y"] - prev["y"]
                        dz = rec["z"] - prev["z"]
                        speed = np.sqrt(dx*dx + dy*dy + dz*dz) / dt
                roll_d = np.rad2deg(rec["roll"])
                pitch_d = np.rad2deg(rec["pitch"])
                yaw_d = np.rad2deg(rec["yaw"])
                print(
                    f"\r[拉花 #{len(segments_info)} #{latte_idx:4d} t={latte_t:6.2f}s] "
                    f"X={rec['x']:8.4f} Y={rec['y']:8.4f} Z={rec['z']:7.4f} | "
                    f"R={roll_d:6.1f}° P={pitch_d:5.1f}° Y={yaw_d:6.1f}° | "
                    f"v={speed:.3f}m/s  ",
                    end="", flush=True)
                was_latte = True
            else:
                seg_hint = f" 已录{len(segments_info)}段" if segments_info else ""
                print(
                    f"\r[录制中] 点位:{n:5d}  t={rec['t']:6.1f}s  "
                    f"X={rec['x']:7.3f} Y={rec['y']:7.3f} Z={rec['z']:6.3f}{seg_hint}  ",
                    end="", flush=True)

                # 拉花段结束汇总
                if seg_summary is not None:
                    s_idx, e_idx, p_start, p_end = seg_summary
                    latte_n = e_idx - s_idx
                    ddx = p_end["x"] - p_start["x"]
                    ddy = p_end["y"] - p_start["y"]
                    ddz = p_end["z"] - p_start["z"]
                    latte_dur = p_end["t"] - p_start["t"]
                    print(
                        f"\n── 拉花 #{len(segments_info)} 汇总: "
                        f"{latte_n}点 {latte_dur:.1f}s "
                        f"Δ=({ddx:+.3f},{ddy:+.3f},{ddz:+.3f})m ──")
                was_latte = False

            # ── 自动保存 ──
            if time.time() - last_autosave > 10:
                self.auto_save()
                last_autosave = time.time()

        print()  # 换行


# ═══════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════

def main():
    # ── 录制模式 ──
    rclpy.init()
    shutdown_evt = threading.Event()
    node = TrajectoryRecorder(shutdown_evt, "")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def spin():
        while rclpy.ok() and not shutdown_evt.is_set():
            executor.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=spin, daemon=True, name="ros-spin")
    spin_thread.start()

    def on_signal(sig, frame):
        shutdown_evt.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGHUP, on_signal)

    def _atexit_save():
        try:
            node.save_data()
        except Exception:
            pass
    atexit.register(_atexit_save)

    try:
        node.run_record()
    finally:
        shutdown_evt.set()
        node.get_logger().info("正在保存数据...")
        try:
            node.save_data()
        except Exception as e:
            node.get_logger().error(f"保存数据失败: {e}")
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()
        print("\n轨迹录制已停止喵~")
        print(f"浏览器查看: Web Dashboard → 轨迹回放")


if __name__ == "__main__":
    main()
