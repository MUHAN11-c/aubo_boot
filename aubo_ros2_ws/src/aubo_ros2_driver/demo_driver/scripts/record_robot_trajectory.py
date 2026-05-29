#!/usr/bin/env python3
"""订阅 /robot_status 话题，实时 3D 显示 + 记录轨迹喵~"""

import os
import signal
import sys
import threading
import time
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")  # 交互式后端
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt

# 注册中文字体，避免 CJK 字符缺失警告
_cjk_font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if os.path.exists(_cjk_font_path):
    fm.fontManager.addfont(_cjk_font_path)
    _cjk_name = fm.FontProperties(fname=_cjk_font_path).get_name()
    matplotlib.rcParams["font.sans-serif"] = [_cjk_name, "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    # 清除字体缓存，确保新注册字体被使用
    fm._load_fontmanager(try_read_cache=False)

import numpy as np
import rclpy
from ivg_interfaces.msg import RobotStatus
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from rclpy.node import Node


class TrajectoryRecorder(Node):
    def __init__(self, output_dir: str = ""):
        super().__init__("trajectory_recorder")
        self.sub = self.create_subscription(
            RobotStatus, "/robot_status", self._cb, 10
        )
        self._lock = threading.Lock()
        self._records: list[dict] = []
        self._start_time = time.time()

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._output_dir = output_dir or os.path.join(
            os.path.expanduser("~"), "robot_trajectories", ts
        )
        os.makedirs(self._output_dir, exist_ok=True)
        self.get_logger().info(f"输出目录: {self._output_dir}")

        self._plot_running = threading.Event()
        self._plot_running.set()
        self._plot_thread = threading.Thread(
            target=self._plot_loop, daemon=True
        )
        self._plot_thread.start()
        self.get_logger().info("实时 3D 显示已启动，按 Ctrl+C 停止录制喵~")

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

    def _get_arrays(self):
        with self._lock:
            if len(self._records) == 0:
                return None
            keys = ["t", "x", "y", "z", "roll", "pitch", "yaw"] + [f"j{i}" for i in range(1, 7)]
            return {k: np.array([r[k] for r in self._records]) for k in keys}

    def _plot_loop(self):
        plt.ion()
        fig = plt.figure(figsize=(16, 9))
        fig.canvas.manager.set_window_title("Robot Trajectory — 实时")

        # 布局：左侧大 3D 视图，右侧信息面板
        gs = fig.add_gridspec(2, 3, width_ratios=[2, 1, 1], height_ratios=[2, 1])

        ax_3d = fig.add_subplot(gs[0, 0], projection="3d")
        ax_info = fig.add_subplot(gs[0, 1:])
        ax_xyz = fig.add_subplot(gs[1, 0])
        ax_rpy = fig.add_subplot(gs[1, 1])
        ax_joint = fig.add_subplot(gs[1, 2])

        # --- 3D 轨迹 (Line3DCollection 高性能渲染) ---
        ax_3d.set_xlabel("X (m)"); ax_3d.set_ylabel("Y (m)"); ax_3d.set_zlabel("Z (m)")
        ax_3d.set_title("TCP 3D Trajectory (实时)")
        trail = Line3DCollection([], cmap=plt.cm.viridis, linewidth=1.0)
        ax_3d.add_collection(trail)
        start_pt = ax_3d.scatter([], [], [], c="lime", s=60, edgecolors="k", zorder=5, label="Start")
        now_pt = ax_3d.scatter([], [], [], c="red", s=80, edgecolors="k", zorder=5, label="Now")
        ax_3d.legend()

        # 信息面板
        ax_info.axis("off")
        info_text = ax_info.text(0.05, 0.95, "", transform=ax_info.transAxes,
                                  fontsize=10, verticalalignment="top")

        # XYZ 时序
        ax_xyz.set_xlabel("Time (s)"); ax_xyz.set_ylabel("Position (m)")
        ax_xyz.set_title("TCP Position"); ax_xyz.grid(True, alpha=0.3)
        line_x, = ax_xyz.plot([], [], label="X", linewidth=0.8)
        line_y, = ax_xyz.plot([], [], label="Y", linewidth=0.8)
        line_z, = ax_xyz.plot([], [], label="Z", linewidth=0.8)
        ax_xyz.legend(loc="upper right")

        # RPY 时序
        ax_rpy.set_xlabel("Time (s)"); ax_rpy.set_ylabel("Angle (deg)")
        ax_rpy.set_title("TCP Orientation"); ax_rpy.grid(True, alpha=0.3)
        line_roll, = ax_rpy.plot([], [], label="Roll", linewidth=0.8)
        line_pitch, = ax_rpy.plot([], [], label="Pitch", linewidth=0.8)
        line_yaw, = ax_rpy.plot([], [], label="Yaw", linewidth=0.8)
        ax_rpy.legend(loc="upper right")

        # 关节角度时序
        ax_joint.set_xlabel("Time (s)"); ax_joint.set_ylabel("Angle (deg)")
        ax_joint.set_title("Joint Angles"); ax_joint.grid(True, alpha=0.3)
        joint_lines = [ax_joint.plot([], [], label=f"J{i+1}", linewidth=0.8)[0] for i in range(6)]
        ax_joint.legend(ncol=3, fontsize=7, loc="upper right")

        plt.tight_layout(pad=2)
        fig.canvas.draw()
        fig.canvas.flush_events()

        while self._plot_running.is_set():
            arr = self._get_arrays()
            if arr is not None and len(arr["t"]) > 0:
                n = len(arr["t"])
                t = arr["t"]

                # --- 3D 轨迹 (增量更新, 不 cla) ---
                if n >= 2:
                    pts = np.column_stack([arr["x"], arr["y"], arr["z"]])
                    segs = np.stack([pts[:-1], pts[1:]], axis=1)  # (n-1, 2, 3)
                    trail.set_segments(segs)
                    trail.set_array(np.linspace(0, 1, n - 1))
                start_pt._offsets3d = (arr["x"][:1], arr["y"][:1], arr["z"][:1])
                now_pt._offsets3d = (arr["x"][-1:], arr["y"][-1:], arr["z"][-1:])
                ax_3d.relim(); ax_3d.autoscale_view()

                # --- 信息面板 ---
                dur = t[-1]
                dx = arr["x"][-1] - arr["x"][0]
                dy = arr["y"][-1] - arr["y"][0]
                dz = arr["z"][-1] - arr["z"][0]
                info_text.set_text(
                    f"点位: {n}\n"
                    f"时长: {dur:.1f}s  频率: {n/dur:.1f}Hz\n"
                    f"当前位置:\n"
                    f"  X={arr['x'][-1]:.4f}  Y={arr['y'][-1]:.4f}  Z={arr['z'][-1]:.4f} m\n"
                    f"  Roll={np.rad2deg(arr['roll'][-1]):.2f}°  "
                    f"Pitch={np.rad2deg(arr['pitch'][-1]):.2f}°  "
                    f"Yaw={np.rad2deg(arr['yaw'][-1]):.2f}°\n"
                    f"  J1={arr['j1'][-1]:.2f}°  J2={arr['j2'][-1]:.2f}°  "
                    f"J3={arr['j3'][-1]:.2f}°  J4={arr['j4'][-1]:.2f}°  "
                    f"J5={arr['j5'][-1]:.2f}°  J6={arr['j6'][-1]:.2f}°\n"
                    f"行程: {np.sqrt(dx**2+dy**2+dz**2):.3f}m  "
                    f"(ΔX={dx:+.3f} ΔY={dy:+.3f} ΔZ={dz:+.3f})"
                )

                # --- XYZ / RPY / 关节时序 (set_data 增量更新) ---
                line_x.set_data(t, arr["x"]); line_y.set_data(t, arr["y"]); line_z.set_data(t, arr["z"])
                ax_xyz.relim(); ax_xyz.autoscale_view()
                line_roll.set_data(t, np.rad2deg(arr["roll"]))
                line_pitch.set_data(t, np.rad2deg(arr["pitch"]))
                line_yaw.set_data(t, np.rad2deg(arr["yaw"]))
                ax_rpy.relim(); ax_rpy.autoscale_view()
                for i in range(6):
                    joint_lines[i].set_data(t, arr[f"j{i+1}"])
                ax_joint.relim(); ax_joint.autoscale_view()

            # plt.pause 内部 draw + flush_events + GUI 事件循环，比分开写更高效
            try:
                plt.pause(0.08)  # ~12Hz
            except Exception:
                break

        plt.close("all")

    def stop_plot(self):
        self._plot_running.clear()
        if self._plot_thread.is_alive():
            self._plot_thread.join(timeout=2)

    def save_data(self):
        n = len(self._records)
        if n == 0:
            self.get_logger().warn("没有录到数据喵~")
            return

        arr = {k: np.array([r[k] for r in self._records]) for k in self._records[0]}

        # NPZ
        npz_path = os.path.join(self._output_dir, "trajectory.npz")
        np.savez(npz_path, **arr)
        self.get_logger().info(f"数据已保存: {npz_path} ({n} 条记录)")

        # CSV
        csv_path = os.path.join(self._output_dir, "trajectory.csv")
        header = ",".join(arr.keys())
        data = np.column_stack([arr[k] for k in arr])
        np.savetxt(csv_path, data, delimiter=",", header=header, comments="")
        self.get_logger().info(f"CSV 已保存: {csv_path}")

        # 统计
        dur = arr["t"][-1]
        dx = arr["x"][-1] - arr["x"][0]
        dy = arr["y"][-1] - arr["y"][0]
        dz = arr["z"][-1] - arr["z"][0]
        self.get_logger().info(
            f"录制时长: {dur:.1f}s, 点位: {n}, 频率: {n/dur:.1f}Hz, "
            f"行程: ({dx:+.3f}, {dy:+.3f}, {dz:+.3f})m"
        )


def main():
    rclpy.init()
    node = TrajectoryRecorder()

    shutdown_flag = threading.Event()

    def shutdown(sig=None, frame=None):
        if shutdown_flag.is_set():
            return
        shutdown_flag.set()
        if rclpy.ok():
            node.get_logger().info("正在停止...")
            node.stop_plot()
            node.save_data()
            node.destroy_node()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        while rclpy.ok() and not shutdown_flag.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()


if __name__ == "__main__":
    main()
