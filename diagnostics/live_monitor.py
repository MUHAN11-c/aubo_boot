#!/usr/bin/env python3
"""实时监测：与 bringup（sim/real）+ RViz2 并行运行，只订阅 ROS 话题，
不新增 SDK 连接，对控制通道零干扰。

订阅（均为运行中系统已发布的话题）：
  /joint_states                          反馈频率/延迟（joint_state_broadcaster 200Hz）
  /aubo_io_controller/rib_status         [RIB 水位, 发送队列点数, 瞬时吞吐 pts/s]
  /aubo_io_controller/joint_status       各关节 following_error（可选，需 aubo_msgs）

输出（每次运行覆盖最新）：
  results/live_monitor_latest.csv        逐帧合并时间线（~200 行/s）
  results/live_monitor_latest.png        退出时的全程汇总曲线

用法:
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  aubo_py3.12/bin/python diagnostics/live_monitor.py            # 实时窗口 + 落盘
  aubo_py3.12/bin/python diagnostics/live_monitor.py --no-gui   # 无头记录（Ctrl-C 出图）
  aubo_py3.12/bin/python diagnostics/live_monitor.py --window 30
"""

import argparse
import csv
import os
import signal
import sys
import threading
import time

import matplotlib

import numpy as np
import rclpy
import rclpy.executors
import rclpy.signals
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

DEFAULT_RESULTS_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "results"
)

JS_TOPIC = "/joint_states"
RIB_TOPIC = "/aubo_io_controller/rib_status"
JOINT_STATUS_TOPIC = "/aubo_io_controller/joint_status"

# 与硬件参数一致的参考线（aubo_e5.ros2_control.xacro 默认值）
RIB_SLOWDOWN_1 = 300
RIB_SLOWDOWN_2 = 350


class LiveMonitor(Node):
    def __init__(self, csv_path):
        super().__init__("aubo_live_monitor")
        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.last_js_wall = None
        self.latest_rib = (np.nan, np.nan, np.nan)  # rib, queue, rate
        self.latest_following = np.nan
        self.rows = []  # 全程数据（内存），退出时绘图用
        self.js_count = 0

        self.csv_file = open(csv_path, "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(
            [
                "t_sec",
                "js_interval_ms",
                "js_delay_ms",
                "rib_level",
                "send_queue_points",
                "send_rate_pps",
                "max_following_error_rad",
            ]
        )

        self.create_subscription(
            JointState, JS_TOPIC, self.on_joint_states, qos_profile_system_default
        )
        self.create_subscription(
            Int32MultiArray, RIB_TOPIC, self.on_rib, qos_profile_system_default
        )
        # aubo_msgs 为本工作区消息：source install/setup.bash 后可用；
        # 不可用时降级为不记录 following_error，其余功能不变。
        try:
            from aubo_msgs.msg import JointStatus as AuboJointStatus

            self.create_subscription(
                AuboJointStatus,
                JOINT_STATUS_TOPIC,
                self.on_joint_status,
                qos_profile_system_default,
            )
            self.has_joint_status = True
        except ImportError:
            self.has_joint_status = False
            self.get_logger().warn(
                "aubo_msgs 不可用（未 source install/setup.bash?），"
                "跳过 following_error 监测"
            )

    def on_rib(self, msg):
        data = list(msg.data) + [np.nan] * 3
        with self.lock:
            self.latest_rib = (float(data[0]), float(data[1]), float(data[2]))

    def on_joint_status(self, msg):
        with self.lock:
            self.latest_following = float(max(abs(v) for v in msg.following_error))

    def on_joint_states(self, msg):
        now_wall = time.monotonic()
        now_ros = self.get_clock().now().nanoseconds / 1e9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        delay_ms = (now_ros - stamp) * 1000.0 if stamp > 0 else np.nan
        with self.lock:
            interval_ms = (
                (now_wall - self.last_js_wall) * 1000.0
                if self.last_js_wall is not None
                else np.nan
            )
            self.last_js_wall = now_wall
            rib, queue, rate = self.latest_rib
            following = self.latest_following
            row = (
                now_wall - self.t0,
                interval_ms,
                delay_ms,
                rib,
                queue,
                rate,
                following,
            )
            self.rows.append(row)
            self.js_count += 1
        self.csv.writerow([f"{v:.6f}" if isinstance(v, float) else v for v in row])
        if self.js_count % 200 == 0:
            self.csv_file.flush()

    def snapshot(self):
        with self.lock:
            return np.array(self.rows, dtype=float) if self.rows else None

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()


def make_panels(fig, axes):
    """创建一次双轴面板（twinx 只建一次，重绘时 clear 即可；
    每次重绘新建 twinx 会导致 ylabel 跑到左侧重叠）。"""
    return {
        "rib": axes[0],
        "rib_rate": axes[0].twinx(),
        "js": axes[1],
        "js_interval": axes[1].twinx(),
        "err": axes[2],
    }


def draw_panels(data, panels, window=None):
    """绘制 RIB/延迟/误差曲线。window 给定时只画最近 window 秒。"""
    if window is not None:
        data = data[data[:, 0] >= data[-1, 0] - window]
    t = data[:, 0]

    ax, ax2 = panels["rib"], panels["rib_rate"]
    ax.clear()
    ax2.clear()
    # clear() 会把 twinx 轴的 ylabel/ticks 位置重置回左侧，需显式恢复
    ax2.yaxis.set_label_position("right")
    ax2.yaxis.tick_right()
    ax.plot(t, data[:, 3], lw=0.8, label="RIB level")
    ax.axhline(RIB_SLOWDOWN_1, color="orange", ls="--", alpha=0.6,
               label=f"slowdown1={RIB_SLOWDOWN_1}")
    ax.axhline(RIB_SLOWDOWN_2, color="r", ls="--", alpha=0.6,
               label=f"slowdown2={RIB_SLOWDOWN_2}")
    ax2.plot(t, data[:, 5], lw=0.6, color="g", alpha=0.7, label="send pts/s")
    ax2.set_ylabel("send rate (pts/s)", color="g")
    ax.set_ylabel("RIB level")
    ax.set_title("RIB buffer / command throughput")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    ax, ax2 = panels["js"], panels["js_interval"]
    ax.clear()
    ax2.clear()
    ax2.yaxis.set_label_position("right")
    ax2.yaxis.tick_right()
    ax.plot(t, data[:, 2], lw=0.6, label="feedback delay (ms)")
    ax.set_ylabel("delay (ms)")
    ax.set_title("joint_states feedback delay / interval")
    ax.grid(True, alpha=0.3)
    ax2.plot(t, data[:, 1], lw=0.6, color="purple", alpha=0.6,
             label="interval (ms)")
    ax2.set_ylabel("interval (ms)", color="purple")
    ax.legend(loc="upper left", fontsize=8)

    ax = panels["err"]
    ax.clear()
    if np.isnan(data[:, 6]).all():
        ax.text(0.5, 0.5, "following_error unavailable",
                ha="center", va="center", transform=ax.transAxes)
    else:
        ax.plot(t, data[:, 6] * 1000.0, lw=0.6, color="tab:red")
        ax.set_ylabel("max following error (mrad)")
    ax.set_xlabel("time (s)")
    ax.set_title("trajectory following error")
    ax.grid(True, alpha=0.3)


def save_summary_png(data, png_path):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle(f"live_monitor summary ({data[-1, 0]:.1f}s, "
                 f"{data.shape[0]} joint_states frames)")
    draw_panels(data, make_panels(fig, axes))
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(png_path, dpi=120)
    plt.close(fig)
    print(f"saved {png_path}")


def print_summary(data):
    def stat(name, col, scale=1.0, unit=""):
        v = data[:, col] * scale
        v = v[~np.isnan(v)]
        if v.size == 0:
            return f"{name}: no data"
        return (f"{name}: min={v.min():.3f} avg={v.mean():.3f} "
                f"p99={np.percentile(v, 99):.3f} max={v.max():.3f}{unit}")

    span = data[-1, 0] - data[0, 0]
    rate = (data.shape[0] - 1) / span if span > 0 else 0.0
    print(f"duration={span:.1f}s frames={data.shape[0]} "
          f"joint_states_rate={rate:.1f} Hz")
    print(stat("feedback_delay_ms", 2))
    print(stat("feedback_interval_ms", 1))
    print(stat("rib_level", 3))
    print(stat("send_rate_pps", 5))
    print(stat("following_error_mrad", 6, scale=1000.0))
    rib = data[:, 3]
    rib = rib[~np.isnan(rib)]
    if rib.size:
        starve = (rib == 0).mean() * 100.0
        over = (rib >= RIB_SLOWDOWN_2).mean() * 100.0
        print(f"rib_starve(==0)={starve:.2f}% rib_over_target(>={RIB_SLOWDOWN_2})"
              f"={over:.2f}%")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", default=DEFAULT_RESULTS_DIR)
    parser.add_argument("--window", type=float, default=60.0,
                        help="实时窗口显示最近 N 秒（全程仍记录 CSV）")
    parser.add_argument("--no-gui", action="store_true",
                        help="无头模式：只记录 CSV，退出时出 PNG")
    args = parser.parse_args()

    os.makedirs(args.results_dir, exist_ok=True)
    csv_path = os.path.join(args.results_dir, "live_monitor_latest.csv")
    png_path = os.path.join(args.results_dir, "live_monitor_latest.png")

    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = LiveMonitor(csv_path)

    # 显式信号处理：进程以 shell 后台任务（`cmd &`）启动时 SIGINT 会被继承为
    # SIG_IGN，KeyboardInterrupt 永远不会触发；signal.signal 会覆盖忽略语义，
    # 保证 SIGINT/SIGTERM 都能干净退出（落盘 + 出图）。
    stop_event = threading.Event()

    def _on_signal(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    def spin():
        try:
            rclpy.spin(node)
        except rclpy.executors.ExternalShutdownException:
            pass  # 主线程显式 shutdown 后的正常退出路径

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    if args.no_gui:
        matplotlib.use("Agg")
        print(f"记录中（无头）：{csv_path}  Ctrl-C / SIGTERM 结束并出图")
        while not stop_event.is_set():
            stop_event.wait(1.0)
    else:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        fig.suptitle("AUBO E5 live monitor (Ctrl-C / close window to stop)")
        panels = make_panels(fig, axes)

        def update(_frame):
            if stop_event.is_set():
                plt.close(fig)
                return
            data = node.snapshot()
            if data is not None and data.shape[0] > 2:
                draw_panels(data, panels, window=args.window)

        anim = FuncAnimation(fig, update, interval=1000, cache_frame_data=False)
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        _ = anim  # 防止被 GC

    node.close()
    data = node.snapshot()
    node.destroy_node()
    # SIGINT 时 rclpy 自己的信号处理可能已关闭上下文，重复 shutdown 会抛错
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    if data is not None and data.shape[0] > 2:
        print_summary(data)
        save_summary_png(data, png_path)
    else:
        print("没有收到数据（bringup 是否在运行？）", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
