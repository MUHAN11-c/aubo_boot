#!/usr/bin/env python3
"""
JointState 运动过程分析器。

功能：
1. 订阅 /joint_states
2. 实时计算位置/速度/加速度（若消息无速度则由位置差分估计）
3. 自动识别运动段（开始/结束）
4. 可选保存每个采样点到 CSV，便于离线分析
5. 节点退出时打印汇总统计

示例：
  ros2 run demo_driver joint_states_motion_analyzer.py

  # 默认输出（覆盖最新）：
  #   /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/scripts/joint_states_motion_latest.csv
  #   /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/scripts/joint_states_motion_latest_events.csv
  # 默认静默采集，Ctrl+C 后一次性输出分析汇总

  ros2 run demo_driver joint_states_motion_analyzer.py --ros-args \
    -p joint_state_topic:=/joint_states \
    -p target_joints:="['shoulder_joint','elbow_joint']" \
    -p realtime_console_log:=true \
    -p print_hz:=2.0
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class MotionSegment:
    index: int
    start_time: float
    end_time: float
    duration: float
    peak_speed: float
    peak_acc: float


@dataclass
class AnomalyEvent:
    t: float
    event_type: str
    joint: str
    value: float
    threshold: float
    note: str


class JointStatesMotionAnalyzer(Node):
    def __init__(self) -> None:
        super().__init__("joint_states_motion_analyzer")

        default_csv = (
            "/home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/"
            "demo_driver/scripts/joint_states_motion_latest.csv"
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("target_joints", [])
        self.declare_parameter("output_csv", default_csv)
        self.declare_parameter("save_samples", True)
        self.declare_parameter("save_events", True)
        self.declare_parameter("print_hz", 0.0)
        self.declare_parameter("motion_start_threshold", 0.02)  # rad/s
        self.declare_parameter("motion_stop_threshold", 0.01)   # rad/s
        self.declare_parameter("sudden_acc_threshold", 1.5)     # rad/s^2
        self.declare_parameter("sudden_jerk_threshold", 20.0)   # rad/s^3
        self.declare_parameter("stutter_speed_threshold", 0.03) # rad/s
        self.declare_parameter("stutter_min_duration", 0.2)     # s
        self.declare_parameter("dt_gap_threshold", 0.08)        # s
        self.declare_parameter("warn_cooldown_sec", 0.5)        # s
        self.declare_parameter("realtime_console_log", False)

        self.topic = self.get_parameter("joint_state_topic").value
        self.target_joints = list(self.get_parameter("target_joints").value)
        self.output_csv = self.get_parameter("output_csv").value
        self.save_samples = bool(self.get_parameter("save_samples").value)
        self.save_events = bool(self.get_parameter("save_events").value)
        self.print_hz = float(self.get_parameter("print_hz").value)
        self.motion_start_threshold = float(self.get_parameter("motion_start_threshold").value)
        self.motion_stop_threshold = float(self.get_parameter("motion_stop_threshold").value)
        self.sudden_acc_threshold = float(self.get_parameter("sudden_acc_threshold").value)
        self.sudden_jerk_threshold = float(self.get_parameter("sudden_jerk_threshold").value)
        self.stutter_speed_threshold = float(self.get_parameter("stutter_speed_threshold").value)
        self.stutter_min_duration = float(self.get_parameter("stutter_min_duration").value)
        self.dt_gap_threshold = float(self.get_parameter("dt_gap_threshold").value)
        self.warn_cooldown_sec = float(self.get_parameter("warn_cooldown_sec").value)
        self.realtime_console_log = bool(self.get_parameter("realtime_console_log").value)
        self.events_csv_path = self.output_csv.replace(".csv", "_events.csv")

        self.selected_names: List[str] = []
        self.selected_indices: List[int] = []
        self.prev_stamp_s: Optional[float] = None
        self.prev_pos: Optional[List[float]] = None
        self.prev_vel: Optional[List[float]] = None
        self.prev_acc: Optional[List[float]] = None

        self.sample_count = 0
        self.total_duration = 0.0
        self.last_speed = 0.0
        self.last_acc = 0.0
        self.last_jerk = 0.0

        self.pos_min: Dict[str, float] = {}
        self.pos_max: Dict[str, float] = {}
        self.vel_abs_max: Dict[str, float] = {}
        self.acc_abs_max: Dict[str, float] = {}

        self.is_moving = False
        self.segment_start_time: Optional[float] = None
        self.segment_peak_speed = 0.0
        self.segment_peak_acc = 0.0
        self.motion_segments: List[MotionSegment] = []
        self.anomaly_events: List[AnomalyEvent] = []
        self.last_warn_time = 0.0
        self.stutter_start_time: Optional[float] = None
        self.stutter_reported = False

        self.csv_file = None
        self.csv_writer = None
        if self.save_samples and self.output_csv:
            out_dir = os.path.dirname(self.output_csv)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            self.csv_file = open(self.output_csv, "w", newline="", encoding="utf-8")
        self.events_file = None
        self.events_writer = None
        if self.save_events and self.output_csv:
            self.events_file = open(self.events_csv_path, "w", newline="", encoding="utf-8")
            self.events_writer = csv.writer(self.events_file)
            self.events_writer.writerow(["t", "event_type", "joint", "value", "threshold", "note"])
            self.events_file.flush()

        self.sub = self.create_subscription(JointState, self.topic, self.joint_state_cb, 100)
        self.timer = None
        if self.print_hz > 0.0 and self.realtime_console_log:
            period = 1.0 / self.print_hz
            self.timer = self.create_timer(period, self.print_realtime_status)

    def _stamp_to_sec(self, msg: JointState) -> float:
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return self.get_clock().now().nanoseconds * 1e-9
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _init_selection_if_needed(self, msg: JointState) -> bool:
        if self.selected_indices:
            return True
        if not msg.name or not msg.position:
            return False

        if self.target_joints:
            name_to_index = {n: i for i, n in enumerate(msg.name)}
            missing = [n for n in self.target_joints if n not in name_to_index]
            if missing:
                self.get_logger().error(f"以下目标关节在 joint_states 中不存在: {missing}")
                return False
            self.selected_names = self.target_joints[:]
            self.selected_indices = [name_to_index[n] for n in self.selected_names]
        else:
            self.selected_names = list(msg.name)
            self.selected_indices = list(range(len(msg.name)))

        for name in self.selected_names:
            self.pos_min[name] = math.inf
            self.pos_max[name] = -math.inf
            self.vel_abs_max[name] = 0.0
            self.acc_abs_max[name] = 0.0

        if self.csv_file:
            header = ["t", "max_speed", "max_acc", "max_jerk", "is_moving"]
            for name in self.selected_names:
                header.extend([f"{name}_pos", f"{name}_vel", f"{name}_acc", f"{name}_jerk"])
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(header)
            self.csv_file.flush()

        self.get_logger().info(f"分析关节: {self.selected_names}")
        return True

    def joint_state_cb(self, msg: JointState) -> None:
        if not self._init_selection_if_needed(msg):
            return

        t = self._stamp_to_sec(msg)
        pos = [msg.position[i] for i in self.selected_indices]

        if self.prev_stamp_s is None:
            self.prev_stamp_s = t
            self.prev_pos = pos
            self.prev_vel = [0.0] * len(pos)
            self.prev_acc = [0.0] * len(pos)
            return

        dt = t - self.prev_stamp_s
        if dt <= 1e-6:
            return

        if msg.velocity and len(msg.velocity) >= max(self.selected_indices) + 1:
            vel = [msg.velocity[i] for i in self.selected_indices]
        else:
            vel = [(pos[i] - self.prev_pos[i]) / dt for i in range(len(pos))]

        acc = [(vel[i] - self.prev_vel[i]) / dt for i in range(len(vel))]
        jerk = [(acc[i] - self.prev_acc[i]) / dt for i in range(len(acc))]

        max_speed = max(abs(v) for v in vel) if vel else 0.0
        max_acc = max(abs(a) for a in acc) if acc else 0.0
        max_jerk = max(abs(j) for j in jerk) if jerk else 0.0
        self.last_speed = max_speed
        self.last_acc = max_acc
        self.last_jerk = max_jerk
        self.sample_count += 1
        self.total_duration += dt

        for i, name in enumerate(self.selected_names):
            self.pos_min[name] = min(self.pos_min[name], pos[i])
            self.pos_max[name] = max(self.pos_max[name], pos[i])
            self.vel_abs_max[name] = max(self.vel_abs_max[name], abs(vel[i]))
            self.acc_abs_max[name] = max(self.acc_abs_max[name], abs(acc[i]))

        self._update_motion_segments(t, max_speed, max_acc)
        self._detect_anomalies(t, dt, vel, acc, jerk, max_speed, max_acc, max_jerk)

        if self.csv_writer is not None:
            row = [f"{t:.6f}", f"{max_speed:.6f}", f"{max_acc:.6f}", f"{max_jerk:.6f}", int(self.is_moving)]
            for i in range(len(self.selected_names)):
                row.extend([f"{pos[i]:.8f}", f"{vel[i]:.8f}", f"{acc[i]:.8f}", f"{jerk[i]:.8f}"])
            self.csv_writer.writerow(row)
            self.csv_file.flush()

        self.prev_stamp_s = t
        self.prev_pos = pos
        self.prev_vel = vel
        self.prev_acc = acc

    def _record_event(self, event: AnomalyEvent) -> None:
        self.anomaly_events.append(event)
        if self.events_writer is not None:
            self.events_writer.writerow(
                [f"{event.t:.6f}", event.event_type, event.joint, f"{event.value:.6f}", f"{event.threshold:.6f}", event.note]
            )
            self.events_file.flush()
        if self.realtime_console_log:
            self.get_logger().warn(
                f"[异常] {event.event_type} joint={event.joint} value={event.value:.4f} "
                f"(threshold={event.threshold:.4f}) {event.note}"
            )

    def _detect_anomalies(
        self,
        t: float,
        dt: float,
        vel: List[float],
        acc: List[float],
        jerk: List[float],
        max_speed: float,
        max_acc: float,
        max_jerk: float,
    ) -> None:
        if dt > self.dt_gap_threshold:
            self._record_event(
                AnomalyEvent(t=t, event_type="sample_gap", joint="-", value=dt, threshold=self.dt_gap_threshold,
                             note="joint_states 时间间隔过大，可能通信或调度抖动")
            )

        if t - self.last_warn_time < self.warn_cooldown_sec:
            return

        if max_acc >= self.sudden_acc_threshold and acc:
            i = max(range(len(acc)), key=lambda k: abs(acc[k]))
            self._record_event(
                AnomalyEvent(
                    t=t,
                    event_type="sudden_accel",
                    joint=self.selected_names[i],
                    value=abs(acc[i]),
                    threshold=self.sudden_acc_threshold,
                    note="疑似突然加速/减速",
                )
            )
            self.last_warn_time = t
            return

        if max_jerk >= self.sudden_jerk_threshold and jerk:
            i = max(range(len(jerk)), key=lambda k: abs(jerk[k]))
            self._record_event(
                AnomalyEvent(
                    t=t,
                    event_type="sudden_jerk",
                    joint=self.selected_names[i],
                    value=abs(jerk[i]),
                    threshold=self.sudden_jerk_threshold,
                    note="加速度突变，疑似控制不连续",
                )
            )
            self.last_warn_time = t
            return

        if self.is_moving:
            if max_speed <= self.stutter_speed_threshold:
                if self.stutter_start_time is None:
                    self.stutter_start_time = t
                elif (t - self.stutter_start_time) >= self.stutter_min_duration and not self.stutter_reported:
                    self._record_event(
                        AnomalyEvent(
                            t=t,
                            event_type="stutter",
                            joint="-",
                            value=t - self.stutter_start_time,
                            threshold=self.stutter_min_duration,
                            note="运动中速度长期接近 0，疑似卡顿",
                        )
                    )
                    self.stutter_reported = True
                    self.last_warn_time = t
            else:
                self.stutter_start_time = None
                self.stutter_reported = False
        else:
            self.stutter_start_time = None
            self.stutter_reported = False

    def _update_motion_segments(self, t: float, max_speed: float, max_acc: float) -> None:
        if not self.is_moving:
            if max_speed >= self.motion_start_threshold:
                self.is_moving = True
                self.segment_start_time = t
                self.segment_peak_speed = max_speed
                self.segment_peak_acc = max_acc
                if self.realtime_console_log:
                    self.get_logger().info(
                        f"[运动开始] segment={len(self.motion_segments)+1}, t={t:.3f}, speed={max_speed:.4f} rad/s"
                    )
            return

        self.segment_peak_speed = max(self.segment_peak_speed, max_speed)
        self.segment_peak_acc = max(self.segment_peak_acc, max_acc)

        if max_speed <= self.motion_stop_threshold:
            duration = t - self.segment_start_time if self.segment_start_time is not None else 0.0
            seg = MotionSegment(
                index=len(self.motion_segments) + 1,
                start_time=self.segment_start_time if self.segment_start_time is not None else t,
                end_time=t,
                duration=duration,
                peak_speed=self.segment_peak_speed,
                peak_acc=self.segment_peak_acc,
            )
            self.motion_segments.append(seg)
            if self.realtime_console_log:
                self.get_logger().info(
                    f"[运动结束] segment={seg.index}, duration={seg.duration:.3f}s, "
                    f"peak_speed={seg.peak_speed:.4f}, peak_acc={seg.peak_acc:.4f}"
                )
            self.is_moving = False
            self.segment_start_time = None
            self.segment_peak_speed = 0.0
            self.segment_peak_acc = 0.0
            self.stutter_start_time = None
            self.stutter_reported = False

    def print_realtime_status(self) -> None:
        if not self.realtime_console_log:
            return
        if not self.selected_names:
            self.get_logger().info("等待 joint_states 首帧...")
            return
        self.get_logger().info(
            f"样本={self.sample_count}, 已运行={self.total_duration:.2f}s, "
            f"当前速度峰值={self.last_speed:.4f} rad/s, 当前加速度峰值={self.last_acc:.4f} rad/s^2, "
            f"当前加加速度峰值={self.last_jerk:.4f} rad/s^3, "
            f"异常事件={len(self.anomaly_events)}, "
            f"运动状态={'moving' if self.is_moving else 'idle'}"
        )

    def print_summary(self) -> None:
        self.get_logger().info("=" * 60)
        self.get_logger().info("JointState 运动分析汇总")
        self.get_logger().info(f"样本数: {self.sample_count}")
        self.get_logger().info(f"总时长: {self.total_duration:.3f} s")
        self.get_logger().info(f"运动段数量: {len(self.motion_segments)}")
        self.get_logger().info(f"异常事件数量: {len(self.anomaly_events)}")

        for name in self.selected_names:
            pmin = self.pos_min.get(name, 0.0)
            pmax = self.pos_max.get(name, 0.0)
            vmax = self.vel_abs_max.get(name, 0.0)
            amax = self.acc_abs_max.get(name, 0.0)
            self.get_logger().info(
                f"[{name}] pos_range=[{pmin:.6f}, {pmax:.6f}] rad, "
                f"|vel|max={vmax:.6f} rad/s, |acc|max={amax:.6f} rad/s^2"
            )

        for seg in self.motion_segments:
            self.get_logger().info(
                f"[segment {seg.index}] t=[{seg.start_time:.3f}, {seg.end_time:.3f}], "
                f"duration={seg.duration:.3f}s, peak_speed={seg.peak_speed:.4f}, peak_acc={seg.peak_acc:.4f}"
            )
        if self.anomaly_events:
            counts: Dict[str, int] = {}
            for ev in self.anomaly_events:
                counts[ev.event_type] = counts.get(ev.event_type, 0) + 1
            self.get_logger().info(f"异常分类统计: {counts}")

        if self.csv_file:
            self.get_logger().info(f"采样数据已保存: {self.output_csv}")
        if self.events_file:
            self.get_logger().info(f"异常事件已保存: {self.events_csv_path}")
        self.get_logger().info("=" * 60)

    def close(self) -> None:
        self.print_summary()
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        if self.events_file:
            self.events_file.close()
            self.events_file = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[JointStatesMotionAnalyzer] = None
    try:
        node = JointStatesMotionAnalyzer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
