#!/usr/bin/env python3
"""AUBO E5 运动测试分析工具（单文件、单窗口）。

两种模式，一次运行只弹一个图文窗口（曲线 + 文字分析同屏，同时存 PNG）：

  run  脚本轨迹主动测试（原 passthrough_trace_analyzer）：
       发内置轨迹给 passthrough 控制器，录制 /joint_states 与 RIB 水位，
       分析墙钟/标称时长比、终点误差、RIB、joint_states 频率。
       曲线：位置(实际 vs 标称)、速度、逐关节跟踪误差、RIB 水位。

  rec  RViz 手动运动被动录制（原 rviz_motion_analyzer）：
       只被动录制，运动由 RViz2 Plan & Execute（或任何途径）指定，
       自动按速度阈值切段，Ctrl-C 后所有段汇总进同一个图文窗口，
       逐段按 A 准确性/B 平稳性/C 平滑性/D 实时性 量化
       （指标定义见 docs/ur_motion_evaluation_standards.md）。

用法:
  python3 tools/motion_analyzer.py run <轨迹名> [out_prefix] [--real] [--no-gui]
      轨迹名同 passthrough_traj_client.py（wave_shoulder / wave_all / sine_shoulder）
      --real  真机模式：RIB 按执行期 >0 且 <400 判 PASS/FAIL；
              默认 sim 只记 INFO（sim 无发送侧水位流控，瞬时入队超 400 属预期）
  python3 tools/motion_analyzer.py rec [out_prefix] [--no-gui]
      [--goal-tol 0.01] [--path-tol 0.2] [--goal-time-tol 1.0]
      [--sparc-min -1.6] [--stop-vel-tol 0.2] [--drift-tol 0.01]

out_prefix 省略时默认存到项目内 test_results/<时间戳>_<轨迹名|rec>；
显式给定则按给定路径（父目录自动创建）。

输出:
  <out_prefix>_report.png             图文分析报告
  <out_prefix>_joints.csv _rib.csv    原始录制数据（run）
  <out_prefix>_seg<N>_joints.csv _seg<N>_rib.csv（rec 逐段 CSV，汇总图一张 PNG）
依赖: rclpy + numpy + matplotlib（moveit_msgs 可选，缺失时 rec 禁用标称轨迹对照）。
"""
import argparse
import csv
import math
import os
import sys
import textwrap
import time
from collections import deque

import numpy as np

import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")  # 无显示环境：只存 PNG 不弹窗
import matplotlib.pyplot as plt  # noqa: E402

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402

from action_msgs.msg import GoalStatus  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Int32MultiArray  # noqa: E402
from trajectory_msgs.msg import JointTrajectory  # noqa: E402

try:
    from moveit_msgs.msg import DisplayTrajectory
except ImportError:
    # 无 moveit_msgs 的环境（未装 MoveIt）仍可运行，只是 rec 禁用标称轨迹对照
    DisplayTrajectory = None

sys.dont_write_bytecode = True
from passthrough_traj_client import BUILDERS, JOINTS, ACTION_NAME  # noqa: E402

# xacro 蓝本限值（前 3 关节大臂，后 3 关节腕部）
VEL_LIMITS = [2.596177] * 3 + [3.110177] * 3      # rad/s
ACC_LIMITS = [17.30878] * 3 + [20.73676] * 3      # rad/s^2

MOVE_TH = 0.01        # 段切分速度阈值 rad/s
QUIET_HOLD = 0.5      # 低于阈值持续该时长记段结束 s
PLAN_WINDOW = 2.0     # 标称轨迹有效窗口（段开始前）s
SETTLE_BAND_FLOOR = 0.001  # 稳定时间 ±2% 带的下限 rad

# 分析数据默认保存到项目内（工作区根/test_results/），运行时自动创建
RESULTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "test_results")


def resolve_prefix(prefix, tag):
    """out_prefix 缺省 -> test_results/<时间戳>_<tag>；显式给定则确保父目录存在。"""
    if not prefix:
        prefix = os.path.join(RESULTS_DIR, f"{time.strftime('%Y%m%d_%H%M%S')}_{tag}")
    parent = os.path.dirname(os.path.abspath(prefix))
    if parent:
        os.makedirs(parent, exist_ok=True)
    print(f"输出前缀: {prefix}")
    return prefix


# =====================================================================
# 通用小函数
# =====================================================================

def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def trapz(y, x):
    """梯形积分（手写，避开 numpy 2.x 的 trapz/trapezoid 改名问题）。"""
    return float(0.5 * np.sum((y[:-1] + y[1:]) * np.diff(x)))


def zero_phase_smooth(v, dt, win_sec=0.05):
    """固定 50ms 零相位滑动平均（正反向各一次，filtfilt 的均值版）。"""
    win = max(1, int(round(win_sec / dt)))
    win += (win + 1) % 2  # 取奇数
    if win < 3 or len(v) <= win:
        return v
    kernel = np.ones(win) / win
    y = np.convolve(v, kernel, mode="same")
    y = np.convolve(y[::-1], kernel, mode="same")[::-1]
    return y


def sparc(v, t):
    """SPARC 频谱弧长（Balasubramanian 2015）。越接近 0 越平滑，>= -1.6 判平滑。"""
    n = len(v)
    if n < 8:
        return float("nan")
    dt = float(np.median(np.diff(t)))
    V = np.abs(np.fft.rfft(v))
    if V[0] <= 0:
        return float("nan")
    Vn = V / V[0]
    w = 2.0 * math.pi * np.fft.rfftfreq(n, d=dt)
    # 自适应截止：V̂ 其后恒低于 0.05 的最小 ω，再与 20Hz 取小
    wc = 2.0 * math.pi * 20.0
    idx = np.where(Vn >= 0.05)[0]
    if len(idx) > 1:
        kc = idx[-1] + 1
        wc_adapt = w[kc] if kc < len(w) else w[-1]
        wc = min(wc, wc_adapt)
    if wc <= 0:
        return float("nan")
    k = max(int(np.searchsorted(w, wc, side="right")), 2)
    ws, Vs = w[:k], Vn[:k]
    dV = np.gradient(Vs, ws)
    sal = -trapz(np.sqrt((1.0 / wc) ** 2 + dV ** 2), ws)
    return float(sal)


def ldlj(v, t):
    """LDLJ 对数无量纲 jerk（速度剖面，计算前固定 50ms 零相位滑动平均）。

    DLJ = -(t2-t1)^5 / v_peak^2 * ∫(d²v/dt²)²dt，LDLJ = -ln|DLJ|。参考 ≈ -6。
    """
    n = len(v)
    if n < 8:
        return float("nan")
    dt = float(np.median(np.diff(t)))
    T = float(t[-1] - t[0])
    y = zero_phase_smooth(v, dt)
    v_peak = float(np.max(np.abs(y)))
    if T <= 0 or v_peak <= 1e-9:
        return float("nan")
    acc = np.gradient(np.gradient(y, t), t)
    dlj = -(T ** 5 / v_peak ** 2) * trapz(acc ** 2, t)
    if dlj == 0:
        return float("nan")
    return float(-math.log(abs(dlj)))


def pf(ok):
    return "PASS" if ok else "FAIL"


# =====================================================================
# 单窗口图文渲染（曲线在左、文字报告在右，同图存 PNG）
# =====================================================================

_FONT_SET = False
_LINE_COLORS = {
    "[PASS]": "#1a7f37",
    "[FAIL]": "#c81e1e",
    "[SKIP]": "#b25e09",
    "[INFO]": "#555555",
}
_HEAD_PREFIXES = ("===", "---")


def _setup_cjk_font():
    """配置中文字体，避免报告文字渲染成方块。"""
    global _FONT_SET
    if _FONT_SET:
        return
    from matplotlib import font_manager
    names = {f.name for f in font_manager.fontManager.ttflist}
    for cand in ("Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei",
                 "WenQuanYi Micro Hei", "AR PL UMing CN", "SimHei"):
        if cand in names:
            plt.rcParams["font.sans-serif"] = [cand, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False
    _FONT_SET = True


def gui_available():
    return bool(os.environ.get("DISPLAY"))


def _wrap_lines(lines, width):
    out = []
    for ln in lines:
        if not ln:
            out.append("")
            continue
        wrapped = textwrap.wrap(ln, width=width, break_long_words=True,
                                break_on_hyphens=False)
        out.extend(wrapped or [""])
    return out


def show_report(title, plots, lines, png_path=None, block=True):
    """渲染单窗口图文报告。

    plots: [{"title", "ylabel", "series": [{"t","y","label","dashed"}],
             "hlines": [(y,label)], "vlines": [(x,label)]}]
    lines: 文字报告行；[PASS]/[FAIL]/[INFO]/[SKIP]/=== 开头自动着色/加粗。
    无显示环境时只存 PNG 不弹窗。返回是否弹出了交互窗口。
    """
    _setup_cjk_font()
    n = max(len(plots), 1)
    fig = plt.figure(figsize=(17, 9.5))
    gs = fig.add_gridspec(n, 2, width_ratios=[2.5, 1.2], wspace=0.18,
                          hspace=0.45, left=0.05, right=0.97, top=0.93,
                          bottom=0.05)

    for i, spec in enumerate(plots):
        ax = fig.add_subplot(gs[i, 0])
        for s in spec["series"]:
            ax.plot(s["t"], s["y"],
                    linestyle="--" if s.get("dashed") else "-",
                    linewidth=1.0 if s.get("dashed") else 1.3,
                    label=s.get("label"))
        for y, lbl in spec.get("hlines", []):
            ax.axhline(y, color="red", linewidth=0.9, linestyle=":", label=lbl)
        for x, lbl in spec.get("vlines", []):
            ax.axvline(x, color="gray", linewidth=0.9, linestyle=":", label=lbl)
        ax.set_title(spec["title"], fontsize=10, loc="left")
        if spec.get("ylabel"):
            ax.set_ylabel(spec["ylabel"], fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        if any(s.get("label") for s in spec["series"]) or spec.get("hlines"):
            ax.legend(fontsize=7, ncol=3, loc="upper right")
        ax.margins(x=0)

    axt = fig.add_subplot(gs[:, 1])
    axt.axis("off")
    wrapped = _wrap_lines(lines, 44)
    max_lines = 96
    if len(wrapped) > max_lines:
        wrapped = wrapped[:max_lines - 1] + [
            f"…（其余 {len(wrapped) - max_lines + 1} 行略，完整见终端输出）"]
    fontsize = 9.0 if len(wrapped) <= 40 else (
        8.0 if len(wrapped) <= 56 else (7.0 if len(wrapped) <= 76 else 6.0))
    dy = 0.97 / max(len(wrapped), 1)
    y = 0.995
    for ln in wrapped:
        color, weight = "#111111", "normal"
        for prefix, c in _LINE_COLORS.items():
            if ln.startswith(prefix):
                color = c
                weight = "bold" if prefix in ("[PASS]", "[FAIL]") else "normal"
                break
        else:
            if ln.startswith(_HEAD_PREFIXES):
                color, weight = "#111111", "bold"
        axt.text(0.0, y, ln, transform=axt.transAxes, fontsize=fontsize,
                 color=color, fontweight=weight, va="top", ha="left")
        y -= dy

    fig.suptitle(title, fontsize=13, fontweight="bold")
    if png_path:
        fig.savefig(png_path, dpi=140)
        print(f"分析图已写出: {png_path}")
    if block and gui_available():
        plt.show()
        return True
    plt.close(fig)
    return False


# =====================================================================
# run 模式：脚本轨迹主动测试
# =====================================================================

class TrajRunner(Node):
    def __init__(self):
        super().__init__("motion_analyzer_run")
        self._base = None
        self.joint_rows = []
        self.rib_rows = []
        self._js_count = 0
        self._js_t0 = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 100)
        self.create_subscription(
            Int32MultiArray, "/aubo_io_controller/rib_status", self._on_rib, 100)
        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def _on_js(self, msg):
        t = time.time()
        if self._js_t0 is None:
            self._js_t0 = t
        self._js_count += 1
        try:
            idx = [msg.name.index(j) for j in JOINTS]
        except ValueError:
            return
        if self._base is None:
            self._base = [msg.position[i] for i in idx]
        pos = [msg.position[i] for i in idx]
        vel = [msg.velocity[i] if len(msg.velocity) > i else 0.0 for i in idx]
        self.joint_rows.append((t, pos, vel))

    def _on_rib(self, msg):
        if len(msg.data) > 0:
            self.rib_rows.append((time.time(), msg.data[0]))

    def wait_ready(self, timeout=5.0):
        end = time.time() + timeout
        while self._base is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._base is None:
            raise RuntimeError("未收到 /joint_states")

    def execute(self, traj_name):
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        traj.points = BUILDERS[traj_name](self._base)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"action 服务器不可用: {ACTION_NAME}")
        # 清空录制缓存，只留本次执行窗口
        self.joint_rows.clear()
        self.rib_rows.clear()
        self._js_count = 0
        self._js_t0 = None
        t0 = time.time()
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().accepted:
            raise RuntimeError("goal 被拒绝")
        rf = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        wrapped = rf.result()
        wall = time.time() - t0
        # 再录 1s 稳定段，取终点实际位置
        settle_end = time.time() + 1.0
        while time.time() < settle_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return traj, wrapped, wall

    def report(self, traj, wrapped, wall, real_hw=False):
        """生成文字分析行（同时打印到终端），供终端与图文窗口共用。"""
        expect = stamp_to_sec(traj.points[-1].time_from_start)
        goal_pos = traj.points[-1].positions
        final_pos = self.joint_rows[-1][1] if self.joint_rows else self._base
        errs = [abs(g - a) for g, a in zip(goal_pos, final_pos)]
        # RIB 只统计执行窗口（goal 发出 → 结果返回），不含尾部的 1s 静止段
        exec_end = (self.joint_rows[0][0] + wall) if self.joint_rows else None
        ribs = [r for ts, r in self.rib_rows if exec_end is None or ts <= exec_end]
        js_rate = self._js_count / max(self.joint_rows[-1][0] - self.joint_rows[0][0], 1e-6) \
            if len(self.joint_rows) > 1 else 0.0
        status = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                  GoalStatus.STATUS_CANCELED: "CANCELED",
                  GoalStatus.STATUS_ABORTED: "ABORTED"}.get(wrapped.status, str(wrapped.status))
        lines = [
            "=== 执行分析 ===",
            f"[{pf(wrapped.status == GoalStatus.STATUS_SUCCEEDED)}] "
            f"结果: {status} (error_code={wrapped.result.error_code}) "
            f"{wrapped.result.error_string}",
            f"[{pf(wall / max(expect, 1e-6) <= 1.2)}] 墙钟时间: "
            f"{wall:.2f}s  轨迹标称: {expect:.2f}s  比值: {wall / max(expect, 1e-6):.2f}",
            f"[{pf(max(errs) <= 0.02)}] 终点误差: "
            f"max={max(errs):.5f} rad  mean={sum(errs) / len(errs):.5f} rad (≤0.02)",
        ]
        if ribs:
            starve = sum(1 for r in ribs if r == 0)
            overflow = sum(1 for r in ribs if r >= 400)
            stat = (f"RIB 水位: min={min(ribs)}  max={max(ribs)}  "
                    f"mean={sum(ribs) / len(ribs):.1f}  饿死(=0) {starve} 次  "
                    f"溢出(≥400) {overflow} 次")
            if real_hw:
                lines.append(f"[{pf(starve == 0 and overflow == 0)}] "
                             f"{stat}（真机执行期应 >0 且 <400）")
            else:
                lines.append(f"[INFO] {stat}（sim 无发送侧水位流控，瞬时入队超 400 "
                             f"属预期；真机加 --real 按 >0 且 <400 判定）")
        else:
            lines.append("[INFO] RIB 水位: 执行窗口内无样本")
        lines.append(
            f"[{pf(js_rate >= 190)}] joint_states 频率: "
            f"{js_rate:.1f} Hz（{self._js_count} 帧，标称 200Hz）")
        for ln in lines:
            print(ln)
        return lines

    def build_plots(self, traj):
        """组装曲线子图规格：位置跟踪（实际 vs 标称）、速度、逐关节跟踪误差、RIB。"""
        if not self.joint_rows:
            return []
        t0 = self.joint_rows[0][0]
        t = np.array([r[0] for r in self.joint_rows]) - t0
        pos = np.array([r[1] for r in self.joint_rows])
        vel = np.array([r[2] for r in self.joint_rows])
        T_nom = np.array([stamp_to_sec(p.time_from_start) for p in traj.points])
        Q_nom = np.array([p.positions for p in traj.points])
        pos_series, err_series = [], []
        for j in range(6):
            pos_series.append({"t": t, "y": pos[:, j], "label": JOINTS[j]})
            pos_series.append({"t": T_nom, "y": Q_nom[:, j],
                               "label": f"{JOINTS[j]} 标称", "dashed": True})
            nom_at_t = np.interp(np.minimum(t, T_nom[-1]), T_nom, Q_nom[:, j])
            err_series.append({"t": t, "y": np.abs(pos[:, j] - nom_at_t),
                               "label": JOINTS[j]})
        plots = [
            {"title": "关节位置：实际(实线) vs 标称轨迹(虚线)", "ylabel": "rad",
             "series": pos_series},
            {"title": "关节速度（实际）", "ylabel": "rad/s",
             "series": [{"t": t, "y": vel[:, j], "label": JOINTS[j]}
                        for j in range(6)]},
            {"title": "逐关节跟踪误差 |实际 − 标称|", "ylabel": "rad",
             "series": err_series,
             "hlines": [(0.02, "goal 容差 0.02")]},
        ]
        if self.rib_rows:
            rib_t = np.array([r[0] for r in self.rib_rows]) - t0
            rib_v = np.array([r[1] for r in self.rib_rows], dtype=float)
            plots.append({
                "title": "RIB 水位（真机流控判据：执行期 >0 且 <400）", "ylabel": "点",
                "series": [{"t": rib_t, "y": rib_v, "label": "rib_level"}],
                "hlines": [(0, "饿死线 0"), (400, "溢出线 400")]})
        return plots

    def write_csv(self, prefix):
        with open(f"{prefix}_joints.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t"] + [f"{j}_pos" for j in JOINTS] + [f"{j}_vel" for j in JOINTS])
            for t, pos, vel in self.joint_rows:
                w.writerow([f"{t:.3f}"] + [f"{v:.6f}" for v in pos] + [f"{v:.6f}" for v in vel])
        with open(f"{prefix}_rib.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "rib_level"])
            for t, r in self.rib_rows:
                w.writerow([f"{t:.3f}", r])
        print(f"CSV 已写出: {prefix}_joints.csv, {prefix}_rib.csv")


# =====================================================================
# rec 模式：RViz 手动运动被动录制与逐段分析
# =====================================================================

class Segment:
    """一段运动：rows=[(t_stamp, pos[6], vel[6])]，含到达后 0.5s 静止尾。"""

    def __init__(self, t_start_wall, start_pos):
        self.rows = []
        self.t_start_wall = t_start_wall  # time.time()，用于标称轨迹 2s 窗口
        self.start_pos = start_pos        # 段开始前静止基线位置（6 关节）
        self.nominal = None               # JointTrajectory 或 None
        self.t_last_move = None           # 最后一次 ||q̇||>=阈值的 header 时刻

    @property
    def t0(self):
        return self.rows[0][0]

    @property
    def t_arrive(self):
        return self.t_last_move if self.t_last_move is not None else self.rows[-1][0]

    def arrays(self):
        t = np.array([r[0] for r in self.rows]) - self.t0
        pos = np.array([r[1] for r in self.rows])
        vel = np.array([r[2] for r in self.rows])
        return t, pos, vel


class Recorder(Node):
    def __init__(self):
        super().__init__("motion_analyzer_rec")
        self.segments = []
        self._seg = None                  # 进行中的段
        self._pre_pos = deque(maxlen=40)  # 空闲期位置环形缓冲（~0.2s，作段基线）
        self._latest_plan = None          # (recv_wall_time, JointTrajectory)
        self.rib_rows = []                # 全程 RIB（t_stamp, rib_level）
        self._rib_seen = False
        self._js_count = 0
        self.create_subscription(JointState, "/joint_states", self._on_js, 200)
        if DisplayTrajectory is not None:
            self.create_subscription(
                DisplayTrajectory, "/display_planned_path", self._on_plan, 10)
        else:
            self.get_logger().warn(
                "moveit_msgs 不可用（MoveIt 未安装/未 source），标称轨迹对照已禁用")
        self.create_subscription(
            Int32MultiArray, "/aubo_io_controller/rib_status", self._on_rib, 100)

    def _on_plan(self, msg):
        for robot_traj in msg.trajectory:
            jt = robot_traj.joint_trajectory
            if len(jt.joint_names) > 0 and len(jt.points) > 0:
                self._latest_plan = (time.time(), jt)
                self.get_logger().info(
                    f"捕获标称轨迹: {len(jt.points)} 路点, "
                    f"时长 {stamp_to_sec(jt.points[-1].time_from_start):.2f}s")
                return

    def _on_js(self, msg):
        t = stamp_to_sec(msg.header.stamp)
        self._js_count += 1
        try:
            idx = [msg.name.index(j) for j in JOINTS]
        except ValueError:
            return
        pos = [msg.position[i] for i in idx]
        vel = [msg.velocity[i] if len(msg.velocity) > i else 0.0 for i in idx]
        speed = math.sqrt(sum(v * v for v in vel))
        moving = speed > MOVE_TH
        if self._seg is None:
            if moving:
                start_pos = (np.median(np.array(self._pre_pos), axis=0).tolist()
                             if self._pre_pos else list(pos))
                self._seg = Segment(time.time(), start_pos)
                # 段开始前 2s 内最后一条标称轨迹
                if (self._latest_plan is not None
                        and self._seg.t_start_wall - self._latest_plan[0] <= PLAN_WINDOW):
                    self._seg.nominal = self._latest_plan[1]
                self._seg.rows.append((t, pos, vel))
                self._seg.t_last_move = t
                self.get_logger().info(
                    f"段 {len(self.segments) + 1} 开始"
                    f"（{'有' if self._seg.nominal is not None else '无'}标称轨迹）")
                self._pre_pos.clear()
            else:
                self._pre_pos.append(pos)
            return
        self._seg.rows.append((t, pos, vel))
        if moving:
            self._seg.t_last_move = t
        elif t - self._seg.t_last_move > QUIET_HOLD:
            self.segments.append(self._seg)
            self.get_logger().info(
                f"段 {len(self.segments)} 结束"
                f"（{len(self._seg.rows)} 样本，"
                f"运动 {self._seg.t_arrive - self._seg.t0:.2f}s）")
            self._seg = None

    def _on_rib(self, msg):
        if len(msg.data) > 0:
            self._rib_seen = True
            self.rib_rows.append((self.get_clock().now().nanoseconds * 1e-9, msg.data[0]))

    def close_open_segment(self):
        """Ctrl-C 时把进行中的段也并入汇总。"""
        if self._seg is not None and len(self._seg.rows) > 8:
            self.segments.append(self._seg)
        self._seg = None


def nominal_arrays(traj):
    """标称轨迹 -> (T, Q) 数组，关节序映射到 JOINTS。"""
    try:
        idx = [traj.joint_names.index(j) for j in JOINTS]
    except ValueError:
        return None, None
    T = np.array([stamp_to_sec(p.time_from_start) for p in traj.points])
    Q = np.array([[p.positions[i] for i in idx] for p in traj.points])
    return T, Q


def analyze_segment(seg, rib_rows, rib_seen, args, seg_no, prefix):
    """逐段分析：终端打印 + 写 CSV，返回 (文字报告行, 绘图数据 dict)。"""
    lines = []

    def emit(s=""):
        print(s)
        lines.extend(s.split("\n"))

    t, pos, vel = seg.arrays()
    t_arr = seg.t_arrive - seg.t0            # 运动段时长（墙钟，不含静止尾）
    moving = t <= (t_arr + 1e-9)
    tail = t >= (t_arr - 1e-9)               # 到达后窗口
    vmag = np.sqrt(np.sum(vel ** 2, axis=1))
    T_nom, Q_nom = nominal_arrays(seg.nominal) if seg.nominal is not None else (None, None)
    has_nom = T_nom is not None and len(T_nom) >= 1
    nom_dur = float(T_nom[-1]) if has_nom else None
    dev_t = dev = None                       # 路径偏差曲线数据（有标称轨迹时）

    emit(f"\n===== 段 {seg_no} =====  样本 {len(t)}  运动时长 {t_arr:.2f}s"
         + (f"  标称时长 {nom_dur:.2f}s" if has_nom else "  无标称轨迹"))

    # ---------- A 准确性 ----------
    emit("--- A 准确性（UR 驱动标准）---")
    final_pos = np.median(pos[t >= max(t[-1] - 0.3, 0.0)], axis=0)
    if has_nom and len(T_nom) >= 2:
        goal_err = np.abs(final_pos - Q_nom[-1])
        emit(f"[{pf(goal_err.max() <= args.goal_tol)}] 终点逐关节误差 "
             f"max={goal_err.max():.4f} rad (≤{args.goal_tol})")
        # 路径偏差：标称按 time_from_start 线性插值到各采样时刻（超出取末点）
        tau = np.minimum(t[moving], nom_dur)
        dev_t = t[moving]
        dev = np.abs(pos[moving] - np.column_stack(
            [np.interp(tau, T_nom, Q_nom[:, j]) for j in range(6)]))
        dev_max = float(dev.max())
        dev_rms = float(np.sqrt(np.mean(dev ** 2)))
        emit(f"[{pf(dev_max <= args.path_tol)}] 路径偏差 "
             f"max={dev_max:.4f}  RMS={dev_rms:.4f} rad (≤{args.path_tol})")
        dt_err = abs(t_arr - nom_dur)
        emit(f"[{pf(dt_err <= args.goal_time_tol)}] 执行时长 "
             f"墙钟={t_arr:.2f}s 标称={nom_dur:.2f}s "
             f"|Δ|={dt_err:.2f}s (≤{args.goal_time_tol})")
        moveit_rule = t_arr <= 1.1 * nom_dur + 0.5
        emit(f"[{pf(moveit_rule)}] MoveIt 默认规则 ≤1.1×标称+0.5s "
             f"（上限 {1.1 * nom_dur + 0.5:.2f}s；本项目已放宽为 5.0×/10s）")
    else:
        if not has_nom:
            emit("[SKIP] 终点误差/路径偏差/执行时长/MoveIt 规则: 无标称轨迹")
        else:
            emit("[SKIP] 标称轨迹路点不足，无法对照")
    stop_vel = np.abs(vel[t >= max(t[-1] - 0.5, 0.0)]).max()
    emit(f"[{pf(stop_vel <= args.stop_vel_tol)}] 停止速度（末 0.5s |vel|max）"
         f" {stop_vel:.4f} rad/s (≤{args.stop_vel_tol})")
    vel_max = np.abs(vel[moving]).max(axis=0) if moving.any() else np.zeros(6)
    vel_ok = all(vel_max[j] <= VEL_LIMITS[j] for j in range(6))
    emit(f"[{pf(vel_ok)}] 速度限值 |vel|max="
         f"{np.array2string(vel_max, precision=3, separator=',')} rad/s")
    # 加速度：上报速度差分
    if len(t) > 2:
        acc = np.diff(vel, axis=0) / np.diff(t)[:, None]
        acc_max = np.abs(acc[moving[:-1] | moving[1:]]).max(axis=0) \
            if moving.any() else np.zeros(6)
    else:
        acc_max = np.zeros(6)
    acc_ok = all(acc_max[j] <= ACC_LIMITS[j] for j in range(6))
    emit(f"[{pf(acc_ok)}] 加速度限值（速度差分）|acc|max="
         f"{np.array2string(acc_max, precision=1, separator=',')} rad/s²")

    # ---------- B 平稳性 ----------
    emit("--- B 平稳性（ISO 9283 关节空间类比，非合规测量）---")
    disp = final_pos - np.array(seg.start_pos)
    moved = np.abs(disp) > 1e-3
    if moved.any():
        # 越过最终位置的最大幅度（仅统计有净位移的关节，沿各自位移方向）
        direction = np.sign(disp)
        over = np.maximum((pos - final_pos) * direction, 0.0).max(axis=0)
        j = int(np.argmax(np.where(moved, over, -1.0)))
        po = 100.0 * over[j] / abs(disp[j])
        emit(f"[INFO] 超调量 max={over[j]:.4f} rad  PO={po:.1f}%"
             f"（{JOINTS[j]}，无标准阈值）")
        band = np.maximum(0.02 * np.abs(disp), SETTLE_BAND_FLOOR)
    else:
        emit("[INFO] 超调量/PO: N/A（段首末净位移≈0，如往返轨迹）")
        band = np.full(6, SETTLE_BAND_FLOOR)
    # 稳定时间：从名义到达时刻（有标称）或峰值速度时刻（无标称）起算
    if has_nom:
        t_ref = nom_dur
        ref_desc = f"名义到达（段开始+{nom_dur:.2f}s）"
    else:
        t_ref = float(t[moving][np.argmax(vmag[moving])]) if moving.any() else 0.0
        ref_desc = "峰值速度时刻（无标称轨迹）"
    err = np.abs(pos - final_pos)
    settle = None
    for i in np.where(t >= t_ref)[0]:
        if (err[i:] <= band).all():
            settle = float(t[i] - t_ref)
            break
    if settle is None:
        emit(f"[INFO] 稳定时间 >{t[-1] - t_ref:.2f}s（录制窗口内未稳定进 ±2% 带，"
             f"自{ref_desc}起算）")
    else:
        emit(f"[INFO] 稳定时间 {settle:.3f}s（±2% 带，自{ref_desc}起算）")
    # 到达后振荡次数：各关节速度符号变化（0.01 rad/s 死区）取最大
    osc = 0
    for j in range(6):
        signs = np.sign(vel[tail, j])
        signs = signs[np.abs(vel[tail, j]) > 0.01]
        if len(signs) > 1:
            osc = max(osc, int(np.sum(signs[1:] * signs[:-1] < 0)))
    emit(f"[INFO] 到达后振荡次数 {osc}（速度符号变化，各关节取最大）")
    drift = (pos[t >= max(t[-1] - 0.5, 0.0)].max(axis=0)
             - pos[t >= max(t[-1] - 0.5, 0.0)].min(axis=0)).max()
    emit(f"[{pf(drift <= args.drift_tol)}] 静止漂移（末 0.5s 各关节 max−min）"
         f" {drift:.4f} rad (≤{args.drift_tol})")
    # 振动带 RMS：到达后窗口速度模 FFT，>5Hz 分量 Parseval 折算
    x = vmag[tail]
    if len(x) >= 8:
        X = np.fft.rfft(x)
        freqs = np.fft.rfftfreq(len(x), d=float(np.median(np.diff(t[tail]))))
        bandmask = freqs > 5.0
        # 单边谱能量：除直流与 Nyquist 外翻倍（本带不含直流）
        e = 2.0 * np.sum(np.abs(X[bandmask]) ** 2)
        vib_rms = math.sqrt(e) / len(x)
        emit(f"[INFO] 振动带 RMS（>5Hz 速度模） {vib_rms:.4f} rad/s（相对对比量）")
    else:
        emit("[INFO] 振动带 RMS: 样本不足")

    # ---------- C 平滑性 ----------
    emit("--- C 平滑性（v=||q̇(t)||；LDLJ 前固定 50ms 零相位滑动平均；"
         "跨段对比不用 RMS jerk）---")
    tm, vm = t[moving], vmag[moving]
    if len(tm) >= 8:
        s = sparc(vm, tm)
        if math.isnan(s):
            emit("[INFO] SPARC: 无法计算")
        else:
            emit(f"[{pf(s >= args.sparc_min)}] SPARC {s:.3f} (≥{args.sparc_min} 判平滑)")
        l = ldlj(vm, tm)
        if math.isnan(l):
            emit("[INFO] LDLJ: 无法计算")
        else:
            emit(f"[INFO] LDLJ {l:.2f}（参考 ≈ -6，仅同配置纵向对比）")
        v_peak = float(vm.max())
        peaks = int(np.sum((vm[1:-1] > vm[:-2]) & (vm[1:-1] >= vm[2:])
                           & (vm[1:-1] > 0.1 * v_peak)))
        emit(f"[INFO] 速度峰数 {peaks}（局部极大且 >0.1×峰值）")
        emit(f"[INFO] mean/max 速度比 {float(vm.mean()) / max(v_peak, 1e-9):.3f}")
    else:
        emit("[INFO] 平滑性: 运动样本不足")

    # ---------- D 实时性 ----------
    emit("--- D 实时性 ---")
    if len(t) > 1:
        gaps = np.diff(t)
        med = float(np.median(gaps))
        late = int(np.sum(gaps > med + 0.001))
        emit(f"[INFO] joint_states 间隔 mean={gaps.mean() * 1e3:.2f}ms "
             f"std={gaps.std() * 1e3:.2f}ms max={gaps.max() * 1e3:.2f}ms "
             f"标称周期(中位数)={med * 1e3:.2f}ms")
        emit(f"[INFO] 超期次数（>中位周期+1ms）{late}/{len(gaps)}  "
             f"实测发布频率 {1.0 / max(gaps.mean(), 1e-9):.1f} Hz")
    if has_nom:
        emit(f"[INFO] 时间膨胀比 {t_arr / max(nom_dur, 1e-9):.3f}（墙钟/标称）")
    else:
        emit("[SKIP] 时间膨胀比: 无标称轨迹")
    seg_rib_pairs = []
    if not rib_seen:
        emit("[SKIP] RIB 水位: /aubo_io_controller/rib_status 无数据，跳过")
    else:
        seg_rib_pairs = [(ts - seg.t0, r) for ts, r in rib_rows
                         if seg.rows[0][0] - 1.0 <= ts <= seg.rows[-1][0] + 1.0]
        seg_rib = [r for _, r in seg_rib_pairs]
        if seg_rib:
            starve = sum(1 for r in seg_rib if r == 0)
            overflow = sum(1 for r in seg_rib if r >= 400)
            emit(f"[INFO] RIB 水位 min={min(seg_rib)} max={max(seg_rib)} "
                 f"mean={sum(seg_rib) / len(seg_rib):.1f}  "
                 f"饿死(=0) {starve} 次  溢出(≥400) {overflow} 次"
                 f"（真机执行期应 >0 且 <400；sim 无发送侧水位流控，超 400 属预期）")
        else:
            emit("[INFO] RIB 水位: 本段时间窗内无样本")

    # ---------- CSV ----------
    joints_csv = f"{prefix}_seg{seg_no}_joints.csv"
    with open(joints_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + [f"{j}_pos" for j in JOINTS] + [f"{j}_vel" for j in JOINTS])
        for ts, p, v in seg.rows:
            w.writerow([f"{ts:.3f}"] + [f"{x:.6f}" for x in p] + [f"{x:.6f}" for x in v])
    rib_csv = f"{prefix}_seg{seg_no}_rib.csv"
    with open(rib_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "rib_level"])
        for ts, r in rib_rows:
            if seg.rows[0][0] - 1.0 <= ts <= seg.rows[-1][0] + 1.0:
                w.writerow([f"{ts:.3f}", r])
    emit(f"CSV 已写出: {joints_csv} ({len(seg.rows)} 行), {rib_csv}")

    data = {"t": t, "pos": pos, "vel": vel, "t_arr": t_arr,
            "T_nom": T_nom if has_nom else None,
            "Q_nom": Q_nom if has_nom else None,
            "dev_t": dev_t, "dev": dev, "rib_pairs": seg_rib_pairs}
    return lines, data


def build_rec_plots(seg_datas, path_tol):
    """把所有段拼到同一时间轴（段间留 1s 间隔），单窗口展示。"""
    pos_series, vel_series, dev_series, rib_series = [], [], [], []
    vlines = []
    offset = 0.0
    for i, d in enumerate(seg_datas, 1):
        t = d["t"] + offset
        for j in range(6):
            lbl = JOINTS[j] if i == 1 else None  # 图例只标第一段的关节名
            pos_series.append({"t": t, "y": d["pos"][:, j], "label": lbl})
            vel_series.append({"t": t, "y": d["vel"][:, j], "label": lbl})
        if d["T_nom"] is not None:
            for j in range(6):
                pos_series.append({"t": d["T_nom"] + offset, "y": d["Q_nom"][:, j],
                                   "label": f"{JOINTS[j]} 标称" if i == 1 else None,
                                   "dashed": True})
        if d["dev"] is not None:
            for j in range(6):
                dev_series.append({"t": d["dev_t"] + offset, "y": d["dev"][:, j],
                                   "label": JOINTS[j] if not dev_series else None})
        if d["rib_pairs"]:
            rib_series.append({
                "t": np.array([p[0] for p in d["rib_pairs"]]) + offset,
                "y": np.array([p[1] for p in d["rib_pairs"]], dtype=float),
                "label": "rib_level" if i == 1 else None})
        vlines.append((offset, f"段{i}"))
        offset += d["t"][-1] + 1.0
    plots = [
        {"title": "关节位置：实际(实线) vs 标称轨迹(虚线)，灰线=段界",
         "ylabel": "rad", "series": pos_series, "vlines": vlines},
        {"title": "关节速度（实际）", "ylabel": "rad/s",
         "series": vel_series, "vlines": vlines},
    ]
    if dev_series:
        plots.append({"title": "逐关节路径偏差 |实际 − 标称|", "ylabel": "rad",
                      "series": dev_series,
                      "hlines": [(path_tol, f"路径容差 {path_tol}")],
                      "vlines": vlines})
    if rib_series:
        plots.append({"title": "RIB 水位（真机流控判据：执行期 >0 且 <400）",
                      "ylabel": "点", "series": rib_series,
                      "hlines": [(0, "饿死线 0"), (400, "溢出线 400")],
                      "vlines": vlines})
    return plots


# =====================================================================
# 入口
# =====================================================================

def cmd_run(args):
    if args.traj not in BUILDERS:
        print(f"未知轨迹 {args.traj}，可选: {', '.join(BUILDERS)}")
        return 1
    prefix = resolve_prefix(args.prefix, args.traj)
    rclpy.init()
    node = TrajRunner()
    node.wait_ready()
    traj, wrapped, wall = node.execute(args.traj)
    lines = node.report(traj, wrapped, wall, real_hw=args.real)
    node.write_csv(prefix)
    show_report(f"passthrough 执行分析 — {args.traj}",
                node.build_plots(traj), lines,
                png_path=f"{prefix}_report.png", block=not args.no_gui)
    ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


def cmd_rec(args):
    args.prefix = resolve_prefix(args.prefix, "rec")
    rclpy.init()
    node = Recorder()
    print(f"录制中… 在 RViz2 拖拽目标 Plan & Execute（可多次），Ctrl-C 汇总。"
          f"段切分: ||q̇||>{MOVE_TH} rad/s 开始，<{MOVE_TH} 持续 {QUIET_HOLD}s 结束。")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.close_open_segment()
    if not node.segments:
        print("\n未录到任何运动段。")
    else:
        print(f"\n共录制 {len(node.segments)} 段运动，逐段分析："
              f"（阈值: goal={args.goal_tol} path={args.path_tol} "
              f"time={args.goal_time_tol}s sparc≥{args.sparc_min}）")
        all_lines, seg_datas = [], []
        for i, seg in enumerate(node.segments, 1):
            lines, data = analyze_segment(
                seg, node.rib_rows, node._rib_seen, args, i, args.prefix)
            all_lines.extend(lines)
            seg_datas.append(data)
        # 所有段汇总进同一个图文窗口
        show_report(f"运动分析 — 共 {len(node.segments)} 段",
                    build_rec_plots(seg_datas, args.path_tol), all_lines,
                    png_path=f"{args.prefix}_report.png", block=not args.no_gui)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="AUBO E5 运动测试分析工具（单文件、单窗口图文报告）")
    sub = ap.add_subparsers(dest="mode", required=True)

    pr = sub.add_parser("run", help="脚本轨迹主动测试")
    pr.add_argument("traj", help=f"轨迹名: {', '.join(BUILDERS)}")
    pr.add_argument("prefix", nargs="?", default=None,
                    help="输出前缀，默认 项目test_results/<时间戳>_<轨迹名>")
    pr.add_argument("--real", action="store_true",
                    help="真机模式：RIB 水位按执行期 >0 且 <400 判 PASS/FAIL")
    pr.add_argument("--no-gui", action="store_true",
                    help="不弹图文窗口（只存 PNG/CSV）")
    pr.set_defaults(func=cmd_run)

    pc = sub.add_parser("rec", help="RViz 手动运动被动录制与逐段分析")
    pc.add_argument("prefix", nargs="?", default=None,
                    help="输出前缀，默认 项目test_results/<时间戳>_rec")
    pc.add_argument("--goal-tol", type=float, default=0.01,
                    help="终点逐关节误差阈值 rad（UR 集成测试严格值）")
    pc.add_argument("--path-tol", type=float, default=0.2,
                    help="路径偏差阈值 rad（UR scaled JTC 默认）")
    pc.add_argument("--goal-time-tol", type=float, default=1.0,
                    help="执行时长偏差阈值 s（UR passthrough 集成测试）")
    pc.add_argument("--sparc-min", type=float, default=-1.6,
                    help="SPARC 平滑判定下限（Balasubramanian 2015）")
    pc.add_argument("--stop-vel-tol", type=float, default=0.2,
                    help="停止速度阈值 rad/s（UR stopped_velocity_tolerance）")
    pc.add_argument("--drift-tol", type=float, default=0.01,
                    help="静止漂移阈值 rad（UR abort 静止漂移判据）")
    pc.add_argument("--no-gui", action="store_true",
                    help="不弹图文窗口（只存 PNG/CSV）")
    pc.set_defaults(func=cmd_rec)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
