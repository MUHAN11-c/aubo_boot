#!/usr/bin/env python3
"""拉花轨迹播放器 — 40 条笛卡尔末端轨迹叠加 + 流畅播放（纯 Python，不依赖 ROS2）。

用法:
    python3 scripts/visualize_latte_trajectory.py
    python3 scripts/visualize_latte_trajectory.py --arm left

键盘:
    空格  播放/暂停
    ← →  逐帧进退
    ↑ ↓  变速
    [ ]  切换 episode
    r     重置
    a     叠加/单条
"""

import argparse
import os
import sys

import matplotlib
matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Micro Hei",
                                          "AR PL UMing CN", "SimHei", "sans-serif"]
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.widgets import Button
import numpy as np

_script_dir = os.path.dirname(os.path.abspath(__file__))
_pkg_dir = os.path.dirname(_script_dir)
if _pkg_dir not in sys.path:
    sys.path.insert(0, _pkg_dir)

from latte_imitation.trajectory import CartesianTrajectory  # noqa: E402

PHASES_RATIO = [
    (0.00, 0.25, "进杯", "#e74c3c"),
    (0.25, 0.50, "调整", "#f39c12"),
    (0.50, 0.75, "拉花", "#2ecc71"),
    (0.75, 1.00, "退杯", "#3498db"),
]
EPISODE_COLORS = plt.cm.tab20.colors + plt.cm.tab20b.colors + plt.cm.Set3.colors[:2]


class LattePlayer:
    def __init__(self, carts, arm_label):
        """
        carts: OrderedDict[int, CartesianTrajectory]
        """
        self.carts = carts
        self.ep_ids = list(carts.keys())
        self.label = arm_label
        self.idx = 0
        self.playing = True
        self.speed = 1.0
        self.show_all = True
        self.current_ep = 0

        self._build_ui()
        self._draw_all_trajectories_2d()
        self._draw_phase_lines_2d()
        self._draw_static_overlay()
        self._draw_dynamic()
        self._refresh_3d()
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    @property
    def positions(self):
        return self.carts[self.ep_ids[self.current_ep]].positions

    @property
    def cart(self):
        return self.carts[self.ep_ids[self.current_ep]]

    def _phases(self):
        """根据当前轨迹的帧数计算四阶段的帧索引。"""
        n = len(self.positions)
        return [(int(s * n), int(e * n), name, color)
                for s, e, name, color in PHASES_RATIO]

    def _phase_name(self, idx):
        for s, e, name, _ in self._phases():
            if s <= idx < e:
                return name
        return "?"

    # ── UI ───────────────────────────────────────────────────
    def _build_ui(self):
        self.fig = plt.figure(figsize=(18, 10))
        self.fig.canvas.manager.set_window_title(
            f"拉花轨迹 — {self.label} ({len(self.ep_ids)}条)")

        gs = self.fig.add_gridspec(2, 3, height_ratios=[5, 1],
                                   left=0.05, right=0.98, top=0.93, bottom=0.06,
                                   hspace=0.25, wspace=0.25)
        self.ax3d = self.fig.add_subplot(gs[0, 0], projection="3d")
        self.ax_xy = self.fig.add_subplot(gs[0, 1])
        self.ax_xz = self.fig.add_subplot(gs[0, 2])
        self.ax_bar = self.fig.add_subplot(gs[1, :])

        self.btn_ax = self.fig.add_axes([0.02, 0.01, 0.04, 0.03])
        self.btn = Button(self.btn_ax, "⏯")
        self.btn.on_clicked(self._toggle_play)

        self.speed_ax = self.fig.add_axes([0.08, 0.01, 0.10, 0.03])
        self.speed_ax.set_xticks([]); self.speed_ax.set_yticks([])
        self.speed_text = self.speed_ax.text(0.5, 0.5, "Speed: 1.0x",
                                              transform=self.speed_ax.transAxes,
                                              ha="center", va="center", fontsize=9)
        self.info_text = self.fig.text(0.5, 0.97, "", transform=self.fig.transFigure,
                                        ha="center", fontsize=12, fontweight="bold")

    # ── 2D 轨迹 ─────────────────────────────────────────────
    def _draw_all_trajectories_2d(self):
        self.all_xy = {}
        self.all_xz = {}
        for i, ep_id in enumerate(self.ep_ids):
            pos = self.carts[ep_id].positions
            c = EPISODE_COLORS[i % len(EPISODE_COLORS)]
            alpha = 0.8 if i == 0 else (0.15 if self.show_all else 0.0)
            (self.all_xy[ep_id],) = self.ax_xy.plot(
                pos[:, 0], pos[:, 1], color=c, linewidth=0.4, alpha=alpha, zorder=2)
            (self.all_xz[ep_id],) = self.ax_xz.plot(
                pos[:, 0], pos[:, 2], color=c, linewidth=0.4, alpha=alpha, zorder=2)

    def _draw_phase_lines_2d(self):
        self.phase_xy, self.phase_xz = [], []
        pos = self.positions
        for s, e, _, c in self._phases():
            if s >= e:
                continue
            (lxy,) = self.ax_xy.plot(pos[s:e, 0], pos[s:e, 1], color=c, linewidth=1.8, zorder=5)
            (lxz,) = self.ax_xz.plot(pos[s:e, 0], pos[s:e, 2], color=c, linewidth=1.8, zorder=5)
            self.phase_xy.append(lxy); self.phase_xz.append(lxz)

    def _draw_static_overlay(self):
        pos = self.positions
        self.start_xy = self.ax_xy.scatter(*pos[0, :2], c="green", s=50, marker="o",
                                            zorder=7, edgecolors="white", linewidths=0.5)
        self.end_xy = self.ax_xy.scatter(*pos[-1, :2], c="red", s=50, marker="s",
                                          zorder=7, edgecolors="white", linewidths=0.5)
        self.start_xz = self.ax_xz.scatter(*pos[0, [0, 2]], c="green", s=50, marker="o",
                                            zorder=7, edgecolors="white", linewidths=0.5)
        self.end_xz = self.ax_xz.scatter(*pos[-1, [0, 2]], c="red", s=50, marker="s",
                                          zorder=7, edgecolors="white", linewidths=0.5)
        self.ax_xy.set_xlabel("X (m)"); self.ax_xy.set_ylabel("Y (m)")
        self.ax_xy.set_title("XY Latte Pattern"); self.ax_xy.set_aspect("equal")
        self.ax_xz.set_xlabel("X (m)"); self.ax_xz.set_ylabel("Z (m)")
        self.ax_xz.set_title("XZ"); self.ax_xz.set_aspect("equal")

        t_end = len(self.positions) * self.cart.dt
        self.ax_bar.set_xlim(0, t_end); self.ax_bar.set_ylim(0, 1)
        self.ax_bar.set_xlabel("Time (s)"); self.ax_bar.set_yticks([])
        for s, e, _, c in self._phases():
            self.ax_bar.axvspan(s * self.cart.dt, (e - 1) * self.cart.dt, alpha=0.2, color=c)

    def _draw_dynamic(self):
        p = self.positions
        (self.dot3d,) = self.ax3d.plot([p[0, 0]], [p[0, 1]], [p[0, 2]], "o",
                                        color="cyan", markersize=10, zorder=10,
                                        markeredgecolor="black", markeredgewidth=0.5)
        (self.dot_xy,) = self.ax_xy.plot([p[0, 0]], [p[0, 1]], "o", color="cyan",
                                          markersize=10, zorder=10, markeredgecolor="black", markeredgewidth=0.5)
        (self.dot_xz,) = self.ax_xz.plot([p[0, 0]], [p[0, 2]], "o", color="cyan",
                                          markersize=10, zorder=10, markeredgecolor="black", markeredgewidth=0.5)
        (self.vline,) = self.ax_bar.plot([0, 0], [0, 1], "k", linewidth=2)

    # ── 3D（按需刷新） ──────────────────────────────────────
    def _refresh_3d(self):
        self.ax3d.clear()
        all_p = np.concatenate([c.positions for c in self.carts.values()], axis=0)
        ranges = [all_p[:, i].max() - all_p[:, i].min() for i in range(3)]
        mid = [0.5 * (all_p[:, i].max() + all_p[:, i].min()) for i in range(3)]
        span = max(ranges) * 0.6

        for i, ep_id in enumerate(self.ep_ids):
            pos = self.carts[ep_id].positions
            c = EPISODE_COLORS[i % len(EPISODE_COLORS)]
            alpha = 0.8 if i == self.current_ep else (0.15 if self.show_all else 0.0)
            self.ax3d.plot(pos[:, 0], pos[:, 1], pos[:, 2], color=c,
                           linewidth=0.5 if i == self.current_ep else 0.3,
                           alpha=alpha, zorder=2)

        pos = self.positions
        for s, e, _, c in self._phases():
            if s >= e:
                continue
            self.ax3d.plot(pos[s:e, 0], pos[s:e, 1], pos[s:e, 2],
                           color=c, linewidth=1.8, zorder=5)
        self.ax3d.scatter(*pos[0], c="green", s=80, marker="o", zorder=7,
                          edgecolors="white", linewidths=0.5)
        self.ax3d.scatter(*pos[-1], c="red", s=80, marker="s", zorder=7,
                          edgecolors="white", linewidths=0.5)
        # 重新创建 3D 球（ax3d.clear() 会移除旧的）
        (self.dot3d,) = self.ax3d.plot(
            [pos[self.idx, 0]], [pos[self.idx, 1]], [pos[self.idx, 2]],
            "o", color="cyan", markersize=10, zorder=10,
            markeredgecolor="black", markeredgewidth=0.5)

        self.ax3d.set_xlim(mid[0] - span, mid[0] + span)
        self.ax3d.set_ylim(mid[1] - span, mid[1] + span)
        self.ax3d.set_zlim(mid[2] - span, mid[2] + span)
        self.ax3d.set_xlabel("X"); self.ax3d.set_ylabel("Y"); self.ax3d.set_zlabel("Z")
        self.ax3d.set_title(f"3D EE Trajectory  Ep{self.ep_ids[self.current_ep]}")

    # ── 切换 episode ────────────────────────────────────────
    def _switch_episode(self, new_idx):
        self.current_ep = new_idx
        for i, eid in enumerate(self.ep_ids):
            alpha = 0.8 if i == new_idx else (0.15 if self.show_all else 0.0)
            self.all_xy[eid].set_alpha(alpha)
            self.all_xz[eid].set_alpha(alpha)

        pos = self.positions
        for i, (s, e, _, c) in enumerate(self._phases()):
            if s >= e or i >= len(self.phase_xy):
                continue
            seg = pos[s:e]
            self.phase_xy[i].set_data(seg[:, 0], seg[:, 1]); self.phase_xy[i].set_color(c)
            self.phase_xz[i].set_data(seg[:, 0], seg[:, 2]); self.phase_xz[i].set_color(c)

        self.start_xy.set_offsets(pos[0, :2]); self.end_xy.set_offsets(pos[-1, :2])
        self.start_xz.set_offsets(pos[0, [0, 2]]); self.end_xz.set_offsets(pos[-1, [0, 2]])
        self.idx = 0
        self._update_2d_dots(0)
        self._refresh_3d()
        self.fig.canvas.draw_idle()

    def _toggle_all_overlay(self):
        self.show_all = not self.show_all
        for i, eid in enumerate(self.ep_ids):
            alpha = 0.8 if i == self.current_ep else (0.15 if self.show_all else 0.0)
            self.all_xy[eid].set_alpha(alpha)
            self.all_xz[eid].set_alpha(alpha)
        self._refresh_3d()
        self.fig.canvas.draw_idle()

    # ── 帧更新 ─────────────────────────────────────────────
    def _update_info(self):
        ep_id = self.ep_ids[self.current_ep]
        t = self.idx * self.cart.dt
        state = ">" if self.playing else "||"
        p = self.positions[self.idx]
        self.info_text.set_text(
            f"{state} Ep{ep_id}({self.current_ep+1}/{len(self.ep_ids)})  "
            f"F{self.idx+1}/{len(self.positions)}  t={t:.2f}s  "
            f"{self._phase_name(self.idx)}  x{self.speed:.1f}  "
            f"({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})  overlay:{'on' if self.show_all else 'off'}"
        )

    def _update_2d_dots(self, frame_idx):
        self.idx = frame_idx % len(self.positions)
        p = self.positions[self.idx]
        t = self.idx * self.cart.dt
        self.dot3d.set_data([p[0]], [p[1]])
        self.dot3d.set_3d_properties([p[2]])
        self.dot_xy.set_data([p[0]], [p[1]])
        self.dot_xz.set_data([p[0]], [p[2]])
        self.vline.set_data([t, t], [0, 1])
        self._update_info()

    def _animate_step(self, _frame):
        if self.playing:
            skip = max(1, round(self.speed))
            self.idx = (self.idx + skip) % len(self.positions)
        self._update_2d_dots(self.idx)

    # ── 键盘 ────────────────────────────────────────────────
    def _on_key(self, event):
        if event.key == " ":
            self._toggle_play(None)
        elif event.key == "right":
            self.playing = False
            self.idx = min(self.idx + 1, len(self.positions) - 1)
            self._update_2d_dots(self.idx); self.fig.canvas.draw_idle()
        elif event.key == "left":
            self.playing = False
            self.idx = max(self.idx - 1, 0)
            self._update_2d_dots(self.idx); self.fig.canvas.draw_idle()
        elif event.key == "up":
            self.speed = min(10.0, self.speed + 0.5)
            self.speed_text.set_text(f"Speed: {self.speed:.1f}x")
            self.fig.canvas.draw_idle()
        elif event.key == "down":
            self.speed = max(0.5, self.speed - 0.5)
            self.speed_text.set_text(f"Speed: {self.speed:.1f}x")
            self.fig.canvas.draw_idle()
        elif event.key == "r":
            self.idx = 0; self._update_2d_dots(0); self.fig.canvas.draw_idle()
        elif event.key == "]":
            if self.current_ep + 1 < len(self.ep_ids):
                self._switch_episode(self.current_ep + 1)
        elif event.key == "[":
            if self.current_ep > 0:
                self._switch_episode(self.current_ep - 1)
        elif event.key == "a":
            self._toggle_all_overlay()

    def _toggle_play(self, _event):
        self.playing = not self.playing
        if not self.playing:
            self._refresh_3d(); self.fig.canvas.draw_idle()
        self._update_info()

    def run(self):
        interval = int(1000 * self.cart.dt)
        self._ani = anim.FuncAnimation(self.fig, self._animate_step, interval=interval,
                                        cache_frame_data=False)
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Latte Trajectory Player")
    parser.add_argument("--arm", choices=["left", "right"], default="right")
    parser.add_argument("--speed", type=float, default=1.0)
    args = parser.parse_args()

    res_dir = os.path.join(_pkg_dir, "resource")
    carts = CartesianTrajectory.load_all(res_dir, args.arm)
    if not carts:
        print(f"错误: 未找到 {args.arm} arm 的轨迹数据")
        sys.exit(1)
    print(f"Loaded {len(carts)} episodes ({args.arm} arm)")

    label = {"left": "Left (cup)", "right": "Right (latte)"}[args.arm]
    player = LattePlayer(carts, label)
    player.speed = args.speed
    player.speed_text.set_text(f"Speed: {args.speed:.1f}x")
    print("Controls: [ ] episode  Space play/pause  <- -> frame  up/down speed  a overlay")
    player.run()


if __name__ == "__main__":
    main()
