#!/usr/bin/env python3
"""
ProMP 条件推理 + 置信区间可视化 — 参考 movement_primitives/plot_conditional_promp.py 喵~

功能:
  - 从多条示教学习 ProMP → 均值轨迹 + 2σ 置信带 (灰色阴影)
  - 交互式滑块调节起点/终点 → 实时更新条件推理轨迹
  - 每条原始示教叠加在背景 (半透明)
  - 3 个子图: XY投影 / XZ投影 / 各维度误差带

使用:
  python3 scripts/vis_promp_conditioning.py
  python3 scripts/vis_promp_conditioning.py --episodes top5
  python3 scripts/vis_promp_conditioning.py --episodes all
"""

import os, sys, argparse
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, PKG_DIR)

from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
from collections import OrderedDict

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QSlider, QCheckBox, QStatusBar,
)
from PyQt5.QtCore import Qt

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.font_manager as fm

# 中文字体
_cn = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if os.path.exists(_cn):
    fm.fontManager.addfont(_cn)
    matplotlib.rcParams['font.family'] = fm.FontProperties(fname=_cn).get_name()
matplotlib.rcParams['axes.unicode_minus'] = False

from latte_imitation.promp_learner import ProMP3D
from latte_imitation.dmp_learner import DMP3D
from latte_imitation.trajectory import CartesianTrajectory

CARTESIAN_DIR = os.path.join(PKG_DIR, "resource", "cartesian", "right")
TOP5 = [32, 25, 23, 6, 8]
DT = 0.05


def extract_forming(pos):
    z = pos[:, 2]
    w = min(21, len(z) // 10 * 2 + 1)
    if w < 3: return pos
    zs = savgol_filter(z, w, 3)
    lo = zs < np.median(zs)
    ch = np.diff(lo.astype(int))
    st = np.where(ch == 1)[0] + 1; en = np.where(ch == -1)[0] + 1
    if lo[0]: st = np.concatenate([[0], st])
    if lo[-1]: en = np.concatenate([en, [len(lo)]])
    if len(st) == 0: return pos
    b = np.argmax(en - st); fs, fe = st[b], en[b]
    if fe - fs < 40: fs, fe = len(pos) // 4, 3 * len(pos) // 4
    return pos[fs:fe]


class PrompConditioningGUI(QMainWindow):
    """ProMP 条件推理交互面板 喵~"""

    def __init__(self, episodes=None):
        super().__init__()
        self.setWindowTitle("ProMP 条件推理 + 置信区间 — 心形轨迹")
        self.setGeometry(50, 50, 1500, 850)

        if episodes is None:
            episodes = TOP5
        self.episodes = episodes
        self.n_basis = 20

        # 加载数据
        self._load_demos()

        # ProMP 学习
        self.promp = ProMP3D(n_basis=self.n_basis, sigma=0.03)
        self.promp.learn_multiple(self.demos)

        # 当前调节参数 (偏移量)
        self.dx0, self.dy0, self.dz0 = 0.0, 0.0, 0.0  # 起点偏移 (mm)
        self.dxg, self.dyg, self.dzg = 0.0, 0.0, 0.0  # 终点偏移 (mm)

        self._setup_ui()
        self._draw()

    def _load_demos(self):
        self.demos = []
        for ep in self.episodes:
            path = os.path.join(CARTESIAN_DIR, f"episode_{ep:06d}.npz")
            if os.path.exists(path):
                cart = CartesianTrajectory.load(path)
                self.demos.append(extract_forming(cart.positions))
        print(f"加载 {len(self.demos)} 条示教: {self.episodes}")

    def _setup_ui(self):
        cw = QWidget(); self.setCentralWidget(cw)
        ml = QVBoxLayout(cw)

        # ── 控制栏 ──
        bar = QHBoxLayout()
        bar.addWidget(QLabel("起点偏移 (mm):"))
        self.sliders_s = {}
        for label, attr in [("X", "dx0"), ("Y", "dy0"), ("Z", "dz0")]:
            bar.addWidget(QLabel(label))
            sl = QSlider(Qt.Horizontal); sl.setRange(-80, 80); sl.setValue(0)
            sl.setTickInterval(10); sl.valueChanged.connect(self._on_slider)
            self.sliders_s[attr] = sl
            bar.addWidget(sl)

        bar.addSpacing(20)
        bar.addWidget(QLabel("终点偏移 (mm):"))
        self.sliders_g = {}
        for label, attr in [("X", "dxg"), ("Y", "dyg"), ("Z", "dzg")]:
            bar.addWidget(QLabel(label))
            sl = QSlider(Qt.Horizontal); sl.setRange(-80, 80); sl.setValue(0)
            sl.setTickInterval(10); sl.valueChanged.connect(self._on_slider)
            self.sliders_g[attr] = sl
            bar.addWidget(sl)

        bar.addStretch()
        self.btn_reset = QPushButton("重置")
        self.btn_reset.clicked.connect(self._reset)
        bar.addWidget(self.btn_reset)

        ml.addLayout(bar)

        # ── Matplotlib 画布 ──
        self.fig = Figure(figsize=(15, 8), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        ml.addWidget(self.canvas, stretch=1)

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage(
            f"ProMP n_basis={self.n_basis} | {len(self.demos)} 条示教 | "
            f"拖动滑块改变起点/终点 → 实时条件推理"
        )

    def _on_slider(self):
        for attr, sl in {**self.sliders_s, **self.sliders_g}.items():
            setattr(self, attr, sl.value())
        self._draw()

    def _reset(self):
        for sl in {**self.sliders_s, **self.sliders_g}.values():
            sl.setValue(0)
        self._draw()

    def _draw(self):
        self.fig.clear()
        gs = self.fig.add_gridspec(2, 3, hspace=0.35, wspace=0.3)

        # 目标帧数
        T = self.promp.promp_x._T_learned
        t_vec = np.linspace(0, 1, T)

        # 原始起点/终点 (均值)
        y0 = np.array([self.promp.promp_x._y0, self.promp.promp_y._y0, self.promp.promp_z._y0])
        goal = np.array([self.promp.promp_x._g, self.promp.promp_y._g, self.promp.promp_z._g])

        # 偏移后的起点/终点 (mm → m)
        start_new = y0 + np.array([self.dx0, self.dy0, self.dz0]) / 1000.0
        goal_new = goal + np.array([self.dxg, self.dyg, self.dzg]) / 1000.0

        # ── 生成多条轨迹样本 (用于置信区间) ──
        # 对每条示教单独学 ProMP, 然后在新起点/终点下生成
        individual_promps = []
        for demo in self.demos:
            p = ProMP3D(n_basis=self.n_basis, sigma=0.03)
            p.learn(demo)
            gen = p.generate(T=T, start=start_new, goal=goal_new)
            individual_promps.append(gen)

        stack = np.stack(individual_promps, axis=0)  # (n_demos, T, 3)
        mean_traj = np.mean(stack, axis=0)  # (T, 3)
        std_traj = np.std(stack, axis=0)    # (T, 3)

        # 条件推理轨迹 (learn_multiple 的结果)
        cond_traj = self.promp.generate(T=T, start=start_new, goal=goal_new)

        # ── (0,0): XY 投影 + 置信带 ──
        ax_xy = self.fig.add_subplot(gs[0, 0])
        # 灰色置信带 (用 fill_between 在 2D 不太好, 改用多条轨迹叠加)
        for i, gen in enumerate(individual_promps):
            ax_xy.plot(gen[:, 0], gen[:, 1], 'gray', lw=0.3, alpha=0.3)
        ax_xy.plot(cond_traj[:, 0], cond_traj[:, 1], 'g-', lw=2.0, label='条件推理')
        ax_xy.plot(mean_traj[:, 0], mean_traj[:, 1], 'b--', lw=1.5, label='均值')
        # 起点/终点
        ax_xy.scatter(*start_new[:2], c='green', s=80, zorder=5, marker='o', label='起点')
        ax_xy.scatter(*goal_new[:2], c='red', s=80, zorder=5, marker='s', label='终点')
        ax_xy.set_xlabel('X (m)'); ax_xy.set_ylabel('Y (m)')
        ax_xy.set_title(f'XY 投影 — {len(self.demos)} 条示教分布'); ax_xy.legend(fontsize=7); ax_xy.grid(alpha=0.3)
        ax_xy.set_aspect('equal')

        # ── (0,1): XZ 投影 + 置信带 ──
        ax_xz = self.fig.add_subplot(gs[0, 1])
        for gen in individual_promps:
            ax_xz.plot(gen[:, 0], gen[:, 2], 'gray', lw=0.3, alpha=0.3)
        ax_xz.plot(cond_traj[:, 0], cond_traj[:, 2], 'g-', lw=2.0, label='条件推理')
        ax_xz.plot(mean_traj[:, 0], mean_traj[:, 2], 'b--', lw=1.5, label='均值')
        ax_xz.scatter(start_new[0], start_new[2], c='green', s=80, zorder=5, marker='o')
        ax_xz.scatter(goal_new[0], goal_new[2], c='red', s=80, zorder=5, marker='s')
        ax_xz.set_xlabel('X (m)'); ax_xz.set_ylabel('Z (m)')
        ax_xz.set_title('XZ 投影 — ProMP 条件推理'); ax_xz.legend(fontsize=7); ax_xz.grid(alpha=0.3)
        ax_xz.set_aspect('equal')

        # ── (0,2): YZ 投影 ──
        ax_yz = self.fig.add_subplot(gs[0, 2])
        for gen in individual_promps:
            ax_yz.plot(gen[:, 1], gen[:, 2], 'gray', lw=0.3, alpha=0.3)
        ax_yz.plot(cond_traj[:, 1], cond_traj[:, 2], 'g-', lw=2.0)
        ax_yz.plot(mean_traj[:, 1], mean_traj[:, 2], 'b--', lw=1.5)
        ax_yz.scatter(start_new[1], start_new[2], c='green', s=80, zorder=5)
        ax_yz.scatter(goal_new[1], goal_new[2], c='red', s=80, zorder=5)
        ax_yz.set_xlabel('Y (m)'); ax_yz.set_ylabel('Z (m)')
        ax_yz.set_title('YZ 投影'); ax_yz.grid(alpha=0.3)
        ax_yz.set_aspect('equal')

        # ── (1,0): X 维度误差带 (均值 ±2σ) ──
        ax_xb = self.fig.add_subplot(gs[1, 0])
        idx = np.arange(T)
        ax_xb.fill_between(idx, mean_traj[:, 0] - 2*std_traj[:, 0],
                          mean_traj[:, 0] + 2*std_traj[:, 0],
                          alpha=0.25, color='green', label='2σ 置信带')
        ax_xb.plot(idx, mean_traj[:, 0], 'b-', lw=1.5, label='均值')
        ax_xb.plot(idx, cond_traj[:, 0], 'g-', lw=2.0, label='条件推理')
        # 叠加原始示教
        for demo in self.demos:
            Td = len(demo)
            ax_xb.plot(np.linspace(0, T-1, Td), demo[:, 0], 'gray', lw=0.2, alpha=0.4)
        ax_xb.set_xlabel('帧'); ax_xb.set_ylabel('X (m)')
        ax_xb.set_title('X 维度 — 均值 + 2σ 置信带'); ax_xb.legend(fontsize=7); ax_xb.grid(alpha=0.3)

        # ── (1,1): Y 维度误差带 ──
        ax_yb = self.fig.add_subplot(gs[1, 1])
        ax_yb.fill_between(idx, mean_traj[:, 1] - 2*std_traj[:, 1],
                          mean_traj[:, 1] + 2*std_traj[:, 1],
                          alpha=0.25, color='green', label='2σ 置信带')
        ax_yb.plot(idx, mean_traj[:, 1], 'b-', lw=1.5, label='均值')
        ax_yb.plot(idx, cond_traj[:, 1], 'g-', lw=2.0, label='条件推理')
        for demo in self.demos:
            Td = len(demo)
            ax_yb.plot(np.linspace(0, T-1, Td), demo[:, 1], 'gray', lw=0.2, alpha=0.4)
        ax_yb.set_xlabel('帧'); ax_yb.set_ylabel('Y (m)')
        ax_yb.set_title('Y 维度 — 均值 + 2σ 置信带'); ax_yb.legend(fontsize=7); ax_yb.grid(alpha=0.3)

        # ── (1,2): Z 维度误差带 + 方差统计 ──
        ax_zb = self.fig.add_subplot(gs[1, 2])
        ax_zb.fill_between(idx, mean_traj[:, 2] - 2*std_traj[:, 2],
                          mean_traj[:, 2] + 2*std_traj[:, 2],
                          alpha=0.25, color='green', label='2σ 置信带')
        ax_zb.plot(idx, mean_traj[:, 2], 'b-', lw=1.5, label='均值')
        ax_zb.plot(idx, cond_traj[:, 2], 'g-', lw=2.0, label='条件推理')
        for demo in self.demos:
            Td = len(demo)
            ax_zb.plot(np.linspace(0, T-1, Td), demo[:, 2], 'gray', lw=0.2, alpha=0.4)
        ax_zb.set_xlabel('帧'); ax_zb.set_ylabel('Z (m)')
        # 方差统计
        mean_std = std_traj.mean(axis=0) * 1000
        ax_zb.set_title(f'Z 维度 | 平均 σ=(X:{mean_std[0]:.1f} Y:{mean_std[1]:.1f} Z:{mean_std[2]:.1f})mm')
        ax_zb.legend(fontsize=7); ax_zb.grid(alpha=0.3)

        self.fig.suptitle(
            f'ProMP 条件推理 — 起点偏移=({self.dx0},{self.dy0},{self.dz0})mm '
            f'终点偏移=({self.dxg},{self.dyg},{self.dzg})mm',
            fontsize=12, fontweight='bold'
        )
        self.fig.subplots_adjust(hspace=0.35, wspace=0.3, top=0.92, bottom=0.06)
        self.canvas.draw()


def main():
    parser = argparse.ArgumentParser(description="ProMP 条件推理可视化")
    parser.add_argument("--episodes", type=str, default="top5",
                        help="示教集: top5 | all | 32,25,23")
    args = parser.parse_args()

    if args.episodes == "top5":
        eps = TOP5
    elif args.episodes == "all":
        eps = list(range(40))
    else:
        eps = [int(x.strip()) for x in args.episodes.split(",")]

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    gui = PrompConditioningGUI(episodes=eps)
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
