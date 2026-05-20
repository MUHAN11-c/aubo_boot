#!/usr/bin/env python3
"""
心形轨迹 Open3D 3D 渲染 — 高质量交互式可视化喵~

参考: movement_primitives/vis_cartesian_dmp.py (DFKI)
使用 Open3D 替代 matplotlib 3D, 支持:
  - GPU 加速渲染 (WebGL)
  - 鼠标旋转/缩放/平移
  - 轨迹线宽/颜色可调
  - 截图导出

使用:
  python3 scripts/vis_3d_heart.py
  python3 scripts/vis_3d_heart.py --episode 32
  python3 scripts/vis_3d_heart.py --compare-all  # 对比原始/ProMP/DMP/Ensemble
"""

import os, sys, argparse
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, PKG_DIR)

from scipy.signal import savgol_filter
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

from latte_imitation.promp_learner import ProMP3D
from latte_imitation.dmp_learner import DMP3D
from latte_imitation.trajectory import CartesianTrajectory

CARTESIAN_DIR = os.path.join(PKG_DIR, "resource", "cartesian", "right")
HEART_DIR = os.path.join(PKG_DIR, "resource", "heart")
TOP5 = [32, 25, 23, 6, 8]
DT = 0.05

# 颜色 (RGB)
C_ORIG = (0.1, 0.1, 0.1)       # 黑色 — 原始
C_PROMP = (0.18, 0.8, 0.24)     # 绿色 — ProMP
C_DMP = (0.9, 0.24, 0.24)       # 红色 — DMP
C_ENS = (0.2, 0.5, 1.0)         # 蓝色 — Ensemble


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


def make_lineset(points, color, width=2.0):
    """将轨迹点转为 Open3D LineSet 喵~"""
    n = len(points)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    lines = o3d.geometry.LineSet()
    lines.points = o3d.utility.Vector3dVector(points)
    edges = [[i, i + 1] for i in range(n - 1)]
    lines.lines = o3d.utility.Vector2iVector(edges)
    # 颜色 (每段)
    colors = np.tile(color, (len(edges), 1))
    lines.colors = o3d.utility.Vector3dVector(colors)
    return lines


def make_sphere(center, radius=0.002, color=(1, 0, 0)):
    """创建小球标记起点/终点 喵~"""
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.translate(center)
    sphere.paint_uniform_color(color)
    return sphere


class Heart3DApp:
    """Open3D GUI 应用 — 心形轨迹 3D 渲染 喵~"""

    def __init__(self, episode=32, compare_all=False):
        self.ep = episode
        self.compare_all = compare_all
        self.show_orig = True
        self.show_promp = True
        self.show_dmp = False  # DMP 默认关, 不可靠
        self.show_ens = True
        self.line_width = 3.0

        # 加载数据
        self._load_data()
        # 初始化窗口 (先不创建, 等 start() 调用)
        self.window = None
        self.scene = None

    def _load_data(self):
        ep_path = os.path.join(CARTESIAN_DIR, f"episode_{self.ep:06d}.npz")
        cart = CartesianTrajectory.load(ep_path)
        self.pos_full = cart.positions
        self.form_raw = extract_forming(cart.positions)
        Tf = len(self.form_raw)

        # ProMP
        promp = ProMP3D(n_basis=20, sigma=0.03)
        promp.learn(self.form_raw)
        self.recon_p = promp.generate(T=Tf)
        self.rmse_p = np.sqrt(np.mean(np.sum((self.form_raw - self.recon_p) ** 2, axis=1))) * 1000

        # DMP
        dmp = DMP3D(n_basis=25, dt=0.01)
        dmp.learn(self.form_raw, tau=1.0)
        self.recon_d = dmp.generate(tau=1.0, T=Tf)
        self.rmse_d = np.sqrt(np.mean(np.sum((self.form_raw - self.recon_d) ** 2, axis=1))) * 1000

    def _rebuild_scene(self):
        self.scene.scene.clear_geometry()

        if self.show_orig:
            ls = make_lineset(self.form_raw, C_ORIG, self.line_width)
            self.scene.scene.add_geometry("orig", ls, rendering.MaterialRecord())

        if self.show_promp:
            ls = make_lineset(self.recon_p, C_PROMP, self.line_width)
            self.scene.scene.add_geometry("promp", ls, rendering.MaterialRecord())

        if self.show_dmp:
            ls = make_lineset(self.recon_d, C_DMP, self.line_width)
            self.scene.scene.add_geometry("dmp", ls, rendering.MaterialRecord())

        # 起点/终点小球
        s = make_sphere(self.form_raw[0], 0.003, (0, 1, 0))  # 绿色起点
        e = make_sphere(self.form_raw[-1], 0.003, (1, 0, 0))  # 红色终点
        self.scene.scene.add_geometry("start", s, rendering.MaterialRecord())
        self.scene.scene.add_geometry("end", e, rendering.MaterialRecord())

        # 更新图例文字
        self._update_legend()

    def _update_legend(self):
        if self.window is None:
            return
        # 用调试文本显示 RMSE
        txt = f"ep{self.ep} | 原始={len(self.form_raw)}帧"
        if self.show_promp:
            txt += f" | ProMP RMSE={self.rmse_p:.1f}mm"
        if self.show_dmp:
            txt += f" | DMP RMSE={self.rmse_d:.1f}mm"
        if hasattr(self, '_info_label'):
            self._info_label.text = txt

    def start(self):
        """启动 Open3D GUI 喵~"""
        gui.Application.instance.initialize()
        self.window = gui.Application.instance.create_window(
            "心形轨迹 3D 渲染 — ProMP vs DMP", 1200, 800
        )

        # 3D 场景
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self.scene)

        # 控制面板
        panel = gui.Vert(0, gui.Margins(10, 10, 10, 10))
        panel.add_child(gui.Label("心形轨迹 3D 渲染"))
        panel.add_child(gui.Label(f"Episode {self.ep} | ProMP RMSE={self.rmse_p:.1f}mm"))
        panel.add_child(gui.Label(""))

        # 复选框
        self._cb_orig = gui.Checkbox("原始轨迹")
        self._cb_orig.checked = self.show_orig
        self._cb_orig.set_on_checked(self._on_toggle_orig)
        panel.add_child(self._cb_orig)

        self._cb_promp = gui.Checkbox("ProMP (绿色)")
        self._cb_promp.checked = self.show_promp
        self._cb_promp.set_on_checked(self._on_toggle_promp)
        panel.add_child(self._cb_promp)

        self._cb_dmp = gui.Checkbox("DMP (红色,不可靠)")
        self._cb_dmp.checked = self.show_dmp
        self._cb_dmp.set_on_checked(self._on_toggle_dmp)
        panel.add_child(self._cb_dmp)

        panel.add_child(gui.Label(""))
        panel.add_child(gui.Label("操作: 鼠标左键旋转"))
        panel.add_child(gui.Label("      鼠标滚轮缩放"))
        panel.add_child(gui.Label("      鼠标右键平移"))
        panel.add_child(gui.Label("      Ctrl+S 截图"))

        self._info_label = gui.Label(f"ep{self.ep} ProMP={self.rmse_p:.1f}mm | 原始={len(self.form_raw)}帧")
        panel.add_child(self._info_label)

        self.window.set_on_layout(self._on_layout)
        self._panel = panel

        self._rebuild_scene()

        # 设置相机视角
        bounds = self.scene.scene.bounding_box
        self.scene.setup_camera(60, bounds, bounds.get_center())

        gui.Application.instance.run()

    def _on_layout(self, layout_context):
        r = self.window.content_rect
        panel_width = 220
        self._panel.frame = gui.Rect(r.x, r.y, panel_width, r.height)
        self.scene.frame = gui.Rect(r.x + panel_width, r.y,
                                     r.width - panel_width, r.height)

    def _on_toggle_orig(self, checked):
        self.show_orig = checked
        self._rebuild_scene()

    def _on_toggle_promp(self, checked):
        self.show_promp = checked
        self._rebuild_scene()

    def _on_toggle_dmp(self, checked):
        self.show_dmp = checked
        self._rebuild_scene()


def main():
    parser = argparse.ArgumentParser(description="心形轨迹 Open3D 3D 渲染")
    parser.add_argument("--episode", type=int, default=32)
    args = parser.parse_args()

    app = Heart3DApp(episode=args.episode)
    app.start()


if __name__ == "__main__":
    main()
