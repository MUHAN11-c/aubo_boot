"""版本对比标签页 — 原始/ProMP/优化后 三版心形轨迹 3D 并排对比 喵~"""

import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

C_VERSIONS = {"original": "#1a1a1a", "promp": "#2ecc71", "optimized": "#f39c12"}
DT = 0.05


class TabStages(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.gui = parent
        layout = QVBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0)
        self.fig = Figure(figsize=(16, 7), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

    def draw(self, data, ensemble=None):
        """三版对比: 原始 / ProMP / 贝叶斯优化 喵~

        Args:
            data: dict from _cur with form_raw, recon_p, rmse_p, ep, Tfm
            opt_params: dict of optimized PourConfig params (or None)
            ensemble: ensemble median trajectory (or None)
        """
        if data is None: return
        self.fig.clear()
        gs = self.fig.add_gridspec(2, 3, hspace=0.35, wspace=0.3)

        fm = data.get("form_raw")
        rp = data.get("recon_p")
        ep = data.get("ep", "?")

        # ── 优化 = ProMP + SG平滑 (只去噪, 不改变形状) ──
        opt_traj = None; opt_jerk = None
        try:
            if rp is not None and len(rp) > 3:
                from scipy.signal import savgol_filter
                w = min(7, len(rp)//20*2+1)
                opt_traj = savgol_filter(rp.copy(), w, 3, axis=0) if w>=3 else rp.copy()
                vel = np.diff(opt_traj, axis=0)/0.05
                acc = np.diff(vel, axis=0)/0.05
                jerk = np.diff(acc, axis=0)/0.05
                opt_jerk = float(np.max(np.abs(jerk)))
        except Exception:
            opt_traj = None

        # ── 计算原始轨迹的jerk (作为对比) ──
        raw_jerk = None
        if fm is not None and len(fm) > 3:
            rv = np.diff(fm, axis=0)/0.05
            ra = np.diff(rv, axis=0)/0.05
            rj = np.diff(ra, axis=0)/0.05
            raw_jerk = float(np.max(np.abs(rj)))

        # ── 三版定义 ──
        versions = []
        if fm is not None:
            pl = np.sum(np.linalg.norm(np.diff(fm, axis=0), axis=1))*1000 if len(fm)>1 else 0
            jk = f"Jerk={raw_jerk:.0f}" if raw_jerk else ""
            versions.append(("原始轨迹", fm, C_VERSIONS["original"], pl, jk))
        if rp is not None:
            pl = np.sum(np.linalg.norm(np.diff(rp, axis=0), axis=1))*1000 if len(rp)>1 else 0
            rmse = data.get("rmse_p", 0)
            versions.append(("ProMP学习", rp, C_VERSIONS["promp"], pl, f"RMSE={rmse:.1f}mm"))
        if opt_traj is not None:
            pl = np.sum(np.linalg.norm(np.diff(opt_traj, axis=0), axis=1))*1000 if len(opt_traj)>1 else 0
            jk = f"Jerk={opt_jerk:.0f}" if opt_jerk else ""
            versions.append(("ProMP+平滑", opt_traj, C_VERSIONS["optimized"], pl, jk))

        if len(versions) < 2:
            # 至少需要原始 + ProMP
            return

        # ── Row 0: 三列 3D ──
        for col, (name, traj, color, pl, subtitle) in enumerate(versions):
            ax = self.fig.add_subplot(gs[0, col], projection='3d')
            if len(traj) < 2: continue
            ax.plot(traj[:,0], traj[:,1], traj[:,2], color=color, lw=2.0, alpha=0.9)
            ax.scatter(*traj[0], c='green', s=40, marker='o', label='起点')
            ax.scatter(*traj[-1], c='red', s=40, marker='s', label='终点')
            pad = 0.005
            ax.set_xlim(traj[:,0].min()-pad, traj[:,0].max()+pad)
            ax.set_ylim(traj[:,1].min()-pad, traj[:,1].max()+pad)
            ax.set_zlim(traj[:,2].min()-pad, traj[:,2].max()+pad)
            ax.set_title(f'{name}\n{subtitle} | {pl:.0f}mm', fontsize=10)
            ax.legend(fontsize=6)

        # ── Row 1: XY投影 / 误差对比 / 统计表 ──
        # XY投影叠加
        ax_xy = self.fig.add_subplot(gs[1, 0])
        for name, traj, color, pl, _ in versions:
            if len(traj) < 2: continue
            ax_xy.plot(traj[:,0], traj[:,1], color=color, lw=1.2 if "原始" in name else 1.5,
                      alpha=0.6 if "原始" in name else 0.9, label=f'{name}({pl:.0f}mm)')
        ax_xy.set_xlabel('X (m)'); ax_xy.set_ylabel('Y (m)')
        ax_xy.set_title('XY 投影叠加'); ax_xy.legend(fontsize=7); ax_xy.grid(alpha=0.3)
        ax_xy.set_aspect('equal')

        # XZ投影叠加
        ax_xz = self.fig.add_subplot(gs[1, 1])
        for name, traj, color, pl, _ in versions:
            if len(traj) < 2: continue
            ax_xz.plot(traj[:,0], traj[:,2], color=color, lw=1.2 if "原始" in name else 1.5,
                      alpha=0.6 if "原始" in name else 0.9, label=f'{name}')
        ax_xz.set_xlabel('X (m)'); ax_xz.set_ylabel('Z (m)')
        ax_xz.set_title('XZ 投影叠加'); ax_xz.legend(fontsize=7); ax_xz.grid(alpha=0.3)
        ax_xz.set_aspect('equal')

        # 统计表 — 列数动态匹配版本数
        ax_tbl = self.fig.add_subplot(gs[1, 2]); ax_tbl.axis('off')
        n_ver = len(versions)
        ver_names = [v[0] for v in versions]
        rows = [['指标'] + ver_names]
        # X振幅
        rows.append(['X振幅(mm)'] + [f'{v[1][:,0].std()*1000:.1f}' if len(v[1])>1 else '-' for v in versions])
        # Y范围
        rows.append(['Y范围(mm)'] + [f'{(v[1][:,1].max()-v[1][:,1].min())*1000:.0f}' if len(v[1])>1 else '-' for v in versions])
        # Z范围
        rows.append(['Z范围(mm)'] + [f'{(v[1][:,2].max()-v[1][:,2].min())*1000:.0f}' if len(v[1])>1 else '-' for v in versions])
        # 路径长
        rows.append(['路径长(mm)'] + [f'{v[3]:.0f}' for v in versions])
        # 帧数
        rows.append(['帧数'] + [str(len(v[1])) for v in versions])
        # 最大速度 (mm/s)
        max_vels = []
        for _, traj, _, _, _ in versions:
            if len(traj)>2:
                v = np.linalg.norm(np.diff(traj,axis=0)/0.05, axis=1)
                max_vels.append(f'{v.max()*1000:.0f}')
            else: max_vels.append('-')
        rows.append(['最大速度(mm/s)'] + max_vels)
        # 最大加速度 (mm/s²)
        max_accs = []
        for _, traj, _, _, _ in versions:
            if len(traj)>3:
                v = np.linalg.norm(np.diff(traj,axis=0)/0.05, axis=1)
                a = np.diff(v)/0.05
                max_accs.append(f'{a.max()*1000:.0f}')
            else: max_accs.append('-')
        rows.append(['最大加速度(mm/s²)'] + max_accs)
        # RMSE
        if data.get("rmse_p"):
            rmse_row = ['ProMP RMSE'] + ['-' for _ in range(n_ver)]
            rmse_row[2] = f'{data["rmse_p"]:.1f}mm'
            rows.append(rmse_row)

        tbl = ax_tbl.table(cellText=rows, loc='center', cellLoc='center')
        tbl.auto_set_font_size(False); tbl.set_fontsize(9)
        for i in range(len(rows[0])): tbl[0,i].set_facecolor('#e8e8e8')
        ax_tbl.set_title('三版对比统计', fontsize=10)

        self.canvas.draw_idle()
