"""贝叶斯优化标签页 — Optuna 自动搜索最优参数 喵~"""

import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QSpinBox
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

C = {"promp":"#2ecc71","ens":"#3498db"}


class TabOptimize(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.gui = parent
        layout = QVBoxLayout(self); layout.setContentsMargins(0,0,0,0)

        bar = QHBoxLayout()
        bar.addWidget(QLabel("轮数:"))
        self.trials_spin = QSpinBox(); self.trials_spin.setRange(10,200); self.trials_spin.setValue(50); bar.addWidget(self.trials_spin)
        self.btn_start = QPushButton("开始优化"); bar.addWidget(self.btn_start)
        self.btn_stop = QPushButton("停止"); self.btn_stop.setEnabled(False); bar.addWidget(self.btn_stop)
        self.lbl_progress = QLabel("就绪"); bar.addWidget(self.lbl_progress); bar.addStretch()
        layout.addLayout(bar)

        self.fig = Figure(figsize=(16,6), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        self.data = {"scores":[], "params_history":[], "best_score":0.0, "best_params":{}}

    def on_progress(self, trial_num, score, params):
        self.data["scores"].append(score); self.data["params_history"].append(params)
        if score > self.data["best_score"]:
            self.data["best_score"] = score; self.data["best_params"] = dict(params)
        self.lbl_progress.setText(f"Trial {trial_num+1}: {score:.1f} | 最优={self.data['best_score']:.1f}")

    def draw(self):
        if not self.data["scores"]: return
        self.fig.clear(); gs = self.fig.add_gridspec(2,3,hspace=0.4,wspace=0.35)
        scores=self.data["scores"]; n=len(scores); best=np.maximum.accumulate(scores)

        # 收敛曲线
        ax=self.fig.add_subplot(gs[0,0])
        ax.plot(range(n), scores, 'o-', color='gray', ms=3, lw=0.5, alpha=0.6)
        ax.plot(range(n), best, '-', color=C["promp"], lw=2, label=f'最优={best[-1]:.1f}')
        ax.set_title('收敛曲线'); ax.set_xlabel('Trial'); ax.set_ylabel('评分'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

        # 参数收敛
        ax=self.fig.add_subplot(gs[0,1])
        for k,c in [("wiggle_frequency","blue"),("wiggle_amplitude","orange"),("draw_height_offset","green")]:
            vals=[p.get(k,0) for p in self.data["params_history"]]
            if vals: ax.plot(range(len(vals)), vals, 'o-', color=c, ms=2, lw=0.5, label=k.replace("_"," "), alpha=0.7)
        ax.set_title('参数收敛'); ax.set_xlabel('Trial'); ax.legend(fontsize=7); ax.grid(alpha=0.3)

        # 最优参数表
        ax=self.fig.add_subplot(gs[0,2]); ax.axis('off')
        bp=self.data["best_params"]
        if bp:
            rows=[['参数','最优值']]+[[k.replace('_',' '),f'{v:.4f}'] for k,v in bp.items() if isinstance(v,float)]
            tbl=ax.table(cellText=rows, loc='center', cellLoc='center'); tbl.auto_set_font_size(False); tbl.set_fontsize(8)
            for i in range(len(rows[0])): tbl[0,i].set_facecolor('#e8e8e8')
        ax.set_title('最优参数')

        # 最优轨迹3D + XY
        if bp:
            try:
                from latte_imitation.latte_art import LatteArtTrajectory, CupConfig, PourConfig, compose_full_trajectory, apply_anti_sloshing
                cup=CupConfig(surface_z=bp.get("cup_surface_z",0.15), radius=bp.get("cup_radius",0.04))
                pour=PourConfig(mix_height_offset=bp.get("mix_height_offset",0.076), draw_height_offset=bp.get("draw_height_offset",0.006), finish_height_offset=bp.get("finish_height_offset",0.076), wiggle_amplitude=bp.get("wiggle_amplitude",0.006), wiggle_frequency=bp.get("wiggle_frequency",2.4), max_velocity=bp.get("max_velocity",0.05), max_acceleration=bp.get("max_acceleration",0.1))
                gen=LatteArtTrajectory(cup,pour); xyz=gen.heart()
                xyz=compose_full_trajectory(xyz,cup,pour); xyz=apply_anti_sloshing(xyz,pour)
                ax3=self.fig.add_subplot(gs[1,0], projection='3d')
                ax3.plot(xyz[:,0],xyz[:,1],xyz[:,2], 'g-', lw=1.5); ax3.set_title(f'最优心形(评分={self.data["best_score"]:.1f})')
                ax_xy=self.fig.add_subplot(gs[1,1])
                ax_xy.plot(xyz[:,0],xyz[:,1], 'g-', lw=1.2); ax_xy.set_xlabel('X'); ax_xy.set_ylabel('Y'); ax_xy.set_title('最优心形 XY'); ax_xy.grid(alpha=0.3)
            except: pass

        # 评分分布
        ax=self.fig.add_subplot(gs[1,2])
        ax.hist(scores, bins=min(20,n//2+1), color='steelblue', alpha=0.7, edgecolor='white')
        ax.axvline(self.data["best_score"], color=C["promp"], ls='--', lw=2)
        ax.set_title(f'评分分布(n={n})'); ax.set_xlabel('评分')
        self.canvas.draw_idle()
