"""主窗口 — 组装所有标签页 + 工具栏 喵~"""

import os, sys, subprocess, numpy as np
SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, PKG_DIR)

from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QSpinBox, QCheckBox, QTabWidget,
    QStatusBar, QTextEdit, QMessageBox,
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.font_manager as fm
_cn = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if os.path.exists(_cn):
    fm.fontManager.addfont(_cn)
    matplotlib.rcParams['font.family'] = fm.FontProperties(fname=_cn).get_name()
matplotlib.rcParams['axes.unicode_minus'] = False

from .workers import TrainWorker, CalibrateWorker, CVWorker, extract_forming
from .tab_train import TabTrain
from .tab_generalize import TabGeneralize
from .tab_tf import TabTF
from .tab_promp import TabPromp
from .tab_stages import TabStages

DT = 0.05; CARTESIAN_DIR = os.path.join(PKG_DIR, "resource", "cartesian", "right")
TOP5 = [32, 25, 23, 6, 8]


class MainWindow(QMainWindow):
    def __init__(self, episode=32):
        super().__init__()
        self.setWindowTitle("心形拉花训练 — 全功能可视化")
        self.setGeometry(50, 50, 1650, 900)

        # ── 状态 ──
        self._cur = None; self._ensemble = None; self._all_forming = None
        self._cv_results = None; self._active_tab = 0

        self._setup_ui()
        self._preload_data()
        self.tabs.setCurrentIndex(0)

    def _setup_ui(self):
        cw = QWidget(); self.setCentralWidget(cw)
        ml = QVBoxLayout(cw); ml.setContentsMargins(5,5,5,5)

        # ── 工具栏 ──
        bar = QHBoxLayout()
        bar.addWidget(QLabel("Ep:"))
        self.ep_combo = QComboBox()
        self.ep_combo.addItem("all")
        self.ep_combo.addItems([str(i) for i in range(40)])
        self.ep_combo.setCurrentText("32"); bar.addWidget(self.ep_combo)
        bar.addWidget(QLabel("基函数:"))
        self.nb_spin = QSpinBox(); self.nb_spin.setRange(5,60); self.nb_spin.setValue(20); bar.addWidget(self.nb_spin)
        self.chk_o = QCheckBox("原始"); self.chk_o.setChecked(True)
        self.chk_p = QCheckBox("ProMP"); self.chk_p.setChecked(True)
        self.chk_e = QCheckBox("Ensemble"); self.chk_e.setChecked(True)
        bar.addWidget(self.chk_o); bar.addWidget(self.chk_p); bar.addWidget(self.chk_e)
        bar.addSpacing(10)
        self.btn_cal = QPushButton("参数标定"); bar.addWidget(self.btn_cal)
        self.btn_train = QPushButton("训练ProMP"); bar.addWidget(self.btn_train)
        self.btn_open3d = QPushButton("Open3D"); bar.addWidget(self.btn_open3d)
        self.btn_export = QPushButton("导出数据"); bar.addWidget(self.btn_export)
        self.btn_export.clicked.connect(self._export_all)
        bar.addStretch(); self.lbl_status = QLabel("就绪"); bar.addWidget(self.lbl_status)
        ml.addLayout(bar)

        # ── 标签页 ──
        self.tabs = QTabWidget(); self.tabs.currentChanged.connect(self._on_tab_changed)

        self.tab_train = TabTrain(self); self.tabs.addTab(self.tab_train, "单条训练")
        self.tab_generalize = TabGeneralize(self); self.tabs.addTab(self.tab_generalize, "泛化分析")
        self.tab_tf = TabTF(self); self.tabs.addTab(self.tab_tf, "TF对齐")
        self.tab_promp = TabPromp(self); self.tabs.addTab(self.tab_promp, "ProMP推理")
        self.tab_stages = TabStages(self); self.tabs.addTab(self.tab_stages, "三版对比")

        # 日志
        self.log_view = QTextEdit(); self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("monospace", 9)); self.tabs.addTab(self.log_view, "日志")

        ml.addWidget(self.tabs, stretch=1)
        self.setStatusBar(QStatusBar())

        # ── 信号连接 ──
        self.ep_combo.currentTextChanged.connect(self._on_ep_change)
        self.nb_spin.valueChanged.connect(self._on_ep_change)
        self.chk_o.toggled.connect(self._redraw_current)
        self.chk_p.toggled.connect(self._redraw_current)
        self.chk_e.toggled.connect(self._redraw_current)
        self.btn_cal.clicked.connect(self._run_calibrate)
        self.btn_train.clicked.connect(lambda: self._on_ep_change())
        self.btn_open3d.clicked.connect(self._launch_open3d)
        self.tab_generalize.btn_cv.clicked.connect(self._run_cv)
        self.tab_tf.btn_fetch.clicked.connect(lambda: self.tab_tf.poll_tf())
        self.tab_tf.btn_retarget.clicked.connect(self._do_retarget)
        self.tab_promp.btn_run.clicked.connect(self._run_promp)

        # TF 轮询
        self._tf_timer = QTimer(); self._tf_timer.timeout.connect(self._tf_poll)
        self._tf_timer.start(3000)

    def _tf_poll(self):
        self.tab_tf.start_tf()
        self.tab_tf.poll_tf()

    def closeEvent(self, event):
        if hasattr(self.tab_tf, '_tf') and self.tab_tf._tf is not None:
            self.tab_tf._tf.shutdown()
        super().closeEvent(event)

        self._train_worker = None; self._cal_worker = None
        self._cv_worker = None

    # ═══════════════════════ 数据 ═══════════════════════

    def _preload_data(self):
        self._log("预加载40条成形段...")
        from latte_imitation.trajectory import CartesianTrajectory
        all_f = []
        for ep in range(40):
            path = os.path.join(CARTESIAN_DIR, f"episode_{ep:06d}.npz")
            if os.path.exists(path):
                cart = CartesianTrajectory.load(path)
                all_f.append(extract_forming(cart.positions)[0])
        self._all_forming = all_f
        resampled = []
        for ep in TOP5:
            if ep < len(all_f):
                seg = all_f[ep]; T = len(seg)
                to,tn = np.linspace(0,1,T), np.linspace(0,1,120)
                resampled.append(np.column_stack([interp1d(to, seg[:,d], kind='linear')(tn) for d in range(3)]))
        self._ensemble = np.median(np.stack(resampled, axis=0), axis=0)
        w = min(11, 120//10*2+1)
        if w>=3: self._ensemble = savgol_filter(self._ensemble, w, 3, axis=0)
        self._log(f"OK: {len(all_f)}条成形段, Ensemble TOP5={self._ensemble.shape[0]}帧")
        self._on_ep_change()

    # ═══════════════════════ 训练 ═══════════════════════

    def _on_ep_change(self, *_):
        if self._all_forming is None: return
        txt = self.ep_combo.currentText()
        from latte_imitation.trajectory import CartesianTrajectory
        if txt == "all":
            # 全部40条: 中位数聚合作"原始", ProMP在train_done里用learn_multiple
            form_raw = self._ensemble.copy()
            Tf = len(form_raw)
            # pos_full: 构造一个"虚拟全轨迹"(仅成形段, 无进杯/退杯)
            pos_full = form_raw.copy()
            ep_label = "all"; fs, fe = 0, Tf
        else:
            ep = int(txt)
            ep_path = os.path.join(CARTESIAN_DIR, f"episode_{ep:06d}.npz")
            cart = CartesianTrajectory.load(ep_path)
            pos_full = cart.positions; Tf = len(pos_full)
            form_raw, fs, fe = extract_forming(pos_full)
            ep_label = ep
        self._cur = {"ep":ep_label,"pos_full":pos_full,"Tf":Tf,"fs":fs,"fe":fe,
                      "form_raw":form_raw,"Tfm":len(form_raw),
                      "recon_p":None,"rmse_p":0,"recon_d":None,"rmse_d":0,
                      "nb":self.nb_spin.value()}
        self.lbl_status.setText(f"训练 ep{ep_label}...")
        self._train_worker = TrainWorker(form_raw, self.nb_spin.value())
        self._train_worker.finished.connect(self._on_train_done)
        self._train_worker.start()

    def _on_train_done(self, result):
        if self._cur is None: return
        self._cur.update(result)
        if self._active_tab == 0: self.tab_train.draw(self._cur,
            self.chk_o.isChecked(), self.chk_p.isChecked(), self.chk_e.isChecked(), self._ensemble)
        self.lbl_status.setText(f"ep{self._cur['ep']}: ProMP={result['rmse_p']:.1f}mm | DMP={result['rmse_d']:.0f}mm")
        self._log(f"ep{self._cur['ep']}: ProMP={result['rmse_p']:.1f}mm DMP={result['rmse_d']:.0f}mm 改善={result['rmse_d']/max(result['rmse_p'],0.01):.0f}x")
        # 更新阶段对比 (如果标签页可见)
        if self._active_tab == 4:
            self.tab_stages.draw(self._cur, ensemble=self._ensemble)

    # ═══════════════════════ 标签页切换 ═══════════════════════

    def _on_tab_changed(self, idx):
        self._active_tab = idx
        QTimer.singleShot(50, lambda: self._redraw_tab(idx))

    def _redraw_tab(self, idx):
        if idx==0 and self._cur and self._cur.get("recon_p") is not None:
            self.tab_train.draw(self._cur, self.chk_o.isChecked(), self.chk_p.isChecked(), self.chk_e.isChecked(), self._ensemble)
        elif idx==1 and self._cv_results:
            self.tab_generalize.draw(self._cv_results)
        elif idx==2 and self.tab_tf.retargeted_traj is not None:
            self.tab_tf.draw(self._cur["form_raw"] if self._cur else None)
        elif idx==3 and self.tab_promp.data:
            self.tab_promp.draw()
        elif idx==4 and self._cur:
            self.tab_stages.draw(self._cur, ensemble=self._ensemble)

    def _redraw_current(self):
        self._redraw_tab(self._active_tab)

    # ═══════════════════════ 各功能入口 ═══════════════════════

    def _run_calibrate(self):
        self._log("开始参数标定..."); self.btn_cal.setEnabled(False)
        worker = CalibrateWorker("all")
        def on_prog(msg): self._log(msg)
        def on_done(r):
            self.btn_cal.setEnabled(True)
            self._log(f"标定: 频率中位数={r['freq_median']:.1f}Hz IQR=[{r['freq_iqr'][0]:.1f},{r['freq_iqr'][1]:.1f}] 振幅={r['amp_mean']:.2f}mm n={r['n_episodes']}")
            QMessageBox.information(self, "参数标定", f"频率中位数: {r['freq_median']:.1f}Hz\n振幅均值: {r['amp_mean']:.2f}mm\n标定样本: {r['n_episodes']}条")
        worker.progress.connect(on_prog); worker.finished.connect(on_done)
        self._cal_worker = worker; worker.start()

    def _run_cv(self):
        if self._all_forming is None: return
        self.tab_generalize.progress.setVisible(True); self.tab_generalize.btn_cv.setEnabled(False)
        self._cv_worker = CVWorker(self._all_forming)
        self._cv_worker.progress.connect(lambda p,m: self.tab_generalize.progress.setValue(p))
        self._cv_worker.finished.connect(self._on_cv_done)
        self._cv_worker.start()

    def _on_cv_done(self, results):
        self._cv_results = results
        self.tab_generalize.progress.setVisible(False); self.tab_generalize.btn_cv.setEnabled(True)
        if self._active_tab==1: self.tab_generalize.draw(results)
        p,d,e=results["promp"],results["dmp"],results["ens"]
        self._log(f"LOO: ProMP={p.mean():.0f}±{p.std():.0f} DMP={d.mean():.0f}±{d.std():.0f} Ensemble={e.mean():.0f}±{e.std():.0f} mm")

    def _do_retarget(self):
        if self._cur is None: return
        self.tab_tf.retarget(self._cur["form_raw"])
        if self._active_tab==2: self.tab_tf.draw(self._cur["form_raw"])
        pl = np.sum(np.linalg.norm(np.diff(self.tab_tf.retargeted_traj,axis=0),axis=1)) if self.tab_tf.retargeted_traj is not None else 0
        self._log(f"轨迹已对齐到末端: 路径长={pl:.3f}m")

    def _run_promp(self):
        self.lbl_status.setText("ProMP推理中...")
        data = self.tab_promp.run()
        if self._active_tab==4: self.tab_promp.draw()
        self.lbl_status.setText(f"ProMP推理: {data['n']}条示教")
        self._log(f"ProMP推理: {data['n']}条示教, σ=({data['std'][:,0].mean()*1000:.1f},{data['std'][:,1].mean()*1000:.1f},{data['std'][:,2].mean()*1000:.1f})mm")

    def _launch_open3d(self):
        script = os.path.join(SCRIPT_DIR, "vis_3d_heart.py")
        subprocess.Popen([sys.executable, script, "--episode", str(int(self.ep_combo.currentText()))], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._log(f"启动Open3D: ep{self.ep_combo.currentText()}")

    def _log(self, msg): self.log_view.append(msg)

    def _export_all(self):
        """导出全部数据用于离线分析 喵~"""
        import json
        from datetime import datetime
        export_dir = os.path.join(PKG_DIR, "export")
        os.makedirs(export_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # 每条轨迹的元数据
        per_ep = {}
        for i, frm in enumerate(self._all_forming or []):
            per_ep[f'ep{i:02d}'] = {
                'frames': len(frm),
                'path_mm': float(np.sum(np.linalg.norm(np.diff(frm,axis=0),axis=1))*1000),
                'x_amp_mm': float(frm[:,0].std()*1000),
                'y_range_mm': float((frm[:,1].max()-frm[:,1].min())*1000),
                'z_mean': float(frm[:,2].mean()),
            }

        # ProMP统计
        promp_stats = {}
        if self._cur and self._cur.get("rmse_p"):
            promp_stats['current_ep_rmse_mm'] = round(self._cur['rmse_p'], 2)
        if self._cv_results:
            promp_stats['loo_mean_mm'] = round(float(self._cv_results['promp'].mean()),1)
            promp_stats['loo_std_mm'] = round(float(self._cv_results['promp'].std()),1)

        export = {
            'timestamp': ts, 'n_episodes': len(self._all_forming or []),
            'per_episode': per_ep, 'promp_stats': promp_stats,
            'ensemble_top5': [int(e) for e in [32,25,23,6,8]],
        }

        path = os.path.join(export_dir, f'heart_export_{ts}.json')
        with open(path, 'w') as f:
            json.dump(export, f, indent=2, ensure_ascii=False)
        self._log(f"数据已导出: {path}")
        self.lbl_status.setText(f"已导出: heart_export_{ts}.json")
