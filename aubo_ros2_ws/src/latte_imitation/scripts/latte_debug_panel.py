#!/usr/bin/env python3
"""
latte_imitation 可视化调试面板 — PySide6 + matplotlib 3D 喵~

功能:
  - 3D 显示 base_link / tool_tcp TF 关系 + 轨迹预览
  - 欧拉角滑块实时旋转轨迹预览
  - 一键下发执行指令

用法:
  cd /home/mu/IVG2.0/aubo_ros2_ws
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 src/latte_imitation/scripts/latte_debug_panel.py
"""

import os
import sys
import subprocess
import threading
import numpy as np

_script_dir = os.path.dirname(os.path.abspath(__file__))
_pkg_dir = os.path.dirname(_script_dir)
if _pkg_dir not in sys.path:
    sys.path.insert(0, _pkg_dir)

from latte_imitation.trajectory import CartesianTrajectory
from latte_imitation.trajectory_transform import (
    euler_deg_to_quat, apply_start_pose, quat_to_rot,
)
from latte_imitation.tf_utils import TfQueryNode
from geometry_msgs.msg import Pose, Point, Quaternion

# TF 查询 — 持久 ROS 节点 + 后台线程 spin, 避免阻塞 GUI 喵~
_tf_query = None


def _ensure_tf_node():
    """启动持久 TF 查询节点 (单例) 喵~"""
    global _tf_query
    if _tf_query is None:
        _tf_query = TfQueryNode()


def get_current_ee_pose(timeout=1.5):
    """查询 base_link → tool_tcp 的当前位姿, 返回 Pose 或 None 喵~"""
    _ensure_tf_node()
    return _tf_query.get_pose(timeout)


# ═══════════════════════════════════════════════════════════════
# 服务调用
# ═══════════════════════════════════════════════════════════════

def call_replay_service(episode_idx, arm, speed_scale, mode,
                        px, py, pz, qx, qy, qz, qw):
    """调用 /latte_imitation/replay_trajectory 服务, 返回 (success, message) 喵~"""
    import yaml as _yaml
    data = {
        "episode_idx": int(episode_idx),
        "arm": str(arm),
        "speed_scale": float(speed_scale),
        "mode": str(mode),
        "collision_check": True,
        "start_pose": {
            "position": {"x": float(px), "y": float(py), "z": float(pz)},
            "orientation": {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
        },
    }
    yaml_str = _yaml.dump(data, default_flow_style=True, sort_keys=False).strip().replace("\n", "")
    cmd = (
        "ros2 service call /latte_imitation/replay_trajectory "
        "ivg_interfaces/srv/ReplayLatteTrajectory "
        f"'{yaml_str}'"
    )
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=130)
        output = (result.stdout or "") + (result.stderr or "")
        # ros2 service call 输出格式:
        #   response:
        #   ivg_interfaces.srv.ReplayLatteTrajectory_Response(success=True, message='...', ...)
        import re
        m = re.search(r"success\s*[:=]\s*True", output)
        if m:
            # 提取 message 字段
            mm = re.search(r"message\s*[:=]\s*['\"]?([^'\",}\n]+)", output)
            return True, mm.group(1).strip("'\" ") if mm else "执行成功"
        # 失败: 提取 error message
        mm = re.search(r"message\s*[:=]\s*['\"]?([^'\",}\n]+)", output)
        if mm:
            return False, mm.group(1).strip("'\" ")
        # 兜底: 显示最后 200 字符
        tail = output.strip()[-200:]
        return False, tail if tail else "(空输出)"
    except subprocess.TimeoutExpired:
        return False, "命令超时 (130s)"
    except Exception as e:
        return False, str(e)


# ═══════════════════════════════════════════════════════════════
# PySide6 主窗口
# ═══════════════════════════════════════════════════════════════

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QDoubleSpinBox, QComboBox, QSpinBox,
    QGroupBox, QTextEdit, QSplitter, QFrame, QGridLayout, QStyleFactory,
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont

# 修复 pip matplotlib 与系统 mpl_toolkits 冲突 (系统 .pth 强制注册旧路径到 sys.modules) 喵~
import sys as _sys
_PIP_MPL = "/home/mu/.local/lib/python3.10/site-packages"
if "mpl_toolkits" in _sys.modules:
    _mt = _sys.modules["mpl_toolkits"]
    _pip_mtk = _PIP_MPL + "/mpl_toolkits"
    if hasattr(_mt, "__path__") and _pip_mtk not in _mt.__path__:
        _mt.__path__.insert(0, _pip_mtk)
    for _k in list(_sys.modules.keys()):
        if _k.startswith("mpl_toolkits."):
            del _sys.modules[_k]

import matplotlib
matplotlib.use("QtAgg")
# PySide6 6.11.1 将 KeyboardModifier 从 IntEnum 改为普通 Enum, int() 无法转换.
# 必须在 backend_qt 被导入之前 patch _to_int 走 .value 路径 喵~
import matplotlib.backends.qt_compat as _mpl_qtc
_mpl_qtc._to_int = lambda x: x.value
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
matplotlib.rcParams["font.family"] = ["AR PL UKai CN", "sans-serif"]
matplotlib.rcParams["axes.unicode_minus"] = False  # CJK 字体不含 U+2212, 用 ASCII 减号 喵~
# 3D projection 由 add_subplot(projection='3d') 自动注册, 无需显式 import Axes3D 喵~


class LatteDebugPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("latte_imitation 可视化调试面板")
        self.setMinimumSize(1200, 750)

        # ── 数据 ──
        self._carts = {}           # {ep_id: CartesianTrajectory}
        self._current_ep = 0
        self._arm = "right"
        self._speed = 1.0
        self._ee_pose = None       # 当前 EE Pose (from TF)
        self._exec_busy = False   # 防止重复点击执行 喵~
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

        # ── 加载轨迹 ──
        res_dir = os.path.join(_pkg_dir, "resource")
        self._carts = CartesianTrajectory.load_all(res_dir, self._arm)
        self._ep_ids = list(self._carts.keys())

        # ── UI ──
        self._build_ui()

        # ── 初始 EE 位姿 ──
        self._refresh_tf()
        self._update_preview()

    # ═══════════════════════════════════════════════════════════
    # UI 构建
    # ═══════════════════════════════════════════════════════════

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        splitter = QSplitter(Qt.Horizontal)

        # ── 左侧: 3D 画布 ──
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)

        self._fig = Figure(figsize=(7, 6), dpi=100)
        self._canvas = FigureCanvas(self._fig)
        self._ax = self._fig.add_subplot(111, projection="3d")
        left_layout.addWidget(self._canvas)

        # ── 右侧: 控制面板 ──
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setSpacing(8)

        # --- 参数 ---
        grp_param = QGroupBox("基础参数")
        grid = QGridLayout(grp_param)

        grid.addWidget(QLabel("Episode:"), 0, 0)
        self._ep_spin = QSpinBox()
        self._ep_spin.setRange(0, 39)
        self._ep_spin.setValue(0)
        self._ep_spin.valueChanged.connect(self._on_ep_changed)
        grid.addWidget(self._ep_spin, 0, 1)
        self._ep_label = QLabel(f" (共 {len(self._ep_ids)} 条)")
        grid.addWidget(self._ep_label, 0, 2)

        grid.addWidget(QLabel("Arm:"), 1, 0)
        self._arm_combo = QComboBox()
        self._arm_combo.addItems(["right", "left"])
        self._arm_combo.currentTextChanged.connect(self._on_arm_changed)
        grid.addWidget(self._arm_combo, 1, 1)

        grid.addWidget(QLabel("Speed:"), 2, 0)
        self._speed_spin = QDoubleSpinBox()
        self._speed_spin.setRange(0.1, 10.0)
        self._speed_spin.setValue(1.0)
        self._speed_spin.setSingleStep(0.5)
        grid.addWidget(self._speed_spin, 2, 1)

        right_layout.addWidget(grp_param)

        # --- TF / EE 状态 ---
        grp_tf = QGroupBox("当前末端位姿 (TF: base_link→tool_tcp)")
        tf_layout = QVBoxLayout(grp_tf)
        self._ee_status = QLabel("未获取")
        self._ee_status.setWordWrap(True)
        tf_layout.addWidget(self._ee_status)
        btn_tf = QPushButton("🔄 刷新 TF")
        btn_tf.clicked.connect(self._on_refresh_tf)
        tf_layout.addWidget(btn_tf)
        right_layout.addWidget(grp_tf)

        # --- 欧拉角旋转 ---
        grp_rot = QGroupBox("旋转控制 (欧拉角 °)")
        rot_layout = QGridLayout(grp_rot)

        self._sliders = {}
        self._spinboxes = {}
        for i, (name, key) in enumerate([
            ("Roll  / X轴", "roll"),
            ("Pitch / Y轴", "pitch"),
            ("Yaw   / Z轴", "yaw"),
        ]):
            rot_layout.addWidget(QLabel(name), i, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(45)
            slider.valueChanged.connect(self._on_slider_changed)
            self._sliders[key] = slider
            rot_layout.addWidget(slider, i, 1)

            spin = QDoubleSpinBox()
            spin.setRange(-180.0, 180.0)
            spin.setValue(0.0)
            spin.setSingleStep(5.0)
            spin.setSuffix("°")
            spin.valueChanged.connect(self._on_spin_changed)
            self._spinboxes[key] = spin
            rot_layout.addWidget(spin, i, 2)

        # 快捷按钮行
        quick_row = QHBoxLayout()
        for label, r, p, y in [
            ("0°", 0, 0, 0), ("Z+90°", 0, 0, 90), ("Z+180°", 0, 0, 180),
            ("Z-90°", 0, 0, -90), ("X+90°", 90, 0, 0), ("Y+90°", 0, 90, 0),
        ]:
            btn = QPushButton(label)
            btn.setFixedWidth(55)
            btn.clicked.connect(lambda checked, r=r, p=p, y=y: self._set_rpy(r, p, y))
            quick_row.addWidget(btn)
        quick_row.addStretch()
        rot_layout.addLayout(quick_row, 3, 0, 1, 3)

        right_layout.addWidget(grp_rot)

        # --- 执行按钮 ---
        grp_exec = QGroupBox("执行")
        exec_layout = QVBoxLayout(grp_exec)
        self._btn_preview = QPushButton("🔄 刷新预览")
        self._btn_preview.clicked.connect(self._on_preview)
        exec_layout.addWidget(self._btn_preview)

        # Debug / Action 按钮
        btn_row = QHBoxLayout()
        self._btn_debug = QPushButton("👁 Debug (仅发布话题)")
        self._btn_debug.clicked.connect(lambda: self._on_execute("debug"))
        btn_row.addWidget(self._btn_debug)

        self._btn_action = QPushButton("▶ 执行轨迹 (Action)")
        self._btn_action.setStyleSheet(
            "QPushButton { background-color: #27ae60; color: white; font-weight: bold; "
            "padding: 8px; border-radius: 4px; }"
            "QPushButton:hover { background-color: #2ecc71; }"
        )
        self._btn_action.clicked.connect(lambda: self._on_execute("action"))
        btn_row.addWidget(self._btn_action)
        exec_layout.addLayout(btn_row)
        right_layout.addWidget(grp_exec)

        # --- 日志 ---
        grp_log = QGroupBox("日志")
        log_layout = QVBoxLayout(grp_log)
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumHeight(200)
        self._log.setFont(QFont("monospace", 9))
        log_layout.addWidget(self._log)
        right_layout.addWidget(grp_log)

        right_layout.addStretch()

        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setSizes([700, 500])

        layout = QVBoxLayout(central)
        layout.addWidget(splitter)

    # ═══════════════════════════════════════════════════════════
    # 交互回调
    # ═══════════════════════════════════════════════════════════

    def _on_ep_changed(self, val):
        self._current_ep = val
        self._update_preview()

    def _on_arm_changed(self, arm):
        self._arm = arm
        res_dir = os.path.join(_pkg_dir, "resource")
        self._carts = CartesianTrajectory.load_all(res_dir, arm)
        self._ep_ids = list(self._carts.keys())
        self._ep_spin.setMaximum(max(39, len(self._ep_ids) - 1))
        self._ep_label.setText(f" (共 {len(self._carts)} 条)")
        self._update_preview()

    def _on_slider_changed(self):
        """slider → spinbox 同步 (避免循环) 喵~"""
        for key, slider in self._sliders.items():
            spin = self._spinboxes[key]
            if spin.value() != slider.value():
                spin.blockSignals(True)
                spin.setValue(float(slider.value()))
                spin.blockSignals(False)
        self._sync_rpy_from_ui()
        self._update_preview()

    def _on_spin_changed(self):
        """spinbox → slider 同步 (避免循环) 喵~"""
        for key, spin in self._spinboxes.items():
            slider = self._sliders[key]
            if slider.value() != int(spin.value()):
                slider.blockSignals(True)
                slider.setValue(int(spin.value()))
                slider.blockSignals(False)
        self._sync_rpy_from_ui()
        self._update_preview()

    def _sync_rpy_from_ui(self):
        self._roll = self._spinboxes["roll"].value()
        self._pitch = self._spinboxes["pitch"].value()
        self._yaw = self._spinboxes["yaw"].value()

    def _set_rpy(self, r, p, y):
        for key, val in [("roll", r), ("pitch", p), ("yaw", y)]:
            self._spinboxes[key].setValue(float(val))
        # spinbox setValue triggers _on_spin_changed → _sync → _update_preview

    def _on_refresh_tf(self):
        self._refresh_tf()
        self._update_preview()

    def _on_preview(self):
        self._refresh_tf()
        self._update_preview()
        self._log_info("预览已刷新")

    def _on_execute(self, mode):
        if self._exec_busy:
            self._log_err("上一指令仍在执行中, 请等待完成后再试")
            return
        ep = self._ep_spin.value()
        arm = self._arm
        spd = self._speed_spin.value()
        roll, pitch, yaw = self._roll, self._pitch, self._yaw
        q = euler_deg_to_quat(roll, pitch, yaw)

        use_tf = (self._ee_pose is not None)
        if use_tf:
            px, py, pz = (self._ee_pose.position.x,
                          self._ee_pose.position.y,
                          self._ee_pose.position.z)
        else:
            px = py = pz = 0.0

        self._log_info(
            f"下发: ep={ep} arm={arm} speed={spd} mode={mode} "
            f"pos=({'TF' if use_tf else '0'}) "
            f"rpy=({roll:.0f},{pitch:.0f},{yaw:.0f}) "
            f"→ quat=[{q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f}]"
        )
        self._log_info("⏳ 等待服务响应 (action 模式需执行完整轨迹, 请耐心等待)...")
        self._exec_busy = True
        self._btn_action.setEnabled(False)
        self._btn_debug.setEnabled(False)
        QTimer.singleShot(100, lambda: self._do_execute(
            ep, arm, spd, mode, px, py, pz, q[0], q[1], q[2], q[3]))

    def _do_execute(self, ep, arm, spd, mode, px, py, pz, qx, qy, qz, qw):
        """后台线程执行 ros2 service call, 避免阻塞 GUI 喵~"""
        self._exec_result = None

        def _worker():
            self._exec_result = call_replay_service(
                ep, arm, spd, mode, px, py, pz, qx, qy, qz, qw)

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

        # 用闭包持有 timer 引用, 避免 self._poll_timer 被覆盖导致旧 timer 失控 喵~
        poll_timer = QTimer()
        poll_timer.timeout.connect(lambda: _check_done(t, poll_timer))
        poll_timer.start(200)

        def _check_done(thread, timer):
            if thread.is_alive():
                return
            timer.stop()
            self._exec_busy = False
            self._btn_action.setEnabled(True)
            self._btn_debug.setEnabled(True)
            ok, msg = self._exec_result or (False, "执行线程无返回")
            if ok:
                self._log_info(f"✅ {msg}")
            else:
                self._log_err(f"❌ {msg}")

    # ═══════════════════════════════════════════════════════════
    # TF 查询
    # ═══════════════════════════════════════════════════════════

    def _refresh_tf(self):
        pose = get_current_ee_pose(timeout=1.5)
        if pose is not None:
            self._ee_pose = pose
            self._ee_status.setText(
                f"位置: ({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f})\n"
                f"姿态: xyzw=({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, "
                f"{pose.orientation.z:.4f}, {pose.orientation.w:.4f})"
            )
        else:
            self._ee_status.setText("⚠ TF 不可达 (ROS 未运行或 TF 未就绪)")

    # ═══════════════════════════════════════════════════════════
    # 3D 预览更新
    # ═══════════════════════════════════════════════════════════

    def _update_preview(self):
        ax = self._ax
        ax.clear()

        # ── 获取当前轨迹 ──
        ep_id = self._ep_ids[self._ep_spin.value() % len(self._ep_ids)]
        cart = self._carts[ep_id]
        positions = cart.positions.copy()

        # ── 起点位置 ──
        if self._ee_pose is not None:
            p0 = np.array([self._ee_pose.position.x,
                           self._ee_pose.position.y,
                           self._ee_pose.position.z])
        else:
            p0 = positions[0].copy()

        # ── 应用旋转 — 使用 apply_start_pose 标准库函数喵~
        q_rot = euler_deg_to_quat(self._roll, self._pitch, self._yaw)
        target_pose = Pose(
            position=Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])),
            orientation=Quaternion(x=float(q_rot[0]), y=float(q_rot[1]),
                                    z=float(q_rot[2]), w=float(q_rot[3])),
        )
        transformed = apply_start_pose(cart, target_pose, rotate_orientation=True)
        rotated = transformed.positions

        # ── 绘制 ──
        # 基座坐标系 (base_link)
        axis_len = 0.15
        o = np.zeros(3)
        ax.quiver(*o, *[axis_len, 0, 0], color='r', linewidth=2, label='base X')
        ax.quiver(*o, *[0, axis_len, 0], color='g', linewidth=2, label='base Y')
        ax.quiver(*o, *[0, 0, axis_len], color='b', linewidth=2, label='base Z')

        # 末端坐标系 (tool_tcp)
        if self._ee_pose is not None:
            tp = p0
            q_ee = np.array([self._ee_pose.orientation.x,
                            self._ee_pose.orientation.y,
                            self._ee_pose.orientation.z,
                            self._ee_pose.orientation.w])
            R_ee = quat_to_rot(q_ee)
        else:
            tp = p0
            R_ee = np.eye(3)
        ax.quiver(*tp, *(R_ee[:, 0] * axis_len), color='#ff6666', linewidth=2)
        ax.quiver(*tp, *(R_ee[:, 1] * axis_len), color='#66ff66', linewidth=2)
        ax.quiver(*tp, *(R_ee[:, 2] * axis_len), color='#6666ff', linewidth=2)

        # 起点球
        ax.scatter(*tp, c='cyan', s=100, marker='o', zorder=10,
                   edgecolors='black', linewidths=0.5, label='EE / 轨迹起点')

        # 旋转后轨迹
        ax.plot(rotated[:, 0], rotated[:, 1], rotated[:, 2],
                color='#ff6b6b', linewidth=1.5, alpha=0.9, label='旋转后轨迹')

        # 原始轨迹 (灰色半透明参考)
        raw_p0 = positions[0]
        raw_shifted = positions - raw_p0 + p0  # 平移到同一起点
        ax.plot(raw_shifted[:, 0], raw_shifted[:, 1], raw_shifted[:, 2],
                color='gray', linewidth=0.5, alpha=0.3, label='原始朝向参考')

        # 起止标记
        ax.scatter(*rotated[0], c='green', s=50, marker='o', zorder=8)
        ax.scatter(*rotated[-1], c='red', s=50, marker='s', zorder=8)

        # ── 视图范围 ──
        all_pts = rotated
        center = np.mean(all_pts, axis=0)
        span = max(np.ptp(all_pts[:, i]) for i in range(3)) * 0.7
        span = max(span, 0.3)
        for i in range(3):
            ax.set_xlim(center[0] - span, center[0] + span)
            ax.set_ylim(center[1] - span, center[1] + span)
            ax.set_zlim(center[2] - span, center[2] + span)

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title(
            f"Ep{ep_id} ({self._arm})  |  "
            f"rpy=({self._roll:.0f},{self._pitch:.0f},{self._yaw:.0f})°  |  "
            f"起点=({tp[0]:.3f},{tp[1]:.3f},{tp[2]:.3f})"
        )
        ax.legend(loc='upper left', fontsize=7)
        self._canvas.draw_idle()

    # ═══════════════════════════════════════════════════════════
    # 日志
    # ═══════════════════════════════════════════════════════════

    def _log_info(self, msg):
        self._log.append(f"<span style='color:#2ecc71;'>[INFO]</span> {msg}")

    def _log_err(self, msg):
        self._log.append(f"<span style='color:#e74c3c;'>[ERROR]</span> {msg}")


# ═══════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════

def main():
    app = QApplication(sys.argv)
    app.setStyle(QStyleFactory.create("Fusion"))
    panel = LatteDebugPanel()
    panel.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
