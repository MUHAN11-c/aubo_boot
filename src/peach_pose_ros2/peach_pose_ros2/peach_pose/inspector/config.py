"""Inspector 全局配置 — 相机内参、数据集路径与可视化参数。

职责: 集中存放 GUI 与可视化共用的常量，避免 magic number 散落各处。

在管线中的位置:
  支撑/GUI — 不参与算法推理，为 pinhole 反投影、YOLO 类别色、3D 标记色
  提供单一配置源。

核心理论要点:
  - 本工作区默认内参一律本机 Percipio（color_camera_info.yaml 棋盘标定）
  - 实机节点优先订阅 /camera/color/camera_info（与下方 K 数值同源）
  - K_AZURE 仅历史 Azure 离线包；本机回放/工具默认不要用
  - 深度单位: 管线内部 uint16「毫米」/1000→米；Percipio 原始图需先 × DepthScaleUnit

主要常量:
  K / K_PERCIPIO, K_AZURE, DATASET_DIR, YOLO_MODEL, KEYPOINT_COLORS, TOOL_GEOMETRY
"""
from pathlib import Path

# ── 本机 Percipio 彩色内参（640×480，棋盘标定）──
# 权威源: src/percipio_camera/config/color_camera_info.yaml — 改标定后请同步此处
K_PERCIPIO = {
    "fx": 466.174635,
    "fy": 465.556589,
    "cx": 326.071333,
    "cy": 244.789156,
    "width": 640,
    "height": 480,
}
# 本工作区默认内参别名（工具/回放缺省一律走本相机）
K = K_PERCIPIO
CALIBRATION_VERSION = (
    "percipio-640x480-chessboard|hand_eye:import_humble_20260128T114006")

# ── 仅历史 Azure Kinect 离线包（1280×720，FOV 推导）──
# 回放 Azure 数据集时显式传 --fx/--fy/--cx/--cy，勿作本机默认
K_AZURE = {"fx": 640.0, "fy": 636.0, "cx": 640.0, "cy": 360.0}

# ── 数据集 / 模型路径（相对 peach_pose_ros2 包根：.../src/peach_pose_ros2/）──
_PKG_ROOT = Path(__file__).resolve().parent.parent.parent.parent
DATASET_DIR = _PKG_ROOT / "data" / "dataset"
YOLO_MODEL = _PKG_ROOT / "model" / "best.pt"
MODEL_VERSION = (
    "yolo:6981750db67a726e|mobile_sam:6dbb90523a35330f")

# ── 类别定义 (与 YOLO best.pt 训练标签一致) ──
CLASS_COLORS = {0: (0, 220, 0), 1: (255, 200, 0)}  # OpenCV BGR: bag=绿, nobag=橙

# ── 关键点可视化颜色 (2D BGR / 3D RGB 归一化 [0,1]) ──
KEYPOINT_COLORS = {
    "center":  {"label": "球心(参考)", "bgr": (0, 230, 0),   "rgb3d": (0.0, 0.9, 0.0)},
    "entry_start": {"label": "入口起点(TCP)", "bgr": (0, 255, 255), "rgb3d": (1.0, 1.0, 0.0)},
    "cylinder":    {"label": "圆柱包络",      "bgr": (100, 100, 255), "rgb3d": (0.5, 0.5, 0.5)},
}

# ── 工具几何 (从 contracts 导入, 单一数据源) ──
from ..contracts import TOOL_GEOMETRY  # noqa: F401  # 重新导出供 visualization / GUI 使用
