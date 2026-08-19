"""
offline 全局配置 — 相机内参、数据集路径与可视化参数.

职责: 集中存放离线评估工具共用的常量，避免 magic number 散落各处。

在管线中的位置:
  支撑/离线 — 不参与算法推理与在线管线，为 pinhole 反投影提供单一
  配置源。

核心理论要点:
  - 本工作区默认内参一律本机 Percipio（color_camera_info.yaml 棋盘标定）
  - 实机节点优先订阅 /camera/color/camera_info（与下方 K 数值同源）
  - K_AZURE 仅历史 Azure 离线包；本机回放/工具默认不要用
  - 深度单位: 管线内部 uint16「毫米」/1000→米；Percipio 原始图需先 × DepthScaleUnit

主要常量:
  K / K_PERCIPIO, K_AZURE, DATASET_DIR, YOLO_MODEL, TOOL_GEOMETRY
"""
from pathlib import Path

# ── 工具几何 (从 contracts 导入, 单一数据源) ──
from ..contracts import TOOL_GEOMETRY  # noqa: F401  # 重新导出供离线工具使用

# ── 本机 Percipio 彩色内参（640×480，棋盘标定）──
# 权威源: src/percipio_camera/config/color_camera_info.yaml — 改标定后请同步此处
K_PERCIPIO = {
    'fx': 466.174635,
    'fy': 465.556589,
    'cx': 326.071333,
    'cy': 244.789156,
    'width': 640,
    'height': 480,
}
# 本工作区默认内参别名（工具/回放缺省一律走本相机）
K = K_PERCIPIO
CALIBRATION_VERSION = (
    'percipio-640x480-chessboard|hand_eye:import_humble_20260128T114006')

# ── 仅历史 Azure Kinect 离线包（1280×720，FOV 推导）──
# 回放 Azure 数据集时显式传 --fx/--fy/--cx/--cy，勿作本机默认
K_AZURE = {'fx': 640.0, 'fy': 636.0, 'cx': 640.0, 'cy': 360.0}

# ── 数据集 / 模型路径（相对 peach_pose_ros2 包根：.../src/peach_pose_ros2/）──
_PKG_ROOT = Path(__file__).resolve().parent.parent.parent.parent
DATASET_DIR = _PKG_ROOT / 'data' / 'dataset'
YOLO_MODEL = _PKG_ROOT / 'model' / 'best.pt'
MODEL_VERSION = (
    'yolo:6981750db67a726e|mobile_sam:6dbb90523a35330f')
