"""Inspector 子包 — PySide6 数据集浏览 GUI。

职责: 提供 ``PeachInspector`` 主窗口，用于离线浏览 RGB-D 数据集、运行收敛安全
管线 (SAM∩膨胀深度)、叠加 2D 检测/SAM 与 3D 点云/圆柱工具可视化。

在管线中的位置:
  支撑/GUI — 不参与在线抓取决策，而是调用 ``InferenceEngine`` 与
  ``CandidateEstimator`` 对单帧图像做完整管线演示与人工验收。

核心理论要点:
  - 惰性导入: 本 ``__init__`` 不加载 PySide6，避免 ``import peach_pose``
    时强依赖 GUI 栈
  - 相机光学系 (ROS optical): Xc 右、Yc 下、Zc 前；与 contracts 中 3D 位姿一致
  - 安全管线输出 ``BagGraspReference3D``：圆柱套入位姿 (entry_start, translation_direction)

主要对外 API:
  - ``main_window.PeachInspector`` — 主窗口 (按需 ``from .main_window import PeachInspector``)
  - ``main_window.main()`` — CLI 入口 ``python3 -m peach_pose.inspector.main_window``

详见 ``docs/architecture.md`` 启动与阅读顺序章节。
"""
# 按需: from .main_window import PeachInspector
