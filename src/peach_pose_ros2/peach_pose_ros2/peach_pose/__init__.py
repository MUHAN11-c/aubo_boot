"""
PeachPose 桃姿 — 桃树果实 RGB-D 位姿感知算法包。

职责: 将 Azure Kinect RGB-D 观测转为 2D/3D 桃子袋体抓取位姿；本包为算法核心，
GUI 入口见 ``inspector/``。

收敛后的唯一感知路线:
  RGB-D → YOLO 检测 → MobileSAM 分割 → SAM掩膜 ∩ 膨胀深度连通域 (hybrid_dilated)
  → 仅用传感器实测深度做几何拟合与工具安全门控 → 三态输出

核心理论要点:
  - pinhole 相机模型: 深度 mm → 米 (/1000)，反投影 X=(u-cx)·Z/fx
  - 三态门控: ACCEPT / REOBSERVE / REJECT (pipeline.py 安全管线)
  - 抓取坐标系: Zg=袋体轴线(底→颈), Xg=宽度主方向, Yg=Zg×Xg (右手系)
  - 学习式/补全深度不参与几何与安全判定；SAM 缺失显式 REOBSERVE，禁止静默回退

模块:
  contracts.py     — 数据合约 (BagObservation, BagGrasp2D, BagGraspReference3D, ToolGeometry)
  inference.py     — InferenceEngine: YOLO + SAM 懒加载，CUDA 调用线程安全
  candidates.py    — CandidateEstimator: 收敛前景模式的统一入口
  pipeline.py      — 单目标实测深度几何与工具安全门控
  validation.py    — 离线真值加载与指标累计
  e2e_validate.py  — 批量端到端验证与 JSON/Markdown 报告
  depth.py         — Open3D 点云转换 (可视化用)
  visualization.py — Open3D 3D 可视化 (QTimer 非阻塞窗口)
  inspector/       — PySide6 GUI 数据集浏览器

使用:
  from peach_pose.candidates import CandidateEstimator
  from peach_pose.contracts import BagObservation

  estimator = CandidateEstimator()
  obs = BagObservation(rgb=..., depth=..., camera_K=..., detections=[...])
  results = estimator.estimate_modes(obs, target_id, bbox, sam_mask)

启动 GUI: ``./run.sh`` 或 ``python3 -m peach_pose.inspector.main_window``
"""
