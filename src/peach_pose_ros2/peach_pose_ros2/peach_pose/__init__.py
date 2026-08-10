"""
PeachPose 桃姿 — 桃树果实 RGB-D 位姿感知算法包（在线算法核心）.

职责: 将 RGB-D 观测转为 2D/3D 桃子袋体抓取位姿；本包为纯算法核心，
不 import rclpy，ROS 面见上层 ``peach_pose_node.py`` 及同层模块
（tf_utils / conversions / visualization / cloud_utils）。

收敛后的唯一感知路线:
  RGB-D → YOLO 检测 → MobileSAM 分割 → SAM掩膜 ∩ 膨胀深度连通域 (hybrid_dilated)
  → 仅用传感器实测深度做几何拟合与工具安全门控 → 三态输出

核心理论要点:
  - pinhole 相机模型: 深度 mm → 米 (/1000)，反投影 X=(u-cx)·Z/fx
  - 三态门控: ACCEPT / REOBSERVE / REJECT (pipeline.py 安全管线)
  - 抓取坐标系: Zg=袋体轴线(底→颈), Xg=宽度主方向, Yg=Zg×Xg (右手系)
  - 学习式/补全深度不参与几何与安全判定；SAM 缺失显式 REOBSERVE，禁止静默回退

在线算法模块:
  contracts.py     — 数据合约 (BagObservation, BagGrasp2D, BagGraspReference3D, ToolGeometry)
  inference.py     — InferenceEngine: YOLO + SAM 懒加载，CUDA 调用线程安全
  candidates.py    — CandidateEstimator: 收敛前景模式的统一入口
  pipeline.py      — 单目标实测深度几何与工具安全门控
  fitting.py       — 法线估计 / 球 RANSAC / 圆柱 RANSAC 拟合原语
  depth_geometry.py — 深度图归一化（uint16 比例因子 / 32FC1 米制 → uint16 毫米）
  target_registry.py — TargetRegistry: 世界系目标身份记忆（跨帧稳定 target_id）

offline/ 子包（离线数据集评估/回放工具，不参与在线管线）:
  offline/e2e_validate.py — 批量端到端验证 CLI 与 JSON/Markdown 报告
  offline/validation.py   — 离线真值标注加载与指标累计
  offline/config.py       — 离线工具全局配置（内参 K_PERCIPIO/K_AZURE、数据集路径）
  offline/sphere_ref.py   — 球拟合可视化参考（仅人工验收，不进安全判定）

使用:
  from peach_pose.candidates import CandidateEstimator
  from peach_pose.contracts import BagObservation

  estimator = CandidateEstimator()
  obs = BagObservation(rgb=..., depth=..., camera_K=..., detections=[...])
  results = estimator.estimate_modes(obs, target_id, bbox, sam_mask)

离线评估 CLI: ``python -m peach_pose_ros2.peach_pose.offline.e2e_validate``
"""
