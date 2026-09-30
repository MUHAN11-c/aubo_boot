# peach2_core

Peach v2 纯算法核（ament_python，**零 ROS 依赖**）：地标提取、深度反投影、3D 多目标跟踪、摆动估计、
多视融合、刀具几何/标定读取、套入/剪切预算、失败码镜像。所有节点包（`peach2_perception`、
`peach2_target_model` 等）只做 I/O，业务判断都在这里，便于 pytest 不起 DDS。

依赖：numpy 1.26.4、scipy 1.11.4、PyYAML（Jazzy apt / 系统 python3）。坐标系约定：3D 全部 `base_link`，
重力 `(0, 0, -1)`；刀具约定 `tcp = neck + L_blade · axis`（刀口平面在 TCP 后 L_blade）。

## 公有 API

| 模块 | 符号 |
|------|------|
| `types` | `Landmark`、`Landmarks2D`、`Landmarks3D`、`invalid_landmark()`、`failed_landmarks2d(flags)`、`failed_landmarks3d(flags)`、`unit(v)`、`axial_lateral_cov(axis, σ_lat, σ_ax)` |
| `landmarks2d` | `landmarks_from_mask(mask, gravity_px, n_bins=20) -> Landmarks2D` |
| `landmarks3d` | `landmarks_from_points(points, gravity, axis_hint=None, sigma_point_m=None, n_bins=12, taper_ratio=0.85) -> Landmarks3D` |
| `depth` | `depth_sigma_m(z_m, fx_px, baseline_m, disparity_sigma_px=0.25)`、`backproject(u, v, depth_m, K, confidence=None, win=1) -> (p, cov)`、`mask_points(mask, depth_m, K, confidence=None, min_conf=0.5, stride=1)` |
| `tracker` | `Detection3D`、`Track`、`Tracker(confirm_hits=3, ttl_s=3.0, gate_chi2=11.34, process_accel_sigma=0.5, id_prefix='target_')`：`update(stamp_s, detections)`、`tracks()`、`reset()` |
| `swing` | `estimate_swing(times_s, positions) -> (peak_amplitude_m, period_s)`（不足返回 `nan, nan`） |
| `fusion` | `ViewSample`、`FusedModel`、`fuse_views(views, min_views=1) -> FusedModel` |
| `tool` | `ToolGeometry`、`ToolCalibration`、`CALIBRATED`、`load_tool_geometry(tool_id, description_config_dir)`、`load_tool_calibration(tool_id, calibration_dir)` |
| `budget` | `BudgetResult`、`radial_structural(tool, calib)`、`axial_structural(calib)`、`evaluate_budget(model, tool, calib, swing_amplitude_m=0, cut_to_fruit_m=inf)`、`tcp_target_for_blade(neck, axis, tool)` |
| `codes` | `FailureCode.msg` 全部常量（`NONE`…`DEPENDENCY_UNAVAILABLE`，含接口变更 01 的 28/45/65）+ `all_codes()` |

`_` 前缀符号为私有。

## 算法要点

- **landmarks3d**：PCA 主轴（病态时用 hint/重力），轴限制在 −重力 45° 锥内；径向离群（中位数 + 4 倍稳健 σ）
  剔除飞点；分段圆拟合（Kasa 初值 + Tukey 加权几何 Gauss-Newton，弧覆盖 ≥ 3/12 扇区才采信）→ Tukey 重加权
  直线拟合轴线，迭代两次；径向剖面 p95 半径中值平滑；颈 = 上半段最窄且 < 0.85·袋身半径的平台起点，
  再以 1/8 bin 步长细搜；底 = 轴向 q05 外推；σ 由直线残差与点噪声传播。极性由重力决定（锥度矛盾时保留重力并打 flag）。
- **landmarks2d**：同结构的 2D 版本（宽度剖面 p2–p98 + 中值平滑 + 平台细搜），极性重力优先，锥度补充。
- **tracker**：每目标常速 KF（Joseph 形式更新），门限用创新协方差 `P_pred + R` 的 Mahalanobis χ²(3) 0.99=11.34，
  类别约束 + `scipy.optimize.linear_sum_assignment`；确认 = 连续命中 `confirm_hits`，TTL 按时间（秒）；`reset()` 不清 ID 计数。
- **swing**：[1, t, sin, cos] 联合最小二乘（去趋势），频率网格 + 有界一维搜索，幅值 = 三轴系数矩阵最大奇异值。
- **fusion**：底/颈 Huber IRLS 均值、轴 Huber 角均值；`σ95 = max(MAD·1.4826, 单视协方差传播) · 1.96`，
  横向用底+颈横向残差，轴向用颈轴向残差，θ 用轴散布与 `hypot(σ_lat_b, σ_lat_n)/L` 的较大者。
- **budget**：方案 §5.1 RSS（独立 95% 项平方和开方）。摆幅 A 方向未知，径向/轴向都计入。
  `status != calibrated` 时 `cut_ok=False`（码 25 `calibration_pending`）。
- **tool**：几何只读 `aubo_description/config/<tool_id>.yaml`，误差常数只读 `peach2_calibration/results/<tool_id>.yaml`；
  代码里没有刀具默认值，缺键/非法值直接抛 `ValueError`。

## 怎么测

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
timeout 1200 colcon build --base-paths src/peach2 --packages-select peach2_core \
  --build-base build/v2/peach2_core --install-base build/v2/peach2_core_install
timeout 1200 colcon test --base-paths src/peach2 --packages-select peach2_core \
  --build-base build/v2/peach2_core --install-base build/v2/peach2_core_install
colcon test-result --test-result-base build/v2/peach2_core --verbose
```

测试全部为合成数据（`test/synthetic.py`：带噪声/飞点的截锥袋 + 颈柱点云、合成掩膜）；`test_codes.py` 逐行对照
`peach2_interfaces/msg/FailureCode.msg`；`test_budget.py` 表驱动三套刀具 profile，并锁定“现行常数下线性和为负、RSS 为正”。

## 接口需求

- 无（本包不接触 IDL；`codes` 以测试对齐 `FailureCode.msg`）。

## 已知限制与 TODO(M0)

- TODO(M0)：`landmarks3d` 合成精度（底 < 5 mm、颈轴向 < 3 mm、轴 < 0.3°）来自“截锥 + 颈柱”模型；纯截锥无颈柱时
  颈会落在锥度平台起点（比真颈低约 3–4 cm）。需用台架真袋点云复核 `taper_ratio` 与平台规则。
- TODO(M0)：`depth.depth_sigma_m` 的视差 σ 与 Percipio 实测深度噪声曲线对齐。
- TODO(M0)：`tracker` 过程噪声 `process_accel_sigma` 与门限用真实风况 bag 标定。
- TODO(M0)：`budget` 中袋变形余量（profile `bag_deformation_margin95`）暂未计入 RSS，台架确认后决定是否加项。
