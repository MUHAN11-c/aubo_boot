# peach2_target_model

Peach v2 目标模型：把感知的单帧观测按目标、按视点融合成带 95% 不确定度的袋模型，按“目标 × 刀具”算
`GraspDecision`（只授权，不下发任何运动或 IO），并提供 `ObserveTarget` 等收敛 / 近距颈复测动作。
纯逻辑在 `decision.py` / `store.py` / `params.py`（零 ROS，pytest 覆盖）；`target_model_node.py` 只做接线。

## 接口

| 名称 | 类型 | 方向 | QoS / 说明 |
|------|------|------|-----------|
| `/peach/perception/observations` | `peach2_interfaces/TargetObservationArray` | 订 | reliable, volatile, depth 10 |
| `/peach/target_model/models` | `peach2_interfaces/TargetModelArray` | 发（LifecyclePublisher） | reliable, transient_local, depth 1；内容变化或 epoch 清空时发 |
| `/peach/target_model/get_decision` | `peach2_interfaces/GetDecision` | 服务端 | 同步、只读快照 |
| `/peach/target_model/observe` | `peach2_interfaces/ObserveTarget` | 动作端 | M1：等收敛 / 颈复测 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发 | 目标数、收敛数、最近融合时刻/耗时、未收敛原因计数、丢弃原因计数（含 `camera_pose_invalid`）、决策 flag 计数（`fruit_top_proxy` / `swing_unknown`）、摆幅未知目标数、复测状态 |
| `/bond` | `bond/Status` | 发 | activate 起、deactivate 停（Nav2 lifecycle_manager 同款；缺 bondpy 时降级 WARN） |

话题 / 服务 / 动作名固定在代码里，不是参数（改名用 launch remap）。

回调组：观测订阅（MutuallyExclusive）、`get_decision`（MutuallyExclusive）、`observe`（Reentrant，目标在条件变量上等待）；
`MultiThreadedExecutor(4)`。融合在锁外跑，锁内只做“取快照 / 提交结果”，服务不会被融合阻塞。

## 观测接收与融合

只接收 `confirmed == true`、`category == CATEGORY_BAG`、底和颈都 `valid`、协方差有限且正定、`frame_id == base_link`、
`camera_pose` 有限且四元数模长在 [0.9, 1.1]（全零默认值 = 生产方没填，拒收）的观测；其余按原因计数进 `/diagnostics`（`dropped.*`）。按 `target_id` 分组；`scene_epoch` 变化清空全部历史、模型和复测状态
（`model_revision` 计数跨 epoch 不清零，保证 `(target_id, revision)` 在一次运行内唯一）。

**视点分段**：同一相机位姿的连续帧高度相关，逐帧当独立视图会让 MAD 项随帧数虚假收缩、`n_views` 不动相机就到上限。
因此用 `TargetObservation.camera_pose`（相机光学系在 base_link、图像时刻）分段：与当前视点**首帧**相比，相机平移 ≤
`view_translation_change_m` 且旋转角 ≤ `view_rotation_change_deg`，并且与上一帧时间间隔 ≤ `view_gap_s`，才归入同一视点；
任一超阈值即开新视点。该视点的帧聚合为一个
`ViewSample`（分量中位数位置 + 中位迹的单帧协方差），再交给 `peach2_core.fusion.fuse_views`。`TargetModel.n_views` = 视点数。

**摆幅**：只用 `swing_known == true` 的观测（它们的上报幅值 + 最新视点内这些帧的底点位置正弦拟合），
`max(窗口内上报幅值, 拟合幅值)`，窗口 `swing_window_s`；`swing_known == false` 的观测完全不参与摆幅（仍参与几何融合）。
窗口内没有任何 swing_known 帧时摆幅未知：`TargetModel.swing_amplitude_m = NaN`。

**果顶**：`TargetModel.fruit_top_offset_m`（颈沿 −axis 到果顶）。现有观测没有任何果实证据字段，暂时恒为 NaN（未知）。
**枝方向**：`branch_direction_known = false`、`branch_direction` 置零（留给后续），manipulation 按全周 roll 处理。

**model_revision**：每目标严格递增；只有内容变化（底/颈/结点位移 > `revision_position_tol_m`、d95/长度/σ/摆幅变化 >
同一容差、摆幅/果顶未知↔已知切换、轴角或 θ95 变化 > `revision_angle_tol_deg`、视点数或收敛状态改变）才递增。复测结果不改 revision
（避免 manipulation 在同一周期内看到 MODEL_STALE）。

## GraspDecision 规则

`GetDecision(target_id, tool_id, min_model_revision)`：

- `found=false`：节点未激活、无该目标模型、`tool_id` 为空或刀具文件读不到/非法、模型 revision < `min_model_revision`（非 0 时）。
- 刀具：`ToolGeometry` 读 `description_config_dir/<tool_id>.yaml`，`ToolCalibration` 读 `calibration_dir/<tool_id>.yaml`，按 tool_id 缓存。
- 预算：`peach2_core.budget.evaluate_budget(model, tool, calib, swing, cut_to_fruit)`，其中
  `cut_to_fruit = fruit_top_offset_m`（已知时）；NaN 时回退旧近似 `length_m − d95_m`（果贴袋底、果径 ≈ 袋 d95），
  并在决策 flag 里记 `fruit_top_proxy`（计入 `/diagnostics`）。摆幅未知时预算按 0 计并记 `swing_unknown`。

许可链（后一级蕴含前一级）：

| 许可 | 条件 |
|------|------|
| `approach_allowed` | 模型有效 **且** 收敛 **且** 未过期（最后观测距今 ≤ `validity_s`）**且** 摆幅已知（`require_known_swing` 为真时）且 ≤ `swing_max_m` **且** 非径向结构性不可套（`radial_structural`）**且** `radial_margin_m ≥ −approach_radial_slack_m` |
| `sleeve_allowed` | `approach_allowed` **且** `budget.sleeve_ok`（径向余量 > 0） |
| `cut_allowed` | `sleeve_allowed` **且** `budget.cut_ok`（标定 `calibrated`、轴向余量 > 0、果距够、无结构性问题）**且** 无颈复测不符 **且**（`cut_requires_remeasure` 为真时）颈复测已通过 |

“径向余量不严重为负”的含义：预抓取位在袋底下方 `pregrasp_standoff_m`，还没套入；余量在 `[−slack, 0]` 内允许去预抓取位
（近距复测可能让模型收紧），但不允许套入。

`failure_code` / `reason` 取第一个失败层级：`MODEL_NOT_CONVERGED(20) model_invalid|model_not_converged` →
`MODEL_EXPIRED(22)` → `SWING_TOO_LARGE(27) swing_unknown|swing_too_large` → `BUDGET_STRUCTURAL(25) radial_structural` →
`BUDGET_RADIAL_NEGATIVE(23) radial_margin_below_approach_slack` → 预算码（23/24/25，含 `calibration_pending`）→
`NECK_REMEASURE_MISMATCH(26) neck_remeasure_mismatch` / `NECK_REMEASURE_PENDING(28) neck_remeasure_pending`。全部许可时 `0 / ok`。

几何字段：

- `pregrasp_tcp.position = bottom − pregrasp_standoff_m · axis`；`orientation` = 把 TCP +Z 转到 `axis` 的最短弧四元数
  （只定位置 + Z 对齐，绕 Z 的 roll 由 peach2_manipulation 决定）。
- `blade_target = neck`。
- `insert_travel_m = (tcp_target_for_blade(neck, axis, tool) − pregrasp) · axis = length + L_blade + standoff`。
- `valid_until = now + validity_s`；`header.frame_id = base_link`。
- 模型无效/过期时三个许可全 false、几何字段置零、余量为 NaN。

## ObserveTarget（M1）

M1 不动相机：视点运动由 peach2_task 负责（task 先 `MoveTo` 视点再调本动作）。**M2 加 NBV 视点选择与内部 `MoveTo`。**

- `neck_remeasure=false`：在 `observe_timeout_s` 内等待模型 `converged` 或 `n_views ≥ max_views`（`max_views=0` 用
  `default_max_views`）；每次 revision 变化发 feedback（n_views、σ）。收敛 → succeed `NONE`；到视点上限未收敛或超时 →
  abort `MODEL_NOT_CONVERGED`；超时且从无模型 → abort `PERCEPTION_NO_TARGET`；取消/节点失活 → `CANCELED`。
- `neck_remeasure=true`：以目标开始时刻为界，只接受**图像 stamp 晚于 goal 开始**、相机距离 ≤ `remeasure_max_camera_distance_m`
  的新观测；凑够 `remeasure_min_frames` 帧后取颈位置分量中位数，与 goal 开始时融合模型的颈比较 3D 距离：
  ≤ `remeasure_mismatch_m` → 复测通过（succeed），> → `NECK_REMEASURE_MISMATCH`（abort），该目标 `cut_allowed` 在本 epoch
  内保持 false（mismatch 不可被后续通过覆盖）。超时：有新观测但不够近/不够帧 → `PERCEPTION_LOW_QUALITY`，完全无新观测 →
  `PERCEPTION_NO_TARGET`。复测帧本身也是普通观测，照常参与融合。

## 参数

`config/target_model.yaml`（ROS 参数只有 `config_file` 指向它）。直读 + 显式校验：每个键必填、未知键拒绝、类型与范围
越界拒绝 configure（`TransitionCallbackReturn.FAILURE`）。目录值支持 `$(find-pkg-share <pkg>)`，configure 时检查目录存在。

| 键 | 默认 | 含义 |
|----|------|------|
| `validity_s` | 120 | 决策有效期 / 模型过期阈值 [s] |
| `pregrasp_standoff_m` | 0.03 | 预抓取位在袋底下方的距离 [m] |
| `converge_sigma_lateral95_m` / `converge_theta95_deg` / `converge_sigma_axial95_m` | 0.006 / 4.0 / 0.010 | 收敛门（严格 <） |
| `min_views` | 1 | 收敛所需最少视点 |
| `default_max_views` / `max_views_per_target` | 3 / 8 | ObserveTarget 默认上限 / 每目标保留视点数 |
| `observe_timeout_s` | 30 | ObserveTarget 超时 [s] |
| `view_gap_s` / `view_translation_change_m` / `view_rotation_change_deg` / `max_frames_per_view` | 0.8 / 0.05 / 10.0 / 30 | 视点分段（camera_pose） |
| `remeasure_mismatch_m` / `remeasure_max_camera_distance_m` / `remeasure_min_frames` | 0.010 / 0.30 / 2 | 颈复测 |
| `cut_requires_remeasure` | true | 剪切前必须复测通过 |
| `swing_max_m` / `swing_window_s` | 0.015 / 5.0 | 摆幅上限与窗口 |
| `require_known_swing` | true | 摆幅未知时拒绝 approach（`SWING_TOO_LARGE` / `swing_unknown`，策略=等待）；false 时按 0 计 |
| `approach_radial_slack_m` | 0.005 | approach 允许的径向负余量 |
| `revision_position_tol_m` / `revision_angle_tol_deg` | 0.0005 / 0.1 | revision 递增阈值 |
| `description_config_dir` / `calibration_dir` | find-pkg-share 路径 | 刀具几何 / 标定结果目录 |

## 运行

```bash
ros2 launch peach2_target_model target_model.launch.py            # 未配置，交给 lifecycle manager
ros2 launch peach2_target_model target_model.launch.py autostart:=true   # 自己 configure + activate
```

`autostart` 只驱动本节点的 lifecycle 迁移，不触发任何运动 / IO / RunHarvest。

## 怎么测

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
timeout 1200 colcon build --base-paths src/peach2 --packages-up-to peach2_target_model \
  --packages-skip peach2_interfaces \
  --build-base build/v2/peach2_target_model --install-base build/v2/peach2_target_model_install
timeout 1200 colcon test --base-paths src/peach2 --packages-select peach2_target_model \
  --build-base build/v2/peach2_target_model --install-base build/v2/peach2_target_model_install
colcon test-result --test-result-base build/v2/peach2_target_model --verbose
```

pytest 只测纯逻辑（`test_params.py`、`test_decision.py`、`test_store.py`）+ flake8 / pep257；不调用 `rclpy.init()`。
节点接线（lifecycle、QoS、服务/动作往返）留给 launch_testing（isolated domain），见 TODO。

## 接口需求

接口变更 01（2026-09-30）已满足：`TargetObservation.camera_pose` / `swing_known`、`TargetModel.fruit_top_offset_m` /
`branch_direction*`、`FailureCode.NECK_REMEASURE_PENDING`。仍待定：

- 果实证据：`TargetObservation` 仍无果顶关键点 / 果径字段，`fruit_top_offset_m` 只能发 NaN、决策走 `length − d95` 近似。
- `TargetModel.tie` 没有独立协方差来源：结点暂用颈的 1σ 协方差填充。
- `TargetObservationArray.locked_target_ids` 本节点暂未使用（锁定集合由 task 侧消费）。

## 已知限制与 TODO(M0)

- TODO(M0)：收敛门、`swing_max_m`、`approach_radial_slack_m`、`remeasure_mismatch_m` 均为方案设计值，需台架/田间 bag 回放标定。
- TODO(M0)：`cut_to_fruit` 代理（果贴袋底、果径 = d95）需真袋验证；感知给出果实证据后填 `fruit_top_offset_m`。
- TODO(M0)：视点分段阈值（`view_translation_change_m` / `view_rotation_change_deg` / `view_gap_s`）需用真机停走节拍核对。
- TODO：`branch_direction` 估计（枝条方向，供 bite/shear roll 约束）。
- TODO：launch_testing（isolated domain，mock 观测发布器）覆盖 lifecycle、latched 模型、GetDecision / ObserveTarget 往返。
- M2：ObserveTarget 内部 NBV 视点选择 + `MoveTo`（经 peach2_manipulation 命令门）。
