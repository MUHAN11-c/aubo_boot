# peach2_perception

Peach v2 单帧感知：RGB-D 帧 → YOLO（`peach_bag` / `peach_nobag`）→ MobileSAM → 深度质量 → 2D/3D 袋底·袋颈·结点
→ 3D 跟踪 → `TargetObservationArray`（base_link）。只产出观测事实，不做接受/拒绝裁决（融合与预算在
`peach2_target_model`），不发任何运动或 IO，不含任何刀具几何。

纯逻辑全部在零 ROS 模块里（pytest 覆盖），`perception_node.py` 只做订阅 / 发布 / 参数 / lifecycle。

## 运行环境（必读）

推理依赖 torch / ultralytics，只装在工作区 venv `aubo_py3.12`（numpy==1.26.4，不得升级）。`setup.py` 与旧
`peach_harvester` 同法：构建时若 `<ws>/aubo_py3.12/bin/python` 存在，安装的入口脚本
`lib/peach2_perception/perception_node` 的 shebang 就写成该解释器（venv 开了 `include-system-site-packages`，
rclpy / cv_bridge 仍解析到 Jazzy apt 副本）。因此：

- 必须在 venv 存在的机器上 `colcon build`，否则 shebang 回落到构建用的 python，节点 configure 时 `import ultralytics` 失败（FAILURE，不静默）。
- 默认 `detector.device: cuda:0`、`allow_cpu: false`：无 GPU 时 configure 失败，不会静默退到 CPU（旧栈的坑）。
- 模型默认 `$(find-pkg-share peach_harvester)/model/{best.pt,mobile_sam.pt}`，configure 时经 ament_index 解析；
  也可写绝对路径。`detector.engine_path` 配置且文件存在时优先 TensorRT engine。

## 接口

| 名称 | 类型 | 方向 | QoS / 说明 |
|------|------|------|-----------|
| `/camera/color/image_raw` | `sensor_msgs/Image` bgr8 | 订（同步） | `sensor_qos_reliable` 决定 RELIABLE / BEST_EFFORT，depth 5 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` 16UC1/mono16/32FC1 | 订（同步） | 须配准到彩色网格；uint16 × `depth_unit_m` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 订（同步） | K 取自此；D 非零时打 `distorted_intrinsics` 并 WARN |
| `/camera/depth/confidence` | `sensor_msgs/Image` mono8/float | 订（可缺） | 按 stamp 匹配（`confidence_match_tol_s`）；缺失时用深度梯度代理置信度 |
| `/joint_states` | `sensor_msgs/JointState` | 订 | BEST_EFFORT depth 100；判静止 |
| `/tf`、`/tf_static` | | 订 | 只按图像 stamp 查 `base_link ← camera_frame`，超时 `tf_timeout_s`；失败丢帧计数，**无 latest 回退** |
| `/peach/perception/observations` | `peach2_interfaces/TargetObservationArray` | 发（LifecyclePublisher） | reliable, volatile, depth 10；header=图像 stamp + base_link；`locked_target_ids` 锁定前为空、锁定后为排序后的冻结集合 |
| `~/debug_image` | `sensor_msgs/Image` bgr8 | 发（可选） | `debug.publish_image`；best effort depth 1 |
| `/peach/perception/begin_scene` | `peach2_interfaces/srv/BeginScene` | 服务端 | 清空跟踪、`scene_epoch += 1`、开始收集锁定；回 `accepted`、`scene_epoch`；非 active 时 `accepted=false` 并回当前 epoch；`request_id` 只记日志 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发 | fps、各原因丢帧、TF 失败、分段耗时 EMA、锁定状态/epoch、设备、置信度来源、臂运动状态 |
| `/bond` | `bond/Status` | 发 | activate 起、deactivate 停（缺 bondpy 时降级 WARN） |

话题 / 服务名固定在代码里，不是参数（改名用 launch remap）。

## 数据流

```
color+depth+camera_info ──ApproximateTime(slop 0.05)──▶ LatestSlot(容量1, 丢最旧) ──▶ 推理线程
confidence(缓存, 按 stamp 配) ─┘                              joint_states ─▶ JointMotionBuffer
推理线程：
  TF@stamp(0.2 s，失败丢帧) → depth_to_metres → 有效量程 → 置信度(发布的或代理) → 可信深度
  → YOLO(单阈值 + 跨类去重) → 袋框优先排序 → MobileSAM(每帧编码一次，所有框共享 embedding；
     外扩框 + 中心正点 + 4 外角负点) → 掩膜 ∩ 可信深度 → 去深度跳变 → 取含框中心的连通域 → 开/闭
  → 每实例：2D 地标(重力=base −Z 经 TF 投影) · 3D 地标(掩膜点 → 相机系 → base_link，协方差在 base_link)
     · 2D↔3D 交叉校验(袋底/袋颈/结点各自校验，>15 mm 标 invalid；只有袋底/袋颈失配才放大锚点协方差)
     · 跟踪锚点(3D 袋底；退化到 2D 袋底 / 框中心)
  → [状态锁] Tracker(按图像 stamp) · 静止窗口内摆动估计 · 臂运动中则地标置 invalid · LockPolicy
  → TargetObservationArray(scene_epoch, target_set_locked, locked_target_ids；每条观测带同一 camera_pose = 图像时刻 TF)
     + 可选 debug 图
```

回调组：同步三路 + 置信度（MutuallyExclusive）、joint_states（MutuallyExclusive）、begin_scene（MutuallyExclusive）；
`MultiThreadedExecutor(4)`。回调里不做推理；推理线程与 begin_scene 共享一把状态锁，推理期间 epoch 变化的帧整帧丢弃；
`LockPolicy.accepts()` 拒收 stamp 早于 BeginScene 时刻的帧。

锁定规则（`lock_policy.py`）：连续静止帧 ≥ `min_stationary_frames` 且确认集合（臂静止下）连续 `stable_s` 不变、无待确认
轨迹、集合非空 → 锁定（reason `stable`）；收集超过 `max_collect_s` 无条件锁定（可为空集，reason `timeout`）。锁定后集合
冻结到下一次 begin_scene；观测照常发布，锁定集合外的确认目标带 `not_in_locked_set` 旗标。

## 参数（`config/perception.yaml`，全部必填，未知键拒绝）

| 键 | 默认 | 说明 |
|----|------|------|
| `frame_id` | base_link | 输出系，必须 base_link |
| `camera_frame` | camera_color_optical_frame | 配准深度/彩色光轴所在系；空=深度图 header.frame_id |
| `depth_unit_m` | 0.00025 | uint16 深度单位；stereo 0.25 mm，Percipio 0.001 |
| `min_depth_m` / `max_depth_m` | 0.3 / 1.5 | 有效量程 |
| `min_confidence` | 0.5 | 可信深度下限（0..1） |
| `confidence_match_tol_s` | 0.005 | 置信度图与深度 stamp 的匹配容差 |
| `sync_slop_s` / `sync_queue_size` | 0.05 / 10 | ApproximateTime |
| `sensor_qos_reliable` | false | BEST_EFFORT 可连 RELIABLE/BEST_EFFORT 发布者，不回压大图 |
| `tf_timeout_s` | 0.2 | 按 stamp 查 TF 的等待 |
| `depth_noise.*` | 0.0622 m / 549 px / 0.25 px | 立体深度噪声 σz = z²σd/(f·b)，进 3D 地标协方差 |
| `surrogate_confidence.jump_rel_lo/hi` | 0.01 / 0.04 | 代理置信度：相对邻域深度跳变线性映射 1→0 |
| `detector.*` | best.pt, cuda:0, allow_cpu false, 640, conf 0.35, iou 0.5 | 单阈值；`dedup_ios` 0.6、碎片规则 0.2/0.5；`class_names` 必须与权重一致 |
| `segmenter.*` | mobile_sam.pt, 1024, max_boxes 16 | `box_expand_frac` 0.10、`neg_point_offset_px` 8、`depth_jump_rel/abs` 0.03/0.01 m、`seed_frac` 0.34、`morph_kernel_px` 5、`min_area_px` 100、`segment_nobag` false |
| `landmarks.*` | 20 / 12 bins, taper 0.85 | `cross_check_tol_m` 0.015、`cross_check_inset_px` 3、`backproject_win` 1、`point_stride` 1 |
| `tracker.*` | 3 / 3.0 s / 11.34 / 0.5 | 确认命中数、TTL（按 stamp 秒）、χ² 门、过程加速度 σ |
| `stationary.*` | 0.02 rad/s / 0.2 s / 0.1 s / 5 s | 阈值、回看窗口、joint_states 与图像 stamp 最大间隙、缓冲时长 |
| `swing.*` | 2.0 s / 1.0 s | 摆动估计窗口、最短跨度 |
| `lock.*` | 10 帧 / 2.0 s / 25 s | 见锁定规则 |
| `debug.*` | false / 0.5 | debug 图开关与缩放 |

阈值在 configure 时校验（范围、交叉约束如 `min_depth < max_depth`、`imgsz` 倍数、核为奇数），非法即 FAILURE。

## 公有 Python API（纯模块，零 ROS）

- `depth_quality`：`depth_to_metres`、`valid_depth_mask`、`normalise_confidence`、`surrogate_confidence`、`depth_edges`、`confident_depth`、`mask_depth_coverage`
- `detector`：`Detection`、`Detector`（`detect`、`check_class_names`）、`UltralyticsYoloBackend`、`clip_box`、`box_iou`、`dedup_overlapping`、`weights_path`、`resolve_device`
- `segmenter`：`Segmenter.segment`、`MobileSamBackend`、`build_prompt`、`expand_box`、`refine_mask`、`RefineParams`、`MaskResult`、`Prompt`
- `stationary`：`JointMotionBuffer`（`add`、`speed_at`、`classify`）、`Motion`、`JOINT_NAMES`
- `observation_builder`：`ObservationBuilder`（`measure`、`associate`、`track_summary`、`reset`）、`FrameData`、`Measurement`、`ObservationRecord`（含 `swing_known`）、`BuilderParams`、`gravity_in_image`、`cross_check_landmark`、`transform_matrix`、`pose_from_matrix`、`transform_points`
- `msg_conversion`（只依赖消息包，不需 `rclpy.init`）：`observation_array_msg`、`observation_msg`、`bag_landmark_msg`、`pose_msg`、`NOT_IN_LOCKED_SET`
- `lock_policy`：`LockPolicy`（`begin_scene`、`accepts`、`update`、`status`）、`LockState`、`LockStatus`
- `frame_pipeline`：`FramePipeline.run`、`FrameInput`、`FrameResult`、`builder_params`
- `worker`：`LatestSlot`；`debug_view`：`draw_debug`；`params`：`load_params`、`params_from_dict`、`PerceptionParams`

## v2.1 关键点升级位

v2.1 用学习的关键点模型替换 `Detector + Segmenter + landmarks_from_mask`：新后端直接给出每实例框、（可选）掩膜与
2D 袋底/袋颈/结点像素。替换点是 `FramePipeline.run` 里的「检测 → 分割」段与 `ObservationBuilder._bag_geometry` 里的
`landmarks_from_mask` 调用：产出同样的 `Landmarks2D`（关键点置信度填 `confidence`），3D 地标、交叉校验、跟踪、锁定与
消息**全部不变**；BagLandmark.source 改填 `SOURCE_KEYPOINT`。输出契约（`TargetObservationArray`）不变。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --base-paths src/peach2 --packages-select peach2_perception \
  --packages-skip peach2_interfaces peach2_core --build-base build/v2/peach2_perception --install-base build/v2/peach2_perception_install
colcon test  --base-paths src/peach2 --packages-select peach2_perception \
  --packages-skip peach2_interfaces peach2_core --build-base build/v2/peach2_perception --install-base build/v2/peach2_perception_install
colcon test-result --test-result-base build/v2/peach2_perception --verbose
```

pytest 用系统 python 跑纯模块（合成 z-buffer 渲染的吊袋场景 + 假模型后端）；`test_model_smoke.py` 在 venv 与权重
存在时用 venv 解释器在 CPU 上跑真 best.pt + mobile_sam.pt，否则 skip。没有任何测试调 `rclpy.init()`。

运行：`ros2 launch peach2_perception perception.launch.py`（`autostart` 默认 false，交给 lifecycle manager）。

## 观测字段语义

- `camera_pose`：`camera_frame`（彩色光学系）在 base_link 下的位姿，取自按图像 stamp 查到的同一个 TF（与反投影所用的
  变换相同，无 latest 回退）；同一帧所有观测相同。
- `swing_known`：只有臂静止、锚点来自 3D 袋底、且静止窗口跨度 ≥ `swing.min_span_s` 并估计出有限值时为 true；
  false 时 `swing_amplitude_m / swing_period_s` 填 0，不再表示“不摆”。
- 结点 `tie` 的协方差独立于袋颈：来自 `peach2_core.landmarks_from_points` 在袋尖处的轴向（一个剖面 bin 宽）与侧向
  （轴线拟合在该点的外推误差）不确定度（合成场景里轴向 σ 约为袋颈的 2 倍）；结点有自己的 2D↔3D 交叉校验（2D 袋尖向内缩
  `cross_check_inset_px` 取深度），失配只把结点置 invalid（`tie_2d3d_mismatch`），协方差非有限时置 invalid（`tie_cov_invalid`）。

## 已知限制

- 假设图像已去畸变（stereo 前端输出矫正图）；`camera_info.D` 非零只打旗标 + 诊断 WARN，不做去畸变。
- 代理置信度只是深度梯度 / 飞点启发式；前端发布 `/camera/depth/confidence` 后自动改用真实置信度。
- 锚点来源会在 3D 袋底 / 2D 袋底 / 框中心之间退化，来源切换的帧可能让同一袋分裂成新轨迹（带 `anchor_*` 旗标）。
- 交叉校验沿射线的深度区间用 `[−tol, d95 + tol]`，对袋颈偏松（颈半径远小于 d95）；侧向偏差是主判据。
- `mask_quality` 的面积比（0.4）与贴边惩罚（0.5）是启发式常数，只用于排序/诊断，不做裁决。
- 立体左侧 ~15% 盲区里的袋没有深度：掩膜为空（`mask_no_depth`），只能靠框中心锚点显示，不产出地标。
- `max_boxes` 之外的框（袋优先排序后）不分割，带 `sam_truncated` 旗标并计入诊断。
- 本包未在本机起过 ROS 运行时（实现规范禁止）；节点接线需要后续 launch_testing（mock 相机/TF）与真机验证。
  GPU / TensorRT 路径未在本环境验证（沙箱无 CUDA），CPU 路径由 smoke 测覆盖。

## 接口需求

已由变更 01 落地：`camera_pose`、`swing_known`、`locked_target_ids`、`BeginScene` 返回 `scene_epoch`。仍待评估：

1. 臂运动状态只体现为旗标（`arm_moving` 帧的地标已置 invalid）；若融合端要区分，建议观测加 `uint8 motion_state`。
2. 掩膜不对外发布；若 peach2_scene 需要把袋体从障碍体素中扣除，需要约定掩膜/袋体包络的话题。

## 许可

BSD-3-Clause
