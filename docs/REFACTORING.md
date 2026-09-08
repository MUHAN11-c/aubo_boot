# 工程重构过程记录

**不是活文档。** 不驱动现行设计；现行架构/接口/怎么跑仍以 [architecture.md](architecture.md)、[io.md](io.md)、[testing.md](testing.md) 为准。本文件只记基线、文件映射与阶段结果。真机轮次仍只追加 [testing-log.md](testing-log.md)。

日期：2026-09-07。入口：`ros2 launch peach_executor harvest_system.launch.py hardware_mode:=mock camera_enabled:=false`（`QT_QPA_PLATFORM=offscreen`；RViz 无显示会刷 GLX 错并可能退出，不影响能力节点）。

---

## Baseline（改代码前）

### Lifecycle

| 节点 | 状态 |
|------|------|
| `/peach_scene_perception_node` | active |
| `/peach_target_reconstruction_node` | active |
| `/peach_manipulation_node` | active |
| `/peach_executor` | active |
| `/peach_observability` | active |

`/peach/lifecycle/managed_nodes_activated` = `true`。

### 调度闸门（param）

- `execute_pregrasp_only` = true
- `execution_enabled` = false
- `require_managed_stack` = true
- lifecycle `node_names` = 场景 → 重建 → 技能 → 调度；`startup_timeout_s` = 60
- `HarvestState`: `batch_state=waiting_ready`，`auto_start_enabled=false`，三档使能全关

### 能力节点（图名）

`peach_executor`、`peach_lifecycle_manager`、`peach_scene_perception_node`、`peach_target_reconstruction_node`、`peach_manipulation_node`、`peach_observability`。图上另有 MoveIt/`ros2_control` 驱动节点。`ros2 node list` 会把技能进程内同名实体列多次（lifecycle 壳 + MoveIt companion 参数覆盖），不是第二套技能节点。

### 采摘动作

- `/peach_executor/run_harvest`
- `/peach_manipulation_node/survey_scene`
- `/peach_manipulation_node/execute_target`
- `/peach_target_reconstruction_node/build_target_model`

无 `NavigateToWorksite`（预留、未接线）。

### 监控

`GET http://127.0.0.1:8090/api/state` → 200。键：`debug` `job` `manipulation` `metrics` `params` `perception` `reconstruction` `record` `refined` `robot` `system` `task_executor`。

### 清单 vs 真实订阅（基线漂移）

调度只订 `/peach/perception/target_observations` 与 `/peach/lifecycle/managed_nodes_activated`。清单仍把 `peach_executor` 写成下列 consumer（错误，Phase B 纠正）：

- `/peach/perception/initial_pose`
- `/peach/perception/diagnostics`
- `/peach/reconstruction/diagnostics`
- `/peach/reconstruction/grasp_decision`
- `/peach/reconstruction/refined_pose`
- `/peach/reconstruction/tsdf_cloud`
- `/peach/reconstruction/shape_hypothesis`

---

## Files mapping（旧 → 新）

包树不变。旧路径保留为 shim（`from .foo import *` + 模块级常量），避免 `from …capture import GATE_ALLOW` 一类旧 import 断裂。`*_node.py` 壳与 C++ `stages.cpp` 未拆。

### peach_perception/common

| 旧 | 新 |
|----|----|
| `common/runtime.py` | shim → `clock.py`、`bounded_worker.py`、`harvest_data.py`、`ema.py` |
| `common/geometry.py` | shim → `fitting.py`、`depth_geometry.py`、`tf_utils.py` |

### peach_perception/scene_perception

| 旧 | 新 |
|----|----|
| `identity.py` | shim → `harvest_plan.py`、`target_registry.py`、`anchor_memory.py` |
| `visualization.py` | 仍持绘制；并 re-export `conversions.py` / `cloud_utils.py` |

### peach_perception/target_reconstruction

| 旧 | 新 |
|----|----|
| `capture.py` | shim → `captured_frame.py`、`skip_codes.py`、`capture_gate.py`、`bind_holdoff.py`、`timing.py`、`frame_collector.py`、`mask_gate.py`、`auto_controller.py` |
| `integrate.py` | shim → `tsdf_volume.py`、`cloud_builder.py`、`icp_refiner.py`、`icp_target_cache.py`、`overlap.py`、`view_coverage.py` |
| `refine.py` | shim → `candidate_contract.py`、`geometry_refiner.py` |
| `publish.py` | shim → `publish_throttle.py`、`status_messages.py`、`session_io.py`、`publishers.py` |

### peach_executor

| 旧 | 新 |
|----|----|
| `batch.py` | shim → `select.py`、`control.py`、`summary.py`、`ledger.py` |
| `observability/recorder.py` / `state.py` | 编排仍在原文件；助手 → `codec.py`、`job.py`、`metrics.py` |

### 生成模块（gitignore，不入库）

构建期写入源码包，避免本机 `PYTHONPATH` 指向 `src/` 时挡住 `install/` 里的 GPL 模块：

- `peach_perception/{scene_perception,target_reconstruction}_parameters.py`（原有）
- `peach_executor/{executor,observability,lifecycle_manager}_parameters.py`（Phase C 对齐感知写法）

---

## Phase log

### Phase A — Completed

- mock 栈五节点 Active；话题/动作/服务名与图名一致。
- 未改业务代码。
- Remaining：当时列出的 P0/P1/拼接/GPL/纯核，见后续阶段。

### Phase B — Completed

- **P0** `grasp_hyp_pub_` 改为 `LifecyclePublisher`，`on_activate` / `on_deactivate` 与 status/markers 对齐。
- mock Active 后 `ros2 topic info -v /peach/manipulation/grasp_hypothesis`：1 个 publisher（`peach_manipulation_node`）、1 个 subscriber（`peach_observability`），QoS RELIABLE + TRANSIENT_LOCAL。无相机、未开批，接触几何阶段不跑，故 `topic hz` 无样本（报文只在 `ExecuteTarget` 写出入口几何时发出）。
- **P1** `interface_manifest.yaml`：调度 consumer 只留真实订阅；`initial_pose` / 重建诊断与模型话题的 executor 行删除；监控仍订的留给 `peach_observability`。
- `check_interface_manifest.py`：consumer 字面量弱校验；`peach_interfaces` `CMakeLists.txt` `add_test`。拆文件后 consumer 路径补上 `select.py` 等。反向扫描跳过 gitignore 的 GPL 生成 `*_parameters.py`（其 default_value 会引用节点内调试服务名，那些名字不进跨包清单）。
- io.md 图与 architecture 图 B：`initial_pose` 只到重建。
- 过时注释：场景不再提已删的 `observation_quality` / `segmentation_gate.py`；重建 docstring 指向 `integrate.IcpTargetCache` / `publish.PublishThrottle`。

### Phase C — Completed

- `config/lifecycle_manager_parameters.yaml` GPL（`node_names` 顺序与 `startup_timeout_s=60` 原样）；运行 `lifecycle_manager.yaml` 只覆盖。
- `lifecycle_manager.py` 走 `ParamListener`，不 `declare_parameter`。不加 bond。
- mock 复核：`node_names` = 场景 → 重建 → 技能 → 调度；超时 60；五节点 Active。
- 本机 `aubo_py3.12` 会把 `src/` 放进 `PYTHONPATH`。仅写 install 时 lifecycle/调度/监控在 import 期 `ModuleNotFoundError`。`setup.py` 对齐感知：GPL 同时写入源码包（gitignore）。

### Phase D — Completed

- 按已有 `# === foo.py ===` 缝拆 Python；shim 导出 **class/def 与模块级常量**（如 `GATE_ALLOW`、`STATE_IDLE`、`unit_vector`）。
- 每批 `colcon build --packages-select …` + flake8/pep257。
- 未拆：`scene_perception_node.py` / `target_reconstruction_node.py` / `executor_node.py` 壳、`stages.cpp`。
- 未改算法常数、yaml 默认值、QoS、图名。

### Phase E — Completed

- `peach_executor/test/test_harvest_fsm.py`：`RUN_REQUESTED→NAVIGATE`、`SURVEY_AT_POSE→BEGIN_SCENE`、`TARGET_SELECTED→DISPATCH`、`READY_FULL→EXECUTE_FULL`、`CYCLE_DONE→SURVEY`、未知组合保持原态。
- `peach_perception/test/test_runtime_core.py`：`ManualClock` 前进/拒负 dt；`BoundedWorker` capacity=1 `drop_oldest`。
- 决策 0006、`testing.md`、`AGENTS.md`、`CLAUDE.md`：test/ = lint **加** 零 ROS 纯核；仍禁 DDS 假现场 / gtest 业务测 / launch_testing / 采摘仿真。
- `colcon test`：peach_interfaces（含清单脚本）、peach_perception、peach_executor 纯核 + lint 通过。peach_manipulation cpplint copyright/include_order 为既有噪音，本轮不补版权头。

### Phase F — Completed

- Web 仍是内部驾驶舱：系统/批次/许可/TCP/错误/调试三重门（默认 `debug.enabled=false`）。未改 `web/` 现场未提交调参。
- `GET /api/state` → 200，键与基线相同。`job.flags` 三档使能全关。
- `require_managed_stack` 只看 `/peach/lifecycle/managed_nodes_activated`，不依赖 HTTP 8090。observability **不进** lifecycle 名单（自行 configure/activate）。`harvest_system.launch.py` 仍始终带上监控节点（不加「关掉 Web」launch 参数，避免变成产品开关）。
- 场景 RGB/深度转码 WARN 改为中文，带 `frame` + `stamp`；高频 callback 原有 throttle 保留。

### Phase G — Completed（只改文档，不改算法）

对照 architecture §8：`PublishThrottle` 已节流 `local_cloud` / `tsdf_cloud` / `markers`；状态三件套不节流；`IcpTargetCache` 已落地。§8 过时「六消息每帧重发」已改写。

mock 无相机、未开批（因此 `_process_rgbd` / TSDF 积分不跑）空闲采样（`/api/state` metrics，约 Active 后 4 min）：

| 进程 | CPU% | RSS MB |
|------|------|--------|
| peach_target_reconstruction_node | ~21 | ~182 |
| peach_observability | ~7 | ~100 |
| ros2_control_node | ~5 | ~59 |
| peach_scene_perception_node | ~1 | ~638（模型已加载） |
| peach_manipulation_node | ~0 | ~97 |
| peach_executor | ~0 | — |

无相机故无 `_process_rgbd` 墙钟。`_collect_bag_views` 每次 refit 仍重估 landmarks——无测量支撑，不改。未授权带相机 SURVEY_ONLY，本轮不做。

### Phase H — Completed

再跑 Phase A 清单，相对 Baseline：

| 项 | 结果 |
|----|------|
| 五能力节点 Active | 同 |
| `managed_nodes_activated` | true |
| `execute_pregrasp_only` | true |
| `execution_enabled` | false |
| lifecycle `node_names` 顺序与超时 | 同 |
| `HarvestState` waiting_ready / 不自动开批 / 三档关 | 同 |
| 四个采摘动作名 | 同；仍无 `NavigateToWorksite` |
| `/api/state` 键 | 同 |
| 图名（节点/话题/动作/服务） | 零变化 |
| GPL 默认值 | 零变化（lifecycle 只是迁入 GPL，默认等同原 `declare_parameter`） |

RViz offscreen GLX 失败与基线相同，能力节点不依赖它。

三份活文档已与源码对齐（图 B、参数分层、文件树、决策 0006、测试政策、lifecycle GPL）。本文件记录映射与阶段结果，不驱动设计。

---

## 2026-09-08 保行为修复轮（决策 0017）

入口同上（mock 冒烟五节点 Active）。包树零变化（无拆分/无新文件/无路径迁移）；图名、yaml 参数键、默认值零变化。文件映射按批次：

### B1 调度 bug（`peach_executor`）

| 文件 | 改动 |
|------|------|
| `executor_node.py` | `batch_paused` 事件条件 PAUSE_PENDING→PAUSED（原恒假，事件从未发出）；`_action_active` 早退路径清零（`_wait_result` None handle、`_wait_build_after_observe` 两个超时 return）；`_recovery_required` 两处读-改-写收进 `_lock`；ControlTask `reason` 暂存并写入 `batch_paused`/`batch_resumed`/`recovery_acknowledged` details；主循环 NONE 分支账本双写去重；「单槽 Build」反向注释改正；删恒空 `_blockers` |
| `observability/recorder.py` | 终局集 `{6,7,8}`→`{6,8}`（RECOVERY_REQUIRED 归入活动期，批内恢复不再提前结算/丢数据）+ 删重复赋值；`_open_batch` run_id 过 `_safe_run_component`（防穿越）；`_reason_text` 死条件、`build_target_rows` 死语句、summary「复扫轮数」死解析（round_started/completed 无生产方）删除 |
| `summary.py` | `elapsed_msg` 纳秒进位（对齐 ledger.set_elapsed） |
| `observability/debug_actions.py` | 删 `summary.harvested` 幽灵字段读取 |
| `observability/metrics.py` | 采样线程名 perception→observability |
| `web/` | 删轮次徽标（index.html 元素、app.js 解析、app.css 样式） |

### B2 感知 bug（`peach_perception`）

| 文件 | 改动 |
|------|------|
| `target_reconstruction/frame_store.py` | `_push_frame_ring` 纳入 `_state_lock`（读者 `list(...)` 快照迭代与无锁写并发可抛 RuntimeError） |
| `target_reconstruction_node.py` | `_refined`/`_bag_model` 成对写入（`_merge_fused_bag_model` 不再直写缓存；keep_last_good 两缓存都不动；`_reset_products`/finalize 失败路径同步清 `_bag_model`）；`_on_target_observations` 几何缓存段与 `_on_initial_pose` 轴 hint 加 frame_id 门（≠base 系不混入漂移/串扰门） |
| `target_reconstruction/cloud_builder.py` | `apply_target_mask` 掩膜路径补 `depth<65535` 饱和剔除（对齐 io.md「有效深度」口径与无掩膜路径）；`build_cloud_base` 返回值增补 masked_depth（`_accept_frame` 复用，删二次全图掩膜） |
| `scene_perception/scene_perception_node.py` | `_bbox_at_edge` 跳过 ambiguous_* 每帧新键（无界增长、无人回读） |

### B3 技能 bug（`peach_manipulation`）

| 文件 | 改动 |
|------|------|
| `src/grasp_task.cpp` | `previewFullContact` 在 SKIP（已对轴、无接近段）时不做接近护栏——原 `skip_tail=2` 对 `size==2` 剥不掉，回退门对「插入再原路撤出」恒拒，FULL 套入规划自 09-03 加护栏后必败（未上真机故未暴露） |
| `safety_gate.hpp/.cpp` | 自适应新鲜度上限改 `std::atomic<double>`（订阅回调写/worker 读，原平凡 double 并发是 UB） |
| `src/manipulation_skills_node.cpp` | `on_configure` 依赖链复核移入 try（`get_params` 异常走 FAILURE 回滚而非逸出生命周期回调） |

### B4 死代码

感知：`FrameCollector.check_view`（连同 CollectorConfig 三个死字段）、`SpatialEmaMatcher.match`（类保留，参数属性仍用）、`refine_geometry`（含 refine.py shim 导出）、`backproject_depth`（含 integrate.py shim 导出）、`build_refined_marker`、skip 码 `motion_jump`、`_pipeline_for` 未用 bbox 参。技能：`kStageNames`（与 stageForState 平行的零引用声明）、stages.cpp 两个未用 include。executor 见 B1。

### B5 去重

- `common/`：`unit_vector/angle_between_deg/axis_radial_distance` 三份逐字复制收敛到 `fitting.py` 单源（depth_geometry/tf_utils re-export）。
- `select.py`：reach_queries 与 next_target 的资格过滤提公共生成器 `_eligible_locked_items`。
- 技能：接触入口三元组提 `contactEntryGeometry`（preview 与 VerifyPregrasp 修正两处；FinalizeAndValidate 因 TF 检查刻意前置保留原结构）；接近日志块提 `logApproachSplit`。
- 技能 B6 顺带：`ApproachSplit` 增 `current_tip` 快照，接近分档内 TF 三查收敛为一查。

### B6 性能（同值复用/去重算，零行为差）

见上：接近分档 TF 一查、`_accept_frame` 掩膜单算、executor 账单单写。

### B7 注释·文档·lint 门

- `peach_manipulation/CMakeLists.txt` 整测项跳过 cpplint（testing.md 口径：版权块与 Google include 序与本项目约定冲突；风格门=uncrustify，静态分析=cppcheck）；`peach_executor/setup.cfg` 补 `[flake8]` exclude（GPL 生成模块，与感知包同口径）；`peach_interfaces` 不再安装清单脚本（install 副本路径推导不成立，ctest 用源码路径）。
- IDL 注释：`CanonicalEvent.msg` 事件码全集补齐（含审计/过滤码）；`HarvestState.msg` `blockers`/`navigation_enabled` 标注预留恒值。
- 过期注释/文档修正：感知 11 个拆分模块 docstring 按实际职责重写；重建 `_process_rgbd`「只缓存最新一帧」→ 5 帧环；`cycle_context.reference_anchor` 重算描述；`motion.hpp` 默认值来源说明；`ScanBudget.poll` 不可达分支标注；`_pipeline_for`/refine_geometry 陈旧引用。
- 活文档同步：io.md（审计事件 reason、recorder 终局/开合语义）；testing.md（cpplint 口径、summary 时机）；architecture.md（缺口表 +5 条、决策 0017）。

### 验证

每批 `colcon build` + `colcon test` 过门；收官全仓 build 18 包通过，`peach_*` 四包 + serial_imu 全绿（manipulation 70 测试 0 失败，含 uncrustify）。剩余红项均非本轮引入：驱动四包（aubo_dashboard/bringup/hardware/moveit_config）的 copyright/cpplint 版权提示为只读红线包既有噪音；`src/graspnet_ros2`、`src/ivg_utils` 为会话期间出现的未跟踪厂商包，测试步失败，不在本仓管理范围。

mock 冒烟（`hardware_mode:=mock camera_enabled:=false`，`QT_QPA_PLATFORM=offscreen`）：五节点 `active [3]`、`managed_nodes_activated=true`、`/api/state` 与 `/api/trajectory` 均 200、`HarvestState` `batch_state=0 / execution_enabled=false / recovery_required=false`（默认档全关）、四个采摘动作齐、无 `NavigateToWorksite`、关停无残留。冒烟另抓到两处 B4 清理引发的构造期断裂（`CollectorConfig` 死字段 kwargs、诊断字典幽灵属性）与三处死参数的 yaml 声明，均已随批修复——lint/纯核测不 import 节点模块，构造签名断裂只有冒烟能拦。本机起栈须依次欠铺 `ros2_ws`（moveit_configs_utils）与 `ws_moveit`（MTC 动态库），并把 venv site-packages 追加进 PYTHONPATH（open3d 等）；testing.md §1 复现命令已同步。

---

## C9 感知包文件精简（2026-09-08，范围仅 peach_perception）

目标口径：可读性优先、注释完整、代码简洁、精简文件、逻辑零变化、保留既有缝位（`pipeline.*_impl` / `refitter.*_impl` 不动，换实现仍改源码或映射，不新增缝）。旁路抓取包（graspnet_ros2 等，并行会话工作区）与 manipulation（0017 轮已深度处理）不在本轮。

### 删除：7 个纯转发 shim（净 −608 行，44 文件改动）

| 删除文件 | 原 re-export 来源 | 消费者改写 |
|----------|------------------|-----------|
| `common/geometry.py` | fitting / depth_geometry / tf_utils | 15 文件导入直指真实模块（名字住在哪一目了然，不再有第二套命名空间） |
| `common/runtime.py` | clock / bounded_worker / harvest_data / ema | 6 文件 |
| `scene_perception/identity.py` | harvest_plan / target_registry / anchor_memory | scene_node 1 文件 |
| `target_reconstruction/capture.py` | captured_frame / skip_codes / capture_gate / bind_holdoff / timing / frame_collector / mask_gate / auto_controller | frame_store、recon node |
| `target_reconstruction/integrate.py` | tsdf_volume / cloud_builder / icp_refiner / icp_target_cache / overlap / view_coverage | 5 文件 |
| `target_reconstruction/refine.py` | candidate_contract / geometry_refiner | 3 文件（markers 的 `.refine` 相对导入 → `.geometry_refiner`） |
| `target_reconstruction/publish.py` | publish_throttle / publishers / session_io / status_messages | recon node |

`scene_perception/visualization.py` 回归真实绘图模块：删 6 个纯转发再导出与未用的 `_px`，`__all__` 收敛为 `_draw_debug` / `_to_markers`；scene_node 直接从 cloud_utils / conversions 取转换器。

理由：AGENTS「拆文件留 shim」是拆分期的兼容规则；本仓内 shim 已无仓外消费者（offline 脚本已归档），纯转发层迫使读者多开一个文件才知道名字定义在哪，正是「杜绝纯转发壳文件」要除的形态。导入改写用显式映射脚本完成（33 处 + 2 处手工），全部通过包内 flake8 的 import-order 校验（改写后修了 20 处 I100/I101 排序）。

### 评估后否决（记录防重提）

- valid-depth 掩膜逐目标重复计算：无 profile 证据表明是热点（2.5 FPS、ROI 级计算），且改动面在算法管线签名链——按「性能优化必须有证据」不动。
- `InferenceEngine` 组合壳：非纯转发（组合 YOLO/SAM/候选估计 + 计时埋点 + SAM 异常兜底），删除会把编排逻辑散进节点——保留。
- 微碎片（captured_frame/skip_codes/timing 等 <70 行文件）回并：拆分是已收敛决策（Phase D），职责各自成立，回并即「把收敛结构再拆一遍」的反向重复。

### 验证

`colcon build` + 包测试 5/0 全绿（flake8 含 import-order、pep257）；清单脚本 33 active + 4 reserved 通过；mock 冒烟五节点 Active、API 200、默认档全关——真实 import 链全量走通，证明 shim 删除无断裂。文档同步：architecture 文件树节/「从哪读源码」表 5 处/缝位表 REFITTERS 行、io.md 2 处路径引用已更新。

---

## Remaining（明确不做 / 记下缺口）

- C++ `stages.cpp` 未拆（接触安全面）。
- 两个感知节点壳与 `executor_node.py` 仍大（计划允许）。
- lifecycle 无 bond（architecture 已记缺口；现场未出现「静默死掉而名单仍 Active」）。
- 驱动/厂商 `package.xml` 漂移默认不动。
- `_collect_bag_views` landmarks 重复写入未改。
- 未提交的现场 `web/` 与若干运行 yaml 未覆盖。

---

## 旁路视觉抓取移植（2026-09-08）

从 `~/aubo_boot/aubo_ros2_ws` 迁入 `ivg_interfaces`、`ivg_utils`、`visual_pose_estimation`（仅 Python 包）、`graspnet_ros2`。采摘四包 / 驱动栈未接线。GraspNet 按 anygrasp_with_ros 的「工作区过滤 + get_grasp + ROS 分层」重写，后端为本地 GraspNet 权重 + 纯 torch 算子（明确不用 AnyGrasp SDK）。活文档口径：architecture §3 旁路节、io.md §8、testing.md 旁路 colcon/venv 命令。

## 旁路适配本区（2026-09-08）

裁掉旧仓机械臂/IO/软触发 IDL 与 `ivg_utils`；估姿数学收回包内。软触发对齐 Percipio `/camera/soft_trigger`，`T_B_C` 走本区 TF。GraspNet 检测 launch 不再起相机或手眼节点，点云兜底 frame 为 `camera_depth_optical_frame`。估姿 Web 运动类 HTTP 返回 501。旁路由四包改为三包。

## C8 代码量削减批（2026-09-08，可读性优先，量只是副产品）

判据：只删「第二套命名空间 / 死兼容壳」这类误导读者的表面；同构数据拷贝、防御分支一律不动（评估后回退了 stages.cpp bbox 七字段拷贝的泛型 lambda 去重——原文 14 行显式赋值自明可 grep，省 6 行换间接层不划算）。

| 文件 | 改动 | 动机 |
|------|------|------|
| `peach_perception/common/__init__.py` | 19 项聚合 re-export → 仅包 docstring | 全仓零显式导入者（子模块均从真实来源显式导入）；包初始化不再连带加载 scipy，且不再有与子模块平行的第二套导出面需要同步 |
| `peach_perception/common/runtime.py` | 删 `default_harvest_root` 再导出 | 该名仅在 harvest_data.py 内部使用；shim 面收敛为实际经它导入的名 |
| `peach_perception/scene_perception/identity.py` | 删 `MemoryGrasp` / `_finite_or` / `_selectable` 再导出 | 全仓无经 identity 的消费者；`_` 前缀私有名出现在公开 `__all__` 是误导 |
| `peach_executor/select.py` + `batch.py` | 删 `next_target_id` 兼容壳及其再导出 | 「不带窗过滤的原语义」在产品路径零使用（executor 直用 `next_target`，replay_metrics.py 不 import 包内模块）；留着会让读者误以为存在第二种选果语义。batch.py 文件级 shim 保留 |
| `peach_manipulation/src/stages.cpp` | 无改动（评估后回退） | 见判据 |

净变化约 −75 行，零行为差。评估否决项（记录防重提）：executor `_ack_recovery` 并入 `_call_service`（服务缺席路径从立即拒变为阻塞等超时，响应时序可观察）；`_apply` 的 operation_mode 防御分支删除（删后未来 FSM 表新增非 AUTO 反应会被静默吞掉）；target_cache 锚点采纳两段合并（update 类型不同、注释语义各自成立，合并要八参辅助函数）。

验证：三包 build+test 全绿（perception 5 / executor 8 / manipulation 70，0 失败）；mock 冒烟五节点 Active、`managed_nodes_activated=true`、API 200、默认档全关、关停无残留（log 尾 KeyboardInterrupt/-2 为 Ctrl+C 正常关停信号）。本轮一次 cwd 污染把 colcon 产物建进了 `src/peach_perception/{build,install,log}`，已删（gitignore 只盖根级，源码树内不设防，跑 colcon 前留意 cwd）。
