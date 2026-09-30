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

mock 冒烟（`hardware_mode:=mock camera_enabled:=false`，`QT_QPA_PLATFORM=offscreen`）：五节点 `active [3]`、`managed_nodes_activated=true`、`/api/state` 与 `/api/trajectory` 均 200、`HarvestState` `batch_state=0 / execution_enabled=false / recovery_required=false`（默认档全关）、四个采摘动作齐、无 `NavigateToWorksite`、关停无残留。冒烟另抓到两处 B4 清理引发的构造期断裂（`CollectorConfig` 死字段 kwargs、诊断字典幽灵属性）与三处死参数的 yaml 声明，均已随批修复——lint/纯核测不 import 节点模块，构造签名断裂只有冒烟能拦。本机起栈须依次欠铺 `ros2_ws`（moveit_configs_utils）与 `ws_moveit`（MTC 动态库），并把 venv site-packages 追加进 PYTHONPATH（open3d 等）；testing.md §1 复现命令已同步。（该铺层要求已于 2026-09-15 退役：moveit 全家迁 Jazzy apt、ws_moveit 删除，现行命令见 testing.md §4；此段保留作当轮过程记录。）

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

## 2026-09-08 旁路视觉抓取四包定名（ivg_* 家族）

四包统一 `ivg_*` 前缀（与包内 "IVG2.0" 品牌一致，区别于 `peach_*` / `aubo_*`）：`ivg_interfaces`、`ivg_utils` 沿用；`visual_pose_estimation(_python)` → **`ivg_pose_estimation`**（拍平双层目录，Python 模块与包同名，templates/models 收进包根，去掉冗余 `_python` 后缀）；`graspnet_ros2` → **`ivg_graspnet`**。估姿节点名同步为 `ivg_pose_estimation`（entry point `ivg_pose_estimation_node`/`ivg_pose_estimation_web`）；graspnet 节点名 `graspnet_demo_points_node` / `publish_grasps_client` 与 launch 名保持契约。launch 改名：`ivg_pose_estimation.launch.py` / `ivg_pose_estimation_web.launch.py`。AGENTS/architecture/io/testing 同步。

教训复录：在包目录内跑 `colcon test` 会因找不到包内 install 空间报 "Failed to find package.sh / Check that the following packages have been built"，审查曾据此误判 graspnet_ros2/ivg_utils 测试失败——colcon 一律在工作区根执行；包内的 `debug/session_debug/features.csv` 为 vpe Web 测试残留（save_debug_features 旧 cwd 写路径），已删，现输出改落包内 `debug_sessions/`。

---

## 2026-09-14 反向 Phase D（过拆回并，三能力包）

C9 删 shim 后碎文件即「真实模块」，单职责文件过小、跳转过多。本轮把同职责碎文件并回聚合模块，**不留旧路径 shim**。图名 / 话题 / 动作 / yaml / 算法常数不动。已很大的壳（`*_node.py`、`pose_pipelines.py`、`inference.py`、`harvest_fsm.py`、`params.py`/`params.hpp`、`stages.cpp`、`grasp_task.cpp`、`motion.cpp`）不往里塞。`common/__init__.py` 仍不聚合 re-export。`math_utils.hpp` 与 `eigen_conversions.hpp` 保持分开（后者 tf2，会污染 `_core` 零 ROS）。

### peach_perception

| 现行模块 | 并入后删除 |
|----------|------------|
| `common/runtime.py` | `clock.py`、`bounded_worker.py`、`harvest_data.py`、`ema.py` |
| `common/geometry.py` | `fitting.py`、`depth_geometry.py`、`tf_utils.py` |
| `scene_perception/identity.py` | `assignment.py`、`harvest_plan.py`、`target_registry.py`、`anchor_memory.py` |
| `scene_perception/visualization.py` | `conversions.py`、`cloud_utils.py` |
| `target_reconstruction/integrate.py` | `tsdf_volume.py`、`overlap.py`、`view_coverage.py`、`cloud_builder.py`、`icp_refiner.py`、`icp_target_cache.py` |
| `target_reconstruction/refine.py` | `candidate_contract.py`、`geometry_refiner.py`、`bag_model.py`、`pregrasp_verification.py` |
| `target_reconstruction/capture.py` | `captured_frame.py`、`skip_codes.py`、`capture_gate.py`、`bind_holdoff.py`、`timing.py`、`frame_collector.py`、`mask_gate.py`、`frame_store.py`、`auto_controller.py` |
| `target_reconstruction/publish.py` | `publish_throttle.py`、`status_messages.py`、`session_io.py`、`markers.py`、`publishers.py` |

未合：`common/bag_landmarks.py`、`common/tool_budget.py`、`common/ros/clock_adapter.py`。`identity` ↔ `pose_pipelines` 环只在 `memory_grasp` 内懒加载 `grasp_frame_from_axis`。`refine.py` 公开 `axis_angle_deg` 保持 refiner 语义（退化→`None`）；预抓取版改名 `_pregrasp_axis_angle_deg`。

### peach_executor

| 现行模块 | 并入后删除 |
|----------|------------|
| `batch.py` | `select.py`、`control.py`、`summary.py`、`ledger.py` |
| `observability/state.py` | `codec.py`、`metrics.py`（`job.py` 仍独立） |
| `observability/debug_actions.py` | `audit.py` |
| `observability/tcp_trajectory.py` | `ros_viz.py`（ROS `_color` 译名 `_color_msg`，避开 Marker 字典 `_color`） |

### peach_manipulation

| 现行头 | 并入后删除 |
|--------|------------|
| `cycle_context.hpp` | `cycle_state.hpp` |
| `cycle_support.hpp` | `execution_authority.hpp`（`MotionStage`） |
| （直接 include 节点头） | `cycle.hpp`（纯转发） |

`cycle.cpp` / `stages.cpp` 改为 include `manipulation_skills_node.hpp`（再确认纯核 `reconfirm_policy.hpp` 由 `stages.cpp` 直引）。CMake 源列表不变。

活文档：architecture 文件树 / 「从哪读源码」/ REFITTERS 映射；io.md `common.runtime` 与 `capture.py`。`check_interface_manifest.py` 的调度 consumer 路径只留 `batch.py`。

---

## 2026-09-16 R0 基线缺陷回归

规格：审查 F01–F19 的可复现子集。驱动只读；launch 仍不自动 RunHarvest。

| 现行 | 本轮 |
|------|------|
| `peach_perception/params.py` `_check` | `peach_perception/param_rules.py` `check`（executor 同名各一份） |
| `identity.py` 边提交边淘汰 | 整帧保护已分配 ID 再注册；歧义看已占用列 |
| `harvest_fsm.react` + `_wait_result` 写 `PAUSE_PENDING` | `apply_event` / `EventHold`；暂停只改 `operation_mode` |
| `stages.cpp` 残差失败抬 `LEVEL_PREGRASP_VERIFIED` | `pregrasp_level.hpp` + gtest |
| `tcp_trajectory.path_metrics` 绑 `geometry_msgs` | `observability/path_metrics.py` 零 ROS |
| manifest 11 个未登记字面量 | 命令服务与 job/metrics 进清单 |
| 无 CI | `.github/workflows/jazzy.yaml` + `scripts/r0_gate.sh` |

## 2026-09-16 R1–R10 框架渐进迁移

规格：审查路线 B。不改驱动；不自动 RunHarvest；不新增第四份活文档。

| 现行 | 本轮 |
|------|------|
| 节点与纯核混在 `scene_perception/` / executor 根 | `peach_*/domain/` 零 ROS；import guard |
| 手写 ParamListener 为唯一机制 | 键名冻结 + `contract.param.yaml` 同 schema；C++ `execution_contract_parameters` GPL；禁止第三套生成器 |
| TargetModel 无 run_id；心跳可当新鲜 | 身份元组 + `valid_until`；`replaceModelSnapshot`；诊断不续签 |
| 暂停 overlay `PAUSE_PENDING` | R0 EventHold + R3 `domain/reducer.py` + generation |
| lifecycle 无心跳 | `HeartbeatWatchdog`（bondpy 未装） |
| 工具链 × 整张 octomap 豁免 | `allowToolVersusWholeOctomap()==false` |
| SetIO 失败仍可能撤退 | UNKNOWN，不自动撤退/重发 |
| harvest_system 在 executor | `peach_bringup` 入口；executor 薄转发；`peach_observability` 独立 bag |
| 无 system_tests | `peach_system_tests` isolated launch_testing |
| pluginlib | R9：无第二实现、未迁 |

## 2026-09-16 审查修复

| 缺陷 | 修复 |
|------|------|
| 暂停后取消仍可能释放 EventHold 的 EXECUTE_FULL | `take_after_pause(cancel=True)` 丢弃暂存 |
| 生产 OBSERVE→FULL 不存 plan | OBSERVE_ONLY 存 plan_id+身份；观察动臂不冻关节 |
| `evaluate_capabilities.allowed` 与 `allowed_from_capabilities` 双路径 | 预算 allowed 只走后者 |

## 2026-09-16 端到端审查续

| 缺陷 | 修复 |
|------|------|
| 预检 cmdline 子串误伤编辑器路径 | argv basename；`harvest_system.launch` 仍跳过 |
| launch_testing 只起 keepalive | mock `harvest_system`：六关节名 + lifecycle Active，不发 RunHarvest；JSB name 字母序；move_group SIGINT 段错误不纳入 peach 退出门 |
| executor 仍 exec_depend 驱动/IMU | 删 `aubo_e5_bringup` / `serial_imu`（入口在 peach_bringup） |
| architecture `record_mcap` 残留 | 会话 bag 在 observability 节点；独立 record_bag 默认关 |
| 调度/观测/lifecycle yaml 未进冻结测试 | `test_frozen_keys.py` 对照三份 yaml |
| CI 仅 r0_gate | `industrial_ci` job（忽略 IVG/`imu_follow`；ICI apt scipy/pytest/yaml，不 Docker pip） |
| 8090 仍由 executor 可执行文件承载 | `peach_observability` 安装可执行文件并承载实现模块；调度包 `observability/` 仅 shim；yaml 仍在 executor |
| 预检把 colcon `--packages-select peach_executor` 当残留栈 | 只认 argv0 或 `.../lib/<pkg>/<node>` 包装路径 |
| 调度 `exec_depend peach_manipulation` | 删：跨包只走 IDL；技能由 `peach_bringup` Include |
| 活文档仍写 33 active / GPL `*_parameters.yaml` / 三节点同包 | 改为 44 active；运行参数指向现行 `config/<节点>.yaml`；8090 可执行文件在 `peach_observability` |
| 感知 `tool_profiles` 读 `aubo_description` 未声明依赖 | `peach_perception` `exec_depend aubo_description` |
| 调度包残留已迁走的 8090 `web/` | 删除 `peach_executor/web/`；静态页只在 `peach_observability/web/` |
| testing.md 仍写 bag_report 在 executor、系统测包缺口 | 路径改 `peach_observability/test`；`peach_system_tests` 已落地，Gazebo 仍缺口 |
| `evaluate_sleeve_cut` 仍自算 `allowed` | 几何核只返回 sleeve/cut；`allowed` 仅 `allowed_from_capabilities` |
| AGENTS SNAPSHOT 仍写调度承载 8090、四包 | 改为七包；8090 在 `peach_observability` |
| 观测 `setup.py` 安装不存在的 `config/` | 删除空 glob；yaml 仍在调度包 |
| PAUSE 后 RESUME/RESET 不重新武装 GetState 心跳 | `watchdog_armed_after`：成功 STARTUP/RESUME/RESET 才武装 |
| ICI 会编 Percipio 厂商 TYCam | `COLCON_IGNORE percipio_camera`（mock 测 `camera_enabled:=false`） |
| 纯核枚举与 IDL 无对账 | `test_idl_constants.py` 对照 HarvestState / ControlTask / ManageLifecycleNodes |

---

## 2026-09-16 peach 标准化重构设计（长期路线 R11+）

**定位：** 设计文档（路线规格，未实施）。R0–R10 已在途（上两节，工作区未提交）；本节是其后到「长期稳定框架」的剩余路线，按框架/算法/流程/性能四维综合分析后分期。实施仍按 AGENTS 教义逐轮做，每轮同轮改活文档；现行快照以 [architecture.md](architecture.md) 为准，本节不驱动现行设计。

**权威基线（本设计检索对照，均按第 3 章「文档+源码」口径回链）：** ROS 2 Jazzy Developer Guide（测试塔、包布局、防御式）；REP-2004/2005（QL 等级）；Nav2（lifecycle+bond、diagnostics、Collision Monitor「命令链最后一环」）；MoveIt 2（MTC、sensors_3d octomap、TEM）；ros2_control（mock_components、错误停控制器）；UR ROS2 Driver（mock/真机同管线、P-stop 禁 resume）；Autoware（interface manifest、fail-safe 命令门、参数指南）；TurtleBot 4 / Stretch（分包、runstop 双超时）；OSU apple-harvest（`enable_*` 门）；industrial_ci。本机已核实：`ros-jazzy-nav2-lifecycle-manager` 1.3.13 在装、`bond/bondcpp` 头在装（`bondpy` 未装——HeartbeatWatchdog 等价成立）。

### 一、现状综合分析

**已稳固（本设计不再动，防重复提案）：**

| 项 | 证据 |
|----|------|
| 七包拓扑 + 依赖单向（interfaces ← 能力包；跨包只走 IDL） | architecture §3；executor 已删 `aubo_e5_bringup` / `serial_imu` / `peach_manipulation` 依赖（R8/审查轮） |
| domain 纯核 + import guard | `peach_executor/domain/`（reducer/ledger/lifecycle/watchdog，零 ROS）；`test_import_guard.py` |
| 参数键名冻结 + 合同 schema | `contract.param.yaml` + `param_rules.py` + `test_frozen_keys.py`（契约面，不是第三/四套参数框架） |
| IDL 双向核对 + 常量对账 | manifest 44 active + 4 reserved；`test_idl_constants.py` |
| lifecycle 心跳 | `HeartbeatWatchdog`（GetState 心跳，STARTUP/RESUME/RESET 重新武装）——bond 的显式 watchdog 等价物（AGENTS 认可口径） |
| 接触 ACM 按目标×工具×阶段 | `acm_policy.hpp`：整图 octomap 豁免已撤（gtest 断言 false）；仅 Sleeve/Cut × 指定目标放行 |
| 过程记录 | 会话 bag（0019）+ `bag_report` 自动重算 + 预算回收 + retention 审计 |
| 测试塔下半 + CI | 纯核 pytest + gtest（pregrasp_level）+ `peach_system_tests` mock launch_testing；r0_gate + industrial_ci 双 job |
| 安全分层 | 使能三档默认关、`PREGRASP_ONLY` 默认、launch 不自动 RunHarvest、停轨=透传 abort + `RobotMoveStop`、故障后禁 resume 原轨迹 |

**四维缺口：**

| 维 | 现行 | 权威做法 | 缺口 |
|----|------|----------|------|
| 框架 | lifecycle 管理器手写（watchdog 等价 bond）；peach 主路径无 `/diagnostics`（`serial_imu` 已示范）；缝位=2 处 dict；算法/配置版本不入账 | Nav2 lifecycle_manager（bond，本机已装）；`diagnostic_updater` 周期诊断；MoveIt/Nav2 pluginlib；消息带版本便于归因 | 诊断主干缺；版本可追溯缺；pluginlib 无第二实现（R9 已缓，判例成立） |
| 算法 | 停走 2.5 FPS 感知（YOLO+SAM+χ²匈牙利+TSDF+Huber 融合+动态预算）；接近=staging PTP+轴向 LIN（同 seed 66/100、现场包络 39/41、09-11 typical 26/30、绕行比 ≤1.70）；四层接近护栏 | Open3D NBV 停准则（已对齐）；MTC/Pilz（已对齐）；接触确认 ACK≠切断（UR 口径，已对齐） | 观察效率（6 视 33.5 s、landmarks 逐 refit 重估、`max_views=24` 与现场 4–6 脱节）；octomap 非豁免后套入段与袋自身点云的关系未真机验证；刀 DI 未接线；ContactMonitor 未标定（默认关） |
| 流程 | 回放回归靠手工脚本（`scripts/sim_field_targets.py` / `analyze_approach_envelope.py`）；无物理仿真；真机 FULL 未验收 | 测试塔 unit→launch_test→system→field；replay 进 colcon；Gazebo Harmonic 配 Jazzy | 回放未入塔；仿真缺（条件项，非禁令）；FULL 验收待授权轮 |
| 性能 | `PublishThrottle` / `IcpTargetCache` / `BoundedWorker` / 帧环已有；空闲基线已测（recon ~21% CPU / 182 MB、scene ~638 MB） | 证据先行（C9 判例：无 profile 证据不动管线签名链） | 运行期每帧墙钟/队列丢弃未成持续指标；638 MB 模型常驻未评估 |

### 二、长期稳定的目标态（不变量）

不是新框架，是把已验证的约束固化成「改任何一块都不许破」的规则：

1. **契约面三重冻结。** 图名/QoS=manifest 双向核对；参数键名=`contract.param.yaml`+冻结测试；纯核常量=IDL 对账测试。新接口先过这三道才准合入。
2. **单一事实源清单。** 末端工具=`aubo_description/config/<profile>.yaml`；调度合同=`peach_executor/config/contract.param.yaml`；跨包契约=`interface_manifest.yaml`；部署值=`config/<节点>.yaml`。同量多处收敛到唯一源+注入/派生，禁止第二份手抄。
3. **缝位按需升级。** dict `*.impl` 只剩袋/果管线与柱/球 refitter 两处；升级 pluginlib 的唯一触发条件=出现要 A/B 的第二实现（R9 口径）。升级时先冻接口再迁注册，不先造框架。
4. **生命周期健康二选一。** watchdog（现行）或 bond（若迁 `nav2_lifecycle_manager`）；不允许「名单 Active 但进程已死」的无检测态；观测/旁路节点不进名单。
5. **诊断是护栏的眼睛。** 「出了再 ERROR」的健康信号逐步收进 `/diagnostics` 周期任务；session bag 保证事后复盘，诊断保证事中可见。
6. **证据先行的性能循环。** profile（diagnostics+bag metrics）→ 改 → 回放/冒烟对比 → 数字入 testing-log；无证据不动算法管线。
7. **测试塔单调。** unit（纯核/gtest）→ launch_testing（mock 图）→ replay（bag 回归）→ system（物理仿真，条件）→ field（命名轮次）。下层绿才上上层；field 永远是套袋方向最终权威。
8. **演化教义本身是流程。** MUST/DEFAULT/KEEP/UNWIND + 三活文档同轮 + 只追加记录已是机制；本设计只给它排接下来要做的事，不另立一套。

### 三、分期路线（每期一 R，独立可验收，可按现场优先级调序）

#### R11 诊断主干（框架，小～中）

- 四能力节点接 `diagnostic_updater`（apt，`serial_imu` 同款）：感知（TF staleness 率、检测/SAM 异常率、worker 队列丢弃）、重建（ICP/TF 拒帧率、积分帧率、融合失败率）、技能（规划失败与护栏拒发分类、`robot_status` 断流）、调度（FSM 停留时长、watchdog 状态）。观测节点把 `/diagnostics` 汇总进 session bag。
- 同期出评估结论（不一定迁）：`peach_lifecycle_manager` 换 `nav2_lifecycle_manager`（本机已装 1.3.13，自带 bond）。成本=`ManageLifecycleNodes` 图契约变更（manifest+io.md+客户端）+ `managed_nodes_activated` 旗标改由生命周期状态派生；收益=社区维护+bond。**触发条件：真机多日会话需要 bond 级死检**；不满足则 watchdog 维持现状。
- 验收门：mock 起栈后 `/diagnostics` 各节点 OK，拔相机/杀节点指标可变；现有 lint/纯核/launch_testing 全绿；manifest 若动则核对绿。

#### R12 回放回归入塔（流程，中）

- 把手工回放收进 `peach_system_tests` replay 档：`bag_reader`/纯核驱动、不起 DDS 图，输出成功率/绕行比/拒发分类与基线对比，覆盖三个既有语料（同 seed 100 随机位姿、现场真实包络、09-11 typical 30 例 + 09-14 护栏回归）。
- 基线数字表（66/100、39/41、26/30、绕行比 ≤1.70、姿态 ≤71° 等）入 testing.md 作回归容差。
- 规则：接近、融合、护栏任一改动必跑该门。
- 验收门：`colcon test --packages-select peach_system_tests` 在无图 mock 环境可跑；数字超容差即红。

#### R13 观察节拍（算法，中；证据门先行）

- 先测后改：用 R11 诊断分解单颗观察耗时（移动/等帧/积分/融合），确认 33.5 s 主耗项再动手。
- 候选改动（各有证据才动）：`_collect_bag_views` landmarks 按机位簇缓存（refit 不重估，architecture §8 已记缺口）；视图选择代价模型并入覆盖停准则；`max_views` 与覆盖门关系复核（现场 4–6 视是事实）。
- 验收门：R12 回放门数字不降；单颗观察时长改善或「不改」结论入 testing-log。

#### R14 接触验收支持（算法+流程，中～大；含真机授权轮）

- 刀具 DI 切断确认接 `/aubo_io_controller/io_states`（只读消费；未确认仍 `CUT_FEEDBACK_TIMEOUT`，ACK≠切断口径不变）。
- 按目标限界 octomap 豁免实验：套入段工具×**本目标包围盒内** octomap 体素放行（`acm_policy` 已撤的整图豁免不回来）；顺序=R12 回放包络仿真 → mock → 真机。
- `ContactMonitor` 标定流程成文（空载/接触电流特征采集、阈值入 yaml；标定前默认关保持）。
- 出口：`field_full_*` 命名轮次（书面授权；`PREGRASP_ONLY` 仍是默认门）。
- 验收门：FULL 干跑+真剪各一轮入 testing-log；R12 回放门不降。

#### R15 契约版本化与缝位预案（框架，小）

- 版本指纹入账：`HarvestSummary`/`CanonicalEvent`（或 ledger extra）记检测/分割权重版本、refitter 版本、工具档案版本、接近护栏参数指纹——事后归因不再靠回忆（§8「消息无 algo/config 版本」缺口闭环）。
- pluginlib 升级预案成文（接口冻结构+plugins.xml 骨架，触发条件见不变量 3），不实施。
- 验收门：bag_report 能打印版本行；冻结测试绿。

#### R16 物理仿真（条件项，大）

- 触发条件：需要回归接触动力学 / ContactMonitor / octomap 撞枝时才立项：Gazebo Harmonic + `gz_ros2_control`，同一 URDF 与控制器，独立测试包，全图 `use_sim_time`。不满足不立项（「无 Gazebo」是缺口不是禁令，但 YAGNI）。
- 验收门：仿真栈与 mock 同管线；系统测包独立不进运行 launch；真机仍是权威。

### 四、防走偏（本设计明确不做）

- 不为 pluginlib 而 pluginlib、不预迁 dict 缝（R9 判例）。
- 不把感知 Python 节点塞进 component container：rclcpp 的 composable+intra-process 零拷贝不覆盖 rclpy，Python 侧收益为零、改拓扑风险为真。
- 不新增第四份活文档；本节是过程记录里的路线规格，实施轮才动活文档。
- 不把 `nav2_lifecycle_manager` 迁移当必做（触发条件见 R11）。
- 不做无 profile 证据的性能微优化；不动相机 2.5 FPS（驱动只读）。
- 不把 8090 / ROS 任何软件通道当 e-stop；FULL 真机轮永远要书面授权 + 示教器急停可达。

### 五、优先级建议

默认顺序 **R11→R12**（诊断与回归塔是其余各期的量尺，先立尺再动刀）→ **R14**（接触验收是最接近产品价值的缺口）→ R13/R15 穿插 → R16 条件触发。现场若要先做 FULL 验收，R14 可提前，但 R12 回放门必须先立——否则接近/护栏改动没有回归证据。

---

## 2026-09-16 清洁重写轮（定稿方案，取代上文 R11+ 路线）

**背景：** 用户核定后授权全面重写——范围 peach 七包 + `serial_imu` + `imu_follow`（IVG/驱动不动）；图名/IDL/参数键全破；包边界自由重切；参数全迁 GPL；验收门=回放塔+mock 冒烟+launch_testing；本轮不碰真机。上文 R11+ 各期被本节吸收或取代。

**红线核定（逐条，2026-09-16）：**
- 维持：驱动栈只读；示教器上电；未授权不动臂/SetIO（**real 上操作员发起 launch/指令=授权**）；硬件急停不经 ROS；保护停止后不 resume；Jazzy/numpy 1.26.4；六关节序；相机驱动只读。
- 删除：launch 自动开批禁令→`autostart` 参数；三档默认关→操作台运行时开关；8090 非控制面→**操作台**；工具帧名冻结→设计自由（沿用四帧名）。
- 算法核+标定常数**原值移植**（staging 接近/融合预算/护栏/停走门/IK 采样集）；系统语义全部可重设计。

**参照系（完整采收机器人调研，2026-09-16）：** Tevel/FFRobotics/Harvest CROO/Agrobot/Panasonic/Advanced Farm/Ripe Robotics（工业）；OSU apple-harvest（唯一完整 ROS 2 学术开源栈）、SWEEPER/CROPS、猕猴桃 Williams 2019/2020、Fu 2024 猕猴桃成簇剪切（**AUBO E5 同臂先例 88%**）、荔枝 Fcaf3d/AHPPEBot、Bac 2014/Tang 2020/Huang 2025 综述。可迁移要点已入定稿方案（节拍预算工程/视点两档化/跳过调度参数/补采清单/RETAINED 承接检查点/误差-容差链/操作台四栏/每果档案/KPI 换算链）。

**目标架构（6 peach + 2 IMU）：**

| 包 | 职责 |
|----|------|
| `peach_interfaces` | 新契约（RunHarvest 带批次策略 / 接触 ExecuteTarget 带检查点 AT_STAGING→…→CUT_CONFIRMED→RETAINED→RETREATED→STOWED / MoveTo / CheckReachability / Console 服务组 / Clearance 令牌） |
| `peach_harvester`（Python，一进程两节点） | `peach_vision`（粗扫+细看，两级视点）+ `peach_supervisor`（周期状态机/选果/视点规划/批次排程/操作台后端/账本+单果档案） |
| `peach_arm`（C++，manipulation 演进） | MoveTo / 接触 ExecuteTarget / CheckReachability；**命令门**=enables×clearance×robotReady×¬cancel；接触核原值移植 |
| `peach_bringup` | 唯一组合点：autostart/预检/工具档案+standoff 注入 |
| `peach_recorder` | 只读会话 bag+报告+回收+诊断归档，零控制面 |
| `peach_system_tests` | launch_testing + 回放塔 |
| `serial_imu`/`imu_follow` | 随图名微调 / GPL 化 |

生命周期：nav2_lifecycle_manager（bond_timeout 0）+ HeartbeatWatchdog。**工艺推导：** 周期状态机归一进程（消灭 BeginScene/SurveyScene/BuildTargetModel 编舞，观察循环内化）；进程边界=语言边界（Python 脑 / C++ 臂）；意图源大脑、强制点臂命令门。

**阶段（每阶段末可编可测、独立提交）：**
0 基线封存（提交 R0–R10、回放塔、F1–F13+KPI 入 testing.md、快照）→ 1 契约设计 → 2 臂服务器 → 3 大脑成型（3a 并包→3b 进程合并→3c 观察内化+视点两档→3d 纯核收口）→ 4 操作台+记录器 → 5 IMU+组合 → 6 整删+AGENTS 红线改写+活文档终稿+总验收。

**视点两档：** fast（新默认，单视决策+低置信补视封顶 3 固定视）/ conservative（现行多视原值）；融合/预算/门限常数两档共用原值。

**失败分类学：** 定位不准/遮挡/不可达/损伤/脱离失败/落果未承接（新）；跳过自动入 `rework_list.json` 补采清单；批次策略参数 `target_harvest_ratio`/`per_target_timeout_s`/`sector_timeout_s`/`view_policy`。

**明确不做：** 不改驱动；不动算法常数；不动 IVG；不做 Gazebo；不迁 Python 进 composition；学习型剪切点回归与主动照明只留缝；真机另授权轮。

---

## 2026-09-16 清洁重写轮执行记录（阶段 0–6-1，21 提交）

| 阶段 | 提交 | 内容 |
|------|------|------|
| 0 | a955cea | 基线封存：R0–R10 在途 106 文件验证后落库 |
| 0 | 36d5c90 | 回放塔（replay_oracle 移植+冻结基线+bag legacy 别名+KPI 链入 testing.md） |
| 1 | 15d2091 | 契约层：MoveTo/Clearance/Enables/SetEnables/SetBatchPolicy/FireStep 新 IDL+manifest 修真 |
| 2a | 3f29d30 | peach_manipulation→peach_arm 机械重命名（88 文件） |
| 2b | 9c63eaf | 功能手术：MoveTo 服务器+令牌双路+检查点+使能订阅（MoveTo goal 实发走通） |
| 2c | 8ff1d33 | 参数全迁 GPL：删手写 params.hpp，arm_parameters.yaml 单源 |
| 3a | 57d5a00 | 并包：perception+executor→peach_harvester（126 文件 git mv，图名零变化） |
| 3b | 887b0de | 进程合并：brain.py 一进程三节点（MultiThreadedExecutor） |
| 3c-1 | 896691a | 纯核件：view_policy 两档+batch_policy/rework_list（13 测试） |
| 3c-2a | 466ff5c | 批次策略消费+令牌装配+补采接线（executor_node） |
| 3c-2b | 6cfdde7 | view_planner C++→Python 全量移植（5 测试） |
| 3c-2c | 608c3f7 | fast 档观察内化：supervisor 直驱补视+MoveTo camera_frame |
| 4-1 | fbcf655 | 操作台服务面：SetEnables/SetBatchPolicy/FireStep |
| 5 | (本条) | nav2_lm+autostart+lifecycle_flag_bridge+package.xml/manifest |
| 6-1 | (本条) | 活文档红线与包名同步（AGENTS MUST 区改写+四活文档 62 处） |

**功能清单核销状态（vs F1–F13）：**
F1✅ F2✅ F3✅ F4✅（两档并存）F5✅ F6✅ F7✅ F8⚠️（服务面✅，Web 四栏未升级）F9✅ F10✅ F11✅ F12⚠️（observe details 带耗时，全段计量未收口）F13⚠️（ledger 扩展+yield 视图未落）

**验证欠账（用户口径：单独测试轮）：** colcon test 全量、brain 入口整栈 mock 冒烟（3b 后未起栈）、fast/conservative 观察对照、回放塔重跑、Mimosa 完整审计（scanner_enobufs 持续）。

**剩余队列（阶段 6 余项）：** 节点改名 peach_supervisor/peach_vision+图名波+deprecated IDL 删除+Survey/Begin/Build/OBSERVE 旧模式删除+感知/调度两份手写 params 迁 GPL+imu_follow GPL+Web 操作台四栏升级+recorder 剥 Web+数据集导出脚本（Mimosa 路径穿越拦截排队）+旧件整删+总验收。

---

## 2026-09-16 清洁重写轮收官（25 提交，全部编译/导入/清单门过绿）

### 提交全表

| # | 阶段 | 提交 | 内容 |
|---|------|------|------|
| 1 | 0 | a955cea | 基线封存：R0–R10 在途 106 文件验证后落库 |
| 2 | 0 | 36d5c90 | 回放塔（replay_oracle 移植+冻结基线+bag legacy+KPI） |
| 3 | 1 | 15d2091 | 契约层：MoveTo/Clearance/Enables/SetEnables/SetBatchPolicy/FireStep |
| 4 | 2a | 3f29d30 | peach_manipulation→peach_arm（88 文件） |
| 5 | 2b | 9c63eaf | MoveTo 服务器+令牌双路+检查点+使能订阅（goal 实发走通） |
| 6 | 2c | 8ff1d33 | arm 参数全迁 GPL |
| 7 | 3a | 57d5a00 | 并包 harvester（126 文件 git mv） |
| 8 | 3b | 887b0de | 进程合并 brain.py |
| 9 | 3c-1 | 896691a | 纯核件 view_policy/batch_policy（13 测试） |
| 10 | 3c-2a | 466ff5c | 批次策略+令牌+补采（executor_node） |
| 11 | 3c-2b | 6cfdde7 | view_planner 移植（5 测试） |
| 12 | 3c-2c | 608c3f7 | fast 观察内化+MoveTo camera_frame |
| 13 | 4-1 | fbcf655 | 操作台服务面 |
| 14 | 5 | 91fe5db | nav2_lm+autostart+bridge |
| 15 | 6-1 | 209ebe4 | 活文档包名同步 |
| 16 | 6-1 | 0242421 | 文档红线+nav2_lm/autostart 收官+过程记录 |
| 17 | 6-2 | e2f37ff | 死件 IDL 删（HarvestEvent/MatchStatus/JobIntent） |
| 18 | 6-3 | 1b9c489 | 节点改名 peach_executor→peach_supervisor（37 文件） |

### 系统最终架构

```
peach_interfaces       47 active + 4 reserved；消费者校验器
peach_harvester        大脑（一进程三节点）：
  vision/              感知全链
  supervisor/          批次 FSM/选果/视点两档/策略/操作台/账本/补采
  cycle_core/          零 ROS 纯核
peach_arm              臂服务器（C++）：MoveTo/ExecuteTarget/CheckReachability；
                       命令门=enables×clearance×robotReady×¬cancel；GPL 单源
peach_bringup          nav2_lm + autostart + 桥 + 预检 + 档案注入
peach_observability    只读记录器：8090/bag/report/回收
peach_system_tests     launch_testing + 回放塔
serial_imu / imu_follow  可选包
```

### 功能清单核销（vs 定稿 F1–F13）

F1✅ F2✅ F3✅ F4✅ F5✅ F6✅ F7✅ F8⚠️ F9✅ F10✅ F11✅ F12⚠️ F13⚠️

### 明天的测试优先序

1. **colcon test 全量**（7 包）
2. **brain 入口 mock 整栈冒烟**（3b+nav2_lm+改名后从未起栈——最重要）
3. **fast/conservative 观察两档对照**
4. **回放塔重跑**
5. **Mimosa 完整审计**（scanner_enobufs 持续 25 次）

### 非阻塞后续

Web 四栏升级、recorder 剥 Web、imu_follow/感知/调度 GPL、数据集导出（Mimosa 拦截）、Survey/Begin/Build/OBSERVE 删除（conservative 依赖）。



---

## 2026-09-17 IVG 旁路包精简轮（四包→三包，ivg_utils 删除）

检索依据（AGENTS 第 3 章）：scipy 官方 `Rotation` 文档 + ros2_control/UR 无关本域；`message_filters` 因触发-等待节拍是产品 KEEP 不替换。前置等价性验证：6.1 万随机旋转对比手写公式 vs `scipy.spatial.transform.Rotation`（quat→矩阵逐元素一致；矩阵→quat 同旋转、6.5% 情形返回等价反号四元数且两处调用方均为旋转级消费；RPY 含万向节锁一致）。

### 删除

- `ivg_utils` 整包（伪共享：全仓唯一消费者 ivg_pose_estimation；`constants.py` 7 常量零引用，原 Worker 类已不存在；手写四元数/旋转公式换 scipy，`filter_components_by_params` 迁入 `feature_extractor.py` 私有函数；`normalize_angle_to_180/pi` 死代码删，pose_estimator 自有副本与 3 处 while 归一化统一 `np.mod`）
- `ivg_pose_estimation/config_reader.py`（死兼容 shim，全仓零模块 import）；4 处「委托 ivg_utils」包装方法与 `__init__` 转出口；2 处内联四元数公式
- 模板垃圾：GBK「副本」json、rembg 残留 nobg.png、零引用 hand_eye_calibration.xml、测试工件 `55555555555555/`（912K 无位姿）、3211242785 半成品 pose_5/6
- 死配置键链：`enable_zero_interp` / `enable_smooth_edges` / `smooth_edges_blur_sigma` / `max_threads`（config/ros2_communication/native_api 三处映射）
- Web：前端零调用的 `/api/save_debug_features` 端点（连同测试断言）；`/api/get_template_image` 初判「前端零调用」**误判**——app.js `displayTemplateImage`（模板列表动态按钮调用）依赖它，已恢复并加固（文件名拒路径分隔符 + `_safe_template_dir` 越界校验，测试补穿越用例）；`WebPaths` 4 个零消费者成员（docs_dir/legacy_scripts_dir/debug_thresholds_file/pose_list_dir）及 manager→runtime_support→native_api 末端链；`bridge_module` property、`_load_bridge_module` importlib 间接层、`_app_config_cached`
- pose_estimator `template_results` 只写不读结构；`save_metadata` 死参；setup.py 死 glob（web_ui/*.txt|*.sh、scripts/*.py）；app.js 硬编码 `/home/nvidia/RVG_ws`、空 for、假 deleteTemplate

### 修正（顺带，最小 diff）

- `EstimatePose.srv` 补 `string message`（服务端 4 处赋值原先抛 AttributeError 被兜底吞掉、失败文案丢失——存量 bug）；5 个 srv 注释去「喵~」噪声、服务方旧名 `visual_pose_estimation` 改 `ivg_pose_estimation` 节点
- **存量 bug：Web 桥从未启动过**——`RosBridgeManager.start()` 访问 `node_runtime.rclpy`，但该模块只有 `from rclpy.node import Node` 不绑定 `rclpy` 名字 → AttributeError → `startup_error`，所有需 node 的端点恒 500（测试全用 dummy node 掩盖）。修复：node_runtime 顶部显式 `import rclpy`（noqa 注明供 manager 模块属性访问）。Web 冒烟复验 `ros_bridge_ready:true`
- Web 模板端点（get_template_image/read/save/capture_template_image）加 `resolve()`+`is_relative_to` 越界校验（闭路径穿越读写）；CORS `allow_credentials=True`+通配源（规范禁止的组合）改 False
- launch 默认 calib_file 从不存在的 `web_ui/configs/hand_eye_calibration.yaml` 改空串走标准候选链
- testing.md 旁路 pytest 路径笔误（test_web_app 双层路径）与 graspnet 须在包目录下执行的说明

### 教训

- `graspnet_lib/AMENT_IGNORE` 是**有效**机制（ament lint 尊重子目录标记）：删除后 colcon test 的 flake8 用例立刻对 vendored 代码报一串风格违规（该包唯一测试失败），已恢复——审计结论「对 ament_python 无功能」被实测推翻，精简前先跑测试再删标记
- `manager.py` 的 importlib 间接层看似冗余实为懒加载（保 web 层无 rclpy 可导入）；简化时保留懒加载语义（函数内 `from . import node_runtime`）

### 验收

colcon build/test 三包全绿（222 tests 0 failures）；venv 回归 test_web_app 12 passed、test_grasp_core 8 passed；Web 冒烟（:18088）health/templates/redirect 通、桥 ready；ROS 节点冒烟启动成功（scipy 链）。活文档同步：AGENTS/architecture/io/testing 四包→三包口径；CI workflow touch 行删 ivg_utils。遗留（记入本表不实施）：templates 未装进 share 依赖源码树解析；连通域筛选两套、内参解析两份、阈值映射三份、estimate_pose/2d 管线五处中风险合并留后续轮。

### 同日复核轮（逻辑/流程/数学审查）

数值实测：np.mod 两种归一化 40 万样本扫描与旧 while 循环等价（仅 ±180 边界互换=同角度、1e-18 级浮点噪声）；下游角度消费经 `arctan2(sin,cos)` 归一化对边界互换天然免疫；模板库 70 个四元数全部单位（scipy 非单位归一化差异不触达）；cv2 0°/360° 矩阵一致。复核发现并修复：守卫初版漏 `..`（pathlib 视其为普通组件，`Path('..').name=='..'`，只查 name 拦不住）且 `pose_id` 含 `/` 可沙箱内重定向——改单段组件校验（非空/单组件/不含 `..`）+ 越界双重校验，对抗 12 用例与真实 HTTP 层（三类越界全 400）复验；3D handler 兜底 except 补 `response.message`（加字段正是为此路径）。另发现存量无害项：StandardizeTemplate 处理器成功摘要写入不存在的 message 字段被 hasattr 静默跳过（错误信息走 error_message 不丢）。

---

## 2026-09-18 peach_vegetation GPU 枝/叶分割包

新建 `src/peach_vegetation`（ament_python）：零 ROS 核 `frangi.py` / `split.py`（torch Hessian Frangi + Excess Green/HSV），Lifecycle 节点发 `/peach/vegetation/{leaf_mask,branch_mask,overlay,status}`，`diagnostic_updater` → `/diagnostics`。独立 launch，不进 `harvest_system` / lifecycle，不写 PlanningScene。参数 0017 同款 ParamListener。清单 +4 active。活文档 architecture / io / testing 同轮。

---

## 2026-09-18 scene_perception 参数回迁 GPL（Python 端）+ 动态改参

`peach_scene_perception_node` 参数声明/兜底默认/校验从手写 `vision/params.py`（决策 0017）迁到 generate_parameter_library 0.7.6 Python 生成：声明单源 `config/scene_perception.params.yaml`（59 参，键名/校验逐条转写冻结，默认值对齐部署清单）→ 生成物 `vision/scene_perception/params_gen.py` 随库提交（ament_python 无 cmake 钩），`scripts/gen_scene_perception_params.sh` 再生成，`test_vision_params_gen.py` 精确再生校同步。主节点两行接 `ScenePerceptionParams.declare + from_params` 改一行 `attach(node)`，on-set 动态刷新：逐帧读取键 `ros2 param set` 即时热生效；构造期捕获键（模型/管线/记忆/收齐策略）仍需重启。

踩坑（生成物源码核实）：① GPL Python 生成类的嵌套组是**类级共享实例**（`pipeline = __Pipeline()` 为类属性），对 Params 整体 deepcopy 不复制它们、update() 原地改共享实例会穿透所有引用——手写版是实例属性无此问题；派生层 `_structural_copy`（逐属性 SimpleNamespace 树）隔离后才实现「派生失败整体冻结在上一份一致快照」。② Python 端校验算子无跨参数比较（lt_param 仅 C++），`min_depth < max_depth` 留派生层。③ apt 入口点 `generate_parameter_library_python` 缺 dist-info 元数据跑不起来，用 `python3 -m generate_parameter_library_py.generate_python_module` 直调模块。④ 生成物 `# flake8: noqa` 全文件豁免、ament pep257 约定本就忽略 D100-D107，lint 无需改测试。

验收：colcon test peach_harvester 110/110 绿（新增同步门 1、派生层纯核 5、冻结键测试改读声明 yaml）；隔离域 rclpy 冒烟——部署清单覆盖声明、非法 set 拒绝、热更新重建 ToolGeometry、跨字段坏组合冻结+恢复全过。`vision/params.py` 删 scene 类仅剩重建（键冻结测试拆两侧）；package.xml 增 `generate_parameter_library_py` exec_depend。活文档 AGENTS（缝位+偏离表）/architecture（参数模块行、文件树、§感知、§参数分层、GPL 行）/testing（venv 段、调参分层）同轮。遗留：target_reconstruction / supervisor / observability / vegetation 仍 0017 手写，迁法照本 round。

---

## 2026-09-18 Python 参数改为 yaml 直读（决策 0024）

删 scene 的 GPL Python 生成物（`scene_perception.params.yaml` / `params_gen.py` / `scripts/gen_scene_perception_params.sh` / `test_vision_params_gen.py`）与重建/调度/观测/lifecycle/vegetation 的手写 ParamListener DEFAULTS 双源。共用 `yaml_params.attach(node, yaml)`：按部署清单叶子 `declare_parameter`，`ros2 param set` 原地改同一棵 namespace；主节点一行 `Xxx.attach(self)`。空 YOLO/SAM 路径在参数层拒绝，不写主节点。`peach_arm` C++ 仍 GPL。活文档 architecture / io / testing 同轮。

### 同日补全轮（校验防线 / bringup / 清单核对）

0024 落地时 attach 未挂校验，0017 的数值规则防线丢失——本轮补回并推广到全部 peach Python 节点：各 params 模块持 `_RULES` 规则表经 `validate=` 进 attach（越界/白名单启动期拒启、运行期非法 set 即拒；scene 完整 33 键含空模型路径、重建 48 键、supervisor 8 键、observability 9 键、vegetation 12 键、lifecycle 1 键），跨字段窗经 `preview=`（scene `min_depth<max_depth`、supervisor 选果 reach/depth 双窗、vegetation `leaf.h_min<h_max`：整批拒绝、保持当前一致快照）。`peach_bringup` 两小组件（lifecycle_flag_bridge / autostart_client）从内联 declare 迁入 `config/bringup.yaml` + `peach_bringup.params` attach（下限规则；autostart 的 scene_key/intent 发批时热读）；setup.py 装 config。接口清单核对器 `_CONSUMER_PATHS` 补 `peach_bringup/config`（服务名字面量随参数迁 yaml 后反向扫描失配）。测试：各包新增「规则键⊆部署清单键」对账 + vegetation/observability/bringup validate 单测；r0_gate 四包 + 清单全绿；隔离域真 rclpy 冒烟（规则拒非法、跨字段拒、热更新、派生重建）全过。三份 `yaml_params` 副本（harvester/vegetation/bringup）byte 级一致并加 vendoring 注记（与 param_rules 同款每包自持模式）。AGENTS 残留 GPL 措辞清一致性（缝位/偏离表/反模式/新节点流程）。

---

## 2026-09-30 peach2 M1 骨架退出门

隔离重写 `src/peach2/`（不进 `harvest_system`）。方案 §14 M1 退出门关闭：launch_testing 全绿 + mock PREGRASP 链。生产入口仍是旧 `peach_*`。

验收数字：`peach2_system_tests` 33/33（域 95–97）；`peach2_manipulation` gtest 191/0 fail。四处系统测 xfail 收口为硬断言：`from bondpy.bondpy import Bond`；`RunBatch(execution=false)` goal REJECT（`execution_disabled`）；拍照位 MoveTo `at_goal` 空运动；PREGRASP_ONLY `reached` 停在 PREGRASP。残差 LIN `collapse_at_goal=false`，默认 `at_goal_tolerance_rad=0.002`。

未过：M0 台架（标定文件仍 `design_reference` → `cut_allowed` 恒 false）、mock FULL、果园指标、YOLO11 v2.1、TensorRT、腕力。下一步：M0 或 mock FULL 系统测（仍禁止真机 SetIO）。

活文档：architecture 产品定位补 peach2 隔离条；testing §1 补系统测入口；方案两份（`docs/` 与 `.cursor/plans/`）§14 现行进度。包 README：`peach2_system_tests` / `peach2_bringup` / `peach2_calibration` / `peach2_manipulation`。未新开第四份活文档。

