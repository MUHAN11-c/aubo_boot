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

## Remaining（明确不做 / 记下缺口）

- C++ `stages.cpp` 未拆（接触安全面）。
- 两个感知节点壳与 `executor_node.py` 仍大（计划允许）。
- lifecycle 无 bond（architecture 已记缺口；现场未出现「静默死掉而名单仍 Active」）。
- 驱动/厂商 `package.xml` 漂移默认不动。
- `_collect_bag_views` landmarks 重复写入未改。
- 未提交的现场 `web/` 与若干运行 yaml 未覆盖。
