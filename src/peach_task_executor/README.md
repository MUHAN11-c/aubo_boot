# peach_task_executor

五个能力包之一：**整栈调度**。三节点：执行器（开批/选果/账本）、lifecycle 管理器（五节点名单）、observability（只读 Web/jsonl）。串联导航适配、视觉、臂技能。不做视觉、不直接规划接触或导航。现行只接通感知抓取；默认跳过 `NavigateToWorksite`。**launch 绝不自动开批。** 详细作用见 [docs/architecture.md](../../docs/architecture.md) §3 `peach_task_executor`。

总览：[docs/architecture.md](../../docs/architecture.md)。跨包调用：[docs/io.md](../../docs/io.md)。怎么跑：[docs/testing.md](../../docs/testing.md)。

```
peach_task_executor/
  peach_task_executor/       # import peach_task_executor.*
    task_executor_node.py
    harvest_fsm.py
    observability/           # peach_observability 节点
  config/  launch/  web/
  resource/  test/
  package.xml  setup.py  setup.cfg
```

## 从哪读

| 文件 | 职责 |
|------|------|
| `harvest_fsm.py` | 纯核：`(batch_state, Event) → Reaction`（下一态、`command`、记录器事件码） |
| `task_executor_node.py` | Lifecycle 节点：`_run_harvest` 执行 `command`；`_make_state` 填满 `HarvestState` |
| `control.py` | `ControlTask`：`expected_state_seq` 防乱序 |
| `select.py` | 下一个 `target_id`（goal 优先，否则已确认观测） |
| `ledger.py` | `runs/<request_id>/ledger.json` 读写 |
| `summary.py` | `HarvestSummary` 计数 |
| `lifecycle_manager.py` | 有序 configure→activate；逆序拆除；`~/manage_nodes`；闩锁 `/peach/lifecycle/managed_nodes_activated` |
| `observability/` | 只读 Web / JSONL；不发运动 |
| `config/peach_task_executor.yaml` + `peach_task_executor/task_executor_parameters.yaml` | 运行 yaml 与 generate_parameter_library 根键都是节点名 `peach_task_executor` |
| `config/observability.yaml` | 监控参数 |
| `launch/harvest_system.launch.py` | 整栈 include；能力包 `autostart:=false` |

读批次：先 `react()` 表，再 `_run_harvest` 里对 `Command.NAVIGATE` / `BEGIN_SCENE` / `SURVEY` / `DISPATCH` / `EXECUTE_FULL` / `SETTLE` 的分支。

## 谁调谁

本节点是客户端：`NavigateToWorksite`（仅 `navigation_enabled`）、`BeginScene`、`SurveyScene`、`BuildTargetModel`、`ExecuteTarget`。取消时 `_cancel_inflight` 级联所有在飞 handle。

订阅 `/peach/perception/target_observations`。发布闩锁 `~/state`、`~/events`、`~/scene_snapshot`。

`require_managed_stack`（整栈 launch 为 true）：未收到 lifecycle 旗标则拒绝 `RunHarvest`。

## 流程（现行源码）

```
显式 RunHarvest（须 Lifecycle Active，且栈就绪）
  → NavigateToWorksite（navigation_enabled=false 则跳过）
  → BeginScene（感知：清身份、scene_epoch）
  → SurveyScene（技能：拍照位姿）
  → 等待 survey_wait_s 或 target_set_locked
  → 无目标则再扫，连续 empty_survey_limit 次空集结算；SURVEY_ONLY 只扫不采
  → execution_enabled=false：Survey 后直接结算
  → 循环未入账目标
       → 并行 BuildTargetModel + ExecuteTarget OBSERVE_ONLY
         → 观察失败则取消 Build；模型失败则 SKIPPED_QUALITY
         → ExecuteTarget FULL（skip_observation）
  → 账本落盘（同 id 可续跑）→ HarvestSummary
```

PAUSE 时循环卡住。`SKIP_TARGET` 跳过当前目标。`~/state.target_id` 是重建/感知的当前作业绑定。

## 启动

```bash
ros2 launch peach_task_executor peach_task_executor.launch.py
ros2 launch peach_task_executor observability.launch.py
ros2 launch peach_task_executor harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
```

整栈默认 `hardware_mode:=mock`、`camera_enabled:=false`；真机须显式 `real`。

## 接口

| | 名 |
|--|--|
| 动作 | `~/run_harvest` |
| 服务 | `~/control`（`ControlTask`） |
| 生命周期管理 | `/peach_lifecycle_manager/manage_nodes`（`ManageLifecycleNodes`） |
| 发布 | `~/state`、`~/events`、`~/scene_snapshot`（TRANSIENT_LOCAL） |

参数要点：`execution_enabled` 默认 false；`navigation_enabled` 默认 false；`survey_wait_s` 8；`empty_survey_limit` 2；`persist_ledger` true。

禁止自动 `RunHarvest`。bringup 不起 `aubo_dashboard`。
