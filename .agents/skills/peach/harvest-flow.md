# 一批 RunHarvest

源：`harvest_fsm.py`（纯核表）+ `domain/reducer.py`（三维 reducer）+ `executor_node.py`（命令循环）。活文档对照 architecture §4 图 C。

## 开批

人发 `/peach_supervisor/run_harvest`（**launch 默认不发**；`autostart:=true` 才由 `peach_autostart_client` 发，授权=操作员发起 real launch）。

`_run_harvest`：空 `request_id` → `auto_<时间戳>`（账本不复用）。`react(WAITING_READY, RUN_REQUESTED)` → DISCOVERY / NAVIGATE。`finally` 落 `runs/<request_id>/ledger.json` 与 `rework_list.json`。

## 命令循环

`_run_harvest_body`：`reaction.command` 直到 SETTLE/ABORT/INTERRUPT。

| Command | handler | 做什么 |
|---------|---------|--------|
| NAVIGATE | `_cmd_navigate` | 固定座直通 `NAV_OK`，不发 `NavigateToWorksite` |
| SURVEY | `_cmd_survey` | `SurveyScene`：臂 PTP 拍照位；首巡成功 → `SURVEY_AT_POSE`；回访 → `SURVEY_DONE`；失败 INTERRUPTED |
| BEGIN_SCENE | `_cmd_begin` | **仅首巡** `scene_epoch==0` 后一次；换 `scene_key` 清身份表 |
| WAIT_LOCK | `_cmd_wait_lock` | 等观测 `scene_epoch` 对齐且 `target_set_locked`；SURVEY_ONLY→结算；`execution_enabled` 关→RECORD_DISABLED 出循环 |
| SELECT | `_cmd_select` | 采收率门 → `CheckReachability` → `next_target` → claim |
| DISPATCH | `_cmd_dispatch` | 并行 Build + 观察；过门 → READY_FULL |
| EXECUTE_FULL | `_cmd_full` | `execute_pregrasp_only` 默认 true → PREGRASP_ONLY；否则 FULL。`skip_observation=true`，带着 clearance 令牌 |
| NONE | `_cmd_cycle_done` | CYCLE_DONE → 回 DISCOVERY 再 Survey（**禁止再 Begin**） |

RECORD_DISABLED / 未知命令：handler 表无项 → break。

暂停：`operation_mode=PAUSED`，**不覆盖**作业 `batch_state`。`EventHold` 暂存第一条非 NONE 命令，RESUME 只释放一次。接触/PREGRASP 真动过后 `recovery_required`，须 ACK 才 Survey / 派下一颗。

## FSM 表（harvest_fsm._TABLE）

```
WAITING_READY + RUN_REQUESTED → DISCOVERY / NAVIGATE
DISCOVERY + NAV_OK            → SURVEY
DISCOVERY + SURVEY_AT_POSE    → BEGIN_SCENE
DISCOVERY + BEGIN_OK          → WAIT_LOCK
DISCOVERY + LOCK_READY|SURVEY_DONE → SELECT
DISCOVERY + TARGET_SELECTED   → RUNNING / DISPATCH
DISCOVERY + NO_TARGET         → 再 SURVEY（不 Begin）
DISCOVERY + EMPTY_LIMIT|SURVEY_ONLY|EXECUTION_DISABLED → SETTLE
DISCOVERY + NAV/SURVEY/BEGIN_FAILED → INTERRUPTED / ABORT
RUNNING + READY_FULL          → EXECUTE_FULL（节点再选 PREGRASP/FULL）
RUNNING + FULL_* / SKIP / OBSERVE_FAILED / BUILD_FAILED → NONE（_cmd_cycle_done）
RUNNING + CYCLE_DONE          → DISCOVERY / SURVEY
任意 + CANCEL                 → INTERRUPTED / INTERRUPT
```

未知组合：保持原态，command=NONE。

## 选果（batch.next_target）

输入：最近 `/peach/perception/target_observations`（**调度不订 initial_pose**）+ claimed 集。

资格：锁定集已确认、非裸果产品范围、未贴边。preferred=`goal.target_ids` 也过资格谓词。

窗：有效深度 ∩ TCP 可达（`CheckReachability`：入口换成预抓取停位 IK；FULL 再套入终点 IK+沿轴笛卡尔）。服务不可用回退入口半径窗（现场约 0.15–0.88 m）。

次序：`priority` 升序，同级检测框面积降序（近距大框优先）。超窗进 `targets_filtered` 事件。

跳过是策略不是失败：采收率达标 / 单果时限 / 人工 SKIP → 入 `ReworkList`。

## DISPATCH 观察两档

`BatchPolicy.view_policy`：`VIEW_FAST=0`（默认）supervisor 直驱补视（`observe.py`，封顶约 3 视）；`VIEW_CONSERVATIVE=1` 发臂 `ExecuteTarget OBSERVE_ONLY`（最多 4 次重试）。

同时发 `BuildTargetModel`。等 COLLECTING 后观察，再 `_wait_build_after_observe`（`min_views` + grace）。失败走 `_fail_dispatch` 记 SKIPPED_QUALITY。

例外：

- `skip_reconstruction`：不 Build/不观察，unrefined 七元组进接触（须臂 `allow_unrefined_geometry`）
- `reconstruct_in_trajectory`：Build 已 COLLECTING 即 READY_FULL + skip_observation（默认关）

## EXECUTE_FULL 装配要点

goal 带：`run_id/cycle_id/target_id/scene_epoch`、三 revision、`plan_id={request_id}:{target_id}:{generation}`、`clearance`←缓存的 `GraspDecision`、`generation`。臂侧双路：令牌优先，快照回退。

outcome → `event_for_outcome`：0 SUCCEEDED / 1–2 SKIPPED / 3 FAILED / 4 CANCELED。PREGRASP 到位也 `recovery_required=true`。

## ControlTask

`permissions_for(batch_state, recovery_required, paused)`。接触锁与批次态无关：空闲也必须能 ACK。MAINTENANCE 未接线。
