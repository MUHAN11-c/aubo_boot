# 一批 RunHarvest

源：`harvest_fsm.py`（纯核表）+ `domain/reducer.py`（三维 reducer）+ `executor_node.py`（命令循环）。活文档对照 `peach_bringup/doc/architecture.md` 与 `peach_harvester/doc/`。

## 开批

人发 `/peach_supervisor/run_harvest`（**launch 默认不发**；`autostart:=true` 才由 `peach_autostart_client` 发，授权=操作员发起 real launch）。autostart 硬等 observability `/peach/observability/selfcheck_passed`。

`_run_harvest`：空 `request_id` → `auto_<时间戳>`（账本不复用）。`react(WAITING_READY, RUN_REQUESTED)` → DISCOVERY / NAVIGATE。`finally` 落 `runs/<request_id>/ledger.json` 与 `rework_list.json`。

## 命令循环

`_run_harvest_body`：`reaction.command` 直到 SETTLE/ABORT/INTERRUPT。

| Command | handler | 做什么 |
|---------|---------|--------|
| NAVIGATE | `_cmd_navigate` | 固定座直通 `NAV_OK`，不发 `NavigateToWorksite` |
| SURVEY | `_cmd_survey` | `SurveyScene` 到拍照位；**成功即发** `/peach/scene/obstacles_refresh`；首巡 → `SURVEY_AT_POSE`；回访 → `SURVEY_DONE` |
| BEGIN_SCENE | `_cmd_begin` | **仅首巡** `scene_epoch==0` 后一次；换 `scene_key` 清身份表 |
| WAIT_LOCK | `_cmd_wait_lock` | 等观测 `scene_epoch` 对齐且 `target_set_locked`；三路：`SURVEY_ONLY`→SETTLE；`execution_enabled` 关→`EXECUTION_DISABLED`/`RECORD_DISABLED`；否则 `LOCK_READY`→SELECT |
| SELECT | `_cmd_select` | 采收率门 → `CheckReachability`（`reach_queries`）→ `next_target` → claim |
| DISPATCH | `_cmd_dispatch` | 默认 Build 绑定后立刻派臂（见下）；过门 → READY_FULL |
| EXECUTE_FULL | `_cmd_full` | `execute_pregrasp_only` 默认 true → PREGRASP_ONLY；否则 FULL。`skip_observation=true`；默认并行 Build **不装**未 finalize 令牌 |
| NONE | `_cmd_cycle_done` | 点名已摘完 → EMPTY_LIMIT 结算；否则 CYCLE_DONE → 回 DISCOVERY 再 Survey（**禁止再 Begin**） |

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
DISCOVERY + EMPTY_LIMIT|SURVEY_ONLY → SETTLE
DISCOVERY + EXECUTION_DISABLED     → COMPLETED / RECORD_DISABLED
DISCOVERY + NAV_FAILED             → INTERRUPTED / ABORT  event=navigate_failed
DISCOVERY + SURVEY_FAILED     → INTERRUPTED / ABORT  event=survey_failed
DISCOVERY + BEGIN_FAILED      → INTERRUPTED / ABORT  event=begin_scene_failed
RUNNING + READY_FULL          → EXECUTE_FULL（节点再选 PREGRASP/FULL）
RUNNING + FULL_* / SKIP / OBSERVE_FAILED / BUILD_FAILED → NONE（_cmd_cycle_done）
RUNNING + CYCLE_DONE          → DISCOVERY / SURVEY
RUNNING + EMPTY_LIMIT         → COMPLETED / SETTLE（点名 target_ids 全部入账）
任意 + CANCEL                 → INTERRUPTED / INTERRUPT
```

未知组合：保持原态，command=NONE。

单果墙钟预算 40 s（相对人工偏保守）。点名 `target_ids` 摘完走 `EMPTY_LIMIT`，不再空扫。2026-10-16 真实相机+mock 5/5 见 `peach_system_tests/doc/testing.md`（账本 24–27 s，RunHarvest 38–43 s）。

## 选果（batch.next_target）

输入：最近 `/peach/perception/target_observations`（**不订 initial_pose**）+ claimed 集。

资格 `_eligible_locked_items`（`next_target` 与 `reach_queries` **共用**，防两份过滤漂移）：锁定集、已确认、未入账、非裸果（`unbagged_display_only` / `strategy_id` 含 `fruit`）、非贴边 `bbox_edge`、非 `tool_clearance_failed`。`goal.target_ids`（preferred）**过同一资格谓词**，但**不受**深度/IK 窗限；`reach_queries` 不含 preferred（点名不进 IK 预检名单）。

窗：有效深度（参数 `selection_depth_*`）∩ TCP 可达（`CheckReachability`：入口换成预抓取停位 IK。默认 PREGRASP_ONLY 不检套入；仅 `execute_pregrasp_only=false` 才 `require_sleeve`）。服务不可用回退入口半径窗（参数 `selection_reach_*`，函数默认 0.15–0.88 m）。SELECT 用感知建议行程，Finalize 用精化行程，两次数值不同是预期。

次序：`priority` 升序，同级检测框面积降序。超窗进 `targets_filtered` 事件。

跳过是策略不是失败：采收率达标 / 单果时限 / 人工 SKIP → 入 `ReworkList`。

## DISPATCH

默认 `reconstruct_in_trajectory=true`（`peach_supervisor.yaml`）：Build 进 COLLECTING 后立刻 `READY_FULL` + `skip_observation`，不走下方补视。臂侧 Finalize 非观察周期先等 GraspDecision（提交含精化几何），不许用粗几何先动；不再因 `waitForRefined` 先超时而决策到达后仍跳过。

`skip_reconstruction`：不 Build/不观察，unrefined 七元组进接触（须臂 `allow_unrefined_geometry`）。臂侧 promote 把 `suggested_travel_m` 置 0，走 `(neck-entry)-neck_margin` 回退（实测 0.05–0.10 m）；感知 `suggested_travel_m` 也不再用 `L_insert=0.03` 封顶（2026-10-21）。

仅显式 `reconstruct_in_trajectory=false` 才走观察两档：`BatchPolicy.view_policy` `VIEW_FAST=0` supervisor 直驱补视（`observe.py`，拍照位算 1 视、补视封顶 2、总视 ≤3）；`VIEW_CONSERVATIVE=1` 发臂 `ExecuteTarget OBSERVE_ONLY`（goal 被拒最多 4 次重试）。同时发 `BuildTargetModel`。等 COLLECTING 后观察，再 `_wait_build_after_observe`（`min_views` + grace）。失败走 `_fail_dispatch` 记 SKIPPED_QUALITY。

## EXECUTE_FULL 装配要点

goal 带：`run_id/cycle_id/target_id/scene_epoch`、三 revision、`plan_id={request_id}:{target_id}:{generation}`、`generation`。`skip_reconstruction` 或默认 `reconstruct_in_trajectory` 时 **不装** 未 finalize 的 GraspDecision 令牌（臂 Finalize 等精化后走快照复检）；仅串行补视路径且缓存 `target_id` 匹配才装配 `clearance`。臂侧双路：令牌优先，快照回退。

outcome → `event_for_outcome`：0 SUCCEEDED / 1–2 SKIPPED / 3 FAILED / 4 CANCELED。PREGRASP 到位也 `recovery_required=true`。

## ControlTask

`permissions_for(batch_state, recovery_required, paused)`。接触锁与批次态无关：空闲也必须能 ACK。MAINTENANCE 未接线。
