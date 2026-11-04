# 日志与错误码规范

## 原则

- 结构化字段优先，文本只做人类可读补充。
- 稳定错误码只来自 `peach_interfaces/msg/FailureCode.msg`；不新增自由字符串码。
- 逐帧 INFO 禁止；启动/状态变化/失败/耗时用 INFO/WARN/ERROR。
- ROS 节点用 `get_logger()`，不用 `print`；高频失败要节流。
- 日志不得作为控制/安全通道；功能安全仍走硬件急停。

## 稳定字段

| 字段 | 含义 | 典型出现 |
|---|---|---|
| `event` | 事件名（snake_case） | 调度/技能/观测 |
| `request_id` | 批次请求 ID | supervisor / arm / events |
| `cycle_id` | 单果周期 ID | 调度/技能 |
| `target_id` | 感知注册 ID | scene/recon/supervisor/arm |
| `scene_epoch` | 场景世代 | scene/recon/supervisor |
| `failure_code` | FailureCode 名称 | 失败路径 |
| `failure_code_n` | FailureCode 数值 | 遥测 details |
| `elapsed_ms` / `elapsed_s` | 阶段耗时 | 动作/重建/接触 |
| `source_uuid` / `sequence` | 力帧身份 | serial_force |

## Logger 名

- scene：`peach_scene_perception_node` / `peach_scene_perception.*`
- reconstruction：`peach_target_reconstruction_node`
- supervisor：`peach_supervisor` / `peach_supervisor.*`
- arm：`peach_arm.*`
- observability：`peach_observability`
- force：`serial_force`

## 排查入口

1. `/api/state` 与页面事件时间线。
2. `runs/session_*/events.jsonl` / `ledger.json`。
3. 会话 bag（MCAP）+ `bag_report.md`。
4. `/diagnostics` 聚合。
5. `runs/debug_audit/*.jsonl`（调试 POST）。

## FailureCode

见 [08-failure-codes.md](08-failure-codes.md)。`15`/`16` 等旧复用已拆分；`CUT_FRUIT_CLEARANCE=35` 专门表示刀口果距不足。
