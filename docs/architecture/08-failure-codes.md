# FailureCode

`peach_interfaces/msg/FailureCode.msg` 是跨包稳定数值码来源。事件、账本、动作 Result 和诊断应使用常量，不解析自由文本。

## 已完成

- supervisor 的 `execution_disabled`、`canceled`、`observe_failed`、`build_rejected`、`build_start_timeout`、`observe_build_view_race`、`timeout` 已改成枚举名。
- `reconstruction_core/refine/publish` 的 `3/4/11/12` 已改成 `DECISION_REJECTED`、`PREGRASP_RESIDUAL`、`EXACT_TF_MISSING`、`DYNAMIC_BUDGET_NEGATIVE`。
- supervisor 遥测自动补 `failure_code_n`；旧字符串保留兼容读取。

## 待完成

- 感知/重建纯核中剩余裸数字（如 `16`）的语义归并。
- 日志字段统一：`event/request_id/cycle_id/target_id/scene_epoch/failure_code/elapsed_ms`。
- 自动检查脚本：扫描裸数字/自由字符串 failure_code。

- `CUT_FRUIT_CLEARANCE=35`：刀口/果距不足；原 `16` 是 `CORRIDOR_BLOCKED`，已不再复用。
