# FailureCode 与日志投影

跨包稳定数值由 [FailureCode.msg](../../src/peach/peach_interfaces/msg/FailureCode.msg) 定义，当前常量范围为 0–35。消息、动作结果和服务响应使用这些数值；日志与旧账本的字符串名称是投影/兼容表示，不是另一套数值分配源。

## 当前实现

- supervisor 在节点边界使用 FailureCode 枚举名组织失败事件，`executor_node._emit` 为有错误名称的遥测补 `failure_code_n`；未知旧名称的回退数值不能当作“成功”。具体阶段原因仍须结合 outcome 和批次终态。
- 重建的 `refine.py` / `publish.py` 使用 `DECISION_REJECTED`、`PREGRASP_RESIDUAL`、`EXACT_TF_MISSING`、`DYNAMIC_BUDGET_NEGATIVE` 等常量。失败码不能替代模型身份、能力与时效检查。
- 零 ROS 的 `peach_vision_core/tool_budget.py` 仍以带常量注释的 `35` 表示 `CUT_FRUIT_CLEARANCE`；该函数的 reason、sleeve_ok、cut_ok 不保证一一对应，不能只看 failure_code 推导阶段授权。重建还会依据袋口几何和观测证据重组最终能力。
- `CORRIDOR_BLOCKED=16` 与 `CUT_FRUIT_CLEARANCE=35` 语义不同，历史数据不得把二者相互改写。0 表示 OK；UNKNOWN、取消、超时、未执行应按各自既有字段解释。

## 维护与核对

新增或修改跨包错误码先改 IDL，再核对生产/消费方、接口清单和包级 IO；不能通过解析自由文本消息恢复错误码。纯核数值、节点枚举与离线兼容映射需要按来源逐项核对，不将尚未存在的全局错误码扫描脚本写成已部署检查。

日志身份与时间字段规范见 [10-logging.md](10-logging.md)。当前检查入口为 `python3 src/peach/peach_interfaces/scripts/check_interface_manifest.py`，它验证图名/清单一致性，不证明每条运行失败路径的语义或真实采摘正确性。
