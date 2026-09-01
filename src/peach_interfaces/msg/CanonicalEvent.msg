# 编排器过程/审计事件。peach_executor 的 ~/events 发布（transient_local）。
# 事件码须与 canonical_code_for_outcome 一致：target_dispatched / target_succeeded /
# target_skipped / target_failed / target_canceled / target_operator_skipped / round_locked。
# HarvestEvent 是契约预留，线上走本消息。

std_msgs/Header header
uint64 sequence          # 节点内单调序号（与 HarvestState.state_seq 独立）
uint8 INFO=0
uint8 WARNING=1
uint8 ERROR=2
uint8 AUDIT=3
uint8 severity           # INFO/WARNING/ERROR/AUDIT
string code              # 稳定事件码（见上）；监控与 JSONL 按此聚合，不要只解析 message
string message           # 人读说明；执行器常填 JSON 载荷
string request_id        # 对应 RunHarvest goal.request_id；无批次时为空
string run_id            # 批次 run_id
string target_id         # 相关目标；批次级事件可为空
uint64 state_seq         # 发出时的 HarvestState.state_seq，便于与状态快照对齐
diagnostic_msgs/KeyValue[] details  # 可选键值（失败码、阶段名等）
