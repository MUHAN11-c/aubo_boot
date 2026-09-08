# 【预留】批次事件旧形状（含 cycle_id，无 state_seq）。线上未接线。
# 新代码与监控请用 CanonicalEvent。保留本类型以免下游记录器字段分叉。

uint8 INFO=0                           # 过程信息
uint8 WARNING=1                        # 可恢复告警
uint8 ERROR=2                          # 失败/中断
uint8 AUDIT=3                          # 人工操作审计

std_msgs/Header header                 # stamp=事件时刻
uint64 sequence                        # 节点内单调序号
uint8 severity                         # INFO / WARNING / ERROR / AUDIT
string code                            # 稳定事件码（与 CanonicalEvent.code 同一套）
string message                         # 人读说明
string request_id                      # RunHarvest.request_id
string run_id                          # 批次 run_id
string cycle_id                        # 单目标周期 ID（run_id:target_id）；批次级可空
string target_id                       # 相关目标；批次级可空
diagnostic_msgs/KeyValue[] details     # 可选键值
