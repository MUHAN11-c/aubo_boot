# 契约预留的批次事件形状（含 cycle_id，无 state_seq）。
# 线上编排器发布 CanonicalEvent，本类型未接线；保留以免下游记录器字段分叉。
# 新代码请用 CanonicalEvent。

uint8 INFO=0
uint8 WARNING=1
uint8 ERROR=2
uint8 AUDIT=3

std_msgs/Header header
uint64 sequence          # 节点内单调序号
uint8 severity           # INFO/WARNING/ERROR/AUDIT
string code              # 稳定事件码
string message           # 人读说明
string request_id
string run_id
string cycle_id          # 单目标周期 ID（run_id:target_id）；批次级事件可为空
string target_id
diagnostic_msgs/KeyValue[] details
