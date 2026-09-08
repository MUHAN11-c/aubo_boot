# 编排器过程/审计事件。线上唯一事件流（不要用预留的 HarvestEvent）。
# 话题：/peach_executor/events（reliable, transient_local, depth 50）
# 生产：peach_executor    消费：peach_observability
# 监控与 JSONL 按 code 聚合，不要只解析 message。

# 现行 code 全集：
#   终局目标：target_dispatched / target_succeeded / target_skipped /
#     target_failed / target_canceled / target_operator_skipped
#   过程：photo_pose_reached / round_locked / survey_failed
#   选果过滤：targets_filtered（details 列超窗目标与原因
#     out_of_reach_window / out_of_depth_window / ik_no_solution）
#   人工审计：batch_paused / batch_resumed / recovery_required /
#     recovery_acknowledged（details 可带 ControlTask reason）
#   派发异常：observe_build_view_race（观察成功但 Build 机位数不足）

std_msgs/Header header    # stamp=事件时刻，frame_id 一般为空或 base_link
uint64 sequence           # 节点内单调序号（预留，现行恒 0；与 HarvestState.state_seq 独立）

uint8 INFO=0              # 过程信息
uint8 WARNING=1           # 可恢复告警
uint8 ERROR=2             # 失败/中断
uint8 AUDIT=3             # 人工操作审计
uint8 severity            # INFO / WARNING / ERROR / AUDIT

string code               # 稳定事件码（见上）
string message            # 人读说明；执行器终局常填 JSON（含 failure_code）
string request_id         # 对应 RunHarvest.goal.request_id；无批次时为空
string run_id             # 批次 run_id
string target_id          # 相关目标；批次级事件可为空
uint64 state_seq          # 发出时的 HarvestState.state_seq，便于与快照对齐
diagnostic_msgs/KeyValue[] details  # 可选键值（失败码、阶段名、过滤原因、ACK reason）
