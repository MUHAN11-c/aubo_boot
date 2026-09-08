# 技能本周期抓取假说。只读监控用，不构成运动指令，也不是批次门。
# 话题：/peach/manipulation/grasp_hypothesis（reliable, transient_local, depth 1）
# 生产：peach_manipulation    消费：peach_observability（调度/重建不订）
# 实际接触几何以 ExecuteTarget 内部 MTC 为准。

std_msgs/Header header             # stamp=本周期规划时刻，frame_id=base_frame
string target_id                   # 当前目标
geometry_msgs/Pose entry_pose      # 袋外预入口（尖端/工具，与技能约定一致）
float32 standoff_m                 # 袋外余量 [m]
float32 travel_m                   # 插入行程 [m]
float32 envelope_clearance_m       # 包络净空 [m]；未估则 0
float32 rank_score                 # 排序分；精化几何约 1，降级锚点约 0.5
string[] diagnostic_flags          # 如 degraded_anchor
