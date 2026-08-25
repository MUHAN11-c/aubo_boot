# 技能侧抓取假设（只读监控）。/peach/manipulation/grasp_hypothesis 闩锁发布。
# 不构成运动指令；实际接触几何以 ExecuteTarget 内部 MTC 为准。

std_msgs/Header header             # frame_id=base_frame
string target_id
geometry_msgs/Pose entry_pose      # 袋外预入口（尖端/工具，与技能约定一致）
float32 standoff_m                 # 袋外余量 [m]
float32 travel_m                   # 插入行程 [m]
float32 envelope_clearance_m       # 包络净空 [m]；未估则 0
float32 rank_score                 # 排序分；精化几何约 1，降级锚点约 0.5
string[] diagnostic_flags          # 如 degraded_anchor
