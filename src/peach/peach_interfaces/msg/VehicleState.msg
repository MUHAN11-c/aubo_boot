# 【预留】外部导航 → 采摘：车辆是否到位且静止。导航拥有路径主权。
# 话题：/peach/navigation/vehicle_state（清单 reserved_interfaces）
# 生产/消费：固定座核心栈均无。随 NavigateToWorksite 一并归档。

std_msgs/Header header             # stamp=导航状态时刻
string target_id               # 导航侧关联的作业目标；可空
string site_id                 # 作业位点 ID
bool matched                   # 是否匹配到目标/站点
bool arrived                   # 是否到达作业位
bool stationary                # 是否静止（采摘允许动臂的前提）
bool fresh                     # 本状态在 freshness_s 内
float32 freshness_s            # 新鲜度窗 [s]
string message                 # 人读说明
