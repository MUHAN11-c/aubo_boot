# 重建精化几何的只读抓取许可（/peach/reconstruction/grasp_decision，
# transient_local 闩锁，1Hz 心跳）。不发送任何运动指令。
# allowed=false 时 entry/axis/diameter_m/rmse_m/inlier_ratio 全部无意义：
# entry/axis 填零、标量填 0/-1（占位，禁止消费）——等价于 refined_pose
# 发空数组覆盖的闩锁惯例，防止后启动订阅者误用陈旧几何；reason 是唯一有效信息。

std_msgs/Header header  # stamp=发布时刻，frame_id=base_frame

string harvest_run_id  # 所属批次（空=无批次）
string target_id       # 许可对应的目标（空=未绑定；消费方按当前目标调和）
bool allowed           # true=精化几何达标允许抓取
string reason          # 判定原因（reconstruction_not_ready/refined_geometry_unavailable/
                       # refined_quality_requires_reobserve/refined_geometry_accept）

# 以下字段仅 allowed=true 有效（entry/axis 为 base 系）：
geometry_msgs/Point entry      # 抓取进入点（bottom − axis×standoff）[m]
geometry_msgs/Vector3 axis     # 精化轴（bottom→neck 单位向量）
float64 diameter_m             # 袋径/果径（= 2r）[m]
float64 rmse_m                 # 拟合 RMSE [m]
float64 inlier_ratio           # 拟合内点比 [0, 1]
