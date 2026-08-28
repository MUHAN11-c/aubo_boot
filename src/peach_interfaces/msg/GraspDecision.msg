# 重建精化几何的只读抓取许可（/peach/reconstruction/grasp_decision，
# transient_local 闩锁，1Hz 心跳）。不发送任何运动指令。
# 融合成功时 entry/axis/pregrasp/cut_pose 有效，供预抓取与目视。
# allowed 只表示套入/剪切接触许可；allowed=false 不得降级接触。
# 无融合几何时入口/轴填零，reason/failure_code 说明原因。

std_msgs/Header header  # stamp=发布时刻，frame_id=base_frame

string harvest_run_id
string target_id
string model_revision
string tool_profile_id
bool allowed
string reason
uint32 failure_code            # FailureCode.*；0=无失败

# 以下字段在融合几何有效时填写（entry/axis 为 base 系）；
# allowed=false 时仍可有几何，仅禁止套入/剪切：
geometry_msgs/Point entry
geometry_msgs/Point pregrasp
geometry_msgs/Point cut_pose
geometry_msgs/Vector3 axis
float64 diameter_m
float64 d95_m
float64 travel_m
float64 cut_travel_m
float64 radial_margin_m
float64 axial_margin_m
bool corridor_clear
float64 rmse_m
float64 inlier_ratio
