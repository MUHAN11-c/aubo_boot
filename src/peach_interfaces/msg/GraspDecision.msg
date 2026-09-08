# 融合后的抓取几何 + 套入/剪切许可。不发送任何运动指令。
# 话题：/peach/reconstruction/grasp_decision（reliable, transient_local, depth 1；约 1 Hz 心跳）
# 生产：peach_target_reconstruction    消费：peach_manipulation、peach_observability
# allowed 只拦套入和 SetIO，不拦预抓取。allowed=false 时仍可有几何，禁止降级接触。
# 无融合几何时 entry/axis 填零，reason / failure_code 说明原因。

std_msgs/Header header    # stamp=发布时刻，frame_id=base_frame（入口/轴坐标系）

string harvest_run_id     # 所属批次；空=无批次
string target_id          # 绑定目标；空=未绑定
string model_revision     # 模型修订；空=无模型
string tool_profile_id    # 工具剖面；空=未指定
bool allowed              # true=允许套入/剪切；false=禁止接触（预抓取仍可走）
string reason             # 许可/拒绝原因（稳定短语，不是自由散文）
uint32 failure_code       # FailureCode.*；0=NONE 无失败

# 以下在融合几何有效时填写（base 系）；allowed=false 时仍可有值：
geometry_msgs/Point entry       # 袋外预入口位置 [m]
geometry_msgs/Point pregrasp    # 预抓取停位 [m]（沿 −axis 后撤）
geometry_msgs/Point cut_pose    # 剪切参考点 [m]（袋口/框极限）
geometry_msgs/Vector3 axis      # 袋轴单位向量（袋底→袋口）
float64 diameter_m              # 袋径 [m]
float64 d95_m                   # 径向 D95 [m]
float64 travel_m                # 建议插入行程 [m]
float64 cut_travel_m            # 剪切相关行程 [m]
float64 radial_margin_m         # 径向余量 [m]；负值不可套
float64 axial_margin_m          # 轴向余量 [m]；负值不可剪
bool corridor_clear             # 插入走廊是否清空
float64 rmse_m                  # 融合拟合 RMSE [m]
float64 inlier_ratio            # 内点比 0..1
