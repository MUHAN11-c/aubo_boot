# 袋/果 3D 抓取候选。感知初值与重建精化共用本类型。
# 话题（数组）：
#   /peach/perception/initial_pose     生产感知  消费仅重建（单帧初值，不授权运动）
#   /peach/reconstruction/refined_pose 生产重建  消费技能、监控（融合几何）
# 坐标系见 header.frame_id（一般为 base_link）。套入许可仍只看 GraspDecision.allowed。

std_msgs/Header header                    # stamp=图像或融合时刻，frame_id=几何坐标系
string target_id                          # 稳定目标 ID；空=未关联身份
geometry_msgs/Pose entry_pose             # 袋外预入口位姿（工具/尖端，消费方按约定）
geometry_msgs/Point bag_bottom            # 袋底 [m]
geometry_msgs/Point bag_neck              # 袋颈/袋口 [m]
geometry_msgs/Vector3 translation_direction  # 插入方向（bottom→neck 单位向量）
float32 bag_diameter_upper_m              # 袋上部直径 [m]
float32 suggested_travel_m                # 建议插入行程 [m]
float32 confidence                        # 0..1
uint8 status                              # 见下。感知 ACCEPT 只当初值/画面，不发运动
uint8 ACCEPT=0                            # 几何可作初值（感知）或融合可用（重建）
uint8 REOBSERVE=1                         # 几何不够，需再观察
uint8 REJECT=2                            # 拒绝本候选
string[] diagnostic_flags                 # 门控标记（tf_stale 等）
string strategy_id                        # 管线/前景模式标识（bag/fruit 实现名）
string model_version                      # 检测/分割模型版本
string calibration_version                # 相机内外参版本
string tool_version                       # 工具几何版本
float64[9] position_covariance            # 位置 3×3 协方差，行优先；未知填 0
float64[9] direction_covariance           # 方向 3×3 协方差，行优先；未知填 0
uint8 match_status                        # MatchStatus.status（身份匹配）
