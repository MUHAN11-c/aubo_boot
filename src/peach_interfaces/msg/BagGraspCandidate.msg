# 视觉→机械臂的冻结抓取候选（源 peach_canopy interfaces）。
# 感知 /peach/perception/initial_pose 与重建 /peach/reconstruction/refined_pose
# 都发本类型数组（transient_local）。坐标系见 header.frame_id（一般为 base_link）。

std_msgs/Header header
string target_id                          # 稳定目标 ID；空=未关联身份
geometry_msgs/Pose entry_pose             # 袋外预入口位姿（工具/尖端，消费方按约定）
geometry_msgs/Point bag_bottom            # 袋底 [m]
geometry_msgs/Point bag_neck              # 袋颈 [m]
geometry_msgs/Vector3 translation_direction  # 插入方向（bottom→neck 单位向量）
float32 bag_diameter_upper_m              # 袋上部直径 [m]
float32 suggested_travel_m                # 建议插入行程 [m]
float32 confidence                        # 0..1
uint8 status                              # ACCEPT 才允许当抓取几何用
uint8 ACCEPT=0
uint8 REOBSERVE=1                         # 几何不够，需再观察
uint8 REJECT=2
string[] diagnostic_flags                 # 门控标记（tf_stale 等）
string strategy_id                        # 管线/前景模式标识
string model_version                      # 检测/分割模型版本
string calibration_version                # 相机内外参版本
string tool_version                       # 工具几何版本

float64[9] position_covariance            # 位置 3×3 协方差，行优先；未知填 0
float64[9] direction_covariance           # 方向 3×3 协方差，行优先；未知填 0
uint8 match_status                        # MatchStatus.status（身份匹配）
