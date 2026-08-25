# 重建几何假设（不含工具抓取量）。/peach/reconstruction/shape_hypothesis 闩锁发布。
# 契约预留话题已在 manifest 中；监控只读。抓取许可走 GraspDecision，不要单独凭本消息发运动。

std_msgs/Header header             # frame_id=base_frame
string target_id
uint32 scene_epoch
geometry_msgs/Point center         # 袋/果中心 [m]
geometry_msgs/Vector3 axis         # 主轴单位向量（bottom→neck）
float32 diameter_m                 # 直径 [m]
float32 length_m                   # 轴向长度 [m]
float64[9] position_covariance     # 中心 3×3，行优先；未知填 0
float64[9] direction_covariance    # 轴向 3×3，行优先；未知填 0
float32 confidence                 # 0..1，常取内点比
string model_kind                  # 拟合种类（cylinder/sphere/bag 等）；空=未知
