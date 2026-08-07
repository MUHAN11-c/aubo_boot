# RefineFeedback.msg — peach_recon → peach_pose_ros2 的先验反哺
# 融合精化后的轴/位置先验，供感知下一帧轴估计/掩膜 ROI 参考
std_msgs/Header header
string target_id                  # 关联目标（target_N / 注册表 ID）
geometry_msgs/Vector3 axis_prior  # 融合后的套入轴先验（单位向量，输出系）
geometry_msgs/Point bottom_prior  # 融合后的袋底位置先验（输出系，米）
float32 confidence                # 融合置信度 [0,1]
uint32 support_frames             # 支撑该先验的累积帧数
bool valid                        # 先验是否可用（false=忽略本条）
