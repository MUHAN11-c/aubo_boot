# 单目标连续观测：全局目标计划中的稳定 ID + 当前帧结果/掩膜
std_msgs/Header header
string target_id
uint32 priority
bool confirmed
bool selected
string harvest_status  # PLANNED/WAITING_QUALITY/SELECTED/HARVESTED

uint8 tracking_status
uint8 OBSERVED=0
uint8 OCCLUDED=1     # 检测框在但掩膜缺失（被遮挡/SAM 未产出）
uint8 LOST=2         # 检测消失（其余情形）
uint8 INVALID=3
# 阶段 D1 室外感知追加（纯增量，旧常量值不变；分类逻辑见
# peach_pose/observation_quality.py classify_tracking_status）：
uint8 OUT_OF_VIEW=4  # 检测框触图像边缘后消失（走出视野，复扫无益）
uint8 DEPTH_VOID=5   # 掩膜在但掩膜内有效深度占比低于阈值（深度空洞）

float32 camera_distance_m
float32 confidence
BagGraspCandidate candidate
BagGrasp2D candidate_2d
BagFitting fitting
sensor_msgs/Image mask
string[] diagnostic_flags
