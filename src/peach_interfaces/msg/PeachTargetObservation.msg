# 单目标连续观测：全局目标计划中的稳定 ID + 当前帧结果/掩膜。
# 由 peach_perception 放入 PeachTargetObservationArray；能力包按 target_id 对齐。
std_msgs/Header header         # stamp=图像时刻，frame_id=相机或 base（与候选一致）
string target_id               # 稳定身份 ID；未匹配成功时仍可能有临时 ID
uint32 priority                # 采摘优先级，越小越先；由 harvest_plan 填
bool confirmed                 # 已转正（过身份记忆/确认窗）
bool selected                  # 是否为当前作业目标（须与 HarvestState.target_id 调和）
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

float32 camera_distance_m      # 相机到目标距离 [m]；无效时 0 或配合 diagnostic_flags
float32 confidence             # 检测/分割综合置信度 0..1
BagGraspCandidate candidate    # 3D 抓取候选（header 坐标系）
BagGrasp2D candidate_2d        # 同帧 2D 参考，按 target_id 对齐
BagFitting fitting             # 拟合诊断；无效标量 -1
sensor_msgs/Image mask         # SAM/分割掩膜；空图=本帧无掩膜
string[] diagnostic_flags      # 如 tf_stale、mask_unavailable
