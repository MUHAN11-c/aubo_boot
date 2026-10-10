# 单颗当前帧观测：稳定 target_id + 本帧几何/掩膜/跟踪态。
# 载体：PeachTargetObservationArray.observations（话题 /peach/perception/target_observations）
# 能力包按 target_id 对齐。调度选果看数组级锁定与本条 confirmed/tracking，不单独凭 candidate 发运动。

std_msgs/Header header         # stamp=图像时刻，frame_id=相机或 base（与 candidate 一致）
string target_id               # 稳定身份 ID；未匹配成功时仍可能有临时 ID
uint32 priority                # 采摘优先级，越小越先；由 harvest_plan 填
bool confirmed                 # 已转正（过身份记忆/确认窗 confirm_frames）
bool selected                  # 是否为当前作业目标（须与 HarvestState.target_id 调和）
string harvest_status          # PLANNED / WAITING_QUALITY / SELECTED / HARVESTED

uint8 tracking_status          # 本帧跟踪态（见下）；分类在 scene_perception/assignment.classify_tracking_status
uint8 OBSERVED=0               # 检测+掩膜均在
uint8 OCCLUDED=1               # 检测框在但掩膜缺失（被遮挡/SAM 未产出）
uint8 LOST=2                   # 检测消失（非出视野、非深度空洞的其余情形）
uint8 INVALID=3                # 无效（质量拒绝，不入库）
uint8 OUT_OF_VIEW=4            # 检测框触图像边缘后消失（走出视野，复扫无益）
uint8 DEPTH_VOID=5             # 掩膜在但掩膜内有效深度占比低于阈值

float32 camera_distance_m      # 相机到目标距离 [m]；无效时 0 或配合 diagnostic_flags
float32 confidence             # 检测/分割综合置信度 0..1
BagGraspCandidate candidate    # 本帧 3D 袋/果候选（header 坐标系）；ACCEPT 只当初值
BagGrasp2D candidate_2d        # 同帧 2D 像素参考，按 target_id 对齐
BagFitting fitting             # 拟合诊断；无效标量填 -1
sensor_msgs/Image mask         # SAM/分割掩膜；空图=本帧无掩膜
string[] diagnostic_flags      # 如 tf_stale、mask_unavailable、target_swinging
