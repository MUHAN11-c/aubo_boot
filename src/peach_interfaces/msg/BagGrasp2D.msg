# 2D 像素抓取参考，与 3D BagGraspCandidate 按 target_id 对齐。
# 载体：PeachTargetObservation.candidate_2d。不单独发话题。不授权运动。

std_msgs/Header header             # stamp=图像时刻，frame_id=相机光学系
string target_id                   # 稳定目标 ID
int32 bbox_x                       # 检测框左上 x [px]
int32 bbox_y                       # 检测框左上 y [px]
int32 bbox_w                       # 检测框宽 [px]
int32 bbox_h                       # 检测框高 [px]
geometry_msgs/Point bottom_px      # 袋底像素；z 固定 0。有效看 has_bottom_px
geometry_msgs/Point neck_px        # 袋颈像素；z 固定 0
geometry_msgs/Point grasp_px       # 抓取参考像素；z 固定 0
geometry_msgs/Point travel_end_px  # 行程终点像素；z 固定 0
bool has_bottom_px                 # bottom_px 有效
bool has_neck_px                   # neck_px 有效
bool has_grasp_px                  # grasp_px 有效
bool has_travel_end_px             # travel_end_px 有效
float32 confidence                 # 0..1
uint8 status                       # ACCEPT 才可作为 2D 参考（仍不发运动）
uint8 ACCEPT=0                         # 可作为 2D 参考
uint8 REOBSERVE=1                      # 需再观察
uint8 REJECT=2                         # 拒绝
string[] diagnostic_flags          # 门控标记
