# 2D 抓取参考（对应 contracts.BagGrasp2D；与 3D 按 target_id 对齐）
std_msgs/Header header
string target_id
# detection_bbox: x, y, w, h（像素）
int32 bbox_x
int32 bbox_y
int32 bbox_w
int32 bbox_h
# 像素点用 Point，z 固定 0
geometry_msgs/Point bottom_px
geometry_msgs/Point neck_px
geometry_msgs/Point grasp_px
geometry_msgs/Point travel_end_px
bool has_bottom_px
bool has_neck_px
bool has_grasp_px
bool has_travel_end_px
float32 confidence
uint8 status
uint8 ACCEPT=0
uint8 REOBSERVE=1
uint8 REJECT=2
string[] diagnostic_flags
