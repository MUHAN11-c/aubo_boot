# TargetPoseResult.metrics / diagnostic_info 的平坦诊断消息；无效标量填 -1
std_msgs/Header header
string target_id
string target_kind
string mask_source
string axis_source
float32 axis_confidence
float32 axis_disagreement_deg
float32 theta_err_deg
float32 error_budget_mm
float32 radial_clearance_mm
float32 valid_depth_ratio
float32 foreground_ratio
float32 boundary_touch_ratio
int32 boundary_sides_touched
int32 n_points
float32 bag_length_m
float32 bag_diameter_upper_m
float32 travel_m
# bag 线圆柱拟合（追加字段）
float32 cylinder_rms_m
float32 cylinder_inlier_ratio
# fruit 线球拟合 / 梗洼
float32 fruit_radius_m
float32 sphere_rms_m
float32 sphere_inlier_ratio
float32 cavity_dip_mm
bool axis_polarity_corrected
uint8 status
uint8 ACCEPT=0
uint8 REOBSERVE=1
uint8 REJECT=2
string[] diagnostic_flags
