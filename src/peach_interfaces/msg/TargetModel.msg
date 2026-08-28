# BuildTargetModel 终局模型。动作 Result.model 携带；shape 来自重建 ShapeHypothesis。
# 袋底/袋颈/切割面与 D95 是接触几何；球先验只作果体禁切包络。

std_msgs/Header header
string target_id
uint32 scene_epoch
string model_revision
string tool_profile_id
string algorithm_version
string calibration_version
peach_interfaces/ShapeHypothesis shape
peach_interfaces/TargetQuality quality
uint32 view_count
bool accepted
string message

geometry_msgs/Point bag_bottom
geometry_msgs/Point bag_neck
geometry_msgs/Point cut_plane_point
geometry_msgs/Vector3 bag_axis
geometry_msgs/Vector3 cut_normal
float32 d95_m
float32 fruit_prior_radius_m
bool fruit_prior_auxiliary
float64[9] bottom_covariance
float64[9] neck_covariance
float64[9] axis_covariance
float32 radial_margin_m
float32 axial_margin_m
bool corridor_clear
string occlusion_class
builtin_interfaces/Time source_start
builtin_interfaces/Time source_end
