# BuildTargetModel 终局模型（动作 Result.model 携带；无独立话题）。
# shape 来自重建 ShapeHypothesis。袋底/袋颈/切割面与 D95 是接触几何；
# 球先验只作果体禁切包络，不是套入许可（许可走 GraspDecision）。

std_msgs/Header header                 # stamp=finalize 时刻，frame_id=base_frame
string target_id                       # 须与 HarvestState.target_id 一致
uint32 scene_epoch                     # 须与当前 BeginScene 世代一致
string model_revision                  # 本模型修订号
string tool_profile_id                 # 工具剖面
string algorithm_version               # 重建算法版本
string calibration_version             # 相机标定版本
peach_interfaces/ShapeHypothesis shape # 形状假说（中心/轴/径/长）
peach_interfaces/TargetQuality quality # 质量档
uint32 view_count                      # 参与 finalize 的机位数
bool accepted                          # 过门并被接受
string message                         # 拒绝/降级原因；接受可空

geometry_msgs/Point bag_bottom         # 袋底 [m]
geometry_msgs/Point bag_neck           # 袋颈/袋口 [m]
geometry_msgs/Point cut_plane_point    # 切割面参考点 [m]
geometry_msgs/Vector3 bag_axis         # 袋轴（bottom→neck）
geometry_msgs/Vector3 cut_normal       # 切割面法向
float32 d95_m                          # 径向 D95 [m]
float32 fruit_prior_radius_m           # 果体球先验半径 [m]；禁切包络
bool fruit_prior_auxiliary             # true=球先验仅辅助，不是主几何
float64[9] bottom_covariance           # 袋底 3×3，行优先；未知填 0
float64[9] neck_covariance             # 袋颈 3×3，行优先；未知填 0
float64[9] axis_covariance             # 轴向 3×3，行优先；未知填 0
float32 radial_margin_m                # 径向余量 [m]
float32 axial_margin_m                 # 轴向余量 [m]
bool corridor_clear                    # 插入走廊是否清空
string occlusion_class                 # 遮挡档（空=未评）
builtin_interfaces/Time source_start   # 积分源时间范围起点
builtin_interfaces/Time source_end     # 积分源时间范围终点
