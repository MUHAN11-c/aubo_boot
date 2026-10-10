# 袋/果拟合诊断（平坦字段）。无效标量一律填 -1；消费方把 <0 视为无数据。
# 话题（数组）：
#   /peach/perception/diagnostics          生产感知  消费重建
#   /peach/reconstruction/refined_diagnostics 生产重建  消费技能、监控
# status=ACCEPT 只建议使用对应几何，不授权运动（套入看 GraspDecision.allowed）。

std_msgs/Header header                 # stamp=图像或融合时刻
string target_id                       # 稳定目标 ID；空=未关联
string target_kind                     # bag / fruit 等
string mask_source                     # 掩膜来源（sam / det / …）
string axis_source                     # 轴向估计来源
float32 axis_confidence                # 轴向置信度 0..1；无效 -1
float32 axis_disagreement_deg          # 多源轴向夹角 [deg]；无效 -1
float32 theta_err_deg                  # 轴向相对期望偏差 [deg]；无效 -1
float32 error_budget_mm                # 剩余误差预算 [mm]；无效 -1
float32 radial_clearance_mm            # 径向净空 [mm]；无效 -1
float32 valid_depth_ratio              # 掩膜内有效深度占比 0..1；无效 -1
float32 foreground_ratio               # 前景占比；无效 -1
float32 boundary_touch_ratio           # 掩膜贴图像边比例；无效 -1
int32 boundary_sides_touched           # 触及的图像边数 0..4；无效 -1
int32 n_points                         # 参与拟合点数；无效 -1
float32 bag_length_m                   # 袋长 [m]；无效 -1
float32 bag_diameter_upper_m           # 袋上部直径 [m]；无效 -1
float32 travel_m                       # 建议行程 [m]；无效 -1
float32 cylinder_rms_m                 # 袋线圆柱拟合 RMS [m]；无效 -1
float32 cylinder_inlier_ratio          # 圆柱内点比 0..1；无效 -1
float32 fruit_radius_m                 # 果半径 [m]；无效 -1
float32 sphere_rms_m                   # 球拟合 RMS [m]；无效 -1
float32 sphere_inlier_ratio            # 球内点比 0..1；无效 -1
float32 cavity_dip_mm                  # 梗洼深度 [mm]；无效 -1
bool axis_polarity_corrected           # 是否校正过轴向极性（bottom↔neck）
uint8 status                           # ACCEPT 才建议使用对应几何
uint8 ACCEPT=0                         # 建议使用对应几何（仍不发运动）
uint8 REOBSERVE=1                      # 几何不够，需再观察
uint8 REJECT=2                         # 拒绝
string[] diagnostic_flags              # 门控标记
