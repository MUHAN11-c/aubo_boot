# 预抓取停位验证。重建在近场停稳后发布；技能 VerifyPregrasp 消费。
# 任何路径均不授权 SetIO。

std_msgs/Header header
string target_id
string model_revision
string tool_profile_id
bool frames_consistent          # 两帧 exact-TF 观测一致
uint8 correction_count          # 已做短修正次数
float32 axis_angle_deg          # 工具轴 vs 袋轴
float32 lateral_error_m         # 袋底/切割面侧向残差
float32 axial_error_m           # 切割面轴向残差
float32 radial_margin_m         # 剩余径向余量；负值不可套
float32 axial_margin_m          # 剩余轴向余量；负值不可剪
bool needs_correction
bool passed
uint32 failure_code             # FailureCode.*
string reason
