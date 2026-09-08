# 预抓取停位残差。任何路径均不授权 SetIO。
# 话题：/peach/reconstruction/pregrasp_verification（reliable, transient_local, depth 1）
# 生产：peach_target_reconstruction    消费：核内无订阅（清单 consumers 空）
# 技能 VerifyPregrasp 用工具 TF 自填同类型，经 ExecuteTarget.Result.pregrasp 回传调度。

std_msgs/Header header          # stamp=观测或验证时刻，frame_id=base_frame
string target_id                # 目标 ID
string model_revision           # 对照的模型修订；空=未知
string tool_profile_id          # 工具剖面；空=未指定
bool frames_consistent          # 两帧 exact-TF 观测一致
uint8 correction_count          # 已做短修正次数
float32 axis_angle_deg          # 工具轴相对袋轴夹角 [deg]
float32 lateral_error_m         # 袋底/切割面侧向残差 [m]
float32 axial_error_m           # 切割面轴向残差 [m]
float32 radial_margin_m         # 剩余径向余量 [m]；负值不可套
float32 axial_margin_m          # 剩余轴向余量 [m]；负值不可剪
bool needs_correction           # 建议再做短修正
bool passed                     # 本帧/本次验证过门（技能 Result 口径）
uint32 failure_code             # FailureCode.*；0=NONE
string reason                   # 过门/失败原因
