# 跨包稳定数值失败码。事件、账本、GraspDecision / ExecuteTarget.Result 写本常量，
# 不得解析自由文本 reason。本类型也可作为「码+名」载荷携带，但线上多只填 uint32 字段。

uint32 NONE=0                       # 无失败
uint32 OBSERVE_FAILED=1             # 观察失败（视角/掩膜/静止门）
uint32 BUILD_FAILED=2               # BuildTargetModel 失败或超时
uint32 DECISION_REJECTED=3          # GraspDecision 拒绝套入
uint32 PREGRASP_RESIDUAL=4          # 预抓取残差未过
uint32 SLEEVE_PLAN_FAILED=5         # 套入规划失败
uint32 CUT_COMMAND_FAILED=6         # 刀指令发送/受理失败
uint32 CUT_FEEDBACK_TIMEOUT=7       # 切断反馈超时（无硬件反馈时不得 CUT_CONFIRMED）
uint32 RETREAT_FAILED=8             # 撤退失败
uint32 RECOVERY_REQUIRED=9          # 须人工 ACK 才允许下一颗
uint32 VEHICLE_NOT_STATIONARY=10    # 车未静止（导航预留）
uint32 EXACT_TF_MISSING=11          # 缺图像时刻精确 TF（重建禁止 latest）
uint32 DYNAMIC_BUDGET_NEGATIVE=12   # 动态径向/轴向余量为负
uint32 UNBAGGED_NOT_IN_SCOPE=13     # 未套袋目标不在本产品范围
uint32 DEGRADED_CONTACT_FORBIDDEN=14  # 禁止用单帧/降级锚点接触
uint32 MODEL_STALE=15               # 模型与当前 target_id / scene_epoch 不符
uint32 CORRIDOR_BLOCKED=16          # 插入走廊被挡

uint32 code                         # 上列常量之一
string name                         # 常量名（如 BUILD_FAILED）；可空，以 code 为准
