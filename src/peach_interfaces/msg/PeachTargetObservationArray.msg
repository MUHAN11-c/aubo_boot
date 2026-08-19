# 全局发现锁定后的固定目标集合；每帧更新各 ID 的观测状态
std_msgs/Header header
uint32 snapshot_id
string harvest_run_id
bool target_set_locked
uint32 target_count
string selected_target_id
# 发现进度摘要（缺陷 R-D8）：锁定前 observations 恒空，下游看不到收齐进度，
# 故每帧发布本两字段供进度跟踪（锁定前 observations 仍为空数组，只发摘要）。
# collecting_count：锁定前=收齐窗口内累积的已确认目标数（随窗口攒帧单调增长）；
# 锁定后=锁定集大小（与 target_count 一致），窗口已关闭不再变化。
int32 collecting_count
# pending_count：锁定前=确认中（注册表内未转正）记录数；锁定后恒 0
# （身份记忆禁用时每帧记录即确认，无攒帧过程，恒 0）。
int32 pending_count
PeachTargetObservation[] observations
