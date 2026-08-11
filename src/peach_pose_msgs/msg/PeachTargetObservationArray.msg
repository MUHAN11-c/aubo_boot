# 全局发现锁定后的固定目标集合；每帧更新各 ID 的观测状态
std_msgs/Header header
uint32 snapshot_id
string harvest_run_id
bool target_set_locked
uint32 target_count
string selected_target_id
PeachTargetObservation[] observations
