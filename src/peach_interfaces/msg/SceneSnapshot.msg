# 一场扫描结束后的锁定集快照。
# 话题：/peach_executor/scene_snapshot（reliable, transient_local, depth 1）
# 生产：peach_executor    消费：核内无订阅（清单 consumers 空；监控未订）
# WAIT_LOCK 结束或回访 dwell 后由调度发布。scene_epoch 须为 Begin 之后的值。

std_msgs/Header header           # stamp=快照时刻
string snapshot_id               # 本场快照 ID（感知锁定窗 ID 的字符串）
uint32 scene_epoch               # BeginScene 推进的世代；与观测/重建绑定对齐
string scene_key                 # 开批时的场景键（RunHarvest.scene_key）
bool degraded                    # true=目标集未锁或扫描降级（空集、超时等）
uint32 observation_count         # 本场观测条数
string[] target_ids              # 本场可见/锁定的目标 ID
string message                   # 人读说明
