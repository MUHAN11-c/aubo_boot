# 一场扫描结束后的场景引用。执行器 ~/scene_snapshot 闩锁发布，供监控只读镜像。
# SurveyScene.Result 携带同源字段（snapshot_id / scene_epoch / degraded）。

std_msgs/Header header
string snapshot_id           # 本场快照 ID（感知锁定窗 ID 的字符串）
uint32 scene_epoch           # BeginScene 推进的世代；与观测/重建绑定对齐
string scene_key             # 开批时的场景键（RunHarvest.scene_key）
bool degraded                # true=目标集未锁或扫描降级（空集、超时等）
uint32 observation_count     # 本场观测条数
string[] target_ids          # 本场可见/锁定的目标 ID
string message               # 人读说明
