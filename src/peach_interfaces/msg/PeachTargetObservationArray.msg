# 场景里有哪些桃。调度选果的唯一观测入口（须 scene_epoch 对齐且 target_set_locked）。
# 话题：/peach/perception/target_observations（reliable, volatile, depth 10）
# 生产：peach_scene_perception
# 消费：peach_executor（选果）、重建（对齐掩膜）、技能（新鲜度）、监控
# 锁定前 observations 恒空，只发 collecting_count / pending_count 进度摘要。

std_msgs/Header header            # stamp=本帧图像时刻
uint32 snapshot_id                # 发现窗/锁定集世代；锁定后不变
uint32 scene_epoch                # BeginScene 世代；0=尚未 Begin。SELECT/WAIT_LOCK 须与调度一致
string harvest_run_id             # 所属批次；空=尚未开批
bool target_set_locked            # true=锁定集已关闭，observations 为固定 ID 集合
uint32 target_count               # 锁定后等于锁定集大小；锁定前可为 0
string selected_target_id         # 当前选中目标；空=未选。须与 HarvestState.target_id 调和
int32 collecting_count            # 锁定前=收齐窗内已确认目标数（单调增）；锁定后=锁定集大小
int32 pending_count               # 锁定前=确认中（未转正）记录数；锁定后恒 0
PeachTargetObservation[] observations  # 锁定后各 ID 本帧状态；锁定前为空数组
