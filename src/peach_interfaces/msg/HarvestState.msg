# 批次唯一状态快照。能力包作业目标只认本消息的 target_id
# （感知 SAM 焦点、重建绑定、观测 selected 都须与此调和）。
# 话题：/peach_executor/state（reliable, transient_local, depth 1）
# 生产：peach_executor    消费：感知、重建、监控；调度自订
# batch_state / target_phase 只由 harvest_fsm.react 推导，禁止节点手写。

# operation_mode：人机模式（与 batch_state 正交）
uint8 MODE_AUTO=0         # 自动跑批
uint8 MODE_PAUSED=1       # 人工暂停（含 PAUSE_PENDING）
uint8 MODE_MAINTENANCE=2  # 维护态，不选下一颗

# batch_state：批次级状态机
uint8 WAITING_READY=0     # 等待显式 RunHarvest（launch 不自动开批）
uint8 DISCOVERY=1         # 首巡 Survey→Begin→WAIT_LOCK→SELECT；回访只 Survey。尚未派目标
uint8 RUNNING=2           # 正在观察/建模/接触（含 PREGRASP_ONLY）
uint8 PAUSE_PENDING=3     # 接触段收到 PAUSE，当前动作结束后再停
uint8 PAUSED=4            # 已暂停，等待 RESUME
uint8 MAINTENANCE=5       # 维护态
uint8 COMPLETED=6         # 正常结算
uint8 RECOVERY_REQUIRED=7 # 接触或预抓取停住未 ACK，不选下一颗
uint8 INTERRUPTED=8       # 取消或 SurveyScene / BeginScene 失败等中断
uint8 NAVIGATING=9        # 预留：到位作业点；navigation_enabled=false 时不进入

# target_phase：当前目标阶段（无目标时为 TARGET_IDLE）
uint8 TARGET_IDLE=0       # 无当前目标
uint8 SELECTING=1         # 正在选下一颗
uint8 OBSERVING=2         # ExecuteTarget OBSERVE_ONLY / 补视角
uint8 FINALIZING=3        # BuildTargetModel finalize
uint8 VALIDATING=4        # 质量门
uint8 APPROACHING=5       # 接触接近（预抓取/套入规划）
uint8 TOOL_ACTION=6       # 工具 IO / 剪切
uint8 RETREATING=7        # 撤离
uint8 COMPLETING=8        # 卸果/收尾
uint8 TARGET_SUCCEEDED=9  # 本颗周期 SUCCEEDED（采摘成功另看 HarvestResult.grasped）
uint8 TARGET_SKIPPED=10   # 本颗跳过（质量/不可达/人工 SKIP）
uint8 TARGET_FAILED=11    # 本颗失败

std_msgs/Header header    # stamp=发布时刻，frame_id 一般为 base_link
uint64 revision           # 状态版本；记录器按此去重写入 state.jsonl（现行与 state_seq 同值）
uint64 state_seq          # 乐观并发序号；ControlTask.expected_state_seq 须匹配（0=不检查）
string run_id             # 本批 run；空=尚未开批
string cycle_id           # 当前单目标周期；RUNNING 时为 run_id:target_id
string target_id          # 当前作业目标；空=未选中。能力包只认此字段
uint8 operation_mode      # MODE_*
uint8 batch_state         # WAITING_READY..NAVIGATING
uint8 target_phase        # TARGET_IDLE..TARGET_FAILED
bool action_active        # 调度侧有进行中的技能/重建动作
bool auto_start_enabled   # 是否允许自动开批；现行执行器恒填 false
bool execution_enabled    # 参数 execution_enabled；false 时只扫不派目标
bool grasp_enabled        # 是否允许接触规划（预抓取/套入）
bool tool_enabled         # 是否允许工具 SetIO；默认 false
bool recovery_required    # 接触或 PREGRASP_ONLY 停住，待 ACKNOWLEDGE_RECOVERY
float32 progress          # 已尝试 / max(发现数, 已尝试, 1)，约 0..1
string message            # 批次态名或当前周期说明
string[] blockers         # 阻塞原因列表（预留，现行恒空；无写入方）
uint8[] permissions       # 当前允许的 ControlTask.command；编排器按 FSM 允许表填充
bool navigation_enabled   # 预留：固定座现行恒 false（调度到位直通 NAV_OK）
