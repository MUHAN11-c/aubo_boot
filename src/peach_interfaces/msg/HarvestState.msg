# 批次唯一状态快照。peach_task_executor 的 ~/state 发布（transient_local, depth 1）。
# batch_state / target_phase 只由 harvest_fsm.react 推导，禁止节点手写枚举。
# 作业目标只认本消息的 target_id（感知 SAM 焦点、重建绑定、观测 selected）。

# operation_mode
uint8 MODE_AUTO=0         # 自动跑批
uint8 MODE_PAUSED=1       # 人工暂停（含 PAUSE_PENDING）
uint8 MODE_MAINTENANCE=2  # 维护态，不选下一颗

# batch_state：批次级状态机
uint8 WAITING_READY=0     # 等待显式 RunHarvest（launch 不自动开批）
uint8 DISCOVERY=1         # BeginScene + SurveyScene，尚未派目标
uint8 RUNNING=2           # 正在观察/建模/FULL 接触
uint8 PAUSE_PENDING=3     # 接触段收到 PAUSE，动作结束后再停
uint8 PAUSED=4            # 已暂停，等待 RESUME
uint8 MAINTENANCE=5       # 维护态
uint8 COMPLETED=6         # 正常结算
uint8 RECOVERY_REQUIRED=7 # 接触恢复未 ACK，不选下一颗
uint8 INTERRUPTED=8       # 取消或 BeginScene 失败等中断
uint8 NAVIGATING=9        # 预留：到位作业点；navigation_enabled=false 时不进入

# target_phase：当前目标阶段（无目标时为 TARGET_IDLE）
uint8 TARGET_IDLE=0       # 无当前目标
uint8 SELECTING=1         # 正在选下一颗
uint8 OBSERVING=2         # ExecuteTarget OBSERVE_ONLY / 补视角
uint8 FINALIZING=3        # BuildTargetModel finalize
uint8 VALIDATING=4        # 质量门
uint8 APPROACHING=5       # 接触接近
uint8 TOOL_ACTION=6       # 工具 IO / 抓取
uint8 RETREATING=7        # 撤离
uint8 COMPLETING=8        # 卸果/收尾
uint8 TARGET_SUCCEEDED=9
uint8 TARGET_SKIPPED=10
uint8 TARGET_FAILED=11

std_msgs/Header header    # stamp=发布时刻，frame_id 一般为 base_link
uint64 revision           # 状态版本；记录器按此去重写入 state.jsonl（当前与 state_seq 同值）
uint64 state_seq          # 乐观并发序号；ControlTask.expected_state_seq 须匹配（0=不检查）
string run_id             # 本批 run；空=尚未开批
string cycle_id           # 当前单目标周期；RUNNING 时为 run_id:target_id
string target_id          # 当前作业目标；空=未选中。能力包只认此字段
uint8 operation_mode      # MODE_*
uint8 batch_state         # WAITING_READY..NAVIGATING
uint8 target_phase        # TARGET_IDLE..TARGET_FAILED
bool action_active        # 执行器侧有进行中的技能/重建动作
bool auto_start_enabled   # 是否允许自动开批；当前执行器恒填 false
bool execution_enabled    # 参数 execution_enabled；false 时只扫不派目标
bool grasp_enabled        # 是否允许接触规划
bool tool_enabled         # 是否允许工具 SetIO；默认 false
bool recovery_required    # 接触恢复待 ACK
float32 progress          # 已尝试 / max(发现数, 已尝试, 1)，约 0..1
string message            # 批次态名或当前周期说明
string[] blockers         # 阻塞原因列表（未就绪、lifecycle 等）
# 当前状态下允许的控制命令列表（取值为 ControlTask.srv 的命令常量），
# 由编排器按状态机允许表逐状态填充；只读消费方据此使能/禁用控制入口。
uint8[] permissions
bool navigation_enabled   # 参数；false=固定座，不调用 NavigateToWorksite
