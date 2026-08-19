uint8 MODE_AUTO=0
uint8 MODE_PAUSED=1
uint8 MODE_MAINTENANCE=2

uint8 WAITING_READY=0
uint8 DISCOVERY=1
uint8 RUNNING=2
uint8 PAUSE_PENDING=3
uint8 PAUSED=4
uint8 MAINTENANCE=5
uint8 COMPLETED=6
uint8 RECOVERY_REQUIRED=7
uint8 INTERRUPTED=8

uint8 TARGET_IDLE=0
uint8 SELECTING=1
uint8 OBSERVING=2
uint8 FINALIZING=3
uint8 VALIDATING=4
uint8 APPROACHING=5
uint8 TOOL_ACTION=6
uint8 RETREATING=7
uint8 COMPLETING=8
uint8 TARGET_SUCCEEDED=9
uint8 TARGET_SKIPPED=10
uint8 TARGET_FAILED=11

std_msgs/Header header
uint64 revision
string run_id
string cycle_id
string target_id
uint8 operation_mode
uint8 batch_state
uint8 target_phase
bool action_active
bool auto_start_enabled
bool execution_enabled
bool grasp_enabled
bool tool_enabled
bool recovery_required
float32 progress
string message
string[] blockers
# 当前状态下允许的控制命令列表（取值为 ControlHarvest.srv 的命令常量），
# 由编排器按状态机允许表逐状态填充；只读消费方据此使能/禁用控制入口。
uint8[] permissions
