# Topic: /peach/task/state (reliable, transient_local, depth 1).

uint8 IDLE=0
uint8 SURVEYING=1
uint8 SELECTING=2
uint8 OBSERVING=3
uint8 HARVESTING=4
uint8 WAITING_ACK=5
uint8 PAUSED=6
uint8 COMPLETED=7
uint8 ABORTED=8

std_msgs/Header header
string request_id
uint8 phase
string current_target_id
string[] blockers
uint32 attempted
uint32 succeeded
uint32 skipped
uint32 failed
bool recovery_required
string message
