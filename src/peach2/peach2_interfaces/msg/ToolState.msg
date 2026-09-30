# Topic: /peach/end_effector/tool_state (reliable, transient_local, depth 1).

uint8 UNKNOWN=0
uint8 OPEN_CONFIRMED=1
uint8 CLOSING=2
uint8 CLOSED_CONFIRMED=3
uint8 OPENING=4
uint8 FAULT=5

uint8 FEEDBACK_UNKNOWN=0
uint8 FEEDBACK_OPEN=1
uint8 FEEDBACK_CLOSED=2

std_msgs/Header header
string tool_id
uint8 state
bool command_closed          # last commanded output level
uint8 feedback               # independent feedback input, FEEDBACK_*
bool suspected_loopback      # feedback followed the command with ~zero delay (self read-back)
float32 actuator_current_a   # [A]; NaN if not measured
string fault_reason
