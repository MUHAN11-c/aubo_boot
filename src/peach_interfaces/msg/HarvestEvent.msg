uint8 INFO=0
uint8 WARNING=1
uint8 ERROR=2
uint8 AUDIT=3

std_msgs/Header header
uint64 sequence
uint8 severity
string code
string message
string request_id
string run_id
string cycle_id
string target_id
diagnostic_msgs/KeyValue[] details
