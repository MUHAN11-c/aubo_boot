# Topic: /peach/enables (reliable, transient_local, depth 1, republished at 1 Hz).
# Dependency chain enforced by the command gate: tool => grasp => execution.
# Missing heartbeat (> timeout) is treated as all false.

std_msgs/Header header
uint32 seq
bool execution
bool grasp
bool tool
