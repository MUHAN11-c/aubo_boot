# Robot operating mode (mirrors industrial_msgs/RobotMode semantics, simplified)
int8 mode

# Emergency stop state (safety circuit / soft emergency)
int8 e_stopped

# Drives powered (48V arm power status)
int8 drives_powered

# Motion possible (no active trajectory streaming)
int8 motion_possible

# Robot currently executing a trajectory
int8 in_motion

# In error (protective stop / fault)
int8 in_error

# Controller error code (0 = no error)
int32 error_code
