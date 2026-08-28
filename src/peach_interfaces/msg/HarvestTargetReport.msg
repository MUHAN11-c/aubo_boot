# 采摘→外部导航：当前作业目标位置/方向/状态。本仓不发 cmd_vel。
std_msgs/Header header
string target_id
string site_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/Vector3 bag_axis
uint8 status
uint8 IDLE=0
uint8 ASSIGNED=1
uint8 IN_CONTACT=2
uint8 COMPLETED=3
uint8 ABANDONED=4
string message
