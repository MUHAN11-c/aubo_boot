# 机械臂状态消息
std_msgs/Header header             # 标准消息头（时间戳 + 坐标系帧ID）
bool is_online                     # 机器人在线状态（true=在线，false=离线）
bool enable                        # 机器人使能状态（true=已使能，false=未使能）
bool in_motion                     # 机器人运动状态（true=运动中，false=静止）
string planning_status             # MoveIt2 规划状态（idle=空闲 / planning=规划中 / executing=执行中 / error=错误）
float64[6] joint_position_rad      # 6关节角度（弧度）
float64[6] joint_position_deg      # 6关节角度（度）
geometry_msgs/Pose cartesian_position      # TCP 位姿（位置m + 四元数姿态）
geometry_msgs/Point cartesian_position_xyz # TCP 位置（x, y, z 单位m）
geometry_msgs/Vector3 cartesian_rpy        # TCP 姿态（欧拉角 roll, pitch, yaw 单位rad）
