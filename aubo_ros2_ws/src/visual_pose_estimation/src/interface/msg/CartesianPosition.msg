# 笛卡尔位置消息
# 包含机器人末端执行器的位置、姿态和关节角度

# 位置（米）
geometry_msgs/Point position

# 姿态四元数
geometry_msgs/Quaternion orientation

# 欧拉角（弧度）[roll, pitch, yaw]
float64[3] euler_orientation_rpy_rad

# 欧拉角（度）[roll, pitch, yaw]
float64[3] euler_orientation_rpy_deg

# 关节角度（弧度）
float64[6] joint_position_rad

# 关节角度（度）
float64[6] joint_position_deg
