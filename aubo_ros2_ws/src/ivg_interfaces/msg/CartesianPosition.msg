# 笛卡尔位置消息
#
# 用途：携带机器人末端执行器（TCP）的完整位姿状态，包含位置、姿态（四元数 + 欧拉角双表示）、
#       以及关节角度，用于视觉推理结果输出、抓取位姿通信和状态记录喵~
# 发布者：
#   - visual_pose_estimation 的 inference 结果发布节点（~5-10Hz），将 GraspNet 预测的
#     抓取位姿封装为此消息发布喵~
#   - demo_interface 的状态聚合节点（~30Hz），将当前机器人 TCP 位姿以此格式发布喵~
# 订阅者：
#   - execute_grasp_pose_worker、grasp_executor（接收视觉推理的抓取目标位姿）
#   - Web Dashboard 前端（用于显示当前 TCP 位置/姿态/关节角）喵~
#

# TCP 位置（米），在基座标系下的三维坐标 [x, y, z]，
# 由机器人正解运动学（FK）从关节角度计算得出喵~
geometry_msgs/Point position

# 姿态四元数 [x, y, z, w]，在基座标系下的 TCP 姿态表示，
# 用于 MoveIt2 规划中的姿态约束和 IK 求解喵~
geometry_msgs/Quaternion orientation

# 欧拉角（ZYX 万向节顺序 / RPY 顺序），单位弧度 [roll, pitch, yaw]，
# 与四元数等价，用于前端显示和角度范围检查喵~
float64[3] euler_orientation_rpy_rad

# 欧拉角（ZYX 万向节顺序 / RPY 顺序），单位度 [roll, pitch, yaw]，
# 从 rad 版本转换而来，便于人类阅读和调试喵~
float64[3] euler_orientation_rpy_deg

# 关节角度数组（弧度），6 维 [joint_1 ~ joint_6]，
# 与 position + orientation 对应（正解一致），
# 用于验证 FK 一致性和关节空间重规划喵~
float64[6] joint_position_rad

# 关节角度数组（度），6 维 [joint_1 ~ joint_6]，
# 从 rad 版本转换而来，用于前端显示和日志记录喵~
float64[6] joint_position_deg
