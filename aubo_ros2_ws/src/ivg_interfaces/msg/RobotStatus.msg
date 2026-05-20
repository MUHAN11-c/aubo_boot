# 机械臂状态消息（综合状态）
#
# 用途：将 AUBO 机械臂的在线/使能/运动状态、关节角度、TCP 位姿等信息聚合为单一消息，
#       供前端 Web Dashboard、demo_interface 和监控节点一站式获取机器人完整状态喵~
# 发布者：aubo_state_broadcaster 节点 (~50Hz), 从 SDK 状态查询聚合而来喵~
# 订阅者：Web Dashboard 前端（websocket gateway）、monitoring dashboard、rosbag 记录喵~
#

# 标准消息头：时间戳标记状态聚合时刻，frame_id="base_link" 表示位姿数据所在坐标系喵~
std_msgs/Header header
# 机器人在线状态：true=与控制箱 TCP 已连接且数据流正常，false=离线或连接中断喵~
bool is_online
# 机器人使能状态：true=驱动电源已接通且伺服已使能，可接受运动指令；false=未使能喵~
bool enable
# 机器人运动状态：true=当前有轨迹正在执行中（关节或笛卡尔运动），false=静止且无活动轨迹喵~
bool in_motion
# MoveIt2 规划执行状态字符串：值域为 "idle" / "planning" / "executing" / "error"，
# 用于前端显示当前 MoveIt 正在做什么（规划、执行、或空闲）喵~
string planning_status
# 6 个关节当前角度数组（弧度），顺序为 base 到 tip（joint_1 ~ joint_6），用于数值计算喵~
float64[6] joint_position_rad
# 6 个关节当前角度数组（度），顺序同 joint_position_rad，用于前端可视化显示喵~
float64[6] joint_position_deg
# TCP 在当前规划组基座标系下的完整位姿（位置 m + 四元数姿态），用于笛卡尔路径规划和显示喵~
geometry_msgs/Pose cartesian_position
# TCP 位置分量（x, y, z 单位 m），从 cartesian_position 中提取的位置分量，用于前端快捷引用喵~
geometry_msgs/Point cartesian_position_xyz
# TCP 姿态欧拉角（roll, pitch, yaw 单位 rad），从 cartesian_position 转换而来，用于前端显示喵~
geometry_msgs/Vector3 cartesian_rpy
