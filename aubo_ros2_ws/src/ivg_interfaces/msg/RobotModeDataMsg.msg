# 机器人模式数据消息
#
# 用途：从 AUBO SDK 辅助数据流（~10Hz，类比 UR TCP 30002 端口）中获取的机器人整体模式状态，
#       用于安全状态监控和启动流程控制喵~
# 发布者：aubo_driver_ros2 节点的状态采集线程，每次 SDK 辅助数据回调触发一次发布喵~
# 订阅者：demo_interface（用于前端安全状态显示）、tool_changer_interface（工具切换前检查安全条件）、
#         execute_grasp_pose_worker（抓取执行前确认机器人状态）喵~
# 注意：本消息是 UR RobotModeData 的子集，仅携带 IVG 系统关心的关键字段，不包含全部原始数据喵~
#

# SDK 内部时间戳（微秒级），机器人控制器上电后累计，可用于状态变化的时间序列分析喵~
uint64 timestamp
# 与控制箱的 TCP 连接状态：true=已建立连接，false=连接断开喵~
bool is_robot_connected
# 真实机器人使能状态（非仿真）：true=机器人已上使能，false=未使能或处于仿真模式喵~
bool is_real_robot_enabled
# 机器人动力电源状态：true=电机驱动电源已接通（48V上电），false=驱动电源断开喵~
bool is_power_on_robot
# 急停状态：true=急停按钮被按下或急停电路触发，需人工复位才能恢复喵~
bool is_emergency_stopped
# 防护性停止状态：true=安全防护触发（如安全门开关、光栅、碰撞检测等），排除后可自动恢复喵~
bool is_protective_stopped
# 程序运行状态：true=机器人正在执行程序（含动作指令或脚本），false=程序已停止或未启动喵~
bool is_program_running
# 程序暂停状态：true=程序执行被用户暂停（非停止），可通过恢复指令继续执行喵~
bool is_program_paused
