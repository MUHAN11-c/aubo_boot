# 机器人实时状态消息
#
# 用途：从 AUBO SDK 实时数据流（~125Hz，类比 UR TCP 30003 端口）中获取的完整机器人状态，
#       包含关节角度、速度、电流、TCP 力/位姿等信息喵~
# 发布者：aubo_driver_ros2 节点的状态采集线程，每次 SDK 实时数据回调触发一次发布喵~
# 订阅者：robot_state_publisher（用于 TF 发布）、demo_interface（用于前端实时监控）、
#         rosbag 记录（用于离线回放和分析）喵~
# 注意：此消息的字段设计参考 Universal Robots 实时客户端接口协议，AUBO 兼容部分字段喵~
# 参考：http://wiki03.lynero.net/Technical/RealTimeClientInterface?rev=9
#

# SDK 内部计时器值（秒），从机器人控制器启动开始累计，可用于轨迹对齐和时间戳校验喵~
float64 time
# 目标关节位置（弧度），当前控制周期设定的期望关节角度，6 维数组对应 6 个关节喵~
float64[] q_target
# 目标关节速度（弧度/秒），当前控制周期设定的期望关节速度喵~
float64[] qd_target
# 目标关节加速度（弧度/秒²），当前控制周期设定的期望关节加速度喵~
float64[] qdd_target
# 目标关节电流（A），当前控制周期设定的期望关节电机电流喵~
float64[] i_target
# 目标关节力矩（Nm），当前控制周期设定的期望关节输出力矩喵~
float64[] m_target
# 实际关节位置（弧度），编码器反馈的实际关节角度，用于笛卡尔正解计算和 TF 发布喵~
float64[] q_actual
# 实际关节速度（弧度/秒），编码器反馈的实际关节速度喵~
float64[] qd_actual
# 实际关节电流（A），电机驱动器反馈的实际相电流喵~
float64[] i_actual
# 工具端加速度计原始值（m/s²），三轴（X, Y, Z）加速度数据，用于振动检测和安全监控喵~
float64[] tool_acc_values
# TCP 力/力矩传感器值（N/Nm），六维力/力矩（Fx, Fy, Fz, Mx, My, Mz），仅当安装力传感器时有有效值喵~
float64[] tcp_force
# TCP 在基座标系中的位姿向量 [x, y, z, rx, ry, rz]，位置（m）+ 轴角（rad），用于手眼标定验证喵~
float64[] tool_vector
# TCP 在基座标系中的线速度和角速度 [vx, vy, vz, ωx, ωy, ωz]（m/s, rad/s）喵~
float64[] tcp_speed
# 数字量输入位掩码，bit i 对应第 i 个数字量输入端口的逻辑电平，按位与运算后可得特定端口状态喵~
float64 digital_input_bits
# 各关节电机温度数组（℃），6 维，用于电机过热保护监控喵~
float64[] motor_temperatures
# 控制器内部定时器累计值（秒），与 time 字段类似但精度不同，用于冗余校验喵~
float64 controller_timer
# 测试/保留字段，SDK 内部诊断用，生产环境中通常为 0 喵~
float64 test_value
# 机器人模式枚举值：0=断开、1=手动减速模式、2=手动全速模式、3=自动模式，对应 AUBO SDK 定义喵~
float64 robot_mode
# 各关节驱动器模式数组，6 维，枚举值含义依赖具体 SDK 版本，用于诊断关节驱动状态喵~
float64[] joint_modes
