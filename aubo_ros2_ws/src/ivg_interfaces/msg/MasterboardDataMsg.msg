# 主控板数据消息
#
# 用途：从 AUBO SDK 辅助数据流（~10Hz）中获取的主控板级状态，包含控制箱的
#       数字量/模拟量 IO 值、温度、电压、电流和安全状态等信息喵~
# 发布者：aubo_driver_ros2 节点的状态采集线程，每次 SDK 辅助数据回调触发一次发布喵~
# 订阅者：demo_interface（用于前端硬件监控面板）、monitoring dashboard（故障预警）喵~
# 参考：本消息参考 UR MasterboardData 协议设计（UR 技术文章 #16496），AUBO 兼容部分字段喵~
#

# 数字量输入位掩码，bit i 对应第 i 个数字量输入端口的逻辑电平（0=低, 1=高），
# 与 Digital.msg 互为冗余表示，位掩码适用于批量读写场景喵~
uint32 digital_input_bits
# 数字量输出位掩码，bit i 对应第 i 个数字量输出端口的逻辑电平喵~
uint32 digital_output_bits
# 模拟量输入通道 0 的量程类型：0=电流(4-20mA)，1=电压(0-10V)，配置异常时为 -1喵~
int8 analog_input_range0
# 模拟量输入通道 1 的量程类型，同 analog_input_range0 语义喵~
int8 analog_input_range1
# 模拟量输入通道 0 的当前值，CURRENT 模式下单位 mA，VOLTAGE 模式下单位 V喵~
float64 analog_input0
# 模拟量输入通道 1 的当前值，同 analog_input0 语义喵~
float64 analog_input1
# 模拟量输出通道 0 的输出域类型：0=电流输出，1=电压输出喵~
int8 analog_output_domain0
# 模拟量输出通道 1 的输出域类型，同 analog_output_domain0 语义喵~
int8 analog_output_domain1
# 模拟量输出通道 0 的当前设定值，CURRENT 模式下单位 mA，VOLTAGE 模式下单位 V喵~
float64 analog_output0
# 模拟量输出通道 1 的当前设定值，同 analog_output0 语义喵~
float64 analog_output1
# 主控板温度（℃），用于判断控制箱散热状态，异常高温需报警喵~
float32 masterboard_temperature
# 机器人 48V 驱动电源实测电压（V），用于判断供电是否正常（正常范围 ~48V ±5%）喵~
float32 robot_voltage_48v
# 机器人总电流（A），所有关节电机电流之和 + 控制箱自身功耗，用于过流保护判断喵~
float32 robot_current
# 主控板 IO 供电电流（A），为数字量/模拟量 IO 电路提供的电流喵~
float32 master_io_current
# 主控板安全状态码：0=正常，非零值对应具体安全错误类型，需查阅 AUBO SDK 文档喵~
uint8 master_safety_state
# 主控板开关状态码：0=关闭，1=运行中，2=待机，具体枚举值依赖 SDK 版本喵~
uint8 master_onoff_state
