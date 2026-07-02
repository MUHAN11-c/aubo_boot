# 机器人整体 IO 状态消息（含控制箱 IO + 工具端 IO）
#
# 用途：聚合 AUBO 控制箱的全部 IO（数字量/模拟量输入输出）和工具端 IO 状态，
#       供前端一站式获取所有硬件 IO 信息喵~
# 发布者：demo_interface 节点的 IO 状态发布线程（~10Hz），从多个底层 IO 消息合并而来喵~
# 订阅者：Web Dashboard 前端（IO 面板）、monitoring dashboard、IO 诊断工具喵~
#

# 标准消息头：时间戳标记 IO 状态聚合时刻喵~
std_msgs/Header header
# 控制箱数字量输入状态数组，布尔数组，索引 i 对应物理引脚 i 的电平值（true=高电平）喵~
bool[] digital_inputs
# 控制箱数字量输出状态数组，布尔数组，索引 i 对应物理引脚 i 的电平值（true=高电平）喵~
bool[] digital_outputs
# 控制箱模拟量输入值数组，浮点数，CURRENT 模式单位 mA / VOLTAGE 模式单位 V喵~
float32[] analog_inputs
# 控制箱模拟量输出值数组，浮点数，单位同 analog_inputs喵~
float32[] analog_outputs
# 工具端 IO 状态子消息，包含工具接口的数字量和模拟量 IO 值喵~
ToolIOStatus tool_io_status
# 与控制箱的连接状态（硬件层）：true=控制箱与主控板通讯正常，false=通讯中断喵~
bool is_connected
