# 工具端 IO 状态消息
#
# 用途：承载 AUBO 机械臂末端工具接口（Tool I/O Connector）的 IO 状态，
#       与 RobotIOStatus 配合使用，提供完整的工具级 IO 信息喵~
# 发布者：demo_interface 节点的 IO 状态发布线程（~10Hz），作为 RobotIOStatus 的内部嵌套子消息喵~
# 订阅者：Web Dashboard 前端（工具 IO 面板）、tool_changer_interface、gripper_control 节点喵~
#

# 工具端数字量输入状态数组，布尔数组，索引 i 对应工具接口引脚 i 的电平值喵~
bool[] digital_inputs
# 工具端数字量输出状态数组，布尔数组，索引 i 对应工具接口引脚 i 的电平值喵~
bool[] digital_outputs
# 工具端模拟量输入值数组，浮点数，CURRENT 模式单位 mA / VOLTAGE 模式单位 V，
# 用于读取工具端传感器（如夹爪力传感器等）喵~
float32[] analog_inputs
# 工具端模拟量输出值数组，浮点数，单位同 analog_inputs，用于控制工具端可调参数喵~
float32[] analog_outputs
