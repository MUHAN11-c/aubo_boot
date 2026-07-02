# 工具端数据消息
#
# 用途：从 AUBO SDK 辅助数据流中获取的末端工具（Tool I/O 接口）的模拟量输入、电压、
#       电流、温度和工作模式等状态信息喵~
# 发布者：aubo_driver_ros2 节点的状态采集线程，~10Hz 频率喵~
# 订阅者：demo_interface、tool_changer_interface（工具连接状态诊断）喵~
#

# 模拟量输入通道 2 的量程类型：电流模式（0）
int8 ANALOG_INPUT_RANGE_CURRENT = 0
# 模拟量输入通道 2 的量程类型：电压模式（1）
int8 ANALOG_INPUT_RANGE_VOLTAGE = 1

# 模拟量输入通道 2 的量程类型：取值为 ANALOG_INPUT_RANGE_CURRENT 或 ANALOG_INPUT_RANGE_VOLTAGE喵~
int8 analog_input_range2
# 模拟量输入通道 3 的量程类型：同 analog_input_range2 语义喵~
int8 analog_input_range3
# 模拟量输入通道 2 的当前采样值，单位取决于量程配置（mA 或 V）喵~
float64 analog_input2
# 模拟量输入通道 3 的当前采样值，单位取决于量程配置（mA 或 V）喵~
float64 analog_input3
# 工具端 48V 电源实测电压（V），与主控板 robot_voltage_48v 应一致，用于工具供电诊断喵~
float32 tool_voltage_48v
# 工具端输出电压设定值（V），用于供电工具（如相机光源、传感器等）喵~
uint8 tool_output_voltage
# 工具端电流测量值（A），通过工具供电接口流出的总电流，用于过载保护判断喵~
float32 tool_current
# 工具端温度（℃），靠近工具连接器接口的温度传感器读数，用于过热预警喵~
float32 tool_temperature

# 工具模式常量：引导加载程序模式，固件升级状态下激活喵~
uint8 TOOL_BOOTLOADER_MODE = 249
# 工具模式常量：正常运行模式，工具功能完整可用喵~
uint8 TOOL_RUNNING_MODE = 253
# 工具模式常量：空闲模式，工具上电但未执行任何操作喵~
uint8 TOOL_IDLE_MODE = 255

# 工具当前工作模式：取值为 TOOL_BOOTLOADER_MODE / TOOL_RUNNING_MODE / TOOL_IDLE_MODE 之一喵~
uint8 tool_mode
