# 机器人 IO 状态集合消息
#
# 用途：将 AUBO 控制箱各类 IO（数字量输入/输出、标志位、模拟量输入/输出）聚合为单一消息，
#       便于订阅者一次性获取完整的 IO 面板状态喵~
# 发布者：aubo_driver_ros2 节点的 IO 状态发布线程（~10Hz）喵~
# 订阅者：demo_interface / monitoring dashboard 的 IO 面板模块喵~
#

# 数字量输入通道状态数组，数组索引对应物理引脚编号，值表示逻辑电平喵~
Digital[] digital_in_states
# 数字量输出通道状态数组，数组索引对应物理引脚编号，值表示逻辑电平喵~
Digital[] digital_out_states
# 标志位（flag）状态数组，AUBO 控制系统内部状态标志位，与通用数字量 IO 共用物理引脚但语义不同喵~
Digital[] flag_states
# 模拟量输入通道状态数组，包含通道引脚号和当前量程类型（电压/电流）及采样值喵~
Analog[] analog_in_states
# 模拟量输出通道状态数组，包含通道引脚号和当前量程类型（电压/电流）及设定值喵~
Analog[] analog_out_states
