# 模拟量 IO 通道消息
#
# 用途：表示 AUBO 控制箱上单个模拟量通道的当前值，包含通道编号和量程类型喵~
# 发布者：aubo_driver_ros2 节点的 IO 状态发布线程（~10Hz），从 SDK 回调中读取喵~
# 订阅者：demo_interface / monitoring dashboard，用于可视化 IO 面板和状态监控喵~
#

# 模拟量量程类型常量：电流模式（4-20mA）
uint8 CURRENT=0
# 模拟量量程类型常量：电压模式（0-10V）
uint8 VOLTAGE=1

# 模拟量通道引脚编号（0~3），对应控制箱模拟量输入/输出物理端口喵~
uint8 pin
# 模拟量量程类型：CURRENT（0）或 VOLTAGE（1），由物理接线和主控板配置决定喵~
uint8 domain
# 模拟量当前值：CURRENT 模式下单位为 mA，VOLTAGE 模式下单位为 V，浮点数精度为单次采样值喵~
float32 state
