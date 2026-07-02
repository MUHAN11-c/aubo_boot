# 数字量 IO 通道消息
#
# 用途：表示 AUBO 控制箱上单个数字量通道的开关状态喵~
# 发布者：aubo_driver_ros2 节点的 IO 状态发布线程（~10Hz），从 SDK 回调中读取喵~
# 订阅者：demo_interface / monitoring dashboard，用于可视化 IO 面板和状态监控喵~
#

# 数字量通道引脚编号（0~15），对应控制箱数字量输入/输出物理端口喵~
uint8 pin
# 数字量当前逻辑状态：true = 高电平（ON），false = 低电平（OFF）喵~
bool state
