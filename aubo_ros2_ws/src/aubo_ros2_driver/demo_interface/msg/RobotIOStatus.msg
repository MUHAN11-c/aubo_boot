# 机器人整体IO状态消息（含控制箱IO + 工具端IO）
std_msgs/Header header             # 标准消息头（时间戳 + 坐标系帧ID）
bool[] digital_inputs              # 控制箱数字量输入
bool[] digital_outputs             # 控制箱数字量输出
float32[] analog_inputs            # 控制箱模拟量输入
float32[] analog_outputs           # 控制箱模拟量输出
ToolIOStatus tool_io_status        # 工具端IO状态
bool is_connected                  # 与控制箱的连接状态（true=已连接）
