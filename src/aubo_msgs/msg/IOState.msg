# 板载 / 工具 / 安全 IO 汇总。aubo_io_controller 的 ~/io_states 每控制周期发布。
# 手眼标定 Web 等只读订阅；写 IO 走 SetIO，不要根据本消息回写。

string stamp  # 发布时刻的纳秒时间戳（十进制字符串）。ROS 1 遗留，不是 std_msgs/Header

Digital[] digital_in_states   # 板载用户 DI，长度 16，pin=0..15，flag 恒 true
Digital[] digital_out_states  # 板载用户 DO，长度 16。无 DO 状态接口：flag=已命令过，state=命令回显
Analog[] analog_in_states     # 板载用户 AI，长度 4，pin=0..3
Analog[] analog_out_states    # 板载用户 AO，长度 4。未命令过时 state 回显 0
Digital[] tool_io_states      # 工具数字输入（不是输出），长度 2，pin=0..1
Analog[] tool_ai_states       # 工具模拟输入，长度 2，pin=0..1
Digital[] safety_in_states    # 长度 2：pin 0=急停，pin 1=防护停止；true=该安全信号有效
Digital[] safety_out_states   # 当前 aubo_io 无安全输出接口，发布时保持空数组
