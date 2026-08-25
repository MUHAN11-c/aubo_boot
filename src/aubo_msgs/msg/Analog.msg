# 模拟量通道快照，嵌在 IOState 的 analog_in_states / analog_out_states / tool_ai_states 里。
# 由 aubo_io_controller 每周期发布；当前栈不单独发本类型。

uint8 pin     # 通道号：板载 AI/AO 为 0..3，工具 AI 为 0..1
float32 state # 控制器原始读数或命令回显（通常为电压 [V]；未命令过的 AO 回显 0）
