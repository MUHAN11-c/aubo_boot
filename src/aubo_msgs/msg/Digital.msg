# 数字量通道快照，嵌在 IOState 的 digital_* / tool_io / safety_* 数组里。
# 由 aubo_io_controller 每周期发布；当前栈不单独发本类型。

uint8 pin  # 通道号：板载 DI/DO 为 0..15，工具 DI 为 0..1；安全输入见 IOState
bool flag  # 读数是否有效。输入侧恒 true；板载 DO 在尚未 SetIO 命令过时为 false
bool state # 电平：true=高/接通，false=低/断开。DO 无状态接口，回显最近命令值
