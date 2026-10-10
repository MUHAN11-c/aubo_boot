# 末端工具三轴状态（批次3，2026-09-23 重构；FINAL_PLAN §7.2/§12）。
# 三轴独立观测、各自 UNKNOWN 优先；刀闭合 ≠ 果柄分离（L86 红线）。
# 发布：/peach_arm/tool_state（peach_arm，on-change + 1Hz 心跳，latched）。
std_msgs/Header header

# 刀轴：BLADE_CLOSED 只证明机构闭合。
uint8 BLADE_UNKNOWN=0
uint8 BLADE_OPEN=1
uint8 BLADE_CLOSING=2
uint8 BLADE_CLOSED=3
uint8 BLADE_FAULT=4
uint8 blade_state

# 保持轴（承接/夹持；空心筒被动承接=HOLDING 由工艺档案声明）。
uint8 RETENTION_UNKNOWN=0
uint8 RETENTION_READY=1
uint8 RETENTION_ENGAGING=2
uint8 RETENTION_HOLDING=3
uint8 RETENTION_RELEASING=4
uint8 RETENTION_FAULT=5
uint8 retention_state

# 载荷轴：剪后承接证据（P0 无传感器=UNKNOWN，M0 后接光电/力/重量）。
uint8 PAYLOAD_UNKNOWN=0
uint8 PAYLOAD_ABSENT=1
uint8 PAYLOAD_PRESENT=2
uint8 PAYLOAD_SECURED=3
uint8 PAYLOAD_LOST=4
uint8 payload_state

builtin_interfaces/Time last_edge_stamp  # 最近一次状态沿（DI/命令回执）
string tool_serial                        # 工具档案序列（空=未上报）
string evidence_source                    # 本帧证据来源摘要（di:pin0-rise 等）
string fault_code                         # 非空=FAULT 态原因
