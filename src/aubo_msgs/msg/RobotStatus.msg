# 机器人运行/安全摘要。aubo_io_controller 的 ~/robot_status 每周期发布。
# 技能节点据此判断碰撞、急停、是否允许发轨。字段语义简化自 industrial_msgs。
# 除 mode 外，0=否，1=是。mode：-1=未知（未上电），1=已上电但急停，2=已上电无急停。
# 旧版 SDK 不区分示教/回放，mode 不做 teach/play 细分。

int8 mode             # 运行模式（见上）
int8 e_stopped        # 急停有效（安全回路 / 软急停）
int8 drives_powered   # 48V 臂电已上
int8 motion_possible  # 已上电、无急停/防护停、且当前无轨迹在流（可接新轨）
int8 in_motion        # 正在执行轨迹
int8 in_error         # 防护停、碰撞或任一关节错误码非 0
int32 error_code      # 第一个非零关节错误码；0=无错误
