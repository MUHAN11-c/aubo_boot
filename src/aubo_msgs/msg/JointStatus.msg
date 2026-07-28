# 关节级详细状态：SDK JointStatus 推送帧的全量透传 + 派生量，
# 由 aubo_io_controller 的 ~/joint_status 话题每周期发布（数组长度 = 关节数 6）。

# 关节电流。SDK 原始单位（jointCurrentI，int 驱动器读数；SDK 未给出到安培
# 的换算系数，硬件插件不做猜测性换算，消费侧自行标定）。
float64[] current

# 关节温度 [°C]
float64[] temperature

# 机器人控制器当前正在执行的目标关节角 [rad]（jointTagPosJ，含 RIB 缓冲相位）
float64[] tag_pos

# 目标关节速度 [rad/s]（电机侧目标速度经减速比换算，与关节速度同一路径）
float64[] tag_vel

# 跟随误差 |tag_pos - 实际关节角| [rad]（io 控制器计算）
float64[] following_error

# 关节错误码（jointErrorNum，0 = 无错误）
uint16[] error_code
