"""Aubo E5 常量：IO 引脚与机器人运动参数."""

# 夹爪控制 IO 引脚
IO_GRIPPER = 6

# 快换盘 IO 引脚
IO_QUICK_SWAP = 7

# Aubo 底层 IO 服务名
IO_AUBO_SET_SERVICE = '/set_robot_io'

# 逻辑语义常量（统一 IO 语义，消除 true=打开/true=闭合 的歧义）
# ExecuteGraspPoseWorker:  true=打开, false=闭合
# PublishGraspsClientWorker: true=闭合, false=打开
# 本常量采用 ExecuteGraspPoseWorker 语义（true=打开）
GRIPPER_OPEN = True   # 夹爪打开
GRIPPER_CLOSE = False  # 夹爪闭合

# 机械臂 Home 位关节角 (rad)
HOME_JOINTS_RAD = (-0.026576, -0.553842, 1.999915,
                   -3.019336, -1.572099, 0.000308)

# 笛卡尔路径插值步长 (m)
CARTESIAN_EEF_STEP = 0.015
