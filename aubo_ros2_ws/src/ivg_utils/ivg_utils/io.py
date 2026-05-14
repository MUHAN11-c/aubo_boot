"""Aubo E5 IO 引脚定义"""

# 夹爪控制 IO 引脚
IO_GRIPPER = 6

# 快换盘 IO 引脚
IO_QUICK_SWAP = 7

# Aubo 底层 IO 服务名
IO_AUBO_SET_SERVICE = "/set_robot_io"

# 逻辑语义常量（统一 IO 语义，消除 true=打开/true=闭合 的歧义）
# ExecuteGraspPoseWorker:  true=打开, false=闭合
# PublishGraspsClientWorker: true=闭合, false=打开
# 本常量采用 ExecuteGraspPoseWorker 语义（true=打开）
GRIPPER_OPEN = True   # 夹爪打开
GRIPPER_CLOSE = False  # 夹爪闭合
