# 【预留】采摘 → 外部导航：当前作业目标位置/方向/状态。本仓不发 cmd_vel。
# 话题：/peach/navigation/target_report（清单 reserved_interfaces）
# 生产/消费：固定座核心栈均无。随 NavigateToWorksite 一并归档。

std_msgs/Header header                 # stamp=发布时刻
string target_id                       # 当前作业目标
string site_id                         # 作业位点
geometry_msgs/PoseWithCovariance pose  # 目标位姿（世界/地图系，由导航约定）
geometry_msgs/Vector3 bag_axis         # 袋轴（供底盘朝向）
uint8 status                           # 见下
uint8 IDLE=0                           # 无作业目标
uint8 ASSIGNED=1                       # 已派目标
uint8 IN_CONTACT=2                     # 臂接触中
uint8 COMPLETED=3                      # 本颗完成
uint8 ABANDONED=4                      # 放弃
string message                         # 人读说明
