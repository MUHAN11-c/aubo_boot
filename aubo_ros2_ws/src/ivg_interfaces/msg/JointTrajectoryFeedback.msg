# 自定义关节轨迹反馈消息
#
# 用途：提供关节轨迹跟踪的实时反馈，包含期望值、实际值和误差值，
#       与 control_msgs/FollowJointTrajectoryFeedback 格式一致，用于 MoveIt2 和
#       下游监控节点的轨迹跟踪状态评估喵~
# 发布者：aubo_driver_ros2 的 JointTrajectoryController Action Server（~50Hz），
#         Action 执行的 feedback 通道中发送喵~
# 订阅者：MoveIt2 / move_group 的 FollowJointTrajectory Action Client，
#         用于轨迹执行状态监控和误差超限处理喵~
#

# 标准消息头：时间戳标记反馈生成时刻，frame_id 通常设为空字符串或规划组名称喵~
std_msgs/Header header
# 关节名称列表，与 desired/actual/error 数组中的元素一一对应，排序为 base 到 tip 的关节顺序喵~
string[] joint_names
# 期望轨迹点：包含期望的位置（弧度）、速度（弧度/秒）、加速度（弧度/秒²）和 time_from_start 喵~
trajectory_msgs/JointTrajectoryPoint desired
# 实际轨迹点：编码器实际反馈的关节位置、速度和加速度喵~
trajectory_msgs/JointTrajectoryPoint actual
# 误差轨迹点：desired - actual 的差值，用于评估跟踪精度喵~
trajectory_msgs/JointTrajectoryPoint error
