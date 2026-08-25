# 扁平轨迹点列（ROS 1 AUBO 驱动遗留）。当前 Jazzy 驱动栈不发布、不消费。
# 透传轨迹走 JointTrajectory，不走本类型。

int8 bus                # 原驱动总线/空间选择
int32 num_of_trapoint   # 轨迹点数 N
float32[] trapoints     # 扁平化点列，长度应为 N×6，每点 6 关节 [rad]
