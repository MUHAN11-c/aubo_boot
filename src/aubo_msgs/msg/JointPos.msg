# 六轴关节角（ROS 1 AUBO 驱动遗留）。当前 Jazzy 驱动栈不发布、不消费；
# 运动与 FK/IK 走 FollowJointTrajectory / MoveIt。
# 关节顺序：shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint；单位 rad。

float32 joint1
float32 joint2
float32 joint3
float32 joint4
float32 joint5
float32 joint6
