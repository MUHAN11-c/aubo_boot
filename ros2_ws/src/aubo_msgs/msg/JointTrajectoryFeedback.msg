# 自定义关节轨迹反馈消息
# 格式与 control_msgs/FollowJointTrajectoryFeedback 相同，用于 ROS 1 到 ROS 2 桥接
std_msgs/Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint desired
trajectory_msgs/JointTrajectoryPoint actual
trajectory_msgs/JointTrajectoryPoint error

