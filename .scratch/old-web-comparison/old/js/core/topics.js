// core/topics.js — 共享 ROS topic 常量
// 单一数据源，避免 topic 名/类型拼写不一致

export const ROBOT_STATUS_TOPIC = '/aubo_driver/robot_status';
export const ROBOT_STATUS_TYPE  = 'demo_interface/msg/RobotStatus';

export const MODE_TOPIC = '/aubo/mode';
export const MODE_TYPE  = 'std_msgs/msg/String';

export const JOINT_STATES_TOPIC = '/joint_states';
export const JOINT_STATES_TYPE  = 'sensor_msgs/msg/JointState';

export const TF_TOPIC       = '/tf';
export const TF_STATIC_TOPIC = '/tf_static';
export const TF_TYPE        = 'tf2_msgs/msg/TFMessage';
