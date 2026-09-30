// core/topics.js — 共享 ROS topic 常量
// 单一数据源，避免 topic 名/类型拼写不一致

// 2026-09-30 移植适配：机械臂状态换本仓 aubo_io_controller 发布源
// （aubo_msgs/RobotStatus，字段 mode/e_stopped/drives_powered/motion_possible…，
// 消费端按 drives_powered/motion_possible 映射旧 is_online/enable 语义）
export const ROBOT_STATUS_TOPIC = '/aubo_io_controller/robot_status';
export const ROBOT_STATUS_TYPE  = 'aubo_msgs/msg/RobotStatus';

export const MODE_TOPIC = '/aubo/mode';
export const MODE_TYPE  = 'std_msgs/msg/String';

export const JOINT_STATES_TOPIC = '/joint_states';
export const JOINT_STATES_TYPE  = 'sensor_msgs/msg/JointState';

export const TOOL_CHANGER_STATUS_TOPIC = '/tool_changer_status';
export const TOOL_CHANGER_STATUS_TYPE  = 'ivg_interfaces/msg/ToolChangerStatus';

export const TF_TOPIC       = '/tf';
export const TF_STATIC_TOPIC = '/tf_static';
export const TF_TYPE        = 'tf2_msgs/msg/TFMessage';

// aubo_msgs/RobotStatus → 旧 ivg RobotStatus 字段语义映射（消费端保持零改动）
// is_online ≈ drives_powered（48V 臂电）；enable ≈ motion_possible（可接新轨）
export function mapRobotStatus(msg) {
  if (!msg) return msg;
  return {
    ...msg,
    is_online: !!msg.drives_powered,
    enable: !!msg.motion_possible,
    in_motion: !!msg.in_motion,
    e_stopped: !!msg.e_stopped,
  };
}
