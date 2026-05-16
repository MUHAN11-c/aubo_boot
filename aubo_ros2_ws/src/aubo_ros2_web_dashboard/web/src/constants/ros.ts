/**
 * ROS 话题/服务常量 — 项目全局唯一数据源
 *
 * 设计原则:
 *   - 所有话题名和消息类型在此集中定义
 *   - 避免各组件分散硬编码导致拼写不一致
 *   - 修改话题名只需改一处
 */

// ═══════════════════════ 机械臂状态 ═══════════════════════

/** 机械臂综合状态 (在线/使能/运动/规划) — aubo_state_broadcaster 发布 */
export const ROBOT_STATUS_TOPIC = '/aubo_driver/robot_status'
export const ROBOT_STATUS_TYPE  = 'ivg_interfaces/msg/RobotStatus'

/** 驱动模式 (real/simulation) — aubo_mode 节点发布，transient_local QoS */
export const MODE_TOPIC = '/aubo/mode'
export const MODE_TYPE  = 'std_msgs/msg/String'

// ═══════════════════════ 关节与 TF ═══════════════════════

/** 关节角 position/velocity — 200Hz，由 robot_state_publisher 发布 */
export const JOINT_STATES_TOPIC = '/joint_states'
export const JOINT_STATES_TYPE  = 'sensor_msgs/msg/JointState'

/** 动态 TF 变换 — 30Hz */
export const TF_TOPIC       = '/tf'
export const TF_STATIC_TOPIC = '/tf_static'
export const TF_TYPE        = 'tf2_msgs/msg/TFMessage'

// ═══════════════════════ 工具快换 ═══════════════════════

/** 工具快换状态 — gripper_swap_worker 发布 */
export const TOOL_CHANGER_STATUS_TOPIC = '/tool_changer_status'
export const TOOL_CHANGER_STATUS_TYPE  = 'ivg_interfaces/msg/ToolChangerStatus'

// ═══════════════════════ 咖啡拉花 ═══════════════════════

/** DI 反馈 (JSON/CSV 格式字符串) — latte_node 发布 */
export const LATTE_DI_STATUS_TOPIC = '/latte_di_status'
export const LATTE_DI_STATUS_TYPE  = 'std_msgs/msg/String'

// ═══════════════════════ 并排 Web 服务端口（与 start_aubo_new_driver.sh 默认一致）════════════════════════

/** 视觉位姿 FastAPI — WEB_PORT 默认 */
export const VPE_WEB_PORT = 8088

/** 手眼标定 Flask — HAND_EYE_PORT 默认 */
export const HAND_EYE_WEB_PORT = 8070
