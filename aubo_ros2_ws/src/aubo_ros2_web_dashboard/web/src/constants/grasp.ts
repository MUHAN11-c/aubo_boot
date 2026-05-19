/**
 * 视觉抓取配置 — 话题/服务/工具静态数据
 *
 * 用途: SettingsView 动态表单渲染 + VisionGraspView 默认值
 * 来源: 旧版 vision_grasp/config.js (BFF 回退)
 */

// ═══════════════════════ 设置项定义 ═══════════════════════

export interface VisionSettingDef {
  id: string
  category: 'topic' | 'service'
  defaultValue: string
  allowEmpty?: boolean
  msgType?: string
  serviceType?: string
  isTf?: boolean
}

/** 所有可配置的话题/服务设置项 — 设置页据此生成表单 */
export const VISION_SETTING_DEFS: VisionSettingDef[] = [
  { id: 'topic-color', category: 'topic', defaultValue: '/camera/color/image_raw', msgType: 'sensor_msgs/msg/Image' },
  { id: 'topic-result', category: 'topic', defaultValue: '', allowEmpty: true, msgType: 'sensor_msgs/msg/Image' },
  { id: 'topic-robot', category: 'topic', defaultValue: '/robot_status', msgType: 'ivg_interfaces/msg/RobotStatus' },
  { id: 'topic-joints', category: 'topic', defaultValue: '/joint_states', msgType: 'sensor_msgs/msg/JointState' },
  { id: 'topic-tool-status', category: 'topic', defaultValue: '/tool_changer_status', msgType: 'ivg_interfaces/msg/ToolChangerStatus' },
  { id: 'urdf-param', category: 'topic', defaultValue: '/robot_state_publisher:robot_description' },
  { id: 'tf-fixed-frame', category: 'topic', defaultValue: 'base_link' },
  { id: 'topic-vpe-status', category: 'topic', defaultValue: '/system_status', msgType: 'std_msgs/msg/String' },
  { id: 'topic-grasp-poses', category: 'topic', defaultValue: '/grasp_poses_base', msgType: 'geometry_msgs/msg/PoseArray' },
  { id: 'topic-tf', category: 'topic', defaultValue: '/tf', msgType: 'tf2_msgs/msg/TFMessage', isTf: true },
  { id: 'topic-tf-static', category: 'topic', defaultValue: '/tf_static', msgType: 'tf2_msgs/msg/TFMessage', isTf: true },
  { id: 'svc-loop-grasp-control', category: 'service', defaultValue: '/loop_grasp_control', serviceType: 'std_srvs/srv/SetBool' },
  { id: 'svc-graspnet-capture', category: 'service', defaultValue: '/graspnet_capture_control', serviceType: 'std_srvs/srv/SetBool' },
  { id: 'svc-publish-grasps-loop', category: 'service', defaultValue: '/publish_grasps_worker_loop_control', serviceType: 'std_srvs/srv/SetBool' },
  { id: 'svc-gripper-swap', category: 'service', defaultValue: '/run_gripper_swap', serviceType: 'ivg_interfaces/srv/RunGripperSwap' },
]

/** 固定服务类型 — 不在设置面板暴露的服务 */
export const FIXED_SERVICE_TYPES: Record<string, string> = {
  'execute-single-grasp': 'ivg_interfaces/srv/ExecuteGraspPose',
}

// ═══════════════════════ 快换工具列表 ═══════════════════════

/** 可用快换工具 — 与 tools.yaml 保持同步 */
export const TOOL_LIST = [
  { id: 'gripper0', label: '夹爪0 φ40' },
  { id: 'gripper2', label: '夹爪2 φ60' },
]

// ═══════════════════════ 派生映射 ═══════════════════════

/** id → 默认值 */
export const VISION_SETTINGS_DEFAULTS: Record<string, string> = Object.fromEntries(
  VISION_SETTING_DEFS.map(d => [d.id, d.defaultValue])
)

/** id → msgType (非 TF 话题) */
export const TOPIC_TYPE_MAP: Record<string, string> = Object.fromEntries(
  VISION_SETTING_DEFS.filter(d => d.category === 'topic' && d.msgType).map(d => [d.id, d.msgType!])
)

/** id → serviceType */
export const SERVICE_TYPE_MAP: Record<string, string> = Object.fromEntries(
  VISION_SETTING_DEFS.filter(d => d.category === 'service' && d.serviceType).map(d => [d.id, d.serviceType!])
)
