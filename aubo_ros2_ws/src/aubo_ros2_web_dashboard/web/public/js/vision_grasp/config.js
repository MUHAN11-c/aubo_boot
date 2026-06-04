/**
 * 视觉抓取配置声明表：
 * - 统一管理页面设置项 id / 默认值 / 类型信息
 * - 避免在编排代码中分散硬编码
 */

const VISION_SETTING_DEFS = [
	{ id: 'topic-color', category: 'topic', defaultValue: '/camera/color/image_raw', msgType: 'sensor_msgs/msg/Image' },
	{ id: 'topic-result', category: 'topic', defaultValue: '', allowEmpty: true, msgType: 'sensor_msgs/msg/Image' },
	{ id: 'topic-robot', category: 'topic', defaultValue: '/robot_status', msgType: 'ivg_interfaces/msg/RobotStatus' },
	{ id: 'topic-joints', category: 'topic', defaultValue: '/joint_states', msgType: 'sensor_msgs/msg/JointState' },
	{ id: 'topic-vpe-status', category: 'topic', defaultValue: '/system_status', msgType: 'std_msgs/msg/String' },
	{ id: 'topic-grasp-poses', category: 'topic', defaultValue: '/grasp_poses_base', msgType: 'geometry_msgs/msg/PoseArray' },
	{ id: 'topic-tf', category: 'topic', defaultValue: '/tf', msgType: 'tf2_msgs/msg/TFMessage', isTf: true },
	{ id: 'topic-tf-static', category: 'topic', defaultValue: '/tf_static', msgType: 'tf2_msgs/msg/TFMessage', isTf: true },
	{ id: 'svc-loop-grasp-control', category: 'service', defaultValue: '/loop_grasp_control', serviceType: 'std_srvs/srv/SetBool' },
	{ id: 'svc-graspnet-capture', category: 'service', defaultValue: '/graspnet_capture_control', serviceType: 'std_srvs/srv/SetBool' },
	{ id: 'svc-publish-grasps-loop', category: 'service', defaultValue: '/publish_grasps_worker_loop_control', serviceType: 'std_srvs/srv/SetBool' },
	{ id: 'svc-gripper-swap', category: 'service', defaultValue: '/run_gripper_swap', serviceType: 'ivg_interfaces/srv/RunGripperSwap' },
	{ id: 'topic-tool-changer-status', category: 'topic', defaultValue: '/tool_changer_status', msgType: 'ivg_interfaces/msg/ToolChangerStatus' }
];
/** 固定业务服务（不在设置面板中暴露 id 编辑） */
const VISION_FIXED_SERVICE_TYPES = {
	'execute-single-grasp': 'ivg_interfaces/srv/ExecuteGraspPose'
};

function buildSettingsDefaults(defs) {
	const out = {};
	(defs || []).forEach(def => {
		out[def.id] = def.defaultValue;
	});
	return out;
}

function collectSettingIds(defs) {
	return (defs || []).map(def => def.id);
}

function collectTopicIds(defs) {
	return (defs || []).filter(def => def.category === 'topic' && !def.isTf).map(def => def.id);
}

function collectTfTopicIds(defs) {
	return (defs || []).filter(def => def.category === 'topic' && def.isTf).map(def => def.id);
}

function collectServiceIds(defs) {
	return (defs || []).filter(def => def.category === 'service').map(def => def.id);
}

function buildTopicTypeMap(defs) {
	const out = {};
	(defs || []).forEach(def => {
		if (def.category === 'topic' && def.msgType) out[def.id] = def.msgType;
	});
	return out;
}

/** service id -> serviceType 映射（供调用层统一引用） */
function buildServiceTypeMap(defs) {
	const out = {};
	(defs || []).forEach(def => {
		if (def.category === 'service' && def.serviceType) out[def.id] = def.serviceType;
	});
	return out;
}

const VISION_SETTINGS_DEFAULTS = buildSettingsDefaults(VISION_SETTING_DEFS);
const VISION_ALL_SETTING_IDS = collectSettingIds(VISION_SETTING_DEFS);
const VISION_TOPIC_IDS = collectTopicIds(VISION_SETTING_DEFS);
const VISION_TF_TOPIC_IDS = collectTfTopicIds(VISION_SETTING_DEFS);
const VISION_SERVICE_IDS = collectServiceIds(VISION_SETTING_DEFS);
const VISION_TOPIC_TYPE_MAP = buildTopicTypeMap(VISION_SETTING_DEFS);
const VISION_SERVICE_TYPE_MAP = buildServiceTypeMap(VISION_SETTING_DEFS);

export {
	VISION_SETTING_DEFS,
	VISION_SETTINGS_DEFAULTS,
	VISION_ALL_SETTING_IDS,
	VISION_TOPIC_IDS,
	VISION_TF_TOPIC_IDS,
	VISION_SERVICE_IDS,
	VISION_TOPIC_TYPE_MAP,
	VISION_SERVICE_TYPE_MAP,
	VISION_FIXED_SERVICE_TYPES
};
