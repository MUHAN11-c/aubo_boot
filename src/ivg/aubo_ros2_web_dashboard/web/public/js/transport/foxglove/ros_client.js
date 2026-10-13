// transport/foxglove/ros_client.js — L7 ROS 门面（订阅/发布/服务/参数）
//
// 在 player（L1）之上提供面板可直用的 API，语义对齐原 rosbridge 侧 ivg_transport：
//   subscribe(topic, msgType, cb, {hz?, dispatchMode?}) — hz 由服务端默认 QoS 决定，
//     本层不再下发 throttle（foxglove 协议无 throttle 字段；节流语义由 dispatchMode 承担）
//   publish(topic, msgType, message) — CDR 客户端发布
//   callService(name, requestObj, {timeoutMs}) — CDR 服务调用（双完成路径+超时簿记在 player）
//   getParams(node, names) / setParam(node, name, value) — 直调各节点标准参数服务
//     （rcl_interfaces/srv/GetParameters|SetParameters，经同一条服务调用通道，零新增 ROS 节点）
//
// URL 构造复用 ivg_runtime（runtime API / defaults.yaml 单一来源）。

import { FoxglovePlayer } from './player.js';
import { foxgloveWebSocketUrl } from '../../ivg_runtime.js';

// rcl_interfaces/msg/ParameterValue.type（rosidl 定义）
const PARAM_TYPE = {
	NOT_SET: 0,
	BOOL: 1,
	INTEGER: 2,
	DOUBLE: 3,
	STRING: 4,
	BYTE_ARRAY: 5,
	BOOL_ARRAY: 6,
	INTEGER_ARRAY: 7,
	DOUBLE_ARRAY: 8,
	STRING_ARRAY: 9,
};

/** ParameterValue(msg 解码对象) → JS 原始值 */
function valueFromParameterValue(pv) {
	if (!pv) return undefined;
	switch (pv.type) {
		case PARAM_TYPE.BOOL: return pv.bool_value;
		case PARAM_TYPE.INTEGER: return Number(pv.integer_value);
		case PARAM_TYPE.DOUBLE: return pv.double_value;
		case PARAM_TYPE.STRING: return pv.string_value;
		case PARAM_TYPE.BYTE_ARRAY: return pv.byte_array_value;
		case PARAM_TYPE.BOOL_ARRAY: return pv.bool_array_value;
		case PARAM_TYPE.INTEGER_ARRAY: return (pv.integer_array_value || []).map(Number);
		case PARAM_TYPE.DOUBLE_ARRAY: return pv.double_array_value;
		case PARAM_TYPE.STRING_ARRAY: return pv.string_array_value;
		default: return undefined;
	}
}

/** JS 原始值 → ParameterValue 构造对象（供 MessageWriter 编码） */
function parameterValueFrom(v) {
	if (typeof v === 'boolean') return { type: PARAM_TYPE.BOOL, bool_value: v };
	if (typeof v === 'number') {
		return Number.isInteger(v)
			? { type: PARAM_TYPE.INTEGER, integer_value: v }
			: { type: PARAM_TYPE.DOUBLE, double_value: v };
	}
	if (typeof v === 'string') return { type: PARAM_TYPE.STRING, string_value: v };
	if (Array.isArray(v)) {
		if (v.length === 0) return { type: PARAM_TYPE.NOT_SET };
		if (v.every((x) => typeof x === 'number')) {
			return Number.isInteger(v[0])
				? { type: PARAM_TYPE.INTEGER_ARRAY, integer_array_value: v }
				: { type: PARAM_TYPE.DOUBLE_ARRAY, double_array_value: v };
		}
		if (v.every((x) => typeof x === 'string')) {
			return { type: PARAM_TYPE.STRING_ARRAY, string_array_value: v };
		}
		if (v.every((x) => typeof x === 'boolean')) {
			return { type: PARAM_TYPE.BOOL_ARRAY, bool_array_value: v };
		}
	}
	return { type: PARAM_TYPE.NOT_SET };
}

class FoxgloveRosClient {
	constructor() {
		this.player = new FoxglovePlayer();
		this._url = null;
	}

	get isConnected() {
		return this.player.isConnected;
	}

	get state() {
		return this.player.state;
	}

	get stats() {
		return { ...this.player.stats, codec: this.player.codec.stats };
	}

	onStateChange(fn) {
		return this.player.onStateChange(fn);
	}

	onStatus(fn) {
		return this.player.onStatus(fn);
	}

	async connect(urlOverride) {
		// 单飞语义：页面内多消费方（hub 适配器/TF 缓冲/URDF 岛）并发 connect 只建一条连接
		if (this.player.isConnected) return;
		if (this._connectPromise) return this._connectPromise;
		this._url = urlOverride || this._url || foxgloveWebSocketUrl();
		this._connectPromise = this.player.connect(this._url).finally(() => {
			this._connectPromise = null;
		});
		return this._connectPromise;
	}

	disconnect() {
		this.player.stop();
	}

	// ── 话题 ──────────────────────────────────────────────────────────────

	subscribe(topic, msgType, callback, opts = {}) {
		// msgType 仅作交叉校验提示（权威类型来自服务器 channel schema）
		const channel = this.player.channels.get(topic);
		if (channel && msgType && channel.schemaName !== msgType) {
			console.warn(
				`[fgclient] ${topic} 类型不一致：请求 ${msgType}，图上 ${channel.schemaName}（以图上为准）`);
		}
		return this.player.subscribe(topic, callback, { dispatchMode: opts.dispatchMode });
	}

	publish(topic, msgType, message) {
		return this.player.publish(topic, msgType, message);
	}

	// ── 服务 ──────────────────────────────────────────────────────────────

	callService(name, requestObj, opts = {}) {
		return this.player.callService(name, requestObj, opts);
	}

	// ── 连接图（topics/nodes/services 图谱查询）─────────────────────────

	/** 图谱快照（首次调用时订阅 connectionGraph，之后持续更新） */
	getGraph() {
		return this.player.subscribeConnectionGraph();
	}

	// ── 参数（标准参数服务直调） ──────────────────────────────────────────

	/**
	 * 读节点参数。names 为空数组时读全部（get_parameters 空名单=全量）。
	 * @returns {Promise<Object<string, *>>} name → JS 值
	 */
	async getParams(node, names = []) {
		const nodeName = node.startsWith('/') ? node : `/${node}`;
		const resp = await this.player.callService(`${nodeName}/get_parameters`, { names }, { timeoutMs: 10000 });
		const out = {};
		const values = resp.values || [];
		for (let i = 0; i < values.length; i++) {
			out[names[i] ?? `#${i}`] = valueFromParameterValue(values[i]);
		}
		return out;
	}

	/**
	 * 设置节点参数并返回各赋值是否成功。
	 * @returns {Promise<{successful: boolean, reason?: string}>}
	 */
	async setParam(node, name, value) {
		const nodeName = node.startsWith('/') ? node : `/${node}`;
		const parameter = { name, value: parameterValueFrom(value) };
		const resp = await this.player.callService(
			`${nodeName}/set_parameters`, { parameters: [parameter] }, { timeoutMs: 10000 });
		const result = (resp.results || [])[0] || {};
		return { successful: !!result.successful, reason: result.reason };
	}
}

// 单例（页面直接 import；hub 适配器也复用同一实例）
let _instance = null;

function foxgloveRosClient() {
	if (!_instance) _instance = new FoxgloveRosClient();
	return _instance;
}

export {
	foxgloveRosClient,
	FoxgloveRosClient,
	valueFromParameterValue,
	parameterValueFrom,
	PARAM_TYPE,
};
