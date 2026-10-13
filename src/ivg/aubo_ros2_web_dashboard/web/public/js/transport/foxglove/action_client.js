// transport/foxglove/action_client.js — ROS 2 Action 客户端（服务+话题组合实现）
//
// foxglove WebSocket 协议没有 action 概念；ROS 2 action 本身 = 3 服务 + 2 话题
// （_action/send_goal、_action/cancel_goal、_action/get_result；_action/status、
// _action/feedback）。本模块在 ros_client 的服务调用与话题订阅之上做编排，
// 使 web 端能力面对齐 rosbridge+roslibjs 2.x（其 action 支持同理是组合实现）。
//
// 用法：
//   import { createActionClient } from './action_client.js';
//   const ac = createActionClient('/fibonacci');
//   const { status, result } = await ac.sendGoal({ order: 8 }, {
//     onFeedback: (fb) => console.log('feedback', fb.sequence),
//     timeoutMs: 120000,
//   });
//   await ac.cancelAll();  // 可选中途取消（sendGoal 的 promise 会随 get_result 结算）
//
// 服务 schema（含嵌套 UUID/GoalInfo/Goal/Result）由桥 advertiseServices 下发，
// codec 自动 CDR 编解码；本层只做编排与字段名对齐：
//   send_goal 请求: { goal: <ActionGoal>, goal_id: { uuid: uint8[16] } }
//   get_result 请求: { goal_info: { goal_id: {uuid}, stamp: {sec,nanosec} } }
//   get_result 响应: { status: int8, result: <ActionResult> }
//   feedback 消息: { goal_id: {uuid}, feedback: <ActionFeedback> }
// status 码（action_msgs/GoalStatus）：0 UNKNOWN 1 ACCEPTED 2 EXECUTING 3 CANCELING
//   4 SUCCEEDED 5 CANCELED 6 ABORTED

import { foxgloveRosClient } from './ros_client.js';

const GOAL_STATUS = {
	UNKNOWN: 0, ACCEPTED: 1, EXECUTING: 2, CANCELING: 3,
	SUCCEEDED: 4, CANCELED: 5, ABORTED: 6,
};
const GOAL_STATUS_NAMES = ['UNKNOWN', 'ACCEPTED', 'EXECUTING', 'CANCELING', 'SUCCEEDED', 'CANCELED', 'ABORTED'];

let _uuidCounter = 0;
function randomUuid() {
	// goal_id 仅作关联用途（非安全令牌）；优先 CSPRNG，无 crypto 时用时间熵+计数器
	if (globalThis.crypto && crypto.getRandomValues) {
		const u = new Uint8Array(16);
		crypto.getRandomValues(u);
		return u;
	}
	const u = new Uint8Array(16);
	const t = performance.now();
	const c = ++_uuidCounter;
	for (let i = 0; i < 6; i++) u[i] = (Math.floor(t / 2 ** (8 * i)) ^ 0) & 0xff;
	for (let i = 0; i < 4; i++) u[6 + i] = (c >>> (8 * i)) & 0xff;
	for (let i = 10; i < 16; i++) u[i] = (Date.now() >>> ((i - 10) * 5)) & 0xff;
	return u;
}

function toPlainArray(v) {
	if (v instanceof Uint8Array) return Array.from(v);
	if (Array.isArray(v)) return v;
	if (v && typeof v === 'object' && typeof v.length === 'number') return Array.from(v);
	return v;
}

function createActionClient(actionName, opts = {}) {
	const name = actionName.replace(/\/+$/, '');
	const client = foxgloveRosClient();
	const feedbackTopic = `${name}/_action/feedback`;
	const statusTopic = `${name}/_action/status`;

	let _fbHandle = null;
	let _stHandle = null;
	let _fbCallbacks = new Set();
	let _lastStatusArray = null;

	function _ensureConnected() {
		if (!client.isConnected) return client.connect().catch(() => { });
		return Promise.resolve();
	}

	function _stampNow() {
		const ms = Date.now();
		return { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1e6 };
	}

	/**
	 * 发送 goal 并等待终结（get_result 在 goal 终结前阻塞，服务端语义保证）。
	 * @param {object} goalMsg Action 的 Goal 消息（字段名与 .action 定义一致）
	 * @param {{onFeedback?: (fb, goalId) => void, onStatus?: (statusArray) => void,
	 *          resultTimeoutMs?: number}} [callOpts]
	 * @returns {Promise<{status: number, statusText: string, result: object}>}
	 */
	async function sendGoal(goalMsg, callOpts = {}) {
		await _ensureConnected();
		const goalId = randomUuid();

		// 反馈/状态订阅（首个消费者时建立，长期保留——action server 生命周期内复用）
		if (callOpts.onFeedback && !_fbCallbacks.size) {
			_fbHandle = client.subscribe(feedbackTopic, '', (m) => {
				const d = m.data || {};
				for (const cb of [..._fbCallbacks]) {
					try { cb(d.feedback, toPlainArray(d.goal_id && d.goal_id.uuid)); } catch (_) { }
				}
			}, { dispatchMode: 'latest' });
		}
		if (callOpts.onFeedback) _fbCallbacks.add(callOpts.onFeedback);
		if (callOpts.onStatus && !_stHandle) {
			_stHandle = client.subscribe(statusTopic, 'action_msgs/msg/GoalStatusArray', (m) => {
				_lastStatusArray = (m.data && m.data.status_list) || [];
				try { callOpts.onStatus(_lastStatusArray); } catch (_) { }
			}, { dispatchMode: 'latest' });
		}

		// 1) send_goal（服务刚起时 advertiseServices 首帧可能未达——一次性补等重试）
		// 注：send_goal 的 requestSchema 是扁平展开的（goal 嵌套消息的字段直接在顶层，
		// 如 `{order: N}` 而非 `{goal: {order}}`）——把 goalMsg 的字段展开到请求顶层
		const flatReq = { ...goalMsg, goal_id: { uuid: goalId } };
		let accepted = null;
		for (let attempt = 0; attempt < 2 && accepted === null; attempt++) {
			try {
				const r = await client.callService(
					`${name}/_action/send_goal`, flatReq,
					{ timeoutMs: callOpts.sendTimeoutMs || 15000 },
				);
				accepted = !!(r && r.accepted);
			} catch (_) {
				if (attempt === 0) {
					await new Promise((r2) => setTimeout(r2, 1500));
				} else {
					accepted = null;
				}
			}
		}
		if (accepted === null) {
			if (callOpts.onFeedback) _fbCallbacks.delete(callOpts.onFeedback);
			throw new Error('send_goal 调用失败');
		}
		if (!accepted) {
			if (callOpts.onFeedback) _fbCallbacks.delete(callOpts.onFeedback);
			return { status: -1, statusText: 'REJECTED', result: null };
		}

		// 2) get_result（阻塞至终结）
		// 注：get_result 的 requestSchema 也是扁平展开的——goal_info 被展开为顶层
		// {goal_id}（不含 stamp），响应为顶层 {status, <Result 字段...>} 而非嵌套 result
		try {
			const res = await client.callService(
				`${name}/_action/get_result`,
				{ goal_id: { uuid: goalId } },
				{ timeoutMs: callOpts.resultTimeoutMs || 120000 },
			);
			const status = Number(res.status);
			// 响应扁平：除 status 外的字段都是 <Action>Result 的字段
			const result = { ...res };
			delete result.status;
			return {
				status,
				statusText: GOAL_STATUS_NAMES[status] || `STATUS_${status}`,
				result,
				goalId: Array.from(goalId),
			};
		} catch (e) {
			// get_result 失败不回滚 goal——取消兜底
			try {
				await cancelGoal(goalId);
			} catch (_) { }
			throw new Error(`get_result 失败: ${e.message || e}`);
		} finally {
			if (callOpts.onFeedback) _fbCallbacks.delete(callOpts.onFeedback);
		}
	}

	/** 取消指定 goal；goalId 省略时取消该 server 全部 goal。 */
	async function cancelGoal(goalId) {
		await _ensureConnected();
		const uuid = goalId ? toPlainArray(goalId) : new Array(16).fill(0);
		const resp = await client.callService(
			`${name}/_action/cancel_goal`,
			{ goal_info: { goal_id: { uuid }, stamp: _stampNow() } },
			{ timeoutMs: 10000 },
		);
		return { returnCode: Number(resp.return_code), goalsCanceling: resp.goals_canceling || [] };
	}

	async function cancelAll() {
		return cancelGoal(null);
	}

	function dispose() {
		if (_fbHandle) { try { _fbHandle.cancel(); } catch (_) { } _fbHandle = null; }
		if (_stHandle) { try { _stHandle.cancel(); } catch (_) { } _stHandle = null; }
		_fbCallbacks.clear();
		_lastStatusArray = null;
	}

	function getStatusSnapshot() {
		return _lastStatusArray;
	}

	return { sendGoal, cancelGoal, cancelAll, dispose, getStatusSnapshot, actionName: name };
}

export { createActionClient, GOAL_STATUS, GOAL_STATUS_NAMES };
