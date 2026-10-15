// transport/foxglove/protocol.js — L0 协议客户端（自研，替代旧 client.js）
//
// 职责（刻意最薄）：WebSocket 连接 + foxglove.websocket.v1 子协议协商 +
// JSON op 分发 + 二进制帧编解码 + 发送助手。连接生命周期/重连/订阅簿记在上层 player.js。
//
// 二进制帧格式（与 vendored ivg_foxglove_bridge 服务器源码逐字节核对，
// vendor/foxglove-websocket/include/foxglove/websocket/{common,websocket_server}.hpp）：
//   服务器→客户端（首字节 opcode，小端）：
//     [0x01][subscriptionId u32][timestamp u64][payload]      消息数据
//     [0x02][timestamp u64]                                    时间
//     [0x03][serviceId u32][callId u32][encLen u32][enc][data] 服务响应
//     [0x04] fetchAsset 响应（未用，忽略）
//   客户端→服务器：
//     [0x01][channelId u32][payload]                           客户端发布
//     [0x02][serviceId u32][callId u32][encLen u32][enc][data] 服务调用请求
//   （ServiceRequest 与 ServiceResponse 同布局：common.hpp `using ServiceRequest = ServiceResponse`）
//
// JSON op：serverInfo/status/advertise/unadvertise/advertiseServices/unadvertiseServices/
//          serviceCallFailure/parameterValues 等（服务器→客户端）；
//          subscribe/unsubscribe/advertise/unadvertise（客户端→服务器）。

const SUBPROTOCOL = 'foxglove.websocket.v1';

const SERVER_OPCODE = {
	MESSAGE_DATA: 0x01,
	TIME: 0x02,
	SERVICE_CALL_RESPONSE: 0x03,
	FETCH_ASSET_RESPONSE: 0x04,
};

const CLIENT_OPCODE = {
	MESSAGE_DATA: 0x01,
	SERVICE_CALL_REQUEST: 0x02,
};

function u32le(view, offset) {
	return view.getUint32(offset, true);
}

function u64le(view, offset) {
	return view.getBigUint64(offset, true);
}

class FoxgloveProtocolClient {
	constructor() {
		this._ws = null;
		this._handlers = new Map(); // op/event → Set<fn>
		this._everConnected = false;
	}

	get isOpen() {
		return this._ws !== null && this._ws.readyState === 1;
	}

	get everConnected() {
		return this._everConnected;
	}

	on(event, fn) {
		if (!this._handlers.has(event)) this._handlers.set(event, new Set());
		this._handlers.get(event).add(fn);
		return () => this.off(event, fn);
	}

	off(event, fn) {
		const set = this._handlers.get(event);
		if (set) set.delete(fn);
	}

	_emit(event, arg) {
		const set = this._handlers.get(event);
		if (!set) return;
		for (const fn of [...set]) {
			try { fn(arg); } catch (e) { console.error(`[fgproto] ${event} handler error:`, e); }
		}
	}

	// ── 连接 ──────────────────────────────────────────────────────────────

	/**
	 * 连接并等待 serverInfo（协议首帧）。
	 * @param {string} url ws:// 地址（经网关代理或直连）
	 * @param {{connectTimeoutMs?: number}} [opts]
	 * @returns {Promise<object>} serverInfo
	 */
	connect(url, opts = {}) {
		const timeoutMs = opts.connectTimeoutMs ?? 10000;
		return new Promise((resolve, reject) => {
			if (this.isOpen) {
				reject(new Error('already connected'));
				return;
			}
			let settled = false;
			const timer = setTimeout(() => {
				if (settled) return;
				settled = true;
				try { this._ws && this._ws.close(); } catch (_) {}
				reject(new Error(`connect timeout after ${timeoutMs}ms`));
			}, timeoutMs);

			let ws;
			try {
				ws = new WebSocket(url, SUBPROTOCOL);
			} catch (e) {
				clearTimeout(timer);
				reject(e);
				return;
			}
			ws.binaryType = 'arraybuffer';
			this._ws = ws;

			const offInfo = this.on('serverInfo', (info) => {
				if (settled) return;
				settled = true;
				clearTimeout(timer);
				this._everConnected = true;
				resolve(info);
			});
			const offFail = this.on('_connectFail', (err) => {
				if (settled) return;
				settled = true;
				clearTimeout(timer);
				reject(err);
			});

			ws.onopen = () => { /* 等 serverInfo 再 resolve */ };
			ws.onclose = (ev) => {
				// 迟到的旧 socket close 事件不得误杀新连接（重连后旧 ws 才完成关闭时触发）
				if (this._ws !== ws) return;
				this._ws = null;
				offInfo();
				offFail();
				if (!settled) {
					settled = true;
					clearTimeout(timer);
					this._emit('_connectFail', new Error(`connection closed before serverInfo (code ${ev.code})`));
				}
				this._emit('close', ev);
			};
			ws.onerror = () => {
				if (this._ws !== ws && !this.isOpen) return;
				if (!settled) {
					this._emit('_connectFail', new Error('websocket error'));
				}
				this._emit('error');
			};
			ws.onmessage = (ev) => this._onMessage(ev.data);
		});
	}

	close() {
		if (this._ws) {
			const ws = this._ws;
			this._ws = null;
			try { ws.close(); } catch (_) {}
		}
	}

	_onMessage(data) {
		if (typeof data === 'string') {
			let msg;
			try {
				msg = JSON.parse(data);
			} catch (e) {
				console.warn('[fgproto] invalid json frame');
				return;
			}
			if (msg && msg.op) this._emit(msg.op, msg);
			return;
		}
		if (!(data instanceof ArrayBuffer)) return;
		const bytes = new Uint8Array(data);
		if (bytes.length < 1) return;
		const view = new DataView(data);
		const op = bytes[0];
		switch (op) {
			case SERVER_OPCODE.MESSAGE_DATA: {
				if (bytes.length < 13) return;
				const subscriptionId = u32le(view, 1);
				const timestamp = u64le(view, 5);
				this._emit('message', {
					subscriptionId,
					timestamp: Number(timestamp),
					payload: bytes.subarray(13),
				});
				break;
			}
			case SERVER_OPCODE.TIME: {
				if (bytes.length < 9) return;
				this._emit('time', { timestamp: Number(u64le(view, 1)) });
				break;
			}
			case SERVER_OPCODE.SERVICE_CALL_RESPONSE: {
				// [0x03][serviceId u32][callId u32][encLen u32][enc][data]
				if (bytes.length < 13) return;
				const serviceId = u32le(view, 1);
				const callId = u32le(view, 5);
				const encLen = u32le(view, 9);
				if (bytes.length < 13 + encLen) return;
				const encoding = new TextDecoder().decode(bytes.subarray(13, 13 + encLen));
				const payload = bytes.subarray(13 + encLen);
				this._emit('serviceCallResponse', { serviceId, callId, encoding, payload });
				break;
			}
			case SERVER_OPCODE.FETCH_ASSET_RESPONSE:
				// 未使用 assets capability，忽略
				break;
			default:
				break;
		}
	}

	// ── 发送 ──────────────────────────────────────────────────────────────

	_sendJson(obj) {
		if (!this.isOpen) return false;
		this._ws.send(JSON.stringify(obj));
		return true;
	}

	sendJsonOp(op) {
		return this._sendJson(op);
	}

	_sendBinary(frame) {
		if (!this.isOpen) return false;
		this._ws.send(frame);
		return true;
	}

	// 注：vendored ivg_foxglove_bridge 服务器（0.8.5 独立库线）的四个订阅/发布 op
	// 均为批量格式（websocket_server.hpp handleSubscribe/handleUnsubscribe/handleAdvertise/
	// handleUnadvertise 均遍历数组字段），与归档 spec 的单条形式不同——以服务器源码为准。
	subscribe(subscriptionId, channelId) {
		return this._sendJson({
			op: 'subscribe',
			subscriptions: [{ id: subscriptionId, channelId }],
		});
	}

	unsubscribe(subscriptionId) {
		return this._sendJson({ op: 'unsubscribe', subscriptionIds: [subscriptionId] });
	}

	advertise(channelId, { topic, encoding = 'cdr', schemaName, schema, schemaEncoding = 'ros2msg' }) {
		return this._sendJson({
			op: 'advertise',
			channels: [{
				id: channelId,
				topic,
				encoding,
				schemaName,
				...(schema !== undefined ? { schema, schemaEncoding } : {}),
			}],
		});
	}

	unadvertise(channelId) {
		return this._sendJson({ op: 'unadvertise', channelIds: [channelId] });
	}

	/** 客户端发布：[0x01][channelId u32][payload] */
	sendMessageData(channelId, payload) {
		const frame = new Uint8Array(5 + payload.byteLength);
		frame[0] = CLIENT_OPCODE.MESSAGE_DATA;
		new DataView(frame.buffer).setUint32(1, channelId, true);
		frame.set(payload, 5);
		return this._sendBinary(frame);
	}

	/** 服务调用请求：[0x02][serviceId][callId][encLen]['cdr'][payload] */
	sendServiceCallRequest(serviceId, callId, payload, encoding = 'cdr') {
		const encBytes = new TextEncoder().encode(encoding);
		const frame = new Uint8Array(13 + encBytes.length + payload.byteLength);
		frame[0] = CLIENT_OPCODE.SERVICE_CALL_REQUEST;
		const view = new DataView(frame.buffer);
		view.setUint32(1, serviceId, true);
		view.setUint32(5, callId, true);
		view.setUint32(9, encBytes.length, true);
		frame.set(encBytes, 13);
		frame.set(payload, 13 + encBytes.length);
		return this._sendBinary(frame);
	}
}

export { FoxgloveProtocolClient, SUBPROTOCOL };
