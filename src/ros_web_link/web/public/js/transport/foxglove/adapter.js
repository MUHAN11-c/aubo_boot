// transport/foxglove/adapter.js — FoxgloveAdapter implements MessageAdapter
//
// v2（2026-10-10 自研传输栈）：本适配器改为 ros_client（protocol/player/codec）之上的薄壳，
// 旧 client.js（JSON op + 裸 CDR 的非规范服务帧）退役，仅保留文件作兼容期参考。
// 能力面：订阅（immediate/latest 派发）、CDR 客户端发布、CDR 服务调用（双完成路径+超时）、
// 经标准参数服务的参数读写（getParams/setParam，由上层直接用 ros_client）。
//
// TypedMessage 语义与其余适配器一致：{ topic, type, data, timestamp(ms), bridge: 'foxglove' }。

import { foxgloveRosClient } from './ros_client.js';
import { MessageAdapter } from '../adapter.js';

class FoxgloveAdapter extends MessageAdapter {
	constructor() {
		super();
		this._client = foxgloveRosClient();
		this._messageCbs = new Set();
		this._statusCbs = new Set();
		this._status = { state: 'disconnected', since: Date.now() };
		this._topicCallbacks = new Map(); // topic → Set<cb>（unsubscribe 全量语义需要）
		this._unsubHooks = new Map(); // topic → cancel 句柄
		this._offState = null;
	}

	get id() { return 'foxglove'; }
	get isConnected() { return this._client.isConnected; }

	// ── 连接管理 ──────────────────────────────────────────────────────────

	async connect(url) {
		this._setStatus('connecting');
		this._offState = this._client.onStateChange((s) => {
			if (s === 'connected') this._setStatus('connected');
			else if (s === 'reconnecting' || s === 'connecting') this._setStatus('connecting');
			else this._setStatus('disconnected');
		});
		try {
			await this._client.connect(url);
			this._setStatus('connected');
		} catch (e) {
			// 首连失败时 player 仍在后台退避重连：给宽限期等转正，避免 hub 因瞬态失败误判降级
			const recovered = await new Promise((resolve) => {
				if (this._client.isConnected) return resolve(true);
				const off = this._client.onStateChange((s) => {
					if (s === 'connected') {
						cleanup();
						resolve(true);
					}
				});
				const timer = setTimeout(() => {
					cleanup();
					resolve(false);
				}, 15000);
				const cleanup = () => {
					clearTimeout(timer);
					off();
				};
			});
			if (recovered) {
				this._setStatus('connected');
			} else {
				this._setStatus('error', e.message);
				throw e;
			}
		}
	}

	disconnect() {
		if (this._offState) this._offState();
		this._client.disconnect();
		this._setStatus('disconnected');
	}

	_setStatus(state, error) {
		this._status = { state, ...(error ? { error } : {}), since: Date.now() };
		for (const cb of [...this._statusCbs]) {
			try { cb(this._status); } catch (_) {}
		}
	}

	// ── 话题订阅 ──────────────────────────────────────────────────────────

	subscribe(topic, msgType, callback) {
		let cbs = this._topicCallbacks.get(topic);
		if (!cbs) {
			cbs = new Set();
			this._topicCallbacks.set(topic, cbs);
		}
		cbs.add(callback);

		if (!this._unsubHooks.has(topic)) {
			const handle = this._client.subscribe(topic, msgType, (typed) => {
				for (const cb of [...cbs]) {
					try { cb(typed); } catch (_) {}
				}
				for (const cb of [...this._messageCbs]) {
					try { cb(typed); } catch (_) {}
				}
			}, { dispatchMode: 'latest' });
			this._unsubHooks.set(topic, handle);
		}
		return { cancel: () => this._removeCallback(topic, callback) };
	}

	unsubscribe(topic) {
		// MessageAdapter 语义：取消该话题全部回调（与 hub/SubRegistry 对齐）
		this._topicCallbacks.delete(topic);
		const handle = this._unsubHooks.get(topic);
		if (handle) {
			handle.cancel();
			this._unsubHooks.delete(topic);
		}
	}

	_removeCallback(topic, callback) {
		const cbs = this._topicCallbacks.get(topic);
		if (!cbs) return;
		cbs.delete(callback);
		if (cbs.size === 0) this.unsubscribe(topic);
	}

	// ── 发布 / 服务 ───────────────────────────────────────────────────────

	async publish(topic, msgType, msg) {
		const ok = this._client.publish(topic, msgType, msg);
		if (!ok) throw new Error(`publish(${topic}) 失败：话题不存在或无 schema`);
	}

	async callService(name, type, request, timeoutMs = 60000) {
		return this._client.callService(name, request, { timeoutMs });
	}

	// ── 回调 / 能力 / 统计 ────────────────────────────────────────────────

	onMessage(callback) {
		this._messageCbs.add(callback);
		return () => this._messageCbs.delete(callback);
	}

	onStatusChange(callback) {
		this._statusCbs.add(callback);
		return () => this._statusCbs.delete(callback);
	}

	getCapabilities() {
		const caps = new Set(['clientPublish']);
		if (this._client.player.capabilities.includes('services')) caps.add('services');
		return caps;
	}

	getStats() {
		const s = this._client.stats;
		return {
			channelsCount: this._client.player.channels.size,
			servicesCount: s.codec ? s.codec.services : 0,
			bytesReceived: s.bytesReceived,
			messagesReceived: s.messagesReceived,
			connectedAt: s.connectedAt,
		};
	}
}

const foxgloveAdapter = new FoxgloveAdapter();

export { FoxgloveAdapter, foxgloveAdapter };
