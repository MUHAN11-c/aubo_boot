/**
 * IVG 浏览器传输：控制面 JSON + 二进制（仅网关已拆包后的固定头格式）。
 * 不在此做 ROS 解码、业务计算；MJPEG 像素解码由浏览器原生 <img> 完成。
 */
(function (g) {
	'use strict';

	function uuid() {
		return `ivg_${Date.now()}_${Math.random().toString(16).slice(2)}`;
	}

	function IvgTransport() {
		this.runtime = null;
		this.ctrl = null;
		this.bin = null;
		this._topicSubs = new Set();
		this._rosHandlers = [];
		this._svcPending = new Map();
		this._binaryHandlers = [];
		this._controlHandlers = [];
		this._reconnect = { gen: 0, attempts: 0, timer: null };
		/** rosbridge 分片 JSON 重组（与 roslib handleRosbridgeFragmentMessage 行为一致） */
		this._fragmentBuf = new Map();
		/** topic → subscribe id，供 rosbridge 侧 unsubscribe 与 roslib 对齐 */
		this._rosbridgeSubIds = new Map();
	}

	/** 魔数 0x56504301 = "VPC\\x01"，小端 uint32 顶点数，随后 float32 xyz（由 ivg_web_serve 自内部帧转换） */
	var TAG_VPC = 0x56504301;
	/** 魔数 0x564a5001 = "VJP\\x01" + u32 jpeg 长度 + 裸 JPEG */
	var TAG_VJP = 0x564a5001;

	/** 运行时显式给出 ``ivg_ws_control`` 时用 IVG 原生 JSON；否则走 ``rosbridge_ws_path``（与 web_dashboard.launch 网关一致）。 */
	IvgTransport.prototype._nativeIvgWire = function () {
		const rt = this.runtime || {};
		return !!(rt.ivg_ws_control && String(rt.ivg_ws_control).trim());
	};

	IvgTransport.prototype._controlWebSocketUrl = function () {
		const rt = this.runtime || {};
		if (rt.ivg_ws_control && String(rt.ivg_ws_control).trim()) return String(rt.ivg_ws_control).trim();
		const path = (rt.rosbridge_ws_path && String(rt.rosbridge_ws_path).trim()) || '/ws/rosbridge';
		const proto = g.location.protocol === 'https:' ? 'wss:' : 'ws:';
		return `${proto}//${g.location.host}${path.startsWith('/') ? path : `/${path}`}`;
	};

	function encodeTopicQueryValue(topic) {
		return String(topic)
			.split('/')
			.map(function (seg) {
				return encodeURIComponent(seg);
			})
			.join('/');
	}

	/** 先 ``/api/v1/runtime``，失败或无 WS 字段时再请求 ``/api/ivg/runtime-config``（与网关路由一致）。 */
	IvgTransport.prototype.loadRuntime = async function () {
		let data = {};
		try {
			const r = await fetch('/api/v1/runtime', { credentials: 'same-origin' });
			if (r.ok) {
				const j = await r.json();
				if (j && typeof j === 'object') data = j;
			}
		} catch (e) { /* ignore */ }
		if (!data.ivg_ws_control) {
			try {
				const r2 = await fetch('/api/ivg/runtime-config', { credentials: 'same-origin' });
				if (r2.ok) {
					const leg = await r2.json();
					if (leg && typeof leg === 'object') data = { ...leg, ...data };
				}
			} catch (e2) { /* ignore */ }
		}
		this.runtime = data && typeof data === 'object' ? data : {};
		return this.runtime;
	};

	/**
	 * @param {string} topic
	 * @param {string} [streamId] 桥侧 encoder id / web_video client_id，默认由 topic 派生
	 * @param {number} [quality] 1–100；IVG 相机 API 用 quality，web_video 代理用同名字段
	 */
	IvgTransport.prototype.cameraStreamUrl = function (topic, streamId, quality) {
		const rt = this.runtime || {};
		const sid = streamId || String(topic).replace(/\//g, '_').replace(/^_/, '') || 'cam';
		const qn = quality != null ? Number(quality) : NaN;
		const qual = !isNaN(qn) && qn >= 1 && qn <= 100 ? Math.round(qn) : 85;
		if (rt.camera_stream_path && String(rt.camera_stream_path).trim()) {
			const base = `${g.location.origin}${String(rt.camera_stream_path).trim()}`;
			const q = new URLSearchParams({ topic: String(topic), stream_id: sid });
			q.set('quality', String(qual));
			return `${base}?${q.toString()}`;
		}
		const proxy = (rt.web_video_proxy_prefix && String(rt.web_video_proxy_prefix).trim()) || '/api/ivg/proxy/web-video';
		const pre = proxy.endsWith('/') ? proxy.slice(0, -1) : proxy;
		const parts = ['topic=' + encodeTopicQueryValue(topic), 'type=mjpeg', 'client_id=' + encodeURIComponent(sid)];
		parts.push('quality=' + encodeURIComponent(String(qual)));
		return `${g.location.origin}${pre}/stream?${parts.join('&')}`;
	};

	IvgTransport.prototype.isConnected = function () {
		return !!(this.ctrl && this.ctrl.readyState === WebSocket.OPEN);
	};

	IvgTransport.prototype.onRosJson = function (topicFilter, fn) {
		this._rosHandlers.push({ topic: topicFilter, fn: fn });
	};

	IvgTransport.prototype.clearRosHandlers = function () {
		this._rosHandlers.length = 0;
	};

	IvgTransport.prototype.onBinary = function (fn) {
		this._binaryHandlers.push(fn);
	};

	IvgTransport.prototype.clearBinaryHandlers = function () {
		this._binaryHandlers.length = 0;
	};

	/** 非 ros_json / service_result 的控制面 JSON（如 tf_snapshot、错误提示） */
	IvgTransport.prototype.onControlJson = function (fn) {
		this._controlHandlers.push(fn);
	};

	IvgTransport.prototype.clearControlJsonHandlers = function () {
		this._controlHandlers.length = 0;
	};

	function canonicalRosTopic(t) {
		const s = String(t || '').trim();
		if (!s) return '';
		return s.startsWith('/') ? s : `/${s}`;
	}

	IvgTransport.prototype._dispatchRos = function (topic, payload) {
		const ct = canonicalRosTopic(topic);
		for (let i = 0; i < this._rosHandlers.length; i++) {
			const h = this._rosHandlers[i];
			const match =
				h.topic == null ||
				h.topic === topic ||
				canonicalRosTopic(h.topic) === ct;
			if (match) {
				try {
					h.fn(payload, topic);
				} catch (e) { /* ignore */ }
			}
		}
	};

	IvgTransport.prototype._dispatchBin = function (buf) {
		for (let i = 0; i < this._binaryHandlers.length; i++) {
			try {
				this._binaryHandlers[i](buf);
			} catch (e) { /* ignore */ }
		}
	};

	IvgTransport.prototype._appendRosbridgeFragment = function (o) {
		const id = o.id;
		const num = o.num;
		const total = o.total;
		const data = o.data;
		if (!id || typeof num !== 'number' || typeof total !== 'number' || typeof data !== 'string') return;
		const t = Math.floor(total);
		if (!this._fragmentBuf.has(id)) {
			this._fragmentBuf.set(id, { fragments: [], received: 0, total: t });
		}
		const buf = this._fragmentBuf.get(id);
		if (!buf) return;
		if (num < t && buf.fragments[num] === undefined) {
			buf.fragments[num] = data;
			buf.received++;
		}
		if (buf.received === t) {
			const joined = buf.fragments.join('');
			this._fragmentBuf.delete(id);
			let full;
			try {
				full = JSON.parse(joined);
			} catch (e) {
				return;
			}
			if (full && typeof full === 'object') this._dispatchControlJsonObject(full);
		}
	};

	/** 已解析的一条控制面 JSON（整帧或分片重组后） */
	IvgTransport.prototype._dispatchControlJsonObject = function (o) {
		if (!o || typeof o !== 'object') return;
		if (o.op === 'fragment') {
			this._appendRosbridgeFragment(o);
			return;
		}
		if (this._nativeIvgWire()) {
			if (o.op === 'ros_json' && o.topic) {
				this._dispatchRos(o.topic, o.payload);
				return;
			}
			if (o.op === 'service_result') {
				const id = o.id;
				const p = id != null ? this._svcPending.get(String(id)) : null;
				if (p) {
					this._svcPending.delete(String(id));
					if (o.ok) p.resolve(o.response);
					else p.reject(new Error(o.error || 'service_failed'));
				}
				return;
			}
		} else {
			if (o.op === 'publish' && o.topic) {
				this._dispatchRos(o.topic, o.msg);
				return;
			}
			if (o.op === 'service_response') {
				const id = o.id;
				const p = id != null ? this._svcPending.get(String(id)) : null;
				if (p) {
					this._svcPending.delete(String(id));
					if (o.result === true) p.resolve(o.values != null ? o.values : {});
					else p.reject(new Error(o.values != null ? String(o.values) : 'service_failed'));
				}
				return;
			}
		}
		this._dispatchCtrlJson(o);
	};

	IvgTransport.prototype._handleControlMessage = function (raw) {
		let o;
		try {
			o = JSON.parse(raw);
		} catch (e) {
			return;
		}
		this._dispatchControlJsonObject(o);
	};

	IvgTransport.prototype._dispatchCtrlJson = function (o) {
		for (let i = 0; i < this._controlHandlers.length; i++) {
			try {
				this._controlHandlers[i](o);
			} catch (e) { /* ignore */ }
		}
	};

	IvgTransport.prototype.connectControl = function () {
		const self = this;
		const url = this._controlWebSocketUrl();
		return new Promise(function (resolve, reject) {
			let settled = false;
			function finish(fn) {
				if (settled) return;
				settled = true;
				fn();
			}
			try {
				const ws = new WebSocket(url);
				self.ctrl = ws;
				ws.onopen = function () {
					finish(function () {
						resolve();
					});
				};
				ws.onerror = function () {
					finish(function () {
						reject(new Error('ws_error'));
					});
				};
				ws.onmessage = function (ev) {
					self._handleControlMessage(ev.data);
				};
				ws.onclose = function () {
					self._topicSubs.clear();
					self._fragmentBuf.clear();
					finish(function () {
						reject(new Error('ws_closed_before_open'));
					});
				};
			} catch (e) {
				reject(e);
			}
		});
	};

	IvgTransport.prototype.connectBinary = function () {
		const rt = this.runtime || {};
		if (!rt.ivg_ws_binary) {
			if (!rt.ivg_ws_control) {
				return Promise.resolve();
			}
		}
		const url = rt.ivg_ws_binary || `${g.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${g.location.host}/ws/ivg/binary`;
		const self = this;
		return new Promise(function (resolve, reject) {
			try {
				const ws = new WebSocket(url);
				ws.binaryType = 'arraybuffer';
				self.bin = ws;
				ws.onopen = function () {
					resolve();
				};
				ws.onerror = function () {
					reject(new Error('ws_bin_error'));
				};
				ws.onmessage = function (ev) {
					const buf = ev.data instanceof ArrayBuffer ? new Uint8Array(ev.data) : new Uint8Array();
					self._dispatchBin(buf);
				};
			} catch (e) {
				reject(e);
			}
		});
	};

	IvgTransport.prototype.close = function () {
		try {
			if (this.ctrl) this.ctrl.close();
		} catch (e) { /* ignore */ }
		try {
			if (this.bin) this.bin.close();
		} catch (e) { /* ignore */ }
		this.ctrl = null;
		this.bin = null;
		this._topicSubs.clear();
		this.clearRosHandlers();
		this.clearBinaryHandlers();
		this.clearControlJsonHandlers();
		this._svcPending.clear();
		this._fragmentBuf.clear();
		this._rosbridgeSubIds.clear();
	};

	IvgTransport.prototype.subscribe = function (spec) {
		const topic = String(spec.topic || '');
		const msgType = String(spec.msgType || spec.msg_type || '');
		if (!topic || !msgType || !this.ctrl || this.ctrl.readyState !== WebSocket.OPEN) return false;
		let payload;
		if (this._nativeIvgWire()) {
			payload = {
				op: 'subscribe',
				topic: topic,
				msg_type: msgType,
				max_hz: spec.maxHz != null ? Number(spec.maxHz) : 30,
				max_points: spec.maxPoints != null ? Number(spec.maxPoints) : 16000
			};
		} else {
			const maxHz = spec.maxHz != null ? Number(spec.maxHz) : 30;
			const throttle = maxHz > 0 ? Math.max(0, Math.round(1000 / maxHz)) : 0;
			const subId = `ivg_sub:${topic}:${uuid()}`;
			this._rosbridgeSubIds.set(topic, subId);
			payload = {
				op: 'subscribe',
				id: subId,
				topic: topic,
				type: msgType,
				throttle_rate: throttle,
				queue_length: 10
			};
		}
		this.ctrl.send(JSON.stringify(payload));
		this._topicSubs.add(topic);
		return true;
	};

	IvgTransport.prototype.unsubscribe = function (topic) {
		const t = String(topic || '');
		if (!t || !this.ctrl || this.ctrl.readyState !== WebSocket.OPEN) return;
		if (this._nativeIvgWire()) {
			this.ctrl.send(JSON.stringify({ op: 'unsubscribe', topic: t }));
		} else {
			const sid = this._rosbridgeSubIds.get(t);
			this._rosbridgeSubIds.delete(t);
			this.ctrl.send(
				JSON.stringify(sid ? { op: 'unsubscribe', id: sid, topic: t } : { op: 'unsubscribe', topic: t })
			);
		}
		this._topicSubs.delete(t);
	};

	IvgTransport.prototype.unsubscribeAll = function () {
		const self = this;
		this._topicSubs.forEach(function (t) {
			self.unsubscribe(t);
		});
		this._topicSubs.clear();
	};

	IvgTransport.prototype.callService = function (spec) {
		const self = this;
		const id = spec.id || uuid();
		const service = String(spec.service || '');
		const typeStr = String(spec.type || spec.srvType || '');
		const request = spec.request && typeof spec.request === 'object' ? spec.request : {};
		return new Promise(function (resolve, reject) {
			if (!self.ctrl || self.ctrl.readyState !== WebSocket.OPEN) {
				reject(new Error('not_connected'));
				return;
			}
			self._svcPending.set(String(id), { resolve: resolve, reject: reject });
			const wire = self._nativeIvgWire()
				? { op: 'call_service', id: id, service: service, type: typeStr, request: request }
				: { op: 'call_service', id: id, service: service, type: typeStr, args: request };
			self.ctrl.send(JSON.stringify(wire));
			setTimeout(function () {
				if (self._svcPending.has(String(id))) {
					self._svcPending.delete(String(id));
					reject(new Error('service_timeout'));
				}
			}, 60000);
		});
	};

	/**
	 * 解析网关下发的浏览器专用二进制帧（非内部 IVG 信封）。
	 * @returns {{kind:'points',count:number,floatOffset:number,u8:Uint8Array}|{kind:'jpeg',jpeg:Uint8Array}|null}
	 */
	IvgTransport.prototype.parseBrowserBinary = function (u8) {
		if (!u8 || u8.byteLength < 8) return null;
		const tag = (u8[0] << 24) | (u8[1] << 16) | (u8[2] << 8) | u8[3];
		if (tag === TAG_VPC) {
			const n = new DataView(u8.buffer, u8.byteOffset + 4, 4).getUint32(0, true);
			const need = 8 + n * 12;
			if (u8.byteLength < need || n > 5000000) return null;
			return { kind: 'points', count: n, floatOffset: 8, u8: u8 };
		}
		if (tag === TAG_VJP) {
			const jlen = new DataView(u8.buffer, u8.byteOffset + 4, 4).getUint32(0, true);
			if (u8.byteLength < 8 + jlen || jlen > 50 * 1024 * 1024) return null;
			return { kind: 'jpeg', jpeg: u8.subarray(8, 8 + jlen) };
		}
		return null;
	};

	g.ivgTransport = new IvgTransport();
})(typeof window !== 'undefined' ? window : globalThis);
