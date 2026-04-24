/**
 * IVG 浏览器传输（重构版）：
 * - 基于 RobotWebTools 官方用法：ROSLIB.Ros / ROSLIB.Topic / ROSLIB.Service
 * - 对外保持 JSON 通道：isConnected / subscribe / callService 等（无独立二进制 WS）
 * - 不改页面样式与 DOM 结构，只替换通信逻辑
 */
import * as ROSLIB from 'roslib';
import { loadIvgRuntime } from './core/runtime_provider.js';
import { rosbridgeWebSocketUrlFromRuntime } from './ivg_runtime.js';

const g = globalThis;

function canonicalRosTopic(t) {
	const s = String(t || '').trim();
	if (!s) return '';
	return s.startsWith('/') ? s : `/${s}`;
}

function encodeTopicQueryValue(topic) {
	return String(topic)
		.split('/')
		.map(seg => encodeURIComponent(seg))
		.join('/');
}

function IvgTransport() {
	this.runtime = null;
	this.ros = null;
	this._topicSubs = new Map(); // topic -> ROSLIB.Topic
	this._rosHandlers = [];
	this._controlHandlers = [];
	this._connectPromise = null;
}

IvgTransport.prototype._rosbridgeWebSocketUrl = function () {
	return rosbridgeWebSocketUrlFromRuntime(this.runtime);
};

IvgTransport.prototype.loadRuntime = async function () {
	const data = await loadIvgRuntime();
	this.runtime = data && typeof data === 'object' ? data : {};
	return this.runtime;
};

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

IvgTransport.prototype.cameraSnapshotUrl = function (topic, streamId, quality) {
	const rt = this.runtime || {};
	const sid = streamId || String(topic).replace(/\//g, '_').replace(/^_/, '') || 'cam';
	const qn = quality != null ? Number(quality) : NaN;
	const qual = !isNaN(qn) && qn >= 1 && qn <= 100 ? Math.round(qn) : 85;
	if (rt.camera_stream_path && String(rt.camera_stream_path).trim()) {
		let rel = String(rt.camera_stream_path).trim();
		if (/\/stream(\?|$)/i.test(rel)) rel = rel.replace(/\/stream(\?|$)/i, '/snapshot$1');
		const base = `${g.location.origin}${rel.startsWith('/') ? rel : `/${rel}`}`;
		const q = new URLSearchParams({ topic: String(topic), stream_id: sid });
		q.set('quality', String(qual));
		return `${base}?${q.toString()}`;
	}
	const proxy = (rt.web_video_proxy_prefix && String(rt.web_video_proxy_prefix).trim()) || '/api/ivg/proxy/web-video';
	const pre = proxy.endsWith('/') ? proxy.slice(0, -1) : proxy;
	const parts = ['topic=' + encodeTopicQueryValue(topic), 'type=jpeg', 'client_id=' + encodeURIComponent(sid)];
	parts.push('quality=' + encodeURIComponent(String(qual)));
	return `${g.location.origin}${pre}/snapshot?${parts.join('&')}`;
};

IvgTransport.prototype.isConnected = function () {
	return !!(this.ros && this.ros.isConnected);
};

IvgTransport.prototype.onRosJson = function (topicFilter, fn) {
	this._rosHandlers.push({ topic: topicFilter, fn });
};

IvgTransport.prototype.clearRosHandlers = function () {
	this._rosHandlers.length = 0;
};

IvgTransport.prototype.onControlJson = function (fn) {
	this._controlHandlers.push(fn);
};

IvgTransport.prototype.clearControlJsonHandlers = function () {
	this._controlHandlers.length = 0;
};

IvgTransport.prototype._dispatchRos = function (topic, payload) {
	const ct = canonicalRosTopic(topic);
	for (let i = 0; i < this._rosHandlers.length; i++) {
		const h = this._rosHandlers[i];
		const match = h.topic == null || h.topic === topic || canonicalRosTopic(h.topic) === ct;
		if (match) {
			try {
				h.fn(payload, topic);
			} catch (e) {
				/* ignore */
			}
		}
	}
};

IvgTransport.prototype._dispatchControlJson = function (obj) {
	for (let i = 0; i < this._controlHandlers.length; i++) {
		try {
			this._controlHandlers[i](obj);
		} catch (e) {
			/* ignore */
		}
	}
};

IvgTransport.prototype.connectControl = function () {
	if (this._connectPromise) return this._connectPromise;
	const self = this;
	const url = this._rosbridgeWebSocketUrl();
	this._connectPromise = new Promise(function (resolve, reject) {
		let settled = false;
		function finishOk() {
			if (settled) return;
			settled = true;
			resolve();
		}
		function finishErr(err) {
			if (settled) return;
			settled = true;
			reject(err);
		}

		try {
			if (self.ros) {
				try { self.ros.close(); } catch (e0) { /* ignore */ }
				self.ros = null;
			}
			const ros = new ROSLIB.Ros({ url });
			self.ros = ros;

			ros.on('connection', function () {
				if (self.ros !== ros) return;
				finishOk();
			});
			ros.on('error', function (err) {
				if (self.ros !== ros) return;
				self._dispatchControlJson({ op: 'error', message: err ? String(err) : 'ros_error' });
				if (!settled) finishErr(new Error('ros_error'));
			});
			ros.on('close', function () {
				if (self.ros !== ros) return;
				self.unsubscribeAll();
				self._dispatchControlJson({ op: 'close', message: 'ros_closed' });
				if (!settled) finishErr(new Error('ros_closed_before_open'));
			});
		} catch (e) {
			finishErr(e);
		}
	}).finally(() => {
		self._connectPromise = null;
	});
	return this._connectPromise;
};

IvgTransport.prototype.close = function () {
	this.unsubscribeAll();
	this._connectPromise = null;
	try {
		if (this.ros) this.ros.close();
	} catch (e) {
		/* ignore */
	}
	this.ros = null;
	this.clearRosHandlers();
	this.clearControlJsonHandlers();
};

IvgTransport.prototype.subscribe = function (spec) {
	const topicName = canonicalRosTopic(spec.topic);
	const msgType = String(spec.msgType || spec.msg_type || '').trim();
	if (!topicName || !msgType || !this.ros) return false;

	const existing = this._topicSubs.get(topicName);
	if (existing) {
		try {
			existing.unsubscribe();
		} catch (e) {
			/* ignore */
		}
		this._topicSubs.delete(topicName);
	}

	const topic = new ROSLIB.Topic({
		ros: this.ros,
		name: topicName,
		messageType: msgType,
		throttle_rate: spec.maxHz ? Math.max(0, Math.round(1000 / Number(spec.maxHz))) : 0,
		queue_length: 10
	});

	topic.subscribe(msg => {
		this._dispatchRos(topicName, msg);
	});

	this._topicSubs.set(topicName, topic);
	return true;
};

IvgTransport.prototype.unsubscribe = function (topic) {
	const topicName = canonicalRosTopic(topic);
	const sub = this._topicSubs.get(topicName);
	if (!sub) return;
	try {
		sub.unsubscribe();
	} catch (e) {
		/* ignore */
	}
	this._topicSubs.delete(topicName);
};

IvgTransport.prototype.unsubscribeAll = function () {
	this._topicSubs.forEach(sub => {
		try {
			sub.unsubscribe();
		} catch (e) {
			/* ignore */
		}
	});
	this._topicSubs.clear();
};

IvgTransport.prototype.callService = function (spec) {
	if (!this.ros) return Promise.reject(new Error('not_connected'));
	const serviceName = canonicalRosTopic(spec.service);
	const serviceType = String(spec.type || spec.srvType || '').trim();
	const requestObj = spec.request && typeof spec.request === 'object' ? spec.request : {};
	const timeoutMs = Number(spec.timeoutMs) > 0 ? Number(spec.timeoutMs) : 60000;
	if (!serviceName || !serviceType) return Promise.reject(new Error('invalid_service_spec'));

	return new Promise((resolve, reject) => {
		let settled = false;
		let timer = null;
		function done(fn) {
			if (settled) return;
			settled = true;
			if (timer) {
				clearTimeout(timer);
				timer = null;
			}
			fn();
		}
		try {
			const service = new ROSLIB.Service({
				ros: this.ros,
				name: serviceName,
				serviceType
			});
			const req =
				typeof ROSLIB.ServiceRequest === 'function'
					? new ROSLIB.ServiceRequest(requestObj)
					: requestObj;
			timer = setTimeout(() => {
				done(() => reject(new Error('service_timeout')));
			}, timeoutMs);
			service.callService(
				req,
				result => done(() => resolve(result || {})),
				err => done(() => reject(new Error(err ? String(err) : 'service_failed')))
			);
		} catch (e) {
			done(() => reject(e));
		}
	});
};

const ivgTransport = new IvgTransport();
g.ivgTransport = ivgTransport;

export { IvgTransport, ivgTransport };
