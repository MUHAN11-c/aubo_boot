// ivg_transport — ROSLIB.Ros 封装：连接管理、话题订阅、服务调用、摄像头 URL
// 链路: 浏览器 ←→ IvgTransport ←→ rosbridge (WebSocket) / web_video (HTTP)
// 全局单例: ivgTransport
import * as ROSLIB from 'roslib';
import { loadIvgRuntime } from './core/runtime_provider.js';
import { rosbridgeWebSocketUrlFromRuntime } from './ivg_runtime.js';
import { canonicalRosTopic, encodeTopicQueryValue } from './core/utils.js';
import { logBus } from './core/log-bus.js';

const g = globalThis;

// ── 工具 ─────────────────────────────────────────────────────────────────────

// canonicalRosTopic / encodeTopicQueryValue 已从 core/utils.js 导入

// ── 传输层 ──────────────────────────────────────────────────────────────────

function IvgTransport() {
    this.runtime = null;          // 运行时配置 (来自 /api/v1/runtime)
    this.ros = null;              // ROSLIB.Ros 实例
    this._topicSubs = new Map();  // topic → ROSLIB.Topic
    this._topicSpecs = new Map(); // topic → msgType (去重用)
    this._rosHandlers = new Map();    // owner → Set<{topic, fn}> 话题消息处理器
    this._controlHandlers = new Map(); // owner → Set<Function> 控制面 JSON 处理器
    this._connectPromise = null;  // 去重: 同一时间只有一个连接尝试
}

// ── 摄像头 URL 构建 ─────────────────────────────────────────────────────────

IvgTransport.prototype.cameraStreamUrl = function (topic, streamId, quality) {
    const rt = this.runtime || {};
    const sid = streamId || String(topic).replace(/\//g, '_').replace(/^_/, '') || 'cam';
    const qn = quality != null ? Number(quality) : NaN;
    const qual = !isNaN(qn) && qn >= 1 && qn <= 100 ? Math.round(qn) : 85;

    if (rt.camera_stream_path && String(rt.camera_stream_path).trim()) {
        const base = `${g.location.origin}${String(rt.camera_stream_path).trim()}`;
        const q = new URLSearchParams({ topic: String(topic), stream_id: sid });
        q.set('quality', String(qual));
        return `${base}?${q}`;
    }
    // 默认走网关代理
    const proxy = (rt.web_video_proxy_prefix && String(rt.web_video_proxy_prefix).trim())
        || '/api/ivg/proxy/web-video';
    const pre = proxy.endsWith('/') ? proxy.slice(0, -1) : proxy;
    const parts = [
        'topic=' + encodeTopicQueryValue(topic),
        'type=mjpeg',
        'client_id=' + encodeURIComponent(sid),
        'quality=' + encodeURIComponent(String(qual)),
    ];
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
        return `${base}?${q}`;
    }
    const proxy = (rt.web_video_proxy_prefix && String(rt.web_video_proxy_prefix).trim())
        || '/api/ivg/proxy/web-video';
    const pre = proxy.endsWith('/') ? proxy.slice(0, -1) : proxy;
    const parts = [
        'topic=' + encodeTopicQueryValue(topic),
        'type=jpeg',
        'client_id=' + encodeURIComponent(sid),
        'quality=' + encodeURIComponent(String(qual)),
    ];
    return `${g.location.origin}${pre}/snapshot?${parts.join('&')}`;
};

// ── 连接管理 ────────────────────────────────────────────────────────────────

IvgTransport.prototype.loadRuntime = async function () {
    const data = await loadIvgRuntime();
    this.runtime = data && typeof data === 'object' ? data : {};
    return this.runtime;
};

IvgTransport.prototype.isConnected = function () {
    return !!(this.ros && this.ros.isConnected);
};

IvgTransport.prototype.connectControl = function () {
    if (this._connectPromise) return this._connectPromise;

    const self = this;
    const url = rosbridgeWebSocketUrlFromRuntime(this.runtime);

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
                // 先取消所有订阅，确保 unsubscribe 消息在 socket 关闭前送达 rosbridge
                self.unsubscribeAll();
                try { self.ros.close(); } catch (e0) { /* ignore */ }
                self.ros = null;
            }
            const ros = new ROSLIB.Ros({ url: url });
            self.ros = ros;

            ros.on('connection', function () {
                if (self.ros !== ros) return;
                self._dispatchControlJson({ op: 'connection' });
                finishOk();
            });
            ros.on('error', function (err) {
                if (self.ros !== ros) return;
                self._dispatchControlJson({
                    op: 'error', message: err ? String(err) : 'ros_error'
                });
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
    try { if (this.ros) this.ros.close(); } catch (e) { /* ignore */ }
    this.ros = null;
    // 处理器不清除：各消费者（状态栏等）的处理器所有权独立于连接生命周期，
    // 通过 clearRosHandlersByOwner / clearControlHandlersByOwner 各自管理喵~
};

// ── 话题订阅 ────────────────────────────────────────────────────────────────

IvgTransport.prototype.subscribe = function (spec) {
    const topicName = canonicalRosTopic(spec.topic);
    const msgType = String(spec.msgType || spec.msg_type || '').trim();
    if (!topicName || !msgType || !this.ros) return false;

    // 已订阅相同 topic + msgType，跳过
    if (this._topicSpecs.get(topicName) === msgType) return true;

    // 覆盖旧订阅
    const existing = this._topicSubs.get(topicName);
    if (existing) {
        try { existing.unsubscribe(); } catch (e) { /* ignore */ }
        this._topicSubs.delete(topicName);
    }

    const topic = new ROSLIB.Topic({
        ros: this.ros,
        name: topicName,
        messageType: msgType,
        throttle_rate: spec.maxHz ? Math.max(0, Math.round(1000 / Number(spec.maxHz))) : 0,
        queue_length: 10,
    });

    topic.subscribe(msg => {
        this._dispatchRos(topicName, msg);
    });

    this._topicSubs.set(topicName, topic);
    this._topicSpecs.set(topicName, msgType);
	logBus.addLog('debug', 'topic', 'subscribed: ' + topicName);
    return true;
};

/** 检查 WebSocket 是否仍处于 OPEN 状态，避免在 CLOSING/CLOSED 时发送消息刷屏喵~ */
IvgTransport.prototype._isSocketOpen = function () {
    try {
        const s = this.ros && this.ros.socket;
        return s && s.readyState === 1; // WebSocket.OPEN
    } catch (_) { return false; }
};

IvgTransport.prototype.unsubscribe = function (topic) {
    const topicName = canonicalRosTopic(topic);
    const sub = this._topicSubs.get(topicName);
    if (!sub) return;
    if (this._isSocketOpen()) {
        try { sub.unsubscribe(); } catch (e) { /* ignore */ }
    }
    this._topicSubs.delete(topicName);
    this._topicSpecs.delete(topicName);
};

IvgTransport.prototype.unsubscribeAll = function () {
    const open = this._isSocketOpen();
    this._topicSubs.forEach(sub => {
        if (open) {
            try { sub.unsubscribe(); } catch (e) { /* ignore */ }
        }
    });
    this._topicSubs.clear();
    this._topicSpecs.clear();
};

// ── 服务调用 ────────────────────────────────────────────────────────────────

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
            if (timer) { clearTimeout(timer); timer = null; }
            fn();
        }

        try {
            const service = new ROSLIB.Service({ ros: this.ros, name: serviceName, serviceType });
            const req = typeof ROSLIB.ServiceRequest === 'function'
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

	// ── 话题发布 ────────────────────────────────────────────────────────────────

	IvgTransport.prototype.publish = function (spec) {
	    if (!this.ros || !this._isSocketOpen()) return Promise.reject(new Error('not_connected'));

	    const topicName = canonicalRosTopic(spec.topic);
	    const msgType = String(spec.type || spec.msgType || '').trim();
	    if (!topicName || !msgType) return Promise.reject(new Error('invalid_publish_spec'));

	    return new Promise((resolve, reject) => {
	        try {
	            const topic = new ROSLIB.Topic({
	                ros: this.ros,
	                name: topicName,
	                messageType: msgType,
	            });
	            const payload = spec.msg && typeof spec.msg === 'object' ? spec.msg : {};
	            const msg = new ROSLIB.Message(payload);
	            topic.publish(msg);
	            resolve();
	        } catch (e) {
	            reject(e);
	        }
	    });
	};

// ── 消息分发 ────────────────────────────────────────────────────────────────

IvgTransport.prototype._dispatchRos = function (topic, payload) {
    const ct = canonicalRosTopic(topic);
    for (const handlerSet of this._rosHandlers.values()) {
        for (const h of handlerSet) {
            const match = h.topic == null || h.topic === topic || canonicalRosTopic(h.topic) === ct;
            if (match) {
                try { h.fn(payload, topic); } catch (e) { /* ignore */ }
            }
        }
    }
};

IvgTransport.prototype._dispatchControlJson = function (obj) {
    for (const handlerSet of this._controlHandlers.values()) {
        for (const fn of handlerSet) {
            try { fn(obj); } catch (e) { /* ignore */ }
        }
    }
};

// ── 处理器注册 ──────────────────────────────────────────────────────────────

/** 注册话题消息处理器。owner 用于按所有者清除，默认 'default' 向后兼容喵~ */
IvgTransport.prototype.onRosJson = function (topicFilter, fn, owner) {
    const o = owner || 'default';
    if (!this._rosHandlers.has(o)) {
        this._rosHandlers.set(o, new Set());
    }
    this._rosHandlers.get(o).add({ topic: topicFilter, fn: fn });
};

IvgTransport.prototype.clearRosHandlers = function () {
    this._rosHandlers.clear();
};

/** 只清除指定所有者的 ros 处理器喵~ */
IvgTransport.prototype.clearRosHandlersByOwner = function (owner) {
    this._rosHandlers.delete(owner || 'default');
};

/** 注册控制面 JSON 处理器。owner 用于按所有者清除喵~ */
IvgTransport.prototype.onControlJson = function (fn, owner) {
    const o = owner || 'default';
    if (!this._controlHandlers.has(o)) {
        this._controlHandlers.set(o, new Set());
    }
    this._controlHandlers.get(o).add(fn);
};

IvgTransport.prototype.clearControlJsonHandlers = function () {
    this._controlHandlers.clear();
};

/** 只清除指定所有者的 control 处理器喵~ */
IvgTransport.prototype.clearControlHandlersByOwner = function (owner) {
    this._controlHandlers.delete(owner || 'default');
};

// ── 全局单例 ────────────────────────────────────────────────────────────────

const ivgTransport = new IvgTransport();
g.ivgTransport = ivgTransport;

export { IvgTransport, ivgTransport };
