// ivg_transport — 控制面门面（2026-10-10 阶段 3 换芯：ROSLIB.Ros → foxglove ros_client）
// 链路: 浏览器 ←→ IvgTransport ←→ foxglove_bridge（经网关 /ws/foxglove，CDR）
// 公开 API 与 rosbridge 时代逐面兼容（subscribe/callService/publish/onRosJson/onControlJson/
// cameraStreamUrl…），消费方（services.js / subscription_binder / status_bar / log-ros-bridge /
// debug_panel / RosManager）零改动。rosbridge 已移除，见同轮提交说明。
//
// 语义差异注记：
//   - maxHz 节流改为客户端侧（foxglove 协议无服务端 throttle 字段）
//   - 解码后的无界基本类型数组是 TypedArray（.length/索引/迭代兼容，Array.isArray 为 false）
//   - close() 只退订不清连接：ros_client 单例与 TF/URDF 岛共享
//   - publish 需图上已有该话题的 schema（vendored 桥客户端发布的固有约束）

import { foxgloveRosClient } from './transport/foxglove/ros_client.js';
import { loadIvgRuntime } from './core/runtime_provider.js';
import { canonicalRosTopic, encodeTopicQueryValue } from './core/utils.js';
import { logBus } from './core/log-bus.js';

const g = globalThis;

function IvgTransport() {
    this.runtime = null;              // 运行时配置 (来自 /api/v1/runtime)
    this._client = foxgloveRosClient();
    this._topicSubs = new Map();      // topic → { handle, lastAt, minIntervalMs }
    this._topicSpecs = new Map();     // topic → msgType (去重用)
    this._rosHandlers = new Map();    // owner → Set<{topic, fn}> 话题消息处理器
    this._controlHandlers = new Map(); // owner → Set<Function> 控制面事件处理器
    this._connectPromise = null;
    this._wired = false;

    // 兼容垫片：旧消费方读 ivgTransport.ros.isConnected / ivgTransport.ros（真 ROSLIB 实例已不存在）
    const self = this;
    this.ros = {
        get isConnected() { return self._client.isConnected; },
        socket: { get readyState() { return self._client.isConnected ? 1 : 3; } },
    };
}

// ── 摄像头 URL 构建（HTTP，与传输芯无关）────────────────────────────────────

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
    const pre = proxy.endsWith('/') ? proxy.slice(0, -1) : '';
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
    const pre = proxy.endsWith('/') ? proxy.slice(0, -1) : '';
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
    return this._client.isConnected;
};

IvgTransport.prototype._wireControlEvents = function () {
    if (this._wired) return;
    this._wired = true;
    this._client.onStateChange((s) => {
        if (s === 'connected') this._dispatchControlJson({ op: 'connection' });
        else if (s === 'reconnecting' || s === 'connecting') this._dispatchControlJson({ op: 'connecting' });
        else this._dispatchControlJson({ op: 'close', message: 'ros_closed' });
    });
};

IvgTransport.prototype.connectControl = function () {
    if (this._client.isConnected) {
        this._wireControlEvents();
        this._dispatchControlJson({ op: 'connection' });
        return Promise.resolve();
    }
    if (this._connectPromise) return this._connectPromise;
    this._wireControlEvents();
    this._connectPromise = this._client.connect()
        .finally(() => { this._connectPromise = null; });
    return this._connectPromise;
};

IvgTransport.prototype.close = function () {
    // 只退订不清连接：ros_client 单例与 TF 缓冲/URDF 岛共享生命周期。
    // 处理器不清除：各消费者通过 clearRosHandlersByOwner / clearControlHandlersByOwner 各自管理。
    this.unsubscribeAll();
    this._connectPromise = null;
};

// ── 话题订阅 ────────────────────────────────────────────────────────────────

IvgTransport.prototype.subscribe = function (spec) {
    const topicName = canonicalRosTopic(spec.topic);
    const msgType = String(spec.msgType || spec.msg_type || '').trim();
    if (!topicName || !msgType) return false;

    if (this._topicSpecs.get(topicName) === msgType) return true;

    const existing = this._topicSubs.get(topicName);
    if (existing) {
        try { existing.handle.cancel(); } catch (e) { /* ignore */ }
        this._topicSubs.delete(topicName);
    }

    const maxHz = Number(spec.maxHz) > 0 ? Number(spec.maxHz) : 0;
    const minIntervalMs = maxHz > 0 ? 1000 / maxHz : 0;
    let lastAt = 0;
    const self = this;

    const handle = this._client.subscribe(topicName, msgType, (typed) => {
        // 客户端侧节流（foxglove 协议无服务端 throttle；对齐旧 throttle_rate 语义）
        if (minIntervalMs > 0) {
            const now = Date.now();
            if (now - lastAt < minIntervalMs) return;
            lastAt = now;
        }
        self._dispatchRos(topicName, typed.data);
    }, { dispatchMode: 'latest' });

    this._topicSubs.set(topicName, { handle, minIntervalMs });
    this._topicSpecs.set(topicName, msgType);
    return true;
};

IvgTransport.prototype.unsubscribe = function (topic) {
    const topicName = canonicalRosTopic(topic);
    const sub = this._topicSubs.get(topicName);
    if (!sub) return;
    try { sub.handle.cancel(); } catch (e) { /* ignore */ }
    this._topicSubs.delete(topicName);
    this._topicSpecs.delete(topicName);
};

IvgTransport.prototype.unsubscribeAll = function () {
    this._topicSubs.forEach((sub) => {
        try { sub.handle.cancel(); } catch (e) { /* ignore */ }
    });
    this._topicSubs.clear();
    this._topicSpecs.clear();
};

// ── 服务调用 ────────────────────────────────────────────────────────────────

IvgTransport.prototype.callService = function (spec) {
    const serviceName = canonicalRosTopic(spec.service);
    const requestObj = spec.request && typeof spec.request === 'object' ? spec.request : {};
    const timeoutMs = Number(spec.timeoutMs) > 0 ? Number(spec.timeoutMs) : 60000;
    if (!serviceName) return Promise.reject(new Error('invalid_service_spec'));
    if (!this._client.isConnected) return Promise.reject(new Error('not_connected'));

    return this._client.callService(serviceName, requestObj, { timeoutMs })
        .catch((e) => {
            const msg = String(e && e.message || e);
            if (msg.includes('timeout')) throw new Error('service_timeout');
            throw new Error(msg || 'service_failed');
        });
};

// ── 话题发布 ────────────────────────────────────────────────────────────────

IvgTransport.prototype.publish = function (spec) {
    const topicName = canonicalRosTopic(spec.topic);
    if (!topicName) return Promise.reject(new Error('invalid_publish_spec'));
    if (!this._client.isConnected) return Promise.reject(new Error('not_connected'));
    try {
        const sent = this._client.publish(topicName, '', spec.msg && typeof spec.msg === 'object' ? spec.msg : {});
        if (!sent) return Promise.reject(new Error('publish_failed_no_schema'));
        return Promise.resolve();
    } catch (e) {
        return Promise.reject(e);
    }
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

/** 注册控制面事件处理器。owner 用于按所有者清除喵~ */
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
