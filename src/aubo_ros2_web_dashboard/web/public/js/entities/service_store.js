// entities/service_store.js — 服务调用状态机
//
// 状态机: IDLE → STARTING → IN_PROGRESS → COMPLETED / FAILED
// 5s 心跳防超时
//
// 用法:
//   import { serviceStore } from '../entities/service_store.js';
//   serviceStore.setExecutor(async (name, type, req, timeout) => { ... });
//   var result = await serviceStore.callService('/svc', 'srv/Type', {});

var g = globalThis;
var HEARTBEAT_MS = 5000;
var MAX_HISTORY = 50;

function ServiceStore() {
    this._events = new EventTarget();
    this._lastResults = new Map();
    this._callHistory = [];
    this._nextCallId = 1;
    this._executor = null;
}

// EventTarget 组合 (DOM EventTarget 不支持 .call() 继承)
ServiceStore.prototype.addEventListener = function(type, fn) {
    this._events.addEventListener(type, fn);
};
ServiceStore.prototype.removeEventListener = function(type, fn) {
    this._events.removeEventListener(type, fn);
};
ServiceStore.prototype.dispatchEvent = function(event) {
    this._events.dispatchEvent(event);
};

ServiceStore.prototype.setExecutor = function(fn) { this._executor = fn; };

ServiceStore.prototype.callService = async function(name, type, request, timeoutMs) {
    timeoutMs = timeoutMs || 60000;
    var callId = this._nextCallId++;
    var startTime = Date.now();

    this._emit('service:start', { callId: callId, service: name, type: type, request: request });

    var self = this;
    var progressTimer = setInterval(function() {
        self._emit('service:progress', {
            callId: callId, service: name, elapsedMs: Date.now() - startTime,
        });
    }, HEARTBEAT_MS);

    try {
        if (!this._executor) throw new Error('ServiceStore: executor not set');
        var result = await this._executor(name, type, request, timeoutMs);
        clearInterval(progressTimer);

        var durationMs = Date.now() - startTime;
        this._lastResults.set(name, { data: result, ts: Date.now() });
        this._callHistory.push({
            service: name, status: 'completed', durationMs: durationMs,
            bridge: (result && result._bridge) ? result._bridge : 'unknown',
        });
        while (this._callHistory.length > MAX_HISTORY) this._callHistory.shift();

        this._emit('service:complete', { callId: callId, service: name, result: result, durationMs: durationMs });
        return result;
    } catch (e) {
        clearInterval(progressTimer);
        var errDurationMs = Date.now() - startTime;

        this._callHistory.push({
            service: name, status: 'failed', durationMs: errDurationMs,
            bridge: 'unknown', error: e.message || String(e),
        });
        while (this._callHistory.length > MAX_HISTORY) this._callHistory.shift();

        this._emit('service:error', { callId: callId, service: name, error: e.message || String(e), durationMs: errDurationMs });
        throw e;
    }
};

ServiceStore.prototype.getLastResult = function(service) {
    return this._lastResults.get(service);
};

ServiceStore.prototype.getCallHistory = function() {
    return this._callHistory.slice(-MAX_HISTORY);
};

ServiceStore.prototype._emit = function(eventType, detail) {
    this.dispatchEvent(new CustomEvent(eventType, { detail: detail }));
};

var serviceStore = new ServiceStore();
g.__serviceStore = serviceStore;

export { ServiceStore, serviceStore };
