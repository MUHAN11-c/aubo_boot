// entities/connection_store.js — 连接状态管理 (BehaviorSubject 模式)
//
// 单一数据源: connectionStatus / activeBridge / bridgeMode / bridgeStats / reconnectAttempts
// BehaviorSubject: 新订阅者立即收到当前状态 (参考 nanostores atom.subscribe)
//
// 用法:
//   import { connectionStore } from '../entities/connection_store.js';
//   var unsub = connectionStore.subscribe(function(state) { ... });
//   connectionStore.setConnectionStatus('connected');

var g = globalThis;

function ConnectionStore() {
    this._events = new EventTarget();
    this._state = {
        connectionStatus: 'disconnected',
        activeBridge: null,
        bridgeMode: 'rosbridge',  // 临时简化: 固定 rosbridge
        bridgeStats: {},
        reconnectAttempts: 0,
        connectedAt: null,
        lastError: null,
    };
}

// EventTarget 组合 (DOM EventTarget 不支持 .call() 继承)
ConnectionStore.prototype.addEventListener = function(type, fn) {
    this._events.addEventListener(type, fn);
};
ConnectionStore.prototype.removeEventListener = function(type, fn) {
    this._events.removeEventListener(type, fn);
};
ConnectionStore.prototype.dispatchEvent = function(event) {
    this._events.dispatchEvent(event);
};

ConnectionStore.prototype.getState = function() {
    var s = this._state;
    return {
        connectionStatus: s.connectionStatus,
        activeBridge: s.activeBridge,
        bridgeMode: s.bridgeMode,
        bridgeStats: s.bridgeStats,
        reconnectAttempts: s.reconnectAttempts,
        connectedAt: s.connectedAt,
        lastError: s.lastError,
    };
};

ConnectionStore.prototype.subscribe = function(fn) {
    try { fn(this.getState()); } catch (_) {}
    var self = this;
    var handler = function(e) { try { fn(e.detail); } catch (_) {} };
    this.addEventListener('connectionchange', handler);
    return function() { self.removeEventListener('connectionchange', handler); };
};

ConnectionStore.prototype.setConnectionStatus = function(status, error) {
    if (this._state.connectionStatus === status && !error) return;
    this._state.connectionStatus = status;
    this._state.lastError = error || null;
    if (status === 'connected') this._state.connectedAt = Date.now();
    this._emit();
};

ConnectionStore.prototype.setActiveBridge = function(bridgeId) {
    if (this._state.activeBridge === bridgeId) return;
    this._state.activeBridge = bridgeId;
    this._emit();
};

ConnectionStore.prototype.setBridgeMode = function(mode) {
    if (this._state.bridgeMode === mode) return;
    this._state.bridgeMode = mode;
    this._emit();
};

ConnectionStore.prototype.setBridgeStats = function(stats) {
    this._state.bridgeStats = stats;
    this.dispatchEvent(new CustomEvent('statsupdate', { detail: this.getState() }));
};

ConnectionStore.prototype.setReconnectAttempts = function(n) {
    if (this._state.reconnectAttempts === n) return;
    this._state.reconnectAttempts = n;
    this._emit();
};

ConnectionStore.prototype._emit = function() {
    this.dispatchEvent(new CustomEvent('connectionchange', { detail: this.getState() }));
};

var connectionStore = new ConnectionStore();
g.__connectionStore = connectionStore;

export { ConnectionStore, connectionStore };
