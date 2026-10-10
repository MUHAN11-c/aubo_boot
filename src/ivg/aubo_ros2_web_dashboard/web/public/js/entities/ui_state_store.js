// entities/ui_state_store.js — UI 状态持久化 (localStorage 同步)
//
// 用法:
//   import { uiStateStore } from '../entities/ui_state_store.js';
//   uiStateStore.set('bridge_mode', 'foxglove');
//   var mode = uiStateStore.get('bridge_mode');

var PREFIX = 'ivg_ui_';

function UIStateStore() {
    this._events = new EventTarget();
}

// EventTarget 组合 (DOM EventTarget 不支持 .call() 继承)
UIStateStore.prototype.addEventListener = function(type, fn) {
    this._events.addEventListener(type, fn);
};
UIStateStore.prototype.removeEventListener = function(type, fn) {
    this._events.removeEventListener(type, fn);
};
UIStateStore.prototype.dispatchEvent = function(event) {
    this._events.dispatchEvent(event);
};

UIStateStore.prototype.get = function(key) {
    try {
        var raw = localStorage.getItem(PREFIX + key);
        if (raw === null) return undefined;
        return JSON.parse(raw);
    } catch (_) {
        return undefined;
    }
};

UIStateStore.prototype.set = function(key, val) {
    try {
        localStorage.setItem(PREFIX + key, JSON.stringify(val));
    } catch (_) {}
    this.dispatchEvent(new CustomEvent('uisettings:change', { detail: { key: key, val: val } }));
};

UIStateStore.prototype.remove = function(key) {
    localStorage.removeItem(PREFIX + key);
    this.dispatchEvent(new CustomEvent('uisettings:change', { detail: { key: key, val: undefined } }));
};

var uiStateStore = new UIStateStore();
globalThis.__uiStateStore = uiStateStore;

export { UIStateStore, uiStateStore };
