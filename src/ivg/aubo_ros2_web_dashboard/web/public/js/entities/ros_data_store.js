// entities/ros_data_store.js — ROS 话题数据缓存 (SWR + BehaviorSubject)
//
// 每个 topic 缓存最新值 (SWR stale-while-revalidate)
// 新订阅者立即收到缓存值 (BehaviorSubject)
// 高频节流: /joint_states 50ms debounce (合并而非丢弃)
// 大消息不缓存: PointCloud2/Image
//
// 参考: nanostores atom + SWR 缓存策略

var g = globalThis;

var TTL_MS = 30000;
var MAX_CACHE = 200;
var DEBOUNCE_MS = 50;
var HIGH_FREQ_TOPICS = new Set(['/joint_states', '/tf', '/dynamic_joint_states']);
var NO_CACHE_TOPICS = new Set([
    '/camera/depth/color/points', '/camera/depth/points',
    '/camera/depth_registered/points', '/camera/color/image_raw', '/camera/depth/image_raw',
]);

function RosDataStore() {
    this._events = new EventTarget();
    this._cache = new Map();
    this._accessOrder = [];
    this._debounceTimers = new Map();
    this._subscribers = new Map();
}

// EventTarget 组合 (DOM EventTarget 不支持 .call() 继承)
RosDataStore.prototype.addEventListener = function(type, fn) {
    this._events.addEventListener(type, fn);
};
RosDataStore.prototype.removeEventListener = function(type, fn) {
    this._events.removeEventListener(type, fn);
};
RosDataStore.prototype.dispatchEvent = function(event) {
    this._events.dispatchEvent(event);
};

RosDataStore.prototype.get = function(topic) {
    var entry = this._cache.get(topic);
    if (!entry) return undefined;
    this._touchAccessOrder(topic);
    if (Date.now() - entry.ts > TTL_MS) entry.stale = true;
    return { data: entry.data, ts: entry.ts, bridge: entry.bridge, stale: entry.stale };
};

RosDataStore.prototype.subscribe = function(topic, fn) {
    var cached = this.get(topic);
    if (cached) {
        try { fn(cached.data); } catch (_) {}
    }
    if (!this._subscribers.has(topic)) this._subscribers.set(topic, new Set());
    this._subscribers.get(topic).add(fn);
    var self = this;
    return function() {
        var subs = self._subscribers.get(topic);
        if (subs) {
            subs.delete(fn);
            if (subs.size === 0) self._subscribers.delete(topic);
        }
    };
};

RosDataStore.prototype.set = function(topic, data, bridge) {
    if (NO_CACHE_TOPICS.has(topic)) {
        this._notifySubs(topic, data);
        return;
    }
    if (HIGH_FREQ_TOPICS.has(topic)) {
        if (this._debounceTimers.has(topic)) {
            this._debounceTimers.get(topic)._pending = { data: data, bridge: bridge };
            return;
        }
        var self = this;
        var timerInfo = { _pending: null };
        this._debounceTimers.set(topic, setTimeout(function() {
            var d = timerInfo._pending || { data: data, bridge: bridge };
            self._debounceTimers.delete(topic);
            self._doSet(topic, d.data, d.bridge);
        }, DEBOUNCE_MS));
        return;
    }
    this._doSet(topic, data, bridge);
};

RosDataStore.prototype._doSet = function(topic, data, bridge) {
    if (!this._cache.has(topic) && this._cache.size >= MAX_CACHE) {
        var oldest = this._accessOrder.shift();
        if (oldest) this._cache.delete(oldest);
    }
    this._cache.set(topic, { data: data, ts: Date.now(), bridge: bridge, stale: false });
    this._touchAccessOrder(topic);
    this._notifySubs(topic, data);
};

RosDataStore.prototype._notifySubs = function(topic, data) {
    var subs = this._subscribers.get(topic);
    if (subs) {
        subs.forEach(function(fn) {
            try { fn(data); } catch (_) {}
        });
    }
    this.dispatchEvent(new CustomEvent('data:' + topic, { detail: data }));
};

RosDataStore.prototype._touchAccessOrder = function(topic) {
    var idx = this._accessOrder.indexOf(topic);
    if (idx > -1) this._accessOrder.splice(idx, 1);
    this._accessOrder.push(topic);
};

var rosDataStore = new RosDataStore();
g.__rosDataStore = rosDataStore;

export { RosDataStore, rosDataStore };
