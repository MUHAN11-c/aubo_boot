// log-bus.js — 统一日志事件总线 + IndexedDB 持久化 + BroadcastChannel 跨页面同步
// 所有模块通过此单例写入日志，替代分散的 console/__ivgLog/ros._log 喵~
//
// 用法:
//   import { logBus, LOG_LEVELS } from '../core/log-bus.js';
//   logBus.addLog('info', 'ros_manager', 'rosbridge connected');
//   logBus.addLog('info', 'service', '执行完成', {}, 'latte');
//   logBus.setLevel(LOG_LEVELS.debug);  // 调试模式
//
// 维度说明:
//   source  = 技术来源 (谁产生的): console / rosbridge / topic / service / rosout / lifecycle / ros_manager / system
//   feature = 功能归属 (属于哪个功能): latte / vision_grasp / tool_change / io_control / view3d / settings
//   source 和 feature 正交 — 同一 service 调用可能属于 latte 或 vision_grasp 喵~
//
// 频率控制:
//   topic  source → 5s 窗口内同消息折叠, 窗口到期输出 "[重复N次]" 摘要
//   rosout source → 3s 窗口按 (node, message) 去重
//   service 等 → 不节流, 全量保留
//   调试模式 → 关闭所有节流, 全量输出
//
// 参考: Grafana Loki 标签体系 (Stream labels + Structured Metadata)
//       + OpenTelemetry BatchLogRecordProcessor (缓冲-批量导出)
//       + Loki 三级采样策略 (ERROR 100% / WARN 节流 / INFO+DEBUG 折叠)

// ── 日志等级 ——————————————————————————————————————————————————————
const LOG_LEVELS = { debug: 0, info: 1, warn: 2, error: 3, silent: 4 };
const LOG_LEVEL_NAMES = ['debug', 'info', 'warn', 'error', 'silent'];

// ── 频率控制配置 ——————————————————————————————————————————————————
// windowMs: 时间窗口 (0=不限流)
// keyFields: 哪些字段参与去重 key 计算 (默认: source, feature, level, msg)
const RATE_PROFILES = {
    topic:     { windowMs: 5000,  keyFields: ['source', 'feature', 'level', 'msg'] },
    rosout:    { windowMs: 3000,  keyFields: ['source', 'feature', 'level', 'node', 'msg'] },
    // service / lifecycle / console 等不限流
};

// ── 技术来源分类 ——————————————————————————————————————————————————
const CATEGORIES = {
    console:    { color: 'var(--text-muted)', label: 'console' },
    error:      { color: 'var(--red)',        label: 'error' },
    rosbridge:  { color: 'var(--green)',      label: 'rosbridge' },
    topic:      { color: '#60a5fa',           label: 'topic' },
    service:    { color: '#c084fc',           label: 'service' },
    rosout:     { color: '#f59e0b',           label: 'rosout' },
    lifecycle:  { color: 'var(--yellow)',     label: 'lifecycle' },
    ros_manager:{ color: 'var(--accent)',     label: 'ros manager' },
    system:     { color: 'var(--orange)',     label: 'system' },
};

// ── 功能模块标签 ——————————————————————————————————————————————————
const FEATURES = {
    latte:          { color: '#a78bfa', label: '☕ 拉花' },
    vision_grasp:   { color: '#34d399', label: '👁 视觉抓取' },
    tool_change:    { color: '#fbbf24', label: '🔧 快换' },
    io_control:     { color: '#f472b6', label: '⚡ IO' },
    view3d:         { color: '#38bdf8', label: '🎨 3D' },
    settings:       { color: '#a3a3a3', label: '⚙ 设置' },
};

// ── IndexedDB 持久化 ——————————————————————————————————————————————
const DB_NAME = 'ivg_logs';
const DB_VERSION = 1;
const STORE_NAME = 'entries';
const MAX_PERSISTED = 5000;

let _db = null;
let _pendingWrites = 0;

// 批量写入缓冲
let _writeBuffer = [];
const WRITE_BATCH_SIZE = 50;      // 每 50 条或 2s flush 一次
let _writeTimer = null;
const WRITE_FLUSH_MS = 2000;

function _openDB() {
    if (_db) return Promise.resolve(_db);
    return new Promise((resolve, reject) => {
        const req = indexedDB.open(DB_NAME, DB_VERSION);
        req.onupgradeneeded = () => {
            req.result.createObjectStore(STORE_NAME, { keyPath: 'id', autoIncrement: true });
        };
        req.onsuccess = () => { _db = req.result; resolve(_db); };
        req.onerror = () => reject(req.error);
    });
}

function _scheduleFlush() {
    if (_writeTimer) return;
    _writeTimer = setTimeout(() => _flushBuffer(), WRITE_FLUSH_MS);
}

async function _flushBuffer() {
    _writeTimer = null;
    if (_writeBuffer.length === 0) return;
    const batch = _writeBuffer;
    _writeBuffer = [];
    _pendingWrites += batch.length;

    try {
        const db = await _openDB();
        const tx = db.transaction(STORE_NAME, 'readwrite');
        for (const entry of batch) {
            tx.objectStore(STORE_NAME).add(entry);
        }
        // 环形缓冲: 超出上限删最旧
        const countReq = tx.objectStore(STORE_NAME).count();
        countReq.onsuccess = () => {
            if (countReq.result > MAX_PERSISTED) {
                const cursorReq = tx.objectStore(STORE_NAME).openCursor();
                let deleted = 0;
                const toDelete = countReq.result - MAX_PERSISTED;
                cursorReq.onsuccess = () => {
                    const cursor = cursorReq.result;
                    if (cursor && deleted < toDelete) {
                        cursor.delete();
                        deleted++;
                        cursor.continue();
                    }
                };
            }
        };
        await new Promise((resolve, reject) => {
            tx.oncomplete = () => resolve();
            tx.onerror = () => reject(tx.error);
        });
    } catch (_) { /* IndexedDB 不可达则静默降级 */ }
    finally { _pendingWrites -= batch.length; }
}

function _bufferWrite(entry) {
    _writeBuffer.push(entry);
    if (_writeBuffer.length >= WRITE_BATCH_SIZE) {
        _flushBuffer();
    } else {
        _scheduleFlush();
    }
}

// ── BroadcastChannel 跨页面同步 ——————————————————————————————————
const CHANNEL_NAME = 'ivg_log_bus';
let _bc = null;
try {
    if (typeof BroadcastChannel !== 'undefined') {
        _bc = new BroadcastChannel(CHANNEL_NAME);
    }
} catch (_) { /* 浏览器不支持则降级 */ }

// BroadcastChannel 节流: 每个 (source, msgKey) 最多每秒广播一次
const _bcThrottle = {};
const BC_THROTTLE_MS = 1000;

function _shouldBroadcast(entry) {
    // 非 info/warn/error 不广播 (debug/topic 摘要不跨页面)
    if (entry.level === 'debug') return false;
    const key = entry.source + ':' + (entry.feature || '') + ':' + entry.msg.slice(0, 60);
    const now = Date.now();
    const last = _bcThrottle[key] || 0;
    if (now - last < BC_THROTTLE_MS) return false;
    _bcThrottle[key] = now;
    return true;
}

// ── 频率折叠引擎 ——————————————————————————————————————————————————
const _rateTrackers = {};  // { key: { count, firstTs, lastTs } }

function _computeRateKey(entry, profile) {
    const fields = profile.keyFields;
    const parts = fields.map(f => {
        if (f === 'node' && entry.meta && entry.meta.node) return entry.meta.node;
        return String(entry[f] || '');
    });
    return parts.join('\x00');
}

function _applyRateLimit(entry) {
    const profile = RATE_PROFILES[entry.source];
    if (!profile || profile.windowMs <= 0) return entry; // 不限流

    const key = _computeRateKey(entry, profile);
    const now = Date.now();
    const tracker = _rateTrackers[key];

    if (!tracker) {
        // 首次出现: 记录并放行
        _rateTrackers[key] = { count: 1, firstTs: now, lastTs: now };
        return entry;
    }

    tracker.count++;
    tracker.lastTs = now;

    if (now - tracker.firstTs >= profile.windowMs) {
        // 窗口到期: 排放折叠摘要, 重置
        const count = tracker.count;
        const summaryEntry = {
            ts: _timestamp(),
            level: entry.level,
            source: entry.source,
            feature: entry.feature || '',
            msg: '[' + count + '次] ' + entry.msg,
            meta: Object.assign({}, entry.meta || {}, { _rateCount: count }),
        };
        _rateTrackers[key] = { count: 1, firstTs: now, lastTs: now };
        return summaryEntry;
    }

    // 窗口内重复: 抑制
    return null;
}

// ── 清理过期 tracker (每 60s 扫描一次) ——————————————————————————
let _trackerCleanupTimer = null;
function _scheduleTrackerCleanup() {
    if (_trackerCleanupTimer) return;
    _trackerCleanupTimer = setInterval(() => {
        const now = Date.now();
        const maxAge = 30000; // 30s 无活动则清理
        for (const [key, t] of Object.entries(_rateTrackers)) {
            if (now - t.lastTs > maxAge) {
                delete _rateTrackers[key];
            }
        }
        // 清理 BC 节流
        for (const [key, ts] of Object.entries(_bcThrottle)) {
            if (now - ts > maxAge) {
                delete _bcThrottle[key];
            }
        }
    }, 60000);
}

// ── LogEventBus 类 ———————————————————————————————————————————————
class LogEventBus {
    constructor() {
        this._entries = [];
        this._handlers = new Set();
        this._maxMemory = 2000;
        this._globalLevel = LOG_LEVELS.info;  // 默认: info 及以上
        this._debugMode = false;              // 调试模式: 关闭所有节流

        // 监听其他页面的日志
        if (_bc) {
            _bc.onmessage = (e) => {
                if (e.data && e.data.type === 'log' && e.data.entry) {
                    this._ingestRemote(e.data.entry);
                }
            };
        }

        // 页面关闭前刷写缓冲
        if (typeof window !== 'undefined') {
            window.addEventListener('pagehide', () => {
                if (_writeTimer) clearTimeout(_writeTimer);
                _flushBuffer();
            });
            window.addEventListener('beforeunload', () => {
                if (_writeTimer) clearTimeout(_writeTimer);
                _flushBuffer();
            });
        }

        _scheduleTrackerCleanup();
    }

    // ── 日志等级 ——————————————————————————————————————————————

    setLevel(level) {
        if (level >= LOG_LEVELS.silent || level < 0) {
            this._globalLevel = LOG_LEVELS.info;
        } else {
            this._globalLevel = level;
        }
    }

    getLevel() { return this._globalLevel; }
    getLevelName() { return LOG_LEVEL_NAMES[this._globalLevel] || 'info'; }

    setDebugMode(on) {
        this._debugMode = !!on;
    }

    isDebugMode() { return this._debugMode; }

    // ── 写入 ——————————————————————————————————————————————————

    addLog(level, source, msg, meta = {}, feature = '') {
        // 1. 全局等级门控
        const lv = LOG_LEVELS[level];
        if (lv === undefined) return;  // 无效等级
        if (lv < this._globalLevel) return;  // 低于阈值, 丢弃

        // 2. 构建条目
        const entry = {
            ts: _timestamp(),
            level,
            source,
            msg: String(msg == null ? '' : msg),
            meta: meta || {},
            feature: String(feature || ''),
        };

        // 3. 频率控制 (调试模式下跳过)
        let finalEntry = entry;
        if (!this._debugMode) {
            finalEntry = _applyRateLimit(entry);
            if (!finalEntry) return;  // 被抑制
        }

        // 4. 存入内存
        this._entries.push(finalEntry);
        while (this._entries.length > this._maxMemory) this._entries.shift();

        // 5. 通知本地监听器
        for (const fn of this._handlers) {
            try { fn(finalEntry); } catch (_) { /* */ }
        }

        // 6. 批量持久化 (非 debug 条目)
        if (finalEntry.level !== 'debug') {
            _bufferWrite(finalEntry);
        }

        // 7. 跨页面广播 (节流)
        if (_bc && _shouldBroadcast(finalEntry)) {
            try { _bc.postMessage({ type: 'log', entry: finalEntry }); } catch (_) { /* */ }
        }
    }

    // ── 远程摄入 ——————————————————————————————————————————————

    _ingestRemote(entry) {
        if (!entry || !entry.ts) return;
        this._entries.push(entry);
        while (this._entries.length > this._maxMemory) this._entries.shift();

        for (const fn of this._handlers) {
            try { fn(entry); } catch (_) { /* */ }
        }
    }

    // ── 监听 ——————————————————————————————————————————————————

    onLog(fn) { this._handlers.add(fn); }
    offLog(fn) { this._handlers.delete(fn); }

    // ── 查询 ——————————————————————————————————————————————————

    getLogs(opts = {}) {
        const { level, source, feature, search, since, limit } = opts;
        let result = this._entries;
        if (level)   result = result.filter(e => e.level === level);
        if (source)  result = result.filter(e => e.source === source);
        if (feature) result = result.filter(e => e.feature === feature);
        if (search) {
            const q = String(search).toLowerCase();
            result = result.filter(e => e.msg.toLowerCase().includes(q));
        }
        if (since)   result = result.filter(e => e.ts >= since);
        if (limit)   result = result.slice(-limit);
        return result;
    }

    count() { return this._entries.length; }

    clear() {
        this._entries.length = 0;
        for (const fn of this._handlers) {
            try { fn(null); } catch (_) { /* */ }
        }
        _openDB().then(db => {
            db.transaction(STORE_NAME, 'readwrite').objectStore(STORE_NAME).clear();
        }).catch(() => {});
        if (_bc) {
            try { _bc.postMessage({ type: 'clear' }); } catch (_) { /* */ }
        }
    }

    export() {
        return this._entries.map(e => {
            const featTag = e.feature ? ' [' + e.feature + ']' : '';
            const rateTag = (e.meta && e.meta._rateCount) ? ' [x' + e.meta._rateCount + ']' : '';
            return `${e.ts} [${e.source}]${featTag}${rateTag} ${e.level.toUpperCase()}  ${e.msg}`;
        }).join('\n');
    }

    getCategories() { return CATEGORIES; }
    getFeatures()  { return FEATURES; }
    getRateProfiles() { return RATE_PROFILES; }

    // ── 恢复历史日志 ——————————————————————————————————————————

    async restore(limit = 500) {
        // 先刷写缓冲区
        if (_writeTimer) { clearTimeout(_writeTimer); _writeTimer = null; }
        await _flushBuffer();

        try {
            const db = await _openDB();
            return new Promise(resolve => {
                const tx = db.transaction(STORE_NAME, 'readonly');
                const req = tx.objectStore(STORE_NAME).getAll();
                req.onsuccess = () => {
                    const all = req.result || [];
                    const recent = all.slice(-limit);
                    for (const e of recent) {
                        // 去重检查
                        const dup = this._entries.some(
                            x => x.ts === e.ts && x.msg === e.msg && x.source === e.source
                        );
                        if (!dup) {
                            this._entries.push(e);
                            for (const fn of this._handlers) {
                                try { fn(e); } catch (_) { /* */ }
                            }
                        }
                    }
                    resolve(recent.length);
                };
                req.onerror = () => resolve(0);
            });
        } catch (_) { return 0; }
    }

    // ── 调试 ——————————————————————————————————————————————————

    pendingWrites() { return _pendingWrites + _writeBuffer.length; }
    rateTrackerCount() { return Object.keys(_rateTrackers).length; }
}

// ── 时间戳 ————————————————————————————————————————————————————————
function _timestamp() {
    const d = new Date();
    const h = String(d.getHours()).padStart(2, '0');
    const m = String(d.getMinutes()).padStart(2, '0');
    const s = String(d.getSeconds()).padStart(2, '0');
    const ms = String(d.getMilliseconds()).padStart(3, '0');
    return `${h}:${m}:${s}.${ms}`;
}

// ── 模块级单例 ————————————————————————————————————————————————————
const logBus = new LogEventBus();

// 向后兼容: window.__ivgLog 仍可用
if (typeof window !== 'undefined') {
    window.__ivgLog = function (level, source, msg, meta, feature) {
        logBus.addLog(level, source, msg, meta, feature);
    };
}

export { logBus, CATEGORIES, FEATURES, LOG_LEVELS, RATE_PROFILES };
