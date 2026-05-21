// log-bus.js — 统一日志事件总线 + IndexedDB 持久化 + BroadcastChannel 跨页面同步
// 所有模块通过此单例写入日志，替代分散的 console/__ivgLog/ros._log 喵~
//
// 用法:
//   import { logBus } from '../core/log-bus.js';
//   logBus.addLog('info', 'ros_manager', 'rosbridge connected');
//   logBus.onLog((entry) => { renderLine(entry); });
//
// 参考: 12-Factor App (日志作为事件流) + Grafana Loki (标签化过滤)
// 跨页面: BroadcastChannel API — 日志面板可实时看到其他页面的日志喵~

// ── 分类定义（颜色 + 标签）─────────────────────────────────────────────────
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

// ── IndexedDB 持久化 ─────────────────────────────────────────────────────
const DB_NAME = 'ivg_logs';
const DB_VERSION = 1;
const STORE_NAME = 'entries';
const MAX_PERSISTED = 5000;

let _db = null;
let _pendingWrites = 0;  // 追踪待写入数量，用于 pagehide flush

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

async function _persistOne(entry) {
    _pendingWrites++;
    try {
        const db = await _openDB();
        const tx = db.transaction(STORE_NAME, 'readwrite');
        tx.objectStore(STORE_NAME).add(entry);
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
    finally { _pendingWrites--; }
}

// ── BroadcastChannel 跨页面同步 ──────────────────────────────────────────
const CHANNEL_NAME = 'ivg_log_bus';
let _bc = null;
try {
    if (typeof BroadcastChannel !== 'undefined') {
        _bc = new BroadcastChannel(CHANNEL_NAME);
    }
} catch (_) { /* 浏览器不支持则降级 */ }

// ── LogEventBus 类 ────────────────────────────────────────────────────────
class LogEventBus {
    constructor() {
        this._entries = [];
        this._handlers = new Set();
        this._maxMemory = 2000;  // 内存上限

        // 监听其他页面的日志
        if (_bc) {
            _bc.onmessage = (e) => {
                if (e.data && e.data.type === 'log' && e.data.entry) {
                    this._ingestRemote(e.data.entry);
                }
            };
        }

        // 页面关闭前尽力刷入 IndexedDB
        if (typeof window !== 'undefined') {
            window.addEventListener('pagehide', () => {
                // 不阻塞页面关闭，尽力而为
            });
        }
    }

    // ── 写入 ──────────────────────────────────────────────────────────

    addLog(level, source, msg, meta = {}) {
        const entry = {
            ts: _timestamp(),
            level,
            source,
            msg: String(msg == null ? '' : msg),
            meta,
        };
        this._entries.push(entry);
        while (this._entries.length > this._maxMemory) this._entries.shift();

        // 通知本地监听器
        for (const fn of this._handlers) {
            try { fn(entry); } catch (_) { /* */ }
        }

        // 异步持久化（不阻塞当前调用）
        _persistOne(entry);

        // 广播到其他页面（日志面板可实时看到）
        if (_bc) {
            try { _bc.postMessage({ type: 'log', entry }); } catch (_) { /* */ }
        }
    }

    // ── 远程摄入（来自其他页面的日志，不写 IndexedDB 避免重复）─────

    _ingestRemote(entry) {
        if (!entry || !entry.ts) return;
        this._entries.push(entry);
        while (this._entries.length > this._maxMemory) this._entries.shift();

        // 通知本地监听器（日志面板 DOM 渲染等）
        for (const fn of this._handlers) {
            try { fn(entry); } catch (_) { /* */ }
        }
    }

    // ── 监听 ──────────────────────────────────────────────────────────

    onLog(fn) { this._handlers.add(fn); }
    offLog(fn) { this._handlers.delete(fn); }

    // ── 查询 ──────────────────────────────────────────────────────────

    getLogs(opts = {}) {
        const { level, source, search, since, limit } = opts;
        let result = this._entries;
        if (level)   result = result.filter(e => e.level === level);
        if (source)  result = result.filter(e => e.source === source);
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
        // 通知监听器清空
        for (const fn of this._handlers) {
            try { fn(null); } catch (_) { /* */ }
        }
        // 清空 IndexedDB
        _openDB().then(db => {
            db.transaction(STORE_NAME, 'readwrite').objectStore(STORE_NAME).clear();
        }).catch(() => {});
        // 通知其他页面清空
        if (_bc) {
            try { _bc.postMessage({ type: 'clear' }); } catch (_) { /* */ }
        }
    }

    export() {
        return this._entries.map(e =>
            `${e.ts} [${e.source}] ${e.level.toUpperCase()}  ${e.msg}`
        ).join('\n');
    }

    getCategories() { return CATEGORIES; }

    // ── 恢复历史日志 ──────────────────────────────────────────────────

    async restore(limit = 500) {
        try {
            const db = await _openDB();
            return new Promise(resolve => {
                const tx = db.transaction(STORE_NAME, 'readonly');
                const req = tx.objectStore(STORE_NAME).getAll();
                req.onsuccess = () => {
                    const all = req.result || [];
                    const recent = all.slice(-limit);
                    for (const e of recent) {
                        // 检查去重：不重复添加已存在的条目
                        const dup = this._entries.some(x => x.ts === e.ts && x.msg === e.msg && x.source === e.source);
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

    // ── 待写入数量 (调试用) ──────────────────────────────────────────

    pendingWrites() { return _pendingWrites; }
}

// ── 时间戳 ────────────────────────────────────────────────────────────────
function _timestamp() {
    const d = new Date();
    const h = String(d.getHours()).padStart(2, '0');
    const m = String(d.getMinutes()).padStart(2, '0');
    const s = String(d.getSeconds()).padStart(2, '0');
    const ms = String(d.getMilliseconds()).padStart(3, '0');
    return `${h}:${m}:${s}.${ms}`;
}

// ── 模块级单例 ────────────────────────────────────────────────────────────
const logBus = new LogEventBus();

// 向后兼容: window.__ivgLog 仍可用
if (typeof window !== 'undefined') {
    window.__ivgLog = logBus.addLog.bind(logBus);
}

export { logBus, CATEGORIES };
