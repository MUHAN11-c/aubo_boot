// ivg_runtime — 运行时配置、WebSocket URL、断线重连状态机
// 依赖: runtime_provider.js 获取的 __IVG_RUNTIME 全局配置
// 对外: ivgPorts 对象（loadRuntime, rosbridge, 重连方法）
import { loadIvgRuntime } from './core/runtime_provider.js';

const g = globalThis;

// ── 运行时配置 ───────────────────────────────────────────────────────────────

async function loadRuntime() {
    return loadIvgRuntime();
}

function parsePort(v) {
    const n = parseInt(String(v), 10);
    return !isNaN(n) && n > 0 ? n : null;
}

function fromQuery(name) {
    const v = new URLSearchParams(g.location.search).get(name);
    if (v == null || v === '') return null;
    return parsePort(v);
}

function fromRuntime(key) {
    const rt = g.__IVG_RUNTIME;
    if (!rt || rt[key] == null) return null;
    return parsePort(rt[key]);
}

// ── WebSocket URL 构建 ───────────────────────────────────────────────────────

function rosbridgeWebSocketUrlFromRuntime(rt) {
    const r = rt || {};
    // 优先使用显式控制通道
    if (r.ivg_ws_control && String(r.ivg_ws_control).trim())
        return String(r.ivg_ws_control).trim();
    // 否则根据运行时配置拼接同源 WebSocket 地址
    const path = (r.rosbridge_ws_path && String(r.rosbridge_ws_path).trim()) || '/ws/rosbridge';
    const proto = g.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return `${proto}//${g.location.host}${path.startsWith('/') ? path : `/${path}`}`;
}

function rosbridgeWebSocketUrl() {
    return rosbridgeWebSocketUrlFromRuntime(g.__IVG_RUNTIME);
}

function foxgloveWebSocketUrl() {
    const rt = g.__IVG_RUNTIME || {};
    const path = rt.foxglove_ws_path || '/ws/foxglove';
    const proto = g.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return `${proto}//${g.location.host}${path}`;
}


// ── 端口获取（优先级: URL参数 > 运行时配置 > 默认值）────────────────────

function rosbridgePort() {
    return fromQuery('rosbridge_port') || fromRuntime('rosbridge_port') || 9090;
}

// ── 断线重连状态机 ───────────────────────────────────────────────────────────

function createRosReconnectState() {
    return { gen: 0, attempts: 0, timer: null };
}

function clearRosReconnectTimer(state) {
    if (state.timer) {
        clearTimeout(state.timer);
        state.timer = null;
    }
    if (state._visHandler && typeof document !== 'undefined') {
        document.removeEventListener('visibilitychange', state._visHandler);
        state._visHandler = null;
    }
}

function bumpRosReconnectGen(state) {
    state.gen++;
    return state.gen;
}

function scheduleRosReconnect(state, connectFn, opts) {
    opts = opts || {};
    const maxAttempts = opts.maxAttempts != null ? opts.maxAttempts : 12;
    const baseMs = opts.baseMs != null ? opts.baseMs : 2000;
    const capMs = opts.capMs != null ? opts.capMs : 30000;
    clearRosReconnectTimer(state);

    if (state.attempts >= maxAttempts) {
        if (opts.onExhausted) opts.onExhausted();
        return;
    }

    // 指数退避: 2s, 4s, 8s, ... 上限 30s
    const delay = Math.min(capMs, baseMs * Math.pow(2, state.attempts));
    state.attempts++;
    if (opts.onSchedule) opts.onSchedule(delay, state.attempts, maxAttempts);

    state.timer = setTimeout(() => {
        state.timer = null;
        const scheduledGen = state.gen;

        function fire() {
            if (scheduledGen !== state.gen) return; // 已有新连接，放弃本次
            connectFn();
        }

        // 页面隐藏时推迟重连，等恢复可见再执行
        if (typeof document !== 'undefined' && document.visibilityState === 'hidden') {
            const handler = () => {
                if (document.visibilityState === 'hidden') return;
                document.removeEventListener('visibilitychange', handler);
                if (state._visHandler === handler) state._visHandler = null;
                fire();
            };
            state._visHandler = handler;
            document.addEventListener('visibilitychange', handler);
        } else {
            fire();
        }
    }, delay);
}

function wireOnlineRosReconnect(state, connectFn) {
    if (typeof window === 'undefined' || typeof window.addEventListener !== 'function') return;
    window.addEventListener('online', () => {
        if (typeof navigator !== 'undefined' && navigator.onLine === false) return;
        clearRosReconnectTimer(state);
        state.attempts = 0;
        connectFn();
    });
}

// ── 对外接口 ─────────────────────────────────────────────────────────────────

const ivgPorts = {
    loadRuntime,
    rosbridge: rosbridgePort,
    rosbridgeWebSocketUrl,
    createRosReconnectState,
    clearRosReconnectTimer,
    bumpRosReconnectGen,
    scheduleRosReconnect,
    wireOnlineRosReconnect,
};
g.ivgPorts = ivgPorts;

export {
    loadRuntime,
    parsePort,
    rosbridgePort as rosbridge,
    rosbridgeWebSocketUrlFromRuntime,
    rosbridgeWebSocketUrl,
    foxgloveWebSocketUrl,
    createRosReconnectState,
    clearRosReconnectTimer,
    bumpRosReconnectGen,
    scheduleRosReconnect,
    wireOnlineRosReconnect,
    ivgPorts,
};
