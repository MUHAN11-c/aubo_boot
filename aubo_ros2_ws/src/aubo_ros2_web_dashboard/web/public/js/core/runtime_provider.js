// runtime_provider — 从后端 BFF 获取运行时配置
// 链路: 浏览器 → GET /api/v1/runtime → {rosbridge_port, web_video_port, ...}
// 全局单例: globalThis.__IVG_RUNTIME，首次 fetch 后缓存
const g = globalThis;

async function fetchPrimaryRuntime() {
    try {
        const r = await fetch('/api/v1/runtime', { credentials: 'same-origin' });
        if (!r.ok) return null;
        const j = await r.json();
        return j && typeof j === 'object' ? j : null;
    } catch (e) {
        return null;
    }
}

async function loadIvgRuntime() {
    if (g.__IVG_RUNTIME_PROMISE) return g.__IVG_RUNTIME_PROMISE;
    g.__IVG_RUNTIME_PROMISE = (async () => {
        const primary = await fetchPrimaryRuntime();
        g.__IVG_RUNTIME = primary || {};
        return g.__IVG_RUNTIME;
    })();
    return g.__IVG_RUNTIME_PROMISE;
}

export { loadIvgRuntime };
