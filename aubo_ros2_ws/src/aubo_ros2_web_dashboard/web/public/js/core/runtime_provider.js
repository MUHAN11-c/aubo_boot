/**
 * IVG runtime 单一提供者：
 * - 唯一请求入口：/api/v1/runtime
 * - 统一缓存位置：globalThis.__IVG_RUNTIME
 * - 统一 Promise 去重：globalThis.__IVG_RUNTIME_PROMISE
 */

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

/**
 * 统一 runtime 加载入口（带全局 Promise 去重）。
 * @returns {Promise<Record<string, any>>}
 */
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
