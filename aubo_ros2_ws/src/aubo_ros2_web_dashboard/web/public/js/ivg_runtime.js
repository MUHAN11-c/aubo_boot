/**
 * IVG 运行时：统一通过 ``/api/v1/runtime`` 获取。
 * 新栈：``ivg_ws_control`` / ``camera_stream_path``（同源 MJPEG，无独立 8089）；旧字段仍作兼容。
 */
import { loadIvgRuntime } from './core/runtime_provider.js';

const g = globalThis;

/** 单次缓存：统一委托到 runtime_provider，结果挂到 ``global.__IVG_RUNTIME``。 */
async function loadRuntime() {
	return loadIvgRuntime();
}

/** 将字符串解析为正整数端口；非法则 ``null``。 */
function parsePort(v) {
	const n = parseInt(String(v), 10);
	return !isNaN(n) && n > 0 ? n : null;
}

/** 从当前页 URL 查询串读取端口类参数并 ``parsePort``。 */
function fromQuery(name) {
	const v = new URLSearchParams(g.location.search).get(name);
	if (v == null || v === '') return null;
	return parsePort(v);
}

/** 从已加载的 ``__IVG_RUNTIME`` 字典读键并 ``parsePort``。 */
function fromRuntime(key) {
	const rt = g.__IVG_RUNTIME;
	if (!rt || rt[key] == null) return null;
	return parsePort(rt[key]);
}

/**
 * 由 runtime 字典构造 rosbridge WebSocket URL（与 ``ivg_transport`` 共用，避免两处逻辑漂移）。
 * @param {Record<string, unknown>|null|undefined} rt
 */
function rosbridgeWebSocketUrlFromRuntime(rt) {
	const r = rt || {};
	if (r.ivg_ws_control && String(r.ivg_ws_control).trim()) return String(r.ivg_ws_control).trim();
	const path = (r.rosbridge_ws_path && String(r.rosbridge_ws_path).trim()) || '/ws/rosbridge';
	const proto = g.location.protocol === 'https:' ? 'wss:' : 'ws:';
	return `${proto}//${g.location.host}${path.startsWith('/') ? path : `/${path}`}`;
}

/** 控制面 WebSocket 完整 URL（IVG：``ivg_ws_control``；默认同源 ``/ws/rosbridge``）。 */
function rosbridgeWebSocketUrl() {
	return rosbridgeWebSocketUrlFromRuntime(g.__IVG_RUNTIME);
}

/** 相机流同源前缀（优先 ``camera_stream_path``，否则 ``web_video_proxy_prefix``）。 */
function webVideoProxyOriginPrefix() {
	const rt = g.__IVG_RUNTIME || {};
	const pre = rt.camera_stream_path || rt.web_video_proxy_prefix || '/api/ivg/proxy/web-video';
	return `${g.location.origin}${pre}`;
}

/** 解析 rosbridge 端口：查询串优先，其次 runtime，默认 9090（遗留展示用；实际 WS 见 ``rosbridgeWebSocketUrl``）。 */
function rosbridge() {
	return fromQuery('rosbridge_port') || fromRuntime('rosbridge_port') || 9090;
}

/**
 * 独立 web_video_server 的端口（遗留）；IVG 页面请用 ``ivg_transport.cameraStreamUrl`` 或 ``ivg_web_video`` 走同源代理。
 * @returns {number|null}
 */
function webVideo(inputEl) {
	if (inputEl && inputEl.value) {
		const pv = parsePort(String(inputEl.value).replace(/[^\d]/g, ''));
		if (pv) return pv;
	}
	return fromQuery('web_video_port') || fromRuntime('web_video_port');
}

/** rosbridge 自动重连：与页面内 connect() 配合；用 gen 丢弃旧实例的 close。 */
function createRosReconnectState() {
	return { gen: 0, attempts: 0, timer: null };
}

/** 清除退避定时器并移除 ``visibilitychange`` 监听（与 ``scheduleRosReconnect`` 成对）。 */
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

/** 递增代数，使已安排的旧重连回调在 ``fire`` 时自动失效。 */
function bumpRosReconnectGen(state) {
	state.gen++;
	return state.gen;
}

/**
 * @param {{ gen: number, attempts: number, timer: ReturnType<typeof setTimeout>|null }} state
 * @param {() => void} connectFn
 * @param {{ maxAttempts?: number, baseMs?: number, capMs?: number, onSchedule?: (delayMs:number, attempt:number, max:number)=>void, onExhausted?: () => void }} [opts]
 */
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
	const delay = Math.min(capMs, baseMs * Math.pow(2, state.attempts));
	state.attempts++;
	if (opts.onSchedule) opts.onSchedule(delay, state.attempts, maxAttempts);
	state.timer = setTimeout(() => {
		state.timer = null;
		const scheduledGen = state.gen;
		function fire() {
			if (scheduledGen !== state.gen) return;
			connectFn();
		}
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

/** 浏览器网络恢复（``online``）时立刻重连 rosbridge：清零退避计数并调用页面的 ``connect()``。 */
function wireOnlineRosReconnect(state, connectFn) {
	if (typeof window === 'undefined' || typeof window.addEventListener !== 'function') return;
	window.addEventListener('online', () => {
		if (typeof navigator !== 'undefined' && navigator.onLine === false) return;
		clearRosReconnectTimer(state);
		state.attempts = 0;
		connectFn();
	});
}

const ivgPorts = {
		loadRuntime,
		rosbridge,
		webVideo,
		rosbridgeWebSocketUrl,
		webVideoProxyOriginPrefix,
		createRosReconnectState,
		clearRosReconnectTimer,
		bumpRosReconnectGen,
		scheduleRosReconnect,
		wireOnlineRosReconnect
	};

g.ivgPorts = ivgPorts;

export {
	loadRuntime,
	parsePort,
	rosbridge,
	webVideo,
	rosbridgeWebSocketUrlFromRuntime,
	rosbridgeWebSocketUrl,
	webVideoProxyOriginPrefix,
	createRosReconnectState,
	clearRosReconnectTimer,
	bumpRosReconnectGen,
	scheduleRosReconnect,
	wireOnlineRosReconnect,
	ivgPorts
};
