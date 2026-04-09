/**
 * IVG 运行时端口：与 FastAPI ``/api/ivg/runtime-config`` 及 URL 查询一致。
 *
 * 浏览器经同源 ``/ws/rosbridge`` 与 ``/api/ivg/proxy/web-video/…`` 访问上游；``rosbridge_port`` / ``web_video_port`` 供 UI 与 ``?rosbridge_port=`` 等展示或兼容旧表单。
 * **自动重连**：``scheduleRosReconnect`` 指数退避；后台标签页等 ``visibilitychange`` 后再 ``connect``；``wireOnlineRosReconnect`` 在 ``window.online`` 时清零计数并立即 ``connect``。
 * @see aubo_ros2_web_dashboard.gateway
 */
(function (g) {
	'use strict';

	let loadPromise = null;

	async function loadRuntime() {
		if (loadPromise) return loadPromise;
		loadPromise = fetch('/api/ivg/runtime-config', { credentials: 'same-origin' })
			.then(r => (r.ok ? r.json() : {}))
			.catch(() => ({}))
			.then(cfg => {
				g.__IVG_RUNTIME = cfg && typeof cfg === 'object' ? cfg : {};
				return g.__IVG_RUNTIME;
			});
		return loadPromise;
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

	/** rosbridge WebSocket 完整 URL（同 host 的 /ws/rosbridge）。 */
	function rosbridgeWebSocketUrl() {
		const rt = g.__IVG_RUNTIME || {};
		const path = rt.rosbridge_ws_path || '/ws/rosbridge';
		const proto = g.location.protocol === 'https:' ? 'wss:' : 'ws:';
		return `${proto}//${g.location.host}${path}`;
	}

	/** web_video 的 origin+代理前缀（同源 /api/ivg/proxy/web-video）。 */
	function webVideoProxyOriginPrefix() {
		const rt = g.__IVG_RUNTIME || {};
		const pre = rt.web_video_proxy_prefix || '/api/ivg/proxy/web-video';
		return `${g.location.origin}${pre}`;
	}

	function rosbridge() {
		return fromQuery('rosbridge_port') || fromRuntime('rosbridge_port') || 9090;
	}

	/** @param {HTMLInputElement|null|undefined} inputEl 有值时优先于 query/runtime */
	function webVideo(inputEl) {
		if (inputEl && inputEl.value) {
			const pv = parsePort(String(inputEl.value).replace(/[^\d]/g, ''));
			if (pv) return pv;
		}
		return fromQuery('web_video_port') || fromRuntime('web_video_port') || 8089;
	}

	/** rosbridge 自动重连：与页面内 connect() 配合；用 gen 丢弃旧实例的 close。 */
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

	g.ivgPorts = {
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
})(typeof window !== 'undefined' ? window : globalThis);
