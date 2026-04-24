/**
 * web_video_server（ROS2）查询串：/stream、/snapshot、/stream_viewer
 * @see https://github.com/RobotWebTools/web_video_server/blob/ros2/README.md
 *
 * topic 查询值勿用 URLSearchParams 整段编码（会把 / 编成 %2F，部分服务端无法解析）。
 */

const global = globalThis;

	/** 当前页 hostname；无 ``window`` 时回退 ``127.0.0.1``（独立 web_video 直连模式）。 */
	function hostname() {
		return (typeof window !== 'undefined' && window.location && window.location.hostname) || '127.0.0.1';
	}

	/** 按段 encodeURIComponent 再以 / 拼接，避免整段编码把 ROS topic 的 / 变成 %2F。 */
	function encodeTopicQueryValue(topic) {
		return String(topic)
			.split('/')
			.map((seg) => encodeURIComponent(seg))
			.join('/');
	}

	/** 非代理模式下的 HTTP 端口：opts.port → location.port → https 443 / 80。 */
	function legacyStandalonePort(opts) {
		let p = opts.port != null && opts.port > 0 ? Number(opts.port) : NaN;
		if (!isNaN(p) && p > 0) return p;
		if (typeof global !== 'undefined' && global.location && global.location.port) {
			const lp = parseInt(String(global.location.port), 10);
			if (!isNaN(lp) && lp > 0) return lp;
		}
		if (typeof global !== 'undefined' && global.location && global.location.protocol === 'https:') return 443;
		return 80;
	}

	/** 构造 web_video_server 查询串（topic 分段编码、type/quality/client_id 等）。 */
	function buildQuery(topic, opts, defaultType) {
		const parts = ['topic=' + encodeTopicQueryValue(topic)];
		parts.push('type=' + encodeURIComponent(opts.type != null ? String(opts.type) : defaultType));
		if (opts.width > 0) parts.push('width=' + encodeURIComponent(String(opts.width)));
		if (opts.height > 0) parts.push('height=' + encodeURIComponent(String(opts.height)));
		if (opts.quality != null && opts.quality > 0) {
			parts.push('quality=' + encodeURIComponent(String(opts.quality)));
		}
		if (opts.qos_profile) parts.push('qos_profile=' + encodeURIComponent(String(opts.qos_profile)));
		if (opts.default_transport) {
			parts.push('default_transport=' + encodeURIComponent(String(opts.default_transport)));
		}
		const cid = opts.client_id != null ? String(opts.client_id) : '';
		if (cid !== '') parts.push('client_id=' + encodeURIComponent(cid));
		return parts.join('&');
	}

	/** MJPEG 流 URL：优先 IVG 同源 camera_stream 或代理前缀，否则直连 host:port/stream。 */
	function streamUrl(topic, opts) {
		opts = opts || {};
		if (typeof global.ivgPorts !== 'undefined' && typeof global.ivgPorts.webVideoProxyOriginPrefix === 'function') {
			const base = global.ivgPorts.webVideoProxyOriginPrefix();
			if (base && base.indexOf('/api/v1/camera/stream') !== -1) {
				const q = new URLSearchParams();
				q.set('topic', String(topic));
				q.set('stream_id', opts.client_id != null ? String(opts.client_id) : 'ivg_web_video');
				const qual = opts.quality != null && opts.quality > 0 ? opts.quality : 85;
				q.set('quality', String(qual));
				return `${base}?${q.toString()}`;
			}
			if (base) return `${base}/stream?${buildQuery(topic, opts, 'mjpeg')}`;
		}
		const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
		const p = legacyStandalonePort(opts);
		return `http://${h}:${p}/stream?${buildQuery(topic, opts, 'mjpeg')}`;
	}

	/** 单帧 JPEG URL；IVG camera_stream 模式下可能与 stream 共用 API（见实现分支）。 */
	function snapshotUrl(topic, opts) {
		opts = opts || {};
		if (typeof global.ivgPorts !== 'undefined' && typeof global.ivgPorts.webVideoProxyOriginPrefix === 'function') {
			const base = global.ivgPorts.webVideoProxyOriginPrefix();
			if (base && base.indexOf('/api/v1/camera/stream') !== -1) {
				return streamUrl(topic, opts);
			}
			if (base) return `${base}/snapshot?${buildQuery(topic, opts, 'jpeg')}`;
		}
		const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
		const p = legacyStandalonePort(opts);
		return `http://${h}:${p}/snapshot?${buildQuery(topic, opts, 'jpeg')}`;
	}

	/** web_video_server 自带 stream_viewer 页面 URL（或 IVG 模式下退化为 stream）。 */
	function viewerUrl(topic, opts) {
		opts = opts || {};
		if (typeof global.ivgPorts !== 'undefined' && typeof global.ivgPorts.webVideoProxyOriginPrefix === 'function') {
			const base = global.ivgPorts.webVideoProxyOriginPrefix();
			if (base && base.indexOf('/api/v1/camera/stream') !== -1) {
				return streamUrl(topic, opts);
			}
			if (base) return `${base}/stream_viewer?topic=${encodeTopicQueryValue(topic)}`;
		}
		const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
		const p = legacyStandalonePort(opts);
		return `http://${h}:${p}/stream_viewer?topic=${encodeTopicQueryValue(topic)}`;
	}

	/**
	 * MJPEG ``<img>`` 断流/5xx 时指数退避重载 ``src``（query 追加 ``_ivgRecover`` 破缓存）。
	 * 同一 img 重复调用会先卸掉上一轮监听，避免 startSubscriptions 叠监听器。
	 */
	function mjpegStreamAttachAutoReload(img, getUrl) {
		if (!img || typeof getUrl !== 'function') return;
		if (img._ivgMjpegRecoverCleanup) {
			img._ivgMjpegRecoverCleanup();
			img._ivgMjpegRecoverCleanup = null;
		}
		let attempts = 0;
		let timer = null;
		const maxAttempts = 15;
		function clearTimer() {
			if (timer) {
				clearTimeout(timer);
				timer = null;
			}
		}
		function reload() {
			if (attempts >= maxAttempts) return;
			attempts++;
			const delay = Math.min(30000, 2000 * Math.pow(2, attempts - 1));
			clearTimer();
			timer = setTimeout(() => {
				timer = null;
				const u = getUrl();
				const sep = u.indexOf('?') >= 0 ? '&' : '?';
				img.src = `${u}${sep}_ivgRecover=${Date.now()}`;
			}, delay);
		}
		function onLoad() {
			attempts = 0;
			clearTimer();
		}
		function onError() {
			reload();
		}
		img.addEventListener('load', onLoad);
		img.addEventListener('error', onError);
		img._ivgMjpegRecoverCleanup = () => {
			img.removeEventListener('load', onLoad);
			img.removeEventListener('error', onError);
			clearTimer();
		};
	}

	global.ivgWebVideo = {
		streamUrl,
		snapshotUrl,
		viewerUrl,
		mjpegStreamAttachAutoReload
	};
