/**
 * web_video_server（ROS2）查询串：/stream、/snapshot、/stream_viewer
 * @see https://github.com/RobotWebTools/web_video_server/blob/ros2/README.md
 *
 * topic 查询值勿用 URLSearchParams 整段编码（会把 / 编成 %2F，部分服务端无法解析）。
 */
(function (global) {
	'use strict';

	function hostname() {
		return (typeof window !== 'undefined' && window.location && window.location.hostname) || '127.0.0.1';
	}

	function encodeTopicQueryValue(topic) {
		return String(topic)
			.split('/')
			.map((seg) => encodeURIComponent(seg))
			.join('/');
	}

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

	function streamUrl(topic, opts) {
		opts = opts || {};
		if (typeof global.ivgPorts !== 'undefined' && typeof global.ivgPorts.webVideoProxyOriginPrefix === 'function') {
			const base = global.ivgPorts.webVideoProxyOriginPrefix();
			if (base) return `${base}/stream?${buildQuery(topic, opts, 'mjpeg')}`;
		}
		const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
		const p = opts.port != null && opts.port > 0 ? Number(opts.port) : 8089;
		return `http://${h}:${p}/stream?${buildQuery(topic, opts, 'mjpeg')}`;
	}

	function snapshotUrl(topic, opts) {
		opts = opts || {};
		if (typeof global.ivgPorts !== 'undefined' && typeof global.ivgPorts.webVideoProxyOriginPrefix === 'function') {
			const base = global.ivgPorts.webVideoProxyOriginPrefix();
			if (base) return `${base}/snapshot?${buildQuery(topic, opts, 'jpeg')}`;
		}
		const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
		const p = opts.port != null && opts.port > 0 ? Number(opts.port) : 8089;
		return `http://${h}:${p}/snapshot?${buildQuery(topic, opts, 'jpeg')}`;
	}

	function viewerUrl(topic, opts) {
		opts = opts || {};
		if (typeof global.ivgPorts !== 'undefined' && typeof global.ivgPorts.webVideoProxyOriginPrefix === 'function') {
			const base = global.ivgPorts.webVideoProxyOriginPrefix();
			if (base) return `${base}/stream_viewer?topic=${encodeTopicQueryValue(topic)}`;
		}
		const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
		const p = opts.port != null && opts.port > 0 ? Number(opts.port) : 8089;
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
})(typeof window !== 'undefined' ? window : this);
