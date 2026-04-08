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
		return parts.join('&');
	}

	function streamUrl(topic, opts) {
		opts = opts || {};
		const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
		const p = opts.port != null && opts.port > 0 ? Number(opts.port) : 8089;
		return `http://${h}:${p}/stream?${buildQuery(topic, opts, 'mjpeg')}`;
	}

	function snapshotUrl(topic, opts) {
		opts = opts || {};
		const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
		const p = opts.port != null && opts.port > 0 ? Number(opts.port) : 8089;
		return `http://${h}:${p}/snapshot?${buildQuery(topic, opts, 'jpeg')}`;
	}

	function viewerUrl(topic, opts) {
		opts = opts || {};
		const h = opts.host != null && opts.host !== '' ? String(opts.host) : hostname();
		const p = opts.port != null && opts.port > 0 ? Number(opts.port) : 8089;
		return `http://${h}:${p}/stream_viewer?topic=${encodeTopicQueryValue(topic)}`;
	}

	global.ivgWebVideo = {
		streamUrl,
		snapshotUrl,
		viewerUrl
	};
})(typeof window !== 'undefined' ? window : this);
