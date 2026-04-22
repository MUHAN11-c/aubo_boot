/**
 * topics_lab 渲染基础：
 * - 只处理消息类型判断、JSON 安全截断、原始消息摘要。
 * - 不依赖 ROS 连接状态，供不同渲染器和控制器共享。
 */
(function (global) {
	'use strict';

	function safeJson(obj, maxLen) {
		try {
			const s = JSON.stringify(obj, null, 2);
			if (maxLen && s.length > maxLen) {
				return `${s.slice(0, maxLen)}\n…\n(truncated, total ${s.length} chars)`;
			}
			return s;
		} catch (e) {
			return String(obj);
		}
	}

	function typeMatch(msgType, needle) {
		if (!msgType) return false;
		return msgType === needle || msgType.indexOf(needle) !== -1;
	}

	/** 避免对大 payload 直接 stringify 卡主线程，右侧仅给元数据摘要。 */
	function rawPreviewForMessage(msgType, msg, maxLen) {
		if (!msg) return '';
		const omit = hint => `[omitted: ${hint} — 降低卡顿；完整显示请用 RViz、本页「3D 图」或 Image 的 MJPEG]`;
		if (typeMatch(msgType, 'Image') && !typeMatch(msgType, 'CompressedImage')) {
			let in0 = 0;
			if (Array.isArray(msg.data)) in0 = msg.data.length;
			else if (typeof msg.data === 'string') in0 = msg.data.length;
			return safeJson({
				header: msg.header,
				height: msg.height,
				width: msg.width,
				encoding: msg.encoding,
				is_bigendian: msg.is_bigendian,
				step: msg.step,
				data: omit(`image payload ~${in0} array el / base64 chars`)
			}, maxLen);
		}
		if (typeMatch(msgType, 'CompressedImage')) {
			let ic = 0;
			if (typeof msg.data === 'string') ic = msg.data.length;
			else if (Array.isArray(msg.data)) ic = msg.data.length;
			return safeJson({
				header: msg.header,
				format: msg.format,
				data: omit(`compressed ~${ic} chars`)
			}, maxLen);
		}
		if (typeMatch(msgType, 'PointCloud2')) {
			let ip = 0;
			if (Array.isArray(msg.data)) ip = msg.data.length;
			else if (typeof msg.data === 'string') ip = msg.data.length;
			return safeJson({
				header: msg.header,
				height: msg.height,
				width: msg.width,
				fields: msg.fields,
				is_bigendian: msg.is_bigendian,
				point_step: msg.point_step,
				row_step: msg.row_step,
				is_dense: msg.is_dense,
				data: omit(`pointcloud payload ~${ip} units`)
			}, maxLen);
		}
		if (typeMatch(msgType, 'LaserScan') && Array.isArray(msg.ranges)) {
			return safeJson({
				header: msg.header,
				angle_min: msg.angle_min,
				angle_max: msg.angle_max,
				angle_increment: msg.angle_increment,
				time_increment: msg.time_increment,
				scan_time: msg.scan_time,
				range_min: msg.range_min,
				range_max: msg.range_max,
				ranges: omit(`${msg.ranges.length} samples`)
			}, maxLen);
		}
		if (typeMatch(msgType, 'OccupancyGrid') && msg.info && Array.isArray(msg.data)) {
			return safeJson({
				header: msg.header,
				info: msg.info,
				data: omit(`${msg.data.length} cells`)
			}, maxLen);
		}
		return safeJson(msg, maxLen);
	}

	global.IVGTopicsLabRenderPreview = {
		safeJson,
		typeMatch,
		rawPreviewForMessage
	};
})(typeof window !== 'undefined' ? window : globalThis);
