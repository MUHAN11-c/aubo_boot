/**
 * rosbridge v2 JSON 中字节字段的常见形态 → Uint8Array（与 roslibjs 反序列化一致）。
 * topics_lab（CompressedImage / PointCloud2 摘要）、ivg_image_canvas 共用；无渲染逻辑。
 */
(function (global) {
	'use strict';

	function toUint8(data) {
		if (!data) return null;
		if (data instanceof Uint8Array) return data;
		if (typeof data === 'string') {
			try {
				const bin = atob(data);
				const out = new Uint8Array(bin.length);
				for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
				return out;
			} catch (e) {
				return null;
			}
		}
		if (Array.isArray(data)) return new Uint8Array(data);
		if (data.data && Array.isArray(data.data)) return new Uint8Array(data.data);
		return null;
	}

	global.IVGRosbridgeBytes = { toUint8 };
})(typeof window !== 'undefined' ? window : this);
