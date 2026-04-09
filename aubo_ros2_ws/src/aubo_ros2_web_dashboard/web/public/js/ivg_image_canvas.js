/**
 * vision_grasp_panel：sensor_msgs/Image（rosbridge）→ Canvas2D。
 * 依赖：先加载 ivg_rosbridge_bytes.js（IVGRosbridgeBytes.toUint8）。
 * topics_lab 的非压缩 Image 仅 MJPEG，不使用本文件。
 */
(function (global) {
	'use strict';

	const R = global.IVGRosbridgeBytes;
	if (!R || typeof R.toUint8 !== 'function') {
		throw new Error('ivg_image_canvas.js requires ivg_rosbridge_bytes.js loaded first');
	}

	/**
	 * @returns {{ ok: true } | { ok: false, hint: string }}
	 */
	function paintSensorImage(msg, canvas) {
		if (!msg || !msg.width || !msg.height || !canvas) {
			return { ok: false, hint: '<p class="hint">Invalid Image</p>' };
		}
		const enc = (msg.encoding || '').toLowerCase();
		const raw = R.toUint8(msg.data);
		if (!raw) {
			return { ok: false, hint: `<p class="hint">Could not decode image data (encoding ${enc})</p>` };
		}
		const w = msg.width;
		const h = msg.height;
		if (canvas.width !== w || canvas.height !== h) {
			canvas.width = w;
			canvas.height = h;
		}
		const ctx = canvas.getContext('2d');
		const imgData = ctx.createImageData(w, h);
		const px = imgData.data;
		if (enc === 'rgb8' || enc === 'rgba8') {
			const step = enc === 'rgba8' ? 4 : 3;
			const need = w * h * step;
			if (raw.length < need) return { ok: false, hint: '<p class="hint">Image buffer too short</p>' };
			let p = 0;
			for (let i = 0; i < w * h; i++) {
				px[p++] = raw[i * step];
				px[p++] = raw[i * step + 1];
				px[p++] = raw[i * step + 2];
				px[p++] = 255;
			}
		} else if (enc === 'bgr8') {
			if (raw.length < w * h * 3) return { ok: false, hint: '<p class="hint">Image buffer too short</p>' };
			let p2 = 0;
			for (let j = 0; j < w * h; j++) {
				px[p2++] = raw[j * 3 + 2];
				px[p2++] = raw[j * 3 + 1];
				px[p2++] = raw[j * 3];
				px[p2++] = 255;
			}
		} else if (enc === 'mono8' || enc === '8uc1') {
			if (raw.length < w * h) return { ok: false, hint: '<p class="hint">Image buffer too short</p>' };
			let p3 = 0;
			for (let k = 0; k < w * h; k++) {
				const g = raw[k];
				px[p3++] = g;
				px[p3++] = g;
				px[p3++] = g;
				px[p3++] = 255;
			}
		} else {
			return {
				ok: false,
				hint: `<p class="hint">Encoding <code>${enc}</code> — supported: rgb8, bgr8, rgba8, mono8</p>`
			};
		}
		ctx.putImageData(imgData, 0, 0);
		return { ok: true };
	}

	function applyVisionPanelImageStyle(canvas) {
		const host = canvas.parentElement;
		if (host) {
			if (document.body.classList.contains('ivg-single-screen')) {
				canvas.style.width = '';
				canvas.style.height = '';
			} else {
				canvas.style.width = '100%';
				canvas.style.height = 'auto';
			}
		} else {
			canvas.style.width = '';
			canvas.style.height = '';
		}
	}

	global.IVGImageCanvas = {
		paintSensorImage,
		applyVisionPanelImageStyle
	};
})(typeof window !== 'undefined' ? window : this);
