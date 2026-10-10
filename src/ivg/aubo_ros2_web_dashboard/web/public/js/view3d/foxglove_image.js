// foxglove_image.js — Foxglove CDR 图像订阅 + Canvas 渲染
// 大数据 (点云/图像) 统一走 foxglove WebSocket, 替代 web_video_server HTTP MJPEG

/**
 * createFoxgloveImageRenderer(opts)
 *   opts.topic     — 图像话题 (默认 '/camera/color/image_raw')
 *   opts.canvasId  — 渲染目标 canvas id
 *   opts.onFrame   — (canvas, width, height) => {} 每帧回调, 用于触发投影重绘
 *   returns { start, stop, isActive }
 */
function createFoxgloveImageRenderer(opts) {
	var topic = opts.topic || '/camera/color/image_raw';
	var canvasId = opts.canvasId || 'foxglove-image-canvas';
	var onFrame = opts.onFrame || null;

	var _subCancel = null;
	var _active = false;
	var _listeningForFox = false;
	var _canvas = null;
	var _ctx = null;
	var _tempBuf = null;

	function _ensureCanvas() {
		if (_canvas) return true;
		_canvas = document.getElementById(canvasId);
		if (!_canvas) { console.warn('[foxglove-image] canvas #' + canvasId + ' not found'); return false; }
		_ctx = _canvas.getContext('2d', { willReadFrequently: false });
		return !!_ctx;
	}

	function _setCanvasSize(w, h) {
		if (_canvas.width !== w || _canvas.height !== h) {
			_canvas.width = w;
			_canvas.height = h;
		}
	}

	function _decodeRgb8(data, width, height, step) {
		var buf = _tempBuf;
		if (!buf || buf.length !== width * height * 4) {
			buf = new Uint8ClampedArray(width * height * 4);
			_tempBuf = buf;
		}
		var rowStride = step || width * 3;
		for (var y = 0; y < height; y++) {
			var srcOff = y * rowStride;
			var dstOff = y * width * 4;
			for (var x = 0; x < width; x++) {
				var si = srcOff + x * 3;
				var di = dstOff + x * 4;
				buf[di] = data[si];
				buf[di + 1] = data[si + 1];
				buf[di + 2] = data[si + 2];
				buf[di + 3] = 255;
			}
		}
		return new ImageData(buf, width, height);
	}

	function _decodeBgr8(data, width, height, step) {
		var buf = _tempBuf;
		if (!buf || buf.length !== width * height * 4) {
			buf = new Uint8ClampedArray(width * height * 4);
			_tempBuf = buf;
		}
		var rowStride = step || width * 3;
		for (var y = 0; y < height; y++) {
			var srcOff = y * rowStride;
			var dstOff = y * width * 4;
			for (var x = 0; x < width; x++) {
				var si = srcOff + x * 3;
				var di = dstOff + x * 4;
				buf[di] = data[si + 2];
				buf[di + 1] = data[si + 1];
				buf[di + 2] = data[si];
				buf[di + 3] = 255;
			}
		}
		return new ImageData(buf, width, height);
	}

	function _decodeMono8(data, width, height, step) {
		var buf = _tempBuf;
		if (!buf || buf.length !== width * height * 4) {
			buf = new Uint8ClampedArray(width * height * 4);
			_tempBuf = buf;
		}
		var rowStride = step || width;
		for (var y = 0; y < height; y++) {
			var srcOff = y * rowStride;
			var dstOff = y * width * 4;
			for (var x = 0; x < width; x++) {
				var v = data[srcOff + x];
				var di = dstOff + x * 4;
				buf[di] = v;
				buf[di + 1] = v;
				buf[di + 2] = v;
				buf[di + 3] = 255;
			}
		}
		return new ImageData(buf, width, height);
	}

	function _render(msg) {
		if (!_active) return;
		if (!_ensureCanvas()) return;
		var w = msg.width || 0;
		var h = msg.height || 0;
		if (!w || !h) return;
		var enc = (msg.encoding || '').toLowerCase();
		var data = msg.data;
		if (!data || !data.length) return;

		_setCanvasSize(w, h);

		var imageData = null;
		if (enc === 'rgb8') {
			imageData = _decodeRgb8(data, w, h, msg.step);
		} else if (enc === 'bgr8') {
			imageData = _decodeBgr8(data, w, h, msg.step);
		} else if (enc === 'mono8' || enc === '8uc1') {
			imageData = _decodeMono8(data, w, h, msg.step);
		} else {
			console.warn('[foxglove-image] unsupported encoding:', enc);
			return;
		}

		if (imageData) {
			_ctx.putImageData(imageData, 0, 0);
			if (typeof onFrame === 'function') {
				try { onFrame(_canvas, w, h); } catch (_) {}
			}
		}
	}

	function start() {
		if (_active) return;
		_active = true;
		if (!_ensureCanvas()) { _active = false; return; }

		var hub = globalThis.__transportHub;
		var foxAdapter = hub && hub._adapters && hub._adapters.get('foxglove');
		if (!foxAdapter || !foxAdapter.isConnected) {
			console.warn('[foxglove-image] foxglove adapter not available, retry in 1s');
			_active = false;
			setTimeout(function() { start(); }, 1000);
			// 也监听 foxglove 就绪事件，快速唤醒喵~
			if (hub && !_listeningForFox) {
				_listeningForFox = true;
				hub.addEventListener('ready', function _onFoxReady(e) {
					var list = e.detail && e.detail.connected;
					if (list && list.indexOf('foxglove') >= 0) {
						hub.removeEventListener('ready', _onFoxReady);
						_listeningForFox = false;
						start();
					}
				});
			}
			return;
		}

		var result = foxAdapter.subscribe(topic, 'sensor_msgs/msg/Image', function(typedMsg) {
			try { _render(typedMsg.data); } catch (e) {}
		});
		_subCancel = (result && typeof result.cancel === 'function') ? result.cancel : null;
		console.log('[foxglove-image] subscribed to', topic, 'via foxglove CDR');
	}

	function stop() {
		_active = false;
		if (_subCancel) { try { _subCancel(); } catch (_) {} _subCancel = null; }
		if (_canvas && _ctx) { _ctx.clearRect(0, 0, _canvas.width, _canvas.height); }
	}

	return { start: start, stop: stop, get isActive() { return _active; } };
}

export { createFoxgloveImageRenderer };
