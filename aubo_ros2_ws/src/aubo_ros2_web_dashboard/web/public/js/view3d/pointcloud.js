/**
 * 3D 点云适配层：
 * - 解析 `sensor_msgs/msg/PointCloud2`，做均匀抽样与颜色提取。
 * - 向 session 暴露兼容 `unsubscribe()` 的客户端对象。
 * - 保留旧版首帧日志、提示文案与 TF 订阅行为，便于现场排障。
 */
(function (global) {
	'use strict';

	const tfApi = global.IVGView3DTf;
	const hints = global.IVGView3DHints;
	if (!tfApi || !hints) throw new Error('IVGView3D pointcloud 依赖未加载');

	const normalizeFrameId = tfApi.normalizeFrameId;
	const removeView3dPc2Hint = hints.removeView3dPc2Hint;

	let ivgPc2NativeProcessMessage = null;

	function ivgRosPackedRgbFloatToColor(rgbFloat, littleEndian) {
		const ab = new ArrayBuffer(4);
		const dv = new DataView(ab);
		dv.setFloat32(0, rgbFloat, littleEndian);
		const u = dv.getUint32(0, littleEndian);
		const r = (u >> 16) & 255;
		const g = (u >> 8) & 255;
		const b = u & 255;
		return new THREE.Color(r / 255, g / 255, b / 255);
	}

	function ivgPc2DataToUint8Array(raw) {
		if (!raw) return null;
		if (raw instanceof Uint8Array) return raw;
		if (raw && typeof raw === 'object' && Array.isArray(raw.data)) return Uint8Array.from(raw.data);
		if (raw && typeof raw === 'object' && typeof raw.data === 'string') raw = raw.data;
		if (ArrayBuffer.isView(raw)) return new Uint8Array(raw.buffer, raw.byteOffset, raw.byteLength);
		if (raw instanceof ArrayBuffer) return new Uint8Array(raw);
		if (Array.isArray(raw)) return Uint8Array.from(raw);
		if (typeof raw === 'string' && typeof global.atob === 'function') {
			const bin = global.atob(raw);
			const out = new Uint8Array(bin.length);
			for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i) & 255;
			return out;
		}
		return null;
	}

	function ivgPc2ReadFieldValue(dv, base, field, littleEndian) {
		if (!field) return NaN;
		const off = base + (field.offset | 0);
		switch (field.datatype | 0) {
			case 1: return dv.getInt8(off);
			case 2: return dv.getUint8(off);
			case 3: return dv.getInt16(off, littleEndian);
			case 4: return dv.getUint16(off, littleEndian);
			case 5: return dv.getInt32(off, littleEndian);
			case 6: return dv.getUint32(off, littleEndian);
			case 7: return dv.getFloat32(off, littleEndian);
			case 8: return dv.getFloat64(off, littleEndian);
			default: return NaN;
		}
	}

	function ivgPc2ColorFromField(dv, base, field, littleEndian) {
		if (!field) return null;
		const dt = field.datatype | 0;
		if (dt === 5 || dt === 6) {
			const u = dv.getUint32(base + (field.offset | 0), littleEndian);
			return new THREE.Color(((u >> 16) & 255) / 255, ((u >> 8) & 255) / 255, (u & 255) / 255);
		}
		if (dt === 7 || dt === 8) {
			return ivgRosPackedRgbFloatToColor(ivgPc2ReadFieldValue(dv, base, field, littleEndian), littleEndian);
		}
		return null;
	}

	function ivgRosStampAgeMs(msg) {
		const stamp = msg && msg.header && msg.header.stamp;
		if (!stamp || typeof Date.now !== 'function') return NaN;
		const sec = Number(stamp.sec != null ? stamp.sec : stamp.secs);
		const nsec = Number(stamp.nanosec != null ? stamp.nanosec : stamp.nsecs);
		if (!Number.isFinite(sec) || !Number.isFinite(nsec)) return NaN;
		return Date.now() - (sec * 1000 + nsec / 1e6);
	}

	function IvgRosPointCloudClient(opts) {
		const o = opts || {};
		this.ros = o.ros || null;
		this.tfClient = o.tfClient || null;
		this.topicName = o.topic || '';
		this.rootObject = o.rootObject || null;
		this.max_pts = Math.max(1000, Number(o.max_pts) || 32000);
		this.messageRatio = Math.max(1, Number(o.messageRatio) || 1);
		this._msgSeq = 0;
		this._pendingMsg = null;
		this._processScheduled = false;
		this._processTimer = null;
		this._processingMsg = false;
		this._renderLogCount = 0;
		this._renderLogAccumMs = 0;
		this._renderLogLastAgeMs = NaN;
		this._frame = '';
		this._boundTfUpdate = this._applyTransform.bind(this);
		this._boundMessage = this._enqueueMessage.bind(this);
		this.group = new THREE.Object3D();
		this.group.visible = false;
		this.group.matrixAutoUpdate = true;
		this.geometry = new THREE.BufferGeometry();
		this.positions = new Float32Array(this.max_pts * 3);
		this.colors = new Float32Array(this.max_pts * 3);
		this.geometry.addAttribute('position', new THREE.BufferAttribute(this.positions, 3));
		this.geometry.addAttribute('color', new THREE.BufferAttribute(this.colors, 3));
		if (this.geometry.attributes.position && typeof this.geometry.attributes.position.setDynamic === 'function') {
			this.geometry.attributes.position.setDynamic(true);
		}
		if (this.geometry.attributes.color && typeof this.geometry.attributes.color.setDynamic === 'function') {
			this.geometry.attributes.color.setDynamic(true);
		}
		this.geometry.setDrawRange(0, 0);
		this.material = new THREE.PointsMaterial({
			size: Math.max(1, Number(o.pointSizePixels) || 6),
			sizeAttenuation: false,
			transparent: true,
			opacity: Number(o.opacity) > 0 ? Math.min(1, Number(o.opacity)) : 0.72,
			vertexColors: THREE.VertexColors
		});
		this.pointsObject = new THREE.Points(this.geometry, this.material);
		this.pointsObject.frustumCulled = false;
		this.group.add(this.pointsObject);
		if (this.rootObject && typeof this.rootObject.add === 'function') this.rootObject.add(this.group);
		this.points = { sn: this.group };
		this.__ivgPc2Sig = { got: false };
		this.__ivgPc2HintHost = o.hintHost || null;
		this.__ivgPc2Frame = '';
		this.__ivgPc2FirstFrameAt = 0;
		this.__ivgLoggedFirstFrame = false;
		this.__ivgLoggedBadPayload = false;
		this.compression = o.compression || 'cbor';
		if (this.ros && this.topicName && typeof ROSLIB !== 'undefined' && typeof ROSLIB.Topic === 'function') {
			this.topic = new ROSLIB.Topic({
				ros: this.ros,
				name: this.topicName,
				messageType: 'sensor_msgs/msg/PointCloud2',
				throttle_rate: Math.max(0, Number(o.throttle_rate) || 0),
				queue_length: 1,
				queue_size: 1,
				compression: this.compression
			});
			this.topic.subscribe(this._boundMessage);
		} else {
			this.topic = null;
		}
	}

	IvgRosPointCloudClient.prototype._scheduleProcess = function () {
		if (this._processScheduled) return;
		this._processScheduled = true;
		const run = () => {
			this._processScheduled = false;
			this._processTimer = null;
			this._drainLatestMessage();
		};
		if (typeof global.requestAnimationFrame === 'function') this._processTimer = global.requestAnimationFrame(run);
		else this._processTimer = global.setTimeout(run, 16);
	};

	IvgRosPointCloudClient.prototype._enqueueMessage = function (msg) {
		this._msgSeq += 1;
		if (this.messageRatio > 1 && ((this._msgSeq - 1) % this.messageRatio) !== 0) return;
		this._pendingMsg = msg;
		if (this._processingMsg) return;
		this._scheduleProcess();
	};

	IvgRosPointCloudClient.prototype._drainLatestMessage = function () {
		if (this._processingMsg) return;
		const msg = this._pendingMsg;
		this._pendingMsg = null;
		if (!msg) return;
		this._processingMsg = true;
		try {
			this._processMessage(msg);
		} finally {
			this._processingMsg = false;
		}
		if (this._pendingMsg) this._scheduleProcess();
	};

	IvgRosPointCloudClient.prototype._setFrame = function (frame) {
		const f = normalizeFrameId(frame);
		if (f === this._frame) return;
		if (this._frame && this.tfClient && typeof this.tfClient.unsubscribe === 'function') {
			this.tfClient.unsubscribe(this._frame, this._boundTfUpdate);
		}
		this._frame = f;
		this.__ivgPc2Frame = f;
		if (f && this.tfClient && typeof this.tfClient.subscribe === 'function') {
			this.tfClient.subscribe(f, this._boundTfUpdate);
		}
		this._applyTransform();
	};

	IvgRosPointCloudClient.prototype._applyTransform = function () {
		if (!this.group) return;
		const frame = this._frame;
		if (!frame) {
			this.group.visible = false;
			return;
		}
		if (!this.tfClient || normalizeFrameId(this.tfClient.fixedFrame) === frame) {
			this.group.position.set(0, 0, 0);
			this.group.quaternion.set(0, 0, 0, 1);
			this.group.visible = true;
			return;
		}
		const tf = typeof this.tfClient.getTransform === 'function' ? this.tfClient.getTransform(frame) : null;
		if (!tf) {
			this.group.visible = false;
			return;
		}
		this.group.position.set(
			Number(tf.translation && tf.translation.x) || 0,
			Number(tf.translation && tf.translation.y) || 0,
			Number(tf.translation && tf.translation.z) || 0
		);
		this.group.quaternion.set(
			Number(tf.rotation && tf.rotation.x) || 0,
			Number(tf.rotation && tf.rotation.y) || 0,
			Number(tf.rotation && tf.rotation.z) || 0,
			Number(tf.rotation && tf.rotation.w) || 1
		);
		this.group.visible = true;
	};

	IvgRosPointCloudClient.prototype._processMessage = function (msg) {
		const t0 = global.performance && typeof global.performance.now === 'function' ? global.performance.now() : Date.now();
		const bytes = ivgPc2DataToUint8Array(msg && msg.data);
		if (!bytes) {
			if (!this.__ivgLoggedBadPayload) {
				this.__ivgLoggedBadPayload = true;
				console.warn('[ivg/pc2] unsupported payload shape', msg && msg.data);
			}
			return;
		}
		const pointStep = (msg && msg.point_step) | 0;
		const total = Math.max(
			0,
			Math.min(((msg && msg.width) || 0) * ((msg && msg.height) || 0), pointStep > 0 ? Math.floor(bytes.byteLength / pointStep) : 0)
		);
		if (pointStep <= 0 || total <= 0) return;
		const fields = Array.isArray(msg.fields) ? msg.fields : [];
		const xField = fields.find(f => f && f.name === 'x');
		const yField = fields.find(f => f && f.name === 'y');
		const zField = fields.find(f => f && f.name === 'z');
		if (!xField || !yField || !zField) return;
		const rgbField = fields.find(f => f && (f.name === 'rgb' || f.name === 'rgba')) || null;
		const littleEndian = msg.is_bigendian !== true;
		const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
		const stride = total > this.max_pts ? Math.ceil(total / this.max_pts) : 1;
		let drawCount = 0;
		for (let i = 0; i < total && drawCount < this.max_pts; i += stride) {
			const base = i * pointStep;
			const x = ivgPc2ReadFieldValue(dv, base, xField, littleEndian);
			const y = ivgPc2ReadFieldValue(dv, base, yField, littleEndian);
			const z = ivgPc2ReadFieldValue(dv, base, zField, littleEndian);
			if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
			const dst = drawCount * 3;
			this.positions[dst] = x;
			this.positions[dst + 1] = y;
			this.positions[dst + 2] = z;
			const c = ivgPc2ColorFromField(dv, base, rgbField, littleEndian);
			this.colors[dst] = c ? c.r : 1;
			this.colors[dst + 1] = c ? c.g : 1;
			this.colors[dst + 2] = c ? c.b : 1;
			drawCount += 1;
		}
		this.geometry.attributes.position.needsUpdate = true;
		this.geometry.attributes.color.needsUpdate = true;
		this.geometry.setDrawRange(0, drawCount);
		if (!this.__ivgPc2FirstFrameAt) this.__ivgPc2FirstFrameAt = Date.now();
		this.__ivgPc2Sig.got = drawCount > 0;
		if (drawCount > 0 && !this.__ivgLoggedFirstFrame) {
			this.__ivgLoggedFirstFrame = true;
			const renderMs = (global.performance && typeof global.performance.now === 'function' ? global.performance.now() : Date.now()) - t0;
			const stampAgeMs = ivgRosStampAgeMs(msg);
			console.warn('[ivg/pc2] first frame rendered', {
				frame: msg && msg.header && msg.header.frame_id,
				drawCount,
				total,
				pointStep,
				renderMs: Number(renderMs.toFixed(1)),
				stampAgeMs: Number.isFinite(stampAgeMs) ? Number(stampAgeMs.toFixed(1)) : null
			});
		}
		if (drawCount > 0) {
			const renderMs = (global.performance && typeof global.performance.now === 'function' ? global.performance.now() : Date.now()) - t0;
			const stampAgeMs = ivgRosStampAgeMs(msg);
			this._renderLogCount += 1;
			this._renderLogAccumMs += renderMs;
			this._renderLogLastAgeMs = stampAgeMs;
			if (this._renderLogCount >= 20) {
				console.warn('[ivg/pc2] render stats', {
					avgRenderMs: Number((this._renderLogAccumMs / this._renderLogCount).toFixed(1)),
					lastStampAgeMs: Number.isFinite(this._renderLogLastAgeMs) ? Number(this._renderLogLastAgeMs.toFixed(1)) : null,
					drawCount,
					maxPts: this.max_pts,
					opacity: Number(this.material && this.material.opacity ? this.material.opacity : 1)
				});
				this._renderLogCount = 0;
				this._renderLogAccumMs = 0;
			}
		}
		if (drawCount > 0) removeView3dPc2Hint(this.__ivgPc2HintHost || null);
		this._setFrame(msg && msg.header && msg.header.frame_id);
	};

	IvgRosPointCloudClient.prototype.unsubscribe = function () {
		this._pendingMsg = null;
		if (this._processTimer != null) {
			if (typeof global.cancelAnimationFrame === 'function') global.cancelAnimationFrame(this._processTimer);
			else global.clearTimeout(this._processTimer);
			this._processTimer = null;
		}
		this._processScheduled = false;
		this._processingMsg = false;
		if (this.topic) {
			try { this.topic.unsubscribe(this._boundMessage); } catch (e) { /* ignore */ }
			this.topic = null;
		}
		if (this._frame && this.tfClient && typeof this.tfClient.unsubscribe === 'function') {
			this.tfClient.unsubscribe(this._frame, this._boundTfUpdate);
		}
		if (this.group && this.group.parent) this.group.parent.remove(this.group);
		if (this.geometry) this.geometry.dispose();
		if (this.material) this.material.dispose();
		this.group = null;
	};

	function ivgPc2RemoveLoadingHintOnFirstFrame(self) {
		const sig = self.__ivgPc2Sig;
		if (!sig || sig.got) return;
		sig.got = true;
		removeView3dPc2Hint(self.__ivgPc2HintHost || null);
	}

	function ivgPointCloud2FillUniformStride(self, msg) {
		const raw = msg.data;
		const total = (msg.width * msg.height) | 0;
		const ps = msg.point_step | 0;
		if (total <= 0 || ps <= 0 || !raw || raw.byteLength == null) return;
		const maxDraw = Math.min(self.max_pts, Math.floor(self.points.positions.array.length / 3));
		let stride = 1;
		if (total > maxDraw) stride = Math.ceil(total / maxDraw);
		let n = Math.min(maxDraw, Math.ceil(total / stride));
		const lastBase = (n - 1) * stride * ps + ps;
		if (lastBase > raw.byteLength) n = Math.max(0, Math.floor(raw.byteLength / (stride * ps)));
		const dv = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
		const littleEndian = msg.is_bigendian !== true;
		const xo = self.points.fields.x.offset;
		const yo = self.points.fields.y.offset;
		const zo = self.points.fields.z.offset;
		for (let i = 0; i < n; i++) {
			const base = i * stride * ps;
			self.points.positions.array[3 * i] = dv.getFloat32(base + xo, littleEndian);
			self.points.positions.array[3 * i + 1] = dv.getFloat32(base + yo, littleEndian);
			self.points.positions.array[3 * i + 2] = dv.getFloat32(base + zo, littleEndian);
			if (self.points.colors) {
				const color = self.points.colormap(self.points.getColor(dv, base, littleEndian));
				self.points.colors.array[3 * i] = color.r;
				self.points.colors.array[3 * i + 1] = color.g;
				self.points.colors.array[3 * i + 2] = color.b;
			}
		}
		self.points.update(n);
	}

	function installIvgPointCloud2ProcessPatch() {
		if (typeof ROS3D === 'undefined' || !ROS3D.PointCloud2) return;
		const P = ROS3D.PointCloud2.prototype;
		if (P.__ivgPc2ProcessPatched) return;
		if (typeof P.processMessage !== 'function') return;
		ivgPc2NativeProcessMessage = P.processMessage;
		P.__ivgPc2ProcessPatched = true;
		P.processMessage = function (msg) {
			this.__ivgPc2Le = msg.is_bigendian === true ? false : true;
			this.__ivgPc2Frame = normalizeFrameId(msg && msg.header && msg.header.frame_id);
			if (!this.__ivgPc2FirstFrameAt) this.__ivgPc2FirstFrameAt = Date.now();
			const rgbF = (msg.fields || []).find(f => f && f.name === 'rgb');
			this.__ivgRgbFieldDt = rgbF ? rgbF.datatype : 0;
			const typedArray = msg.data && typeof msg.data !== 'string' && msg.data.buffer && typeof msg.data.buffer === 'object';
			if (this.__ivgUniformStride && typedArray) {
				if (!this.points.setup(msg.header.frame_id, msg.point_step, msg.fields)) return;
				ivgPc2RemoveLoadingHintOnFirstFrame(this);
				this.buffer = msg.data;
				ivgPointCloud2FillUniformStride(this, msg);
				return;
			}
			ivgPc2RemoveLoadingHintOnFirstFrame(this);
			return ivgPc2NativeProcessMessage.call(this, msg);
		};
	}

	global.IVGView3DPointCloud = {
		ivgRosPackedRgbFloatToColor,
		ivgPc2DataToUint8Array,
		ivgPc2ReadFieldValue,
		ivgPc2ColorFromField,
		ivgRosStampAgeMs,
		IvgRosPointCloudClient,
		installIvgPointCloud2ProcessPatch
	};
})(typeof window !== 'undefined' ? window : globalThis);
