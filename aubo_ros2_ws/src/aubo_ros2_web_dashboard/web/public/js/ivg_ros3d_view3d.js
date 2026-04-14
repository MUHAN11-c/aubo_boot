/**
 * 与 Robot Web Tools ros3djs 官方示例一致的三维视图：
 * ROS3D.Viewer + 官方可视化类（PointCloud2 / LaserScan / MarkerArrayClient / Urdf）。
 * TF 优先按官方固定坐标系思路工作；在 ROS2 + rosbridge 现场，为避免 tf2_web_republisher / action 链路不稳，
 * 这里改为直接订阅 `/tf` / `/tf_static` 在浏览器端维护轻量 TF 树，再喂给 ros3d 的 SceneNode。
 *
 * 参考：https://github.com/RobotWebTools/ros3djs/blob/develop/examples/pointcloud2.html
 * 依赖：roslib@2（ROS2TFClient）、three r89、ros3d.min.js（见 web/public/js/vendor/）
 */
(function (global) {
	'use strict';

	let ivgPc2NativeProcessMessage = null;
	let ivgThreeObjectAddPatched = false;

	function normalizeFrameId(frame) {
		return String(frame || '').trim().replace(/^\/+/, '');
	}

	function installIvgThreeSafeAddPatch() {
		if (ivgThreeObjectAddPatched) return;
		if (typeof THREE === 'undefined' || !THREE.Object3D || typeof THREE.Object3D.prototype.add !== 'function') return;
		const nativeAdd = THREE.Object3D.prototype.add;
		THREE.Object3D.prototype.add = function () {
			const filtered = [];
			for (let i = 0; i < arguments.length; i++) {
				const obj = arguments[i];
				if (!obj || obj.isObject3D !== true) {
					try {
						console.warn(
							`[ivg/three] skipped invalid Object3D.add argument type=${Object.prototype.toString.call(obj)}`
						);
					} catch (e) {
						/* ignore */
					}
					continue;
				}
				filtered.push(obj);
			}
			if (filtered.length === 0) return this;
			return nativeAdd.apply(this, filtered);
		};
		ivgThreeObjectAddPatched = true;
	}

	function ivgIdentityTransform() {
		return {
			translation: { x: 0, y: 0, z: 0 },
			rotation: { x: 0, y: 0, z: 0, w: 1 }
		};
	}

	function ivgCloneTransform(tf) {
		const src = tf || {};
		const tr = src.translation || {};
		const rot = src.rotation || {};
		return {
			translation: {
				x: Number(tr.x) || 0,
				y: Number(tr.y) || 0,
				z: Number(tr.z) || 0
			},
			rotation: {
				x: Number(rot.x) || 0,
				y: Number(rot.y) || 0,
				z: Number(rot.z) || 0,
				w: Number(rot.w) || 1
			}
		};
	}

	function ivgComposeTransforms(a, b) {
		const ta = ivgCloneTransform(a);
		const tb = ivgCloneTransform(b);
		const qa = new THREE.Quaternion(ta.rotation.x, ta.rotation.y, ta.rotation.z, ta.rotation.w);
		const qb = new THREE.Quaternion(tb.rotation.x, tb.rotation.y, tb.rotation.z, tb.rotation.w);
		const va = new THREE.Vector3(ta.translation.x, ta.translation.y, ta.translation.z);
		const vb = new THREE.Vector3(tb.translation.x, tb.translation.y, tb.translation.z).applyQuaternion(qa);
		const q = qa.clone().multiply(qb).normalize();
		const v = va.add(vb);
		return {
			translation: { x: v.x, y: v.y, z: v.z },
			rotation: { x: q.x, y: q.y, z: q.z, w: q.w }
		};
	}

	function ivgInvertTransform(tf) {
		const src = ivgCloneTransform(tf);
		const q = new THREE.Quaternion(src.rotation.x, src.rotation.y, src.rotation.z, src.rotation.w).normalize();
		const qi = q.clone();
		if (typeof qi.invert === 'function') {
			qi.invert();
		} else {
			qi.inverse();
		}
		const t = new THREE.Vector3(src.translation.x, src.translation.y, src.translation.z)
			.multiplyScalar(-1)
			.applyQuaternion(qi);
		return {
			translation: { x: t.x, y: t.y, z: t.z },
			rotation: { x: qi.x, y: qi.y, z: qi.z, w: qi.w }
		};
	}

	function ivgBuildTfPath(frame, edges) {
		const out = [{ frame, transform: ivgIdentityTransform() }];
		const seen = new Set([frame]);
		let cur = frame;
		let acc = ivgIdentityTransform();
		for (let i = 0; i < 256; i++) {
			const edge = edges[cur];
			if (!edge || !edge.parent) break;
			acc = ivgComposeTransforms(edge.transform, acc);
			cur = edge.parent;
			if (seen.has(cur)) break;
			seen.add(cur);
			out.push({ frame: cur, transform: acc });
		}
		return out;
	}

	function ivgFindRelativeTransform(sourceFrame, targetFrame, edges) {
		const src = normalizeFrameId(sourceFrame);
		const dst = normalizeFrameId(targetFrame);
		if (!src || !dst) return null;
		if (src === dst) return ivgIdentityTransform();
		const srcPath = ivgBuildTfPath(src, edges);
		const dstPath = ivgBuildTfPath(dst, edges);
		const srcMap = Object.create(null);
		srcPath.forEach(entry => {
			srcMap[entry.frame] = entry.transform;
		});
		for (let i = 0; i < dstPath.length; i++) {
			const entry = dstPath[i];
			if (!Object.prototype.hasOwnProperty.call(srcMap, entry.frame)) continue;
			return ivgComposeTransforms(ivgInvertTransform(srcMap[entry.frame]), entry.transform);
		}
		return null;
	}

	function IvgRosTfTreeClient(opts) {
		const o = opts || {};
		this.ros = o.ros || null;
		this.fixedFrame = normalizeFrameId(o.fixedFrame || 'base_link') || 'base_link';
		this.frameInfos = Object.create(null);
		this._edges = Object.create(null);
		this._topics = [];
		this._boundHandleTf = this._handleTfMessage.bind(this);
		this._subscribeTopics();
	}

	IvgRosTfTreeClient.prototype._subscribeTopics = function () {
		if (!this.ros || typeof ROSLIB === 'undefined' || typeof ROSLIB.Topic !== 'function') return;
		const topicNames = ['/tf', '/tf_static'];
		topicNames.forEach(name => {
			const t = new ROSLIB.Topic({
				ros: this.ros,
				name,
				messageType: 'tf2_msgs/msg/TFMessage',
				throttle_rate: 0,
				queue_length: 1
			});
			t.subscribe(this._boundHandleTf);
			this._topics.push(t);
		});
	};

	IvgRosTfTreeClient.prototype._handleTfMessage = function (msg) {
		const transforms = (msg && Array.isArray(msg.transforms)) ? msg.transforms : [];
		let changed = false;
		for (let i = 0; i < transforms.length; i++) {
			const tf = transforms[i];
			const parent = normalizeFrameId(tf && tf.header && tf.header.frame_id);
			const child = normalizeFrameId(tf && tf.child_frame_id);
			if (!parent || !child) continue;
			this._edges[child] = {
				parent,
				transform: ivgCloneTransform(tf.transform)
			};
			changed = true;
		}
		if (changed) this._refreshAll();
	};

	IvgRosTfTreeClient.prototype._refreshAll = function () {
		const names = Object.keys(this.frameInfos);
		for (let i = 0; i < names.length; i++) {
			const frame = names[i];
			const info = this.frameInfos[frame];
			if (!info || !info.cbs || info.cbs.length === 0) continue;
			const tf = this.getTransform(frame);
			if (!tf) continue;
			info.transform = tf;
			info.cbs.slice().forEach(cb => {
				try {
					cb(tf);
				} catch (e) {
					/* ignore */
				}
			});
		}
	};

	IvgRosTfTreeClient.prototype.getTransform = function (frame) {
		return ivgFindRelativeTransform(this.fixedFrame, frame, this._edges);
	};

	IvgRosTfTreeClient.prototype.hasTransformFor = function (frame) {
		return !!this.getTransform(frame);
	};

	IvgRosTfTreeClient.prototype.setFixedFrame = function (frame) {
		const f = normalizeFrameId(frame);
		if (!f || f === this.fixedFrame) return false;
		this.fixedFrame = f;
		this._refreshAll();
		return true;
	};

	IvgRosTfTreeClient.prototype.subscribe = function (frame, cb) {
		const f = normalizeFrameId(frame);
		if (!f || typeof cb !== 'function') return;
		if (!this.frameInfos[f]) this.frameInfos[f] = { cbs: [], transform: null };
		this.frameInfos[f].cbs.push(cb);
		const tf = this.getTransform(f);
		if (tf) {
			this.frameInfos[f].transform = tf;
			cb(tf);
		}
	};

	IvgRosTfTreeClient.prototype.unsubscribe = function (frame, cb) {
		const f = normalizeFrameId(frame);
		if (!f || !this.frameInfos[f]) return;
		if (typeof cb === 'function') {
			this.frameInfos[f].cbs = this.frameInfos[f].cbs.filter(fn => fn !== cb);
		} else {
			this.frameInfos[f].cbs = [];
		}
		if (this.frameInfos[f].cbs.length === 0) delete this.frameInfos[f];
	};

	IvgRosTfTreeClient.prototype.dispose = function () {
		this._topics.forEach(t => {
			try {
				t.unsubscribe(this._boundHandleTf);
			} catch (e) {
				/* ignore */
			}
		});
		this._topics.length = 0;
		this.frameInfos = Object.create(null);
		this._edges = Object.create(null);
	};

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
			case 1:
				return dv.getInt8(off);
			case 2:
				return dv.getUint8(off);
			case 3:
				return dv.getInt16(off, littleEndian);
			case 4:
				return dv.getUint16(off, littleEndian);
			case 5:
				return dv.getInt32(off, littleEndian);
			case 6:
				return dv.getUint32(off, littleEndian);
			case 7:
				return dv.getFloat32(off, littleEndian);
			case 8:
				return dv.getFloat64(off, littleEndian);
			default:
				return NaN;
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
			console.warn('[ivg/pc2] subscribe', {
				topic: this.topicName,
				compression: this.compression,
				throttle_rate: Math.max(0, Number(o.throttle_rate) || 0),
				messageRatio: this.messageRatio
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
		if (typeof global.requestAnimationFrame === 'function') {
			this._processTimer = global.requestAnimationFrame(run);
		} else {
			this._processTimer = global.setTimeout(run, 16);
		}
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
		const tf =
			typeof this.tfClient.getTransform === 'function'
				? this.tfClient.getTransform(frame)
				: null;
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
			Math.min(
				((msg && msg.width) || 0) * ((msg && msg.height) || 0),
				pointStep > 0 ? Math.floor(bytes.byteLength / pointStep) : 0
			)
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
					lastStampAgeMs: Number.isFinite(this._renderLogLastAgeMs)
						? Number(this._renderLogLastAgeMs.toFixed(1))
						: null,
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
			try {
				this.topic.unsubscribe(this._boundMessage);
			} catch (e) {
				/* ignore */
			}
			this.topic = null;
		}
		if (this._frame && this.tfClient && typeof this.tfClient.unsubscribe === 'function') {
			this.tfClient.unsubscribe(this._frame, this._boundTfUpdate);
		}
		if (this.group && this.group.parent) {
			this.group.parent.remove(this.group);
		}
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

	/** max_pts 下按步长均匀抽样（ros3d 原版只画前 max_pts 个连续点，大图会缺大半）。 */
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
		if (lastBase > raw.byteLength) {
			n = Math.max(0, Math.floor(raw.byteLength / (stride * ps)));
		}
		const dv = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
		const littleEndian = msg.is_bigendian !== true;
		const xo = self.points.fields.x.offset;
		const yo = self.points.fields.y.offset;
		const zo = self.points.fields.z.offset;
		let base;
		let color;
		for (let i = 0; i < n; i++) {
			base = i * stride * ps;
			self.points.positions.array[3 * i] = dv.getFloat32(base + xo, littleEndian);
			self.points.positions.array[3 * i + 1] = dv.getFloat32(base + yo, littleEndian);
			self.points.positions.array[3 * i + 2] = dv.getFloat32(base + zo, littleEndian);
			if (self.points.colors) {
				color = self.points.colormap(self.points.getColor(dv, base, littleEndian));
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

			const typedArray =
				msg.data && typeof msg.data !== 'string' && msg.data.buffer && typeof msg.data.buffer === 'object';
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

	function removeView3dPc2Hint(hostEl) {
		if (hostEl) {
			hostEl.querySelectorAll('.ivg-pc2-hint').forEach(n => n.remove());
		}
		const leg = document.getElementById('view3d-pc2-hint');
		if (leg) leg.remove();
	}

	function showView3dPc2Hint(hostEl, html) {
		removeView3dPc2Hint(hostEl);
		if (!hostEl) return;
		if (hostEl.hasAttribute('data-ivg-hide-hints')) return;
		const d = document.createElement('div');
		d.className = 'hint ivg-pc2-hint';
		d.style.marginTop = '0.5rem';
		d.innerHTML = html;
		hostEl.appendChild(d);
	}

	function removeView3dUrdfHint(hostEl) {
		if (hostEl) {
			hostEl.querySelectorAll('.ivg-urdf-hint').forEach(n => n.remove());
		}
		const leg = document.getElementById('view3d-urdf-hint');
		if (leg) leg.remove();
	}

	function showView3dUrdfHint(hostEl, html) {
		removeView3dUrdfHint(hostEl);
		if (!hostEl) return;
		if (hostEl.hasAttribute('data-ivg-hide-hints')) return;
		const d = document.createElement('div');
		d.className = 'hint ivg-urdf-hint';
		d.style.marginTop = '0.5rem';
		d.innerHTML = html;
		hostEl.appendChild(d);
	}

	function ivgDecodeRosapiParamString(raw) {
		if (raw == null || raw === '') return '';
		if (typeof raw === 'string') {
			try {
				const v = JSON.parse(raw);
				return typeof v === 'string' ? v : '';
			} catch (e0) {
				return raw;
			}
		}
		return '';
	}

	function ivgExtractRosapiParamValue(resp) {
		if (!resp || typeof resp !== 'object') return '';
		if (Object.prototype.hasOwnProperty.call(resp, 'value')) return resp.value;
		if (resp.values && Object.prototype.hasOwnProperty.call(resp.values, 'value')) return resp.values.value;
		return '';
	}

	function ivgBuildParamNameCandidates(paramFullName) {
		const raw = String(paramFullName || '').trim();
		const out = [];
		function push(v) {
			const name = String(v || '').trim();
			if (!name || out.indexOf(name) !== -1) return;
			out.push(name);
		}
		push(raw);
		push(raw.replace(/^\/+/, ''));
		if (raw.indexOf(':') !== -1) push(raw.slice(raw.indexOf(':') + 1));
		if (raw.indexOf('/') !== -1) push(raw.slice(raw.lastIndexOf('/') + 1));
		return out;
	}

	function ivgAttachUrdfFromRosParam(ros, paramFullName, meshBase, tfClient, rootGroup, $, onErr) {
		if (!ros || typeof ROSLIB.Service !== 'function') {
			onErr('ROSLIB.Service 不可用');
			return;
		}
		const serviceTypes = ['rosapi/GetParam', 'rosapi/srv/GetParam'];
		const paramNames = ivgBuildParamNameCandidates(paramFullName);
		function attachFromXml(xml) {
			const looks =
				xml.indexOf('<robot') !== -1 || xml.indexOf('<?xml') !== -1 || xml.indexOf('<urdf') !== -1;
			if (!looks) {
				onErr(`返回值不像 URDF XML（开头）：${JSON.stringify(xml.slice(0, 96))}`);
				return;
			}
			try {
				const urdfModel = new ROSLIB.UrdfModel({ string: xml });
				const urdfViz = new ROS3D.Urdf({
					urdfModel,
					path: meshBase,
					tfClient,
					tfPrefix: ''
				});
				rootGroup.add(urdfViz);
				const h = $ && $('view3d-host');
				if (h) {
					showView3dUrdfHint(
						h,
						'<strong>机械臂</strong>：URDF 已解析，网格异步加载中；若仍不可见请看 Network 是否对 <code>/api/ivg/robot-mesh/…</code> 404。'
					);
					setTimeout(() => removeView3dUrdfHint(h), 10000);
				}
			} catch (e2) {
				onErr(e2 && e2.message ? e2.message : String(e2));
			}
		}
		function tryParamName(typeIdx, nameIdx, lastErr) {
			if (typeIdx >= serviceTypes.length) {
				onErr(lastErr || 'get_param 返回空或无法解析为字符串（请核对「URDF 参数」全名）');
				return;
			}
			if (nameIdx >= paramNames.length) {
				tryParamName(typeIdx + 1, 0, lastErr);
				return;
			}
			const svc = new ROSLIB.Service({
				ros,
				name: '/rosapi/get_param',
				serviceType: serviceTypes[typeIdx]
			});
			svc.callService(
				{ name: paramNames[nameIdx], default_value: '' },
				resp => {
					const rawVal = ivgExtractRosapiParamValue(resp);
					const xml = ivgDecodeRosapiParamString(rawVal).trim();
					if (!xml) {
						tryParamName(
							typeIdx,
							nameIdx + 1,
							`get_param 返回空或无法解析为字符串（已尝试：${paramNames.join(' / ')}）`
						);
						return;
					}
					attachFromXml(xml);
				},
				err => {
					const msg = err && err.message ? err.message : String(err);
					tryParamName(typeIdx, nameIdx + 1, msg);
				}
			);
		}
		tryParamName(0, 0, '');
	}

	/**
	 * @param {object} ros ROSLIB.Ros 已连接实例
	 * @param {(id:string) => HTMLElement|null} $ getElementById
	 * @param {object} [opts]
	 * @param {string} [opts.viewerInnerId] 传给 ROS3D.Viewer 的 div id（默认 view3d-inner，多视图时需唯一）
	 * @param {number} [opts.viewerHeight] 固定高度像素；未设则按宿主宽度估算
	 */
	function IvgRos3dView3dSession(ros, $, opts) {
		this.ros = ros;
		this.$ = $;
		this.opts = opts || {};
		this._viewerInnerId = this.opts.viewerInnerId || 'view3d-inner';
		const vh = this.opts.viewerHeight;
		this._viewerHeight = typeof vh === 'number' && vh > 0 ? vh : null;
		this.viewer3d = null;
		this.tfClient3d = null;
		this.ros3dPointCloud2 = null;
		this.ros3dLaserScan = null;
		this.ros3dMarkerClient = null;
		this.ros3dMarkerRoot = null;
		this.ros3dAxes = null;
		this.ros3dGrid = null;
		this.ros3dUrdfRoot = null;
		this._pcAutoFrameFallbackTimer = null;
		this._pcAutoFrameFallbackUsed = false;
		this._pcCameraPrimed = false;
		this._deferredStartTimers = [];
		this._markersStarted = false;
		this._urdfStarted = false;
		this._markerFocusTimer = null;
		this._markerCameraPrimed = false;
	}

	IvgRos3dView3dSession.prototype._focusViewerOnObject = function (obj3d, opts) {
		if (!this.viewer3d || !this.viewer3d.camera || !obj3d) return;
		const options = opts || {};
		const center = new THREE.Vector3();
		let radius = 0.25;
		try {
			const box = new THREE.Box3().setFromObject(obj3d);
			if (!box.isEmpty()) {
				box.getCenter(center);
				const sphere = box.getBoundingSphere(new THREE.Sphere());
				radius = Math.max(0.08, Number(sphere && sphere.radius) || 0.25);
			} else if (typeof obj3d.getWorldPosition === 'function') {
				obj3d.getWorldPosition(center);
			}
		} catch (e) {
			if (typeof obj3d.getWorldPosition === 'function') obj3d.getWorldPosition(center);
		}
		const camera = this.viewer3d.camera;
		const distanceFactor = Math.max(1.2, Number(options.distanceFactor) || 2.6);
		const dist = Math.max(Number(options.minDistance) || 0.45, radius * distanceFactor);
		camera.position.set(center.x + dist, center.y + dist * 0.72, center.z + Math.max(dist * 0.52, radius * 1.8));
		camera.lookAt(center);
		if (this.viewer3d.cameraControls) {
			this.viewer3d.cameraControls.center = center.clone();
			if (typeof this.viewer3d.cameraControls.update === 'function') {
				this.viewer3d.cameraControls.update();
			}
		}
	};

	IvgRos3dView3dSession.prototype._defer = function (fn, delayMs) {
		const tid = setTimeout(() => {
			this._deferredStartTimers = this._deferredStartTimers.filter(x => x !== tid);
			try {
				fn();
			} catch (e) {
				console.error('[ivg/view3d] deferred stage failed:', e);
			}
		}, delayMs);
		this._deferredStartTimers.push(tid);
	};

	IvgRos3dView3dSession.prototype._startMarkersStage = function (mk) {
		if (this._markersStarted || !mk) return;
		this._markersStarted = true;
		this.ros3dMarkerRoot = new THREE.Object3D();
		this.viewer3d.addObject(this.ros3dMarkerRoot);
		const MarkerCtor =
			typeof ROS3D.MarkerArrayClient === 'function'
				? ROS3D.MarkerArrayClient
				: ROS3D.MarkerClient;
		const markerOpts = {
			ros: this.ros,
			tfClient: this.tfClient3d,
			topic: mk,
			rootObject: this.ros3dMarkerRoot
		};
		if (MarkerCtor === ROS3D.MarkerClient) {
			markerOpts.lifetime = 0;
		}
		console.warn(
			`[ivg/markers] start topic=${mk} ctor=${MarkerCtor === ROS3D.MarkerArrayClient ? 'MarkerArrayClient' : 'MarkerClient'}`
		);
		this.ros3dMarkerClient = new MarkerCtor(markerOpts);
		this._markerFocusTimer = setInterval(() => {
			if (!this.ros3dMarkerRoot) return;
			if (this.ros3dMarkerRoot.children && this.ros3dMarkerRoot.children.length > 0 && !this._markerCameraPrimed) {
				this._focusViewerOnObject(this.ros3dMarkerRoot, { distanceFactor: 3.2, minDistance: 0.72 });
				this._markerCameraPrimed = true;
				clearInterval(this._markerFocusTimer);
				this._markerFocusTimer = null;
			}
		}, 250);
	};

	IvgRos3dView3dSession.prototype._startUrdfStage = function ($, host, fixedFrame) {
		if (this._urdfStarted) return;
		this._urdfStarted = true;
		this.ros3dUrdfRoot = new THREE.Object3D();
		this.viewer3d.addObject(this.ros3dUrdfRoot);
		const pName = (($('urdf-param') && $('urdf-param').value) || '/robot_state_publisher:robot_description').trim();
		const meshBase = `${global.location.origin}/api/ivg/robot-mesh/`;
		showView3dUrdfHint(
			host,
			`<strong>机械臂 URDF</strong>：阶段式加载中；若仍失败，不会再阻断点云与 Marker 主视图。`
		);
		ivgAttachUrdfFromRosParam(this.ros, pName, meshBase, this.tfClient3d, this.ros3dUrdfRoot, $, err => {
			console.error('URDF:', err);
			showView3dUrdfHint(
				host,
				`<strong>机械臂加载失败</strong>：${String(err)}。当前 3D 主视图已继续工作；请单独检查 <code>${fixedFrame}</code> 与网格资源。`
			);
		});
	};

	IvgRos3dView3dSession.prototype.stop = function () {
		this._deferredStartTimers.forEach(t => clearTimeout(t));
		this._deferredStartTimers.length = 0;
		if (this._pcAutoFrameFallbackTimer) {
			clearInterval(this._pcAutoFrameFallbackTimer);
			this._pcAutoFrameFallbackTimer = null;
		}
		this._pcAutoFrameFallbackUsed = false;
		this._pcCameraPrimed = false;
		this._markersStarted = false;
		this._urdfStarted = false;
		this._markerCameraPrimed = false;
		if (this._markerFocusTimer) {
			clearInterval(this._markerFocusTimer);
			this._markerFocusTimer = null;
		}
		if (this.ros3dMarkerClient) {
			try {
				this.ros3dMarkerClient.unsubscribe();
			} catch (e0) {
				/* ignore */
			}
			this.ros3dMarkerClient = null;
		}
		if (this.ros3dPointCloud2) {
			try {
				this.ros3dPointCloud2.unsubscribe();
			} catch (e1) {
				/* ignore */
			}
			this.ros3dPointCloud2 = null;
		}
		if (this.ros3dLaserScan) {
			try {
				this.ros3dLaserScan.unsubscribe();
			} catch (e2) {
				/* ignore */
			}
			this.ros3dLaserScan = null;
		}
		if (this.viewer3d) {
			try {
				if (this.ros3dUrdfRoot) {
					this.viewer3d.scene.remove(this.ros3dUrdfRoot);
					this.ros3dUrdfRoot = null;
				}
				if (this.ros3dMarkerRoot) {
					this.viewer3d.scene.remove(this.ros3dMarkerRoot);
					this.ros3dMarkerRoot = null;
				}
				if (this.ros3dAxes) {
					this.viewer3d.scene.remove(this.ros3dAxes);
					this.ros3dAxes = null;
				}
				if (this.ros3dGrid) {
					this.viewer3d.scene.remove(this.ros3dGrid);
					this.ros3dGrid = null;
				}
			} catch (e3a) {
				/* ignore */
			}
			try {
				this.viewer3d.stop();
			} catch (e4) {
				/* ignore */
			}
			this.viewer3d = null;
		} else {
			this.ros3dMarkerRoot = null;
			this.ros3dUrdfRoot = null;
			this.ros3dAxes = null;
			this.ros3dGrid = null;
		}
		if (this.tfClient3d) {
			try {
				this.tfClient3d.dispose();
			} catch (e3) {
				/* ignore */
			}
			this.tfClient3d = null;
		}
		const host3 = this.$('view3d-host');
		removeView3dPc2Hint(host3);
		removeView3dUrdfHint(host3);
		if (host3) host3.innerHTML = '';
	};

	IvgRos3dView3dSession.prototype.start = function () {
		const $ = this.$;
		const ros = this.ros;
		const pcn = (($('pc-topic') && $('pc-topic').value) || '').trim();
		const sn = (($('scan3-topic') && $('scan3-topic').value) || '').trim();
		const mk = (($('marker3-topic') && $('marker3-topic').value) || '').trim();
		const urdfEl = $('view3d-show-urdf');
		const wantUrdf = !urdfEl || urdfEl.checked;
		if (!pcn && !sn && !mk && !wantUrdf) {
			alert('请至少填写「点云」「雷达」或「Marker」话题之一，或勾选「显示机械臂 URDF」');
			return;
		}
		if (mk && typeof ROS3D.MarkerArrayClient === 'undefined' && typeof ROS3D.MarkerClient === 'undefined') {
			alert('已填写 Marker 话题但未加载 ROS3D.MarkerArrayClient / MarkerClient（需 ros3d.min.js）');
			return;
		}
		if (wantUrdf && !pcn && !sn && !mk) {
			if (typeof ROS3D.Urdf !== 'function' || typeof ROSLIB.UrdfModel !== 'function') {
				alert('当前脚本缺少 ROS3D.Urdf / ROSLIB.UrdfModel，无法显示机械臂；请更新 ros3d / roslib');
				return;
			}
		}
		this.stop();
		if (typeof ROS3D === 'undefined' || typeof ROSLIB === 'undefined') {
			alert('ROS3D 或 ROSLIB 未加载（需 roslib-2.iife.js + three r89 + ros3d）');
			return;
		}
		installIvgThreeSafeAddPatch();
		const host = $('view3d-host');
		const innerId = this._viewerInnerId;
		const inner = document.createElement('div');
		inner.id = innerId;
		if (host) host.appendChild(inner);
		if (host) host.setAttribute('data-ivg-hide-hints', '1');
		const w = (host && host.clientWidth) || 800;
		let h = 800;
		if (this._viewerHeight != null) {
			h = this._viewerHeight;
		} else if (host) {
			h = Math.max(240, host.clientHeight || Math.round(w * 0.75));
		}
		this.viewer3d = new ROS3D.Viewer({
			divID: innerId,
			width: w,
			height: h,
			antialias: false,
			background: '#ffffff',
			cameraPose: { x: 3, y: 3, z: 3 },
			cameraZoomSpeed: 0.5
		});
		const axEl = $('view3d-show-axes');
		const grEl = $('view3d-show-grid');
		if (!axEl || axEl.checked) {
			this.ros3dAxes = new ROS3D.Axes();
			this.viewer3d.addObject(this.ros3dAxes);
		}
		if (!grEl || grEl.checked) {
			this.ros3dGrid = new ROS3D.Grid({ num_cells: 12, cellSize: 1, color: '#94a3b8' });
			this.viewer3d.addObject(this.ros3dGrid);
		}
		let fixedFrame = (($('tf-fixed-frame') && $('tf-fixed-frame').value) || 'base_link').trim().replace(/^\/+/, '');
		if (fixedFrame === '') fixedFrame = 'base_link';
		this.tfClient3d = new IvgRosTfTreeClient({ ros, fixedFrame });
		const maxPts = Math.max(1000, parseInt(($('pc-max') && $('pc-max').value) || '32000', 10) || 32000);
		const pszEl = $('view3d-point-size');
		const ptSize = Math.max(0.01, parseFloat((pszEl && pszEl.value) || '0.05') || 0.05);
		const pcThrottleEl = $('pc-throttle-ms');
		const pcRatioEl = $('pc-msg-ratio');
		const pcThrottleMsRaw = Math.max(0, parseInt((pcThrottleEl && pcThrottleEl.value) || '120', 10) || 0);
		const pcMsgRatioRaw = Math.max(1, parseInt((pcRatioEl && pcRatioEl.value) || '2', 10) || 1);
		const coarsePointer =
			typeof global.matchMedia === 'function' && global.matchMedia('(pointer: coarse)').matches;
		const dpr = typeof global.devicePixelRatio === 'number' ? global.devicePixelRatio : 1;
		if (coarsePointer && this.viewer3d && this.viewer3d.renderer && typeof this.viewer3d.renderer.setPixelRatio === 'function') {
			this.viewer3d.renderer.setPixelRatio(1);
		}
		const effectiveMaxPts = coarsePointer ? Math.min(maxPts, 4500) : maxPts;
		const pcThrottleMs = coarsePointer ? Math.max(pcThrottleMsRaw, 220) : pcThrottleMsRaw;
		const pcMsgRatio = coarsePointer ? Math.max(pcMsgRatioRaw, 4) : pcMsgRatioRaw;
		const selfSession = this;
		if (pcn) {
			const basePixelSize = Math.max(2, Math.min(24, Math.round(ptSize * 120)));
			const pcPixelSize = coarsePointer
				? 1
				: basePixelSize;
			this.ros3dPointCloud2 = new IvgRosPointCloudClient({
				ros,
				tfClient: this.tfClient3d,
				rootObject: this.viewer3d.scene,
				topic: pcn,
				max_pts: effectiveMaxPts,
				messageRatio: pcMsgRatio,
				throttle_rate: pcThrottleMs,
				pointSizePixels: pcPixelSize,
				opacity: coarsePointer ? 0.9 : 0.72,
				hintHost: host
			});
			showView3dPc2Hint(
				host,
				`<strong>加载点云</strong>：首帧大图仍可能需数秒。当前 <strong>节流 ${pcThrottleMs} ms</strong>、<strong>隔帧 ${pcMsgRatio}</strong>；若仍慢请调大节流、订阅 <code>…/points_web</code> 或调低「最大点数」。`
			);
			this._pcAutoFrameFallbackTimer = setInterval(() => {
				const pc2 = selfSession.ros3dPointCloud2;
				if (!pc2 || !pc2.__ivgPc2Sig || !pc2.__ivgPc2Sig.got) return;
				const pcSceneNode = pc2.points && pc2.points.sn;
				const pcFrame = normalizeFrameId(pc2.__ivgPc2Frame || (pcSceneNode && pcSceneNode.frameID));
				if (!pcFrame) return;
				if (pcSceneNode && !selfSession._pcCameraPrimed && !selfSession._markerCameraPrimed) {
					selfSession._focusViewerOnObject(pcSceneNode, { distanceFactor: 2.4, minDistance: 0.42 });
					selfSession._pcCameraPrimed = true;
				}
				if (pcSceneNode && pcSceneNode.visible === true) {
					clearInterval(selfSession._pcAutoFrameFallbackTimer);
					selfSession._pcAutoFrameFallbackTimer = null;
				} else {
					const firstFrameAt = pc2.__ivgPc2FirstFrameAt || 0;
					if (firstFrameAt && Date.now() - firstFrameAt > 3000) {
						clearInterval(selfSession._pcAutoFrameFallbackTimer);
						selfSession._pcAutoFrameFallbackTimer = null;
					}
				}
			}, 1000);
		}
		if (sn) {
			this.ros3dLaserScan = new ROS3D.LaserScan({
				ros,
				tfClient: this.tfClient3d,
				rootObject: this.viewer3d.scene,
				topic: sn,
				max_pts: maxPts,
				material: { size: Math.max(0.01, ptSize * 0.85), color: 0xff4444 }
			});
		}
		if (mk) {
			this._defer(() => this._startMarkersStage(mk), 1200);
		}
		if (wantUrdf && typeof ROS3D.Urdf === 'function' && typeof ROSLIB.UrdfModel === 'function') {
			this._defer(() => this._startUrdfStage($, host, fixedFrame), 3500);
		}
		if (pcn && ros && ros.isConnected) {
			const pc2FailAfterMs = 45000;
			const hintHost = $('view3d-host');
			setTimeout(() => {
				const got =
					selfSession.ros3dPointCloud2 &&
					selfSession.ros3dPointCloud2.__ivgPc2Sig &&
					selfSession.ros3dPointCloud2.__ivgPc2Sig.got;
				if (!got) {
					const metaT = new ROSLIB.Topic({
						ros,
						name: pcn,
						messageType: 'sensor_msgs/msg/PointCloud2',
						throttle_rate: 0,
						queue_length: 0,
						queue_size: 100
					});
					metaT.getPublishers(
						pubs => {
							const n = Array.isArray(pubs) ? pubs.length : 0;
							if (!hintHost) return;
							if (n === 0) {
								showView3dPc2Hint(
									hintHost,
									`<strong>点云无数据</strong>：<code>${pcn}</code> 上未发现发布者。请确认相机/深度节点已启动，或在侧栏选择实际在发布的 <code>sensor_msgs/msg/PointCloud2</code> 话题后再「启动 3D」。`
								);
							} else {
								showView3dPc2Hint(
									hintHost,
									`<strong>点云无数据</strong>：检测到 <code>${n}</code> 个发布者，但 <strong>${pc2FailAfterMs / 1000} 秒</strong>内浏览器仍未收到可渲染首帧。请查 rosbridge / 网关日志、消息体大小与 QoS；或降低点云分辨率以减轻单帧体积。`
								);
							}
						},
						() => {
							if (hintHost) {
								showView3dPc2Hint(
									hintHost,
									`<strong>点云无数据</strong>：无法查询发布者（rosapi 不可用或调用失败）。请确认 rosbridge 同进程已加载 <code>rosapi</code>，并查看浏览器控制台与 rosbridge 日志。`
								);
							}
						}
					);
				} else {
					const pcSceneNode =
						selfSession.ros3dPointCloud2 &&
						selfSession.ros3dPointCloud2.points &&
						selfSession.ros3dPointCloud2.points.sn;
					const pcFrame = normalizeFrameId(pcSceneNode && pcSceneNode.frameID);
					if (pcSceneNode && pcSceneNode.visible !== true && pcFrame) {
						const tfOk =
							selfSession.tfClient3d &&
							typeof selfSession.tfClient3d.hasTransformFor === 'function' &&
							selfSession.tfClient3d.hasTransformFor(pcFrame);
						if (!tfOk && hintHost) {
							showView3dPc2Hint(
								hintHost,
								`<strong>点云首帧已到但不可见</strong>：已收到 <code>${pcn}</code> 数据，但浏览器端尚未建立 <code>${fixedFrame}</code> → <code>${pcFrame}</code> 的 TF 链。请确认相机/机械臂 TF 已发布到 <code>/tf</code> 或 <code>/tf_static</code>，或先把固定坐标系改为 <code>${pcFrame}</code> 进行验证。`
							);
						}
					}
				}
			}, pc2FailAfterMs);
		}
	};

	global.IvgRos3dView3dSession = IvgRos3dView3dSession;
})(typeof window !== 'undefined' ? window : globalThis);
