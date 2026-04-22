/**
 * 3D TF 适配层：
 * - 封装浏览器本地 `/tf` 树与 `ROSLIB.ROS2TFClient` 两种策略。
 * - 向上提供统一的 `getTransform/subscribe/unsubscribe/setFixedFrame/dispose` 契约。
 * - 这一层不关心点云、URDF、Marker，只负责“给定 frame 如何拿到相对 fixed frame 的变换”。
 */
(function (global) {
	'use strict';

	function normalizeFrameId(frame) {
		return String(frame || '').trim().replace(/^\/+/, '');
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
		if (typeof qi.invert === 'function') qi.invert();
		else qi.inverse();
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
				try { cb(tf); } catch (e) { /* ignore */ }
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
		if (typeof cb === 'function') this.frameInfos[f].cbs = this.frameInfos[f].cbs.filter(fn => fn !== cb);
		else this.frameInfos[f].cbs = [];
		if (this.frameInfos[f].cbs.length === 0) delete this.frameInfos[f];
	};

	IvgRosTfTreeClient.prototype.dispose = function () {
		this._topics.forEach(t => {
			try { t.unsubscribe(this._boundHandleTf); } catch (e) { /* ignore */ }
		});
		this._topics.length = 0;
		this.frameInfos = Object.create(null);
		this._edges = Object.create(null);
	};

	function IvgRos3dTfClient(ros, fixedFrame, opts) {
		const o = opts || {};
		const ff = normalizeFrameId(fixedFrame) || 'base_link';
		this._tree = null;
		this._r2 = null;
		this.fixedFrame = ff;
		let topicOnly = o.useTopicTfOnly === true;
		if (!topicOnly && typeof document !== 'undefined') {
			const el = document.getElementById('view3d-use-topic-tf-only');
			if (el && el.checked) topicOnly = true;
		}
		const preferR2 =
			!topicOnly &&
			o.preferRos2TfClient !== false &&
			typeof ROSLIB !== 'undefined' &&
			typeof ROSLIB.ROS2TFClient === 'function';
		if (preferR2) {
			try {
				this._r2 = new ROSLIB.ROS2TFClient({
					ros,
					fixedFrame: ff,
					angularThres: typeof o.angularThres === 'number' ? o.angularThres : 2,
					transThres: typeof o.transThres === 'number' ? o.transThres : 0.01,
					rate: typeof o.rate === 'number' ? o.rate : 10,
					updateDelay: typeof o.updateDelay === 'number' ? o.updateDelay : 50,
					serverName: o.tf2RepublisherServer || '/tf2_web_republisher'
				});
				this.fixedFrame = this._r2.fixedFrame || ff;
			} catch (err) {
				try {
					console.warn('[ivg/tf] ROS2TFClient 不可用，回退浏览器 /tf 树:', err);
				} catch (e2) {
					/* ignore */
				}
			}
		}
		if (!this._r2) {
			this._tree = new IvgRosTfTreeClient({ ros, fixedFrame: ff });
			this.fixedFrame = this._tree.fixedFrame;
		}
	}

	IvgRos3dTfClient.prototype.getTransform = function (frame) {
		const f = normalizeFrameId(frame);
		if (!f) return null;
		if (this._tree) return this._tree.getTransform(f);
		if (!this._r2) return null;
		const info = this._r2.frameInfos[f];
		if (!info || !info.transform) return null;
		const t = info.transform;
		return {
			translation: { x: t.translation.x, y: t.translation.y, z: t.translation.z },
			rotation: { x: t.rotation.x, y: t.rotation.y, z: t.rotation.z, w: t.rotation.w }
		};
	};

	IvgRos3dTfClient.prototype.hasTransformFor = function (frame) {
		if (this._tree) return this._tree.hasTransformFor(frame);
		const f = normalizeFrameId(frame);
		if (!f || !this._r2) return false;
		const info = this._r2.frameInfos[f];
		return !!(info && info.transform);
	};

	IvgRos3dTfClient.prototype.subscribe = function (frame, cb) {
		if (this._tree) return this._tree.subscribe(frame, cb);
		return this._r2.subscribe(frame, cb);
	};

	IvgRos3dTfClient.prototype.unsubscribe = function (frame, cb) {
		if (this._tree) return this._tree.unsubscribe(frame, cb);
		return this._r2.unsubscribe(frame, cb);
	};

	IvgRos3dTfClient.prototype.setFixedFrame = function (frame) {
		const f = normalizeFrameId(frame);
		if (!f || f === this.fixedFrame) return false;
		this.fixedFrame = f;
		if (this._tree) {
			const ret = this._tree.setFixedFrame(f);
			if (ret) this.fixedFrame = this._tree.fixedFrame;
			return ret;
		}
		if (this._r2) {
			this._r2.fixedFrame = f;
			try { this._r2.updateGoal(); } catch (e) { /* ignore */ }
			return true;
		}
		return false;
	};

	IvgRos3dTfClient.prototype.dispose = function () {
		if (this._tree) {
			try { this._tree.dispose(); } catch (e) { /* ignore */ }
			this._tree = null;
		}
		if (this._r2) {
			try { this._r2.dispose(); } catch (e) { /* ignore */ }
			this._r2 = null;
		}
	};

	global.IVGView3DTf = {
		normalizeFrameId,
		ivgIdentityTransform,
		ivgCloneTransform,
		ivgComposeTransforms,
		ivgInvertTransform,
		ivgBuildTfPath,
		ivgFindRelativeTransform,
		IvgRosTfTreeClient,
		IvgRos3dTfClient
	};
})(typeof window !== 'undefined' ? window : globalThis);
