// tf_clients.js — TF math: quaternion ops, transform chain, ROS2TFClient wrapper
import * as ROSLIB from 'roslib';
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
function ivgQuatNormalize(q) {
	const x = Number(q && q.x) || 0;
	const y = Number(q && q.y) || 0;
	const z = Number(q && q.z) || 0;
	const w = Number(q && q.w) || 1;
	const n = Math.hypot(x, y, z, w) || 1;
	return { x: x / n, y: y / n, z: z / n, w: w / n };
}
function ivgQuatMultiply(a, b) {
	return {
		x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
		w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
	};
}
function ivgQuatConjugate(q) {
	return { x: -q.x, y: -q.y, z: -q.z, w: q.w };
}
function ivgRotateVectorByQuat(v, q) {
	const p = { x: v.x, y: v.y, z: v.z, w: 0 };
	const qp = ivgQuatMultiply(q, p);
	const out = ivgQuatMultiply(qp, ivgQuatConjugate(q));
	return { x: out.x, y: out.y, z: out.z };
}
function ivgComposeTransforms(a, b) {
	const ta = ivgCloneTransform(a);
	const tb = ivgCloneTransform(b);
	const qa = ivgQuatNormalize(ta.rotation);
	const qb = ivgQuatNormalize(tb.rotation);
	const vb = ivgRotateVectorByQuat(tb.translation, qa);
	const q = ivgQuatNormalize(ivgQuatMultiply(qa, qb));
	const v = {
		x: ta.translation.x + vb.x,
		y: ta.translation.y + vb.y,
		z: ta.translation.z + vb.z
	};
	return {
		translation: { x: v.x, y: v.y, z: v.z },
		rotation: { x: q.x, y: q.y, z: q.z, w: q.w }
	};
}
function ivgInvertTransform(tf) {
	const src = ivgCloneTransform(tf);
	const q = ivgQuatNormalize(src.rotation);
	const qi = ivgQuatConjugate(q);
	const t = ivgRotateVectorByQuat(
		{
			x: -src.translation.x,
			y: -src.translation.y,
			z: -src.translation.z
		},
		qi
	);
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
function IvgRos3dTfClient(ros, fixedFrame, opts) {
	const o = opts || {};
	const ff = normalizeFrameId(fixedFrame) || 'base_link';
	this._r2 = null;
	this.fixedFrame = ff;
	if (typeof ROSLIB === 'undefined' || typeof ROSLIB.ROS2TFClient !== 'function') {
		throw new Error('ROSLIB.ROS2TFClient 不可用，请确认 roslib 版本与 tf2_web_republisher。');
	}
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
}
IvgRos3dTfClient.prototype.getTransform = function (frame) {
	const f = normalizeFrameId(frame);
	if (!f) return null;
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
	const f = normalizeFrameId(frame);
	if (!f || !this._r2) return false;
	const info = this._r2.frameInfos[f];
	return !!(info && info.transform);
};
IvgRos3dTfClient.prototype.subscribe = function (frame, cb) {
	return this._r2.subscribe(frame, cb);
};
IvgRos3dTfClient.prototype.unsubscribe = function (frame, cb) {
	return this._r2.unsubscribe(frame, cb);
};
IvgRos3dTfClient.prototype.setFixedFrame = function (frame) {
	const f = normalizeFrameId(frame);
	if (!f || f === this.fixedFrame) return false;
	this.fixedFrame = f;
	if (this._r2) {
		this._r2.fixedFrame = f;
		try { this._r2.updateGoal(); } catch (e) {  }
		return true;
	}
	return false;
};
IvgRos3dTfClient.prototype.dispose = function () {
	if (this._r2) {
		try { this._r2.dispose(); } catch (e) {  }
		this._r2 = null;
	}
};
const IVGView3DTf = {
	normalizeFrameId,
	ivgIdentityTransform,
	ivgCloneTransform,
	ivgQuatNormalize,
	ivgQuatMultiply,
	ivgQuatConjugate,
	ivgRotateVectorByQuat,
	ivgComposeTransforms,
	ivgInvertTransform,
	ivgBuildTfPath,
	ivgFindRelativeTransform,
	IvgRos3dTfClient
};
globalThis.IVGView3DTf = IVGView3DTf;
export {
	normalizeFrameId,
	ivgIdentityTransform,
	ivgCloneTransform,
	ivgQuatNormalize,
	ivgQuatMultiply,
	ivgQuatConjugate,
	ivgRotateVectorByQuat,
	ivgComposeTransforms,
	ivgInvertTransform,
	ivgBuildTfPath,
	ivgFindRelativeTransform,
	IvgRos3dTfClient,
	IVGView3DTf
};
