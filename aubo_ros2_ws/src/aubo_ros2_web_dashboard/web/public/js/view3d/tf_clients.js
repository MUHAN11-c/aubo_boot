// tf_clients.js — ROS2TFClient wrapper (math functions imported from core/tf-math.js)
import * as ROSLIB from 'roslib';
import {
	normalizeFrameId, ivgIdentityTransform, ivgCloneTransform,
	ivgQuatNormalize, ivgQuatMultiply, ivgQuatConjugate,
	ivgRotateVectorByQuat, ivgComposeTransforms, ivgInvertTransform,
	ivgBuildTfPath, ivgFindRelativeTransform
} from '../core/tf-math.js';

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
