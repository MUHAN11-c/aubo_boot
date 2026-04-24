/**
 * 识别结果投影层控制器：
 * - 维护相机信息、抓取位姿、TF 边表。
 * - 负责 Canvas 投影绘制与结果面板标题切换。
 */

import {
	normalizeFrameId,
	ivgCloneTransform,
	ivgQuatNormalize,
	ivgRotateVectorByQuat,
	ivgComposeTransforms,
	ivgFindRelativeTransform
} from '../view3d/tf_clients.js';

function normalizeIvgTopic(t) {
	const s = String(t || '').trim();
	if (!s) return '';
	return s.startsWith('/') ? s : `/${s}`;
}

function ivgVec3(x = 0, y = 0, z = 0) {
	return { x, y, z };
}

function ivgVec3Add(a, b) {
	return {
		x: (Number(a && a.x) || 0) + (Number(b && b.x) || 0),
		y: (Number(a && a.y) || 0) + (Number(b && b.y) || 0),
		z: (Number(a && a.z) || 0) + (Number(b && b.z) || 0)
	};
}

function ivgApplyTransformPoint(tf, point) {
	const t = ivgCloneTransform(tf);
	const q = ivgQuatNormalize(t.rotation);
	const p = ivgVec3(
		Number(point && point.x) || 0,
		Number(point && point.y) || 0,
		Number(point && point.z) || 0
	);
	return ivgVec3Add(ivgRotateVectorByQuat(p, q), t.translation);
}

function createProjectionOverlayController(opts) {
	const options = opts || {};
	const $ = options.getById || (id => document.getElementById(id));
	const defaults = options.defaults || {};
	const getColorTopic = options.getColorTopic || (() => defaults['topic-color'] || '/camera/color/image_raw');

	const state = {
		cameraInfo: null,
		cameraFrame: '',
		graspMsg: null,
		tfEdges: Object.create(null),
		drawRaf: null
	};

	function buildProjectionFrameCandidates(colorTopic, cameraInfo) {
		const out = [];
		function push(v) {
			const f = normalizeFrameId(v);
			if (!f) return;
			if (out.indexOf(f) === -1) out.push(f);
		}
		push(state.cameraFrame);
		push(cameraInfo && cameraInfo.header && cameraInfo.header.frame_id);
		const t = normalizeIvgTopic(colorTopic || defaults['topic-color']);
		const parts = t.split('/').filter(Boolean);
		const cameraName = parts.length > 0 ? parts[0] : 'camera';
		push(`${cameraName}_color_optical_frame`);
		push(`${cameraName}/color_optical_frame`);
		push('camera_color_optical_frame');
		push('camera_color_frame');
		push('camera_link');
		push('camera_frame');
		return out;
	}

	function listProjectedGraspFrames() {
		const names = Object.keys(state.tfEdges).filter(name => /^grasp_pose_\d+$/.test(name));
		names.sort((a, b) => Number(a.replace('grasp_pose_', '')) - Number(b.replace('grasp_pose_', '')));
		return names;
	}

	function clearCanvas() {
		const canvas = $('result-overlay-canvas');
		if (!canvas) return;
		const ctx = canvas.getContext('2d');
		if (!ctx) return;
		ctx.clearRect(0, 0, canvas.width || 0, canvas.height || 0);
		canvas.hidden = true;
	}

	function syncCanvasSize() {
		const canvas = $('result-overlay-canvas');
		const stack = $('result-viz-stack');
		if (!canvas || !stack) return null;
		/* 始终以叠放区 #result-viz-stack 为准：canvas 带 hidden 时 client 尺寸为 0，且与底图对齐必须用同一盒模型 */
		const rect = stack.getBoundingClientRect();
		const width = Math.max(1, Math.round(rect.width));
		const height = Math.max(1, Math.round(rect.height));
		if (canvas.width !== width || canvas.height !== height) {
			canvas.width = width;
			canvas.height = height;
		}
		const ctx = canvas.getContext('2d');
		if (!ctx) return null;
		ctx.setTransform(1, 0, 0, 1, 0, 0);
		return { ctx, width, height };
	}

	function projectPointToImage(point, info, width, height) {
		if (!info || !Array.isArray(info.k) || info.k.length < 9) return null;
		if (!isFinite(point.x) || !isFinite(point.y) || !isFinite(point.z) || point.z <= 0.01) return null;
		const fx = Number(info.k[0]) || 0;
		const fy = Number(info.k[4]) || 0;
		const cx = Number(info.k[2]) || 0;
		const cy = Number(info.k[5]) || 0;
		if (!(fx > 0) || !(fy > 0)) return null;
		const u = fx * (point.x / point.z) + cx;
		const v = fy * (point.y / point.z) + cy;
		const iw = Number(info.width) || width;
		const ih = Number(info.height) || height;
		return { x: (u / iw) * width, y: (v / ih) * height, depth: point.z };
	}

	function drawProjectedSegment(ctx, a, b, style) {
		if (!a || !b) return;
		ctx.save();
		if (style && style.dash) ctx.setLineDash(style.dash);
		ctx.strokeStyle = (style && style.outlineColor) || 'rgba(15, 23, 42, 0.92)';
		ctx.lineWidth = ((style && style.lineWidth) || 2) + ((style && style.outlineWidth) || 2.5);
		ctx.lineCap = 'round';
		ctx.lineJoin = 'round';
		ctx.beginPath();
		ctx.moveTo(a.x, a.y);
		ctx.lineTo(b.x, b.y);
		ctx.stroke();
		ctx.setLineDash([]);
		ctx.strokeStyle = (style && style.color) || '#ffffff';
		ctx.lineWidth = (style && style.lineWidth) || 2;
		ctx.beginPath();
		ctx.moveTo(a.x, a.y);
		ctx.lineTo(b.x, b.y);
		ctx.stroke();
		ctx.restore();
	}

	function pathRoundedRect(ctx, x, y, w, h, r) {
		const rr = Math.max(0, Math.min(r, Math.min(w, h) / 2));
		ctx.beginPath();
		ctx.moveTo(x + rr, y);
		ctx.lineTo(x + w - rr, y);
		ctx.quadraticCurveTo(x + w, y, x + w, y + rr);
		ctx.lineTo(x + w, y + h - rr);
		ctx.quadraticCurveTo(x + w, y + h, x + w - rr, y + h);
		ctx.lineTo(x + rr, y + h);
		ctx.quadraticCurveTo(x, y + h, x, y + h - rr);
		ctx.lineTo(x, y + rr);
		ctx.quadraticCurveTo(x, y, x + rr, y);
		ctx.closePath();
	}

	function drawProjectionBadge(ctx, x, y, text, opts2) {
		const paddingX = 8;
		const boxH = 22;
		const font = (opts2 && opts2.font) || 'bold 12px sans-serif';
		ctx.save();
		ctx.font = font;
		const boxW = Math.ceil(ctx.measureText(text).width) + paddingX * 2;
		ctx.fillStyle = (opts2 && opts2.fillStyle) || 'rgba(15, 23, 42, 0.84)';
		ctx.strokeStyle = (opts2 && opts2.strokeStyle) || 'rgba(255, 255, 255, 0.25)';
		ctx.lineWidth = 1.5;
		pathRoundedRect(ctx, x, y, boxW, boxH, 9);
		ctx.fill();
		ctx.stroke();
		ctx.fillStyle = (opts2 && opts2.textColor) || '#f8fafc';
		ctx.textBaseline = 'middle';
		ctx.fillText(text, x + paddingX, y + boxH / 2);
		ctx.restore();
	}

	/** @returns {boolean} 是否至少将原点成功投影到图像平面（用于决定是否尝试 PoseArray 等回退路径） */
	function drawProjectedPose(ctx, width, height, poseCam, rank) {
		const origin3 = ivgVec3(Number(poseCam.position.x) || 0, Number(poseCam.position.y) || 0, Number(poseCam.position.z) || 0);
		const q = ivgQuatNormalize(poseCam.orientation || {});
		const info = state.cameraInfo;
		const jawWidth = rank === 0 ? 0.09 : 0.075;
		const fingerLength = rank === 0 ? 0.075 : 0.06;
		const wristLength = rank === 0 ? 0.05 : 0.04;
		const bridgeOffset = 0.012;
		const leftRoot3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(jawWidth / 2, 0, bridgeOffset), q));
		const rightRoot3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(-jawWidth / 2, 0, bridgeOffset), q));
		const leftTip3 = ivgVec3Add(leftRoot3, ivgRotateVectorByQuat(ivgVec3(0, 0, fingerLength), q));
		const rightTip3 = ivgVec3Add(rightRoot3, ivgRotateVectorByQuat(ivgVec3(0, 0, fingerLength), q));
		const wrist3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(0, 0, -wristLength), q));
		const approach3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(0, 0, fingerLength + 0.035), q));
		const origin2 = projectPointToImage(origin3, info, width, height);
		if (!origin2) return false;
		const leftRoot2 = projectPointToImage(leftRoot3, info, width, height);
		const rightRoot2 = projectPointToImage(rightRoot3, info, width, height);
		const leftTip2 = projectPointToImage(leftTip3, info, width, height);
		const rightTip2 = projectPointToImage(rightTip3, info, width, height);
		const wrist2 = projectPointToImage(wrist3, info, width, height);
		const approach2 = projectPointToImage(approach3, info, width, height);
		const mainColor = rank === 0 ? '#f59e0b' : 'rgba(56, 189, 248, 0.95)';
		const subColor = rank === 0 ? 'rgba(251, 191, 36, 0.95)' : 'rgba(224, 242, 254, 0.96)';
		const lineWidth = rank === 0 ? 3.8 : 2.6;
		if (leftRoot2 && leftTip2) drawProjectedSegment(ctx, leftRoot2, leftTip2, { color: subColor, lineWidth, outlineWidth: 3 });
		if (rightRoot2 && rightTip2) drawProjectedSegment(ctx, rightRoot2, rightTip2, { color: subColor, lineWidth, outlineWidth: 3 });
		if (leftRoot2 && rightRoot2) drawProjectedSegment(ctx, leftRoot2, rightRoot2, { color: mainColor, lineWidth: lineWidth + 0.4, outlineWidth: 3.2 });
		if (wrist2) drawProjectedSegment(ctx, wrist2, origin2, { color: mainColor, lineWidth: lineWidth - 0.2, outlineWidth: 2.8 });
		if (approach2) drawProjectedSegment(ctx, origin2, approach2, { color: '#fde68a', lineWidth: rank === 0 ? 3 : 2.2, outlineWidth: 2.4, dash: [6, 5] });
		const radius = rank === 0 ? 8 : 5.5;
		ctx.fillStyle = rank === 0 ? '#f59e0b' : '#38bdf8';
		ctx.strokeStyle = 'rgba(15, 23, 42, 0.94)';
		ctx.lineWidth = rank === 0 ? 3 : 2.2;
		ctx.beginPath();
		ctx.arc(origin2.x, origin2.y, radius, 0, Math.PI * 2);
		ctx.fill();
		ctx.stroke();
		drawProjectionBadge(ctx, origin2.x + radius + 6, origin2.y - radius - 18, rank === 0 ? '最终抓取' : `#${rank + 1}`, {
			fillStyle: rank === 0 ? 'rgba(245, 158, 11, 0.92)' : 'rgba(15, 23, 42, 0.78)',
			strokeStyle: rank === 0 ? 'rgba(255,255,255,0.4)' : 'rgba(148, 163, 184, 0.35)',
			textColor: rank === 0 ? '#111827' : '#f8fafc',
			font: rank === 0 ? 'bold 12px sans-serif' : 'bold 11px sans-serif'
		});
		return true;
	}

	function drawOverlay() {
		const active = $('mode-graspnet') && $('mode-graspnet').checked;
		const resultImg = $('result-mjpeg');
		const imgSrc = String((resultImg && (resultImg.getAttribute('src') || resultImg.src)) || '').trim();
		if (!active || !resultImg || resultImg.hidden || !imgSrc) {
			clearCanvas();
			return;
		}
		const size = syncCanvasSize();
		if (!size) {
			clearCanvas();
			return;
		}
		const { ctx, width, height } = size;
		ctx.clearRect(0, 0, width, height);
		const info = state.cameraInfo;
		const graspMsg = state.graspMsg;
		const projectionFrames = buildProjectionFrameCandidates(getColorTopic(), info);
		if (!info || projectionFrames.length === 0) {
			clearCanvas();
			return;
		}
		let drawnCount = 0;
		const graspFrames = listProjectedGraspFrames();
		if (graspFrames.length > 0) {
			let tfCamToGrasp = null;
			for (let j = 0; j < projectionFrames.length; j++) {
				tfCamToGrasp = ivgFindRelativeTransform(projectionFrames[j], graspFrames[0], state.tfEdges);
				if (tfCamToGrasp) break;
			}
			if (tfCamToGrasp) {
				if (drawProjectedPose(ctx, width, height, { position: tfCamToGrasp.translation, orientation: tfCamToGrasp.rotation }, 0)) {
					drawnCount = 1;
				}
			}
		}
		if (drawnCount === 0) {
			const poseFrame = normalizeFrameId(graspMsg && graspMsg.header && graspMsg.header.frame_id);
			if (!graspMsg || !Array.isArray(graspMsg.poses) || !graspMsg.poses.length || !poseFrame) {
				clearCanvas();
				return;
			}
			let baseToCamera = null;
			for (let j = 0; j < projectionFrames.length; j++) {
				baseToCamera = ivgFindRelativeTransform(poseFrame, projectionFrames[j], state.tfEdges);
				if (baseToCamera) break;
			}
			if (!baseToCamera) {
				clearCanvas();
				return;
			}
			const pose = graspMsg.poses[0];
			if (pose && pose.position && pose.orientation) {
				const originCam = ivgApplyTransformPoint(baseToCamera, pose.position);
				const rotCam = ivgComposeTransforms(baseToCamera, { translation: { x: 0, y: 0, z: 0 }, rotation: pose.orientation }).rotation;
				if (drawProjectedPose(ctx, width, height, { position: originCam, orientation: rotCam }, 0)) {
					drawnCount = 1;
				}
			}
		}
		if (drawnCount === 0) {
			clearCanvas();
			return;
		}
		const canvas = $('result-overlay-canvas');
		if (canvas) canvas.hidden = false;
	}

	function scheduleDraw() {
		if (state.drawRaf != null) return;
		state.drawRaf = requestAnimationFrame(() => {
			state.drawRaf = null;
			drawOverlay();
		});
	}

	return {
		resetState() {
			state.cameraInfo = null;
			state.cameraFrame = '';
			state.graspMsg = null;
			state.tfEdges = Object.create(null);
			scheduleDraw();
		},
		clearCanvas,
		scheduleDraw,
		setMode(mode) {
			const gnMode = mode === 'graspnet';
			const title = $('result-panel-title');
			const workWrap = $('result-workpiece-wrap');
			if (title) title.textContent = gnMode ? 'AI 抓取位姿投影视图' : '识别结果图像';
			if (workWrap) {
				workWrap.removeAttribute('hidden');
				workWrap.setAttribute('aria-hidden', 'false');
			}
			scheduleDraw();
		},
		setGraspMsg(msg) {
			state.graspMsg = msg || null;
			scheduleDraw();
		},
		setCameraInfo(msg) {
			state.cameraInfo = msg || null;
			state.cameraFrame = normalizeFrameId(msg && msg.header && msg.header.frame_id);
			scheduleDraw();
		},
		ingestTfMessage(msg) {
			const arr = msg && Array.isArray(msg.transforms) ? msg.transforms : [];
			for (let i = 0; i < arr.length; i++) {
				const t = arr[i];
				const child = normalizeFrameId(t && t.child_frame_id);
				const parent = normalizeFrameId(t && t.header && t.header.frame_id);
				if (!child || !parent || !t.transform) continue;
				state.tfEdges[child] = { parent, transform: ivgCloneTransform(t.transform) };
			}
			scheduleDraw();
		}
	};
}

export { createProjectionOverlayController };
