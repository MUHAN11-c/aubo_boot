// projection_overlay.js — 2D grasp pose projection overlay on camera image
import {
	normalizeFrameId,
	ivgCloneTransform,
	ivgQuatNormalize,
	ivgQuatMultiply,
	ivgRotateVectorByQuat,
	ivgComposeTransforms,
	ivgFindRelativeTransform
} from '../view3d/tf_clients.js';
import { canonicalRosTopic } from '../core/utils.js';

// ── 抑制 THREE URDF 加载噪音日志 ──
(function(){
  var origLog = console.log;
  var silent = ['THREE.ColladaLoader', 'THREE.Loader:', 'DOMParser:', 'File version', 'Parse: ', 'Build: '];
  console.log = function(){
    var s = arguments[0];
    if (typeof s === 'string') {
      for (var i=0; i<silent.length; i++) { if (s.indexOf(silent[i])>=0) return; }
    }
    return origLog.apply(console, arguments);
  };
})();


// ── 与 publish_grasps_client_worker.cpp 对齐的抓取位姿变换常量 ──
// Z180: GraspNet 预测自带 π 偏差, 右乘 q_z180(0,0,1,0)
var Q_Z180 = Object.freeze({ x: 0, y: 0, z: 1, w: 0 });

/**
 * 对 GraspNet 抓取位姿做 Z180 翻转 (applyGraspZFlip180, 与 C++ 对齐):
 *   绕局部 Z 旋转 π, 位置不变。
 *   注意: C++ step4 的 tip→EEF 沿 Z -0.15m 位置偏移仅影响运动规划, 可视化投影忽略。
 * @param {{x,y,z}} position - 原始抓取位置
 * @param {{x,y,z,w}} orientation - 原始抓取四元数
 * @returns {{position: {x,y,z}, orientation: {x,y,z,w}}}
 */
function applyGraspZ180(position, orientation) {
	return {
		position: { x: position.x || 0, y: position.y || 0, z: position.z || 0 },
		orientation: ivgQuatMultiply(ivgQuatNormalize(orientation), Q_Z180)
	};
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
		const t = canonicalRosTopic(colorTopic || defaults['topic-color']);
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

		const stackRect = stack.getBoundingClientRect();
		var width, height, left = 0, top = 0;

		// 优先匹配 foxglove 图像 canvas 实际渲染区域; 未布局时回退
		const img = $('result-foxglove-canvas');
		if (img) {
			const imgRect = img.getBoundingClientRect();
			if (imgRect.width > 10 && imgRect.height > 10) {
				width = Math.round(imgRect.width);
				height = Math.round(imgRect.height);
				left = imgRect.left - stackRect.left;
				top = imgRect.top - stackRect.top;
			}
		}
		if (!width || !height) {
			// 回退: stack 或 cameraInfo 尺寸
			width = Math.round(stackRect.width);
			height = Math.round(stackRect.height);
			if (state.cameraInfo && state.cameraInfo.width) {
				// 按 cameraInfo 等比缩放更精确
				var camW = state.cameraInfo.width;
				var camH = state.cameraInfo.height;
				var scale = Math.min(width / camW, height / camH);
				width = Math.round(camW * scale);
				height = Math.round(camH * scale);
				left = Math.round((stackRect.width - width) / 2);
				top = Math.round((stackRect.height - height) / 2);
			}
		}
		width = Math.max(1, width);
		height = Math.max(1, height);

		if (canvas.width !== width || canvas.height !== height) {
			canvas.width = width;
			canvas.height = height;
		}
		canvas.style.left = Math.round(left) + 'px';
		canvas.style.top = Math.round(top) + 'px';
		canvas.style.width = width + 'px';
		canvas.style.height = height + 'px';

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
	// 高级感线段: 发光描边 + 内线
	function drawProjectedSegment(ctx, a, b, style) {
		if (!a || !b) return;
		ctx.save();
		var lw = (style && style.lineWidth) || 2;
		var outline = (style && style.outlineWidth != null) ? style.outlineWidth : lw + 2;

		// 外层发光 (shadowBlur 模拟)
		if (style && style.glow) {
			ctx.shadowColor = style.glow;
			ctx.shadowBlur = style.glowBlur || 6;
		}

		if (style && style.dash) ctx.setLineDash(style.dash);
		ctx.strokeStyle = (style && style.outlineColor) || 'rgba(15, 23, 42, 0.85)';
		ctx.lineWidth = lw + outline;
		ctx.lineCap = 'round';
		ctx.lineJoin = 'round';
		ctx.beginPath();
		ctx.moveTo(a.x, a.y);
		ctx.lineTo(b.x, b.y);
		ctx.stroke();

		// 内线 — 主色
		ctx.shadowColor = 'transparent';
		ctx.shadowBlur = 0;
		ctx.setLineDash([]);
		ctx.strokeStyle = (style && style.color) || '#ffffff';
		ctx.lineWidth = lw;
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
	// ── 高级感抓取投影: 发光+光晕+填充+十字准星+HUD 信息 ──
	function drawProjectedPose(ctx, width, height, poseCam, rank) {
		const origin3 = ivgVec3(Number(poseCam.position.x) || 0, Number(poseCam.position.y) || 0, Number(poseCam.position.z) || 0);
		const q = ivgQuatNormalize(poseCam.orientation || {});
		const info = state.cameraInfo;
		const isBest = rank === 0;

		// 末端执行器几何 (与 C++ grasp_z_offset 0.15m 对齐)
		const jawWidth = isBest ? 0.09 : 0.075;
		const fingerLength = isBest ? 0.075 : 0.06;
		const wristLength = isBest ? 0.05 : 0.04;
		const bridgeOffset = 0.012;
		const leftRoot3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(jawWidth / 2, 0, bridgeOffset), q));
		const rightRoot3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(-jawWidth / 2, 0, bridgeOffset), q));
		const leftTip3 = ivgVec3Add(leftRoot3, ivgRotateVectorByQuat(ivgVec3(0, 0, fingerLength), q));
		const rightTip3 = ivgVec3Add(rightRoot3, ivgRotateVectorByQuat(ivgVec3(0, 0, fingerLength), q));
		const wrist3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(0, 0, -wristLength), q));
		const approach3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(0, 0, fingerLength + 0.04), q));

		// 3D → 2D 投影
		const origin2 = projectPointToImage(origin3, info, width, height);
		if (!origin2) return false;
		const leftRoot2 = projectPointToImage(leftRoot3, info, width, height);
		const rightRoot2 = projectPointToImage(rightRoot3, info, width, height);
		const leftTip2 = projectPointToImage(leftTip3, info, width, height);
		const rightTip2 = projectPointToImage(rightTip3, info, width, height);
		const wrist2 = projectPointToImage(wrist3, info, width, height);
		const approach2 = projectPointToImage(approach3, info, width, height);

		// ── 配色: 最优抓取琥珀金, 其他冰蓝 ──
		const mainColor = isBest ? '#f59e0b' : '#38bdf8';
		const glowColor = isBest ? '#fbbf24' : '#7dd3fc';
		const fillColor = isBest ? 'rgba(251, 191, 36, 0.18)' : 'rgba(56, 189, 248, 0.14)';
		const lineWidth = isBest ? 3.6 : 2.4;
		const outline = isBest ? 5.5 : 4;

		// ── 1. 夹爪区域半透明填充 (手指之间) ──
		if (leftRoot2 && rightRoot2 && leftTip2 && rightTip2) {
			ctx.save();
			ctx.fillStyle = fillColor;
			ctx.strokeStyle = fillColor.replace('0.18', '0.35').replace('0.14', '0.28');
			ctx.lineWidth = 1.2;
			ctx.beginPath();
			ctx.moveTo(leftRoot2.x, leftRoot2.y);
			ctx.lineTo(leftTip2.x, leftTip2.y);
			ctx.lineTo(rightTip2.x, rightTip2.y);
			ctx.lineTo(rightRoot2.x, rightRoot2.y);
			ctx.closePath();
			ctx.fill();
			ctx.stroke();
			ctx.restore();
		}

		// ── 2. 光晕脉冲圆 (抓取中心) ──
		ctx.save();
		var haloR = isBest ? 28 : 20;
		var gradient = ctx.createRadialGradient(origin2.x, origin2.y, haloR * 0.25, origin2.x, origin2.y, haloR);
		gradient.addColorStop(0, isBest ? 'rgba(251, 191, 36, 0.55)' : 'rgba(56, 189, 248, 0.45)');
		gradient.addColorStop(0.4, isBest ? 'rgba(245, 158, 11, 0.15)' : 'rgba(56, 189, 248, 0.10)');
		gradient.addColorStop(1, 'rgba(0,0,0,0)');
		ctx.fillStyle = gradient;
		ctx.beginPath();
		ctx.arc(origin2.x, origin2.y, haloR, 0, Math.PI * 2);
		ctx.fill();

		// 外环 (细, 高亮)
		ctx.strokeStyle = isBest ? 'rgba(251, 191, 36, 0.7)' : 'rgba(56, 189, 248, 0.55)';
		ctx.lineWidth = isBest ? 2.2 : 1.6;
		ctx.beginPath();
		ctx.arc(origin2.x, origin2.y, haloR * 0.55, 0, Math.PI * 2);
		ctx.stroke();
		ctx.restore();

		// ── 3. 十字准星 ──
		var crossR = isBest ? 14 : 9;
		ctx.save();
		ctx.strokeStyle = isBest ? 'rgba(255,255,255,0.92)' : 'rgba(255,255,255,0.7)';
		ctx.lineWidth = isBest ? 2.6 : 1.8;
		ctx.shadowColor = glowColor;
		ctx.shadowBlur = 6;
		ctx.beginPath();
		ctx.moveTo(origin2.x - crossR, origin2.y);
		ctx.lineTo(origin2.x - crossR * 0.3, origin2.y);
		ctx.moveTo(origin2.x + crossR * 0.3, origin2.y);
		ctx.lineTo(origin2.x + crossR, origin2.y);
		ctx.moveTo(origin2.x, origin2.y - crossR);
		ctx.lineTo(origin2.x, origin2.y - crossR * 0.3);
		ctx.moveTo(origin2.x, origin2.y + crossR * 0.3);
		ctx.lineTo(origin2.x, origin2.y + crossR);
		ctx.stroke();
		ctx.restore();

		// ── 4. 夹爪线段 (带发光) ──
		var segGlow = isBest ? 'rgba(251, 191, 36, 0.7)' : 'rgba(56, 189, 248, 0.55)';
		if (leftRoot2 && leftTip2) drawProjectedSegment(ctx, leftRoot2, leftTip2, {
			color: isBest ? '#fcd34d' : '#bae6fd', lineWidth: lineWidth, outlineWidth: outline,
			outlineColor: 'rgba(15, 23, 42, 0.82)', glow: segGlow, glowBlur: 5
		});
		if (rightRoot2 && rightTip2) drawProjectedSegment(ctx, rightRoot2, rightTip2, {
			color: isBest ? '#fcd34d' : '#bae6fd', lineWidth: lineWidth, outlineWidth: outline,
			outlineColor: 'rgba(15, 23, 42, 0.82)', glow: segGlow, glowBlur: 5
		});
		if (leftRoot2 && rightRoot2) drawProjectedSegment(ctx, leftRoot2, rightRoot2, {
			color: mainColor, lineWidth: lineWidth + 0.6, outlineWidth: outline + 0.5,
			outlineColor: 'rgba(15, 23, 42, 0.88)', glow: segGlow, glowBlur: 7
		});
		if (wrist2) drawProjectedSegment(ctx, wrist2, origin2, {
			color: mainColor, lineWidth: lineWidth - 0.3, outlineWidth: outline - 1,
			outlineColor: 'rgba(15, 23, 42, 0.78)', glow: segGlow, glowBlur: 4
		});

		// ── 5. 接近方向箭头 (实心三角箭头) ──
		if (approach2) {
			drawProjectedSegment(ctx, origin2, approach2, {
				color: isBest ? '#fde68a' : '#e0f2fe', lineWidth: isBest ? 2.6 : 2.0,
				outlineWidth: isBest ? 3.8 : 3, outlineColor: 'rgba(15,23,42,0.8)',
				glow: isBest ? 'rgba(253, 230, 138, 0.45)' : 'rgba(224, 242, 254, 0.35)', glowBlur: 4
			});
			// 箭头尖端
			var dx = approach2.x - origin2.x;
			var dy = approach2.y - origin2.y;
			var arrowLen = Math.hypot(dx, dy);
			if (arrowLen > 2) {
				var ux = dx / arrowLen;
				var uy = dy / arrowLen;
				var aw = isBest ? 10 : 7;
				ctx.save();
				ctx.fillStyle = isBest ? '#fde68a' : '#e0f2fe';
				ctx.strokeStyle = 'rgba(15,23,42,0.82)';
				ctx.lineWidth = 1.5;
				ctx.shadowColor = isBest ? 'rgba(253, 230, 138, 0.5)' : 'rgba(224, 242, 254, 0.4)';
				ctx.shadowBlur = 5;
				ctx.beginPath();
				ctx.moveTo(approach2.x, approach2.y);
				ctx.lineTo(approach2.x - ux * aw * 1.6 - uy * aw * 0.7, approach2.y - uy * aw * 1.6 + ux * aw * 0.7);
				ctx.lineTo(approach2.x - ux * aw * 1.6 + uy * aw * 0.7, approach2.y - uy * aw * 1.6 - ux * aw * 0.7);
				ctx.closePath();
				ctx.fill();
				ctx.stroke();
				ctx.restore();
			}
		}

		// ── 6. 抓取中心实心圆 ──
		var radius = isBest ? 7 : 4.5;
		ctx.save();
		ctx.shadowColor = glowColor;
		ctx.shadowBlur = 8;
		ctx.fillStyle = mainColor;
		ctx.strokeStyle = 'rgba(15, 23, 42, 0.92)';
		ctx.lineWidth = isBest ? 2.6 : 1.8;
		ctx.beginPath();
		ctx.arc(origin2.x, origin2.y, radius, 0, Math.PI * 2);
		ctx.fill();
		ctx.stroke();
		// 内亮点
		ctx.shadowBlur = 0;
		ctx.fillStyle = 'rgba(255,255,255,0.6)';
		ctx.beginPath();
		ctx.arc(origin2.x, origin2.y, radius * 0.35, 0, Math.PI * 2);
		ctx.fill();
		ctx.restore();

		// ── 7. HUD 信息面板 (深度 + 坐标) ──
		var panelX = origin2.x + radius + 12;
		var panelY = origin2.y - 34;
		var depthMM = (origin3.z * 1000).toFixed(0);
		drawProjectionBadge(ctx, panelX, panelY, isBest ? '抓取 · ' + depthMM + 'mm' : '#' + (rank + 1) + ' · ' + depthMM + 'mm', {
			fillStyle: isBest ? 'rgba(245, 158, 11, 0.9)' : 'rgba(15, 23, 42, 0.82)',
			strokeStyle: isBest ? 'rgba(255,255,255,0.35)' : 'rgba(148, 163, 184, 0.30)',
			textColor: isBest ? '#111827' : '#f8fafc',
			font: isBest ? 'bold 11px system-ui, sans-serif' : 'bold 10px system-ui, sans-serif'
		});
		return true;
	}
	var _projLogOnce = {};
	function _projLog(key, msg) {
		if (!_projLogOnce[key]) { _projLogOnce[key] = true; console.log('[proj]', msg); }
	}
	function _projClearLog() { _projLogOnce = {}; }

	var _modeCheckCount = 0;
	var _lastDrawState = '';
	function drawOverlay() {
		_projClearLog();
		// 直接 DOM 访问 (绕过闭包 $, 排除引用问题)
		var gnEl = document.getElementById('mode-graspnet');
		var wpEl = document.getElementById('mode-workpiece');
		var gnChecked = gnEl ? gnEl.checked : null;
		var wpChecked = wpEl ? wpEl.checked : null;
		var hasGraspData = state.graspMsg && Array.isArray(state.graspMsg.poses) && state.graspMsg.poses.length > 0;

		// 每 20 帧输出一次完整的 DOM 诊断 (避免刷屏)
		_modeCheckCount++;
		if (_modeCheckCount % 60 === 1) {
			console.log('[proj] DOM diag: gnEl=' + !!gnEl + ' gnChecked=' + gnChecked + ' wpChecked=' + wpChecked + ' hasGraspData=' + hasGraspData + ' visibilityState=' + document.visibilityState);
		}

		// 抓取数据到达时自动激活 graspnet 模式
		if (hasGraspData && gnEl && !gnEl.checked) {
			gnEl.checked = true;
			console.log('[proj] AUTO-ACTIVATE: checked gnEl, now gnChecked=' + gnEl.checked);
		}
		const active = gnChecked || (hasGraspData && gnEl && gnEl.checked); // 二次检查 post-auto-activate
		if (!active) { if (!_projLogOnce._idleLogged) { _projLogOnce._idleLogged = true; _projLog('idle', '等待模式激活 (gnChecked=' + gnChecked + ' hasData=' + hasGraspData + ')'); } clearCanvas(); return; }
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
		console.log('[proj] draw: camInfo=' + !!info + ' graspMsg=' + !!graspMsg + ' frames=' + projectionFrames.length + ' tfEdges=' + Object.keys(state.tfEdges).length);
		if (!info || projectionFrames.length === 0) {
			clearCanvas();
			return;
		}
		let drawnCount = 0;
		const graspFrames = listProjectedGraspFrames();
		console.log('[proj] paths: graspFrames=' + graspFrames.length);
		if (graspFrames.length > 0) {
			let tfCamToGrasp = null;
			for (let j = 0; j < projectionFrames.length; j++) {
				tfCamToGrasp = ivgFindRelativeTransform(projectionFrames[j], graspFrames[0], state.tfEdges);
				if (tfCamToGrasp) break;
			}
			if (tfCamToGrasp) {
				// Z180 flip (与 publish_grasps_client_worker.cpp 对齐)
				var eefPose = applyGraspZ180(tfCamToGrasp.translation, tfCamToGrasp.rotation);
				if (drawProjectedPose(ctx, width, height, eefPose, 0)) {
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
			const rawPose = graspMsg.poses[0];
			if (rawPose && rawPose.position && rawPose.orientation) {
				// Z180 flip (与 publish_grasps_client_worker.cpp 对齐)
				var eefPose = applyGraspZ180(rawPose.position, rawPose.orientation);
				const originCam = ivgApplyTransformPoint(baseToCamera, eefPose.position);
				const rotCam = ivgComposeTransforms(baseToCamera, { translation: { x: 0, y: 0, z: 0 }, rotation: eefPose.orientation }).rotation;
				if (drawProjectedPose(ctx, width, height, { position: originCam, orientation: rotCam }, 0)) {
					drawnCount = 1;
				}
			}
		}
		console.log('[proj] FINAL: drawnCount=' + drawnCount);
		if (drawnCount === 0) {
			console.log('[proj] FAIL: all paths returned false');
			clearCanvas();
			return;
		}
		const canvas = $('result-overlay-canvas');
		if (canvas) { canvas.hidden = false; console.log('[proj] OK: canvas shown'); }
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
			// cameraInfo/cameraFrame 不清空 (摄像头不变)
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
			if (!state._graspMsgFirst && msg && msg.poses && msg.poses.length) {
				state._graspMsgFirst = true;
				console.log('[proj] setGraspMsg first: frame_id=', msg.header && msg.header.frame_id, 'poses=', msg.poses.length);
			}
			state.graspMsg = msg || null;
			scheduleDraw();
		},
		setCameraInfo(msg) {
			if (!state._camInfoFirst && msg && msg.k && msg.k.length >= 9) {
				state._camInfoFirst = true;
				console.log('[proj] setCameraInfo first: frame_id=', msg.header && msg.header.frame_id, 'K=[', msg.k[0], msg.k[4], msg.k[2], msg.k[5], ']', 'w=', msg.width, 'h=', msg.height);
			}
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
			if (!state._tfFirst && arr.length > 0) {
				state._tfFirst = true;
				console.log('[proj] ingestTf first: ' + arr.length + ' transforms');
			}
			scheduleDraw();
		}
	};
}
export { createProjectionOverlayController };
