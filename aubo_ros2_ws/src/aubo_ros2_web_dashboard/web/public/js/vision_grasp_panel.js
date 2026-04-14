/* global ivgPorts, ivgTransport, ROSLIB, IvgRos3dView3dSession, THREE */
/**
 * 视觉抓取面板：仅连接与 DOM 展示；数值摘要由桥进程 ``ivg_display`` 字段提供，不在此做格式编排/曲线计算/抓取轮询。
 */
(() => {
	const rosReconnect = ivgPorts.createRosReconnectState();
	const ROS_RECONNECT_MAX = 12;
	const PAGE_STREAM_SUFFIX = `${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 8)}`;
	const coarsePointerQuery =
		typeof window !== 'undefined' && typeof window.matchMedia === 'function'
			? window.matchMedia('(pointer: coarse)')
			: null;
	const subs = {};
	const TOPIC_IDS = [
		'topic-color',
		'topic-result',
		'topic-robot',
		'topic-joints',
		'topic-vpe-status',
		'topic-grasp-poses',
		'topic-pc-graspnet',
		'topic-markers-graspnet',
		'gn-tf-fixed-frame',
		'gn-pc-max',
		'gn-view3d-point-size',
		'gn-pc-throttle-ms',
		'gn-pc-msg-ratio',
		'gn-urdf-param'
	];
	const TOPIC_DEFAULTS = {
		'topic-color': '/camera/color/image_raw',
		'topic-result': '',
		'topic-robot': '/aubo_driver/robot_status',
		'topic-joints': '/joint_states',
		'topic-vpe-status': '/system_status',
		'topic-grasp-poses': '/grasp_poses_base',
		'topic-pc-graspnet': '/camera/depth_registered/points_web',
		'topic-markers-graspnet': '/grasp_markers',
		'gn-tf-fixed-frame': 'base_link',
		'gn-pc-max': '32000',
		'gn-view3d-point-size': '0.05',
		'gn-pc-throttle-ms': '120',
		'gn-pc-msg-ratio': '2',
		'gn-urdf-param': '/robot_state_publisher:robot_description'
	};
	const TOPIC_STORAGE_KEY = 'ivg_vision_grasp_topics_v1';
	const JOINT_CHART_MAX_SAMPLES = 280;
	const JOINT_LINE_COLORS = ['#2563eb', '#16a34a', '#d97706', '#db2777', '#7c3aed', '#0d9488'];
	/** @type {{ names: string[], series: number[][] }} */
	let jointChartState = { names: [], series: [] };
	let jointChartDrawRaf = null;
	let jointChartResizeObs = null;
	/** AI 模式：第二条 rosbridge WebSocket + ros3d 会话（与 ivg_transport 并行） */
	let graspRos = null;
	let graspRos3dSession = null;
	let pageRealtimePaused = false;
	/** 识别结果区：平移/缩放（底图与投影层同栈 transform，与 RViz 鼠标交互类比） */
	const resultVizPanState = {
		tx: 0,
		ty: 0,
		scale: 1,
		activeId: null,
		lastX: 0,
		lastY: 0
	};
	const projectionState = {
		cameraInfo: null,
		cameraFrame: '',
		graspMsg: null,
		tfEdges: Object.create(null),
		drawRaf: null
	};
	/** 面板 DOM 稳定，缓存 id 查找；回调内避免重复 getElementById */
	const elCache = Object.create(null);
	function $(id) {
		if (Object.prototype.hasOwnProperty.call(elCache, id)) return elCache[id];
		const n = document.getElementById(id);
		elCache[id] = n;
		return n;
	}

	/** 图例节点不参与 id 缓存，避免首帧误缓存 null 后永不更新 */
	function jointLegendEl() {
		return document.getElementById('joint-legend');
	}

	function normalizeIvgTopic(t) {
		const s = String(t || '').trim();
		if (!s) return '';
		return s.startsWith('/') ? s : `/${s}`;
	}

	function buildPageStreamId(prefix) {
		return `${prefix}_${PAGE_STREAM_SUFFIX}`;
	}

	function projectionPreferred() {
		return !!(coarsePointerQuery && coarsePointerQuery.matches);
	}

	function normalizeFrameId(frame) {
		return String(frame || '').trim().replace(/^\/+/, '');
	}

	function buildCameraInfoTopic(colorTopic) {
		const t = normalizeIvgTopic(colorTopic || TOPIC_DEFAULTS['topic-color']);
		if (!t) return '/camera/color/camera_info';
		if (/\/camera_info$/.test(t)) return t;
		if (/\/image(_raw|_color)?$/.test(t)) return t.replace(/\/image(_raw|_color)?$/, '/camera_info');
		return `${t.replace(/\/+$/, '')}/camera_info`;
	}

	function buildProjectionFrameCandidates(colorTopic, cameraInfo) {
		const out = [];
		function push(v) {
			const f = normalizeFrameId(v);
			if (!f) return;
			if (out.indexOf(f) === -1) out.push(f);
		}
		push(projectionState.cameraFrame);
		push(cameraInfo && cameraInfo.header && cameraInfo.header.frame_id);
		const t = normalizeIvgTopic(colorTopic || TOPIC_DEFAULTS['topic-color']);
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
		const names = Object.keys(projectionState.tfEdges).filter(name => /^grasp_pose_\d+$/.test(name));
		names.sort((a, b) => {
			const na = Number(a.replace('grasp_pose_', ''));
			const nb = Number(b.replace('grasp_pose_', ''));
			return na - nb;
		});
		return names;
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
		const rootFrame = normalizeFrameId(frame);
		const out = [{ frame: rootFrame, transform: ivgIdentityTransform() }];
		const seen = new Set([rootFrame]);
		let cur = rootFrame;
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

	function ivgApplyTransformPoint(tf, point) {
		const t = ivgCloneTransform(tf);
		const q = new THREE.Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w).normalize();
		return new THREE.Vector3(
			Number(point && point.x) || 0,
			Number(point && point.y) || 0,
			Number(point && point.z) || 0
		).applyQuaternion(q).add(new THREE.Vector3(t.translation.x, t.translation.y, t.translation.z));
	}

	function resetProjectionState() {
		projectionState.cameraInfo = null;
		projectionState.cameraFrame = '';
		projectionState.graspMsg = null;
		projectionState.tfEdges = Object.create(null);
		scheduleProjectionDraw();
	}

	function clearProjectionCanvas() {
		const canvas = $('result-overlay-canvas');
		if (!canvas) return;
		const ctx = canvas.getContext('2d');
		if (!ctx) return;
		ctx.clearRect(0, 0, canvas.width || 0, canvas.height || 0);
		canvas.hidden = true;
	}

	function syncProjectionCanvasSize() {
		const canvas = $('result-overlay-canvas');
		const stack = $('result-viz-stack');
		if (!canvas || !stack) return null;
		const rect = stack.getBoundingClientRect();
		const width = Math.max(1, Math.round(rect.width));
		const height = Math.max(1, Math.round(rect.height));
		const dpr = 1;
		const needResize = canvas.width !== width * dpr || canvas.height !== height * dpr;
		if (needResize) {
			canvas.width = width * dpr;
			canvas.height = height * dpr;
		}
		const ctx = canvas.getContext('2d');
		if (!ctx) return null;
		ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
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
		return {
			x: (u / iw) * width,
			y: (v / ih) * height,
			depth: point.z
		};
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

	function drawProjectionBadge(ctx, x, y, text, opts) {
		const paddingX = 8;
		const paddingY = 5;
		const font = (opts && opts.font) || 'bold 12px sans-serif';
		ctx.save();
		ctx.font = font;
		const textWidth = Math.ceil(ctx.measureText(text).width);
		const boxW = textWidth + paddingX * 2;
		const boxH = 22;
		ctx.fillStyle = (opts && opts.fillStyle) || 'rgba(15, 23, 42, 0.84)';
		ctx.strokeStyle = (opts && opts.strokeStyle) || 'rgba(255, 255, 255, 0.25)';
		ctx.lineWidth = 1.5;
		pathRoundedRect(ctx, x, y, boxW, boxH, 9);
		ctx.fill();
		ctx.stroke();
		ctx.fillStyle = (opts && opts.textColor) || '#f8fafc';
		ctx.textBaseline = 'middle';
		ctx.fillText(text, x + paddingX, y + boxH / 2);
		ctx.restore();
	}

	function drawProjectionLegend(ctx, width) {
		const legendX = 10;
		const legendY = 10;
		const legendW = Math.min(250, Math.max(182, width * 0.42));
		const legendH = 44;
		ctx.save();
		ctx.fillStyle = 'rgba(15, 23, 42, 0.48)';
		ctx.strokeStyle = 'rgba(255, 255, 255, 0.16)';
		ctx.lineWidth = 1;
		pathRoundedRect(ctx, legendX, legendY, legendW, legendH, 12);
		ctx.fill();
		ctx.stroke();
		ctx.fillStyle = '#f8fafc';
		ctx.font = 'bold 12px sans-serif';
		ctx.fillText('最终抓取位姿投影', legendX + 12, legendY + 18);
		ctx.font = '11px sans-serif';
		ctx.fillStyle = 'rgba(248, 250, 252, 0.92)';
		ctx.fillText('橙色: 夹爪主体  浅橙: 双指  黄虚线: 接近方向', legendX + 12, legendY + 36);
		ctx.restore();
	}

	function drawProjectedPose(ctx, width, height, poseCam, rank) {
		const origin3 = new THREE.Vector3(poseCam.position.x, poseCam.position.y, poseCam.position.z);
		const q = new THREE.Quaternion(
			Number(poseCam.orientation.x) || 0,
			Number(poseCam.orientation.y) || 0,
			Number(poseCam.orientation.z) || 0,
			Number(poseCam.orientation.w) || 1
		).normalize();
		const info = projectionState.cameraInfo;
		const jawWidth = rank === 0 ? 0.09 : 0.075;
		const fingerLength = rank === 0 ? 0.075 : 0.06;
		const wristLength = rank === 0 ? 0.05 : 0.04;
		const bridgeOffset = 0.012;
		const leftRoot3 = origin3.clone().add(new THREE.Vector3(jawWidth / 2, 0, bridgeOffset).applyQuaternion(q));
		const rightRoot3 = origin3.clone().add(new THREE.Vector3(-jawWidth / 2, 0, bridgeOffset).applyQuaternion(q));
		const leftTip3 = leftRoot3.clone().add(new THREE.Vector3(0, 0, fingerLength).applyQuaternion(q));
		const rightTip3 = rightRoot3.clone().add(new THREE.Vector3(0, 0, fingerLength).applyQuaternion(q));
		const wrist3 = origin3.clone().add(new THREE.Vector3(0, 0, -wristLength).applyQuaternion(q));
		const approach3 = origin3.clone().add(new THREE.Vector3(0, 0, fingerLength + 0.035).applyQuaternion(q));
		const origin2 = projectPointToImage(origin3, info, width, height);
		const leftRoot2 = projectPointToImage(leftRoot3, info, width, height);
		const rightRoot2 = projectPointToImage(rightRoot3, info, width, height);
		const leftTip2 = projectPointToImage(leftTip3, info, width, height);
		const rightTip2 = projectPointToImage(rightTip3, info, width, height);
		const wrist2 = projectPointToImage(wrist3, info, width, height);
		const approach2 = projectPointToImage(approach3, info, width, height);
		if (!origin2) return;
		const mainColor = rank === 0 ? '#f59e0b' : 'rgba(56, 189, 248, 0.95)';
		const subColor = rank === 0 ? 'rgba(251, 191, 36, 0.95)' : 'rgba(224, 242, 254, 0.96)';
		const lineWidth = rank === 0 ? 3.8 : 2.6;
		if (leftRoot2 && leftTip2) {
			drawProjectedSegment(ctx, leftRoot2, leftTip2, {
				color: subColor,
				lineWidth,
				outlineWidth: 3
			});
		}
		if (rightRoot2 && rightTip2) {
			drawProjectedSegment(ctx, rightRoot2, rightTip2, {
				color: subColor,
				lineWidth,
				outlineWidth: 3
			});
		}
		if (leftRoot2 && rightRoot2) {
			drawProjectedSegment(ctx, leftRoot2, rightRoot2, {
				color: mainColor,
				lineWidth: lineWidth + 0.4,
				outlineWidth: 3.2
			});
		}
		if (origin2 && wrist2) {
			drawProjectedSegment(ctx, wrist2, origin2, {
				color: mainColor,
				lineWidth: lineWidth - 0.2,
				outlineWidth: 2.8
			});
		}
		if (approach2) {
			drawProjectedSegment(ctx, origin2, approach2, {
				color: '#fde68a',
				lineWidth: rank === 0 ? 3 : 2.2,
				outlineWidth: 2.4,
				dash: [6, 5]
			});
		}
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
	}

	function drawProjectionOverlay() {
		const active = $('mode-graspnet') && $('mode-graspnet').checked && projectionPreferred();
		const resultImg = $('result-mjpeg');
		if (!active || !resultImg || resultImg.hidden || !resultImg.getAttribute('src')) {
			clearProjectionCanvas();
			return;
		}
		const size = syncProjectionCanvasSize();
		if (!size) {
			clearProjectionCanvas();
			return;
		}
		const { ctx, width, height } = size;
		ctx.clearRect(0, 0, width, height);
		const info = projectionState.cameraInfo;
		const graspMsg = projectionState.graspMsg;
		const colorTopic = ($('topic-color') && $('topic-color').value.trim()) || TOPIC_DEFAULTS['topic-color'];
		const projectionFrames = buildProjectionFrameCandidates(colorTopic, info);
		if (!info || projectionFrames.length === 0) {
			clearProjectionCanvas();
			return;
		}
		const graspFrames = listProjectedGraspFrames();
		let drawnCount = 0;
		if (graspFrames.length > 0) {
			const graspFrame = graspFrames[0];
			let tfCamToGrasp = null;
			for (let j = 0; j < projectionFrames.length; j++) {
				tfCamToGrasp = ivgFindRelativeTransform(projectionFrames[j], graspFrame, projectionState.tfEdges);
				if (tfCamToGrasp) break;
			}
			if (tfCamToGrasp) {
				drawProjectedPose(ctx, width, height, {
					position: tfCamToGrasp.translation,
					orientation: tfCamToGrasp.rotation
				}, 0);
				drawnCount = 1;
			}
		}
		if (drawnCount === 0) {
			const cameraFrame = projectionFrames[0];
			const poseFrame = normalizeFrameId(graspMsg && graspMsg.header && graspMsg.header.frame_id);
			if (!graspMsg || !Array.isArray(graspMsg.poses) || !graspMsg.poses.length || !poseFrame) {
				clearProjectionCanvas();
				return;
			}
			const baseToCamera = ivgFindRelativeTransform(poseFrame, cameraFrame, projectionState.tfEdges);
			if (!baseToCamera) {
				clearProjectionCanvas();
				return;
			}
			const pose = graspMsg.poses[0];
			if (pose && pose.position && pose.orientation) {
				const originCam = ivgApplyTransformPoint(baseToCamera, pose.position);
				const rotCam = ivgComposeTransforms(baseToCamera, { translation: { x: 0, y: 0, z: 0 }, rotation: pose.orientation }).rotation;
				drawProjectedPose(ctx, width, height, {
					position: originCam,
					orientation: rotCam
				}, 0);
				drawnCount = 1;
			}
		}
		if (drawnCount === 0) {
			clearProjectionCanvas();
			return;
		}
		drawProjectionLegend(ctx, width);
		const canvas = $('result-overlay-canvas');
		if (canvas) canvas.hidden = false;
	}

	function scheduleProjectionDraw() {
		if (projectionState.drawRaf != null) return;
		projectionState.drawRaf = requestAnimationFrame(() => {
			projectionState.drawRaf = null;
			drawProjectionOverlay();
		});
	}

	function sanitizeTopicValue(id, value) {
		const raw = String(value == null ? '' : value).trim();
		const fallback = TOPIC_DEFAULTS[id] !== undefined ? String(TOPIC_DEFAULTS[id]) : '';
		if (!raw) return fallback;
		if (id === 'topic-result') return raw;
		if (id === 'topic-markers-graspnet') {
			if (raw.indexOf('__ivg_disabled') !== -1) return fallback;
			return normalizeIvgTopic(raw);
		}
		if (id === 'topic-pc-graspnet' || id === 'topic-color' || id === 'topic-robot' || id === 'topic-joints' || id === 'topic-vpe-status' || id === 'topic-grasp-poses') {
			if (raw.indexOf('__ivg_disabled') !== -1) return fallback;
			return normalizeIvgTopic(raw);
		}
		if (id === 'gn-urdf-param') {
			if (raw.indexOf('__ivg_disabled') !== -1) return fallback;
			// rosapi/get_param on this stack expects "<node_name>:<param_name>".
			if (raw === 'robot_description' || raw === '/robot_description') return fallback;
			if (raw.indexOf(':') === -1) return fallback;
			return raw;
		}
		return raw;
	}

	function sanitizeTopicConfig(obj) {
		const out = {};
		TOPIC_IDS.forEach(id => {
			out[id] = sanitizeTopicValue(id, obj && obj[id]);
		});
		return out;
	}

	/** ros3d 会话用：把 topics_lab 风格 id 映射到视觉面板上的表单 id */
	const GN_ROS3D_DOM_MAP = {
		'pc-topic': 'topic-pc-graspnet',
		'marker3-topic': 'topic-markers-graspnet',
		'scan3-topic': 'gn-scan3-stub',
		'view3d-host': 'gn-ros3d-host',
		'tf-fixed-frame': 'gn-tf-fixed-frame',
		'pc-max': 'gn-pc-max',
		'view3d-point-size': 'gn-view3d-point-size',
		'pc-throttle-ms': 'gn-pc-throttle-ms',
		'pc-msg-ratio': 'gn-pc-msg-ratio',
		'view3d-show-axes': 'gn-view3d-show-axes',
		'view3d-show-grid': 'gn-view3d-show-grid',
		'view3d-show-urdf': 'gn-show-urdf',
		'urdf-param': 'gn-urdf-param'
	};

	function gnRos3dDom$(id) {
		const rid = GN_ROS3D_DOM_MAP[id] || id;
		return document.getElementById(rid);
	}

	function stopGraspRos3d() {
		if (graspRos3dSession) {
			try {
				graspRos3dSession.stop();
			} catch (e) {
				/* ignore */
			}
			graspRos3dSession = null;
		}
	}

	function disconnectGraspRos() {
		stopGraspRos3d();
		if (graspRos) {
			try {
				graspRos.close();
			} catch (e) {
				/* ignore */
			}
			graspRos = null;
		}
	}

	function pageShouldPauseRealtime() {
		return typeof document !== 'undefined' && document.visibilityState === 'hidden';
	}

	function suspendRealtimeForBackground() {
		if (pageRealtimePaused) return;
		pageRealtimePaused = true;
		ivgPorts.clearRosReconnectTimer(rosReconnect);
		rosReconnect.attempts = 0;
		unsubscribeAll();
		ivgTransport.close();
		setConnStatus('页面后台已暂停', null);
	}

	function resumeRealtimeFromForeground() {
		if (!pageRealtimePaused) return;
		pageRealtimePaused = false;
		connect();
	}

	function ensureGraspRosConnected() {
		return new Promise((resolve, reject) => {
			if (pageRealtimePaused || pageShouldPauseRealtime()) {
				reject(new Error('页面处于后台，已暂停 AI 3D 连接'));
				return;
			}
			if (graspRos && graspRos.isConnected) {
				resolve(graspRos);
				return;
			}
			if (typeof ROSLIB === 'undefined' || typeof ROSLIB.Ros !== 'function') {
				reject(new Error('ROSLIB 未加载'));
				return;
			}
			disconnectGraspRos();
			const url = ivgPorts.rosbridgeWebSocketUrl();
			const r = new ROSLIB.Ros();
			const t = setTimeout(() => {
				try {
					r.close();
				} catch (e1) {
					/* ignore */
				}
				reject(new Error('rosbridge 连接超时'));
			}, 20000);
			r.on('connection', () => {
				clearTimeout(t);
				graspRos = r;
				resolve(graspRos);
			});
			r.on('error', () => {
				clearTimeout(t);
				reject(new Error('rosbridge 连接错误'));
			});
			void r.connect(url).catch(() => {
				clearTimeout(t);
				reject(new Error('rosbridge connect() 失败'));
			});
		});
	}

	/**
	 * JointState 等：rosbridge / 部分序列化为普通数组，也可能为 { data: [...] } 或类数组。
	 * @param {object} msg
	 * @param {string} key
	 * @returns {unknown[]}
	 */
	function rosMsgArrayField(msg, key) {
		if (!msg || typeof msg !== 'object') return [];
		const v = msg[key];
		if (v == null) return [];
		if (Array.isArray(v)) return v;
		if (typeof v === 'object' && Array.isArray(v.data)) return v.data;
		if (typeof v === 'object' && typeof v.length === 'number') {
			try {
				return Array.prototype.slice.call(v);
			} catch (e) {
				return [];
			}
		}
		return [];
	}

	/** RobotStatus：服务端 ivg_display 文本，浏览器仅 rAF 合并写 DOM */
	let poseFlushRaf = null;
	let pendingRobotMsg = null;

	function fmtPoseNum(v, digits) {
		const n = Number(v);
		return isFinite(n) ? n.toFixed(digits == null ? 4 : digits) : '--';
	}

	function escapeHtml(value) {
		return String(value == null ? '' : value)
			.replace(/&/g, '&amp;')
			.replace(/</g, '&lt;')
			.replace(/>/g, '&gt;')
			.replace(/"/g, '&quot;')
			.replace(/'/g, '&#39;');
	}

	function poseToRpyDeg(pose) {
		if (!pose || !pose.orientation || typeof THREE === 'undefined') return null;
		const q = new THREE.Quaternion(
			Number(pose.orientation.x) || 0,
			Number(pose.orientation.y) || 0,
			Number(pose.orientation.z) || 0,
			Number(pose.orientation.w) || 1
		).normalize();
		const e = new THREE.Euler().setFromQuaternion(q, 'ZYX');
		return {
			roll: THREE.Math.radToDeg(e.z),
			pitch: THREE.Math.radToDeg(e.y),
			yaw: THREE.Math.radToDeg(e.x)
		};
	}

	function formatPoseBlockHtml(pose, rpyDeg) {
		if (!pose || !pose.position || !pose.orientation) {
			return '<div class="pose-card__empty">等待数据...</div>';
		}
		const rpy = rpyDeg || poseToRpyDeg(pose);
		const px = fmtPoseNum(pose.position.x, 4);
		const py = fmtPoseNum(pose.position.y, 4);
		const pz = fmtPoseNum(pose.position.z, 4);
		const qx = fmtPoseNum(pose.orientation.x, 4);
		const qy = fmtPoseNum(pose.orientation.y, 4);
		const qz = fmtPoseNum(pose.orientation.z, 4);
		const qw = fmtPoseNum(pose.orientation.w, 4);
		const rpyRows =
			rpy != null
				? `<div class="pose-card__triple pose-card__triple--rpy">
						<div class="pose-card__pill"><span class="pose-card__pill-key">R</span><span class="pose-card__pill-val">${fmtPoseNum(rpy.roll, 1)}°</span></div>
						<div class="pose-card__pill"><span class="pose-card__pill-key">P</span><span class="pose-card__pill-val">${fmtPoseNum(rpy.pitch, 1)}°</span></div>
						<div class="pose-card__pill"><span class="pose-card__pill-key">Y</span><span class="pose-card__pill-val">${fmtPoseNum(rpy.yaw, 1)}°</span></div>
					</div>`
				: '';
		return `<div class="pose-card__body">
				<section class="pose-card__section pose-card__section--pose6">
					<h3 class="pose-card__section-title">位姿</h3>
					<div class="pose-card__triple">
						<div class="pose-card__pill pose-card__pill--pos"><span class="pose-card__pill-key">x</span><span class="pose-card__pill-val">${px}</span></div>
						<div class="pose-card__pill pose-card__pill--pos"><span class="pose-card__pill-key">y</span><span class="pose-card__pill-val">${py}</span></div>
						<div class="pose-card__pill pose-card__pill--pos"><span class="pose-card__pill-key">z</span><span class="pose-card__pill-val">${pz}</span></div>
					</div>
					<p class="pose-card__unit-hint">单位：m</p>
					${rpyRows}
				</section>
				<section class="pose-card__section">
					<h3 class="pose-card__section-title">四元数</h3>
					<div class="pose-card__quad">
						<div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">x</span><span class="pose-card__pill-val">${qx}</span></div>
						<div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">y</span><span class="pose-card__pill-val">${qy}</span></div>
						<div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">z</span><span class="pose-card__pill-val">${qz}</span></div>
						<div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">w</span><span class="pose-card__pill-val">${qw}</span></div>
					</div>
				</section>
			</div>`;
	}

	function formatRobotPoseHtml(msg) {
		if (!msg || typeof msg !== 'object') return '<div class="pose-card__empty">等待机械臂末端位姿数据...</div>';
		const xyz = msg.cartesian_position_xyz || {};
		const ori = (msg.cartesian_position && msg.cartesian_position.orientation) || {};
		const pose = {
			position: {
				x: xyz.x,
				y: xyz.y,
				z: xyz.z
			},
			orientation: {
				x: ori.x,
				y: ori.y,
				z: ori.z,
				w: ori.w
			}
		};
		const rpySrc = msg.cartesian_rpy || {};
		const hasPose =
			[pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
				.some(v => isFinite(Number(v)));
		if (!hasPose) {
			return msg.ivg_display != null && String(msg.ivg_display).trim()
				? `<div class="pose-card__empty">${escapeHtml(String(msg.ivg_display))}</div>`
				: '<div class="pose-card__empty">等待机械臂末端位姿数据...</div>';
		}
		/* cartesian_rpy（Vector3）在 aubo_driver / demo_driver 中已为 roll/pitch/yaw 角度（度），勿再 *180/π */
		const rr = Number(rpySrc.x);
		const rp = Number(rpySrc.y);
		const ry = Number(rpySrc.z);
		const rpyDeg =
			[rr, rp, ry].every(v => isFinite(v))
				? { roll: rr, pitch: rp, yaw: ry }
				: poseToRpyDeg(pose);
		return formatPoseBlockHtml(pose, rpyDeg);
	}

	function formatFinalGraspPoseHtml(msg) {
		if (!msg || !Array.isArray(msg.poses) || msg.poses.length === 0) {
			return '<div class="pose-card__empty">等待 AI 大模型最终抓取位姿数据...</div>';
		}
		const pose = msg.poses[0];
		return formatPoseBlockHtml(pose, null);
	}

	function cancelVisionVisualPending() {
		pendingRobotMsg = null;
		if (poseFlushRaf != null) {
			cancelAnimationFrame(poseFlushRaf);
			poseFlushRaf = null;
		}
	}

	function scheduleRobotPoseFlush(poseTextEl) {
		if (poseFlushRaf != null) return;
		poseFlushRaf = requestAnimationFrame(() => {
			poseFlushRaf = null;
			const r = pendingRobotMsg;
			pendingRobotMsg = null;
			if (!r || !poseTextEl) return;
			poseTextEl.innerHTML = formatRobotPoseHtml(r);
		});
	}

	function applyTopicDefaultsToDom() {
		TOPIC_IDS.forEach(id => {
			const el = $(id);
			if (el && TOPIC_DEFAULTS[id] !== undefined) el.value = sanitizeTopicValue(id, TOPIC_DEFAULTS[id]);
		});
	}

	function readTopicsFromDom() {
		const o = {};
		TOPIC_IDS.forEach(id => {
			const el = $(id);
			o[id] = sanitizeTopicValue(id, el ? el.value : '');
		});
		return o;
	}

	function loadTopicsFromStorage() {
		try {
			const raw = localStorage.getItem(TOPIC_STORAGE_KEY);
			if (!raw) return false;
			const o = sanitizeTopicConfig(JSON.parse(raw));
			if (!o || typeof o !== 'object') return false;
			const ok = TOPIC_IDS.every(id => {
				if (typeof o[id] !== 'string') return false;
				if (id === 'topic-result') return true;
				return o[id].length > 0;
			});
			if (!ok) return false;
			TOPIC_IDS.forEach(id => {
				const el = $(id);
				if (el) el.value = o[id];
			});
			return true;
		} catch (e) {
			return false;
		}
	}

	function saveTopicsToStorage() {
		try {
			localStorage.setItem(TOPIC_STORAGE_KEY, JSON.stringify(sanitizeTopicConfig(readTopicsFromDom())));
		} catch (e) { /* ignore quota / private mode */ }
	}

	function clearTopicsStorage() {
		try {
			localStorage.removeItem(TOPIC_STORAGE_KEY);
		} catch (e) { /* ignore */ }
	}

	function topicSettingsModalOpen() {
		const m = $('topic-settings-modal');
		return m && !m.hasAttribute('hidden');
	}

	function openTopicSettingsModal() {
		const m = $('topic-settings-modal');
		if (!m) return;
		m.removeAttribute('hidden');
		m.setAttribute('aria-hidden', 'false');
		document.body.style.overflow = 'hidden';
		const first = $('topic-color');
		if (first) first.focus();
	}

	function closeTopicSettingsModal() {
		const m = $('topic-settings-modal');
		if (!m) return;
		m.setAttribute('hidden', '');
		m.setAttribute('aria-hidden', 'true');
		document.body.style.overflow = '';
		const btn = $('btn-topic-settings-open');
		if (btn) btn.focus();
	}

	/** @param {boolean|null} ok true 已连 / false 异常或断开 / null 连接中 */
	function setConnStatus(text, ok) {
		const el = $('conn-status');
		if (!el) return;
		el.textContent = text;
		if (ok === true) el.className = 'status ok';
		else if (ok === false) el.className = 'status off';
		else el.className = 'status pending';
	}

	function useVisualFromMode() {
		return !!($('mode-workpiece') && $('mode-workpiece').checked);
	}

	function resetCameraDisplay() {
		const canvas = $('cam-canvas');
		const im = $('cam-mjpeg');
		if (im) {
			if (im._ivgMjpegRecoverCleanup) {
				im._ivgMjpegRecoverCleanup();
				im._ivgMjpegRecoverCleanup = null;
			}
			im.removeAttribute('src');
			im.hidden = true;
		}
		if (canvas) canvas.hidden = false;
	}

	function applyResultVizPanZoom(stack) {
		if (!stack) return;
		const s = resultVizPanState;
		stack.style.transform = `translate(${s.tx}px, ${s.ty}px) scale(${s.scale})`;
		stack.style.transformOrigin = 'center center';
	}

	function resetResultVizPanZoom() {
		const stack = $('result-viz-stack');
		resultVizPanState.tx = 0;
		resultVizPanState.ty = 0;
		resultVizPanState.scale = 1;
		resultVizPanState.activeId = null;
		if (stack) {
			stack.style.transform = '';
			stack.classList.remove('result-viz-stack--dragging');
		}
	}

	function setResultPanelMode(mode) {
		const gnMode = mode === 'graspnet';
		const useProjection = gnMode && projectionPreferred();
		const title = $('result-panel-title');
		const workWrap = $('result-workpiece-wrap');
		const gnWrap = $('gn-ros3d-wrap');
		if (title) title.textContent = gnMode ? (useProjection ? 'AI 抓取位姿投影视图' : 'AI 实时 3D 结果') : '识别结果图像';
		if (workWrap) {
			if (gnMode && !useProjection) {
				workWrap.setAttribute('hidden', '');
				workWrap.setAttribute('aria-hidden', 'true');
			} else {
				workWrap.removeAttribute('hidden');
				workWrap.setAttribute('aria-hidden', 'false');
			}
		}
		if (gnWrap) {
			if (gnMode && !useProjection) {
				gnWrap.removeAttribute('hidden');
				gnWrap.setAttribute('aria-hidden', 'false');
			} else {
				gnWrap.setAttribute('hidden', '');
				gnWrap.setAttribute('aria-hidden', 'true');
			}
		}
		scheduleProjectionDraw();
	}

	/** 左键拖拽平移、滚轮缩放、双击复位 */
	function initResultVizPanZoom() {
		const stack = $('result-viz-stack');
		if (!stack || stack._ivgPanZoomInit) return;
		stack._ivgPanZoomInit = true;
		stack.addEventListener(
			'pointerdown',
			ev => {
				if (ev.button !== 0) return;
				if (ev.target.closest && ev.target.closest('a,button,input,textarea,select,label')) return;
				resultVizPanState.activeId = ev.pointerId;
				resultVizPanState.lastX = ev.clientX;
				resultVizPanState.lastY = ev.clientY;
				stack.classList.add('result-viz-stack--dragging');
				try {
					stack.setPointerCapture(ev.pointerId);
				} catch (e) {
					/* ignore */
				}
				ev.preventDefault();
			},
			true
		);
		stack.addEventListener('pointermove', ev => {
			if (resultVizPanState.activeId !== ev.pointerId) return;
			const dx = ev.clientX - resultVizPanState.lastX;
			const dy = ev.clientY - resultVizPanState.lastY;
			resultVizPanState.lastX = ev.clientX;
			resultVizPanState.lastY = ev.clientY;
			resultVizPanState.tx += dx;
			resultVizPanState.ty += dy;
			applyResultVizPanZoom(stack);
		});
		function endPan(ev) {
			if (resultVizPanState.activeId !== ev.pointerId) return;
			resultVizPanState.activeId = null;
			stack.classList.remove('result-viz-stack--dragging');
			try {
				stack.releasePointerCapture(ev.pointerId);
			} catch (e) {
				/* ignore */
			}
		}
		stack.addEventListener('pointerup', endPan);
		stack.addEventListener('pointercancel', endPan);
		stack.addEventListener(
			'wheel',
			ev => {
				const z = Math.exp(-ev.deltaY * 0.002);
				const prev = resultVizPanState.scale;
				resultVizPanState.scale = Math.min(4, Math.max(0.25, prev * z));
				applyResultVizPanZoom(stack);
				ev.preventDefault();
			},
			{ passive: false }
		);
		stack.addEventListener('dblclick', () => {
			resetResultVizPanZoom();
		});
	}

	function resetResultPanel() {
		resetResultVizPanZoom();
		setResultPanelMode('workpiece');
		resetProjectionState();
		const rm = $('result-mjpeg');
		if (rm) {
			if (rm._ivgMjpegRecoverCleanup) {
				rm._ivgMjpegRecoverCleanup();
				rm._ivgMjpegRecoverCleanup = null;
			}
			rm.removeAttribute('src');
			rm.hidden = true;
		}
		clearProjectionCanvas();
	}

	function resetJointChart() {
		jointChartState = { names: [], series: [] };
		if (jointChartDrawRaf != null) {
			cancelAnimationFrame(jointChartDrawRaf);
			jointChartDrawRaf = null;
		}
		const leg = jointLegendEl();
		if (leg) {
			leg.replaceChildren();
			leg.setAttribute('aria-hidden', 'true');
		}
		drawJointChartImmediate();
	}

	function scheduleJointChartDraw() {
		if (jointChartDrawRaf != null) return;
		jointChartDrawRaf = requestAnimationFrame(() => {
			jointChartDrawRaf = null;
			drawJointChartImmediate();
		});
	}

	function drawJointChartImmediate() {
		const canvas = $('joint-chart');
		if (!canvas || !canvas.getContext) return;
		const ctx = canvas.getContext('2d');
		if (!ctx) return;
		const rect = canvas.getBoundingClientRect();
		const cssW = Math.max(200, Math.floor(rect.width) || 640);
		const cssH = Math.max(160, Math.floor(rect.height) || 240);
		const dpr = typeof window !== 'undefined' && window.devicePixelRatio ? window.devicePixelRatio : 1;
		canvas.style.width = `${cssW}px`;
		canvas.style.height = `${cssH}px`;
		canvas.width = Math.floor(cssW * dpr);
		canvas.height = Math.floor(cssH * dpr);
		ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
		const W = cssW;
		const H = cssH;
		const padL = 44;
		const padR = 8;
		const padT = 10;
		const padB = 22;
		const plotW = W - padL - padR;
		const plotH = H - padT - padB;
		const bg = getComputedStyle(document.documentElement).getPropertyValue('--graph-bg').trim() || '#0f172a';
		const border = getComputedStyle(document.documentElement).getPropertyValue('--border').trim() || '#334155';
		const muted = getComputedStyle(document.documentElement).getPropertyValue('--muted').trim() || '#94a3b8';
		ctx.fillStyle = bg;
		ctx.fillRect(0, 0, W, H);
		ctx.strokeStyle = border;
		ctx.lineWidth = 1;
		ctx.strokeRect(0.5, 0.5, W - 1, H - 1);

		const { names, series } = jointChartState;
		const hasData = series.some(arr => arr.length > 0);
		if (!hasData) {
			return;
		}

		let yMin = Infinity;
		let yMax = -Infinity;
		for (let j = 0; j < series.length; j++) {
			for (let t = 0; t < series[j].length; t++) {
				const y = series[j][t];
				if (typeof y === 'number' && isFinite(y)) {
					if (y < yMin) yMin = y;
					if (y > yMax) yMax = y;
				}
			}
		}
		if (!isFinite(yMin) || !isFinite(yMax)) return;
		if (yMin === yMax) {
			yMin -= 1;
			yMax += 1;
		}
		const yPad = (yMax - yMin) * 0.08 || 0.05;
		yMin -= yPad;
		yMax += yPad;

		ctx.strokeStyle = border;
		ctx.globalAlpha = 0.35;
		ctx.beginPath();
		for (let g = 0; g <= 4; g++) {
			const gy = padT + (plotH * g) / 4;
			ctx.moveTo(padL, gy);
			ctx.lineTo(padL + plotW, gy);
		}
		ctx.stroke();
		ctx.globalAlpha = 1;

		ctx.fillStyle = muted;
		ctx.font = '10px ui-monospace, monospace';
		ctx.textAlign = 'right';
		ctx.textBaseline = 'middle';
		for (let g = 0; g <= 4; g++) {
			const v = yMax - ((yMax - yMin) * g) / 4;
			const gy = padT + (plotH * g) / 4;
			ctx.fillText(v.toFixed(3), padL - 6, gy);
		}

		for (let j = 0; j < series.length; j++) {
			const arr = series[j];
			if (arr.length === 0) continue;
			const color = JOINT_LINE_COLORS[j % JOINT_LINE_COLORS.length];
			if (arr.length === 1 && isFinite(arr[0])) {
				const yNorm = (arr[0] - yMin) / (yMax - yMin);
				const y = padT + plotH * (1 - yNorm);
				const x = padL + plotW / 2;
				ctx.fillStyle = color;
				ctx.beginPath();
				ctx.arc(x, y, 3.5, 0, Math.PI * 2);
				ctx.fill();
				continue;
			}
			if (arr.length < 2) continue;
			const denom = Math.max(1, arr.length - 1);
			ctx.strokeStyle = color;
			ctx.lineWidth = 1.5;
			ctx.beginPath();
			for (let t = 0; t < arr.length; t++) {
				const x = padL + (t / denom) * plotW;
				const yNorm = (arr[t] - yMin) / (yMax - yMin);
				const y = padT + plotH * (1 - yNorm);
				if (t === 0) ctx.moveTo(x, y);
				else ctx.lineTo(x, y);
			}
			ctx.stroke();
		}

		ctx.fillStyle = muted;
		ctx.font = '10px system-ui, sans-serif';
		ctx.textAlign = 'center';
		ctx.textBaseline = 'top';
		ctx.fillText('时间 →', padL + plotW / 2, H - padB + 4);
	}

	function updateJointLegend() {
		const leg = jointLegendEl();
		if (!leg) return;
		const { names } = jointChartState;
		leg.replaceChildren();
		if (!names.length) {
			leg.setAttribute('aria-hidden', 'true');
			return;
		}
		const color = i => JOINT_LINE_COLORS[i % JOINT_LINE_COLORS.length];
		names.forEach((n, i) => {
			const row = document.createElement('span');
			row.className = 'legend-j';
			row.setAttribute('role', 'listitem');
			const sw = document.createElement('span');
			sw.className = 'legend-j-swatch';
			sw.style.background = color(i);
			sw.setAttribute('aria-hidden', 'true');
			const lab = document.createElement('span');
			lab.className = 'legend-j-name';
			lab.textContent = n;
			row.appendChild(sw);
			row.appendChild(lab);
			leg.appendChild(row);
		});
		leg.setAttribute('aria-hidden', 'false');
	}

	function pushJointStateSample(msg) {
		const rawNames = rosMsgArrayField(msg, 'name').map(x =>
			x != null && x !== '' ? String(x) : ''
		);
		const pos = rosMsgArrayField(msg, 'position').map(x => Number(x));
		const n = Math.max(rawNames.length, pos.length);
		if (n === 0) return;
		const normNames = [];
		for (let i = 0; i < n; i++) {
			const rn = rawNames[i];
			normNames.push(rn != null && rn !== '' ? String(rn) : `joint_${i}`);
		}
		const keyNew = normNames.join('\0');
		const keyOld = jointChartState.names.join('\0');
		if (jointChartState.series.length !== n || keyNew !== keyOld) {
			jointChartState.names = normNames.slice();
			jointChartState.series = Array.from({ length: n }, () => []);
		}
		for (let i = 0; i < n; i++) {
			const v = pos[i];
			if (!isFinite(v)) continue;
			const row = jointChartState.series[i];
			row.push(v);
			while (row.length > JOINT_CHART_MAX_SAMPLES) row.shift();
		}
		updateJointLegend();
		scheduleJointChartDraw();
	}

	function unsubscribeAll() {
		cancelVisionVisualPending();
		disconnectGraspRos();
		ivgTransport.clearRosHandlers();
		ivgTransport.unsubscribeAll();
		Object.keys(subs).forEach(k => {
			subs[k] = null;
		});
		resetCameraDisplay();
		resetResultPanel();
		resetJointChart();
	}

	/** 按输入框话题名建立全部订阅；connect 成功时调用 */
	function startSubscriptions() {
		unsubscribeAll();
		if (!ivgTransport.isConnected() || pageRealtimePaused || pageShouldPauseRealtime()) return;

		const colorTopic = ($('topic-color') && $('topic-color').value.trim()) || TOPIC_DEFAULTS['topic-color'];
		const robotTopic = ($('topic-robot') && $('topic-robot').value.trim()) || TOPIC_DEFAULTS['topic-robot'];
		const jointTopic = normalizeIvgTopic(
			($('topic-joints') && $('topic-joints').value.trim()) || TOPIC_DEFAULTS['topic-joints']
		);
		const statusTopic = ($('topic-vpe-status') && $('topic-vpe-status').value.trim()) || TOPIC_DEFAULTS['topic-vpe-status'];
		const graspTopic = ($('topic-grasp-poses') && $('topic-grasp-poses').value.trim()) || TOPIC_DEFAULTS['topic-grasp-poses'];
		const cameraInfoTopic = buildCameraInfoTopic(colorTopic);
		const resultTopic = $('topic-result') ? String($('topic-result').value || '').trim() : '';
		const gnMode = !!($('mode-graspnet') && $('mode-graspnet').checked);
		const projectionMode = gnMode && projectionPreferred();

		const poseTextEl = $('pose-text');
		const canvas = $('cam-canvas');
		const camMjpeg = $('cam-mjpeg');
		if (canvas) canvas.hidden = true;
		if (camMjpeg) {
			camMjpeg.hidden = false;
			camMjpeg.src = ivgTransport.cameraStreamUrl(colorTopic, buildPageStreamId('vision_color'));
		}
		subs.color = true;

		const rMjpeg = $('result-mjpeg');
		const underlayTopic = resultTopic.trim();
		if (gnMode) {
			setResultPanelMode('graspnet');
			if (rMjpeg) {
				rMjpeg.hidden = !projectionMode;
				if (projectionMode) {
					rMjpeg.src = ivgTransport.cameraStreamUrl(colorTopic, buildPageStreamId('vision_projection'));
				} else {
					rMjpeg.removeAttribute('src');
				}
			}
			subs.result = null;
		} else if (!underlayTopic) {
			setResultPanelMode('workpiece');
			if (rMjpeg) {
				rMjpeg.hidden = true;
				rMjpeg.removeAttribute('src');
			}
			subs.result = null;
		} else {
			setResultPanelMode('workpiece');
			if (rMjpeg) {
				rMjpeg.hidden = false;
				rMjpeg.src = ivgTransport.cameraStreamUrl(underlayTopic, buildPageStreamId('vision_result'));
			}
			subs.result = true;
		}

		ivgTransport.onRosJson(robotTopic, m => {
			pendingRobotMsg = m;
			scheduleRobotPoseFlush(poseTextEl);
		});
		ivgTransport.subscribe({ topic: robotTopic, msgType: 'demo_interface/msg/RobotStatus', maxHz: 20 });
		subs.robot = true;

		ivgTransport.onRosJson(jointTopic, m => {
			pushJointStateSample(m);
		});
		ivgTransport.subscribe({ topic: jointTopic, msgType: 'sensor_msgs/msg/JointState', maxHz: 30 });
		subs.joints = true;

		ivgTransport.onRosJson(statusTopic, m => {
			const el = $('vpe-status-text');
			if (el) {
				el.textContent =
					m && m.ivg_display != null ? String(m.ivg_display) : (m && m.data) ? String(m.data) : '';
			}
		});
		ivgTransport.subscribe({ topic: statusTopic, msgType: 'std_msgs/msg/String', maxHz: 10 });
		subs.vpe = true;

		ivgTransport.onRosJson(graspTopic, m => {
			const el = $('graspnet-result-text');
			if (el) el.innerHTML = formatFinalGraspPoseHtml(m);
			projectionState.graspMsg = m || null;
			scheduleProjectionDraw();
		});
		ivgTransport.subscribe({ topic: graspTopic, msgType: 'geometry_msgs/msg/PoseArray', maxHz: 15 });
		subs.grasp = true;

		ivgTransport.onRosJson(cameraInfoTopic, m => {
			projectionState.cameraInfo = m || null;
			projectionState.cameraFrame = normalizeFrameId(m && m.header && m.header.frame_id);
			scheduleProjectionDraw();
		});
		ivgTransport.subscribe({ topic: cameraInfoTopic, msgType: 'sensor_msgs/msg/CameraInfo', maxHz: 5 });
		subs.cameraInfo = true;

		function handleTfMessage(msg) {
			const arr = msg && Array.isArray(msg.transforms) ? msg.transforms : [];
			for (let i = 0; i < arr.length; i++) {
				const t = arr[i];
				const child = normalizeFrameId(t && t.child_frame_id);
				const parent = normalizeFrameId(t && t.header && t.header.frame_id);
				if (!child || !parent || !t.transform) continue;
				projectionState.tfEdges[child] = {
					parent,
					transform: ivgCloneTransform(t.transform)
				};
			}
			scheduleProjectionDraw();
		}
		ivgTransport.onRosJson('/tf', handleTfMessage);
		ivgTransport.onRosJson('/tf_static', handleTfMessage);
		ivgTransport.subscribe({ topic: '/tf', msgType: 'tf2_msgs/msg/TFMessage', maxHz: 30 });
		ivgTransport.subscribe({ topic: '/tf_static', msgType: 'tf2_msgs/msg/TFMessage', maxHz: 1 });
		subs.tf = true;

		/*
		 * AI 模式：旧识别结果图实现已移除，结果区直接切到独立的实时 3D 面板。
		 * 该面板由第二条 rosbridge 连接驱动，优先显示点云，再逐步叠加 Marker / URDF。
		 */
		if (gnMode && !projectionMode && typeof IvgRos3dView3dSession === 'function') {
			const wrap = $('gn-ros3d-wrap');
			const hostEl = $('gn-ros3d-host');
			if (wrap && hostEl) {
				void ensureGraspRosConnected()
					.then(() => {
						stopGraspRos3d();
						graspRos3dSession = new IvgRos3dView3dSession(graspRos, gnRos3dDom$, {
							viewerInnerId: 'gn-view3d-inner',
							viewerHeight: null
						});
						graspRos3dSession.start();
					})
					.catch(err => {
						console.warn('[ivg] AI 模式 ros3d:', err);
						const logEl = $('svc-log');
						if (logEl) {
							logEl.textContent = `${new Date().toLocaleTimeString()} AI 3D：rosbridge 未连 — ${err && err.message ? err.message : String(err)}`;
						}
					});
			}
		}
		if (projectionMode) scheduleProjectionDraw();
	}

	function logSvc(msg) {
		const el = $('svc-log');
		if (el) el.textContent = `${new Date().toLocaleTimeString()} ${msg}`;
	}

	/** std_srvs/SetBool；done(err, r) 可选，在成功/失败回调末尾调用 */
	function callSetBool(name, data, done) {
		if (!ivgTransport.isConnected()) {
			logSvc('未连接');
			if (typeof done === 'function') done(new Error('未连接'));
			return;
		}
		ivgTransport
			.callService({ service: name, type: 'std_srvs/srv/SetBool', request: { data: !!data } })
			.then(r => {
				logSvc(`${name} → success=${r.success} ${r.message || ''}`);
				if (typeof done === 'function') done(null, r);
			})
			.catch(e => {
				logSvc(`${name} 错误: ${e}`);
				if (typeof done === 'function') done(e);
			});
	}

	/**
	 * demo_interface/ExecuteGraspPose；done 在成功或失败时都会调用（浏览器循环据此排下一拍）
	 * @param {boolean} useVisual
	 * @param {function(Error=, *=)|undefined} done
	 */
	function callExecuteGrasp(useVisual, done) {
		if (!ivgTransport.isConnected()) {
			logSvc('未连接');
			if (typeof done === 'function') done(new Error('未连接'));
			return;
		}
		const oid = ($('object-id') && $('object-id').value.trim()) || '';
		ivgTransport
			.callService({
				service: '/execute_single_grasp',
				type: 'demo_interface/srv/ExecuteGraspPose',
				request: { object_id: oid, use_visual_estimation: !!useVisual }
			})
			.then(r => {
				logSvc(`/execute_single_grasp → success=${r.success} ${r.message || ''}`);
				if (typeof done === 'function') done(null, r);
			})
			.catch(e => {
				logSvc(`/execute_single_grasp 错误: ${e}`);
				if (typeof done === 'function') done(e);
			});
	}

	function callGripperSwap(direction) {
		if (!ivgTransport.isConnected()) {
			logSvc('未连接');
			return;
		}
		ivgTransport
			.callService({
				service: '/run_gripper_swap',
				type: 'demo_interface/srv/RunGripperSwap',
				request: { direction: direction || 'gripper2' }
			})
			.then(r => {
				logSvc(`/run_gripper_swap → success=${r.success} ${r.message || ''}`);
			})
			.catch(e => {
				logSvc(`/run_gripper_swap 错误: ${e}`);
			});
	}

	/** 连接 ivg_web_serve 控制面 WebSocket；成功后 startSubscriptions */
	function connect() {
		if (pageShouldPauseRealtime()) {
			pageRealtimePaused = true;
			setConnStatus('页面后台已暂停', null);
			return;
		}
		pageRealtimePaused = false;
		ivgPorts.clearRosReconnectTimer(rosReconnect);
		const myGen = ivgPorts.bumpRosReconnectGen(rosReconnect);
		setConnStatus('正在连接…', null);
		ivgTransport.close();
		void (async () => {
			try {
				await ivgTransport.loadRuntime();
				const rt = ivgTransport.runtime || {};
				const bridgeOk = rt.internal_bridge_ok !== false;
				await ivgTransport.connectControl();
				if (myGen !== rosReconnect.gen) return;
				rosReconnect.attempts = 0;
				ivgPorts.clearRosReconnectTimer(rosReconnect);
				ivgTransport.clearControlJsonHandlers();
				ivgTransport.onControlJson(o => {
					if (o && o.op === 'error') {
						logSvc(`IVG: ${o.message != null ? String(o.message) : 'error'}`);
						if (o.message === 'bridge_unavailable') {
							setConnStatus('已连接 · 内部桥不可用（订阅/服务可能失败）', false);
						}
					}
				});
				setConnStatus(
					bridgeOk ? '已连接（IVG 网关）' : '已连接 · 内部桥未就绪（请确认 ivg_ros_bridge）',
					true
				);
				startSubscriptions();
			} catch (e) {
				if (myGen !== rosReconnect.gen) return;
				setConnStatus('连接错误', false);
				unsubscribeAll();
				ivgPorts.scheduleRosReconnect(rosReconnect, connect, {
					maxAttempts: ROS_RECONNECT_MAX,
					onSchedule(delayMs, attempt, max) {
						setConnStatus(`已断开：${Math.round(delayMs / 1000)}s 后自动重连（${attempt}/${max}）`, false);
					},
					onExhausted() {
						setConnStatus('已断开（已达自动重连上限，请刷新页面）', false);
					}
				});
			}
		})();
	}

	/** 按抓取方式切换：控制区 + 右侧状态（VPE / GraspNet 候选二选一）；末端位姿与夹爪快换始终显示 */
	function syncModeUi() {
		const work = !!($('mode-workpiece') && $('mode-workpiece').checked);
		const g = !!($('mode-graspnet') && $('mode-graspnet').checked);
		const fieldObj = $('control-field-workpiece-id');
		const grpW = $('control-group-workpiece');
		const grpG = $('control-group-graspnet');
		const stVpe = $('status-panel-vpe');
		const stGn = $('status-panel-graspnet-poses');
		if (fieldObj) {
			fieldObj.hidden = !work;
			fieldObj.setAttribute('aria-hidden', work ? 'false' : 'true');
		}
		if (grpW) {
			grpW.hidden = !work;
			grpW.setAttribute('aria-hidden', work ? 'false' : 'true');
		}
		if (grpG) {
			grpG.hidden = !g;
			grpG.setAttribute('aria-hidden', g ? 'false' : 'true');
		}
		if (stVpe) {
			stVpe.hidden = !work;
			stVpe.setAttribute('aria-hidden', work ? 'false' : 'true');
		}
		if (stGn) {
			stGn.hidden = !g;
			stGn.setAttribute('aria-hidden', g ? 'false' : 'true');
		}
		setResultPanelMode(g ? 'graspnet' : 'workpiece');
		if (ivgTransport.isConnected()) startSubscriptions();
	}

	document.addEventListener('DOMContentLoaded', () => {
		void (async () => {
			await ivgPorts.loadRuntime();
			loadTopicsFromStorage();

			(function initJointChartResize() {
				const jc = $('joint-chart');
				if (!jc) return;
				if (typeof ResizeObserver !== 'undefined' && jc.parentElement) {
					if (jointChartResizeObs) jointChartResizeObs.disconnect();
					jointChartResizeObs = new ResizeObserver(() => {
						scheduleJointChartDraw();
					});
					jointChartResizeObs.observe(jc.parentElement);
				}
				scheduleJointChartDraw();
			})();

			initResultVizPanZoom();
			window.addEventListener('resize', scheduleProjectionDraw);
			const resultImg = $('result-mjpeg');
			if (resultImg) {
				resultImg.addEventListener('load', scheduleProjectionDraw);
			}
			if (coarsePointerQuery && typeof coarsePointerQuery.addEventListener === 'function') {
				coarsePointerQuery.addEventListener('change', () => {
					syncModeUi();
					scheduleProjectionDraw();
				});
			}

			document.addEventListener('visibilitychange', () => {
				if (pageShouldPauseRealtime()) suspendRealtimeForBackground();
				else resumeRealtimeFromForeground();
			});
			window.addEventListener('pagehide', () => {
				suspendRealtimeForBackground();
			});

			$('mode-workpiece').addEventListener('change', syncModeUi);
			$('mode-graspnet').addEventListener('change', syncModeUi);
			syncModeUi();

			// --- 抓取区：策略见文件头「抓取策略」---
			$('btn-wp-single-start').onclick = () => {
				callExecuteGrasp(useVisualFromMode());
			};
			$('btn-wp-single-stop').onclick = () => {
				callSetBool('/loop_grasp_control', false);
				logSvc('已停止');
			};
			$('btn-wp-loop-start').onclick = () => {
				if (useVisualFromMode()) {
					callSetBool('/loop_grasp_control', true);
					logSvc('循环：后端视觉');
				} else {
					callSetBool('/loop_grasp_control', false, err => {
						if (err) return;
						logSvc('非视觉循环应由 ROS/后端调度；浏览器不再轮询 execute_single_grasp');
					});
				}
			};
			$('btn-wp-loop-stop').onclick = () => {
				callSetBool('/loop_grasp_control', false);
				logSvc('停循环');
			};

			// --- AI大模型抓取（原点云管线）节点默认服务名（launch 若改 remap 需同步改此常量或扩展 UI）---
			$('btn-gn-cap-start').onclick = () => { callSetBool('/graspnet_capture_control', true); };
			$('btn-gn-cap-stop').onclick = () => { callSetBool('/graspnet_capture_control', false); };
			$('btn-gn-loop-start').onclick = () => { callSetBool('/publish_grasps_worker_loop_control', true); };
			$('btn-gn-loop-stop').onclick = () => { callSetBool('/publish_grasps_worker_loop_control', false); };
			if ($('btn-gn-exec-once')) {
				$('btn-gn-exec-once').onclick = () => { callExecuteGrasp(false); };
			}

			if ($('btn-quick-swap-0')) {
				$('btn-quick-swap-0').onclick = () => {
					callGripperSwap('gripper2_to_gripper0');
				};
			}
			$('btn-quick-swap').onclick = () => {
				callGripperSwap('gripper2');
			};

			const btnOpen = $('btn-topic-settings-open');
			if (btnOpen) btnOpen.onclick = openTopicSettingsModal;
			const btnClose = $('btn-topic-settings-close');
			if (btnClose) btnClose.onclick = closeTopicSettingsModal;
			const bd = $('topic-settings-backdrop');
			if (bd) bd.onclick = closeTopicSettingsModal;
			const btnDef = $('btn-topic-restore-defaults');
			if (btnDef) {
				btnDef.onclick = () => {
					applyTopicDefaultsToDom();
					clearTopicsStorage();
					logSvc('已恢复内置默认话题');
				};
			}
			const btnSave = $('btn-topic-save-reconnect');
			if (btnSave) {
				btnSave.onclick = () => {
					saveTopicsToStorage();
					closeTopicSettingsModal();
					connect();
					logSvc('话题已保存');
				};
			}
			document.addEventListener('keydown', ev => {
				if (ev.key !== 'Escape') return;
				if (!topicSettingsModalOpen()) return;
				closeTopicSettingsModal();
			});

			ivgPorts.wireOnlineRosReconnect(rosReconnect, connect);
			connect();
		})();
	});
})();
