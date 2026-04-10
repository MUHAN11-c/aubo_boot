/* global ivgPorts, ivgTransport */
/**
 * 视觉抓取面板：仅连接与 DOM 展示；数值摘要由桥进程 ``ivg_display`` 字段提供，不在此做格式编排/曲线计算/抓取轮询。
 */
(() => {
	const rosReconnect = ivgPorts.createRosReconnectState();
	const ROS_RECONNECT_MAX = 12;
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
		'topic-camera-info',
		'topic-tf',
		'topic-tf-static'
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
		'topic-camera-info': '/camera/color/camera_info',
		'topic-tf': '/tf',
		'topic-tf-static': '/tf_static'
	};
	const TOPIC_STORAGE_KEY = 'ivg_vision_grasp_topics_v1';
	const JOINT_CHART_MAX_SAMPLES = 280;
	const JOINT_LINE_COLORS = ['#2563eb', '#16a34a', '#d97706', '#db2777', '#7c3aed', '#0d9488'];
	/** @type {{ names: string[], series: number[][] }} */
	let jointChartState = { names: [], series: [] };
	let jointChartDrawRaf = null;
	let jointChartResizeObs = null;
	let resultOverlayResizeObs = null;
	/** 识别结果区：平移/缩放（底图与投影层同栈 transform，与 RViz 鼠标交互类比） */
	const resultVizPanState = {
		tx: 0,
		ty: 0,
		scale: 1,
		activeId: null,
		lastX: 0,
		lastY: 0
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
			poseTextEl.textContent = r.ivg_display != null ? String(r.ivg_display) : '';
		});
	}

	function applyTopicDefaultsToDom() {
		TOPIC_IDS.forEach(id => {
			const el = $(id);
			if (el && TOPIC_DEFAULTS[id] !== undefined) el.value = TOPIC_DEFAULTS[id];
		});
	}

	function readTopicsFromDom() {
		const o = {};
		TOPIC_IDS.forEach(id => {
			const el = $(id);
			o[id] = el ? String(el.value || '').trim() : '';
		});
		return o;
	}

	function loadTopicsFromStorage() {
		try {
			const raw = localStorage.getItem(TOPIC_STORAGE_KEY);
			if (!raw) return false;
			const o = JSON.parse(raw);
			if (!o || typeof o !== 'object') return false;
			TOPIC_IDS.forEach(id => {
				if (o[id] === undefined && TOPIC_DEFAULTS[id] !== undefined) o[id] = TOPIC_DEFAULTS[id];
			});
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
			localStorage.setItem(TOPIC_STORAGE_KEY, JSON.stringify(readTopicsFromDom()));
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
		const rm = $('result-mjpeg');
		const rc = $('result-canvas');
		const ro = document.getElementById('result-overlay');
		if (rm) {
			if (rm._ivgGnOvLoad) {
				rm.removeEventListener('load', rm._ivgGnOvLoad);
				rm._ivgGnOvLoad = null;
			}
			if (rm._ivgMjpegRecoverCleanup) {
				rm._ivgMjpegRecoverCleanup();
				rm._ivgMjpegRecoverCleanup = null;
			}
			rm.removeAttribute('src');
			rm.hidden = true;
		}
		if (rc) {
			rc.hidden = true;
			const ctx = rc.getContext('2d');
			if (ctx && rc.width && rc.height) ctx.clearRect(0, 0, rc.width, rc.height);
		}
		if (ro) {
			const ctx = ro.getContext('2d');
			if (ctx && ro.width && ro.height) ctx.clearRect(0, 0, ro.width, ro.height);
		}
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
		if (resultOverlayResizeObs) {
			resultOverlayResizeObs.disconnect();
			resultOverlayResizeObs = null;
		}
		if (typeof IVGGraspnetOverlay !== 'undefined' && IVGGraspnetOverlay.reset) {
			IVGGraspnetOverlay.reset();
		}
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
		if (!ivgTransport.isConnected()) return;

		const colorTopic = ($('topic-color') && $('topic-color').value.trim()) || TOPIC_DEFAULTS['topic-color'];
		const robotTopic = ($('topic-robot') && $('topic-robot').value.trim()) || TOPIC_DEFAULTS['topic-robot'];
		const jointTopic = normalizeIvgTopic(
			($('topic-joints') && $('topic-joints').value.trim()) || TOPIC_DEFAULTS['topic-joints']
		);
		const statusTopic = ($('topic-vpe-status') && $('topic-vpe-status').value.trim()) || TOPIC_DEFAULTS['topic-vpe-status'];
		const graspTopic = ($('topic-grasp-poses') && $('topic-grasp-poses').value.trim()) || TOPIC_DEFAULTS['topic-grasp-poses'];
		const resultTopic = $('topic-result') ? String($('topic-result').value || '').trim() : '';
		const gnMode = !!($('mode-graspnet') && $('mode-graspnet').checked);

		const poseTextEl = $('pose-text');
		const canvas = $('cam-canvas');
		const camMjpeg = $('cam-mjpeg');
		if (canvas) canvas.hidden = true;
		if (camMjpeg) {
			camMjpeg.hidden = false;
			camMjpeg.src = ivgTransport.cameraStreamUrl(colorTopic, 'vision_color');
		}
		subs.color = true;

		const rCanvas = $('result-canvas');
		const rMjpeg = $('result-mjpeg');
		const underlayTopic = (resultTopic || (gnMode ? colorTopic : '')).trim();
		if (!underlayTopic) {
			if (rCanvas) rCanvas.hidden = true;
			if (rMjpeg) {
				rMjpeg.hidden = true;
				rMjpeg.removeAttribute('src');
			}
			subs.result = null;
		} else {
			if (rCanvas) rCanvas.hidden = true;
			if (rMjpeg) {
				rMjpeg.hidden = false;
				rMjpeg.src = ivgTransport.cameraStreamUrl(underlayTopic, 'vision_result');
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
			if (el) el.textContent = m && m.ivg_display != null ? String(m.ivg_display) : '';
		});
		ivgTransport.subscribe({ topic: graspTopic, msgType: 'geometry_msgs/msg/PoseArray', maxHz: 15 });
		subs.grasp = true;

		if (
			gnMode &&
			typeof IVGGraspnetOverlay !== 'undefined' &&
			IVGGraspnetOverlay.registerSubscriptions &&
			underlayTopic
		) {
			const img = $('result-mjpeg');
			const ocv = document.getElementById('result-overlay');
			if (img && ocv) {
				const tpc = normalizeIvgTopic(
					($('topic-pc-graspnet') && $('topic-pc-graspnet').value.trim()) ||
						TOPIC_DEFAULTS['topic-pc-graspnet']
				);
				const tmk = normalizeIvgTopic(
					($('topic-markers-graspnet') && $('topic-markers-graspnet').value.trim()) ||
						TOPIC_DEFAULTS['topic-markers-graspnet']
				);
				const tci = normalizeIvgTopic(
					($('topic-camera-info') && $('topic-camera-info').value.trim()) ||
						TOPIC_DEFAULTS['topic-camera-info']
				);
				const ttf = normalizeIvgTopic(
					($('topic-tf') && $('topic-tf').value.trim()) || TOPIC_DEFAULTS['topic-tf']
				);
				const tts = normalizeIvgTopic(
					($('topic-tf-static') && $('topic-tf-static').value.trim()) ||
						TOPIC_DEFAULTS['topic-tf-static']
				);
				IVGGraspnetOverlay.registerSubscriptions(
					ivgTransport,
					{ pc: tpc, markers: tmk, camInfo: tci, tf: ttf, tfStatic: tts },
					img,
					ocv
				);
				const st = $('result-viz-stack');
				if (st && typeof ResizeObserver !== 'undefined') {
					if (resultOverlayResizeObs) resultOverlayResizeObs.disconnect();
					resultOverlayResizeObs = new ResizeObserver(() => {
						IVGGraspnetOverlay.scheduleDraw(img, ocv);
					});
					resultOverlayResizeObs.observe(st);
				}
				if (img._ivgGnOvLoad) img.removeEventListener('load', img._ivgGnOvLoad);
				const onImg = () => IVGGraspnetOverlay.scheduleDraw(img, ocv);
				img._ivgGnOvLoad = onImg;
				img.addEventListener('load', onImg, { passive: true });
			}
		}
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

	/** 非当前策略的按钮区半透明，仍可点击 */
	function syncModeUi() {
		const work = $('mode-workpiece') && $('mode-workpiece').checked;
		const g = $('mode-graspnet') && $('mode-graspnet').checked;
		const elW = $('section-workpiece-btns');
		const elG = $('section-graspnet-btns');
		if (elW) elW.style.opacity = work ? '1' : '0.45';
		if (elG) elG.style.opacity = g ? '1' : '0.45';
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
