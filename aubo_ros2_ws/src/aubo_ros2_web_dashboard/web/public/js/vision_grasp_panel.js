/* global ROSLIB, ivgPorts */
/**
 * 视觉抓取面板：roslib ↔ rosbridge；彩色图优先 web_video_server MJPEG（ivg_web_video.js），否则 rosbridge Image → Canvas（ivg_image_canvas）。
 * 端口与 ``ivg_runtime.js`` + ``/api/ivg/runtime-config`` 一致；话题表、服务表、抓取策略与 DOM id 约定见包内 README「视觉抓取面板」与本文件内注释。
 */
(() => {
	let ros = null;
	const rosReconnect = ivgPorts.createRosReconnectState();
	const ROS_RECONNECT_MAX = 12;
	const subs = {};
	const TOPIC_IDS = ['topic-color', 'topic-result', 'topic-robot', 'topic-joints', 'topic-vpe-status', 'topic-grasp-poses'];
	const TOPIC_DEFAULTS = {
		'topic-color': '/camera/color/image_raw',
		'topic-result': '',
		'topic-robot': '/aubo_driver/robot_status',
		'topic-joints': '/joint_states',
		'topic-vpe-status': '/system_status',
		'topic-grasp-poses': '/grasp_poses_base'
	};
	const TOPIC_STORAGE_KEY = 'ivg_vision_grasp_topics_v1';
	const CONTROL_COL_PCT_KEY = 'ivg_vision_control_col_pct';
	/** 面板 DOM 稳定，缓存 id 查找；回调内避免重复 getElementById */
	const elCache = Object.create(null);
	function $(id) {
		if (Object.prototype.hasOwnProperty.call(elCache, id)) return elCache[id];
		const n = document.getElementById(id);
		elCache[id] = n;
		return n;
	}

	/** rosbridge Image → Canvas（非 MJPEG 路径）：每帧最多提交一次，避免消息风暴卡主线程 */
	let colorDrawRaf = null;
	let pendingColorMsg = null;
	let resultDrawRaf = null;
	let pendingResultMsg = null;
	/** RobotStatus：合并文本与关节采样到单帧 */
	let poseFlushRaf = null;
	let pendingRobotMsg = null;
	/** /joint_states 与 RobotStatus 并行时限制采样率，减轻 jointHistory 与图表压力 */
	let lastJointStatesPush = 0;
	const JOINT_STATE_MIN_MS = 33;
	let jointChartTimer = null;
	const JOINT_CHART_MS = 200;

	function cancelVisionVisualPending() {
		pendingRobotMsg = null;
		pendingColorMsg = null;
		pendingResultMsg = null;
		if (poseFlushRaf != null) {
			cancelAnimationFrame(poseFlushRaf);
			poseFlushRaf = null;
		}
		if (colorDrawRaf != null) {
			cancelAnimationFrame(colorDrawRaf);
			colorDrawRaf = null;
		}
		if (resultDrawRaf != null) {
			cancelAnimationFrame(resultDrawRaf);
			resultDrawRaf = null;
		}
	}

	function scheduleRobotPoseFlush(poseTextEl) {
		if (poseFlushRaf != null) return;
		poseFlushRaf = requestAnimationFrame(() => {
			poseFlushRaf = null;
			const r = pendingRobotMsg;
			pendingRobotMsg = null;
			if (!r) return;
			if (poseTextEl) poseTextEl.textContent = formatRobotStatus(r);
			if (Array.isArray(r.joint_position_rad) && r.joint_position_rad.length >= 6) {
				pushJointSample(r.joint_position_rad.slice(0, 6));
			}
		});
	}

	function scheduleColorDraw(canvas) {
		if (!canvas) return;
		if (colorDrawRaf != null) return;
		colorDrawRaf = requestAnimationFrame(() => {
			colorDrawRaf = null;
			const msg = pendingColorMsg;
			pendingColorMsg = null;
			if (msg) drawSensorImageToCanvas(msg, canvas);
		});
	}

	function scheduleResultDraw(canvas) {
		if (!canvas) return;
		if (resultDrawRaf != null) return;
		resultDrawRaf = requestAnimationFrame(() => {
			resultDrawRaf = null;
			const msg = pendingResultMsg;
			pendingResultMsg = null;
			if (msg) drawSensorImageToCanvas(msg, canvas);
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

	function setConnStatus(text, ok) {
		const el = $('conn-status');
		if (!el) return;
		el.textContent = text;
		el.className = `status${ok ? ' ok' : ' off'}`;
	}

	function drawSensorImageToCanvas(msg, canvas) {
		const r = IVGImageCanvas.paintSensorImage(msg, canvas);
		if (!r.ok) return;
		IVGImageCanvas.applyVisionPanelImageStyle(canvas);
	}

	// --- 关节曲线缓冲（与 drawJointChart / chartLoop 配合）---
	const jointHistory = {
		max: 280,
		t: [],
		joints: [[], [], [], [], [], []]
	};

	/**
	 * GraspNet 策略下「循环」状态：后端 /loop_grasp_control 线程始终走视觉，无法实现「非视觉循环」，
	 * 故在页面内用 setTimeout 链重复调用 /execute_single_grasp(false)。pauseMs 为两次调用间隔。
	 */
	const clientGraspLoop = { active: false, timeoutId: null, pauseMs: 1500 };

	function useVisualFromMode() {
		return !!($('mode-workpiece') && $('mode-workpiece').checked);
	}

	function stopClientGraspLoop() {
		clientGraspLoop.active = false;
		if (clientGraspLoop.timeoutId) {
			clearTimeout(clientGraspLoop.timeoutId);
			clientGraspLoop.timeoutId = null;
		}
	}

	function startClientGraspLoop() {
		stopClientGraspLoop();
		clientGraspLoop.active = true;
		function tick() {
			if (!clientGraspLoop.active || !ros || !ros.isConnected) return;
			callExecuteGrasp(false, () => {
				if (!clientGraspLoop.active) return;
				clientGraspLoop.timeoutId = setTimeout(() => {
					clientGraspLoop.timeoutId = null;
					tick();
				}, clientGraspLoop.pauseMs);
			});
		}
		tick();
	}

	function pushJointSample(arr6) {
		const now = performance.now() / 1000;
		jointHistory.t.push(now);
		for (let i = 0; i < 6; i++) {
			jointHistory.joints[i].push(typeof arr6[i] === 'number' && isFinite(arr6[i]) ? arr6[i] : 0);
		}
		while (jointHistory.t.length > jointHistory.max) {
			jointHistory.t.shift();
			for (let j = 0; j < 6; j++) jointHistory.joints[j].shift();
		}
	}

	const JOINT_COLORS = ['#2563eb', '#16a34a', '#d97706', '#db2777', '#7c3aed', '#0d9488'];

	function drawJointChart() {
		const canvas = $('joint-chart');
		if (!canvas) return;
		const ctx = canvas.getContext('2d');
		const wPx = canvas.offsetWidth || (canvas.parentElement && canvas.parentElement.clientWidth) || 600;
		const W = canvas.width = Math.max(120, Math.floor(wPx));
		const hPx = canvas.offsetHeight;
		const H = canvas.height = hPx > 48 ? Math.floor(hPx) : 220;
		ctx.fillStyle = '#f8fafc';
		ctx.fillRect(0, 0, W, H);
		const data = jointHistory;
		if (data.t.length < 2) {
			ctx.fillStyle = '#94a3b8';
			ctx.font = '12px sans-serif';
			ctx.fillText('等待关节角数据（RobotStatus 或 /joint_states）…', 12, H / 2);
			return;
		}
		const t0 = data.t[0];
		const t1 = data.t[data.t.length - 1];
		const span = Math.max(0.001, t1 - t0);
		const all = [];
		for (var c = 0; c < 6; c++) {
			for (let k = 0; k < data.joints[c].length; k++) all.push(data.joints[c][k]);
		}
		let vmin = Math.min.apply(null, all);
		let vmax = Math.max.apply(null, all);
		if (vmin === vmax) {
			vmin -= 0.1;
			vmax += 0.1;
		}
		const pad = { l: 44, r: 8, t: 8, b: 22 };
		const pw = W - pad.l - pad.r;
		const ph = H - pad.t - pad.b;
		ctx.strokeStyle = '#e2e8f0';
		ctx.lineWidth = 1;
		for (let g = 0; g <= 4; g++) {
			var y = pad.t + (ph * g) / 4;
			ctx.beginPath();
			ctx.moveTo(pad.l, y);
			ctx.lineTo(W - pad.r, y);
			ctx.stroke();
		}
		for (var c = 0; c < 6; c++) {
			ctx.strokeStyle = JOINT_COLORS[c];
			ctx.lineWidth = 1.5;
			ctx.beginPath();
			const series = data.joints[c];
			for (let i = 0; i < data.t.length; i++) {
				const x = pad.l + ((data.t[i] - t0) / span) * pw;
				const v = series[i];
				var y = pad.t + ph * (1 - (v - vmin) / (vmax - vmin));
				if (i === 0) ctx.moveTo(x, y);
				else ctx.lineTo(x, y);
			}
			ctx.stroke();
		}
		ctx.fillStyle = '#64748b';
		ctx.font = '10px sans-serif';
		ctx.fillText(`${vmax.toFixed(3)} rad`, 4, pad.t + 10);
		ctx.fillText(`${vmin.toFixed(3)} rad`, 4, H - pad.b);
	}

	/** RobotStatus → 末端 xyz、Pose 四元数、cartesian_rpy、关节角等可读多行文本 */
	function formatRobotStatus(m) {
		if (!m) return '（尚无机械臂状态数据）';
		const lines = [];
		lines.push(`is_online: ${m.is_online}  enable: ${m.enable}  in_motion: ${m.in_motion}`);
		if (m.planning_status) lines.push(`planning_status: ${m.planning_status}`);
		if (m.cartesian_position_xyz) {
			const p = m.cartesian_position_xyz;
			lines.push(`位置 xyz (m): ${[p.x, p.y, p.z].map(x => Number(x).toFixed(4)).join(', ')}`);
		}
		if (m.cartesian_position && m.cartesian_position.position && m.cartesian_position.orientation) {
			const pos = m.cartesian_position.position;
			const o = m.cartesian_position.orientation;
			lines.push(`Pose position: ${[pos.x, pos.y, pos.z].map(x => Number(x).toFixed(4)).join(', ')}`);
			lines.push(`Pose quaternion: ${[o.x, o.y, o.z, o.w].map(x => Number(x).toFixed(4)).join(', ')}`);
		}
		if (m.cartesian_rpy) {
			const r = m.cartesian_rpy;
			lines.push(`cartesian_rpy (Vector3, 通常为 roll/pitch/yaw rad): ${[r.x, r.y, r.z].map(x => Number(x).toFixed(4)).join(', ')}`);
		}
		if (Array.isArray(m.joint_position_rad)) {
			lines.push(`joint_position_rad: ${m.joint_position_rad.map(x => Number(x).toFixed(4)).join(', ')}`);
		}
		if (Array.isArray(m.joint_position_deg)) {
			lines.push(`joint_position_deg: ${m.joint_position_deg.map(x => Number(x).toFixed(2)).join(', ')}`);
		}
		return lines.join('\n');
	}

	/** PoseArray 摘要：数量、frame_id、首点 position/orientation */
	function formatGraspPoses(m) {
		if (!m || !Array.isArray(m.poses)) return '（尚无点云抓取位姿 PoseArray 数据）';
		const n = m.poses.length;
		const fid = m.header && m.header.frame_id ? m.header.frame_id : '（空）';
		const lines = [`PoseArray 数量: ${n}`, `frame_id: ${fid}`];
		if (n > 0) {
			const p = m.poses[0].position;
			const o = m.poses[0].orientation;
			lines.push(`首点 position: ${[p.x, p.y, p.z].map(x => Number(x).toFixed(4)).join(', ')}`);
			lines.push(`首点 orientation: ${[o.x, o.y, o.z, o.w].map(x => Number(x).toFixed(4)).join(', ')}`);
		}
		return lines.join('\n');
	}

	function webVideoPortVision() {
		return ivgPorts.webVideo($('web-video-port'));
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

	function resetResultPanel() {
		const rm = $('result-mjpeg');
		const rc = $('result-canvas');
		const ph = $('result-placeholder');
		if (rm) {
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
		if (ph) ph.hidden = false;
	}

	function unsubscribeAll() {
		cancelVisionVisualPending();
		Object.keys(subs).forEach(k => {
			try {
				if (subs[k] && subs[k].dispose) subs[k].dispose();
			} catch (e) { /* ignore */ }
			subs[k] = null;
		});
		resetCameraDisplay();
		resetResultPanel();
	}

	/** 按输入框话题名建立全部订阅；connect 成功时调用 */
	function startSubscriptions() {
		unsubscribeAll();
		if (!ros || !ros.isConnected) return;

		const colorTopic = ($('topic-color') && $('topic-color').value.trim()) || TOPIC_DEFAULTS['topic-color'];
		const robotTopic = ($('topic-robot') && $('topic-robot').value.trim()) || TOPIC_DEFAULTS['topic-robot'];
		const jointTopic = ($('topic-joints') && $('topic-joints').value.trim()) || TOPIC_DEFAULTS['topic-joints'];
		const statusTopic = ($('topic-vpe-status') && $('topic-vpe-status').value.trim()) || TOPIC_DEFAULTS['topic-vpe-status'];
		const graspTopic = ($('topic-grasp-poses') && $('topic-grasp-poses').value.trim()) || TOPIC_DEFAULTS['topic-grasp-poses'];
		const resultTopic = $('topic-result') ? String($('topic-result').value || '').trim() : '';

		const poseTextEl = $('pose-text');
		const canvas = $('cam-canvas');
		const camMjpeg = $('cam-mjpeg');
		if (typeof ivgWebVideo !== 'undefined') {
			if (canvas) canvas.hidden = true;
			if (camMjpeg) {
				camMjpeg.hidden = false;
				camMjpeg.src = ivgWebVideo.streamUrl(colorTopic, {
					port: webVideoPortVision(),
				});
				if (typeof ivgWebVideo.mjpegStreamAttachAutoReload === 'function') {
					ivgWebVideo.mjpegStreamAttachAutoReload(camMjpeg, () =>
						ivgWebVideo.streamUrl(colorTopic, {
							port: webVideoPortVision(),
						})
					);
				}
			}
			subs.color = null;
		} else {
			if (canvas) canvas.hidden = false;
			if (camMjpeg) {
				camMjpeg.hidden = true;
				camMjpeg.removeAttribute('src');
			}
			subs.color = new ROSLIB.Topic({
				ros,
				name: colorTopic,
				messageType: 'sensor_msgs/msg/Image',
				queue_length: 1
			});
			subs.color.subscribe(msg => {
				pendingColorMsg = msg;
				scheduleColorDraw(canvas);
			});
		}

		const rCanvas = $('result-canvas');
		const rMjpeg = $('result-mjpeg');
		const rPh = $('result-placeholder');
		if (!resultTopic) {
			if (rPh) rPh.hidden = false;
			if (rCanvas) rCanvas.hidden = true;
			if (rMjpeg) {
				rMjpeg.hidden = true;
				rMjpeg.removeAttribute('src');
			}
			subs.result = null;
		} else if (typeof ivgWebVideo !== 'undefined') {
			if (rPh) rPh.hidden = true;
			if (rCanvas) rCanvas.hidden = true;
			if (rMjpeg) {
				rMjpeg.hidden = false;
				rMjpeg.src = ivgWebVideo.streamUrl(resultTopic, {
					port: webVideoPortVision(),
				});
				if (typeof ivgWebVideo.mjpegStreamAttachAutoReload === 'function') {
					ivgWebVideo.mjpegStreamAttachAutoReload(rMjpeg, () =>
						ivgWebVideo.streamUrl(resultTopic, {
							port: webVideoPortVision(),
						})
					);
				}
			}
			subs.result = null;
		} else {
			if (rPh) rPh.hidden = true;
			if (rCanvas) rCanvas.hidden = false;
			if (rMjpeg) {
				rMjpeg.hidden = true;
				rMjpeg.removeAttribute('src');
			}
			subs.result = new ROSLIB.Topic({
				ros,
				name: resultTopic,
				messageType: 'sensor_msgs/msg/Image',
				queue_length: 1
			});
			subs.result.subscribe(msg => {
				pendingResultMsg = msg;
				scheduleResultDraw(rCanvas);
			});
		}

		subs.robot = new ROSLIB.Topic({
			ros,
			name: robotTopic,
			messageType: 'demo_interface/msg/RobotStatus',
			queue_length: 1
		});
		subs.robot.subscribe(m => {
			pendingRobotMsg = m;
			scheduleRobotPoseFlush(poseTextEl);
		});

		subs.joints = new ROSLIB.Topic({
			ros,
			name: jointTopic,
			messageType: 'sensor_msgs/msg/JointState',
			queue_length: 1
		});
		subs.joints.subscribe(m => {
			if (!m.position || m.position.length < 6) return;
			const now = performance.now();
			if (now - lastJointStatesPush < JOINT_STATE_MIN_MS) return;
			lastJointStatesPush = now;
			/* 与 RobotStatus 并行推同一 jointHistory：未按 name 对齐关节序，仅作趋势 */
			pushJointSample(m.position.slice(0, 6));
		});

		subs.vpe = new ROSLIB.Topic({
			ros,
			name: statusTopic,
			messageType: 'std_msgs/msg/String',
			queue_length: 1
		});
		subs.vpe.subscribe(m => {
			const el = $('vpe-status-text');
			if (el) el.textContent = m.data || '';
		});

		subs.grasp = new ROSLIB.Topic({
			ros,
			name: graspTopic,
			messageType: 'geometry_msgs/msg/PoseArray',
			queue_length: 1
		});
		subs.grasp.subscribe(m => {
			const el = $('graspnet-result-text');
			if (el) el.textContent = formatGraspPoses(m);
		});
	}

	function logSvc(msg) {
		const el = $('svc-log');
		if (el) el.textContent = `${new Date().toLocaleTimeString()} ${msg}`;
	}

	/** std_srvs/SetBool；done(err, r) 可选，在成功/失败回调末尾调用 */
	function callSetBool(name, data, done) {
		if (!ros || !ros.isConnected) {
			logSvc('未连接');
			if (typeof done === 'function') done(new Error('未连接'));
			return;
		}
		const srv = new ROSLIB.Service({
			ros,
			name,
			serviceType: 'std_srvs/srv/SetBool'
		});
		srv.callService(
			new ROSLIB.ServiceRequest({ data: !!data }),
			r => {
				logSvc(`${name} → success=${r.success} ${r.message || ''}`);
				if (typeof done === 'function') done(null, r);
			},
			e => {
				logSvc(`${name} 错误: ${e}`);
				if (typeof done === 'function') done(e);
			}
		);
	}

	/**
	 * demo_interface/ExecuteGraspPose；done 在成功或失败时都会调用（浏览器循环据此排下一拍）
	 * @param {boolean} useVisual
	 * @param {function(Error=, *=)|undefined} done
	 */
	function callExecuteGrasp(useVisual, done) {
		if (!ros || !ros.isConnected) {
			logSvc('未连接');
			if (typeof done === 'function') done(new Error('未连接'));
			return;
		}
		const oid = ($('object-id') && $('object-id').value.trim()) || '';
		const srv = new ROSLIB.Service({
			ros,
			name: '/execute_single_grasp',
			serviceType: 'demo_interface/srv/ExecuteGraspPose'
		});
		srv.callService(
			new ROSLIB.ServiceRequest({ object_id: oid, use_visual_estimation: !!useVisual }),
			r => {
				logSvc(`/execute_single_grasp → success=${r.success} ${r.message || ''}`);
				if (typeof done === 'function') done(null, r);
			},
			e => {
				logSvc(`/execute_single_grasp 错误: ${e}`);
				if (typeof done === 'function') done(e);
			}
		);
	}

	function callGripperSwap(direction) {
		if (!ros || !ros.isConnected) {
			logSvc('未连接');
			return;
		}
		const srv = new ROSLIB.Service({
			ros,
			name: '/run_gripper_swap',
			serviceType: 'demo_interface/srv/RunGripperSwap'
		});
		srv.callService(
			new ROSLIB.ServiceRequest({ direction: direction || 'gripper2' }),
			r => {
				logSvc(`/run_gripper_swap → success=${r.success} ${r.message || ''}`);
			},
			e => {
				logSvc(`/run_gripper_swap 错误: ${e}`);
			}
		);
	}

	/** 建立 Ros 实例并 connect；on connection 调 startSubscriptions；on close 停浏览器循环并 dispose 订阅 */
	function connect() {
		ivgPorts.clearRosReconnectTimer(rosReconnect);
		const myGen = ivgPorts.bumpRosReconnectGen(rosReconnect);
		if (ros) {
			try {
				ros.close();
			} catch (e) { /* ignore */ }
		}
		ros = new ROSLIB.Ros({ groovyCompatibility: false });
		ros.on('connection', () => {
			if (myGen !== rosReconnect.gen) return;
			rosReconnect.attempts = 0;
			ivgPorts.clearRosReconnectTimer(rosReconnect);
			setConnStatus('已连接（经网关）', true);
			startSubscriptions();
		});
		ros.on('error', () => {
			if (myGen !== rosReconnect.gen) return;
			setConnStatus('连接错误', false);
		});
		ros.on('close', () => {
			if (myGen !== rosReconnect.gen) return;
			stopClientGraspLoop();
			setConnStatus('已断开', false);
			unsubscribeAll();
			ivgPorts.scheduleRosReconnect(rosReconnect, connect, {
				maxAttempts: ROS_RECONNECT_MAX,
				onSchedule(delayMs, attempt, max) {
					setConnStatus(`已断开：${Math.round(delayMs / 1000)}s 后自动重连（${attempt}/${max}）`, false);
				},
				onExhausted() {
					setConnStatus('已断开（已达自动重连上限，请点「重新连接 ROS」）', false);
				}
			});
		});
		ros.connect(ivgPorts.rosbridgeWebSocketUrl());
	}

	/** 非当前策略的按钮区半透明，仍可点击 */
	function syncModeUi() {
		const work = $('mode-workpiece') && $('mode-workpiece').checked;
		const g = $('mode-graspnet') && $('mode-graspnet').checked;
		const elW = $('section-workpiece-btns');
		const elG = $('section-graspnet-btns');
		if (elW) elW.style.opacity = work ? '1' : '0.45';
		if (elG) elG.style.opacity = g ? '1' : '0.45';
	}

	function startJointChartTimer() {
		if (jointChartTimer != null) return;
		jointChartTimer = setInterval(() => drawJointChart(), JOINT_CHART_MS);
	}

	document.addEventListener('DOMContentLoaded', () => {
		void (async () => {
			await ivgPorts.loadRuntime();
			const wvEl0 = $('web-video-port');
			const rt0 = window.__IVG_RUNTIME || {};
			if (wvEl0 && !String(wvEl0.value || '').trim() && rt0.web_video_port != null) {
				wvEl0.value = String(rt0.web_video_port);
			}
			const wvportQ0 = new URLSearchParams(window.location.search).get('web_video_port');
			if (wvportQ0 && wvEl0) wvEl0.value = wvportQ0;

			loadTopicsFromStorage();

			(function initControlColWidth() {
				const el = $('control-col-width');
				if (!el) return;
				let v = 30;
				try {
					const s = localStorage.getItem(CONTROL_COL_PCT_KEY);
					if (s != null && s !== '') {
						const n = parseInt(s, 10);
						if (!isNaN(n)) v = Math.min(45, Math.max(22, n));
					}
				} catch (e) { /* ignore */ }
				el.value = String(v);
				document.documentElement.style.setProperty('--vision-control-col-pct', String(v));
				el.addEventListener('input', () => {
					const n = parseInt(el.value, 10);
					if (!isNaN(n)) {
						document.documentElement.style.setProperty('--vision-control-col-pct', String(n));
					}
				});
				el.addEventListener('change', () => {
					try {
						localStorage.setItem(CONTROL_COL_PCT_KEY, el.value);
					} catch (e) { /* ignore */ }
				});
			})();

		function restartVisionCamIfConnected() {
			if (ros && ros.isConnected) startSubscriptions();
		}
		const wvPortEl = $('web-video-port');
		if (wvPortEl) wvPortEl.addEventListener('change', restartVisionCamIfConnected);

		$('btn-reconnect').onclick = () => { connect(); };

		$('mode-workpiece').addEventListener('change', syncModeUi);
		$('mode-graspnet').addEventListener('change', syncModeUi);
		syncModeUi();

		// --- 抓取区：策略见文件头「抓取策略」---
		$('btn-wp-single-start').onclick = () => {
			callExecuteGrasp(useVisualFromMode());
		};
		$('btn-wp-single-stop').onclick = () => {
			stopClientGraspLoop();
			callSetBool('/loop_grasp_control', false);
			logSvc('已停止');
		};
		$('btn-wp-loop-start').onclick = () => {
			if (useVisualFromMode()) {
				stopClientGraspLoop();
				callSetBool('/loop_grasp_control', true);
				logSvc('循环：后端视觉');
			} else {
				stopClientGraspLoop();
				callSetBool('/loop_grasp_control', false, err => {
					if (err) return;
					startClientGraspLoop();
					logSvc('循环：页面非视觉');
				});
			}
		};
		$('btn-wp-loop-stop').onclick = () => {
			stopClientGraspLoop();
			callSetBool('/loop_grasp_control', false);
			logSvc('停循环');
		};

		// --- 点云 / GraspNet 节点默认服务名（launch 若改 remap 需同步改此常量或扩展 UI）---
		$('btn-gn-cap-start').onclick = () => { callSetBool('/graspnet_capture_control', true); };
		$('btn-gn-cap-stop').onclick = () => { callSetBool('/graspnet_capture_control', false); };
		$('btn-gn-loop-start').onclick = () => { callSetBool('/publish_grasps_worker_loop_control', true); };
		$('btn-gn-loop-stop').onclick = () => { callSetBool('/publish_grasps_worker_loop_control', false); };
		if ($('btn-gn-exec-once')) {
			$('btn-gn-exec-once').onclick = () => { callExecuteGrasp(false); };
		}

		$('btn-quick-swap').onclick = () => { callGripperSwap('gripper2'); };

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
				logSvc('已恢复内置默认话题（未自动重连时可点「重新连接 ROS」）');
			};
		}
		const btnSave = $('btn-topic-save-reconnect');
		if (btnSave) {
			btnSave.onclick = () => {
				saveTopicsToStorage();
				closeTopicSettingsModal();
				connect();
				logSvc('话题已保存并重新连接');
			};
		}
		document.addEventListener('keydown', ev => {
			if (ev.key !== 'Escape') return;
			if (!topicSettingsModalOpen()) return;
			closeTopicSettingsModal();
		});

			startJointChartTimer();
			ivgPorts.wireOnlineRosReconnect(rosReconnect, connect);
			connect();
		})();
	});
})();
