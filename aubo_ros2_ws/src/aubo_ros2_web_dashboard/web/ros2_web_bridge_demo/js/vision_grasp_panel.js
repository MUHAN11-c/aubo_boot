/* global ROSLIB */
/**
 * =============================================================================
 * vision_grasp_panel.js — 视觉抓取面板（仅 aubo_ros2_web_dashboard）
 * =============================================================================
 *
 * 目的
 *   浏览器经 roslib ↔ rosbridge（或 IVG FastAPI 网关转 rosbridge）订阅话题、调用服务，
 *   对接现场约定：VPE、graspnet_ros2、demo_driver、aubo_driver。不修改其它功能包源码。
 *
 * 连接
 *   - 直连：ws://当前主机:rosbridge_port，默认端口 9090，URL 参数 ?rosbridge_port=
 *   - 网关：勾选「经网关」或 ?ros_mode=gateway / ?gateway=1，先 POST /auth/token 取 JWT，
 *     再 ws://主机:gateway_port/ws/ros?token=…（与 ros_console.js 一致）。
 *   - 当前 HTML 使用 body.vision-grasp-no-gateway + CSS 隐藏网关控件；gatewayDisabled() 强制
 *     走直连。恢复网关：见 vision_grasp_panel.html / vision_grasp_panel.css 注释。
 *
 * 订阅（默认话题名可在页面修改，改后需「重连」）
 *   | 输入框 id        | 默认话题                  | 消息类型                          |
 *   |-----------------|---------------------------|-----------------------------------|
 *   | topic-color     | /camera/color/image_raw   | sensor_msgs/msg/Image             |
 *   | topic-robot     | /aubo_driver/robot_status | demo_interface/msg/RobotStatus    |
 *   | topic-joints    | /joint_states             | sensor_msgs/msg/JointState      |
 *   | topic-vpe-status| /system_status            | std_msgs/msg/String             |
 *   | topic-grasp-poses| /grasp_poses_base        | geometry_msgs/msg/PoseArray     |
 *   各 Topic queue_length=1，减轻前端压力。
 *
 *   话题可在「话题设置」模态中修改；「保存并重连」写入 localStorage 键 ivg_vision_grasp_topics_v1
 *   并在下次打开页面时恢复。「恢复默认」填回内置默认值并清除该键。
 *
 *   图像：支持 rgb8 / rgba8 / bgr8 / mono8、8uc1；其余 encoding 不绘制。
 *
 *   关节曲线 jointHistory：RobotStatus.joint_position_rad 与 JointState.position 前 6 维
 *   都会 pushJointSample。若两路同时高频发布，同一时间轴会混合两源（仅作趋势参考）。
 *   图表横轴为相对时间，纵轴为 6 关节在同一 min–max 内归一化，非独立纵轴。
 *
 * 服务
 *   | 名称                                    | 类型                          | 用途 |
 *   |----------------------------------------|-------------------------------|------|
 *   | /execute_single_grasp                | demo_interface/ExecuteGraspPose | 单次抓取：object_id + use_visual_estimation |
 *   | /loop_grasp_control                    | std_srvs/SetBool              | demo_driver 后端循环（线程内固定先视觉估计再抓取） |
 *   | /graspnet_capture_control              | std_srvs/SetBool              | GraspNet 采集开关 |
 *   | /publish_grasps_worker_loop_control    | std_srvs/SetBool              | GraspNet worker 循环 |
 *   | /run_gripper_swap                      | demo_interface/RunGripperSwap | 快换，按钮写死 direction=gripper2 |
 *
 * 抓取策略（单选 mode-workpiece / mode-graspnet）
 *   - useVisualFromMode()：工件=true → ExecuteGraspPose.use_visual_estimation=true；
 *     GraspNet=false → 非视觉（点云/参数位姿等由 demo_driver 解释）。
 *   - 工件区「单次」：callExecuteGrasp(useVisualFromMode())。
 *   - 工件区「循环」：
 *       工件（视觉）：stopClientGraspLoop 后 loop_grasp_control=true。注意 demo_driver
 *       ExecuteGraspPoseWorker::loopGraspThread 内写死先 estimatePoseFromVision，与单次非视觉不同源。
 *       GraspNet（非视觉）：先 loop_grasp_control=false（等服务返回）再 startClientGraspLoop：
 *       浏览器顺序调用 execute_single_grasp(false)，间隔 clientGraspLoop.pauseMs（默认 1500ms），
 *       因后端循环无法传 use_visual_estimation=false。
 *   - 停止：stopClientGraspLoop + loop_grasp_control=false；WebSocket close 时亦停浏览器循环。
 *
 * GraspNet 区「单次抓取（非视觉）」：固定 callExecuteGrasp(false)，不受单选影响，便于在 GraspNet
 * 流程下明确触发非视觉单次。
 *
 * DOM 约定（增删须同步）
 *   话题：topic-color / topic-robot / topic-joints / topic-vpe-status / topic-grasp-poses；
 *   模态：topic-settings-modal、btn-topic-settings-open/close、topic-settings-backdrop、
 *   btn-topic-restore-defaults、btn-topic-save-reconnect。
 *   其余：btn-wp-*、btn-gn-*、btn-quick-swap、cam-canvas、joint-chart、pose-text、
 *   vpe-status-text、graspnet-result-text、svc-log、section-*-btns。
 *
 * 更完整的面向使用者说明：仓库 aubo_ros2_web_dashboard/README.md「视觉抓取面板」。
 *
 * 脚本体为 ES2015+（let/const、箭头函数、模板字符串等），需现代浏览器；无构建转译步骤。
 * =============================================================================
 */
(() => {
    let ros = null;
    const subs = {};
    const IVG_JWT_KEY = 'ivg_gateway_jwt';

    const TOPIC_IDS = ['topic-color', 'topic-robot', 'topic-joints', 'topic-vpe-status', 'topic-grasp-poses'];
    const TOPIC_DEFAULTS = {
		'topic-color': '/camera/color/image_raw',
		'topic-robot': '/aubo_driver/robot_status',
		'topic-joints': '/joint_states',
		'topic-vpe-status': '/system_status',
		'topic-grasp-poses': '/grasp_poses_base'
	};
    const TOPIC_STORAGE_KEY = 'ivg_vision_grasp_topics_v1';

    function $(id) {
		return document.getElementById(id);
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
			const ok = TOPIC_IDS.every(id => typeof o[id] === 'string' && o[id].length > 0);
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

    /** 视觉面板暂关闭网关路径（与 HTML/CSS 隐藏网关 UI 一致）；恢复网关时删去 body 的 vision-grasp-no-gateway 并移除本判断 */
    function gatewayDisabled() {
		return document.body && document.body.classList.contains('vision-grasp-no-gateway');
	}

    function bridgePort() {
		const q = parseInt(new URLSearchParams(window.location.search).get('rosbridge_port') || '9090', 10);
		return q || 9090;
	}

    function gatewayPort() {
		const el = $('gateway-port');
		const p = el && el.value ? parseInt(String(el.value).replace(/[^\d]/g, ''), 10) : NaN;
		if (!isNaN(p) && p > 0) return p;
		const q = parseInt(new URLSearchParams(window.location.search).get('gateway_port') || '8765', 10);
		return q || 8765;
	}

    function rosModeGateway() {
		if (gatewayDisabled()) return false;
		const el = $('use-gateway');
		if (el && el.checked) return true;
		const p = new URLSearchParams(window.location.search);
		return p.get('ros_mode') === 'gateway' || p.get('gateway') === '1';
	}

    function getGatewayToken() {
		try {
			return sessionStorage.getItem(IVG_JWT_KEY) || '';
		} catch (e) {
			return '';
		}
	}

    function setGatewayToken(t) {
		try {
			if (t) sessionStorage.setItem(IVG_JWT_KEY, t);
			else sessionStorage.removeItem(IVG_JWT_KEY);
		} catch (e) { /* ignore */ }
	}

    function setConnStatus(text, ok) {
		const el = $('conn-status');
		if (!el) return;
		el.textContent = text;
		el.className = `status${ok ? ' ok' : ' off'}`;
	}

    function toUint8(data) {
		if (!data) return null;
		if (data instanceof Uint8Array) return data;
		if (typeof data === 'string') {
			try {
				const bin = atob(data);
				const out = new Uint8Array(bin.length);
				for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
				return out;
			} catch (e) {
				return null;
			}
		}
		if (Array.isArray(data)) return new Uint8Array(data);
		if (data.data && Array.isArray(data.data)) return new Uint8Array(data.data);
		return null;
	}

    function drawSensorImageToCanvas(msg, canvas) {
		if (!msg || !msg.width || !msg.height || !canvas) return;
		const enc = (msg.encoding || '').toLowerCase();
		const raw = toUint8(msg.data);
		if (!raw) return;
		const w = msg.width;
		const h = msg.height;
		canvas.width = w;
		canvas.height = h;
		const ctx = canvas.getContext('2d');
		const imgData = ctx.createImageData(w, h);
		const px = imgData.data;
		if (enc === 'rgb8' || enc === 'rgba8') {
			const step = enc === 'rgba8' ? 4 : 3;
			if (raw.length < w * h * step) return;
			let p = 0;
			for (let i = 0; i < w * h; i++) {
				px[p++] = raw[i * step];
				px[p++] = raw[i * step + 1];
				px[p++] = raw[i * step + 2];
				px[p++] = 255;
			}
		} else if (enc === 'bgr8') {
			if (raw.length < w * h * 3) return;
			let p2 = 0;
			for (let j = 0; j < w * h; j++) {
				px[p2++] = raw[j * 3 + 2];
				px[p2++] = raw[j * 3 + 1];
				px[p2++] = raw[j * 3];
				px[p2++] = 255;
			}
		} else if (enc === 'mono8' || enc === '8uc1') {
			if (raw.length < w * h) return;
			let p3 = 0;
			for (let k = 0; k < w * h; k++) {
				const g = raw[k];
				px[p3++] = g;
				px[p3++] = g;
				px[p3++] = g;
				px[p3++] = 255;
			}
		} else {
			return;
		}
		ctx.putImageData(imgData, 0, 0);
		const host = canvas.parentElement;
		if (host) {
			if (document.body.classList.contains('ivg-single-screen')) {
				canvas.style.width = '';
				canvas.style.height = '';
			} else {
				canvas.style.width = '100%';
				canvas.style.height = 'auto';
			}
		} else {
			canvas.style.width = '';
			canvas.style.height = '';
		}
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
		const lines = [`  frame_id: ${m.header.frame_id}PoseArray 数量: ${n}${m.header && m.header.frame_id ? '  frame_id: ' + m.header.frame_id : ''}`];
		if (n > 0) {
			const p = m.poses[0].position;
			const o = m.poses[0].orientation;
			lines.push(`首点 position: ${[p.x, p.y, p.z].map(x => Number(x).toFixed(4)).join(', ')}`);
			lines.push(`首点 orientation: ${[o.x, o.y, o.z, o.w].map(x => Number(x).toFixed(4)).join(', ')}`);
		}
		return lines.join('\n');
	}

    function unsubscribeAll() {
		Object.keys(subs).forEach(k => {
			try {
				if (subs[k] && subs[k].dispose) subs[k].dispose();
			} catch (e) { /* ignore */ }
			subs[k] = null;
		});
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

		subs.color = new ROSLIB.Topic({
			ros,
			name: colorTopic,
			messageType: 'sensor_msgs/msg/Image',
			queue_length: 1
		});
		subs.color.subscribe(msg => {
			drawSensorImageToCanvas(msg, $('cam-canvas'));
		});

		subs.robot = new ROSLIB.Topic({
			ros,
			name: robotTopic,
			messageType: 'demo_interface/msg/RobotStatus',
			queue_length: 1
		});
		subs.robot.subscribe(m => {
			$('pose-text').textContent = formatRobotStatus(m);
			if (Array.isArray(m.joint_position_rad) && m.joint_position_rad.length >= 6) {
				pushJointSample(m.joint_position_rad.slice(0, 6));
			}
		});

		subs.joints = new ROSLIB.Topic({
			ros,
			name: jointTopic,
			messageType: 'sensor_msgs/msg/JointState',
			queue_length: 1
		});
		subs.joints.subscribe(m => {
			if (!m.position || m.position.length < 6) return;
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
		if (ros) {
			try {
				ros.close();
			} catch (e) { /* ignore */ }
		}
		if (rosModeGateway()) {
			if (!getGatewayToken()) {
				ros = null;
				setConnStatus('请先登录网关', false);
				return;
			}
		}
		ros = new ROSLIB.Ros({ groovyCompatibility: false });
		const h = window.location.hostname || '127.0.0.1';
		ros.on('connection', () => {
			setConnStatus('已连接', true);
			startSubscriptions();
		});
		ros.on('error', () => {
			setConnStatus('连接错误', false);
		});
		ros.on('close', () => {
			stopClientGraspLoop();
			setConnStatus('已断开', false);
			unsubscribeAll();
		});
		if (rosModeGateway()) {
			ros.connect(`ws://${h}:${gatewayPort()}/ws/ros?token=${encodeURIComponent(getGatewayToken())}`);
		} else {
			ros.connect(`ws://${h}:${bridgePort()}`);
		}
	}

    function gatewayLogin() {
		const u = ($('gw-user') && $('gw-user').value || '').trim();
		const pw = ($('gw-pass') && $('gw-pass').value) || '';
		if (!u) {
			alert('请输入用户名');
			return;
		}
		const h = window.location.hostname || '127.0.0.1';
		const body = new URLSearchParams();
		body.set('username', u);
		body.set('password', pw);
		fetch(`http://${h}:${gatewayPort()}/auth/token`, {
			method: 'POST',
			headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
			body: body.toString()
		})
			.then(r => {
				if (!r.ok) return r.text().then(t => { throw new Error(t || String(r.status)); });
				return r.json();
			})
			.then(j => {
				if (!j || !j.access_token) throw new Error('无 access_token');
				setGatewayToken(j.access_token);
				connect();
			})
			.catch(e => {
				alert(`登录失败: ${e && e.message ? e.message : e}`);
			});
	}

    function gatewayLogout() {
		setGatewayToken('');
		if (ros) try { ros.close(); } catch (e) { /* ignore */ }
		ros = null;
		setConnStatus('已登出', false);
		unsubscribeAll();
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

    function chartLoop() {
		drawJointChart();
		requestAnimationFrame(() => {
			setTimeout(chartLoop, 200);
		});
	}

    document.addEventListener('DOMContentLoaded', () => {
		loadTopicsFromStorage();

		const p = new URLSearchParams(window.location.search);
		if (!gatewayDisabled() && (p.get('ros_mode') === 'gateway' || p.get('gateway') === '1')) {
			const cb = $('use-gateway');
			if (cb) cb.checked = true;
		}
		const gp = p.get('gateway_port');
		if (!gatewayDisabled() && gp && $('gateway-port')) $('gateway-port').value = gp;

		$('btn-reconnect').onclick = () => { connect(); };
		const ug = $('use-gateway');
		if (ug) ug.addEventListener('change', () => { connect(); });
		$('btn-gw-login').onclick = gatewayLogin;
		$('btn-gw-logout').onclick = gatewayLogout;

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

		chartLoop();
		connect();
	});
})();
