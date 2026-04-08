/* global ROSLIB, ROS2D, createjs, THREE */
/* topics_lab：话题/服务/参数/图/2D/3D。Image（非 Compressed）→ web_video_server MJPEG（?web_video_port=）。 */
(() => {
	/** 当前 ROSLIB.Ros 实例（直连 rosbridge WebSocket）。 */
	let ros = null;
	let activeTopic = null;
	/** 每次取消订阅 +1，用于丢弃旧 Topic 在 dispose 后仍可能到达的回调（避免 raw JSON / 视图刷成上一话题） */
	let subscriptionSeq = 0;
	/** 侧边栏异步列表请求代数，避免快速输入过滤时旧 getTopics 覆盖新结果 */
	let topicsListReq = 0;
	let servicesListReq = 0;
	let nodesListReq = 0;
	let actionsListReq = 0;
	let paramsListReq = 0;
	let paramFetchSeq = 0;
	/** 节点关系图串行构建代数；切换离开「图」标签或再次点击计算时递增，丢弃过期回调 */
	let graphBuildSeq = 0;
	/** 最近一次计算结果，用于在「逻辑图 / 架构图」间切换不重查 rosapi */
	let graphModel = null;
	/** 侧栏列表并发 rosapi 请求数，用于统一「加载中」与刷新按钮禁用（参考常见控制台 UX） */
	let sidebarListInflight = 0;
	let filterDebounceTimer = null;
	let throttleMs = 80;
	let lastEmit = 0;
	let selectedName = '';
	let selectedType = '';
	let tab = 'topics';
	let selectedService = '';
	let selectedParam = '';
	let selectedAction = '';
	let mapTopicSub = null;
	let scanTopicSub = null;
	let viewer2d = null;
	let gridRoot = null;
	let scanShape = null;
	let pcTopicSub = null;
	let scan3TopicSub = null;
	let threeRenderer = null;
	let threeScene = null;
	let threeCamera = null;
	let threeControls = null;
	let pointsObj = null;
	let anim3d = null;
	/** 话题页 viz 渲染合并到下一帧，避免 rosbridge 回调内直接改 DOM 造成掉帧（RobotWebTools/roslib 高频消息场景） */
	let topicVizRaf = null;
	/** 当前话题为 web_video_server MJPEG，未创建 ROSLIB.Topic */
	let webVideoStreamMode = false;
	let pcUpdateRaf = null;
	let pcPendingMsg = null;
	let scan3UpdateRaf = null;
	let scan3PendingMsg = null;

	/** 亮色主题画布色（与 topics_lab.css 一致） */
	const VIZ_CANVAS_BG = '#e8ecf2';
	const LASER_STROKE = '#15803d';
	const LASER_ARC_STROKE = '#94a3b8';
	const GRAPH_BG = '#f1f5f9';
	const GRAPH_EDGE = 'rgba(37, 99, 235, 0.38)';
	const GRAPH_NODE = '#16a34a';
	const GRAPH_LABEL = '#334155';
	const GRAPH_ARCH_BOX = 'rgba(255, 255, 255, 0.94)';
	const GRAPH_ARCH_STROKE = '#94a3b8';
	const GRAPH_ARCH_TITLE = '#0f172a';
	const GRAPH_ARCH_MEMBER = '#475569';
	const GRAPH_ARCH_EDGE = 'rgba(37, 99, 235, 0.28)';
	const ROS2D_VIEWER_BG = '#eef1f6';

	/* ---------- IVG 顶栏：话题排序、VPE FastAPI 端口、快捷订阅与常用服务预设 ---------- */
	/** 与 start_IVG_graspnet_points_fastapi.sh + graspnet_demo_points_with_tf 默认一致 */
	const IVG_TOPIC_ORDER = [
		'/camera/color/image_raw',
		'/camera/depth/image_raw',
		'/camera/depth_registered/points',
		'/grasp_markers',
		'/grasp_poses_base',
		'/system_status',
		'/joint_states',
		'/tf',
		'/tf_static'
	];

	function topicIvgRank(topicName) {
		const idx = IVG_TOPIC_ORDER.indexOf(topicName);
		if (idx !== -1) return idx;
		if (topicName.indexOf('/camera/') === 0) return 20;
		if (topicName.indexOf('grasp') !== -1) return 25;
		return 500;
	}

	function ivgFastapiBase() {
		const el = $('ivg-fastapi-port');
		const p = (el && el.value ? el.value : '8088').replace(/[^\d]/g, '') || '8088';
		const h = window.location.hostname || '127.0.0.1';
		return `http://${h}:${p}`;
	}

	function ivgSubscribeFixed(name, fallbackType) {
		if (!ros || !ros.isConnected) return;
		switchTab('topics');
		ros.getTopicType(name, typ => {
			subscribe(name, typ);
			refreshTopics();
		}, () => {
			if (fallbackType) subscribe(name, fallbackType);
			else $('viz-body').innerHTML = `<p class="hint">未找到话题 <code>${name}</code>，请确认栈已启动。</p>`;
			refreshTopics();
		});
	}

	function ivgTrySubscribeMarkers() {
		if (!ros || !ros.isConnected) return;
		switchTab('topics');
		ros.getTopicType('/grasp_markers', t => {
			subscribe('/grasp_markers', t);
			refreshTopics();
		}, () => {
			ros.getTopicType('grasp_markers', t2 => {
				subscribe('grasp_markers', t2);
				refreshTopics();
			}, () => {
				$('viz-body').innerHTML = '<p class="hint">未找到 grasp_markers，请在列表中查找 MarkerArray 话题。</p>';
				refreshTopics();
			});
		});
	}

	function ivg3dPointsPreset() {
		switchTab('view3d');
		$('pc-topic').value = '/camera/depth_registered/points';
		$('scan3-topic').value = '';
	}

	function ivg3dClearPreset() {
		$('pc-topic').value = '';
		$('scan3-topic').value = '';
	}

	function ivgPresetService(svcName, typeStr, jsonStr) {
		switchTab('services');
		selectedService = svcName;
		if ($('svc-selection')) $('svc-selection').textContent = svcName;
		if ($('svc-type')) $('svc-type').value = typeStr;
		if ($('svc-req-json')) $('svc-req-json').value = jsonStr;
		if ($('svc-resp')) $('svc-resp').textContent = '';
		/* switchTab 已 refreshSidebar→refreshServices，勿再调用以免连续两次 ++servicesListReq 制造竞态 */
	}

	function bindIvgBar() {
		const bar = $('ivg-bar');
		if (!bar) return;
		const link = $('ivg-link-fastapi');
		const portInput = $('ivg-fastapi-port');
		function syncFastapiLinkTitle() {
			if (link) {
				const b = ivgFastapiBase();
				link.title = `${b}/ 与 ${b}/health（与 WEB_PORT 默认 8088 一致时可改上方端口）`;
			}
		}
		if (link) {
			link.addEventListener('click', e => {
				e.preventDefault();
				window.open(`${ivgFastapiBase()}/`, '_blank', 'noopener');
			});
		}
		if (portInput) {
			portInput.addEventListener('input', syncFastapiLinkTitle);
			syncFastapiLinkTitle();
		}
		bar.addEventListener('click', ev => {
			const btn = ev.target.closest('[data-ivg]');
			if (!btn) return;
			const k = btn.getAttribute('data-ivg');
			if (k === 'topics-color') ivgSubscribeFixed('/camera/color/image_raw', 'sensor_msgs/msg/Image');
			else if (k === 'topics-depth') ivgSubscribeFixed('/camera/depth/image_raw', 'sensor_msgs/msg/Image');
			else if (k === 'topics-points') ivgSubscribeFixed('/camera/depth_registered/points', 'sensor_msgs/msg/PointCloud2');
			else if (k === 'topics-markers') ivgTrySubscribeMarkers();
			else if (k === 'topics-poses') ivgSubscribeFixed('/grasp_poses_base', 'geometry_msgs/msg/PoseArray');
			else if (k === '3d-points') ivg3dPointsPreset();
			else if (k === '3d-clear') ivg3dClearPreset();
			else if (k === 'svc-list-templates') {
				ivgPresetService('/list_templates', 'interface/srv/ListTemplates', '{"workpiece_id": ""}');
			} else if (k === 'svc-graspnet-on') {
				ivgPresetService('/graspnet_capture_control', 'std_srvs/srv/SetBool', '{"data": true}');
			} else if (k === 'svc-graspnet-off') {
				ivgPresetService('/graspnet_capture_control', 'std_srvs/srv/SetBool', '{"data": false}');
			} else if (k === 'svc-worker-loop-on') {
				ivgPresetService('/publish_grasps_worker_loop_control', 'std_srvs/srv/SetBool', '{"data": true}');
			} else if (k === 'svc-worker-loop-off') {
				ivgPresetService('/publish_grasps_worker_loop_control', 'std_srvs/srv/SetBool', '{"data": false}');
			} else if (k === 'svc-loop-grasp-on') {
				ivgPresetService('/loop_grasp_control', 'std_srvs/srv/SetBool', '{"data": true}');
			} else if (k === 'svc-loop-grasp-off') {
				ivgPresetService('/loop_grasp_control', 'std_srvs/srv/SetBool', '{"data": false}');
			} else if (k === 'svc-exec-grasp') {
				ivgPresetService(
					'/execute_single_grasp',
					'demo_interface/srv/ExecuteGraspPose',
					'{\n  "object_id": "填写工件ID",\n  "use_visual_estimation": true\n}'
				);
			} else if (k === 'svc-gripper2') {
				ivgPresetService('/run_gripper_swap', 'demo_interface/srv/RunGripperSwap', '{"direction": "gripper2"}');
			} else if (k === 'svc-standardize') {
				ivgPresetService('/standardize_template', 'interface/srv/StandardizeTemplate', '{"workpiece_id": ""}');
			} else if (k === 'svc-estimate-hint') {
				switchTab('services');
				if ($('svc-resp')) {
					$('svc-resp').textContent = [
						'/estimate_pose（interface/srv/EstimatePose）请求需嵌入 sensor_msgs/Image 等，JSON 体积大，',
						'浏览器里不易手写。建议：',
						'1）用顶部「VPE Web」打开 FastAPI（默认 8088，与 start_IVG_graspnet_points_fastapi.sh 中 WEB_PORT 一致）；',
						'2）或在终端: ros2 interface show interface/srv/EstimatePose 后 ros2 service call …',
						''
					].join('\n');
				}
			} else if (k === 'topics-status') {
				ivgSubscribeFixed('/system_status', 'std_msgs/msg/String');
			}
		});
	}

	function bridgePort() {
		const q = parseInt(new URLSearchParams(window.location.search).get('rosbridge_port') || '9090', 10);
		return q || 9090;
	}

	function webVideoPort() {
		const el = $('web-video-port');
		const pv = el && el.value ? parseInt(String(el.value).replace(/[^\d]/g, ''), 10) : NaN;
		if (!isNaN(pv) && pv > 0) return pv;
		const q = parseInt(new URLSearchParams(window.location.search).get('web_video_port') || '8089', 10);
		return q || 8089;
	}

	function reconnectImageTopicIfNeeded() {
		if (tab !== 'topics' || !selectedName || !selectedType || !ros || !ros.isConnected) return;
		if (!typeMatch(selectedType, 'Image') || typeMatch(selectedType, 'CompressedImage')) return;
		subscribe(selectedName, selectedType);
	}

	/** 静态页 id 不变时缓存 getElementById，减轻 topics 高频回调与侧栏刷新时的查找开销 */
	const elCache = Object.create(null);
	function $(id) {
		if (Object.prototype.hasOwnProperty.call(elCache, id)) return elCache[id];
		const n = document.getElementById(id);
		elCache[id] = n;
		return n;
	}

	function setStatus(text, ok) {
		const el = $('conn-status');
		if (!el) return;
		el.textContent = text;
		el.className = 'status';
		if (ok) el.classList.add('ok');
		else if (/断开|错误|未连接|失败|连接错误/i.test(String(text))) el.classList.add('off');
	}

	function beginSidebarListFetch() {
		sidebarListInflight++;
		const sb = document.querySelector('.sidebar');
		const br = $('btn-refresh');
		const sl = $('sidebar-list');
		if (sb) sb.classList.add('sidebar-loading');
		if (br) {
			br.disabled = true;
			br.setAttribute('aria-busy', 'true');
		}
		if (sl && ros && ros.isConnected) {
			sl.innerHTML = '<p class="hint sidebar-loading-hint" role="status" aria-live="polite">正在加载列表…</p>';
		}
	}

	function endSidebarListFetch() {
		sidebarListInflight = Math.max(0, sidebarListInflight - 1);
		if (sidebarListInflight > 0) return;
		const sb = document.querySelector('.sidebar');
		const br = $('btn-refresh');
		if (sb) sb.classList.remove('sidebar-loading');
		if (br) {
			br.disabled = false;
			br.setAttribute('aria-busy', 'false');
		}
	}

	function setSidebarContextLabel(t) {
		const map = {
			topics: '话题',
			services: '服务',
			actions: '动作服务器',
			params: '参数',
			graph: '节点',
			view2d: '话题（2D 选图）',
			view3d: '话题（3D 选图）'
		};
		const el = $('sidebar-context');
		if (el) el.textContent = map[t] || '列表';
	}

	function syncLocalHostLinks() {
		const h = window.location.hostname || '127.0.0.1';
		const he = $('ivg-link-handeye');
		if (he) he.href = `http://${h}:8080/`;
	}

	function flashCopyButton(btn, ok) {
		if (!btn) return;
		if (btn._copyOrig == null) btn._copyOrig = btn.textContent;
		btn.textContent = ok ? '已复制' : '失败';
		btn.disabled = true;
		setTimeout(() => {
			btn.textContent = btn._copyOrig;
			btn.disabled = false;
		}, 900);
	}

	function copyToClipboard(text, btn) {
		text = (text || '').trim();
		if (!text) {
			alert('没有可复制内容');
			return;
		}
		function ok() {
			flashCopyButton(btn, true);
		}
		function fail() {
			flashCopyButton(btn, false);
		}
		if (navigator.clipboard && navigator.clipboard.writeText) {
			navigator.clipboard.writeText(text).then(ok, () => {
				try {
					const ta = document.createElement('textarea');
					ta.value = text;
					ta.style.position = 'fixed';
					ta.style.left = '-9999px';
					document.body.appendChild(ta);
					ta.select();
					document.execCommand('copy');
					document.body.removeChild(ta);
					ok();
				} catch (e2) {
					alert('复制失败（浏览器限制）');
					fail();
				}
			});
			return;
		}
		try {
			const ta2 = document.createElement('textarea');
			ta2.value = text;
			ta2.style.position = 'fixed';
			ta2.style.left = '-9999px';
			document.body.appendChild(ta2);
			ta2.select();
			document.execCommand('copy');
			document.body.removeChild(ta2);
			ok();
		} catch (e3) {
			alert('复制失败');
			fail();
		}
	}

	function stopTopicSubscription() {
		if (tab !== 'topics') switchTab('topics');
		unsubscribe();
		selectedName = '';
		selectedType = '';
		$('selection-label').textContent = '未选择话题（已停止订阅）';
		$('viz-body').innerHTML = '<p class="hint">在左侧点击话题订阅，或使用顶部 IVG 快捷条。过滤框输入后会稍候再刷新列表。</p>';
		$('raw-pre').textContent = '';
		const cv = $('viz-canvas');
		if (cv) cv.style.display = 'none';
		refreshTopics();
	}

	function safeJson(obj, maxLen) {
		try {
			const s = JSON.stringify(obj, null, 2);
			if (maxLen && s.length > maxLen) {
				return `${s.slice(0, maxLen)}\n…\n(truncated, total ${s.length} chars)`;
			}
			return s;
		} catch (e) {
			return String(obj);
		}
	}

	/** 避免对 Image / PointCloud2 等整包 JSON.stringify（会卡主线程数秒）；右侧仅展示元数据 */
	function rawPreviewForMessage(msgType, msg, maxLen) {
		if (!msg) return '';
		const omit = hint => `[omitted: ${hint} — 降低卡顿；看图/3D 画布或 RViz]`;
		if (typeMatch(msgType, 'Image') && !typeMatch(msgType, 'CompressedImage')) {
			let in0 = 0;
			if (Array.isArray(msg.data)) in0 = msg.data.length;
			else if (typeof msg.data === 'string') in0 = msg.data.length;
			return safeJson({
				header: msg.header,
				height: msg.height,
				width: msg.width,
				encoding: msg.encoding,
				is_bigendian: msg.is_bigendian,
				step: msg.step,
				data: omit(`image payload ~${in0} array el / base64 chars`)
			}, maxLen);
		}
		if (typeMatch(msgType, 'CompressedImage')) {
			let ic = 0;
			if (typeof msg.data === 'string') ic = msg.data.length;
			else if (Array.isArray(msg.data)) ic = msg.data.length;
			return safeJson({
				header: msg.header,
				format: msg.format,
				data: omit(`compressed ~${ic} chars`)
			}, maxLen);
		}
		if (typeMatch(msgType, 'PointCloud2')) {
			let ip = 0;
			if (Array.isArray(msg.data)) ip = msg.data.length;
			else if (typeof msg.data === 'string') ip = msg.data.length;
			return safeJson({
				header: msg.header,
				height: msg.height,
				width: msg.width,
				fields: msg.fields,
				is_bigendian: msg.is_bigendian,
				point_step: msg.point_step,
				row_step: msg.row_step,
				is_dense: msg.is_dense,
				data: omit(`pointcloud payload ~${ip} units`)
			}, maxLen);
		}
		if (typeMatch(msgType, 'LaserScan') && Array.isArray(msg.ranges)) {
			return safeJson({
				header: msg.header,
				angle_min: msg.angle_min,
				angle_max: msg.angle_max,
				angle_increment: msg.angle_increment,
				time_increment: msg.time_increment,
				scan_time: msg.scan_time,
				range_min: msg.range_min,
				range_max: msg.range_max,
				ranges: omit(`${msg.ranges.length} samples`)
			}, maxLen);
		}
		if (typeMatch(msgType, 'OccupancyGrid') && msg.info && Array.isArray(msg.data)) {
			return safeJson({
				header: msg.header,
				info: msg.info,
				data: omit(`${msg.data.length} cells`)
			}, maxLen);
		}
		return safeJson(msg, maxLen);
	}

	/** Normalize rosbridge / ROS2 message byte payloads to Uint8Array */
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

	function typeMatch(msgType, needle) {
		if (!msgType) return false;
		return msgType === needle || msgType.indexOf(needle) !== -1;
	}

	function renderScalar(msg) {
		if (msg && Object.prototype.hasOwnProperty.call(msg, 'data')) {
			const d = msg.data;
			if (typeof d === 'boolean' || typeof d === 'number' || typeof d === 'string') {
				return `<div class="viz-scalar">${String(d)}</div>`;
			}
		}
		return null;
	}

	function renderTwist(msg) {
		let t = msg;
		if (msg && msg.twist) t = msg.twist;
		if (!t || !t.linear) return null;
		const L = t.linear;
		const A = t.angular || {};
		const rows = [
			['linear.x', L.x], ['linear.y', L.y], ['linear.z', L.z],
			['angular.x', A.x], ['angular.y', A.y], ['angular.z', A.z]
		];
		let html = '<table class="viz-table"><tbody>';
		for (let i = 0; i < rows.length; i++) {
			html += `<tr><th>${rows[i][0]}</th><td>${rows[i][1]}</td></tr>`;
		}
		html += '</tbody></table>';
		return html;
	}

	function renderPose(msg) {
		let p = msg;
		if (msg && msg.pose) p = msg.pose;
		if (!p || !p.position) return null;
		const pos = p.position;
		const o = p.orientation || {};
		let html = '<table class="viz-table"><tbody>';
		html += `<tr><th>position</th><td>x=${pos.x} y=${pos.y} z=${pos.z}</td></tr>`;
		html += `<tr><th>orientation</th><td>x=${o.x} y=${o.y} z=${o.z} w=${o.w}</td></tr>`;
		html += '</tbody></table>';
		return html;
	}

	function renderJointState(msg) {
		if (!msg || !Array.isArray(msg.name)) return null;
		let html = '<table class="viz-table"><thead><tr><th>name</th><th>position</th><th>velocity</th><th>effort</th></tr></thead><tbody>';
		const n = msg.name.length;
		for (let i = 0; i < n; i++) {
			html += `${msg.effort[i] !== undefined}${msg.velocity[i] !== undefined}${msg.position[i] !== undefined}<tr><td>${msg.name[i]}</td><td>${msg.position && msg.position[i] !== undefined ? msg.position[i] : ''}</td><td>${msg.velocity && msg.velocity[i] !== undefined ? msg.velocity[i] : ''}</td><td>${msg.effort && msg.effort[i] !== undefined ? msg.effort[i] : ''}</td></tr>`;
		}
		html += '</tbody></table>';
		return html;
	}

	function renderImu(msg) {
		if (!msg) return null;
		let html = '<table class="viz-table"><tbody>';
		if (msg.orientation) {
			const o = msg.orientation;
			html += `<tr><th>orientation</th><td>x=${o.x} y=${o.y} z=${o.z} w=${o.w}</td></tr>`;
		}
		if (msg.angular_velocity) {
			const av = msg.angular_velocity;
			html += `<tr><th>angular_velocity</th><td>x=${av.x} y=${av.y} z=${av.z}</td></tr>`;
		}
		if (msg.linear_acceleration) {
			const la = msg.linear_acceleration;
			html += `<tr><th>linear_acceleration</th><td>x=${la.x} y=${la.y} z=${la.z}</td></tr>`;
		}
		html += '</tbody></table>';
		return html;
	}

	function renderBattery(msg) {
		if (!msg) return null;
		let html = '<table class="viz-table"><tbody>';
		const keys = ['voltage', 'temperature', 'current', 'charge', 'capacity', 'percentage', 'power_supply_status'];
		for (let i = 0; i < keys.length; i++) {
			if (Object.prototype.hasOwnProperty.call(msg, keys[i])) {
				html += `<tr><th>${keys[i]}</th><td>${msg[keys[i]]}</td></tr>`;
			}
		}
		html += '</tbody></table>';
		return html;
	}

	function renderRange(msg) {
		if (!msg || typeof msg.range !== 'number') return null;
		let html = '<table class="viz-table"><tbody>';
		html += `<tr><th>range</th><td>${msg.range}</td></tr>`;
		if (msg.min_range !== undefined) html += `<tr><th>min_range</th><td>${msg.min_range}</td></tr>`;
		if (msg.max_range !== undefined) html += `<tr><th>max_range</th><td>${msg.max_range}</td></tr>`;
		if (msg.radiation_type !== undefined) html += `<tr><th>radiation_type</th><td>${msg.radiation_type}</td></tr>`;
		html += '</tbody></table>';
		return html;
	}

	function renderNavSatFix(msg) {
		if (!msg || typeof msg.latitude !== 'number') return null;
		let html = '<table class="viz-table"><tbody>';
		html += `<tr><th>latitude</th><td>${msg.latitude}</td></tr>`;
		html += `<tr><th>longitude</th><td>${msg.longitude}</td></tr>`;
		html += `<tr><th>altitude</th><td>${msg.altitude}</td></tr>`;
		if (msg.status) html += `<tr><th>status</th><td>${safeJson(msg.status, 400)}</td></tr>`;
		html += '</tbody></table>';
		return html;
	}

	function renderLaserScan(msg, canvas) {
		if (!msg || !Array.isArray(msg.ranges)) return '<p class="hint">无效的 LaserScan</p>';
		const w = canvas.parentElement.clientWidth || 400;
		const h = Math.min(360, Math.floor(w * 0.6));
		canvas.width = w;
		canvas.height = h;
		const ctx = canvas.getContext('2d');
		ctx.fillStyle = VIZ_CANVAS_BG;
		ctx.fillRect(0, 0, w, h);
		const cx = w * 0.5;
		const cy = h * 0.92;
		const maxR = Math.min(cx, cy) * 0.95;
		const amin = msg.angle_min;
		const inc = msg.angle_increment;
		const ranges = msg.ranges;
		ctx.strokeStyle = LASER_STROKE;
		ctx.lineWidth = 1;
		ctx.beginPath();
		for (let i = 0; i < ranges.length; i++) {
			const r = ranges[i];
			if (!isFinite(r) || r <= 0 || r > 1e6) continue;
			const ang = amin + i * inc;
			const px = cx + r / (msg.range_max || 10) * maxR * Math.cos(ang);
			const py = cy - r / (msg.range_max || 10) * maxR * Math.sin(ang);
			if (i === 0) ctx.moveTo(px, py);
			else ctx.lineTo(px, py);
		}
		ctx.stroke();
		ctx.strokeStyle = LASER_ARC_STROKE;
		ctx.beginPath();
		ctx.arc(cx, cy, maxR, Math.PI, 2 * Math.PI);
		ctx.stroke();
		return '<p class="hint">LaserScan 极坐标预览（简化，非 RViz 语义）</p>';
	}

	function renderOccupancyGrid(msg, canvas) {
		if (!msg || !msg.info || !Array.isArray(msg.data)) return '<p class="hint">Invalid OccupancyGrid</p>';
		const W = msg.info.width;
		const H = msg.info.height;
		if (W * H !== msg.data.length) {
			return '<p class="hint">Grid size mismatch (width×height vs data)</p>';
		}
		const maxW = canvas.parentElement.clientWidth || 400;
		const scale = Math.max(1, Math.floor(maxW / W));
		canvas.width = W * scale;
		canvas.height = H * scale;
		const ctx = canvas.getContext('2d');
		const img = ctx.createImageData(W, H);
		const d = img.data;
		for (let i = 0; i < msg.data.length; i++) {
			const v = msg.data[i];
			const o = i * 4;
			if (v < 0) { d[o] = 80; d[o + 1] = 80; d[o + 2] = 120; d[o + 3] = 255; }
			else if (v === 0) { d[o] = 255; d[o + 1] = 255; d[o + 2] = 255; d[o + 3] = 255; }
			else { const g = 255 - Math.min(255, v * 2); d[o] = g; d[o + 1] = g; d[o + 2] = g; d[o + 3] = 255; }
		}
		const tmp = document.createElement('canvas');
		tmp.width = W;
		tmp.height = H;
		tmp.getContext('2d').putImageData(img, 0, 0);
		ctx.imageSmoothingEnabled = false;
		ctx.drawImage(tmp, 0, 0, W * scale, H * scale);
		return `<p class="hint">resolution ${msg.info.resolution} m/cell · origin (${msg.info.origin.position.x}, ${msg.info.origin.position.y})</p>`;
	}

	function renderImage(msg, canvas) {
		if (!msg || !msg.width || !msg.height) return '<p class="hint">Invalid Image</p>';
		const enc = (msg.encoding || '').toLowerCase();
		const raw = toUint8(msg.data);
		if (!raw) return `<p class="hint">Could not decode image data (encoding ${enc})</p>`;
		const w = msg.width;
		const h = msg.height;
		canvas.width = w;
		canvas.height = h;
		const ctx = canvas.getContext('2d');
		const imgData = ctx.createImageData(w, h);
		const px = imgData.data;
		if (enc === 'rgb8' || enc === 'rgba8') {
			const step = enc === 'rgba8' ? 4 : 3;
			const need = w * h * step;
			if (raw.length < need) return '<p class="hint">Image buffer too short</p>';
			let p = 0;
			for (let i = 0; i < w * h; i++) {
				px[p++] = raw[i * step];
				px[p++] = raw[i * step + 1];
				px[p++] = raw[i * step + 2];
				px[p++] = 255;
			}
		} else if (enc === 'bgr8') {
			if (raw.length < w * h * 3) return '<p class="hint">Image buffer too short</p>';
			let p2 = 0;
			for (let j = 0; j < w * h; j++) {
				px[p2++] = raw[j * 3 + 2];
				px[p2++] = raw[j * 3 + 1];
				px[p2++] = raw[j * 3];
				px[p2++] = 255;
			}
		} else if (enc === 'mono8' || enc === '8uc1') {
			if (raw.length < w * h) return '<p class="hint">Image buffer too short</p>';
			let p3 = 0;
			for (let k = 0; k < w * h; k++) {
				const g = raw[k];
				px[p3++] = g;
				px[p3++] = g;
				px[p3++] = g;
				px[p3++] = 255;
			}
		} else {
			return `<p class="hint">Encoding <code>${enc}</code> — use Raw JSON or RViz; supported: rgb8, bgr8, rgba8, mono8</p>`;
		}
		ctx.putImageData(imgData, 0, 0);
		const mw = canvas.parentElement.clientWidth || w;
		if (w > mw) {
			canvas.style.width = '100%';
			canvas.style.height = 'auto';
		}
		return '';
	}

	function renderCompressedImage(msg) {
		if (!msg || !msg.format || !msg.data) return null;
		const u8 = toUint8(msg.data);
		if (!u8) return '<p class="hint">无法解析压缩图像数据</p>';
		if (window.__labBlobUrl) {
			try { URL.revokeObjectURL(window.__labBlobUrl); } catch (e) { /* ignore */ }
		}
		const blob = new Blob([u8], { type: `image/${msg.format.indexOf('png') !== -1 ? 'png' : 'jpeg'}` });
		window.__labBlobUrl = URL.createObjectURL(blob);
		return `<img src="${window.__labBlobUrl}" alt="compressed" style="max-width:100%;height:auto;border-radius:4px"/>`;
	}

	function renderPointCloud2(msg) {
		if (!msg || !msg.fields) return null;
		let rows = '<table class="viz-table"><tbody>';
		rows += `<tr><th>height × width</th><td>${msg.height} × ${msg.width}</td></tr>`;
		rows += `<tr><th>point_step</th><td>${msg.point_step}</td></tr>`;
		rows += `<tr><th>row_step</th><td>${msg.row_step}</td></tr>`;
		rows += `<tr><th>fields</th><td>${msg.fields.map(f => f.name + ' (' + f.datatype + '×' + f.count + ')').join(', ')}</td></tr>`;
		if (msg.header && msg.header.frame_id) {
			rows += `<tr><th>frame_id</th><td>${msg.header.frame_id}</td></tr>`;
		}
		rows += '</tbody></table>';
		const raw = toUint8(msg.data);
		const bytes = raw ? raw.length : 0;
		rows += `<p class="hint">Payload ~${bytes} bytes。3D 图内按步进采样 + 每帧合并更新。彩色图经 <a href="https://github.com/RobotWebTools/web_video_server" target="_blank" rel="noopener">web_video_server</a> MJPEG。默认点云：<code>/camera/depth_registered/points</code> → 「3D: 点云」。</p>`;
		return rows;
	}

	function renderPath(msg) {
		if (!msg || !Array.isArray(msg.poses)) return null;
		const n = msg.poses.length;
		if (n === 0) return '<p class="hint">Empty path</p>';
		const last = msg.poses[n - 1];
		const p = last.pose ? last.pose.position : null;
		return `<p class="hint">poses: ${n}</p>${p ? renderPose(last.pose) : ''}`;
	}

	function renderPoseArray(msg) {
		if (!msg || !Array.isArray(msg.poses)) return null;
		const poses = msg.poses;
		const n = poses.length;
		const fid = (msg.header && msg.header.frame_id) ? msg.header.frame_id : '';
		const maxShow = Math.min(12, n);
		let html = ` · frame <code>${fid}</code><p class="hint">poses: ${n}${fid ? ' · frame <code>' + fid + '</code>' : ''}</p>`;
		html += '<table class="viz-table"><thead><tr><th>#</th><th>x</th><th>y</th><th>z</th><th>qx</th><th>qy</th><th>qz</th><th>qw</th></tr></thead><tbody>';
		let i;
		for (i = 0; i < maxShow; i++) {
			const po = poses[i];
			const pos = po.position || {};
			const ori = po.orientation || {};
			html += `<tr><td>${i}</td><td>${pos.x}</td><td>${pos.y}</td><td>${pos.z}</td><td>${ori.x}</td><td>${ori.y}</td><td>${ori.z}</td><td>${ori.w}</td></tr>`;
		}
		html += '</tbody></table>';
		if (n > maxShow) html += `<p class="hint">… 其余 ${n - maxShow} 条见 JSON</p>`;
		return html;
	}

	function renderMarkerArray(msg) {
		if (!msg) return null;
		const arr = msg.markers || msg;
		if (!Array.isArray(arr)) return null;
		const n = arr.length;
		const maxShow = Math.min(12, n);
		let head = `<p class="hint">markers: ${n}（GraspNet 抓取可视化常用）</p>`;
		head += '<table class="viz-table"><thead><tr><th>#</th><th>id</th><th>type</th><th>ns</th><th>frame</th><th>xyz</th><th>scale</th></tr></thead><tbody>';
		let i;
		for (i = 0; i < maxShow; i++) {
			const m = arr[i];
			const pos = (m.pose && m.pose.position) ? m.pose.position : {};
			const sc = m.scale || {};
			head += `${Number(sc.x || 0).toFixed(2)}×${Number(sc.y || 0).toFixed(2)}×${Number(sc.z || 0).toFixed(2)}${Number(pos.x).toFixed(3)}, ${Number(pos.y).toFixed(3)}, ${Number(pos.z).toFixed(3)}${m.type != null}${m.id != null}<tr><td>${i}</td><td>${m.id != null ? m.id : ''}</td><td>${m.type != null ? m.type : ''}</td><td>${m.ns || ''}</td><td>${(m.header && m.header.frame_id) || ''}</td><td>${Number(pos.x).toFixed(3) + ', ' + Number(pos.y).toFixed(3) + ', ' + Number(pos.z).toFixed(3)}</td><td>${Number(sc.x || 0).toFixed(2) + '×' + Number(sc.y || 0).toFixed(2) + '×' + Number(sc.z || 0).toFixed(2)}</td></tr>`;
		}
		head += '</tbody></table>';
		if (n > maxShow) head += '<p class="hint">… 其余见 JSON</p>';
		head += `<details class="viz-details"><summary class="viz-summary">首条原始 JSON</summary><pre class="raw">${safeJson(arr[0] || {}, 4000)}</pre></details>`;
		return head;
	}

	function renderOdometry(msg) {
		if (!msg || !msg.pose || !msg.twist) return null;
		return `<h4 class="viz-subheading">pose</h4>${renderPose(msg.pose)}<h4 class="viz-subheading viz-subheading--spaced">twist</h4>${renderTwist(msg.twist)}`;
	}

	function renderGenericNumericArray(msg) {
		if (!msg || !msg.layout || !Array.isArray(msg.data)) return null;
		const dim = (msg.layout.dim || []).map(d => `${d.label}=${d.size}`).join(', ');
		const sample = msg.data.slice(0, 32).join(', ');
		return `${msg.data.length > 32}<p class="hint">${dim}</p><pre class="raw">data[0..31]: ${sample}${msg.data.length > 32 ? ' …' : ''}</pre>`;
	}

	function renderVisualization(msgType, msg, canvas) {
		let html = '';
		if (typeMatch(msgType, 'LaserScan')) {
			html = renderLaserScan(msg, canvas);
			return { html, usedCanvas: true };
		}
		if (typeMatch(msgType, 'OccupancyGrid')) {
			html = renderOccupancyGrid(msg, canvas);
			return { html, usedCanvas: true };
		}
		if (typeMatch(msgType, 'Image') && !typeMatch(msgType, 'CompressedImage')) {
			html = renderImage(msg, canvas);
			return { html, usedCanvas: true };
		}
		if (typeMatch(msgType, 'CompressedImage')) {
			return { html: renderCompressedImage(msg), usedCanvas: false };
		}
		if (typeMatch(msgType, 'PointCloud2')) {
			return { html: renderPointCloud2(msg), usedCanvas: false };
		}
		if (typeMatch(msgType, 'Path')) {
			return { html: renderPath(msg), usedCanvas: false };
		}
		if (typeMatch(msgType, 'PoseArray')) {
			const pra = renderPoseArray(msg);
			if (pra) return { html: pra, usedCanvas: false };
		}
		if (typeMatch(msgType, 'MarkerArray')) {
			return { html: renderMarkerArray(msg), usedCanvas: false };
		}
		if (typeMatch(msgType, 'Marker') && !typeMatch(msgType, 'MarkerArray')) {
			return { html: `<pre class="raw">${safeJson(msg, 8000)}</pre>`, usedCanvas: false };
		}
		if (typeMatch(msgType, 'Odometry')) {
			return { html: renderOdometry(msg), usedCanvas: false };
		}
		if (typeMatch(msgType, 'JointState')) {
			return { html: renderJointState(msg), usedCanvas: false };
		}
		if (typeMatch(msgType, 'Imu')) {
			return { html: renderImu(msg), usedCanvas: false };
		}
		if (typeMatch(msgType, 'BatteryState')) {
			return { html: renderBattery(msg), usedCanvas: false };
		}
		if (typeMatch(msgType, 'Range')) {
			const rg = renderRange(msg);
			if (rg) return { html: rg, usedCanvas: false };
		}
		if (typeMatch(msgType, 'NavSatFix')) {
			const ns = renderNavSatFix(msg);
			if (ns) return { html: ns, usedCanvas: false };
		}
		if (typeMatch(msgType, 'Twist')) {
			const tw = renderTwist(msg);
			if (tw) return { html: tw, usedCanvas: false };
		}
		if (typeMatch(msgType, 'Pose')) {
			const ps = renderPose(msg);
			if (ps) return { html: ps, usedCanvas: false };
		}
		if (typeMatch(msgType, 'Float32MultiArray') || typeMatch(msgType, 'Float64MultiArray') ||
			typeMatch(msgType, 'Int32MultiArray') || typeMatch(msgType, 'Int8MultiArray') ||
			typeMatch(msgType, 'UInt8MultiArray') || typeMatch(msgType, 'UInt16MultiArray')) {
			const ga = renderGenericNumericArray(msg);
			if (ga) return { html: ga, usedCanvas: false };
		}
		const sc = renderScalar(msg);
		if (sc) return { html: sc, usedCanvas: false };
		return {
			html: '<p class="hint">无专用视图；右侧为完整 JSON。可在 <code>js/ros_console.js</code> 的 <code>renderVisualization()</code> 中扩展。</p>',
			usedCanvas: false
		};
	}

	function onMessage(msg) {
		if (webVideoStreamMode) return;
		const now = Date.now();
		if (now - lastEmit < throttleMs) return;
		lastEmit = now;
		const type = selectedType;
		const seq = subscriptionSeq;
		if (topicVizRaf != null) {
			cancelAnimationFrame(topicVizRaf);
			topicVizRaf = null;
		}
		topicVizRaf = requestAnimationFrame(() => {
			topicVizRaf = null;
			if (seq !== subscriptionSeq) return;
			const vizBody = $('viz-body');
			const canvas = $('viz-canvas');
			const out = renderVisualization(type, msg, canvas);
			vizBody.innerHTML = out.html;
			canvas.style.display = out.usedCanvas ? 'block' : 'none';
			// 大话题勿 JSON.stringify 全量 data，否则主线程阻塞数秒；摘要即可
			$('raw-pre').textContent = rawPreviewForMessage(type, msg, 120000);
		});
	}

	function unsubscribe() {
		subscriptionSeq++;
		webVideoStreamMode = false;
		if (topicVizRaf != null) {
			cancelAnimationFrame(topicVizRaf);
			topicVizRaf = null;
		}
		if (activeTopic) {
			try {
				activeTopic.dispose();
			} catch (e) { /* ignore */ }
			activeTopic = null;
		}
		if (window.__labBlobUrl) {
			try { URL.revokeObjectURL(window.__labBlobUrl); } catch (e) { /* ignore */ }
			window.__labBlobUrl = null;
		}
	}

	/* ---------- 话题：订阅单话题、节流回调、按消息类型渲染 viz/canvas ---------- */
	function subscribe(name, msgType) {
		if (!ros || !ros.isConnected) return;
		unsubscribe();
		selectedName = name;
		selectedType = msgType;
		lastEmit = 0;
		$('selection-label').textContent = `${name}  ·  ${msgType}`;
		if (typeof ivgWebVideo !== 'undefined' && typeMatch(msgType, 'Image') && !typeMatch(msgType, 'CompressedImage')) {
			webVideoStreamMode = true;
			const wvPort = webVideoPort();
			const url = ivgWebVideo.streamUrl(name, {
				port: wvPort,
				quality: 88
			});
			const viewer = ivgWebVideo.viewerUrl(name, { port: wvPort });
			$('viz-canvas').style.display = 'none';
			const vizBody = $('viz-body');
			vizBody.replaceChildren();
			const hint = document.createElement('p');
			hint.className = 'hint';
			hint.append('MJPEG（web_video_server），sensor_msgs/Image 不经 rosbridge。');
			const a = document.createElement('a');
			a.href = viewer;
			a.target = '_blank';
			a.rel = 'noopener';
			a.textContent = ' stream_viewer';
			hint.appendChild(a);
			vizBody.appendChild(hint);
			const wrap = document.createElement('p');
			wrap.className = 'viz-mjpeg-wrap';
			const img = document.createElement('img');
			img.alt = '';
			img.className = 'viz-mjpeg-img';
			img.decoding = 'async';
			img.src = url;
			wrap.appendChild(img);
			vizBody.appendChild(wrap);
			$('raw-pre').textContent = `[MJPEG] ${name}\n${url}\n\nCompressedImage 仍走 rosbridge。`;
			return;
		}
		webVideoStreamMode = false;
		const mySeq = subscriptionSeq;
		activeTopic = new ROSLIB.Topic({
			ros,
			name,
			messageType: msgType,
			queue_length: 1
		});
		activeTopic.subscribe(msg => {
			if (mySeq !== subscriptionSeq) return;
			onMessage(msg);
		});
		$('viz-body').innerHTML = '<p class="hint">等待消息…</p>';
		$('raw-pre').textContent = '';
	}

	function fillSidebar(items, getMeta) {
		const list = $('sidebar-list');
		const q = ($('filter-q').value || '').toLowerCase();
		list.innerHTML = '';
		for (let i = 0; i < items.length; i++) {
			const name = typeof items[i] === 'string' ? items[i] : items[i].name;
			const meta = getMeta ? getMeta(items[i], i) : '';
			if (q && name.toLowerCase().indexOf(q) === -1 && (!meta || meta.toLowerCase().indexOf(q) === -1)) {
				continue;
			}
			const row = document.createElement('div');
			const rowActive = (tab === 'topics' && name === selectedName) ||
				(tab === 'services' && name === selectedService) ||
				(tab === 'params' && name === selectedParam) ||
				(tab === 'graph' && name === selectedName) ||
				(tab === 'actions' && name === selectedAction);
			row.className = `row${rowActive ? ' active' : ''}`;
			row.innerHTML = '<div class="name"></div><div class="meta"></div>';
			row.querySelector('.name').textContent = name;
			row.querySelector('.meta').textContent = meta;
			(((n, m) => {
				row.onclick = () => {
					if (tab === 'topics') subscribe(n, m);
					else if (tab === 'services') {
						selectedService = n;
						$('svc-selection').textContent = n;
						$('svc-resp').textContent = '';
						$('svc-req-json').value = '{}';
						ros.getServiceType(n, typ => {
							$('svc-type').value = typ;
						}, err => {
							$('svc-type').value = String(err);
						});
					} else if (tab === 'params') {
						selectedParam = n;
						fetchParamValue(n);
					} else if (tab === 'actions') {
						selectedAction = n;
						if ($('actions-dump')) {
							$('actions-dump').textContent = `选中: ${n}\n\n发送 goal / cancel 需按对应 .action 类型构造，可在此页扩展。`;
						}
					} else if (tab === 'graph') {
						selectedName = n;
						if ($('graph-node-detail')) {
							$('graph-node-detail').textContent = `选中节点: ${n}\n点击「重新计算关系图」生成发布/订阅边。`;
						}
					} else if (tab === 'view2d') {
						if (m.indexOf('OccupancyGrid') !== -1) $('map-topic').value = n;
						else if (m.indexOf('LaserScan') !== -1) $('scan-topic').value = n;
					} else if (tab === 'view3d') {
						if (m.indexOf('PointCloud2') !== -1) $('pc-topic').value = n;
						else if (m.indexOf('LaserScan') !== -1) $('scan3-topic').value = n;
					} else {
						selectedName = n;
						$('selection-label').textContent = n + (m ? `  ·  ${m}` : '');
						$('viz-body').innerHTML = '<p class="hint">本标签为列表浏览。</p>';
						$('raw-pre').textContent = m || '';
					}
					Array.prototype.forEach.call(list.children, c => { c.classList.remove('active'); });
					row.classList.add('active');
				};
			}))(name, meta);
			list.appendChild(row);
		}
		if (items.length && !list.children.length && q) {
			const hintRow = document.createElement('p');
			hintRow.className = 'hint';
			hintRow.textContent = `过滤无匹配（共 ${items.length} 项）；请清空上方「过滤」或改关键词。`;
			list.appendChild(hintRow);
		}
		const sc = $('sidebar-count');
		if (sc) {
			const nVis = list.querySelectorAll('.row').length;
			const qv = ($('filter-q').value || '').trim();
			if (!items.length) sc.textContent = '';
			else if (qv) sc.textContent = `显示 ${nVis} / ${items.length}`;
			else sc.textContent = `${items.length} 项`;
		}
	}

	function refreshTopics() {
		if (!ros || !ros.isConnected) return;
		beginSidebarListFetch();
		const rid = ++topicsListReq;
		ros.getTopics(r => {
			if (rid !== topicsListReq) {
				endSidebarListFetch();
				return;
			}
			try {
				if (!r || typeof r !== 'object') r = {};
				const topics = Array.isArray(r.topics) ? r.topics : [];
				const types = Array.isArray(r.types) ? r.types : [];
				const combined = [];
				for (let i = 0; i < topics.length; i++) {
					combined.push({ name: topics[i], type: types[i] || '' });
				}
				combined.sort((a, b) => {
					const ra = topicIvgRank(a.name);
					const rb = topicIvgRank(b.name);
					if (ra !== rb) return ra - rb;
					return a.name.localeCompare(b.name);
				});
				fillSidebar(combined, item => item.type);
			} catch (e) {
				if (rid === topicsListReq) {
					const sl = $('sidebar-list');
					if (sl) sl.innerHTML = `<p class="hint">解析话题列表失败: ${e}</p>`;
					const sc0 = $('sidebar-count');
					if (sc0) sc0.textContent = '';
				}
			} finally {
				endSidebarListFetch();
			}
		}, err => {
			if (rid !== topicsListReq) {
				endSidebarListFetch();
				return;
			}
			$('sidebar-list').innerHTML = `<p class="hint">getTopics 失败: ${err}</p>`;
			const sc1 = $('sidebar-count');
			if (sc1) sc1.textContent = '';
			endSidebarListFetch();
		});
	}

	/* ---------- 服务：侧栏 getServices；主区 ROSLIB.Service.callService ---------- */
	function refreshServices() {
		if (!ros || !ros.isConnected) return;
		beginSidebarListFetch();
		const rid = ++servicesListReq;
		ros.getServices(services => {
			if (rid !== servicesListReq) {
				endSidebarListFetch();
				return;
			}
			try {
				if (!Array.isArray(services)) services = [];
				services.sort();
				fillSidebar(services, () => 'service');
			} catch (e) {
				if (rid === servicesListReq) {
					const sl2 = $('sidebar-list');
					if (sl2) sl2.innerHTML = `<p class="hint">解析服务列表失败: ${e}</p>`;
					const sc2 = $('sidebar-count');
					if (sc2) sc2.textContent = '';
				}
			} finally {
				endSidebarListFetch();
			}
		}, err => {
			if (rid !== servicesListReq) {
				endSidebarListFetch();
				return;
			}
			$('sidebar-list').innerHTML = `<p class="hint">getServices 失败: ${err}</p>`;
			const sc3 = $('sidebar-count');
			if (sc3) sc3.textContent = '';
			endSidebarListFetch();
		});
	}

	function refreshNodes() {
		if (!ros || !ros.isConnected) return;
		beginSidebarListFetch();
		const rid = ++nodesListReq;
		ros.getNodes(nodes => {
			if (rid !== nodesListReq) {
				endSidebarListFetch();
				return;
			}
			if (!Array.isArray(nodes)) nodes = [];
			nodes.sort();
			fillSidebar(nodes, () => 'node');
			endSidebarListFetch();
		}, err => {
			if (rid !== nodesListReq) {
				endSidebarListFetch();
				return;
			}
			$('sidebar-list').innerHTML = `<p class="hint">getNodes 失败: ${err}</p>`;
			const scn = $('sidebar-count');
			if (scn) scn.textContent = '';
			endSidebarListFetch();
		});
	}

	function showMain(mainId) {
		const ids = ['main-topics', 'main-services', 'main-actions', 'main-params', 'main-graph', 'main-view2d', 'main-view3d'];
		for (let i = 0; i < ids.length; i++) {
			const el = $(ids[i]);
			if (el) el.classList.toggle('hidden', ids[i] !== mainId);
		}
	}

	function refreshActionsList() {
		if (!ros || !ros.isConnected) return;
		beginSidebarListFetch();
		const rid = ++actionsListReq;
		ros.getActionServers(list => {
			if (rid !== actionsListReq) {
				endSidebarListFetch();
				return;
			}
			if (!Array.isArray(list)) list = [];
			list.sort();
			if ($('actions-dump')) $('actions-dump').textContent = list.length ? list.join('\n') : '(无动作服务器)';
			fillSidebar(list, () => 'action');
			endSidebarListFetch();
		}, err => {
			if (rid !== actionsListReq) {
				endSidebarListFetch();
				return;
			}
			$('sidebar-list').innerHTML = `<p class="hint">getActionServers 失败: ${err}</p>`;
			const sca = $('sidebar-count');
			if (sca) sca.textContent = '';
			endSidebarListFetch();
		});
	}

	function refreshParamsList() {
		if (!ros || !ros.isConnected) return;
		beginSidebarListFetch();
		const rid = ++paramsListReq;
		ros.getParams(names => {
			if (rid !== paramsListReq) {
				endSidebarListFetch();
				return;
			}
			if (!Array.isArray(names)) names = [];
			names.sort();
			fillSidebar(names, () => 'param');
			endSidebarListFetch();
		}, err => {
			if (rid !== paramsListReq) {
				endSidebarListFetch();
				return;
			}
			$('sidebar-list').innerHTML = `<p class="hint">getParams 失败: ${err}</p>`;
			const scp = $('sidebar-count');
			if (scp) scp.textContent = '';
			endSidebarListFetch();
		});
	}

	function fetchParamValue(paramName) {
		if (!ros || !ros.isConnected) return;
		const pid = ++paramFetchSeq;
		if ($('param-value')) $('param-value').textContent = '读取中…';
		const svc = new ROSLIB.Service({
			ros,
			name: '/rosapi/get_param',
			serviceType: 'rosapi/GetParam'
		});
		const req = new ROSLIB.ServiceRequest({ name: paramName, default_value: '' });
		svc.callService(req, res => {
			if (pid !== paramFetchSeq) return;
			if ($('param-value')) $('param-value').textContent = res.value;
		}, err => {
			if (pid !== paramFetchSeq) return;
			if ($('param-value')) $('param-value').textContent = `错误: ${err}`;
		});
	}

	/**
	 * 切换标签时抬高所有侧栏列表的请求代数。
	 * 仅抬高「非当前数据源」会在话题/2D/3D 共用 topics 列表、或 IVG 连点等组合下漏掉应失效的回调，
	 * 晚到的 getTopics/getServices 仍可能把侧栏刷成上一标签的内容；此处一律作废全部进行中请求。
	 */
	function invalidateStaleSidebarLists() {
		topicsListReq++;
		servicesListReq++;
		nodesListReq++;
		actionsListReq++;
		paramsListReq++;
	}

	/* ---------- 侧栏：按当前 tab 拉取 rosapi 列表并渲染 #sidebar-list ---------- */
	function refreshSidebar() {
		if (!ros || !ros.isConnected) {
			const sl0 = $('sidebar-list');
			if (sl0) sl0.innerHTML = '<p class="hint">未连接 rosbridge，请点击顶栏「重连」。</p>';
			const scd = $('sidebar-count');
			if (scd) scd.textContent = '';
			return;
		}
		if (tab === 'topics') refreshTopics();
		else if (tab === 'services') refreshServices();
		else if (tab === 'actions') refreshActionsList();
		else if (tab === 'params') refreshParamsList();
		else if (tab === 'graph') refreshNodes();
		else if (tab === 'view2d' || tab === 'view3d') refreshTopics();
	}

	function switchTab(t) {
		const prevTab = tab;
		if (tab === 'view2d' && t !== 'view2d') stop2d();
		if (tab === 'view3d' && t !== 'view3d') stop3d();
		if (t !== 'topics') {
			unsubscribe();
			if (prevTab === 'topics') {
				selectedName = '';
				selectedType = '';
			}
			$('selection-label').textContent = '未选择话题';
			$('viz-body').innerHTML = '<p class="hint">切换到「话题」可订阅。</p>';
			$('viz-canvas').style.display = 'none';
			$('raw-pre').textContent = '';
		}
		tab = t;
		if (t === 'topics' && prevTab !== 'topics') {
			selectedName = '';
			selectedType = '';
		}
		if (prevTab === 'services' && t !== 'services') {
			selectedService = '';
		}
		if (prevTab === 'params' && t !== 'params') {
			selectedParam = '';
		}
		if (prevTab === 'graph' && t !== 'graph') {
			selectedName = '';
			graphBuildSeq++;
		}
		if (prevTab === 'actions' && t !== 'actions') {
			selectedAction = '';
		}
		const buttons = document.querySelectorAll('.tabs button');
		for (let i = 0; i < buttons.length; i++) {
			const isSel = buttons[i].dataset.tab === t;
			buttons[i].classList.toggle('active', isSel);
			buttons[i].setAttribute('aria-selected', isSel ? 'true' : 'false');
		}
		setSidebarContextLabel(t);
		if (t === 'topics') showMain('main-topics');
		else if (t === 'services') showMain('main-services');
		else if (t === 'actions') showMain('main-actions');
		else if (t === 'params') showMain('main-params');
		else if (t === 'graph') showMain('main-graph');
		else if (t === 'view2d') showMain('main-view2d');
		else if (t === 'view3d') showMain('main-view3d');
		if (prevTab !== t) {
			invalidateStaleSidebarLists();
		}
		refreshSidebar();
		if (t === 'graph' && graphModel) {
			requestAnimationFrame(() => {
				redrawGraphCanvas();
			});
		}
	}

	/** 与 rosgraph 一致：前导 /、合并重复斜杠，避免同一话题因字符串写法不同而无法配对 */
	function normalizeRosGraphName(s) {
		if (s == null || s === '') return '';
		if (typeof s !== 'string') s = String(s);
		s = s.trim();
		if (!s) return '';
		if (s.charAt(0) !== '/') s = `/${s}`;
		return s.replace(/\/+/g, '/');
	}

	function pushUniqueName(arr, name) {
		if (!name || arr.indexOf(name) !== -1) return;
		arr.push(name);
	}

	/** 兼容 roslib 单参整包与三数组回调 */
	function coerceNodeDetails(sub, pub, srv) {
		let s0 = sub;
		let p0 = pub;
		let v0 = srv;
		if (s0 && typeof s0 === 'object' && !Array.isArray(s0) && p0 === undefined) {
			s0 = s0.subscribing;
			p0 = s0.publishing;
			v0 = s0.services;
		}
		return {
			sub: Array.isArray(s0) ? s0 : [],
			pub: Array.isArray(p0) ? p0 : [],
			srv: Array.isArray(v0) ? v0 : []
		};
	}

	/* ---------- 节点关系图：getNodes + 各节点 pub/sub/service 详情，Canvas 绘制 ---------- */
	function buildGraph() {
		if (!ros || !ros.isConnected) return;
		const bid = ++graphBuildSeq;
		const prog = $('graph-progress');
		if (prog) prog.textContent = '读取节点…';
		ros.getNodes(nodesIn => {
			if (bid !== graphBuildSeq) return;
			const nodes = Array.isArray(nodesIn) ? nodesIn.slice() : [];
			nodes.sort();
			let i = 0;
			const details = {};
			function step() {
				if (bid !== graphBuildSeq) return;
				if (i >= nodes.length) {
					finishGraph(bid, nodes, details);
					return;
				}
				const nm = nodes[i++];
				ros.getNodeDetails(nm, (sub, pub, srv) => {
					if (bid !== graphBuildSeq) return;
					const c = coerceNodeDetails(sub, pub, srv);
					details[nm] = { sub: c.sub, pub: c.pub, srv: c.srv };
					if (prog) prog.textContent = `node_details ${i}/${nodes.length}`;
					step();
				}, () => {
					if (bid !== graphBuildSeq) return;
					details[nm] = { sub: [], pub: [], srv: [] };
					step();
				});
			}
			step();
		}, err => {
			if (bid !== graphBuildSeq) return;
			if (prog) prog.textContent = `getNodes 失败: ${err}`;
		});
	}

	function finishGraph(bid, nodes, details) {
		if (bid !== graphBuildSeq) return;
		const topicEnds = {};
		let ei;
		for (ei = 0; ei < nodes.length; ei++) {
			const n = nodes[ei];
			const d = details[n];
			if (!d) continue;
			let p;
			for (p = 0; p < (d.pub || []).length; p++) {
				const tp = normalizeRosGraphName(d.pub[p]);
				if (!tp) continue;
				if (!topicEnds[tp]) topicEnds[tp] = { pub: [], sub: [] };
				pushUniqueName(topicEnds[tp].pub, n);
			}
			for (p = 0; p < (d.sub || []).length; p++) {
				const ts = normalizeRosGraphName(d.sub[p]);
				if (!ts) continue;
				if (!topicEnds[ts]) topicEnds[ts] = { pub: [], sub: [] };
				pushUniqueName(topicEnds[ts].sub, n);
			}
		}
		const edges = [];
		const edgeSeen = {};
		const tk = Object.keys(topicEnds);
		for (ei = 0; ei < tk.length; ei++) {
			const T = tk[ei];
			const tm = topicEnds[T];
			let a;
			for (a = 0; a < tm.pub.length; a++) {
				let b;
				for (b = 0; b < tm.sub.length; b++) {
					if (tm.pub[a] !== tm.sub[b]) {
						const ek = `${tm.pub[a]}\n${tm.sub[b]}\n${T}`;
						if (edgeSeen[ek]) continue;
						edgeSeen[ek] = true;
						edges.push({ from: tm.pub[a], to: tm.sub[b], topic: T });
					}
				}
			}
		}
		const nodeSet = {};
		for (ei = 0; ei < nodes.length; ei++) nodeSet[nodes[ei]] = true;
		for (ei = 0; ei < edges.length; ei++) {
			nodeSet[edges[ei].from] = true;
			nodeSet[edges[ei].to] = true;
		}
		const allNodes = Object.keys(nodeSet).sort();
		graphModel = { nodes: allNodes, edges, topicEnds };
		redrawGraphCanvas();
	}

	function graphRosNamespace(fullName) {
		const n = normalizeRosGraphName(fullName);
		if (!n || n === '/') return '/';
		const idx = n.lastIndexOf('/');
		if (idx <= 0) return '/';
		return n.slice(0, idx);
	}

	function graphRosShortName(fullName) {
		const n = normalizeRosGraphName(fullName);
		const idx = n.lastIndexOf('/');
		if (idx < 0) return n;
		const s = n.slice(idx + 1);
		return s || n;
	}

	function getGraphViewMode() {
		const r = document.querySelector('input[name="graph-view"]:checked');
		return r && r.value === 'architecture' ? 'architecture' : 'logic';
	}

	function setupGraphCanvas2d() {
		const canvas = $('graph-canvas');
		if (!canvas) return null;
		const ctx = canvas.getContext('2d');
		let cssW = canvas.clientWidth;
		let cssH = canvas.clientHeight;
		if (!cssW || !cssH) {
			cssW = canvas.width || 900;
			cssH = canvas.height || 520;
		}
		const dpr = Math.min(window.devicePixelRatio || 1, 2.5);
		canvas.width = Math.max(1, Math.floor(cssW * dpr));
		canvas.height = Math.max(1, Math.floor(cssH * dpr));
		canvas.style.width = `${cssW}px`;
		canvas.style.height = `${cssH}px`;
		ctx.setTransform(1, 0, 0, 1, 0, 0);
		ctx.scale(dpr, dpr);
		return { canvas, ctx, W: cssW, H: cssH };
	}

	function pathRoundRect(ctx, x, y, w, h, rad) {
		const r = Math.min(rad, w / 2, h / 2);
		ctx.beginPath();
		ctx.moveTo(x + r, y);
		ctx.lineTo(x + w - r, y);
		ctx.quadraticCurveTo(x + w, y, x + w, y + r);
		ctx.lineTo(x + w, y + h - r);
		ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
		ctx.lineTo(x + r, y + h);
		ctx.quadraticCurveTo(x, y + h, x, y + h - r);
		ctx.lineTo(x, y + r);
		ctx.quadraticCurveTo(x, y, x + r, y);
		ctx.closePath();
	}

	function drawGraphLogic(ctx, W, H, nodes, edges) {
		const pos = {};
		const N = nodes.length;
		const R = Math.min(W, H) * 0.32;
		let k;
		for (k = 0; k < N; k++) {
			const ang = 2 * Math.PI * k / Math.max(N, 1);
			pos[nodes[k]] = { x: W / 2 + R * Math.cos(ang), y: H / 2 + R * Math.sin(ang), vx: 0, vy: 0 };
		}
		let iter;
		for (iter = 0; iter < 100; iter++) {
			var i;
			for (i = 0; i < N; i++) {
				for (let j = i + 1; j < N; j++) {
					const na = nodes[i];
					const nb = nodes[j];
					const dx = pos[nb].x - pos[na].x;
					const dy = pos[nb].y - pos[na].y;
					const dist = Math.sqrt(dx * dx + dy * dy) + 0.01;
					const f = 800 / (dist * dist);
					pos[na].vx -= f * dx / dist;
					pos[na].vy -= f * dy / dist;
					pos[nb].vx += f * dx / dist;
					pos[nb].vy += f * dy / dist;
				}
			}
			for (i = 0; i < edges.length; i++) {
				const e = edges[i];
				const pa = pos[e.from];
				const pb = pos[e.to];
				if (!pa || !pb) continue;
				const ex = pb.x - pa.x;
				const ey = pb.y - pa.y;
				const d = Math.sqrt(ex * ex + ey * ey) + 0.01;
				const att = 0.03 * d;
				pa.vx += att * ex / d;
				pa.vy += att * ey / d;
				pb.vx -= att * ex / d;
				pb.vy -= att * ey / d;
			}
			for (i = 0; i < N; i++) {
				const nm = nodes[i];
				pos[nm].x += pos[nm].vx;
				pos[nm].y += pos[nm].vy;
				pos[nm].vx *= 0.88;
				pos[nm].vy *= 0.88;
				pos[nm].x = Math.max(40, Math.min(W - 40, pos[nm].x));
				pos[nm].y = Math.max(40, Math.min(H - 40, pos[nm].y));
			}
		}
		ctx.fillStyle = GRAPH_BG;
		ctx.fillRect(0, 0, W, H);
		ctx.strokeStyle = GRAPH_EDGE;
		ctx.lineWidth = 1;
		for (i = 0; i < edges.length; i++) {
			const ed = edges[i];
			if (!pos[ed.from] || !pos[ed.to]) continue;
			ctx.beginPath();
			ctx.moveTo(pos[ed.from].x, pos[ed.from].y);
			ctx.lineTo(pos[ed.to].x, pos[ed.to].y);
			ctx.stroke();
		}
		ctx.fillStyle = GRAPH_NODE;
		for (i = 0; i < N; i++) {
			const nn = nodes[i];
			ctx.beginPath();
			ctx.arc(pos[nn].x, pos[nn].y, 5, 0, Math.PI * 2);
			ctx.fill();
		}
		ctx.fillStyle = GRAPH_LABEL;
		ctx.font = '9px monospace';
		for (i = 0; i < N; i++) {
			let label = nodes[i].replace(/^\//, '');
			if (label.length > 22) label = `${label.slice(0, 20)}…`;
			ctx.fillText(label, pos[nodes[i]].x + 8, pos[nodes[i]].y - 6);
		}
		const gp = $('graph-progress');
		if (gp) {
			gp.textContent = `逻辑图 · 节点 ${N} · 边 ${edges.length}（经话题连接）`;
		}
	}

	function drawGraphArchitecture(ctx, W, H, nodes, edges) {
		const groupMap = {};
		let gi;
		for (gi = 0; gi < nodes.length; gi++) {
			const nm = nodes[gi];
			const ns = graphRosNamespace(nm);
			if (!groupMap[ns]) groupMap[ns] = [];
			pushUniqueName(groupMap[ns], nm);
		}
		const nsKeys = Object.keys(groupMap).sort();
		for (gi = 0; gi < nsKeys.length; gi++) {
			groupMap[nsKeys[gi]].sort();
		}
		const margin = 14;
		const gapX = 18;
		const gapY = 16;
		const pad = 10;
		const titleH = 20;
		const lineH = 14;
		const maxListLines = 18;
		const minBoxW = 108;
		const maxBoxW = 260;
		ctx.font = '10px monospace';
		const boxes = [];
		let x = margin;
		let y = margin;
		let rowH = 0;
		for (gi = 0; gi < nsKeys.length; gi++) {
			const nsK = nsKeys[gi];
			const members = groupMap[nsK];
			const titleText = nsK === '/' ? '/（根）' : nsK;
			const tw = ctx.measureText(titleText).width + pad * 2;
			let mw = tw;
			const listCount = Math.min(members.length, maxListLines);
			let li;
			for (li = 0; li < listCount; li++) {
				let sn = graphRosShortName(members[li]);
				if (sn.length > 32) sn = `${sn.slice(0, 30)}…`;
				mw = Math.max(mw, ctx.measureText(sn).width + pad * 2);
			}
			const bw = Math.max(minBoxW, Math.min(maxBoxW, mw));
			const extraLines = members.length > maxListLines ? 1 : 0;
			const bh = titleH + pad + listCount * lineH + extraLines * lineH + pad;
			if (x + bw > W - margin && x > margin) {
				x = margin;
				y += rowH + gapY;
				rowH = 0;
			}
			boxes.push({
				ns: nsK,
				x,
				y,
				w: bw,
				h: bh,
				members,
				cx: x + bw / 2,
				cy: y + bh / 2
			});
			rowH = Math.max(rowH, bh);
			x += bw + gapX;
		}
		const boxByNs = {};
		for (gi = 0; gi < boxes.length; gi++) {
			boxByNs[boxes[gi].ns] = boxes[gi];
		}
		const interSeen = {};
		const interList = [];
		for (gi = 0; gi < edges.length; gi++) {
			const eg = edges[gi];
			const nf = graphRosNamespace(eg.from);
			const nt = graphRosNamespace(eg.to);
			if (nf === nt) continue;
			const ik = `${nf}\n${nt}`;
			if (interSeen[ik]) continue;
			interSeen[ik] = true;
			interList.push({ from: nf, to: nt });
		}
		ctx.fillStyle = GRAPH_BG;
		ctx.fillRect(0, 0, W, H);
		ctx.strokeStyle = GRAPH_ARCH_EDGE;
		ctx.lineWidth = 1.25;
		for (gi = 0; gi < interList.length; gi++) {
			const ie = interList[gi];
			const bf = boxByNs[ie.from];
			const bt = boxByNs[ie.to];
			if (!bf || !bt) continue;
			const dx = bt.cx - bf.cx;
			const dy = bt.cy - bf.cy;
			const len = Math.sqrt(dx * dx + dy * dy) + 0.01;
			const ox = (-dy / len) * 24;
			const oy = (dx / len) * 24;
			const mx = (bf.cx + bt.cx) / 2 + ox;
			const my = (bf.cy + bt.cy) / 2 + oy;
			ctx.beginPath();
			ctx.moveTo(bf.cx, bf.cy);
			ctx.quadraticCurveTo(mx, my, bt.cx, bt.cy);
			ctx.stroke();
		}
		for (gi = 0; gi < boxes.length; gi++) {
			const b = boxes[gi];
			ctx.fillStyle = GRAPH_ARCH_BOX;
			pathRoundRect(ctx, b.x, b.y, b.w, b.h, 8);
			ctx.fill();
			ctx.strokeStyle = GRAPH_ARCH_STROKE;
			ctx.lineWidth = 1;
			ctx.stroke();
			ctx.fillStyle = GRAPH_ARCH_TITLE;
			ctx.font = 'bold 10px monospace';
			let tShow = b.ns === '/' ? '/（根命名空间）' : b.ns;
			if (tShow.length > 36) tShow = `${tShow.slice(0, 34)}…`;
			ctx.fillText(tShow, b.x + pad, b.y + pad + 10);
			ctx.fillStyle = GRAPH_ARCH_MEMBER;
			ctx.font = '9px monospace';
			const baseY = b.y + titleH + pad;
			let mi;
			const showN = Math.min(b.members.length, maxListLines);
			for (mi = 0; mi < showN; mi++) {
				let line = graphRosShortName(b.members[mi]);
				if (line.length > 34) line = `${line.slice(0, 32)}…`;
				ctx.fillText(`· ${line}`, b.x + pad, baseY + mi * lineH);
			}
			if (b.members.length > maxListLines) {
				ctx.fillStyle = '#64748b';
				ctx.fillText(`… 共 ${b.members.length} 个节点`, b.x + pad, baseY + showN * lineH);
			}
		}
		const gp2 = $('graph-progress');
		if (gp2) {
			gp2.textContent = `架构图 · 命名空间 ${boxes.length} · 组间连接 ${interList.length} · 节点 ${nodes.length} · 边(逻辑) ${edges.length}`;
		}
	}

	function redrawGraphCanvas() {
		const pack = setupGraphCanvas2d();
		if (!pack) return;
		if (!graphModel) {
			pack.ctx.fillStyle = GRAPH_BG;
			pack.ctx.fillRect(0, 0, pack.W, pack.H);
			pack.ctx.fillStyle = GRAPH_ARCH_MEMBER;
			pack.ctx.font = '12px monospace';
			pack.ctx.fillText('请点击「重新计算关系图」生成数据。', 24, 36);
			return;
		}
		const mode = getGraphViewMode();
		if (mode === 'architecture') {
			drawGraphArchitecture(pack.ctx, pack.W, pack.H, graphModel.nodes, graphModel.edges);
		} else {
			drawGraphLogic(pack.ctx, pack.W, pack.H, graphModel.nodes, graphModel.edges);
		}
	}

	function stop2d() {
		if (mapTopicSub) {
			try { mapTopicSub.dispose(); } catch (e1) { /* ignore */ }
			mapTopicSub = null;
		}
		if (scanTopicSub) {
			try { scanTopicSub.dispose(); } catch (e2) { /* ignore */ }
			scanTopicSub = null;
		}
		const h = $('view2d-host');
		if (h) h.innerHTML = '';
		viewer2d = null;
		gridRoot = null;
		scanShape = null;
	}

	/* ---------- 2D（ROS2D）与 3D（Three.js）视图：地图、雷达、点云 ---------- */
	function start2d() {
		stop2d();
		if (typeof ROS2D === 'undefined' || typeof createjs === 'undefined') {
			alert('ROS2D / EaselJS 未加载');
			return;
		}
		const host = $('view2d-host');
		const inner = document.createElement('div');
		inner.id = 'ros2d-inner';
		host.appendChild(inner);
		viewer2d = new ROS2D.Viewer({ divID: 'ros2d-inner', width: 800, height: 500, background: ROS2D_VIEWER_BG });
		gridRoot = new createjs.Container();
		viewer2d.addObject(gridRoot);
		gridRoot.addChild(new ROS2D.Grid({ size: 1 }));
		let currentGrid = null;
		const mapTopic = $('map-topic').value || '/map';
		mapTopicSub = new ROSLIB.Topic({
			ros,
			name: mapTopic,
			messageType: 'nav_msgs/msg/OccupancyGrid',
			queue_length: 1
		});
		mapTopicSub.subscribe(message => {
			if (currentGrid) gridRoot.removeChild(currentGrid);
			currentGrid = new ROS2D.OccupancyGrid({ message });
			gridRoot.addChildAt(currentGrid, 0);
			const info = message.info;
			viewer2d.scaleToDimensions(info.width * info.resolution, info.height * info.resolution);
			viewer2d.shift(-info.origin.position.x, -info.origin.position.y);
		});
		const st = ($('scan-topic').value || '').trim();
		if (st) {
			scanShape = new createjs.Shape();
			gridRoot.addChild(scanShape);
			scanTopicSub = new ROSLIB.Topic({
				ros,
				name: st,
				messageType: 'sensor_msgs/msg/LaserScan',
				queue_length: 1
			});
			scanTopicSub.subscribe(msg => {
				if (!msg || !Array.isArray(msg.ranges)) return;
				const g = scanShape.graphics;
				g.clear();
				g.setStrokeStyle(1);
				g.beginStroke('rgba(255,80,80,0.9)');
				const cx = 0;
				const cy = 0;
				let k;
				for (k = 0; k < msg.ranges.length; k++) {
					const r = msg.ranges[k];
					if (!isFinite(r) || r <= 0) continue;
					const ang = msg.angle_min + k * msg.angle_increment;
					const px = cx + r * Math.cos(ang);
					const py = cy - r * Math.sin(ang);
					if (k === 0) g.moveTo(px, py);
					else g.lineTo(px, py);
				}
				g.endStroke();
			});
		}
	}

	function stop3d() {
		if (pcUpdateRaf != null) {
			cancelAnimationFrame(pcUpdateRaf);
			pcUpdateRaf = null;
		}
		pcPendingMsg = null;
		if (scan3UpdateRaf != null) {
			cancelAnimationFrame(scan3UpdateRaf);
			scan3UpdateRaf = null;
		}
		scan3PendingMsg = null;
		if (anim3d) {
			cancelAnimationFrame(anim3d);
			anim3d = null;
		}
		if (pcTopicSub) {
			try { pcTopicSub.dispose(); } catch (e3) { /* ignore */ }
			pcTopicSub = null;
		}
		if (scan3TopicSub) {
			try { scan3TopicSub.dispose(); } catch (e4) { /* ignore */ }
			scan3TopicSub = null;
		}
		const host = $('view3d-host');
		if (host) host.innerHTML = '';
		threeRenderer = null;
		threeScene = null;
		threeCamera = null;
		threeControls = null;
		pointsObj = null;
	}

	function fieldOffset(fields, name) {
		let f;
		for (f = 0; f < fields.length; f++) {
			if (fields[f].name === name) return fields[f].offset;
		}
		return -1;
	}

	function updatePointsFromPointCloud2(msg, maxPts) {
		if (!pointsObj || !msg.fields) return;
		const ox = fieldOffset(msg.fields, 'x');
		const oy = fieldOffset(msg.fields, 'y');
		const oz = fieldOffset(msg.fields, 'z');
		if (ox < 0 || oy < 0 || oz < 0) return;
		const buf = toUint8(msg.data);
		if (!buf || !buf.buffer) return;
		const le = !msg.is_bigendian;
		const step = msg.point_step;
		const total = msg.width * msg.height;
		if (total <= 0 || step <= 0) return;
		const stride = Math.max(1, Math.ceil(total / maxPts));
		const arr = pointsObj.geometry.attributes.position.array;
		const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
		let count = 0;
		let idx = 0;
		for (; idx < total && count < maxPts; idx += stride) {
			const base = idx * step;
			if (base + step > buf.byteLength) break;
			arr[count * 3] = dv.getFloat32(base + ox, le);
			arr[count * 3 + 1] = dv.getFloat32(base + oy, le);
			arr[count * 3 + 2] = dv.getFloat32(base + oz, le);
			count++;
		}
		pointsObj.geometry.setDrawRange(0, count);
		pointsObj.geometry.attributes.position.needsUpdate = true;
	}

	function updatePointsFromLaserScan3(msg, maxPts) {
		if (!pointsObj || !msg || !Array.isArray(msg.ranges)) return;
		const arr = pointsObj.geometry.attributes.position.array;
		let count = 0;
		let i;
		for (i = 0; i < msg.ranges.length && count < maxPts; i++) {
			const r = msg.ranges[i];
			if (!isFinite(r) || r < msg.range_min || r > msg.range_max) continue;
			const ang = msg.angle_min + i * msg.angle_increment;
			arr[count * 3] = r * Math.cos(ang);
			arr[count * 3 + 1] = r * Math.sin(ang);
			arr[count * 3 + 2] = 0;
			count++;
		}
		pointsObj.geometry.setDrawRange(0, count);
		pointsObj.geometry.attributes.position.needsUpdate = true;
	}

	function start3d() {
		stop3d();
		if (typeof THREE === 'undefined') {
			alert('Three.js 未加载');
			return;
		}
		const host = $('view3d-host');
		const w = host.clientWidth || 800;
		const h = 420;
		threeRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
		threeRenderer.setPixelRatio(window.devicePixelRatio || 1);
		threeRenderer.setSize(w, h);
		threeRenderer.setClearColor(0x111111, 1);
		host.appendChild(threeRenderer.domElement);
		threeScene = new THREE.Scene();
		threeCamera = new THREE.PerspectiveCamera(50, w / h, 0.05, 200);
		threeCamera.position.set(4, 4, 3);
		threeControls = new THREE.OrbitControls(threeCamera, threeRenderer.domElement);
		threeScene.add(new THREE.GridHelper(8, 16, 0x64748b, 0x334155));
		threeScene.add(new THREE.AmbientLight(0x606060));
		const dl = new THREE.DirectionalLight(0xffffff, 0.85);
		dl.position.set(3, 6, 4);
		threeScene.add(dl);
		const maxPts = Math.max(1000, parseInt($('pc-max').value, 10) || 20000);
		const geom = new THREE.BufferGeometry();
		const posArr = new Float32Array(maxPts * 3);
		geom.setAttribute('position', new THREE.BufferAttribute(posArr, 3));
		geom.setDrawRange(0, 0);
		const mat = new THREE.PointsMaterial({ size: 0.04, color: 0x38bdf8 });
		pointsObj = new THREE.Points(geom, mat);
		threeScene.add(pointsObj);
		function loop() {
			anim3d = requestAnimationFrame(loop);
			threeControls.update();
			threeRenderer.render(threeScene, threeCamera);
		}
		loop();
		const pcn = ($('pc-topic').value || '').trim();
		const sn = ($('scan3-topic').value || '').trim();
		if (pcn) {
			pcTopicSub = new ROSLIB.Topic({
				ros,
				name: pcn,
				messageType: 'sensor_msgs/msg/PointCloud2',
				queue_length: 1
			});
			pcTopicSub.subscribe(m => {
				pcPendingMsg = m;
				if (pcUpdateRaf != null) return;
				pcUpdateRaf = requestAnimationFrame(() => {
					pcUpdateRaf = null;
					const mm = pcPendingMsg;
					pcPendingMsg = null;
					if (mm) updatePointsFromPointCloud2(mm, maxPts);
				});
			});
		} else if (sn) {
			scan3TopicSub = new ROSLIB.Topic({
				ros,
				name: sn,
				messageType: 'sensor_msgs/msg/LaserScan',
				queue_length: 1
			});
			scan3TopicSub.subscribe(m => {
				scan3PendingMsg = m;
				if (scan3UpdateRaf != null) return;
				scan3UpdateRaf = requestAnimationFrame(() => {
					scan3UpdateRaf = null;
					const mm = scan3PendingMsg;
					scan3PendingMsg = null;
					if (mm) updatePointsFromLaserScan3(mm, maxPts);
				});
			});
		}
	}

	/** 建立或重建 ROSLIB.Ros：直连 rosbridge WebSocket。 */
	function connect() {
		if (ros) {
			try { ros.close(); } catch (e) { /* ignore */ }
		}
		ros = new ROSLIB.Ros({ groovyCompatibility: false });
		const h = window.location.hostname || '127.0.0.1';
		ros.on('connection', () => {
			syncLocalHostLinks();
			setStatus(`已连接 ws://${h}:${bridgePort()}`, true);
			refreshSidebar();
		});
		ros.on('error', () => {
			setStatus('连接错误', false);
		});
		ros.on('close', () => {
			setStatus('已断开', false);
			refreshSidebar();
		});
		ros.connect(`ws://${h}:${bridgePort()}`);
	}

	document.addEventListener('DOMContentLoaded', () => {
		(function initWebVideoFromQuery() {
			const p = new URLSearchParams(window.location.search);
			const wp = p.get('web_video_port');
			if (wp && $('web-video-port')) $('web-video-port').value = wp;
		})();
		syncLocalHostLinks();
		function scheduleRefreshSidebar() {
			clearTimeout(filterDebounceTimer);
			filterDebounceTimer = setTimeout(() => {
				filterDebounceTimer = null;
				refreshSidebar();
			}, 280);
		}
		$('btn-refresh').onclick = () => {
			clearTimeout(filterDebounceTimer);
			filterDebounceTimer = null;
			/* 与切标签一致：作废所有进行中的列表请求，避免「刷新」仍被旧回调盖掉 */
			invalidateStaleSidebarLists();
			refreshSidebar();
		};
		$('filter-q').oninput = scheduleRefreshSidebar;
		document.addEventListener('keydown', ev => {
			if (ev.key !== 'Escape') return;
			const fq = $('filter-q');
			if (!fq || document.activeElement !== fq) return;
			ev.preventDefault();
			fq.value = '';
			clearTimeout(filterDebounceTimer);
			filterDebounceTimer = null;
			refreshSidebar();
		});
		function applyThrottleMs() {
			throttleMs = Math.max(0, parseInt($('throttle-ms').value, 10) || 0);
		}
		$('throttle-ms').addEventListener('change', applyThrottleMs);
		$('throttle-ms').addEventListener('input', applyThrottleMs);
		const br = $('btn-reconnect');
		if (br) br.onclick = () => { connect(); };
		const wvportEl = $('web-video-port');
		if (wvportEl) wvportEl.addEventListener('change', reconnectImageTopicIfNeeded);
		const bu = $('btn-unsub-topic');
		if (bu) bu.onclick = stopTopicSubscription;
		const bc = $('btn-copy-raw');
		if (bc) bc.onclick = () => { copyToClipboard($('raw-pre').textContent, bc); };
		const bcs = $('btn-copy-svc-resp');
		if (bcs) bcs.onclick = () => { copyToClipboard($('svc-resp').textContent, bcs); };
		const bcp = $('btn-copy-param');
		if (bcp) bcp.onclick = () => { copyToClipboard($('param-value').textContent, bcp); };
		document.querySelectorAll('.tabs button').forEach(b => {
			b.onclick = () => { switchTab(b.dataset.tab); };
		});
		const bbg = $('btn-build-graph');
		if (bbg) bbg.onclick = buildGraph;
		document.querySelectorAll('input[name="graph-view"]').forEach(radio => {
			radio.addEventListener('change', () => {
				if (graphModel) redrawGraphCanvas();
			});
		});
		const b2s = $('btn-2d-start');
		if (b2s) b2s.onclick = start2d;
		const b2t = $('btn-2d-stop');
		if (b2t) b2t.onclick = stop2d;
		const b3s = $('btn-3d-start');
		if (b3s) b3s.onclick = start3d;
		const b3t = $('btn-3d-stop');
		if (b3t) b3t.onclick = stop3d;
		const svcBtn = $('svc-call');
		if (svcBtn) {
			svcBtn.onclick = () => {
				if (!ros || !ros.isConnected) {
					alert('未连接 rosbridge，请先重连');
					return;
				}
				if (!selectedService) {
					alert('请先在左侧选择服务');
					return;
				}
				const typ = $('svc-type').value;
				if (!typ) {
					alert('未获取到 service type');
					return;
				}
				const txt = ($('svc-req-json').value || '').trim() || '{}';
				let reqObj;
				try {
					reqObj = JSON.parse(txt);
				} catch (e) {
					alert('请求 JSON 无效');
					return;
				}
				const origLabel = svcBtn.textContent;
				svcBtn.disabled = true;
				svcBtn.textContent = '调用中…';
				$('svc-resp').textContent = '调用中…';
				const srv = new ROSLIB.Service({
					ros,
					name: selectedService,
					serviceType: typ
				});
				srv.callService(new ROSLIB.ServiceRequest(reqObj), r => {
					svcBtn.disabled = false;
					svcBtn.textContent = origLabel;
					$('svc-resp').textContent = safeJson(r, 80000);
				}, e => {
					svcBtn.disabled = false;
					svcBtn.textContent = origLabel;
					$('svc-resp').textContent = `错误: ${e}`;
				});
			};
		}
		bindIvgBar();
		connect();
	});
})();
