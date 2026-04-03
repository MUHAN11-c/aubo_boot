/* global ROSLIB, ROS2D, createjs, THREE */
(function () {
	'use strict';

	var ros = null;
	var activeTopic = null;
	/** 每次取消订阅 +1，用于丢弃旧 Topic 在 dispose 后仍可能到达的回调（避免 raw JSON / 视图刷成上一话题） */
	var subscriptionSeq = 0;
	/** 侧边栏异步列表请求代数，避免快速输入过滤时旧 getTopics 覆盖新结果 */
	var topicsListReq = 0;
	var servicesListReq = 0;
	var nodesListReq = 0;
	var actionsListReq = 0;
	var paramsListReq = 0;
	var paramFetchSeq = 0;
	/** 节点关系图串行构建代数；切换离开「图」标签或再次点击计算时递增，丢弃过期回调 */
	var graphBuildSeq = 0;
	/** 最近一次计算结果，用于在「逻辑图 / 架构图」间切换不重查 rosapi */
	var graphModel = null;
	/** 侧栏列表并发 rosapi 请求数，用于统一「加载中」与刷新按钮禁用（参考常见控制台 UX） */
	var sidebarListInflight = 0;
	var filterDebounceTimer = null;
	var throttleMs = 80;
	var lastEmit = 0;
	var selectedName = '';
	var selectedType = '';
	var tab = 'topics';
	var selectedService = '';
	var selectedParam = '';
	var selectedAction = '';
	var mapTopicSub = null;
	var scanTopicSub = null;
	var viewer2d = null;
	var gridRoot = null;
	var scanShape = null;
	var pcTopicSub = null;
	var scan3TopicSub = null;
	var threeRenderer = null;
	var threeScene = null;
	var threeCamera = null;
	var threeControls = null;
	var pointsObj = null;
	var anim3d = null;

	/** 亮色主题画布色（与 topics_lab.css 一致） */
	var VIZ_CANVAS_BG = '#e8ecf2';
	var LASER_STROKE = '#15803d';
	var LASER_ARC_STROKE = '#94a3b8';
	var GRAPH_BG = '#f1f5f9';
	var GRAPH_EDGE = 'rgba(37, 99, 235, 0.38)';
	var GRAPH_NODE = '#16a34a';
	var GRAPH_LABEL = '#334155';
	var GRAPH_ARCH_BOX = 'rgba(255, 255, 255, 0.94)';
	var GRAPH_ARCH_STROKE = '#94a3b8';
	var GRAPH_ARCH_TITLE = '#0f172a';
	var GRAPH_ARCH_MEMBER = '#475569';
	var GRAPH_ARCH_EDGE = 'rgba(37, 99, 235, 0.28)';
	var ROS2D_VIEWER_BG = '#eef1f6';

	/** 与 start_IVG_graspnet_points_fastapi.sh + graspnet_demo_points_with_tf 默认一致 */
	var IVG_TOPIC_ORDER = [
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
		var idx = IVG_TOPIC_ORDER.indexOf(topicName);
		if (idx !== -1) return idx;
		if (topicName.indexOf('/camera/') === 0) return 20;
		if (topicName.indexOf('grasp') !== -1) return 25;
		return 500;
	}

	function ivgFastapiBase() {
		var el = $('ivg-fastapi-port');
		var p = (el && el.value ? el.value : '8088').replace(/[^\d]/g, '') || '8088';
		var h = window.location.hostname || '127.0.0.1';
		return 'http://' + h + ':' + p;
	}

	function ivgSubscribeFixed(name, fallbackType) {
		if (!ros || !ros.isConnected) return;
		switchTab('topics');
		ros.getTopicType(name, function (typ) {
			subscribe(name, typ);
			refreshTopics();
		}, function () {
			if (fallbackType) subscribe(name, fallbackType);
			else $('viz-body').innerHTML = '<p class="hint">未找到话题 <code>' + name + '</code>，请确认栈已启动。</p>';
			refreshTopics();
		});
	}

	function ivgTrySubscribeMarkers() {
		if (!ros || !ros.isConnected) return;
		switchTab('topics');
		ros.getTopicType('/grasp_markers', function (t) {
			subscribe('/grasp_markers', t);
			refreshTopics();
		}, function () {
			ros.getTopicType('grasp_markers', function (t2) {
				subscribe('grasp_markers', t2);
				refreshTopics();
			}, function () {
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
		var bar = $('ivg-bar');
		if (!bar) return;
		var link = $('ivg-link-fastapi');
		var portInput = $('ivg-fastapi-port');
		function syncFastapiLinkTitle() {
			if (link) {
				var b = ivgFastapiBase();
				link.title = b + '/ 与 ' + b + '/health（与 WEB_PORT 默认 8088 一致时可改上方端口）';
			}
		}
		if (link) {
			link.addEventListener('click', function (e) {
				e.preventDefault();
				window.open(ivgFastapiBase() + '/', '_blank', 'noopener');
			});
		}
		if (portInput) {
			portInput.addEventListener('input', syncFastapiLinkTitle);
			syncFastapiLinkTitle();
		}
		bar.addEventListener('click', function (ev) {
			var btn = ev.target.closest('[data-ivg]');
			if (!btn) return;
			var k = btn.getAttribute('data-ivg');
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
		var q = parseInt(new URLSearchParams(window.location.search).get('rosbridge_port') || '9090', 10);
		return q || 9090;
	}

	function $(id) {
		return document.getElementById(id);
	}

	function setStatus(text, ok) {
		var el = $('conn-status');
		if (!el) return;
		el.textContent = text;
		el.className = 'status';
		if (ok) el.classList.add('ok');
		else if (/断开|错误|未连接|失败|连接错误/i.test(String(text))) el.classList.add('off');
	}

	function beginSidebarListFetch() {
		sidebarListInflight++;
		var sb = document.querySelector('.sidebar');
		var br = $('btn-refresh');
		var sl = $('sidebar-list');
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
		var sb = document.querySelector('.sidebar');
		var br = $('btn-refresh');
		if (sb) sb.classList.remove('sidebar-loading');
		if (br) {
			br.disabled = false;
			br.setAttribute('aria-busy', 'false');
		}
	}

	function setSidebarContextLabel(t) {
		var map = {
			topics: '话题',
			services: '服务',
			actions: '动作服务器',
			params: '参数',
			graph: '节点',
			view2d: '话题（2D 选图）',
			view3d: '话题（3D 选图）'
		};
		var el = $('sidebar-context');
		if (el) el.textContent = map[t] || '列表';
	}

	function syncLocalHostLinks() {
		var h = window.location.hostname || '127.0.0.1';
		var he = $('ivg-link-handeye');
		if (he) he.href = 'http://' + h + ':8080/';
	}

	function flashCopyButton(btn, ok) {
		if (!btn) return;
		if (btn._copyOrig == null) btn._copyOrig = btn.textContent;
		btn.textContent = ok ? '已复制' : '失败';
		btn.disabled = true;
		setTimeout(function () {
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
			navigator.clipboard.writeText(text).then(ok, function () {
				try {
					var ta = document.createElement('textarea');
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
			var ta2 = document.createElement('textarea');
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
		var cv = $('viz-canvas');
		if (cv) cv.style.display = 'none';
		refreshTopics();
	}

	function safeJson(obj, maxLen) {
		try {
			var s = JSON.stringify(obj, null, 2);
			if (maxLen && s.length > maxLen) {
				return s.slice(0, maxLen) + '\n…\n(truncated, total ' + s.length + ' chars)';
			}
			return s;
		} catch (e) {
			return String(obj);
		}
	}

	/** Normalize rosbridge / ROS2 message byte payloads to Uint8Array */
	function toUint8(data) {
		if (!data) return null;
		if (data instanceof Uint8Array) return data;
		if (typeof data === 'string') {
			try {
				var bin = atob(data);
				var out = new Uint8Array(bin.length);
				for (var i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
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
			var d = msg.data;
			if (typeof d === 'boolean' || typeof d === 'number' || typeof d === 'string') {
				return '<div class="viz-scalar">' + String(d) + '</div>';
			}
		}
		return null;
	}

	function renderTwist(msg) {
		var t = msg;
		if (msg && msg.twist) t = msg.twist;
		if (!t || !t.linear) return null;
		var L = t.linear;
		var A = t.angular || {};
		var rows = [
			['linear.x', L.x], ['linear.y', L.y], ['linear.z', L.z],
			['angular.x', A.x], ['angular.y', A.y], ['angular.z', A.z]
		];
		var html = '<table class="viz-table"><tbody>';
		for (var i = 0; i < rows.length; i++) {
			html += '<tr><th>' + rows[i][0] + '</th><td>' + rows[i][1] + '</td></tr>';
		}
		html += '</tbody></table>';
		return html;
	}

	function renderPose(msg) {
		var p = msg;
		if (msg && msg.pose) p = msg.pose;
		if (!p || !p.position) return null;
		var pos = p.position;
		var o = p.orientation || {};
		var html = '<table class="viz-table"><tbody>';
		html += '<tr><th>position</th><td>x=' + pos.x + ' y=' + pos.y + ' z=' + pos.z + '</td></tr>';
		html += '<tr><th>orientation</th><td>x=' + o.x + ' y=' + o.y + ' z=' + o.z + ' w=' + o.w + '</td></tr>';
		html += '</tbody></table>';
		return html;
	}

	function renderJointState(msg) {
		if (!msg || !Array.isArray(msg.name)) return null;
		var html = '<table class="viz-table"><thead><tr><th>name</th><th>position</th><th>velocity</th><th>effort</th></tr></thead><tbody>';
		var n = msg.name.length;
		for (var i = 0; i < n; i++) {
			html += '<tr><td>' + msg.name[i] + '</td><td>' +
				(msg.position && msg.position[i] !== undefined ? msg.position[i] : '') + '</td><td>' +
				(msg.velocity && msg.velocity[i] !== undefined ? msg.velocity[i] : '') + '</td><td>' +
				(msg.effort && msg.effort[i] !== undefined ? msg.effort[i] : '') + '</td></tr>';
		}
		html += '</tbody></table>';
		return html;
	}

	function renderImu(msg) {
		if (!msg) return null;
		var html = '<table class="viz-table"><tbody>';
		if (msg.orientation) {
			var o = msg.orientation;
			html += '<tr><th>orientation</th><td>x=' + o.x + ' y=' + o.y + ' z=' + o.z + ' w=' + o.w + '</td></tr>';
		}
		if (msg.angular_velocity) {
			var av = msg.angular_velocity;
			html += '<tr><th>angular_velocity</th><td>x=' + av.x + ' y=' + av.y + ' z=' + av.z + '</td></tr>';
		}
		if (msg.linear_acceleration) {
			var la = msg.linear_acceleration;
			html += '<tr><th>linear_acceleration</th><td>x=' + la.x + ' y=' + la.y + ' z=' + la.z + '</td></tr>';
		}
		html += '</tbody></table>';
		return html;
	}

	function renderBattery(msg) {
		if (!msg) return null;
		var html = '<table class="viz-table"><tbody>';
		var keys = ['voltage', 'temperature', 'current', 'charge', 'capacity', 'percentage', 'power_supply_status'];
		for (var i = 0; i < keys.length; i++) {
			if (Object.prototype.hasOwnProperty.call(msg, keys[i])) {
				html += '<tr><th>' + keys[i] + '</th><td>' + msg[keys[i]] + '</td></tr>';
			}
		}
		html += '</tbody></table>';
		return html;
	}

	function renderRange(msg) {
		if (!msg || typeof msg.range !== 'number') return null;
		var html = '<table class="viz-table"><tbody>';
		html += '<tr><th>range</th><td>' + msg.range + '</td></tr>';
		if (msg.min_range !== undefined) html += '<tr><th>min_range</th><td>' + msg.min_range + '</td></tr>';
		if (msg.max_range !== undefined) html += '<tr><th>max_range</th><td>' + msg.max_range + '</td></tr>';
		if (msg.radiation_type !== undefined) html += '<tr><th>radiation_type</th><td>' + msg.radiation_type + '</td></tr>';
		html += '</tbody></table>';
		return html;
	}

	function renderNavSatFix(msg) {
		if (!msg || typeof msg.latitude !== 'number') return null;
		var html = '<table class="viz-table"><tbody>';
		html += '<tr><th>latitude</th><td>' + msg.latitude + '</td></tr>';
		html += '<tr><th>longitude</th><td>' + msg.longitude + '</td></tr>';
		html += '<tr><th>altitude</th><td>' + msg.altitude + '</td></tr>';
		if (msg.status) html += '<tr><th>status</th><td>' + safeJson(msg.status, 400) + '</td></tr>';
		html += '</tbody></table>';
		return html;
	}

	function renderLaserScan(msg, canvas) {
		if (!msg || !Array.isArray(msg.ranges)) return '<p class="hint">无效的 LaserScan</p>';
		var w = canvas.parentElement.clientWidth || 400;
		var h = Math.min(360, Math.floor(w * 0.6));
		canvas.width = w;
		canvas.height = h;
		var ctx = canvas.getContext('2d');
		ctx.fillStyle = VIZ_CANVAS_BG;
		ctx.fillRect(0, 0, w, h);
		var cx = w * 0.5;
		var cy = h * 0.92;
		var maxR = Math.min(cx, cy) * 0.95;
		var amin = msg.angle_min;
		var inc = msg.angle_increment;
		var ranges = msg.ranges;
		ctx.strokeStyle = LASER_STROKE;
		ctx.lineWidth = 1;
		ctx.beginPath();
		for (var i = 0; i < ranges.length; i++) {
			var r = ranges[i];
			if (!isFinite(r) || r <= 0 || r > 1e6) continue;
			var ang = amin + i * inc;
			var px = cx + r / (msg.range_max || 10) * maxR * Math.cos(ang);
			var py = cy - r / (msg.range_max || 10) * maxR * Math.sin(ang);
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
		var W = msg.info.width;
		var H = msg.info.height;
		if (W * H !== msg.data.length) {
			return '<p class="hint">Grid size mismatch (width×height vs data)</p>';
		}
		var maxW = canvas.parentElement.clientWidth || 400;
		var scale = Math.max(1, Math.floor(maxW / W));
		canvas.width = W * scale;
		canvas.height = H * scale;
		var ctx = canvas.getContext('2d');
		var img = ctx.createImageData(W, H);
		var d = img.data;
		for (var i = 0; i < msg.data.length; i++) {
			var v = msg.data[i];
			var o = i * 4;
			if (v < 0) { d[o] = 80; d[o + 1] = 80; d[o + 2] = 120; d[o + 3] = 255; }
			else if (v === 0) { d[o] = 255; d[o + 1] = 255; d[o + 2] = 255; d[o + 3] = 255; }
			else { var g = 255 - Math.min(255, v * 2); d[o] = g; d[o + 1] = g; d[o + 2] = g; d[o + 3] = 255; }
		}
		var tmp = document.createElement('canvas');
		tmp.width = W;
		tmp.height = H;
		tmp.getContext('2d').putImageData(img, 0, 0);
		ctx.imageSmoothingEnabled = false;
		ctx.drawImage(tmp, 0, 0, W * scale, H * scale);
		return '<p class="hint">resolution ' + msg.info.resolution + ' m/cell · origin (' +
			msg.info.origin.position.x + ', ' + msg.info.origin.position.y + ')</p>';
	}

	function renderImage(msg, canvas) {
		if (!msg || !msg.width || !msg.height) return '<p class="hint">Invalid Image</p>';
		var enc = (msg.encoding || '').toLowerCase();
		var raw = toUint8(msg.data);
		if (!raw) return '<p class="hint">Could not decode image data (encoding ' + enc + ')</p>';
		var w = msg.width;
		var h = msg.height;
		canvas.width = w;
		canvas.height = h;
		var ctx = canvas.getContext('2d');
		var imgData = ctx.createImageData(w, h);
		var px = imgData.data;
		if (enc === 'rgb8' || enc === 'rgba8') {
			var step = enc === 'rgba8' ? 4 : 3;
			var need = w * h * step;
			if (raw.length < need) return '<p class="hint">Image buffer too short</p>';
			var p = 0;
			for (var i = 0; i < w * h; i++) {
				px[p++] = raw[i * step];
				px[p++] = raw[i * step + 1];
				px[p++] = raw[i * step + 2];
				px[p++] = 255;
			}
		} else if (enc === 'bgr8') {
			if (raw.length < w * h * 3) return '<p class="hint">Image buffer too short</p>';
			var p2 = 0;
			for (var j = 0; j < w * h; j++) {
				px[p2++] = raw[j * 3 + 2];
				px[p2++] = raw[j * 3 + 1];
				px[p2++] = raw[j * 3];
				px[p2++] = 255;
			}
		} else if (enc === 'mono8' || enc === '8uc1') {
			if (raw.length < w * h) return '<p class="hint">Image buffer too short</p>';
			var p3 = 0;
			for (var k = 0; k < w * h; k++) {
				var g = raw[k];
				px[p3++] = g;
				px[p3++] = g;
				px[p3++] = g;
				px[p3++] = 255;
			}
		} else {
			return '<p class="hint">Encoding <code>' + enc + '</code> — use Raw JSON or RViz; supported: rgb8, bgr8, rgba8, mono8</p>';
		}
		ctx.putImageData(imgData, 0, 0);
		var mw = canvas.parentElement.clientWidth || w;
		if (w > mw) {
			canvas.style.width = '100%';
			canvas.style.height = 'auto';
		}
		return '';
	}

	function renderCompressedImage(msg) {
		if (!msg || !msg.format || !msg.data) return null;
		var u8 = toUint8(msg.data);
		if (!u8) return '<p class="hint">无法解析压缩图像数据</p>';
		if (window.__labBlobUrl) {
			try { URL.revokeObjectURL(window.__labBlobUrl); } catch (e) { /* ignore */ }
		}
		var blob = new Blob([u8], { type: 'image/' + (msg.format.indexOf('png') !== -1 ? 'png' : 'jpeg') });
		window.__labBlobUrl = URL.createObjectURL(blob);
		return '<img src="' + window.__labBlobUrl + '" alt="compressed" style="max-width:100%;height:auto;border-radius:4px"/>';
	}

	function renderPointCloud2(msg) {
		if (!msg || !msg.fields) return null;
		var rows = '<table class="viz-table"><tbody>';
		rows += '<tr><th>height × width</th><td>' + msg.height + ' × ' + msg.width + '</td></tr>';
		rows += '<tr><th>point_step</th><td>' + msg.point_step + '</td></tr>';
		rows += '<tr><th>row_step</th><td>' + msg.row_step + '</td></tr>';
		rows += '<tr><th>fields</th><td>' + msg.fields.map(function (f) {
			return f.name + ' (' + f.datatype + '×' + f.count + ')';
		}).join(', ') + '</td></tr>';
		if (msg.header && msg.header.frame_id) {
			rows += '<tr><th>frame_id</th><td>' + msg.header.frame_id + '</td></tr>';
		}
		rows += '</tbody></table>';
		var raw = toUint8(msg.data);
		var bytes = raw ? raw.length : 0;
		rows += '<p class="hint">Payload ~' + bytes + ' bytes。IVG 默认点云：<code>/camera/depth_registered/points</code> → 顶部「3D: 点云」后启动 3D。</p>';
		return rows;
	}

	function renderPath(msg) {
		if (!msg || !Array.isArray(msg.poses)) return null;
		var n = msg.poses.length;
		if (n === 0) return '<p class="hint">Empty path</p>';
		var last = msg.poses[n - 1];
		var p = last.pose ? last.pose.position : null;
		return '<p class="hint">poses: ' + n + '</p>' + (p ? renderPose(last.pose) : '');
	}

	function renderPoseArray(msg) {
		if (!msg || !Array.isArray(msg.poses)) return null;
		var poses = msg.poses;
		var n = poses.length;
		var fid = (msg.header && msg.header.frame_id) ? msg.header.frame_id : '';
		var maxShow = Math.min(12, n);
		var html = '<p class="hint">poses: ' + n + (fid ? ' · frame <code>' + fid + '</code>' : '') + '</p>';
		html += '<table class="viz-table"><thead><tr><th>#</th><th>x</th><th>y</th><th>z</th><th>qx</th><th>qy</th><th>qz</th><th>qw</th></tr></thead><tbody>';
		var i;
		for (i = 0; i < maxShow; i++) {
			var po = poses[i];
			var pos = po.position || {};
			var ori = po.orientation || {};
			html += '<tr><td>' + i + '</td><td>' + pos.x + '</td><td>' + pos.y + '</td><td>' + pos.z + '</td><td>' +
				ori.x + '</td><td>' + ori.y + '</td><td>' + ori.z + '</td><td>' + ori.w + '</td></tr>';
		}
		html += '</tbody></table>';
		if (n > maxShow) html += '<p class="hint">… 其余 ' + (n - maxShow) + ' 条见 JSON</p>';
		return html;
	}

	function renderMarkerArray(msg) {
		if (!msg) return null;
		var arr = msg.markers || msg;
		if (!Array.isArray(arr)) return null;
		var n = arr.length;
		var maxShow = Math.min(12, n);
		var head = '<p class="hint">markers: ' + n + '（GraspNet 抓取可视化常用）</p>';
		head += '<table class="viz-table"><thead><tr><th>#</th><th>id</th><th>type</th><th>ns</th><th>frame</th><th>xyz</th><th>scale</th></tr></thead><tbody>';
		var i;
		for (i = 0; i < maxShow; i++) {
			var m = arr[i];
			var pos = (m.pose && m.pose.position) ? m.pose.position : {};
			var sc = m.scale || {};
			head += '<tr><td>' + i + '</td><td>' + (m.id != null ? m.id : '') + '</td><td>' + (m.type != null ? m.type : '') + '</td><td>' +
				(m.ns || '') + '</td><td>' + ((m.header && m.header.frame_id) || '') + '</td><td>' +
				(Number(pos.x).toFixed(3) + ', ' + Number(pos.y).toFixed(3) + ', ' + Number(pos.z).toFixed(3)) + '</td><td>' +
				(Number(sc.x || 0).toFixed(2) + '×' + Number(sc.y || 0).toFixed(2) + '×' + Number(sc.z || 0).toFixed(2)) + '</td></tr>';
		}
		head += '</tbody></table>';
		if (n > maxShow) head += '<p class="hint">… 其余见 JSON</p>';
		head += '<details class="viz-details"><summary class="viz-summary">首条原始 JSON</summary><pre class="raw">' +
			safeJson(arr[0] || {}, 4000) + '</pre></details>';
		return head;
	}

	function renderOdometry(msg) {
		if (!msg || !msg.pose || !msg.twist) return null;
		return '<h4 class="viz-subheading">pose</h4>' + renderPose(msg.pose) +
			'<h4 class="viz-subheading viz-subheading--spaced">twist</h4>' + renderTwist(msg.twist);
	}

	function renderGenericNumericArray(msg) {
		if (!msg || !msg.layout || !Array.isArray(msg.data)) return null;
		var dim = (msg.layout.dim || []).map(function (d) {
			return d.label + '=' + d.size;
		}).join(', ');
		var sample = msg.data.slice(0, 32).join(', ');
		return '<p class="hint">' + dim + '</p><pre class="raw">data[0..31]: ' + sample + (msg.data.length > 32 ? ' …' : '') + '</pre>';
	}

	function renderVisualization(msgType, msg, canvas) {
		var html = '';
		if (typeMatch(msgType, 'LaserScan')) {
			html = renderLaserScan(msg, canvas);
			return { html: html, usedCanvas: true };
		}
		if (typeMatch(msgType, 'OccupancyGrid')) {
			html = renderOccupancyGrid(msg, canvas);
			return { html: html, usedCanvas: true };
		}
		if (typeMatch(msgType, 'Image') && !typeMatch(msgType, 'CompressedImage')) {
			html = renderImage(msg, canvas);
			return { html: html, usedCanvas: true };
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
			var pra = renderPoseArray(msg);
			if (pra) return { html: pra, usedCanvas: false };
		}
		if (typeMatch(msgType, 'MarkerArray')) {
			return { html: renderMarkerArray(msg), usedCanvas: false };
		}
		if (typeMatch(msgType, 'Marker') && !typeMatch(msgType, 'MarkerArray')) {
			return { html: '<pre class="raw">' + safeJson(msg, 8000) + '</pre>', usedCanvas: false };
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
			var rg = renderRange(msg);
			if (rg) return { html: rg, usedCanvas: false };
		}
		if (typeMatch(msgType, 'NavSatFix')) {
			var ns = renderNavSatFix(msg);
			if (ns) return { html: ns, usedCanvas: false };
		}
		if (typeMatch(msgType, 'Twist')) {
			var tw = renderTwist(msg);
			if (tw) return { html: tw, usedCanvas: false };
		}
		if (typeMatch(msgType, 'Pose')) {
			var ps = renderPose(msg);
			if (ps) return { html: ps, usedCanvas: false };
		}
		if (typeMatch(msgType, 'Float32MultiArray') || typeMatch(msgType, 'Float64MultiArray') ||
				typeMatch(msgType, 'Int32MultiArray') || typeMatch(msgType, 'Int8MultiArray') ||
				typeMatch(msgType, 'UInt8MultiArray') || typeMatch(msgType, 'UInt16MultiArray')) {
			var ga = renderGenericNumericArray(msg);
			if (ga) return { html: ga, usedCanvas: false };
		}
		var sc = renderScalar(msg);
		if (sc) return { html: sc, usedCanvas: false };
		return {
			html: '<p class="hint">无专用视图；右侧为完整 JSON。可在 <code>js/ros_console.js</code> 的 <code>renderVisualization()</code> 中扩展。</p>',
			usedCanvas: false
		};
	}

	function onMessage(msg) {
		var now = Date.now();
		if (now - lastEmit < throttleMs) return;
		lastEmit = now;
		var type = selectedType;
		var vizBody = $('viz-body');
		var canvas = $('viz-canvas');
		var out = renderVisualization(type, msg, canvas);
		vizBody.innerHTML = out.html;
		canvas.style.display = out.usedCanvas ? 'block' : 'none';
		$('raw-pre').textContent = safeJson(msg, 120000);
	}

	function unsubscribe() {
		subscriptionSeq++;
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

	function subscribe(name, msgType) {
		if (!ros || !ros.isConnected) return;
		unsubscribe();
		var mySeq = subscriptionSeq;
		selectedName = name;
		selectedType = msgType;
		lastEmit = 0;
		$('selection-label').textContent = name + '  ·  ' + msgType;
		activeTopic = new ROSLIB.Topic({
			ros: ros,
			name: name,
			messageType: msgType,
			queue_length: 1
		});
		activeTopic.subscribe(function (msg) {
			if (mySeq !== subscriptionSeq) return;
			onMessage(msg);
		});
		$('viz-body').innerHTML = '<p class="hint">等待消息…</p>';
		$('raw-pre').textContent = '';
	}

	function fillSidebar(items, getMeta) {
		var list = $('sidebar-list');
		var q = ($('filter-q').value || '').toLowerCase();
		list.innerHTML = '';
		for (var i = 0; i < items.length; i++) {
			var name = typeof items[i] === 'string' ? items[i] : items[i].name;
			var meta = getMeta ? getMeta(items[i], i) : '';
			if (q && name.toLowerCase().indexOf(q) === -1 && (!meta || meta.toLowerCase().indexOf(q) === -1)) {
				continue;
			}
			var row = document.createElement('div');
			var rowActive = (tab === 'topics' && name === selectedName) ||
				(tab === 'services' && name === selectedService) ||
				(tab === 'params' && name === selectedParam) ||
				(tab === 'graph' && name === selectedName) ||
				(tab === 'actions' && name === selectedAction);
			row.className = 'row' + (rowActive ? ' active' : '');
			row.innerHTML = '<div class="name"></div><div class="meta"></div>';
			row.querySelector('.name').textContent = name;
			row.querySelector('.meta').textContent = meta;
			(function (n, m) {
				row.onclick = function () {
					if (tab === 'topics') subscribe(n, m);
					else if (tab === 'services') {
						selectedService = n;
						$('svc-selection').textContent = n;
						$('svc-resp').textContent = '';
						$('svc-req-json').value = '{}';
						ros.getServiceType(n, function (typ) {
							$('svc-type').value = typ;
						}, function (err) {
							$('svc-type').value = String(err);
						});
					} else if (tab === 'params') {
						selectedParam = n;
						fetchParamValue(n);
					} else if (tab === 'actions') {
						selectedAction = n;
						if ($('actions-dump')) {
							$('actions-dump').textContent = '选中: ' + n + '\n\n发送 goal / cancel 需按对应 .action 类型构造，可在此页扩展。';
						}
					} else if (tab === 'graph') {
						selectedName = n;
						if ($('graph-node-detail')) {
							$('graph-node-detail').textContent = '选中节点: ' + n + '\n点击「重新计算关系图」生成发布/订阅边。';
						}
					} else if (tab === 'view2d') {
						if (m.indexOf('OccupancyGrid') !== -1) $('map-topic').value = n;
						else if (m.indexOf('LaserScan') !== -1) $('scan-topic').value = n;
					} else if (tab === 'view3d') {
						if (m.indexOf('PointCloud2') !== -1) $('pc-topic').value = n;
						else if (m.indexOf('LaserScan') !== -1) $('scan3-topic').value = n;
					} else {
						selectedName = n;
						$('selection-label').textContent = n + (m ? '  ·  ' + m : '');
						$('viz-body').innerHTML = '<p class="hint">本标签为列表浏览。</p>';
						$('raw-pre').textContent = m || '';
					}
					Array.prototype.forEach.call(list.children, function (c) { c.classList.remove('active'); });
					row.classList.add('active');
				};
			})(name, meta);
			list.appendChild(row);
		}
		if (items.length && !list.children.length && q) {
			var hintRow = document.createElement('p');
			hintRow.className = 'hint';
			hintRow.textContent = '过滤无匹配（共 ' + items.length + ' 项）；请清空上方「过滤」或改关键词。';
			list.appendChild(hintRow);
		}
		var sc = $('sidebar-count');
		if (sc) {
			var nVis = list.querySelectorAll('.row').length;
			var qv = ($('filter-q').value || '').trim();
			if (!items.length) sc.textContent = '';
			else if (qv) sc.textContent = '显示 ' + nVis + ' / ' + items.length;
			else sc.textContent = items.length + ' 项';
		}
	}

	function refreshTopics() {
		if (!ros || !ros.isConnected) return;
		beginSidebarListFetch();
		var rid = ++topicsListReq;
		ros.getTopics(function (r) {
			if (rid !== topicsListReq) {
				endSidebarListFetch();
				return;
			}
			try {
				if (!r || typeof r !== 'object') r = {};
				var topics = Array.isArray(r.topics) ? r.topics : [];
				var types = Array.isArray(r.types) ? r.types : [];
				var combined = [];
				for (var i = 0; i < topics.length; i++) {
					combined.push({ name: topics[i], type: types[i] || '' });
				}
				combined.sort(function (a, b) {
					var ra = topicIvgRank(a.name);
					var rb = topicIvgRank(b.name);
					if (ra !== rb) return ra - rb;
					return a.name.localeCompare(b.name);
				});
				fillSidebar(combined, function (item) { return item.type; });
			} catch (e) {
				if (rid === topicsListReq) {
					var sl = $('sidebar-list');
					if (sl) sl.innerHTML = '<p class="hint">解析话题列表失败: ' + e + '</p>';
					var sc0 = $('sidebar-count');
					if (sc0) sc0.textContent = '';
				}
			} finally {
				endSidebarListFetch();
			}
		}, function (err) {
			if (rid !== topicsListReq) {
				endSidebarListFetch();
				return;
			}
			$('sidebar-list').innerHTML = '<p class="hint">getTopics 失败: ' + err + '</p>';
			var sc1 = $('sidebar-count');
			if (sc1) sc1.textContent = '';
			endSidebarListFetch();
		});
	}

	function refreshServices() {
		if (!ros || !ros.isConnected) return;
		beginSidebarListFetch();
		var rid = ++servicesListReq;
		ros.getServices(function (services) {
			if (rid !== servicesListReq) {
				endSidebarListFetch();
				return;
			}
			try {
				if (!Array.isArray(services)) services = [];
				services.sort();
				fillSidebar(services, function () { return 'service'; });
			} catch (e) {
				if (rid === servicesListReq) {
					var sl2 = $('sidebar-list');
					if (sl2) sl2.innerHTML = '<p class="hint">解析服务列表失败: ' + e + '</p>';
					var sc2 = $('sidebar-count');
					if (sc2) sc2.textContent = '';
				}
			} finally {
				endSidebarListFetch();
			}
		}, function (err) {
			if (rid !== servicesListReq) {
				endSidebarListFetch();
				return;
			}
			$('sidebar-list').innerHTML = '<p class="hint">getServices 失败: ' + err + '</p>';
			var sc3 = $('sidebar-count');
			if (sc3) sc3.textContent = '';
			endSidebarListFetch();
		});
	}

	function refreshNodes() {
		if (!ros || !ros.isConnected) return;
		beginSidebarListFetch();
		var rid = ++nodesListReq;
		ros.getNodes(function (nodes) {
			if (rid !== nodesListReq) {
				endSidebarListFetch();
				return;
			}
			if (!Array.isArray(nodes)) nodes = [];
			nodes.sort();
			fillSidebar(nodes, function () { return 'node'; });
			endSidebarListFetch();
		}, function (err) {
			if (rid !== nodesListReq) {
				endSidebarListFetch();
				return;
			}
			$('sidebar-list').innerHTML = '<p class="hint">getNodes 失败: ' + err + '</p>';
			var scn = $('sidebar-count');
			if (scn) scn.textContent = '';
			endSidebarListFetch();
		});
	}

	function showMain(mainId) {
		var ids = ['main-topics', 'main-services', 'main-actions', 'main-params', 'main-graph', 'main-view2d', 'main-view3d'];
		for (var i = 0; i < ids.length; i++) {
			var el = $(ids[i]);
			if (el) el.classList.toggle('hidden', ids[i] !== mainId);
		}
	}

	function refreshActionsList() {
		if (!ros || !ros.isConnected) return;
		beginSidebarListFetch();
		var rid = ++actionsListReq;
		ros.getActionServers(function (list) {
			if (rid !== actionsListReq) {
				endSidebarListFetch();
				return;
			}
			if (!Array.isArray(list)) list = [];
			list.sort();
			if ($('actions-dump')) $('actions-dump').textContent = list.length ? list.join('\n') : '(无动作服务器)';
			fillSidebar(list, function () { return 'action'; });
			endSidebarListFetch();
		}, function (err) {
			if (rid !== actionsListReq) {
				endSidebarListFetch();
				return;
			}
			$('sidebar-list').innerHTML = '<p class="hint">getActionServers 失败: ' + err + '</p>';
			var sca = $('sidebar-count');
			if (sca) sca.textContent = '';
			endSidebarListFetch();
		});
	}

	function refreshParamsList() {
		if (!ros || !ros.isConnected) return;
		beginSidebarListFetch();
		var rid = ++paramsListReq;
		ros.getParams(function (names) {
			if (rid !== paramsListReq) {
				endSidebarListFetch();
				return;
			}
			if (!Array.isArray(names)) names = [];
			names.sort();
			fillSidebar(names, function () { return 'param'; });
			endSidebarListFetch();
		}, function (err) {
			if (rid !== paramsListReq) {
				endSidebarListFetch();
				return;
			}
			$('sidebar-list').innerHTML = '<p class="hint">getParams 失败: ' + err + '</p>';
			var scp = $('sidebar-count');
			if (scp) scp.textContent = '';
			endSidebarListFetch();
		});
	}

	function fetchParamValue(paramName) {
		if (!ros || !ros.isConnected) return;
		var pid = ++paramFetchSeq;
		if ($('param-value')) $('param-value').textContent = '读取中…';
		var svc = new ROSLIB.Service({
			ros: ros,
			name: '/rosapi/get_param',
			serviceType: 'rosapi/GetParam'
		});
		var req = new ROSLIB.ServiceRequest({ name: paramName, default_value: '' });
		svc.callService(req, function (res) {
			if (pid !== paramFetchSeq) return;
			if ($('param-value')) $('param-value').textContent = res.value;
		}, function (err) {
			if (pid !== paramFetchSeq) return;
			if ($('param-value')) $('param-value').textContent = '错误: ' + err;
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

	function refreshSidebar() {
		if (!ros || !ros.isConnected) {
			var sl0 = $('sidebar-list');
			if (sl0) sl0.innerHTML = '<p class="hint">未连接 rosbridge，请点击顶栏「重连」。</p>';
			var scd = $('sidebar-count');
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
		var prevTab = tab;
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
		var buttons = document.querySelectorAll('.tabs button');
		for (var i = 0; i < buttons.length; i++) {
			var isSel = buttons[i].dataset.tab === t;
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
			requestAnimationFrame(function () {
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
		if (s.charAt(0) !== '/') s = '/' + s;
		return s.replace(/\/+/g, '/');
	}

	function pushUniqueName(arr, name) {
		if (!name || arr.indexOf(name) !== -1) return;
		arr.push(name);
	}

	/** 兼容 roslib 单参整包与三数组回调 */
	function coerceNodeDetails(sub, pub, srv) {
		var s0 = sub;
		var p0 = pub;
		var v0 = srv;
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

	function buildGraph() {
		if (!ros || !ros.isConnected) return;
		var bid = ++graphBuildSeq;
		var prog = $('graph-progress');
		if (prog) prog.textContent = '读取节点…';
		ros.getNodes(function (nodesIn) {
			if (bid !== graphBuildSeq) return;
			var nodes = Array.isArray(nodesIn) ? nodesIn.slice() : [];
			nodes.sort();
			var i = 0;
			var details = {};
			function step() {
				if (bid !== graphBuildSeq) return;
				if (i >= nodes.length) {
					finishGraph(bid, nodes, details);
					return;
				}
				var nm = nodes[i++];
				ros.getNodeDetails(nm, function (sub, pub, srv) {
					if (bid !== graphBuildSeq) return;
					var c = coerceNodeDetails(sub, pub, srv);
					details[nm] = { sub: c.sub, pub: c.pub, srv: c.srv };
					if (prog) prog.textContent = 'node_details ' + i + '/' + nodes.length;
					step();
				}, function () {
					if (bid !== graphBuildSeq) return;
					details[nm] = { sub: [], pub: [], srv: [] };
					step();
				});
			}
			step();
		}, function (err) {
			if (bid !== graphBuildSeq) return;
			if (prog) prog.textContent = 'getNodes 失败: ' + err;
		});
	}

	function finishGraph(bid, nodes, details) {
		if (bid !== graphBuildSeq) return;
		var topicEnds = {};
		var ei;
		for (ei = 0; ei < nodes.length; ei++) {
			var n = nodes[ei];
			var d = details[n];
			if (!d) continue;
			var p;
			for (p = 0; p < (d.pub || []).length; p++) {
				var tp = normalizeRosGraphName(d.pub[p]);
				if (!tp) continue;
				if (!topicEnds[tp]) topicEnds[tp] = { pub: [], sub: [] };
				pushUniqueName(topicEnds[tp].pub, n);
			}
			for (p = 0; p < (d.sub || []).length; p++) {
				var ts = normalizeRosGraphName(d.sub[p]);
				if (!ts) continue;
				if (!topicEnds[ts]) topicEnds[ts] = { pub: [], sub: [] };
				pushUniqueName(topicEnds[ts].sub, n);
			}
		}
		var edges = [];
		var edgeSeen = {};
		var tk = Object.keys(topicEnds);
		for (ei = 0; ei < tk.length; ei++) {
			var T = tk[ei];
			var tm = topicEnds[T];
			var a;
			for (a = 0; a < tm.pub.length; a++) {
				var b;
				for (b = 0; b < tm.sub.length; b++) {
					if (tm.pub[a] !== tm.sub[b]) {
						var ek = tm.pub[a] + '\n' + tm.sub[b] + '\n' + T;
						if (edgeSeen[ek]) continue;
						edgeSeen[ek] = true;
						edges.push({ from: tm.pub[a], to: tm.sub[b], topic: T });
					}
				}
			}
		}
		var nodeSet = {};
		for (ei = 0; ei < nodes.length; ei++) nodeSet[nodes[ei]] = true;
		for (ei = 0; ei < edges.length; ei++) {
			nodeSet[edges[ei].from] = true;
			nodeSet[edges[ei].to] = true;
		}
		var allNodes = Object.keys(nodeSet).sort();
		graphModel = { nodes: allNodes, edges: edges, topicEnds: topicEnds };
		redrawGraphCanvas();
	}

	function graphRosNamespace(fullName) {
		var n = normalizeRosGraphName(fullName);
		if (!n || n === '/') return '/';
		var idx = n.lastIndexOf('/');
		if (idx <= 0) return '/';
		return n.slice(0, idx);
	}

	function graphRosShortName(fullName) {
		var n = normalizeRosGraphName(fullName);
		var idx = n.lastIndexOf('/');
		if (idx < 0) return n;
		var s = n.slice(idx + 1);
		return s || n;
	}

	function getGraphViewMode() {
		var r = document.querySelector('input[name="graph-view"]:checked');
		return r && r.value === 'architecture' ? 'architecture' : 'logic';
	}

	function setupGraphCanvas2d() {
		var canvas = $('graph-canvas');
		if (!canvas) return null;
		var ctx = canvas.getContext('2d');
		var cssW = canvas.clientWidth;
		var cssH = canvas.clientHeight;
		if (!cssW || !cssH) {
			cssW = canvas.width || 900;
			cssH = canvas.height || 520;
		}
		var dpr = Math.min(window.devicePixelRatio || 1, 2.5);
		canvas.width = Math.max(1, Math.floor(cssW * dpr));
		canvas.height = Math.max(1, Math.floor(cssH * dpr));
		canvas.style.width = cssW + 'px';
		canvas.style.height = cssH + 'px';
		ctx.setTransform(1, 0, 0, 1, 0, 0);
		ctx.scale(dpr, dpr);
		return { canvas: canvas, ctx: ctx, W: cssW, H: cssH };
	}

	function pathRoundRect(ctx, x, y, w, h, rad) {
		var r = Math.min(rad, w / 2, h / 2);
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
		var pos = {};
		var N = nodes.length;
		var R = Math.min(W, H) * 0.32;
		var k;
		for (k = 0; k < N; k++) {
			var ang = 2 * Math.PI * k / Math.max(N, 1);
			pos[nodes[k]] = { x: W / 2 + R * Math.cos(ang), y: H / 2 + R * Math.sin(ang), vx: 0, vy: 0 };
		}
		var iter;
		for (iter = 0; iter < 100; iter++) {
			var i;
			for (i = 0; i < N; i++) {
				for (var j = i + 1; j < N; j++) {
					var na = nodes[i];
					var nb = nodes[j];
					var dx = pos[nb].x - pos[na].x;
					var dy = pos[nb].y - pos[na].y;
					var dist = Math.sqrt(dx * dx + dy * dy) + 0.01;
					var f = 800 / (dist * dist);
					pos[na].vx -= f * dx / dist;
					pos[na].vy -= f * dy / dist;
					pos[nb].vx += f * dx / dist;
					pos[nb].vy += f * dy / dist;
				}
			}
			for (i = 0; i < edges.length; i++) {
				var e = edges[i];
				var pa = pos[e.from];
				var pb = pos[e.to];
				if (!pa || !pb) continue;
				var ex = pb.x - pa.x;
				var ey = pb.y - pa.y;
				var d = Math.sqrt(ex * ex + ey * ey) + 0.01;
				var att = 0.03 * d;
				pa.vx += att * ex / d;
				pa.vy += att * ey / d;
				pb.vx -= att * ex / d;
				pb.vy -= att * ey / d;
			}
			for (i = 0; i < N; i++) {
				var nm = nodes[i];
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
			var ed = edges[i];
			if (!pos[ed.from] || !pos[ed.to]) continue;
			ctx.beginPath();
			ctx.moveTo(pos[ed.from].x, pos[ed.from].y);
			ctx.lineTo(pos[ed.to].x, pos[ed.to].y);
			ctx.stroke();
		}
		ctx.fillStyle = GRAPH_NODE;
		for (i = 0; i < N; i++) {
			var nn = nodes[i];
			ctx.beginPath();
			ctx.arc(pos[nn].x, pos[nn].y, 5, 0, Math.PI * 2);
			ctx.fill();
		}
		ctx.fillStyle = GRAPH_LABEL;
		ctx.font = '9px monospace';
		for (i = 0; i < N; i++) {
			var label = nodes[i].replace(/^\//, '');
			if (label.length > 22) label = label.slice(0, 20) + '…';
			ctx.fillText(label, pos[nodes[i]].x + 8, pos[nodes[i]].y - 6);
		}
		var gp = $('graph-progress');
		if (gp) {
			gp.textContent = '逻辑图 · 节点 ' + N + ' · 边 ' + edges.length + '（经话题连接）';
		}
	}

	function drawGraphArchitecture(ctx, W, H, nodes, edges) {
		var groupMap = {};
		var gi;
		for (gi = 0; gi < nodes.length; gi++) {
			var nm = nodes[gi];
			var ns = graphRosNamespace(nm);
			if (!groupMap[ns]) groupMap[ns] = [];
			pushUniqueName(groupMap[ns], nm);
		}
		var nsKeys = Object.keys(groupMap).sort();
		for (gi = 0; gi < nsKeys.length; gi++) {
			groupMap[nsKeys[gi]].sort();
		}
		var margin = 14;
		var gapX = 18;
		var gapY = 16;
		var pad = 10;
		var titleH = 20;
		var lineH = 14;
		var maxListLines = 18;
		var minBoxW = 108;
		var maxBoxW = 260;
		ctx.font = '10px monospace';
		var boxes = [];
		var x = margin;
		var y = margin;
		var rowH = 0;
		for (gi = 0; gi < nsKeys.length; gi++) {
			var nsK = nsKeys[gi];
			var members = groupMap[nsK];
			var titleText = nsK === '/' ? '/（根）' : nsK;
			var tw = ctx.measureText(titleText).width + pad * 2;
			var mw = tw;
			var listCount = Math.min(members.length, maxListLines);
			var li;
			for (li = 0; li < listCount; li++) {
				var sn = graphRosShortName(members[li]);
				if (sn.length > 32) sn = sn.slice(0, 30) + '…';
				mw = Math.max(mw, ctx.measureText(sn).width + pad * 2);
			}
			var bw = Math.max(minBoxW, Math.min(maxBoxW, mw));
			var extraLines = members.length > maxListLines ? 1 : 0;
			var bh = titleH + pad + listCount * lineH + extraLines * lineH + pad;
			if (x + bw > W - margin && x > margin) {
				x = margin;
				y += rowH + gapY;
				rowH = 0;
			}
			boxes.push({
				ns: nsK,
				x: x,
				y: y,
				w: bw,
				h: bh,
				members: members,
				cx: x + bw / 2,
				cy: y + bh / 2
			});
			rowH = Math.max(rowH, bh);
			x += bw + gapX;
		}
		var boxByNs = {};
		for (gi = 0; gi < boxes.length; gi++) {
			boxByNs[boxes[gi].ns] = boxes[gi];
		}
		var interSeen = {};
		var interList = [];
		for (gi = 0; gi < edges.length; gi++) {
			var eg = edges[gi];
			var nf = graphRosNamespace(eg.from);
			var nt = graphRosNamespace(eg.to);
			if (nf === nt) continue;
			var ik = nf + '\n' + nt;
			if (interSeen[ik]) continue;
			interSeen[ik] = true;
			interList.push({ from: nf, to: nt });
		}
		ctx.fillStyle = GRAPH_BG;
		ctx.fillRect(0, 0, W, H);
		ctx.strokeStyle = GRAPH_ARCH_EDGE;
		ctx.lineWidth = 1.25;
		for (gi = 0; gi < interList.length; gi++) {
			var ie = interList[gi];
			var bf = boxByNs[ie.from];
			var bt = boxByNs[ie.to];
			if (!bf || !bt) continue;
			var dx = bt.cx - bf.cx;
			var dy = bt.cy - bf.cy;
			var len = Math.sqrt(dx * dx + dy * dy) + 0.01;
			var ox = (-dy / len) * 24;
			var oy = (dx / len) * 24;
			var mx = (bf.cx + bt.cx) / 2 + ox;
			var my = (bf.cy + bt.cy) / 2 + oy;
			ctx.beginPath();
			ctx.moveTo(bf.cx, bf.cy);
			ctx.quadraticCurveTo(mx, my, bt.cx, bt.cy);
			ctx.stroke();
		}
		for (gi = 0; gi < boxes.length; gi++) {
			var b = boxes[gi];
			ctx.fillStyle = GRAPH_ARCH_BOX;
			pathRoundRect(ctx, b.x, b.y, b.w, b.h, 8);
			ctx.fill();
			ctx.strokeStyle = GRAPH_ARCH_STROKE;
			ctx.lineWidth = 1;
			ctx.stroke();
			ctx.fillStyle = GRAPH_ARCH_TITLE;
			ctx.font = 'bold 10px monospace';
			var tShow = b.ns === '/' ? '/（根命名空间）' : b.ns;
			if (tShow.length > 36) tShow = tShow.slice(0, 34) + '…';
			ctx.fillText(tShow, b.x + pad, b.y + pad + 10);
			ctx.fillStyle = GRAPH_ARCH_MEMBER;
			ctx.font = '9px monospace';
			var baseY = b.y + titleH + pad;
			var mi;
			var showN = Math.min(b.members.length, maxListLines);
			for (mi = 0; mi < showN; mi++) {
				var line = graphRosShortName(b.members[mi]);
				if (line.length > 34) line = line.slice(0, 32) + '…';
				ctx.fillText('· ' + line, b.x + pad, baseY + mi * lineH);
			}
			if (b.members.length > maxListLines) {
				ctx.fillStyle = '#64748b';
				ctx.fillText('… 共 ' + b.members.length + ' 个节点', b.x + pad, baseY + showN * lineH);
			}
		}
		var gp2 = $('graph-progress');
		if (gp2) {
			gp2.textContent = '架构图 · 命名空间 ' + boxes.length + ' · 组间连接 ' + interList.length +
				' · 节点 ' + nodes.length + ' · 边(逻辑) ' + edges.length;
		}
	}

	function redrawGraphCanvas() {
		var pack = setupGraphCanvas2d();
		if (!pack) return;
		if (!graphModel) {
			pack.ctx.fillStyle = GRAPH_BG;
			pack.ctx.fillRect(0, 0, pack.W, pack.H);
			pack.ctx.fillStyle = GRAPH_ARCH_MEMBER;
			pack.ctx.font = '12px monospace';
			pack.ctx.fillText('请点击「重新计算关系图」生成数据。', 24, 36);
			return;
		}
		var mode = getGraphViewMode();
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
		var h = $('view2d-host');
		if (h) h.innerHTML = '';
		viewer2d = null;
		gridRoot = null;
		scanShape = null;
	}

	function start2d() {
		stop2d();
		if (typeof ROS2D === 'undefined' || typeof createjs === 'undefined') {
			alert('ROS2D / EaselJS 未加载');
			return;
		}
		var host = $('view2d-host');
		var inner = document.createElement('div');
		inner.id = 'ros2d-inner';
		host.appendChild(inner);
		viewer2d = new ROS2D.Viewer({ divID: 'ros2d-inner', width: 800, height: 500, background: ROS2D_VIEWER_BG });
		gridRoot = new createjs.Container();
		viewer2d.addObject(gridRoot);
		gridRoot.addChild(new ROS2D.Grid({ size: 1 }));
		var currentGrid = null;
		var mapTopic = $('map-topic').value || '/map';
		mapTopicSub = new ROSLIB.Topic({
			ros: ros,
			name: mapTopic,
			messageType: 'nav_msgs/msg/OccupancyGrid',
			queue_length: 1
		});
		mapTopicSub.subscribe(function (message) {
			if (currentGrid) gridRoot.removeChild(currentGrid);
			currentGrid = new ROS2D.OccupancyGrid({ message: message });
			gridRoot.addChildAt(currentGrid, 0);
			var info = message.info;
			viewer2d.scaleToDimensions(info.width * info.resolution, info.height * info.resolution);
			viewer2d.shift(-info.origin.position.x, -info.origin.position.y);
		});
		var st = ($('scan-topic').value || '').trim();
		if (st) {
			scanShape = new createjs.Shape();
			gridRoot.addChild(scanShape);
			scanTopicSub = new ROSLIB.Topic({
				ros: ros,
				name: st,
				messageType: 'sensor_msgs/msg/LaserScan',
				queue_length: 1
			});
			scanTopicSub.subscribe(function (msg) {
				if (!msg || !Array.isArray(msg.ranges)) return;
				var g = scanShape.graphics;
				g.clear();
				g.setStrokeStyle(1);
				g.beginStroke('rgba(255,80,80,0.9)');
				var cx = 0;
				var cy = 0;
				var k;
				for (k = 0; k < msg.ranges.length; k++) {
					var r = msg.ranges[k];
					if (!isFinite(r) || r <= 0) continue;
					var ang = msg.angle_min + k * msg.angle_increment;
					var px = cx + r * Math.cos(ang);
					var py = cy - r * Math.sin(ang);
					if (k === 0) g.moveTo(px, py);
					else g.lineTo(px, py);
				}
				g.endStroke();
			});
		}
	}

	function stop3d() {
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
		var host = $('view3d-host');
		if (host) host.innerHTML = '';
		threeRenderer = null;
		threeScene = null;
		threeCamera = null;
		threeControls = null;
		pointsObj = null;
	}

	function fieldOffset(fields, name) {
		var f;
		for (f = 0; f < fields.length; f++) {
			if (fields[f].name === name) return fields[f].offset;
		}
		return -1;
	}

	function updatePointsFromPointCloud2(msg, maxPts) {
		if (!pointsObj || !msg.fields) return;
		var ox = fieldOffset(msg.fields, 'x');
		var oy = fieldOffset(msg.fields, 'y');
		var oz = fieldOffset(msg.fields, 'z');
		if (ox < 0 || oy < 0 || oz < 0) return;
		var buf = toUint8(msg.data);
		if (!buf || !buf.buffer) return;
		var le = !msg.is_bigendian;
		var step = msg.point_step;
		var total = msg.width * msg.height;
		var n = Math.min(total, maxPts);
		var arr = pointsObj.geometry.attributes.position.array;
		var dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
		var count = 0;
		var i;
		for (i = 0; i < n; i++) {
			var base = i * step;
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
		var arr = pointsObj.geometry.attributes.position.array;
		var count = 0;
		var i;
		for (i = 0; i < msg.ranges.length && count < maxPts; i++) {
			var r = msg.ranges[i];
			if (!isFinite(r) || r < msg.range_min || r > msg.range_max) continue;
			var ang = msg.angle_min + i * msg.angle_increment;
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
		var host = $('view3d-host');
		var w = host.clientWidth || 800;
		var h = 420;
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
		var dl = new THREE.DirectionalLight(0xffffff, 0.85);
		dl.position.set(3, 6, 4);
		threeScene.add(dl);
		var maxPts = Math.max(1000, parseInt($('pc-max').value, 10) || 20000);
		var geom = new THREE.BufferGeometry();
		var posArr = new Float32Array(maxPts * 3);
		geom.setAttribute('position', new THREE.BufferAttribute(posArr, 3));
		geom.setDrawRange(0, 0);
		var mat = new THREE.PointsMaterial({ size: 0.04, color: 0x38bdf8 });
		pointsObj = new THREE.Points(geom, mat);
		threeScene.add(pointsObj);
		function loop() {
			anim3d = requestAnimationFrame(loop);
			threeControls.update();
			threeRenderer.render(threeScene, threeCamera);
		}
		loop();
		var pcn = ($('pc-topic').value || '').trim();
		var sn = ($('scan3-topic').value || '').trim();
		if (pcn) {
			pcTopicSub = new ROSLIB.Topic({
				ros: ros,
				name: pcn,
				messageType: 'sensor_msgs/msg/PointCloud2',
				queue_length: 1
			});
			pcTopicSub.subscribe(function (m) {
				updatePointsFromPointCloud2(m, maxPts);
			});
		} else if (sn) {
			scan3TopicSub = new ROSLIB.Topic({
				ros: ros,
				name: sn,
				messageType: 'sensor_msgs/msg/LaserScan',
				queue_length: 1
			});
			scan3TopicSub.subscribe(function (m) {
				updatePointsFromLaserScan3(m, maxPts);
			});
		}
	}

	function connect() {
		if (ros) {
			try { ros.close(); } catch (e) { /* ignore */ }
		}
		ros = new ROSLIB.Ros({ groovyCompatibility: false });
		ros.on('connection', function () {
			syncLocalHostLinks();
			setStatus('已连接 ws://' + window.location.hostname + ':' + bridgePort(), true);
			refreshSidebar();
		});
		ros.on('error', function () {
			setStatus('连接错误', false);
		});
		ros.on('close', function () {
			setStatus('已断开', false);
			refreshSidebar();
		});
		ros.connect('ws://' + window.location.hostname + ':' + bridgePort());
	}

	document.addEventListener('DOMContentLoaded', function () {
		syncLocalHostLinks();
		function scheduleRefreshSidebar() {
			clearTimeout(filterDebounceTimer);
			filterDebounceTimer = setTimeout(function () {
				filterDebounceTimer = null;
				refreshSidebar();
			}, 280);
		}
		$('btn-refresh').onclick = function () {
			clearTimeout(filterDebounceTimer);
			filterDebounceTimer = null;
			/* 与切标签一致：作废所有进行中的列表请求，避免「刷新」仍被旧回调盖掉 */
			invalidateStaleSidebarLists();
			refreshSidebar();
		};
		$('filter-q').oninput = scheduleRefreshSidebar;
		document.addEventListener('keydown', function (ev) {
			if (ev.key !== 'Escape') return;
			var fq = $('filter-q');
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
		var br = $('btn-reconnect');
		if (br) br.onclick = function () { connect(); };
		var bu = $('btn-unsub-topic');
		if (bu) bu.onclick = stopTopicSubscription;
		var bc = $('btn-copy-raw');
		if (bc) bc.onclick = function () { copyToClipboard($('raw-pre').textContent, bc); };
		var bcs = $('btn-copy-svc-resp');
		if (bcs) bcs.onclick = function () { copyToClipboard($('svc-resp').textContent, bcs); };
		var bcp = $('btn-copy-param');
		if (bcp) bcp.onclick = function () { copyToClipboard($('param-value').textContent, bcp); };
		document.querySelectorAll('.tabs button').forEach(function (b) {
			b.onclick = function () { switchTab(b.dataset.tab); };
		});
		var bbg = $('btn-build-graph');
		if (bbg) bbg.onclick = buildGraph;
		document.querySelectorAll('input[name="graph-view"]').forEach(function (radio) {
			radio.addEventListener('change', function () {
				if (graphModel) redrawGraphCanvas();
			});
		});
		var b2s = $('btn-2d-start');
		if (b2s) b2s.onclick = start2d;
		var b2t = $('btn-2d-stop');
		if (b2t) b2t.onclick = stop2d;
		var b3s = $('btn-3d-start');
		if (b3s) b3s.onclick = start3d;
		var b3t = $('btn-3d-stop');
		if (b3t) b3t.onclick = stop3d;
		var svcBtn = $('svc-call');
		if (svcBtn) {
			svcBtn.onclick = function () {
				if (!ros || !ros.isConnected) {
					alert('未连接 rosbridge，请先重连');
					return;
				}
				if (!selectedService) {
					alert('请先在左侧选择服务');
					return;
				}
				var typ = $('svc-type').value;
				if (!typ) {
					alert('未获取到 service type');
					return;
				}
				var txt = ($('svc-req-json').value || '').trim() || '{}';
				var reqObj;
				try {
					reqObj = JSON.parse(txt);
				} catch (e) {
					alert('请求 JSON 无效');
					return;
				}
				var origLabel = svcBtn.textContent;
				svcBtn.disabled = true;
				svcBtn.textContent = '调用中…';
				$('svc-resp').textContent = '调用中…';
				var srv = new ROSLIB.Service({
					ros: ros,
					name: selectedService,
					serviceType: typ
				});
				srv.callService(new ROSLIB.ServiceRequest(reqObj), function (r) {
					svcBtn.disabled = false;
					svcBtn.textContent = origLabel;
					$('svc-resp').textContent = safeJson(r, 80000);
				}, function (e) {
					svcBtn.disabled = false;
					svcBtn.textContent = origLabel;
					$('svc-resp').textContent = '错误: ' + e;
				});
			};
		}
		bindIvgBar();
		connect();
	});
})();
