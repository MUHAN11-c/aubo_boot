/* global ivgTransport, ivgPorts, ROSLIB, IvgRos3dView3dSession */
/**
 * IVG 控制台：话题 JSON 经 ivg_transport；服务经 call_service。
 * 3D 点云：另建 ROSLIB.Ros → /ws/rosbridge，与 ros3djs 官方 PointCloud2 示例一致（见 ivg_ros3d_view3d.js）。
 */
(() => {
	const elCache = Object.create(null);
	function $(id) {
		if (Object.prototype.hasOwnProperty.call(elCache, id)) return elCache[id];
		const n = document.getElementById(id);
		elCache[id] = n;
		return n;
	}

	let selectedTopic = '';
	let selectedType = '';
	const rosReconnect = ivgPorts.createRosReconnectState();
	const ROS_RECONNECT_MAX = 12;
	/** ros3d 专用第二条 WebSocket（与 ivg_transport 并行，均为同源 /ws/rosbridge） */
	let labRos = null;
	let view3dSession = null;

	function disconnectLabRos() {
		stop3d();
		if (labRos) {
			try {
				labRos.close();
			} catch (e) {
				/* ignore */
			}
			labRos = null;
		}
	}

	function ensureLabRosConnected() {
		return new Promise((resolve, reject) => {
			if (labRos && labRos.isConnected) {
				resolve(labRos);
				return;
			}
			if (typeof ROSLIB === 'undefined' || typeof ROSLIB.Ros !== 'function') {
				reject(new Error('ROSLIB 未加载'));
				return;
			}
			disconnectLabRos();
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
				labRos = r;
				resolve(labRos);
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

	function setStatus(text, ok) {
		const el = $('conn-status');
		if (!el) return;
		el.textContent = text;
		el.className = 'status';
		if (ok) el.classList.add('ok');
		else el.classList.add('off');
	}

	function switchTab(name) {
		document.querySelectorAll('.tabs [role="tab"]').forEach(b => {
			const on = b.getAttribute('data-tab') === name;
			b.classList.toggle('active', on);
			b.setAttribute('aria-selected', on ? 'true' : 'false');
		});
		document.querySelectorAll('.main-stack').forEach(m => {
			m.classList.toggle('hidden', m.id !== `main-${name}`);
		});
	}

	function ivgSubscribeFixed(topic, msgType) {
		switchTab('topics');
		selectedTopic = topic;
		selectedType = msgType;
		if ($('selection-label')) $('selection-label').textContent = `${topic} (${msgType})`;
		ivgTransport.onRosJson(topic, payload => {
			const pre = $('raw-pre');
			if (pre) {
				try {
					pre.textContent = JSON.stringify(payload, null, 2);
				} catch (e) {
					pre.textContent = String(payload);
				}
			}
			if (msgType === 'sensor_msgs/msg/Image' || msgType.indexOf('Image') >= 0) {
				const host = $('viz-body');
				if (host) {
					host.innerHTML = `<p class="hint">sensor_msgs/Image → 请用 IVG MJPEG：</p><img class="viz-mjpeg" alt="" src="${ivgTransport.cameraStreamUrl(topic, 'lab_img')}" />`;
				}
			}
		});
		ivgTransport.subscribe({ topic, msgType, maxHz: 20 });
	}

	function ivgTrySubscribeMarkers() {
		ivgSubscribeFixed('/grasp_markers', 'visualization_msgs/msg/MarkerArray');
	}

	function ivg3dPointsPreset() {
		switchTab('view3d');
		const pc = $('pc-topic');
		if (pc) pc.value = '/camera/depth_registered/points';
	}

	function ivg3dPointsWebPreset() {
		switchTab('view3d');
		const pc = $('pc-topic');
		if (pc) pc.value = '/camera/depth_registered/points_web';
	}

	function ivg3dClearPreset() {
		const pc = $('pc-topic');
		if (pc) pc.value = '';
		const mt = $('marker3-topic');
		if (mt) mt.value = '';
	}

	function ivgPresetService(svcName, typeStr, jsonStr) {
		switchTab('services');
		if ($('svc-selection')) $('svc-selection').textContent = svcName;
		if ($('svc-type')) $('svc-type').value = typeStr;
		if ($('svc-req-json')) $('svc-req-json').value = jsonStr;
		if ($('svc-resp')) $('svc-resp').textContent = '';
	}

	function bindIvgBar() {
		const bar = $('ivg-bar');
		if (!bar) return;
		const link = $('ivg-link-fastapi');
		const portInput = $('ivg-fastapi-port');
		function ivgFastapiBase() {
			const p = (portInput && portInput.value.trim()) || '8088';
			const h = window.location.hostname || '127.0.0.1';
			return `http://${h}:${p}`;
		}
		if (link) {
			link.addEventListener('click', e => {
				e.preventDefault();
				window.open(`${ivgFastapiBase()}/`, '_blank', 'noopener');
			});
		}
		bar.addEventListener('click', ev => {
			const btn = ev.target.closest('[data-ivg]');
			if (!btn) return;
			const k = btn.getAttribute('data-ivg');
			if (k === 'topics-color') ivgSubscribeFixed('/camera/color/image_raw', 'sensor_msgs/msg/Image');
			else if (k === 'topics-depth') ivgSubscribeFixed('/camera/depth/image_raw', 'sensor_msgs/msg/Image');
			else if (k === 'topics-points') ivgSubscribeFixed('/camera/depth_registered/points', 'sensor_msgs/msg/PointCloud2');
			else if (k === 'topics-points-web') ivgSubscribeFixed('/camera/depth_registered/points_web', 'sensor_msgs/msg/PointCloud2');
			else if (k === 'topics-markers') ivgTrySubscribeMarkers();
			else if (k === 'topics-poses') ivgSubscribeFixed('/grasp_poses_base', 'geometry_msgs/msg/PoseArray');
			else if (k === '3d-points') ivg3dPointsPreset();
			else if (k === '3d-points-web') ivg3dPointsWebPreset();
			else if (k === '3d-clear') ivg3dClearPreset();
			else if (k === 'svc-list-templates') ivgPresetService('/list_templates', 'interface/srv/ListTemplates', '{"workpiece_id": ""}');
			else if (k === 'svc-graspnet-on') ivgPresetService('/graspnet_capture_control', 'std_srvs/srv/SetBool', '{"data": true}');
			else if (k === 'svc-graspnet-off') ivgPresetService('/graspnet_capture_control', 'std_srvs/srv/SetBool', '{"data": false}');
			else if (k === 'svc-worker-loop-on') ivgPresetService('/publish_grasps_worker_loop_control', 'std_srvs/srv/SetBool', '{"data": true}');
			else if (k === 'svc-worker-loop-off') ivgPresetService('/publish_grasps_worker_loop_control', 'std_srvs/srv/SetBool', '{"data": false}');
			else if (k === 'svc-loop-grasp-on') ivgPresetService('/loop_grasp_control', 'std_srvs/srv/SetBool', '{"data": true}');
			else if (k === 'svc-loop-grasp-off') ivgPresetService('/loop_grasp_control', 'std_srvs/srv/SetBool', '{"data": false}');
			else if (k === 'svc-exec-grasp') {
				ivgPresetService(
					'/execute_single_grasp',
					'demo_interface/srv/ExecuteGraspPose',
					'{\n  "object_id": "填写工件ID",\n  "use_visual_estimation": true\n}'
				);
			} else if (k === 'svc-gripper2') ivgPresetService('/run_gripper_swap', 'demo_interface/srv/RunGripperSwap', '{"direction": "gripper2"}');
			else if (k === 'svc-standardize') ivgPresetService('/standardize_template', 'interface/srv/StandardizeTemplate', '{"workpiece_id": ""}');
			else if (k === 'svc-estimate-hint') {
				switchTab('services');
				if ($('svc-resp')) {
					$('svc-resp').textContent = '位姿估计类大请求建议在终端调用对应服务，或使用顶部「视觉位姿网页」工具。';
				}
			} else if (k === 'topics-status') ivgSubscribeFixed('/system_status', 'std_msgs/msg/String');
			else if (k === 'tf-snapshot') {
				switchTab('topics');
				if (ivgTransport.ctrl && ivgTransport.ctrl.readyState === WebSocket.OPEN) {
					ivgTransport.ctrl.send(JSON.stringify({ op: 'tf_snapshot' }));
				}
			}
		});
	}

	function connect() {
		ivgPorts.clearRosReconnectTimer(rosReconnect);
		const myGen = ivgPorts.bumpRosReconnectGen(rosReconnect);
		disconnectLabRos();
		ivgTransport.close();
		void (async () => {
			try {
				await ivgTransport.loadRuntime();
				const rt = ivgTransport.runtime || {};
				const bridgeOk = rt.internal_bridge_ok !== false;
				await ivgTransport.connectControl();
				await ivgTransport.connectBinary().catch(() => {});
				if (myGen !== rosReconnect.gen) return;
				rosReconnect.attempts = 0;
				ivgPorts.clearRosReconnectTimer(rosReconnect);
				setStatus(
					bridgeOk ? '已连接（IVG）' : '已连接 · 内部桥未就绪（相机/点云可能不可用，请确认 ivg_ros_bridge）',
					true
				);
				ivgTransport.clearControlJsonHandlers();
				ivgTransport.onControlJson(o => {
					if (!o) return;
					if (o.op === 'error') {
						const pre = $('raw-pre');
						if (pre) pre.textContent = `[IVG] ${o.message != null ? String(o.message) : 'error'}`;
						if (o.message === 'bridge_unavailable') {
							setStatus('控制面已连 · 桥不可用（ivg_ros_bridge 未连网关）', false);
						}
						return;
					}
					if (o.op === 'tf_snapshot' || o.op === 'tf_lookup_result') {
						const pre = $('raw-pre');
						if (pre) {
							try {
								pre.textContent = JSON.stringify(o, null, 2);
							} catch (e) {
								pre.textContent = String(o);
							}
						}
					}
				});
				ivgTransport.clearBinaryHandlers();
			} catch (e) {
				if (myGen !== rosReconnect.gen) return;
				setStatus('连接失败', false);
				ivgPorts.scheduleRosReconnect(rosReconnect, connect, { maxAttempts: ROS_RECONNECT_MAX });
			}
		})();
	}

	function stop3d() {
		if (view3dSession) {
			try {
				view3dSession.stop();
			} catch (e) {
				/* ignore */
			}
			view3dSession = null;
		}
	}

	function start3d() {
		if (typeof IvgRos3dView3dSession !== 'function') {
			alert('未加载 ivg_ros3d_view3d.js 或 ros3d');
			return;
		}
		void ensureLabRosConnected()
			.then(() => {
				stop3d();
				view3dSession = new IvgRos3dView3dSession(labRos, $);
				view3dSession.start();
			})
			.catch(err => {
				alert(`3D 需要连接 rosbridge：${err && err.message ? err.message : String(err)}`);
			});
	}

	document.addEventListener('DOMContentLoaded', () => {
		void ivgPorts.loadRuntime();
		bindIvgBar();
		const h = window.location.hostname || '127.0.0.1';
		const he = $('ivg-link-handeye');
		if (he) he.href = `http://${h}:8080/`;

		document.querySelectorAll('.tabs [role="tab"]').forEach(btn => {
			btn.addEventListener('click', () => switchTab(btn.getAttribute('data-tab')));
		});

		const stubIds = ['main-actions', 'main-params', 'main-graph', 'main-view2d'];
		stubIds.forEach(id => {
			const m = document.getElementById(id);
			if (!m) return;
			const hint = document.createElement('p');
			hint.className = 'hint';
			hint.textContent =
				'IVG 网关不包含 rosapi：动作/参数/节点图/2D occupancy 等列表不可用。请使用命令行 ros2 CLI 或后续扩展。';
			m.insertBefore(hint, m.firstChild);
		});

		if ($('btn-reconnect')) $('btn-reconnect').onclick = () => connect();
		if ($('btn-unsub-topic')) {
			$('btn-unsub-topic').onclick = () => {
				if (selectedTopic) ivgTransport.unsubscribe(selectedTopic);
				selectedTopic = '';
				if ($('selection-label')) $('selection-label').textContent = '未选择话题';
			};
		}
		if ($('btn-3d-start')) $('btn-3d-start').onclick = () => start3d();
		if ($('btn-3d-stop')) $('btn-3d-stop').onclick = () => stop3d();

		if ($('btn-refresh')) {
			$('btn-refresh').onclick = () => {
				const sl = $('sidebar-list');
				if (sl) {
					sl.innerHTML =
						'<p class="hint" role="status">IVG 无 rosapi：请用主区「手动订阅」或顶部快捷按钮。</p>';
				}
			};
			$('btn-refresh').click();
		}

		if ($('svc-call')) {
			$('svc-call').onclick = () => {
				const name = ($('svc-selection') && $('svc-selection').textContent.trim()) || '';
				const typ = ($('svc-type') && $('svc-type').value.trim()) || '';
				let req = {};
				try {
					req = JSON.parse(($('svc-req-json') && $('svc-req-json').value) || '{}');
				} catch (e) {
					if ($('svc-resp')) $('svc-resp').textContent = '请求 JSON 无效';
					return;
				}
				if (!name || !typ) return;
				ivgTransport
					.callService({ service: name, type: typ, request: req })
					.then(r => {
						if ($('svc-resp')) $('svc-resp').textContent = JSON.stringify(r, null, 2);
					})
					.catch(e => {
						if ($('svc-resp')) $('svc-resp').textContent = String(e);
					});
			};
		}

		const manual = document.createElement('div');
		manual.className = 'controls';
		manual.innerHTML =
			'<label>话题 <input type="text" id="ivg-manual-topic" style="width:14rem" placeholder="/joint_states" /></label> ' +
			'<label>类型 <input type="text" id="ivg-manual-type" style="width:18rem" placeholder="sensor_msgs/msg/JointState" /></label> ' +
			'<button type="button" id="ivg-manual-sub">订阅</button>';
		const mainT = $('main-topics');
		if (mainT) mainT.insertBefore(manual, mainT.firstChild);
		const btnM = $('ivg-manual-sub');
		if (btnM) {
			btnM.onclick = () => {
				const t = ($('ivg-manual-topic') && $('ivg-manual-topic').value.trim()) || '';
				const ty = ($('ivg-manual-type') && $('ivg-manual-type').value.trim()) || '';
				if (t && ty) ivgSubscribeFixed(t, ty);
			};
		}

		ivgPorts.wireOnlineRosReconnect(rosReconnect, connect);
		connect();
	});
})();
