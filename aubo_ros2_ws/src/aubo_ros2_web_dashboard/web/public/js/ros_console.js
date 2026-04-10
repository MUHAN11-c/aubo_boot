/* global ROSLIB, ROS2D, ROS3D, createjs, ivgPorts, IVGTopicsLabRender */
/* topics_lab：编排 roslib/rosapi/2D/3D；消息类型 → HTML 见 js/topics_lab/render.js。sensor_msgs/Image 仅 MJPEG（无 rosbridge 收图）。 */
(() => {
	/** 当前 ROSLIB.Ros 实例（直连 rosbridge WebSocket）。 */
	let ros = null;
	let activeTopic = null;
	/** 每次取消订阅 +1，用于丢弃旧 Topic 在 unsubscribe 后仍可能到达的回调（避免 raw JSON / 视图刷成上一话题） */
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
	let throttleMs = 40;
	let lastEmit = 0;
	let selectedName = '';
	let selectedType = '';
	let tab = 'topics';
	let selectedService = '';
	let selectedParam = '';
	let selectedAction = '';
	/** 2D：与 ros2djs continuous 示例一致 — ROS2D.OccupancyGridClient（上游无数组 dispose，重复启动前建议重连 rosbridge） */
	let mapGridClient = null;
	let viewer2d = null;
	let viewer3d = null;
	let tfClient3d = null;
	let ros3dPointCloud2 = null;
	let ros3dLaserScan = null;
	let ros3dMarkerClient = null;
	let ros3dAxes = null;
	let ros3dGrid = null;
	/** 3D：ROS3D.UrdfClient 根节点（随 ROS2TFClient 更新，与 RViz 类似） */
	let ros3dUrdfRoot = null;
	/** 话题页 viz 渲染合并到下一帧，避免 rosbridge 回调内直接改 DOM 造成掉帧（RobotWebTools/roslib 高频消息场景） */
	let topicVizRaf = null;
	/** 当前话题为 web_video_server MJPEG，未创建 ROSLIB.Topic */
	let webVideoStreamMode = false;
	/** rosbridge 断线自动重连（手动点重连会 bump gen，旧实例的 close 不触发调度） */
	const rosReconnect = ivgPorts.createRosReconnectState();
	const ROS_RECONNECT_MAX = 15;
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

	/** ROS PointCloud2 常见：rgb 字段为 float32，实为 0xRRGGBB 打包进 IEEE754；ros3d 默认 new THREE.Color(x) 会解析错 → 全黑/发灰。 */
	function ivgRosPackedRgbFloatToColor(rgbFloat, littleEndian) {
		const ab = new ArrayBuffer(4);
		const dv = new DataView(ab);
		dv.setFloat32(0, rgbFloat, littleEndian);
		const u = dv.getUint32(0, littleEndian);
		const r = (u >> 16) & 255;
		const g = (u >> 8) & 255;
		const b = u & 255;
		return new THREE.Color(r / 255, g / 255, b / 255);
	}

	/* ---------- IVG 顶栏：话题排序、VPE FastAPI 端口、快捷订阅与常用服务预设 ---------- */
	/** 与 start_IVG_graspnet_points_fastapi.sh + graspnet_demo_points_with_tf 默认一致 */
	const IVG_TOPIC_ORDER = [
		'/camera/color/image_raw',
		'/camera/depth/image_raw',
		'/camera/depth_registered/points',
		'/camera/depth_registered/points_web',
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

	function ivgSubscribeFixed(name, defaultMsgType) {
		if (!ros || !ros.isConnected) return;
		switchTab('topics');
		ros.getTopicType(name, typ => {
			subscribe(name, typ);
			refreshTopics();
		}, () => {
			if (defaultMsgType) subscribe(name, defaultMsgType);
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

	function ivg3dPointsWebPreset() {
		switchTab('view3d');
		$('pc-topic').value = '/camera/depth_registered/points_web';
		$('scan3-topic').value = '';
	}

	function ivg3dClearPreset() {
		$('pc-topic').value = '';
		$('scan3-topic').value = '';
		const mt = $('marker3-topic');
		if (mt) mt.value = '';
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
			else if (k === 'topics-points-web') {
				ivgSubscribeFixed('/camera/depth_registered/points_web', 'sensor_msgs/msg/PointCloud2');
			} else if (k === 'topics-markers') ivgTrySubscribeMarkers();
			else if (k === 'topics-poses') ivgSubscribeFixed('/grasp_poses_base', 'geometry_msgs/msg/PoseArray');
			else if (k === '3d-points') ivg3dPointsPreset();
			else if (k === '3d-points-web') ivg3dPointsWebPreset();
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
						'1）用顶部「视觉位姿网页」入口（默认端口 8088）；',
						'2）或在终端查看接口定义后调用对应服务 …',
						''
					].join('\n');
				}
			} else if (k === 'topics-status') {
				ivgSubscribeFixed('/system_status', 'std_msgs/msg/String');
			}
		});
	}

	function webVideoPort() {
		return ivgPorts.webVideo($('web-video-port'));
	}

	function reconnectImageTopicIfNeeded() {
		if (tab !== 'topics' || !selectedName || !selectedType || !ros || !ros.isConnected) return;
		if (!IVGTopicsLabRender.typeMatch(selectedType, 'Image') || IVGTopicsLabRender.typeMatch(selectedType, 'CompressedImage')) return;
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
			view2d: '话题（2D 地图）',
			view3d: '话题（3D 点云/雷达）'
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
			if (!vizBody || !canvas) return;
			const out = IVGTopicsLabRender.renderVisualization(type, msg, canvas);
			vizBody.innerHTML = out.html;
			canvas.style.display = out.usedCanvas ? 'block' : 'none';
			// 大话题勿 JSON.stringify 全量 data，否则主线程阻塞数秒；摘要即可
			$('raw-pre').textContent = IVGTopicsLabRender.rawPreviewForMessage(type, msg, 120000);
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
				if (typeof activeTopic.unsubscribe === 'function') {
					activeTopic.unsubscribe();
				} else if (typeof activeTopic.dispose === 'function') {
					activeTopic.dispose();
				}
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
		if (IVGTopicsLabRender.typeMatch(msgType, 'Image') && !IVGTopicsLabRender.typeMatch(msgType, 'CompressedImage')) {
			if (typeof ivgWebVideo === 'undefined') {
				$('viz-body').innerHTML =
					'<p class="hint">sensor_msgs/Image 仅通过 <strong>web_video_server</strong> MJPEG 显示。请确认本页已加载 <code>ivg_web_video.js</code> 且 launch 已启动 web_video_server。</p>';
				$('raw-pre').textContent = '';
				return;
			}
			webVideoStreamMode = true;
			const wvPort = webVideoPort();
			const url = ivgWebVideo.streamUrl(name, {
				port: wvPort,
				quality: 88
			});
			const viewer = ivgWebVideo.viewerUrl(name, { port: wvPort });
			const vcMjpeg = $('viz-canvas');
			if (vcMjpeg) vcMjpeg.style.display = 'none';
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
			if (typeof ivgWebVideo !== 'undefined' && typeof ivgWebVideo.mjpegStreamAttachAutoReload === 'function') {
				ivgWebVideo.mjpegStreamAttachAutoReload(img, () =>
					ivgWebVideo.streamUrl(name, {
						port: webVideoPort(),
						quality: 88
					})
				);
			}
			wrap.appendChild(img);
			vizBody.appendChild(wrap);
			$('raw-pre').textContent = `[MJPEG] ${name}\n${url}\n\nCompressedImage 仍走 rosbridge。`;
			return;
		}
		const mySeq = subscriptionSeq;
		activeTopic = new ROSLIB.Topic({
			ros,
			name,
			messageType: msgType,
			queue_length: 1,
			throttle_rate: throttleMs
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
		function showErr(err) {
			if (pid !== paramFetchSeq) return;
			if ($('param-value')) $('param-value').textContent = `错误: ${err}`;
		}
		function callGetParam(serviceType, triedAlt) {
			const svc = new ROSLIB.Service({
				ros,
				name: '/rosapi/get_param',
				serviceType
			});
			svc.callService({ name: paramName, default_value: '' }, res => {
				if (pid !== paramFetchSeq) return;
				if ($('param-value')) $('param-value').textContent = res.value;
			}, err => {
				if (pid !== paramFetchSeq) return;
				if (!triedAlt && serviceType === 'rosapi/GetParam') {
					callGetParam('rosapi/srv/GetParam', true);
					return;
				}
				showErr(err);
			});
		}
		callGetParam('rosapi/GetParam', false);
	}

	/**
	 * 切换标签时抬高所有侧栏列表的请求代数。
	 * 仅抬高「非当前数据源」会在话题/2D/3D 共用 topics 列表、或 IVG 连点等组合下漏掉应失效的回调，
	 * 晚到的 getTopics/getServices 仍可能把侧栏刷成上一标签的内容；此处一律作废全部进行中请求。
	 * paramFetchSeq 同步抬高，避免离开「参数」后 get_param 晚到仍改写 #param-value。
	 */
	function invalidateStaleSidebarLists() {
		topicsListReq++;
		servicesListReq++;
		nodesListReq++;
		actionsListReq++;
		paramsListReq++;
		paramFetchSeq++;
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
				$('selection-label').textContent = '未选择话题';
				$('viz-body').innerHTML = '<p class="hint">切换到「话题」可订阅。</p>';
				const cvTab = $('viz-canvas');
				if (cvTab) cvTab.style.display = 'none';
				$('raw-pre').textContent = '';
			}
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
		const h = $('view2d-host');
		if (h) h.innerHTML = '';
		viewer2d = null;
		mapGridClient = null;
	}

	/* ---------- 2D：ros2djs continuous.html；3D：ros3djs pointcloud2.html、LaserScan、markers.html + 场景坐标轴/网格 ---------- */
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
		const mapTopic = ($('map-topic').value || '/map').trim();
		mapGridClient = new ROS2D.OccupancyGridClient({
			ros,
			topic: mapTopic,
			rootObject: viewer2d.scene,
			continuous: true
		});
		mapGridClient.on('change', () => {
			const mc = mapGridClient;
			const v = viewer2d;
			if (!mc || !v) return;
			const cg = mc.currentGrid;
			if (!cg || !cg.pose) return;
			v.scaleToDimensions(cg.width, cg.height);
			v.shift(cg.pose.position.x, cg.pose.position.y);
		});
	}

	function stop3d() {
		if (ros3dMarkerClient) {
			try { ros3dMarkerClient.unsubscribe(); } catch (e0) { /* ignore */ }
			ros3dMarkerClient = null;
		}
		if (ros3dPointCloud2) {
			try { ros3dPointCloud2.unsubscribe(); } catch (e1) { /* ignore */ }
			ros3dPointCloud2 = null;
		}
		if (ros3dLaserScan) {
			try { ros3dLaserScan.unsubscribe(); } catch (e2) { /* ignore */ }
			ros3dLaserScan = null;
		}
		if (viewer3d) {
			try {
				if (ros3dUrdfRoot) {
					viewer3d.scene.remove(ros3dUrdfRoot);
					ros3dUrdfRoot = null;
				}
				if (ros3dAxes) {
					viewer3d.scene.remove(ros3dAxes);
					ros3dAxes = null;
				}
				if (ros3dGrid) {
					viewer3d.scene.remove(ros3dGrid);
					ros3dGrid = null;
				}
			} catch (e3a) { /* ignore */ }
			try { viewer3d.stop(); } catch (e4) { /* ignore */ }
			viewer3d = null;
		} else {
			ros3dUrdfRoot = null;
			ros3dAxes = null;
			ros3dGrid = null;
		}
		if (tfClient3d) {
			try { tfClient3d.dispose(); } catch (e3) { /* ignore */ }
			tfClient3d = null;
		}
		removeView3dUrdfHint();
		const host3 = $('view3d-host');
		if (host3) host3.innerHTML = '';
	}

	function removeView3dPc2Hint() {
		const el = document.getElementById('view3d-pc2-hint');
		if (el) el.remove();
	}

	/** 点云 6s 内无数据时的排查说明（依赖 rosapi/publishers 区分无发布者 vs QoS/rosbridge） */
	function showView3dPc2Hint(hostEl, html) {
		removeView3dPc2Hint();
		if (!hostEl) return;
		const d = document.createElement('div');
		d.id = 'view3d-pc2-hint';
		d.className = 'hint';
		d.style.marginTop = '0.5rem';
		d.innerHTML = html;
		hostEl.appendChild(d);
	}

	function removeView3dUrdfHint() {
		const el = document.getElementById('view3d-urdf-hint');
		if (el) el.remove();
	}

	function showView3dUrdfHint(hostEl, html) {
		removeView3dUrdfHint();
		if (!hostEl) return;
		const d = document.createElement('div');
		d.id = 'view3d-urdf-hint';
		d.className = 'hint';
		d.style.marginTop = '0.5rem';
		d.innerHTML = html;
		hostEl.appendChild(d);
	}

	/** rosapi GetParam 的 value 为 JSON.dumps；偶发已解包成裸字符串时勿强依赖 JSON.parse。 */
	function ivgDecodeRosapiParamString(raw) {
		if (raw == null || raw === '') return '';
		if (typeof raw === 'string') {
			try {
				const v = JSON.parse(raw);
				return typeof v === 'string' ? v : '';
			} catch (e0) {
				return raw;
			}
		}
		return '';
	}

	function ivgAttachUrdfFromRosParam(ros, paramFullName, meshBase, tfClient, rootGroup, onErr) {
		if (!ros || typeof ROSLIB.Service !== 'function') {
			onErr('ROSLIB.Service 不可用');
			return;
		}
		const svc = new ROSLIB.Service({
			ros,
			name: '/rosapi/get_param',
			serviceType: 'rosapi/GetParam'
		});
		svc.callService(
			{ name: paramFullName, default_value: '' },
			resp => {
				const rawVal = resp && Object.prototype.hasOwnProperty.call(resp, 'value') ? resp.value : '';
				const xml = ivgDecodeRosapiParamString(rawVal).trim();
				if (!xml) {
					onErr('get_param 返回空或无法解析为字符串（请核对「URDF 参数」全名）');
					return;
				}
				const looks =
					xml.indexOf('<robot') !== -1 || xml.indexOf('<?xml') !== -1 || xml.indexOf('<urdf') !== -1;
				if (!looks) {
					onErr(`返回值不像 URDF XML（开头）：${JSON.stringify(xml.slice(0, 96))}`);
					return;
				}
				try {
					const urdfModel = new ROSLIB.UrdfModel({ string: xml });
					const urdfViz = new ROS3D.Urdf({
						urdfModel,
						path: meshBase,
						tfClient,
						tfPrefix: ''
					});
					rootGroup.add(urdfViz);
					const h = $('view3d-host');
					if (h) {
						showView3dUrdfHint(
							h,
							'<strong>机械臂</strong>：URDF 已解析，网格异步加载中；若仍不可见请看 Network 是否对 <code>/api/ivg/robot-mesh/…</code> 404。'
						);
						setTimeout(removeView3dUrdfHint, 10000);
					}
				} catch (e2) {
					onErr(e2 && e2.message ? e2.message : String(e2));
				}
			},
			err => onErr(err && err.message ? err.message : String(err))
		);
	}

	let ivgPc2NativeProcessMessage = null;

	function ivgPc2RemoveLoadingHintOnFirstFrame(self) {
		const sig = self.__ivgPc2Sig;
		if (!sig || sig.got) return;
		sig.got = true;
		removeView3dPc2Hint();
	}

	/** 在 max_pts 限制下按步长均匀抽样整幅点云（ros3d 原版只画前 max_pts 个连续点，大图会缺大半）。 */
	function ivgPointCloud2FillUniformStride(self, msg) {
		const raw = msg.data;
		const total = (msg.width * msg.height) | 0;
		const ps = msg.point_step | 0;
		if (total <= 0 || ps <= 0 || !raw || raw.byteLength == null) return;
		const maxDraw = Math.min(self.max_pts, Math.floor(self.points.positions.array.length / 3));
		let stride = 1;
		if (total > maxDraw) stride = Math.ceil(total / maxDraw);
		let n = Math.min(maxDraw, Math.ceil(total / stride));
		const lastBase = (n - 1) * stride * ps + ps;
		if (lastBase > raw.byteLength) {
			n = Math.max(0, Math.floor(raw.byteLength / (stride * ps)));
		}
		const dv = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
		const littleEndian = msg.is_bigendian !== true;
		const xo = self.points.fields.x.offset;
		const yo = self.points.fields.y.offset;
		const zo = self.points.fields.z.offset;
		let base;
		let color;
		for (let i = 0; i < n; i++) {
			base = i * stride * ps;
			self.points.positions.array[3 * i] = dv.getFloat32(base + xo, littleEndian);
			self.points.positions.array[3 * i + 1] = dv.getFloat32(base + yo, littleEndian);
			self.points.positions.array[3 * i + 2] = dv.getFloat32(base + zo, littleEndian);
			if (self.points.colors) {
				color = self.points.colormap(self.points.getColor(dv, base, littleEndian));
				self.points.colors.array[3 * i] = color.r;
				self.points.colors.array[3 * i + 1] = color.g;
				self.points.colors.array[3 * i + 2] = color.b;
			}
		}
		self.points.update(n);
	}

	/** 替换 PointCloud2.processMessage：TF/颜色辅助字段 + 可选均匀步进（CBOR 二进制路径）。 */
	function installIvgPointCloud2ProcessPatch() {
		if (typeof ROS3D === 'undefined' || !ROS3D.PointCloud2) return;
		const P = ROS3D.PointCloud2.prototype;
		if (P.__ivgPc2ProcessPatched) return;
		if (typeof P.processMessage !== 'function') return;
		ivgPc2NativeProcessMessage = P.processMessage;
		P.__ivgPc2ProcessPatched = true;
		P.processMessage = function (msg) {
			this.__ivgPc2Le = msg.is_bigendian === true ? false : true;
			const rgbF = (msg.fields || []).find(f => f && f.name === 'rgb');
			this.__ivgRgbFieldDt = rgbF ? rgbF.datatype : 0;

			const typedArray =
				msg.data && typeof msg.data !== 'string' && msg.data.buffer && typeof msg.data.buffer === 'object';
			if (this.__ivgUniformStride && typedArray) {
				if (!this.points.setup(msg.header.frame_id, msg.point_step, msg.fields)) return;
				ivgPc2RemoveLoadingHintOnFirstFrame(this);
				this.buffer = msg.data;
				ivgPointCloud2FillUniformStride(this, msg);
				return;
			}

			ivgPc2RemoveLoadingHintOnFirstFrame(this);
			return ivgPc2NativeProcessMessage.call(this, msg);
		};
	}

	function start3d() {
		const pcn = ($('pc-topic').value || '').trim();
		const sn = ($('scan3-topic').value || '').trim();
		const mk = (($('marker3-topic') && $('marker3-topic').value) || '').trim();
		const urdfEl = $('view3d-show-urdf');
		const wantUrdf = !urdfEl || urdfEl.checked;
		if (!pcn && !sn && !mk && !wantUrdf) {
			alert('请至少填写「点云」「雷达」或「Marker」话题之一，或勾选「显示机械臂 URDF」');
			return;
		}
		if (mk && typeof ROS3D.MarkerClient === 'undefined') {
			alert('已填写 Marker 话题但未加载 ROS3D.MarkerClient（需 ros3d.min.js）');
			return;
		}
		if (wantUrdf && !pcn && !sn && !mk) {
			if (typeof ROS3D.Urdf !== 'function' || typeof ROSLIB.UrdfModel !== 'function') {
				alert('当前脚本缺少 ROS3D.Urdf / ROSLIB.UrdfModel，无法显示机械臂；请更新 ros3d / roslib');
				return;
			}
		}
		stop3d();
		if (typeof ROS3D === 'undefined' || typeof ROSLIB === 'undefined' || typeof ROSLIB.ROS2TFClient !== 'function') {
			alert('ROS3D 或官方 roslib@2 未加载（需 js/vendor/roslib-2.iife.js + three r89 + ros3d，且含 ROS2TFClient）');
			return;
		}
		const host = $('view3d-host');
		const inner = document.createElement('div');
		inner.id = 'view3d-inner';
		host.appendChild(inner);
		const w = host.clientWidth || 800;
		const h = 800;
		viewer3d = new ROS3D.Viewer({
			divID: 'view3d-inner',
			width: w,
			height: h,
			antialias: false,
			background: '#ffffff',
			cameraPose: { x: 3, y: 3, z: 3 },
			cameraZoomSpeed: 0.5
		});
		const axEl = $('view3d-show-axes');
		const grEl = $('view3d-show-grid');
		if (!axEl || axEl.checked) {
			ros3dAxes = new ROS3D.Axes();
			viewer3d.scene.add(ros3dAxes);
		}
		if (!grEl || grEl.checked) {
			ros3dGrid = new ROS3D.Grid({ num_cells: 12, cellSize: 1, color: '#94a3b8' });
			viewer3d.scene.add(ros3dGrid);
		}
		/* ROS 2 tf2 帧名通常无前导斜杠；带 / 时部分栈上 republisher 与 URDF link 不一致会导致整场景 TF 不更新 */
		let fixedFrame = (($('tf-fixed-frame') && $('tf-fixed-frame').value) || 'base_link').trim().replace(/^\/+/, '');
		if (fixedFrame === '') fixedFrame = 'base_link';
		tfClient3d = new ROSLIB.ROS2TFClient({
			ros,
			fixedFrame,
			angularThres: 0.01,
			transThres: 0.01,
			rate: 20.0
		});
		const maxPts = Math.max(1000, parseInt($('pc-max').value, 10) || 12000);
		const pszEl = $('view3d-point-size');
		const ptSize = Math.max(0.01, parseFloat((pszEl && pszEl.value) || '0.05') || 0.05);
		const pcThrottleEl = $('pc-throttle-ms');
		const pcRatioEl = $('pc-msg-ratio');
		const pcThrottleMs = Math.max(0, parseInt((pcThrottleEl && pcThrottleEl.value) || '120', 10) || 0);
		const pcMsgRatio = Math.max(1, parseInt((pcRatioEl && pcRatioEl.value) || '2', 10) || 1);
		installIvgPointCloud2ProcessPatch();
		if (pcn) {
			/* throttle_rate：rosbridge 订阅最小间隔（ms），减轻大图 CPU/带宽；0=最快。messageRatio：隔帧绘制。
			 * __ivgUniformStride：max_pts 下按 stride 均匀覆盖整幅。 */
			const pcPixelSize = Math.max(1, Math.min(20, Math.round(ptSize * 60)));
			ros3dPointCloud2 = new ROS3D.PointCloud2({
				ros,
				tfClient: tfClient3d,
				rootObject: viewer3d.scene,
				topic: pcn,
				max_pts: maxPts,
				messageRatio: pcMsgRatio,
				throttle_rate: pcThrottleMs,
				compression: 'cbor',
				material: { size: pcPixelSize, sizeAttenuation: false, color: 0xffffff },
				colormap(x) {
					const littleE = ros3dPointCloud2 ? ros3dPointCloud2.__ivgPc2Le !== false : true;
					const dt = ros3dPointCloud2 ? ros3dPointCloud2.__ivgRgbFieldDt : 7;
					let c;
					if (dt === 5 || dt === 6) {
						const u = x >>> 0;
						c = new THREE.Color(((u >> 16) & 255) / 255, ((u >> 8) & 255) / 255, (u & 255) / 255);
					} else {
						c = ivgRosPackedRgbFloatToColor(x, littleE);
					}
					return c;
				}
			});
			ros3dPointCloud2.__ivgUniformStride = true;
			ros3dPointCloud2.__ivgPc2Sig = { got: false };
			showView3dPc2Hint(
				host,
				`<strong>加载点云</strong>：首帧大图仍可能需数秒。当前 <strong>节流 ${pcThrottleMs} ms</strong>、<strong>隔帧 ${pcMsgRatio}</strong>；若仍慢请调大节流、订阅 <code>…/points_web</code> 或调低「最大点数」。`
			);
		}
		if (sn) {
			ros3dLaserScan = new ROS3D.LaserScan({
				ros,
				tfClient: tfClient3d,
				rootObject: viewer3d.scene,
				topic: sn,
				max_pts: maxPts,
				material: { size: Math.max(0.01, ptSize * 0.85), color: 0xff4444 }
			});
		}
		if (mk) {
			ros3dMarkerClient = new ROS3D.MarkerClient({
				ros,
				tfClient: tfClient3d,
				topic: mk,
				rootObject: viewer3d.scene,
				lifetime: 0
			});
		}
		if (wantUrdf && typeof ROS3D.Urdf === 'function' && typeof ROSLIB.UrdfModel === 'function') {
			ros3dUrdfRoot = new THREE.Group();
			viewer3d.scene.add(ros3dUrdfRoot);
			const pName = (($('urdf-param') && $('urdf-param').value) || '/robot_state_publisher:robot_description').trim();
			const meshBase = `${window.location.origin}/api/ivg/robot-mesh/`;
			showView3dUrdfHint(
				host,
				`<strong>机械臂 URDF</strong>：正在请求参数 <code>${pName}</code> 并加载网格（多 DAE 时请等待）…`
			);
			ivgAttachUrdfFromRosParam(ros, pName, meshBase, tfClient3d, ros3dUrdfRoot, err => {
				console.error('URDF:', err);
				showView3dUrdfHint(
					host,
					`<strong>机械臂加载失败</strong>：${String(err)}。请用「参数」页或 <code>ros2 param list</code> 核对 <code>节点名:robot_description</code>；并确认浏览器能打开 <code>${meshBase}aubo_description/meshes/…</code>（网关需 source 过工作空间）。`
				);
			});
		}
		if (pcn && ros && ros.isConnected) {
			const pc2FailAfterMs = 45000;
			setTimeout(() => {
				const got = ros3dPointCloud2 && ros3dPointCloud2.__ivgPc2Sig && ros3dPointCloud2.__ivgPc2Sig.got;
				if (!got) {
					const metaT = new ROSLIB.Topic({
						ros,
						name: pcn,
						messageType: 'sensor_msgs/msg/PointCloud2',
						throttle_rate: 0,
						queue_length: 0,
						queue_size: 100
					});
					const hintHost = $('view3d-host');
					metaT.getPublishers(
						pubs => {
							const n = Array.isArray(pubs) ? pubs.length : 0;
							if (!hintHost) return;
							if (n === 0) {
								showView3dPc2Hint(
									hintHost,
									`<strong>点云无数据</strong>：<code>${pcn}</code> 上未发现发布者。请确认相机/深度节点已启动，或在侧栏选择实际在发布的 <code>sensor_msgs/msg/PointCloud2</code> 话题后再「启动 3D」。`
								);
							} else {
								showView3dPc2Hint(
									hintHost,
									`<strong>点云无数据</strong>：检测到 <code>${n}</code> 个发布者，但 <strong>${pc2FailAfterMs / 1000} 秒</strong>内 ros3d 仍未收到帧。请查 rosbridge / 网关日志、<code>max_message_size</code>、CBOR 与 QoS（<a href="https://github.com/RobotWebTools/rosbridge_suite/issues/551" target="_blank" rel="noopener">#551</a>）；或降低相机点云分辨率以减轻单帧体积。`
								);
							}
						},
						err => {
							if (hintHost) {
								showView3dPc2Hint(
									hintHost,
									`<strong>点云无数据</strong>：无法查询发布者（rosapi 不可用或调用失败）。请确认 rosbridge 同进程已加载 <code>rosapi</code>，并查看浏览器控制台与 rosbridge 日志。`
								);
							}
						}
					);
				}
			}, pc2FailAfterMs);
		}
	}

	/** 建立或重建 ROSLIB.Ros：直连 rosbridge WebSocket。 */
	function connect() {
		ivgPorts.clearRosReconnectTimer(rosReconnect);
		const myGen = ivgPorts.bumpRosReconnectGen(rosReconnect);
		/* 先停视图与话题：避免旧 WebSocket 上的订阅与下一轮 ros 实例交错（勿把 stop 绑在 ros.on('close')，重连时旧连接晚到的 close 会误伤新视图） */
		if (ros) {
			try {
				stop2d();
				stop3d();
				unsubscribe();
			} catch (e0) { /* ignore */ }
			try { ros.close(); } catch (e) { /* ignore */ }
		}
		/* 官方 roslib@2：Ros + 异步 connect（见 RobotWebTools/roslibjs packages/roslib） */
		ros = new ROSLIB.Ros();
		ros.on('connection', () => {
			if (myGen !== rosReconnect.gen) return;
			rosReconnect.attempts = 0;
			ivgPorts.clearRosReconnectTimer(rosReconnect);
			syncLocalHostLinks();
			setStatus('已连接 rosbridge（经网关）', true);
			refreshSidebar();
		});
		ros.on('error', err => {
			if (myGen !== rosReconnect.gen) return;
			setStatus('连接错误', false);
		});
		ros.on('close', () => {
			if (myGen !== rosReconnect.gen) return;
			setStatus('已断开', false);
			refreshSidebar();
			ivgPorts.scheduleRosReconnect(rosReconnect, connect, {
				maxAttempts: ROS_RECONNECT_MAX,
				onSchedule(delayMs, attempt, max) {
					setStatus(`已断开：${Math.round(delayMs / 1000)}s 后自动重连（${attempt}/${max}）`, false);
				},
				onExhausted() {
					setStatus('已断开（已达自动重连上限，请点「重连」）', false);
				}
			});
		});
		void ros.connect(ivgPorts.rosbridgeWebSocketUrl()).catch(() => {
			if (myGen !== rosReconnect.gen) return;
			setStatus('连接错误', false);
		});
	}

	document.addEventListener('DOMContentLoaded', () => {
		void (async () => {
			await ivgPorts.loadRuntime();
			const wvEl = $('web-video-port');
			const rt = window.__IVG_RUNTIME || {};
			if (wvEl && !String(wvEl.value || '').trim() && rt.web_video_port != null) {
				wvEl.value = String(rt.web_video_port);
			}
			const wp = new URLSearchParams(window.location.search).get('web_video_port');
			if (wp && wvEl) wvEl.value = wp;

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
				srv.callService(reqObj, r => {
					svcBtn.disabled = false;
					svcBtn.textContent = origLabel;
					$('svc-resp').textContent = IVGTopicsLabRender.safeJson(r, 80000);
				}, e => {
					svcBtn.disabled = false;
					svcBtn.textContent = origLabel;
					$('svc-resp').textContent = `错误: ${e}`;
				});
			};
		}
			bindIvgBar();
			ivgPorts.wireOnlineRosReconnect(rosReconnect, connect);
			connect();
		})();
	});
})();
