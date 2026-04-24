/**
 * 视觉抓取面板（ES module 单页编排脚本）
 *
 * 架构分层：
 * - 传输：``ivgPorts``（runtime/WS URL/重连）、``ivgTransport``（控制面 JSON + 服务调用）。
 * - 视图子模块：
 *   - ``createVisionUrdfPanel``：左栏 URDF 会话生命周期
 *   - ``createProjectionOverlayController``：抓取投影叠加绘制
 *   - ``createJointChartController``：关节曲线与图例
 *   - ``createVisionServiceActions``：服务调用与控制按钮绑定
 *   - ``createVisionUiBinder``：模式切换与话题设置弹窗绑定
 *   - ``pose_card``：末端位姿 / 抓取位姿 HTML 渲染
 * - 本文件职责：订阅编排、连接状态、模式切换、页面生命周期。
 *
 * 数值摘要由桥进程 ``ivg_display`` 等字段提供；不在此做格式编排/曲线拟合/抓取轮询。
 */
import { ivgPorts } from './ivg_runtime.js';
import { ivgTransport } from './ivg_transport.js';
import * as ROSLIB from 'roslib';
import * as ROS3D from 'ros3d';
import { createDomCache } from './core/dom_cache.js';
import { IvgRos3dView3dSession } from './view3d/session.js';
import { createVisionSettingsController } from './vision_grasp/ui_settings.js';
import { createVisionUrdfPanel } from './vision_grasp/urdf_panel.js';
import { createProjectionOverlayController } from './vision_grasp/projection_overlay.js';
import { createJointChartController } from './vision_grasp/joint_chart.js';
import { createVisionServiceActions } from './vision_grasp/services.js';
import { createVisionUiBinder } from './vision_grasp/ui_binder.js';
import { bindVisionSubscriptions } from './vision_grasp/subscription_binder.js';
import { createVisionModeController } from './vision_grasp/mode_controller.js';
import {
	VISION_SETTINGS_DEFAULTS,
	VISION_ALL_SETTING_IDS,
	VISION_TOPIC_IDS,
	VISION_TF_TOPIC_IDS,
	VISION_SERVICE_IDS,
	VISION_TOPIC_TYPE_MAP,
	VISION_SERVICE_TYPE_MAP,
	VISION_FIXED_SERVICE_TYPES
} from './vision_grasp/config.js';
import {
	escapeHtml,
	robotPoseHtmlIsRenderable,
	formatRobotPoseHtml,
	formatFinalGraspPoseHtml
} from './vision_grasp/pose_card.js';

(() => {
	if (!ROSLIB || !ROS3D || !IvgRos3dView3dSession) {
		throw new Error('vision_grasp_panel.js requires ROSLIB/ROS3D and IvgRos3dView3dSession');
	}
	const rosReconnect = ivgPorts.createRosReconnectState();
	const ROS_RECONNECT_MAX = 12;
	let connectInFlight = false;
	const PAGE_STREAM_SUFFIX = `${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 8)}`;
	const subs = {};
	/** 左栏 URDF：独立 3D 会话控制器，避免与控制面 WS 生命周期互相干扰。 */
	const visionUrdfPanel =
		typeof createVisionUrdfPanel === 'function'
			? createVisionUrdfPanel({
				ports: ivgPorts,
				SessionCtor: IvgRos3dView3dSession,
				documentRef: document
			})
			: null;
	/** 仅 rosbridge 订阅类话题 */
	const TOPIC_IDS = VISION_TOPIC_IDS;
	const TF_TOPIC_IDS = VISION_TF_TOPIC_IDS;
	/** 按钮触发的服务名（std_srvs/SetBool 或 demo_interface 自定义） */
	const SERVICE_IDS = VISION_SERVICE_IDS;
	/** 工件模式「执行单次抓取」固定调用的服务（不设为可选项，与后端 launch 一致） */
	const SVC_EXECUTE_SINGLE_GRASP = '/execute_single_grasp';
	const ALL_SETTING_IDS = VISION_ALL_SETTING_IDS;
	const SETTINGS_DEFAULTS = VISION_SETTINGS_DEFAULTS;
	/** v3：含 TF 与按钮服务名；v2 仅订阅话题 */
	const TOPIC_STORAGE_KEY = 'ivg_vision_grasp_topics_v3';
	const MONITORING_COLLAPSED_KEY = 'ivg_vision_monitoring_collapsed';
	let monitoringBundleMinRaf = 0;
	let monitoringPoseResizeObserver = null;
	let pageRealtimePaused = false;
	/** AI/GraspNet：左图与投影底图用单帧 JPEG；抓取话题到达后防抖再各刷一帧，避免 MJPEG 卡顿 */
	let graspColorSnapTimer = null;
	const GRASP_COLOR_SNAP_DEBOUNCE_MS = 400;
	/** 共享 DOM 缓存：不缓存 null，避免页面尚未完成渲染时形成永久未命中。 */
	const $ = typeof createDomCache === 'function'
		? createDomCache(document)
		: function fallbackGetById(id) {
			return document.getElementById(id);
		};
	const projectionOverlay = createProjectionOverlayController({
		getById: $,
		defaults: SETTINGS_DEFAULTS,
		getColorTopic: () => ($('topic-color') && $('topic-color').value.trim()) || SETTINGS_DEFAULTS['topic-color']
	});
	const jointChart = createJointChartController({
		getById: $,
		maxSamples: 280,
		lineColors: ['#2563eb', '#16a34a', '#d97706', '#db2777', '#7c3aed', '#0d9488']
	});
	const serviceActions = createVisionServiceActions({
		transport: ivgTransport,
		log: logSvc,
		getById: $,
		getSetting,
		useVisualFromMode,
		executeSingleService: SVC_EXECUTE_SINGLE_GRASP,
		serviceTypeMap: VISION_SERVICE_TYPE_MAP,
		fixedServiceTypes: VISION_FIXED_SERVICE_TYPES
	});
	const uiBinder = createVisionUiBinder({
		getById: $,
		onModeChanged: syncModeUi,
		openTopicSettingsModal,
		closeTopicSettingsModal,
		applyTopicDefaultsToDom,
		clearTopicsStorage,
		saveTopicsToStorage,
		connect,
		logSvc,
		topicSettingsModalOpen,
		scheduleProjectionDraw
	});
	const modeController = createVisionModeController({
		getById: $,
		setResultPanelMode,
		isConnected: () => ivgTransport.isConnected(),
		startSubscriptions
	});

	/*
	 * 本文件内局部函数按主题分组（便于跳转；各 function 建议配合 IDE 大纲阅读）：
	 * - 话题/TF 工具：normalizeIvgTopic … buildCameraInfoTopic
	 * - 2D 投影与 TF 图：vision_grasp/projection_overlay.js（复用 view3d/tf_clients.js 中 ivgFindRelativeTransform 等）
	 * - 投影叠加：resetProjectionState … scheduleProjectionDraw
	 * - 设置持久化：getSetting … clearTopicsStorage
	 * - 模态与连接：topicSettingsModalOpen … connect / syncModeUi
	 * - 关节图与快照：resetJointChart … scheduleGraspColorSnapshotRefresh
	 * - 订阅与服务：unsubscribeAll … callGripperSwap
	 */

	function normalizeIvgTopic(t) {
		const s = String(t || '').trim();
		if (!s) return '';
		return s.startsWith('/') ? s : `/${s}`;
	}

	function buildPageStreamId(prefix) {
		return `${prefix}_${PAGE_STREAM_SUFFIX}`;
	}

	function buildCameraInfoTopic(colorTopic) {
		const t = normalizeIvgTopic(colorTopic || SETTINGS_DEFAULTS['topic-color']);
		if (!t) return '/camera/color/camera_info';
		if (/\/camera_info$/.test(t)) return t;
		if (/\/image(_raw|_color)?$/.test(t)) return t.replace(/\/image(_raw|_color)?$/, '/camera_info');
		return `${t.replace(/\/+$/, '')}/camera_info`;
	}

	function resetProjectionState() {
		projectionOverlay.resetState();
	}

	function clearProjectionCanvas() {
		projectionOverlay.clearCanvas();
	}

	function scheduleProjectionDraw() {
		projectionOverlay.scheduleDraw();
	}

	/** 读 DOM 或回退默认；用于订阅话题、TF、服务名 */
	function getSetting(id) {
		return sanitizeTopicValue(id, $(id) ? $(id).value : '');
	}

	function sanitizeTopicValue(id, value) {
		const fallback =
			SETTINGS_DEFAULTS[id] !== undefined ? String(SETTINGS_DEFAULTS[id]) : '';
		let raw = String(value == null ? '' : value).trim();
		if (id === 'topic-result') return raw;
		if (!raw) raw = fallback.trim();
		if (!raw) return '';
		const isNamedTopicOrSvc =
			id.startsWith('topic-') || id.startsWith('svc-');
		if (!isNamedTopicOrSvc) return raw;
		if (raw.indexOf('__ivg_disabled') !== -1) return fallback;
		return normalizeIvgTopic(raw);
	}

	function sanitizeTopicConfig(obj) {
		const out = {};
		ALL_SETTING_IDS.forEach(id => {
			out[id] = sanitizeTopicValue(id, obj && obj[id]);
		});
		return out;
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

	/** 上一次成功渲染的末端位姿 HTML（静止或单帧缺字段时沿用，避免闪回「等待数据」） */
	const robotPoseCache = { value: '' };

	const settingsController =
		typeof createVisionSettingsController === 'function'
			? createVisionSettingsController({
				getById: $,
				allIds: ALL_SETTING_IDS,
				defaults: SETTINGS_DEFAULTS,
				sanitizeTopicValue,
				sanitizeTopicConfig,
				storageKey: TOPIC_STORAGE_KEY,
				documentRef: document
			})
			: null;

	function applyTopicDefaultsToDom() {
		if (settingsController) settingsController.applyDefaultsToDom();
	}

	function readTopicsFromDom() {
		if (settingsController) return settingsController.readFromDom();
		return {};
	}

	function loadTopicsFromStorage() {
		return settingsController ? settingsController.loadFromStorage() : false;
	}

	function saveTopicsToStorage() {
		if (settingsController) settingsController.saveToStorage();
	}

	function clearTopicsStorage() {
		if (settingsController) settingsController.clearStorage();
	}

	function topicSettingsModalOpen() {
		return settingsController ? settingsController.modalOpen() : false;
	}

	function openTopicSettingsModal() {
		if (settingsController) settingsController.openModal();
	}

	function closeTopicSettingsModal() {
		if (settingsController) settingsController.closeModal();
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
		if (canvas) canvas.hidden = true;
	}

	function stopVisionUrdf3d() {
		if (visionUrdfPanel) visionUrdfPanel.stop();
	}

	function layoutVisionUrdfViewer() {
		if (visionUrdfPanel) visionUrdfPanel.layout();
	}

	function startVisionUrdf3d() {
		if (visionUrdfPanel) visionUrdfPanel.start($);
	}

	function setResultPanelMode(mode) {
		projectionOverlay.setMode(mode);
	}

	function resetResultPanel() {
		const stack = $('result-viz-stack');
		if (stack) {
			stack.style.transform = '';
		}
		setResultPanelMode('workpiece');
		resetProjectionState();
		const resultImageEl = $('result-mjpeg');
		if (resultImageEl) {
			if (resultImageEl._ivgMjpegRecoverCleanup) {
				resultImageEl._ivgMjpegRecoverCleanup();
				resultImageEl._ivgMjpegRecoverCleanup = null;
			}
			resultImageEl.removeAttribute('src');
			resultImageEl.hidden = true;
		}
		clearProjectionCanvas();
	}

	function resetJointChart() {
		jointChart.reset();
	}

	function pushJointStateSample(msg) {
		const rawNames = rosMsgArrayField(msg, 'name').map(x =>
			x != null && x !== '' ? String(x) : ''
		);
		const pos = rosMsgArrayField(msg, 'position').map(x => Number(x));
		jointChart.pushSample(rawNames, pos);
	}

	function setAiColorSnapshotImg(img, url, colorTopic, streamIdForFallback) {
		if (!img) return;
		function fallback() {
			if (!($('mode-graspnet') && $('mode-graspnet').checked)) return;
			img.src = ivgTransport.cameraStreamUrl(colorTopic, streamIdForFallback);
		}
		img.addEventListener('error', fallback, { once: true });
		img.src = url;
	}

	function refreshAiGraspnetColorImages(reason) {
		const graspnetMode = !!($('mode-graspnet') && $('mode-graspnet').checked);
		if (!graspnetMode) return;
		const colorTopic = ($('topic-color') && $('topic-color').value.trim()) || SETTINGS_DEFAULTS['topic-color'];
		const snapFn =
			typeof ivgTransport.cameraSnapshotUrl === 'function'
				? (topic, sid) => ivgTransport.cameraSnapshotUrl(topic, sid)
				: (topic, sid) => ivgTransport.cameraStreamUrl(topic, sid);
		const ts = Date.now();
		const camMjpeg = $('cam-mjpeg');
		const sidCam = `${buildPageStreamId('vision_color')}_${reason}_${ts}`;
		const sidProj = `${buildPageStreamId('vision_projection')}_${reason}_${ts}`;
		if (camMjpeg && !camMjpeg.hidden) {
			setAiColorSnapshotImg(camMjpeg, snapFn(colorTopic, sidCam), colorTopic, buildPageStreamId('vision_color'));
		}
		const resultImageEl = $('result-mjpeg');
		if (resultImageEl && !resultImageEl.hidden) {
			setAiColorSnapshotImg(resultImageEl, snapFn(colorTopic, sidProj), colorTopic, buildPageStreamId('vision_projection'));
		}
	}

	function scheduleGraspColorSnapshotRefresh() {
		const graspnetMode = !!($('mode-graspnet') && $('mode-graspnet').checked);
		if (!graspnetMode) return;
		if (graspColorSnapTimer) clearTimeout(graspColorSnapTimer);
		graspColorSnapTimer = setTimeout(() => {
			graspColorSnapTimer = null;
			refreshAiGraspnetColorImages('grasp');
		}, GRASP_COLOR_SNAP_DEBOUNCE_MS);
	}

	function unsubscribeAll() {
		stopVisionUrdf3d();
		if (graspColorSnapTimer) {
			clearTimeout(graspColorSnapTimer);
			graspColorSnapTimer = null;
		}
		robotPoseCache.value = '';
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
		bindVisionSubscriptions({
			transport: ivgTransport,
			getById: $,
			subs,
			defaults: SETTINGS_DEFAULTS,
			getSetting,
			normalizeIvgTopic,
			buildCameraInfoTopic,
			buildPageStreamId,
			setResultPanelMode,
			refreshAiGraspnetColorImages,
			escapeHtml,
			formatRobotPoseHtml,
			robotPoseHtmlIsRenderable,
			formatFinalGraspPoseHtml,
			projectionOverlay,
			scheduleGraspColorSnapshotRefresh,
			scheduleProjectionDraw,
			startVisionUrdf3d,
			pushJointStateSample,
			robotPoseCache,
			topicTypeMap: VISION_TOPIC_TYPE_MAP
		});
	}

	function logSvc(msg) {
		const el = $('svc-log');
		if (el) el.textContent = `${new Date().toLocaleTimeString()} ${msg}`;
	}

	/** 连接 ivg_web_serve 控制面 WebSocket；成功后 startSubscriptions */
	function connect() {
		if (connectInFlight) return;
		if (pageShouldPauseRealtime()) {
			pageRealtimePaused = true;
			setConnStatus('页面后台已暂停', null);
			return;
		}
		connectInFlight = true;
		pageRealtimePaused = false;
		ivgPorts.clearRosReconnectTimer(rosReconnect);
		const myGen = ivgPorts.bumpRosReconnectGen(rosReconnect);
		setConnStatus('正在连接…', null);
		ivgTransport.close();
		void (async () => {
			try {
				await ivgTransport.loadRuntime();
				await ivgTransport.connectControl();
				if (myGen !== rosReconnect.gen) return;
				rosReconnect.attempts = 0;
				ivgPorts.clearRosReconnectTimer(rosReconnect);
				ivgTransport.clearControlJsonHandlers();
				ivgTransport.onControlJson(o => {
					if (!o || typeof o !== 'object') return;
					if (o.op === 'error') {
						logSvc(o.message != null ? String(o.message) : '通信错误');
						setConnStatus('已连接但发生通信错误，准备重连…', false);
					}
					if (o.op === 'close') {
						setConnStatus('连接已断开，准备重连…', false);
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
				});
				setConnStatus('已连接', true);
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
			} finally {
				connectInFlight = false;
			}
		})();
	}

	/** 按抓取方式切换：控制区 + 右侧状态（VPE / GraspNet 候选二选一）；末端位姿与夹爪快换始终显示 */
	function syncModeUi() {
		modeController.syncModeUi();
	}

	function scheduleSyncMonitoringBundleMinHeight() {
		if (monitoringBundleMinRaf) return;
		monitoringBundleMinRaf = requestAnimationFrame(() => {
			monitoringBundleMinRaf = 0;
			syncMonitoringBundleMinHeight();
		});
	}

	/**
	 * 写入 --ivg-monitoring-bundle-min-px（右列位姿卡 scrollHeight），供底边栏最小高度使用。
	 * 与 PC / iPad 无关：是否采用该变量仅由 vision_grasp_panel.css 的 @media 决定，避免 JS 按视口分叉。
	 */
	function syncMonitoringBundleMinHeight() {
		const section = document.getElementById('layout-monitoring-section');
		const bundle = document.getElementById('layout-monitoring-bundle');
		const poseCol = bundle && bundle.querySelector('.layout-monitoring-pose-col');
		if (!bundle || !poseCol) return;

		if (!section || section.classList.contains('is-monitoring-collapsed')) {
			bundle.style.removeProperty('--ivg-monitoring-bundle-min-px');
			return;
		}

		const intrinsic = Math.ceil(poseCol.scrollHeight);
		const floorPx = 260;
		bundle.style.setProperty('--ivg-monitoring-bundle-min-px', `${Math.max(floorPx, intrinsic)}px`);
	}

	function bindMonitoringBundleMinHeightSync() {
		const bundle = document.getElementById('layout-monitoring-bundle');
		const poseCol = bundle && bundle.querySelector('.layout-monitoring-pose-col');
		if (!poseCol) return;

		if (typeof ResizeObserver !== 'undefined') {
			if (monitoringPoseResizeObserver) monitoringPoseResizeObserver.disconnect();
			monitoringPoseResizeObserver = new ResizeObserver(() => scheduleSyncMonitoringBundleMinHeight());
			monitoringPoseResizeObserver.observe(poseCol);
		}
		window.addEventListener('resize', scheduleSyncMonitoringBundleMinHeight);
		if (document.fonts && typeof document.fonts.ready !== 'undefined' && document.fonts.ready.then) {
			void document.fonts.ready.then(() => scheduleSyncMonitoringBundleMinHeight());
		}
	}

	function bindMonitoringSectionCollapse() {
		const section = document.getElementById('layout-monitoring-section');
		const btn = document.getElementById('btn-monitoring-toggle');
		if (!section || !btn) return;

		function applyCollapsed(collapsed) {
			section.classList.toggle('is-monitoring-collapsed', collapsed);
			btn.setAttribute('aria-expanded', collapsed ? 'false' : 'true');
			const hint = btn.querySelector('.layout-monitoring-toggle__hint');
			if (hint) hint.textContent = collapsed ? '展开' : '收起';
			try {
				localStorage.setItem(MONITORING_COLLAPSED_KEY, collapsed ? '1' : '0');
			} catch (_) {
				/* ignore quota / private mode */
			}
			if (!collapsed) {
				requestAnimationFrame(() => {
					jointChart.observeResize();
					window.dispatchEvent(new Event('resize'));
					scheduleSyncMonitoringBundleMinHeight();
					requestAnimationFrame(() => scheduleSyncMonitoringBundleMinHeight());
				});
			} else {
				const bundleEl = document.getElementById('layout-monitoring-bundle');
				if (bundleEl) bundleEl.style.removeProperty('--ivg-monitoring-bundle-min-px');
			}
		}

		let initialCollapsed = false;
		try {
			initialCollapsed = localStorage.getItem(MONITORING_COLLAPSED_KEY) === '1';
		} catch (_) {
			/* use expanded */
		}
		applyCollapsed(initialCollapsed);

		btn.addEventListener('click', () => {
			applyCollapsed(!section.classList.contains('is-monitoring-collapsed'));
		});
	}

	document.addEventListener('DOMContentLoaded', () => {
		void (async () => {
			await ivgPorts.loadRuntime();
			loadTopicsFromStorage();

			jointChart.observeResize();
			bindMonitoringSectionCollapse();
			bindMonitoringBundleMinHeightSync();
			scheduleSyncMonitoringBundleMinHeight();
			requestAnimationFrame(() => scheduleSyncMonitoringBundleMinHeight());
			window.addEventListener('resize', scheduleProjectionDraw);
			uiBinder.bindResultImageLoad();
			document.addEventListener('visibilitychange', () => {
				if (pageShouldPauseRealtime()) suspendRealtimeForBackground();
				else resumeRealtimeFromForeground();
			});
			window.addEventListener('pagehide', () => {
				suspendRealtimeForBackground();
			});

			uiBinder.bindModeSwitches();
			syncModeUi();

			// --- 抓取区服务按钮 ---
			serviceActions.bindControlButtons();

			uiBinder.bindTopicSettingsUi();

			ivgPorts.wireOnlineRosReconnect(rosReconnect, connect);
			connect();
		})();
	});
})();
