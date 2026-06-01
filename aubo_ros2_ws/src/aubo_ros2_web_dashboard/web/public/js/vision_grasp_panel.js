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
import { ros } from './core/ros.js';
import * as ROSLIB from 'roslib';
import * as ROS3D from 'ros3d';
import { createDomCache } from './core/dom_cache.js';
import { IvgRos3dView3dSession } from './view3d/session.js';
import { createVisionSettingsController } from './vision_grasp/ui_settings.js';
import { createVisionUrdfPanel } from './vision_grasp/urdf_panel.js';
import { createProjectionOverlayController } from './vision_grasp/projection_overlay.js';
import { createJointChartController } from './components/joint-chart.js';
import { createVisionServiceActions } from './vision_grasp/services.js';
import { createVisionUiBinder } from './vision_grasp/ui_binder.js';
import { bindVisionSubscriptions } from './vision_grasp/subscription_binder.js';
import { createVisionModeController } from './vision_grasp/mode_controller.js';
import { canonicalRosTopic, rosMsgArrayField } from './core/utils.js';
import { logBus } from './core/log-bus.js';
import { createMonitoringCollapse } from './components/monitoring-collapse.js';
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
} from './components/pose-card.js';

(() => {
	if (!ROSLIB || !ROS3D || !IvgRos3dView3dSession) {
		throw new Error('vision_grasp_panel.js requires ROSLIB/ROS3D and IvgRos3dView3dSession');
	}
	const PAGE_STREAM_SUFFIX = `${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 8)}`;
	const subs = {};
	/** 左栏 URDF：独立 3D 会话控制器，避免与控制面 WS 生命周期互相干扰。 */
	const visionUrdfPanel =
		typeof createVisionUrdfPanel === 'function'
			? createVisionUrdfPanel({
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
	/** 监控区折叠组件（共享，替代原来 ~80 行重复代码）喵~ */
	const monitoringCollapse = createMonitoringCollapse({ getById: $, jointChart });

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

	const normalizeIvgTopic = canonicalRosTopic;

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
		unsubscribeAll();
		ros.pause();
		setConnStatus('页面后台已暂停', null);
	}

	function resumeRealtimeFromForeground() {
		if (!pageRealtimePaused) return;
		pageRealtimePaused = false;
		ros.resume();
		connect();
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
		if (visionUrdfPanel) visionUrdfPanel.start($, ivgTransport.ros);
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

	function unsubscribeAll(skipTopicUnsub) {
		stopVisionUrdf3d();
		if (graspColorSnapTimer) {
			clearTimeout(graspColorSnapTimer);
			graspColorSnapTimer = null;
		}
		robotPoseCache.value = '';
		ivgTransport.clearRosHandlersByOwner('vision_grasp');
		if (!skipTopicUnsub) {
			ivgTransport.unsubscribeAll();
		}
		Object.keys(subs).forEach(k => {
			subs[k] = null;
		});
		resetCameraDisplay();
		resetResultPanel();
		resetJointChart();
	}

	/** 按输入框话题名建立全部订阅；connect 成功时调用。
	 *  跳过 topic 级 unsubscribeAll，避免销毁状态栏等系统组件的订阅喵~ */
	function startSubscriptions() {
		unsubscribeAll(true);
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
		// 重新订阅 /tool_changer_status（unsubscribeAll 清除了初始订阅，必须重建）喵~
		_toolSubDone = true;  // 接管订阅，阻止旧重试循环
		try {
			ivgTransport.subscribe({ topic: '/tool_changer_status', msgType: 'ivg_interfaces/msg/ToolChangerStatus', maxHz: 5 });
			ivgTransport.onRosJson('/tool_changer_status', _onToolStatusMsg, 'vision_grasp');
		} catch (_e) { logBus.addLog('warn', 'topic', '/tool_changer_status 重订阅失败: ' + (_e.message || _e)); }
	}

	function logSvc(msg) {
		// 转发到统一日志总线，由 setupVisionLogDisplay() 渲染到 #svc-log
		const ts = new Date().toLocaleTimeString();
		const isErr = msg.indexOf('错误') !== -1 || msg.indexOf('失败') !== -1;
		logBus.addLog(isErr ? 'error' : 'info', 'service', msg, { module: 'vision_grasp' }, 'vision_grasp');
	}

	// ── 视觉抓取操作日志显示 ──────────────────────────────────────────────────
	function setupVisionLogDisplay() {
		const host = $('svc-log');
		if (!host) return;

		// 把纯 div 改造为结构化日志框（不改 HTML 布局）
		host.innerHTML = '';
		host.className = 'svc-log svc-log--structured';

		const head = document.createElement('div');
		head.className = 'svc-log__head';
		head.innerHTML = '<span class="svc-log__title">操作日志</span>';

		const clearBtn = document.createElement('button');
		clearBtn.type = 'button';
		clearBtn.className = 'svc-log__clear';
		clearBtn.textContent = '清空';
		head.appendChild(clearBtn);

		const body = document.createElement('div');
		body.className = 'svc-log__body';

		host.appendChild(head);
		host.appendChild(body);

		const MAX_LINES = 150;

		function _timeStr() {
			return new Date().toLocaleTimeString('zh-CN', { hour12: false });
		}

		function _phaseBadge(phase) {
			if (!phase) return '';
			var label = phase === 'in_progress' ? '进行中' :
			            phase === 'completed' ? '完成' :
			            phase === 'failed' ? '失败' : phase === 'start' ? '开始' :
			            phase === 'blocked' ? '阻塞' : phase === 'skipped' ? '跳过' :
			            phase === 'stop' ? '停止' : phase === 'home' ? '归零' : '';
			return '<span class="svc-log-line__phase svc-log-line__phase--' + phase + '">' + (label || phase) + '</span>';
		}

		logBus.onLog(function (entry) {
			if (!entry || !entry.meta || entry.meta.module !== 'vision_grasp') return;

			var line = document.createElement('div');
			var levelCls = '';
			if (entry.level === 'error') levelCls = ' svc-log-line--error';
			else if (entry.level === 'warn') levelCls = ' svc-log-line--warn';
			if (entry.meta && entry.meta.success === true) levelCls += ' svc-log-line--ok';
			if (entry.meta && entry.meta.success === false) levelCls += ' svc-log-line--error';

			line.className = 'svc-log-line' + levelCls;

			var timeHtml = '<span class="svc-log-line__time">' + _timeStr() + '</span>';
			var badgeHtml = _phaseBadge(entry.meta?.phase);
			var msgHtml = '<span class="svc-log-line__msg">' +
				(entry.msg || '').replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;') +
				'</span>';

			line.innerHTML = timeHtml + badgeHtml + msgHtml;
			body.appendChild(line);

			while (body.children.length > MAX_LINES) {
				body.removeChild(body.firstChild);
			}
			body.scrollTop = body.scrollHeight;
		});

		clearBtn.onclick = function () {
			body.innerHTML = '';
		};
	}

	/** 连接/重连 — 统一委托 ros.js 喵~ */
	function connect() {
		if (pageShouldPauseRealtime()) {
			pageRealtimePaused = true;
			setConnStatus('页面后台已暂停', null);
			return;
		}
		pageRealtimePaused = false;
		ros.connect();
	}

	/** 按抓取方式切换：控制区 + 右侧状态（VPE / GraspNet 候选二选一）；末端位姿与夹爪快换始终显示 */
	function syncModeUi() {
		modeController.syncModeUi();
	}

	// 监控区折叠/展开 → 改用共享组件 monitoring-collapse.js 喵~
	// scheduleSyncMonitoringBundleMinHeight 已委托给 monitoringCollapse.scheduleSyncMinHeight()

	// ═══════════════════════════════════════════════════════════════
	// 夹爪快换状态桥接 — UI 状态栏 + 按钮状态 + 初始化面板 + localStorage
	// 三层初始化策略:
	//   1. localStorage → 立即占位 UI
	//   2. /get_current_tool (3s 超时) → 查询后端当前工具
	//   3. /tool_changer_status 订阅 → 实时更新
	//   4. 8s 后仍未检测到 → 显示手动选择面板
	// ═══════════════════════════════════════════════════════════════
	const LS_TOOL_KEY = 'ivg_last_tool_id';
	const TOOL_NAMES = { gripper0: '气动夹爪 φ40', gripper1: '电动夹爪 A', gripper2: '电动夹爪 φ60' };
	var _toolSwapping = false;
	var _initDone = false;    // 初始化是否已结束（收到 topic 或超时后置 true）
	var _initTimer = null;    // 8s 超时定时器
	var _toolSubDone = false; // 防 _subscribeToolStatus 与 startSubscriptions 重复订阅

	function _hideInitBanner() {
		var banner = $('tool-init-banner');
		if (banner) banner.hidden = true;
	}

	function _showInitBanner() {
		if (_initDone) return;
		var banner = $('tool-init-banner');
		if (banner) banner.hidden = false;
	}

	function _finalizeInit() {
		_initDone = true;
		if (_initTimer) { clearTimeout(_initTimer); _initTimer = null; }
	}

	function _updateToolStatusUI(toolId, connected) {
		var led = $('tool-status-led');
		var label = $('tool-status-label');
		var params = $('tool-status-params');
		var name = TOOL_NAMES[toolId] || toolId || '';

		if (led) {
			led.classList.remove('connected', 'loading');
			if (_toolSwapping) led.classList.add('loading');
			else if (connected) led.classList.add('connected');
		}
		if (label) label.textContent = _toolSwapping ? ('切换中: ' + (name || toolId || '...')) : (name || toolId || '未安装工具');
		if (params) params.textContent = toolId || '';
	}

	function _updateGripperButtons(currentId, swapping) {
		var btns = document.querySelectorAll('.gripper-btn');
		btns.forEach(function (btn) {
			var tool = btn.getAttribute('data-tool');
			btn.classList.remove('active', 'swapping');
			btn.disabled = false;
			if (swapping && tool === swapping) {
				// 目标工具 → swapping 动画 + 禁用
				btn.classList.add('swapping'); btn.disabled = true;
			} else if (swapping && currentId && tool === currentId) {
				// 源工具 → 保持 active 标记 + 禁用（防止误点击）
				btn.classList.add('active'); btn.disabled = true;
			} else if (!swapping && currentId && tool === currentId) {
				// 正常状态 → active + 禁用
				btn.classList.add('active'); btn.disabled = true;
			}
		});
	}

	function _onToolStatusMsg(msg) {
		if (!msg) return;
		var toolId = String(msg.tool_id || '');
		var connected = msg.is_connected !== false;
		var hasTool = connected && toolId;
		// 后端确认有工具 → 信任硬件，更新 localStorage 和下拉框
		// 后端无工具 → 保留用户手动设置，不覆盖 localStorage 喵~
		if (hasTool) {
			if (typeof serviceActions.setCurrentToolId === 'function') serviceActions.setCurrentToolId(toolId);
			try { localStorage.setItem(LS_TOOL_KEY, toolId); } catch (e) {}
			var sel = $('init-tool-select');
			if (sel) { try { sel.value = toolId; } catch (e) {} }
		}
		// 有效工具 ID：后端有工具 → 信任硬件，否则保留用户手动设置喵~
		var effectiveId = hasTool ? toolId : (
			typeof serviceActions.getCurrentToolId === 'function' ? serviceActions.getCurrentToolId() : ''
		);
		// 快换进行中时不重置 _toolSwapping（防止中间态"无工具"消息打断 UI）喵~
		var inFlight = typeof serviceActions.isSwapInFlight === 'function' && serviceActions.isSwapInFlight();
		if (!inFlight) { _toolSwapping = false; }
		// 用有效工具更新全部 UI，避免后端空消息冲掉用户手动设置喵~
		_updateToolStatusUI(effectiveId, hasTool ? connected : !!effectiveId);
		_updateGripperButtons(effectiveId, null);
		// 后端确认工具已连接 → 隐藏初始化面板 + 清除手动设定标记
		if (hasTool) {
			_finalizeInit();
			_hideInitBanner();
			var hint = $('tool-init-hint');
			if (hint) hint.hidden = true;
		}
	}

	function _callGetCurrentToolAndSubscribe() {
		// 调用 /get_current_tool（3s 超时）
		if (typeof serviceActions.callGetCurrentTool === 'function') {
			var getToolCalled = false;
			var timeoutId = setTimeout(function () {
				if (getToolCalled) return;
				getToolCalled = true;
			}, 3000);
			serviceActions.callGetCurrentTool(function (err, r) {
				if (getToolCalled) return;
				getToolCalled = true;
				clearTimeout(timeoutId);
				if (!err && r && r.success && r.tool_id && r.is_connected) {
					var toolId = String(r.tool_id);
					try { localStorage.setItem(LS_TOOL_KEY, toolId); } catch (e) {}
					if (typeof serviceActions.setCurrentToolId === 'function') serviceActions.setCurrentToolId(toolId);
					_updateToolStatusUI(toolId, true);
					_updateGripperButtons(toolId, null);
					var sel = $('init-tool-select');
					if (sel) { try { sel.value = toolId; } catch (e) {} }
					var hint3 = $('tool-init-hint');
					if (hint3) hint3.hidden = true;
					_finalizeInit();
					_hideInitBanner();
				}
			});
		}

		// 订阅 /tool_changer_status（仅初始化阶段；重连时由 startSubscriptions 负责）
		function _subscribeToolStatus() {
			if (_toolSubDone) return;  // startSubscriptions 已接管
			if (!ivgTransport || !ivgTransport.isConnected()) {
				setTimeout(_subscribeToolStatus, 1000);
				return;
			}
			_toolSubDone = true;
			try {
				ivgTransport.subscribe({ topic: '/tool_changer_status', msgType: 'ivg_interfaces/msg/ToolChangerStatus', maxHz: 5 });
				ivgTransport.onRosJson('/tool_changer_status', _onToolStatusMsg, 'vision_grasp');
			} catch (e) { logBus.addLog('warn', 'topic', '/tool_changer_status 订阅失败: ' + (e.message || e)); }
		}
		setTimeout(_subscribeToolStatus, 500);

		// 8s 总超时 → 仍未检测到已连接工具则显示手动选择面板
		_initTimer = setTimeout(function () {
			if (_initDone) return;
			var currentId = '';
			if (typeof serviceActions.getCurrentToolId === 'function') currentId = serviceActions.getCurrentToolId();
			if (currentId) {
				_finalizeInit();
				return;
			}
			_finalizeInit();
			_showInitBanner();
		}, 8000);
	}

	function _initToolStatusBridge() {
		// 第一步：localStorage → UI 占位（标记为手动设定）
		var stored = '';
		try { stored = localStorage.getItem(LS_TOOL_KEY) || ''; } catch (e) {}
		if (stored) {
			_updateToolStatusUI(stored, false);
			_updateGripperButtons(stored, null);
			if (typeof serviceActions.setCurrentToolId === 'function') serviceActions.setCurrentToolId(stored);
			var si = $('init-tool-select');
			if (si) { try { si.value = stored; } catch (e) {} }
			var hint = $('tool-init-hint');
			if (hint) hint.hidden = false;
		}

		// init-tool-select → localStorage（可见配置，手动设定）
		var sel = $('init-tool-select');
		if (sel) {
			sel.addEventListener('change', function () {
				var v = sel.value || '';
				try { localStorage.setItem(LS_TOOL_KEY, v); } catch (e) {}
				if (typeof serviceActions.setCurrentToolId === 'function') serviceActions.setCurrentToolId(v);
				_updateToolStatusUI(v, false);
				_updateGripperButtons(v, null);
				// 标记为手动设定
				var hint = $('tool-init-hint');
				if (hint) hint.hidden = false;
			});
		}

		// 按钮点击 → swapping 状态（源工具保持 active+禁用，目标显示 swapping）
		var swapRow = document.getElementById('gripper-swap-btns');
		if (swapRow) {
			swapRow.addEventListener('click', function (e) {
				var btn = e.target.closest('.gripper-btn');
				if (!btn || btn.disabled) return;
				var targetId = btn.getAttribute('data-tool');
				if (!targetId) return;
				// 防重入检查
				if (typeof serviceActions.isSwapInFlight === 'function' && serviceActions.isSwapInFlight()) {
					logBus.addLog('warn', 'service', '快换进行中，忽略重复点击', { module: 'vision_grasp' }, 'vision_grasp');
					return;
				}
				// 同工具跳过
				var currentId = typeof serviceActions.getCurrentToolId === 'function' ? serviceActions.getCurrentToolId() : '';
				if (currentId === targetId) return;
				_toolSwapping = true;
				_updateToolStatusUI(targetId, false);
				_updateGripperButtons(currentId, targetId);  // currentId=源工具保持禁用
			}, true);
		}

		// 手动初始化面板按钮 → localStorage + UI 更新 + 隐藏面板
		var initActions = $('tool-init-actions');
		if (initActions) {
			initActions.addEventListener('click', function (e) {
				var btn = e.target.closest('.tool-init-btn');
				if (!btn) return;
				var toolId = btn.getAttribute('data-init-tool') || '';
				try { localStorage.setItem(LS_TOOL_KEY, toolId); } catch (e) {}
				if (typeof serviceActions.setCurrentToolId === 'function') serviceActions.setCurrentToolId(toolId);
				_updateToolStatusUI(toolId, false);
				_updateGripperButtons(toolId || '', null);
				var si = $('init-tool-select');
				if (si) { try { si.value = toolId || ''; } catch (e) {} }
				var hint2 = $('tool-init-hint');
				if (hint2) hint2.hidden = false;
				_hideInitBanner();
				logBus.addLog('info', 'system', '手动选择初始工具: ' + (toolId || '无工具'), { module: 'vision_grasp',  source: 'gripper_init', tool_id: toolId }, 'vision_grasp');
			});
		}

		// 启动三层初始化
		_callGetCurrentToolAndSubscribe();
	}

	document.addEventListener('DOMContentLoaded', () => {
		void (async () => {
			await ivgPorts.loadRuntime();
			loadTopicsFromStorage();

			jointChart.observeResize();
			monitoringCollapse.bindEvents();
			monitoringCollapse.scheduleSyncMinHeight();
			requestAnimationFrame(() => monitoringCollapse.scheduleSyncMinHeight());
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

			// --- 操作日志显示 ---
			setupVisionLogDisplay();

			// --- 夹爪 swap 完成回调（服务响应后立即重置 UI，不等 topic） ---
			if (typeof serviceActions.onGripperSwapDone === 'function') {
				serviceActions.onGripperSwapDone(function (_err) {
					_toolSwapping = false;
					var curId = typeof serviceActions.getCurrentToolId === 'function' ? serviceActions.getCurrentToolId() : '';
					_updateToolStatusUI(curId, false);
					_updateGripperButtons(curId, null);
				});
			}

			// --- 夹爪状态桥接 ---
			_initToolStatusBridge();

			uiBinder.bindTopicSettingsUi();

			// 统一使用 ros.js 管理连接生命周期喵~
			ros.onStatusChange(setConnStatus);
			ros.onConnected(function () {
				logBus.addLog('info', 'rosbridge', 'rosbridge 已连接，启动订阅');
				startSubscriptions();
			});
			ros.onCleanup(function () { unsubscribeAll(); });
			ros.wireOnlineReconnect();
			connect();
		})();
	});
})();
