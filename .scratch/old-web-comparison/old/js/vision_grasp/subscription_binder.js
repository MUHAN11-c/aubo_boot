/**
 * 视觉抓取订阅绑定器：
 * - 按当前页面设置建立话题订阅
 * - 管理结果图底图与投影层触发
 * - 更新页面状态区域
 */

function bindVisionSubscriptions(opts) {
	const options = opts || {};
	const transport = options.transport;
	const $ = options.getById || (id => document.getElementById(id));
	const subs = options.subs || {};
	const defaults = options.defaults || {};
	const getSetting = options.getSetting || (() => '');
	const normalizeIvgTopic = options.normalizeIvgTopic || (v => String(v || '').trim());
	const buildCameraInfoTopic = options.buildCameraInfoTopic || (() => '/camera/color/camera_info');
	const buildPageStreamId = options.buildPageStreamId || (prefix => `${prefix}_${Date.now()}`);
	const setResultPanelMode = options.setResultPanelMode || (() => {});
	const refreshAiGraspnetColorImages = options.refreshAiGraspnetColorImages || (() => {});
	const escapeHtml = options.escapeHtml || (v => String(v || ''));
	const formatRobotPoseHtml = options.formatRobotPoseHtml || (() => '');
	const robotPoseHtmlIsRenderable = options.robotPoseHtmlIsRenderable || (() => false);
	const formatFinalGraspPoseHtml = options.formatFinalGraspPoseHtml || (() => '');
	const projectionOverlay = options.projectionOverlay || null;
	const scheduleGraspColorSnapshotRefresh = options.scheduleGraspColorSnapshotRefresh || (() => {});
	const scheduleProjectionDraw = options.scheduleProjectionDraw || (() => {});
	const startVisionUrdf3d = options.startVisionUrdf3d || (() => {});
	const pushJointStateSample = options.pushJointStateSample || (() => {});
	const robotPoseCache = options.robotPoseCache || { value: '' };
	const topicTypeMap = options.topicTypeMap || {};

	const colorTopic = ($('topic-color') && $('topic-color').value.trim()) || defaults['topic-color'];
	const robotTopic = normalizeIvgTopic(getSetting('topic-robot'));
	const jointTopic = normalizeIvgTopic(getSetting('topic-joints'));
	const statusTopic = normalizeIvgTopic(getSetting('topic-vpe-status'));
	const graspTopic = normalizeIvgTopic(getSetting('topic-grasp-poses'));
	const tfTopic = normalizeIvgTopic(getSetting('topic-tf'));
	const tfStaticTopic = normalizeIvgTopic(getSetting('topic-tf-static'));
	const cameraInfoTopic = normalizeIvgTopic(buildCameraInfoTopic(colorTopic));
	const resultTopic = $('topic-result') ? String($('topic-result').value || '').trim() : '';
	const graspnetMode = !!($('mode-graspnet') && $('mode-graspnet').checked);
	const projectionMode = graspnetMode;

	const camCanvasEl = $('cam-canvas');
	const camImageEl = $('cam-mjpeg');
	if (camCanvasEl) camCanvasEl.hidden = true;
	if (camImageEl) {
		camImageEl.hidden = true;
		camImageEl.removeAttribute('src');
	}
	subs.color = true;

	const resultImageEl = $('result-mjpeg');
	const underlayTopic = resultTopic.trim();
	if (graspnetMode) {
		setResultPanelMode('graspnet');
		if (resultImageEl) {
			resultImageEl.hidden = !projectionMode;
			resultImageEl.removeAttribute('src');
		}
		subs.result = null;
		refreshAiGraspnetColorImages('init');
	} else if (!underlayTopic) {
		setResultPanelMode('workpiece');
		if (resultImageEl) {
			resultImageEl.hidden = true;
			resultImageEl.removeAttribute('src');
		}
		subs.result = null;
	} else {
		setResultPanelMode('workpiece');
		if (resultImageEl) {
			resultImageEl.hidden = false;
			resultImageEl.src = transport.cameraStreamUrl(underlayTopic, buildPageStreamId('vision_result'));
		}
		subs.result = true;
	}

	const poseEl = $('pose-text');
	if (poseEl && robotTopic) {
		poseEl.innerHTML = `<div class="pose-card__empty">已订阅 ${escapeHtml(robotTopic)}，等待 RobotStatus…（若持续无数值请检查驱动发布与话题名）</div>`;
	}

	if (robotTopic) {
		transport.onRosJson(robotTopic, m => {
			const el = $('pose-text');
			if (!el) return;
			let html;
			try {
				html = formatRobotPoseHtml(m);
			} catch (err) {
				html = `<div class="pose-card__empty">末端位姿解析异常：${escapeHtml(err && err.message ? String(err.message) : String(err))}</div>`;
			}
			if (robotPoseHtmlIsRenderable(html)) {
				robotPoseCache.value = html;
				el.innerHTML = html;
			} else if (robotPoseCache.value) {
				el.innerHTML = robotPoseCache.value;
			} else {
				el.innerHTML = html;
			}
		});
		transport.subscribe({ topic: robotTopic, msgType: topicTypeMap['topic-robot'] || 'demo_interface/msg/RobotStatus', maxHz: 50 });
		subs.robot = true;
	} else {
		subs.robot = null;
	}

	if (jointTopic) {
		transport.onRosJson(jointTopic, m => {
			pushJointStateSample(m);
		});
		transport.subscribe({ topic: jointTopic, msgType: topicTypeMap['topic-joints'] || 'sensor_msgs/msg/JointState', maxHz: 30 });
		subs.joints = true;
	} else {
		subs.joints = null;
	}

	if (statusTopic) {
		transport.onRosJson(statusTopic, m => {
			const el = $('vpe-status-text');
			if (el) {
				el.textContent = m && m.ivg_display != null ? String(m.ivg_display) : (m && m.data) ? String(m.data) : '';
			}
		});
		transport.subscribe({ topic: statusTopic, msgType: topicTypeMap['topic-vpe-status'] || 'std_msgs/msg/String', maxHz: 10 });
		subs.vpe = true;
	} else {
		subs.vpe = null;
	}

	if (graspTopic) {
		transport.onRosJson(graspTopic, m => {
			const el = $('graspnet-result-text');
			if (el) el.innerHTML = formatFinalGraspPoseHtml(m);
			if (projectionOverlay) projectionOverlay.setGraspMsg(m);
			scheduleGraspColorSnapshotRefresh();
		});
		transport.subscribe({ topic: graspTopic, msgType: topicTypeMap['topic-grasp-poses'] || 'geometry_msgs/msg/PoseArray', maxHz: 15 });
		subs.grasp = true;
	} else {
		subs.grasp = null;
	}

	if (cameraInfoTopic) {
		transport.onRosJson(cameraInfoTopic, m => {
			if (projectionOverlay) projectionOverlay.setCameraInfo(m);
		});
		transport.subscribe({ topic: cameraInfoTopic, msgType: 'sensor_msgs/msg/CameraInfo', maxHz: 5 });
		subs.cameraInfo = true;
	} else {
		subs.cameraInfo = null;
	}

	function handleTfMessage(msg) {
		if (projectionOverlay) projectionOverlay.ingestTfMessage(msg);
	}
	if (tfTopic) {
		transport.onRosJson(tfTopic, handleTfMessage);
		transport.subscribe({ topic: tfTopic, msgType: topicTypeMap['topic-tf'] || 'tf2_msgs/msg/TFMessage', maxHz: 30 });
	}
	if (tfStaticTopic) {
		transport.onRosJson(tfStaticTopic, handleTfMessage);
		transport.subscribe({ topic: tfStaticTopic, msgType: topicTypeMap['topic-tf-static'] || 'tf2_msgs/msg/TFMessage', maxHz: 1 });
	}
	subs.tf = !!(tfTopic || tfStaticTopic);

	if (projectionMode) scheduleProjectionDraw();
	startVisionUrdf3d();
}

export { bindVisionSubscriptions };
