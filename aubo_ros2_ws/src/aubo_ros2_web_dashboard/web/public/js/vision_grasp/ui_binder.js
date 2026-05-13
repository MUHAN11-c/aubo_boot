// ui_binder.js — mode switches, topic settings modal, result image load
function createVisionUiBinder(opts) {
	const options = opts || {};
	const getById = options.getById || (id => document.getElementById(id));
	const onModeChanged = typeof options.onModeChanged === 'function' ? options.onModeChanged : function noop() {};
	const openTopicSettingsModal = typeof options.openTopicSettingsModal === 'function' ? options.openTopicSettingsModal : function noop2() {};
	const closeTopicSettingsModal = typeof options.closeTopicSettingsModal === 'function' ? options.closeTopicSettingsModal : function noop3() {};
	const applyTopicDefaultsToDom = typeof options.applyTopicDefaultsToDom === 'function' ? options.applyTopicDefaultsToDom : function noop4() {};
	const clearTopicsStorage = typeof options.clearTopicsStorage === 'function' ? options.clearTopicsStorage : function noop5() {};
	const saveTopicsToStorage = typeof options.saveTopicsToStorage === 'function' ? options.saveTopicsToStorage : function noop6() {};
	const connect = typeof options.connect === 'function' ? options.connect : function noop7() {};
	const logSvc = typeof options.logSvc === 'function' ? options.logSvc : function noop8() {};
	const topicSettingsModalOpen = typeof options.topicSettingsModalOpen === 'function' ? options.topicSettingsModalOpen : (() => false);
	const scheduleProjectionDraw = typeof options.scheduleProjectionDraw === 'function' ? options.scheduleProjectionDraw : function noop9() {};
	function bindModeSwitches() {
		const modeWorkpiece = getById('mode-workpiece');
		const modeGraspnet = getById('mode-graspnet');
		if (modeWorkpiece) modeWorkpiece.addEventListener('change', onModeChanged);
		if (modeGraspnet) modeGraspnet.addEventListener('change', onModeChanged);
	}
	function bindResultImageLoad() {
		const resultImg = getById('result-mjpeg');
		if (resultImg) resultImg.addEventListener('load', scheduleProjectionDraw);
	}
	function bindTopicSettingsUi() {
		const btnOpen = getById('btn-topic-settings-open');
		if (btnOpen) btnOpen.onclick = openTopicSettingsModal;
		const btnClose = getById('btn-topic-settings-close');
		if (btnClose) btnClose.onclick = closeTopicSettingsModal;
		const bd = getById('topic-settings-backdrop');
		if (bd) bd.onclick = closeTopicSettingsModal;
		const btnDef = getById('btn-topic-restore-defaults');
		if (btnDef) {
			btnDef.onclick = () => {
				applyTopicDefaultsToDom();
				clearTopicsStorage();
				logSvc('已恢复内置默认话题');
			};
		}
		const btnSave = getById('btn-topic-save-reconnect');
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
	}
	return {
		bindModeSwitches,
		bindResultImageLoad,
		bindTopicSettingsUi
	};
}
export { createVisionUiBinder };
