// mode_controller.js — workpiece/graspnet mode toggle UI sync
// 模式切换只更新 UI + 触发投影刷新，不重新订阅 — 订阅与模式无关，避免 auto-activate 清空数据喵~
function createVisionModeController(opts) {
	const options = opts || {};
	const getById = options.getById || (id => document.getElementById(id));
	const setResultPanelMode = typeof options.setResultPanelMode === 'function' ? options.setResultPanelMode : function noop() {};
	const scheduleProjectionDraw = typeof options.scheduleProjectionDraw === 'function' ? options.scheduleProjectionDraw : null;
	function syncModeUi() {
		const workpieceMode = !!(getById('mode-workpiece') && getById('mode-workpiece').checked);
		const graspnetMode = !!(getById('mode-graspnet') && getById('mode-graspnet').checked);
		const fieldObj = getById('control-field-workpiece-id');
		const grpW = getById('control-group-workpiece');
		const grpG = getById('control-group-graspnet');
		const stVpe = getById('status-panel-vpe');
		const stGn = getById('status-panel-graspnet-poses');
		if (fieldObj) {
			fieldObj.hidden = !workpieceMode;
			fieldObj.setAttribute('aria-hidden', workpieceMode ? 'false' : 'true');
		}
		if (grpW) {
			grpW.hidden = !workpieceMode;
			grpW.setAttribute('aria-hidden', workpieceMode ? 'false' : 'true');
		}
		if (grpG) {
			grpG.hidden = !graspnetMode;
			grpG.setAttribute('aria-hidden', graspnetMode ? 'false' : 'true');
		}
		if (stVpe) {
			stVpe.hidden = !workpieceMode;
			stVpe.setAttribute('aria-hidden', workpieceMode ? 'false' : 'true');
		}
		if (stGn) {
			stGn.hidden = !graspnetMode;
			stGn.setAttribute('aria-hidden', graspnetMode ? 'false' : 'true');
		}
		setResultPanelMode(graspnetMode ? 'graspnet' : 'workpiece');
		// 不再调用 startSubscriptions() — topic 订阅与模式无关 (grasp/TF/cameraInfo 始终活跃)
		// 仅需触发投影重绘，数据已由初始订阅持续采集，不再清空重建喵~
		if (graspnetMode && scheduleProjectionDraw) scheduleProjectionDraw();
	}
	return {
		syncModeUi
	};
}
export { createVisionModeController };
