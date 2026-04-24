/**
 * 抓取模式控制器：
 * - 同步工件/大模型模式的区块显隐
 * - 同步结果面板模式
 * - 在已连接时触发订阅重建
 */

function createVisionModeController(opts) {
	const options = opts || {};
	const getById = options.getById || (id => document.getElementById(id));
	const setResultPanelMode = typeof options.setResultPanelMode === 'function' ? options.setResultPanelMode : function noop() {};
	const isConnected = typeof options.isConnected === 'function' ? options.isConnected : (() => false);
	const startSubscriptions = typeof options.startSubscriptions === 'function' ? options.startSubscriptions : function noop2() {};

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
		if (isConnected()) startSubscriptions();
	}

	return {
		syncModeUi
	};
}

export { createVisionModeController };
