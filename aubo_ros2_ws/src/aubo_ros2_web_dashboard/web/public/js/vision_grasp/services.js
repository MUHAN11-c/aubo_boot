// services.js — ROS service call bindings for grasp/vacuum/gripper
import { logBus } from '../core/log-bus.js';

function createVisionServiceActions(opts) {
	const options = opts || {};
	const transport = options.transport;
	const log = typeof options.log === 'function' ? options.log : function noop() {};
	const getById = options.getById || (id => document.getElementById(id));
	const getSetting = typeof options.getSetting === 'function' ? options.getSetting : function empty() { return ''; };
	const useVisualFromMode = typeof options.useVisualFromMode === 'function' ? options.useVisualFromMode : (() => true);
	const executeSingleService = options.executeSingleService || '/execute_single_grasp';
	const serviceTypeMap = options.serviceTypeMap || {};
	const fixedServiceTypes = options.fixedServiceTypes || {};
	const setBoolServiceType =
		serviceTypeMap['svc-loop-grasp-control'] ||
		serviceTypeMap['svc-graspnet-capture'] ||
		serviceTypeMap['svc-publish-grasps-loop'] ||
		'std_srvs/srv/SetBool';
	function callSetBool(name, data, done) {
		const serviceName = String(name || '').trim();
		if (!transport.isConnected()) {
			log('未连接');
			if (typeof done === 'function') done(new Error('未连接'));
			return;
		}
		if (!serviceName) {
			log('服务名为空，无法调用 SetBool');
			if (typeof done === 'function') done(new Error('empty_service_name'));
			return;
		}
		logBus.addLog('info', 'service', '开始: ' + serviceName, { phase: 'start', service: serviceName });
		transport
			.callService({
				service: serviceName,
				type: setBoolServiceType,
				request: { data: !!data }
			})
			.then(r => {
				log(`${serviceName} → success=${r.success} ${r.message || ''}`);
				logBus.addLog('info', 'service', '✓ ' + serviceName + ' → success=' + r.success, { phase: 'completed', service: serviceName, success: r.success });
				if (typeof done === 'function') done(null, r);
			})
			.catch(e => {
				log(`${serviceName} 错误: ${e}`);
				logBus.addLog('error', 'service', '✗ ' + serviceName + ' 错误: ' + e, { phase: 'failed', service: serviceName, error: String(e) });
				if (typeof done === 'function') done(e);
			});
	}
	function callExecuteGrasp(useVisual, done) {
		if (!transport.isConnected()) {
			log('未连接');
			if (typeof done === 'function') done(new Error('未连接'));
			return;
		}
		const oid = (getById('object-id') && getById('object-id').value.trim()) || '';
		logBus.addLog('info', 'service', '开始: ' + executeSingleService + ' (' + (useVisual ? '视觉模式' : '固定模式') + ')', { phase: 'start', service: executeSingleService, use_visual: !!useVisual });
		transport
			.callService({
				service: executeSingleService,
				type: fixedServiceTypes['execute-single-grasp'] || 'demo_interface/srv/ExecuteGraspPose',
				request: { object_id: oid, use_visual_estimation: !!useVisual }
			})
			.then(r => {
				log(`${executeSingleService} → success=${r.success} ${r.message || ''}`);
				logBus.addLog('info', 'service', '✓ ' + executeSingleService + ' → success=' + r.success, { phase: 'completed', service: executeSingleService, success: r.success });
				if (typeof done === 'function') done(null, r);
			})
			.catch(e => {
				log(`${executeSingleService} 错误: ${e}`);
				logBus.addLog('error', 'service', '✗ ' + executeSingleService + ' 错误: ' + e, { phase: 'failed', service: executeSingleService, error: String(e) });
				if (typeof done === 'function') done(e);
			});
	}
	var _currentToolId = '';
	function callGripperSwap(targetId, done) {
		if (!transport.isConnected()) {
			log('未连接，无法执行夹爪快换');
			if (typeof done === 'function') done(new Error('未连接'));
			return;
		}
		const svcName = getSetting('svc-gripper-swap');
		if (!String(svcName || '').trim()) {
			log('服务名为空，无法调用夹爪快换');
			if (typeof done === 'function') done(new Error('empty_service_name'));
			return;
		}
		// direction: 空状态→直接目标ID；有工具→"current_to_target"
		var direction = _currentToolId ? (_currentToolId + '_to_' + targetId) : targetId;
		logBus.addLog('info', 'service', '开始: ' + svcName + ' (' + direction + ')', { phase: 'start', service: svcName, direction: direction });
		transport
			.callService({
				service: svcName,
				type: serviceTypeMap['svc-gripper-swap'] || 'ivg_interfaces/srv/RunGripperSwap',
				request: { direction: direction }
			})
			.then(r => {
				log(`${svcName} → success=${r.success} ${r.message || ''}`);
				logBus.addLog('info', 'service', '✓ ' + svcName + ' → success=' + r.success, { phase: 'completed', service: svcName, success: r.success });
				if (typeof done === 'function') done(null, r);
			})
			.catch(e => {
				var reason = String(e);
				if (reason.indexOf('does not exist') !== -1) {
					reason += ' (仿真模式下该服务由真实硬件提供，当前不可用)';
				}
				log(`${svcName} 错误: ${reason}`);
				logBus.addLog('error', 'service', '✗ ' + svcName + ' 错误: ' + reason, { phase: 'failed', service: svcName, error: reason });
				if (typeof done === 'function') done(e);
			});
	}
	function setCurrentToolId(toolId) {
		_currentToolId = String(toolId || '');
	}
	function bindControlButtons() {
		const btnWpSingleStart = getById('btn-wp-single-start');
		if (btnWpSingleStart) {
			btnWpSingleStart.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: 执行单次抓取');
				callExecuteGrasp(useVisualFromMode());
			};
		}
		const btnWpSingleStop = getById('btn-wp-single-stop');
		if (btnWpSingleStop) {
			btnWpSingleStop.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: 停止单次抓取');
				callSetBool(getSetting('svc-loop-grasp-control'), false);
				log('已停止');
			};
		}
		const btnWpLoopStart = getById('btn-wp-loop-start');
		if (btnWpLoopStart) {
			btnWpLoopStart.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: 循环抓取启动');
				if (useVisualFromMode()) {
					callSetBool(getSetting('svc-loop-grasp-control'), true);
					log('循环：后端视觉');
				} else {
					callSetBool(getSetting('svc-loop-grasp-control'), false, err => {
						if (err) return;
						log('非视觉循环应由 ROS/后端调度；浏览器不再轮询 execute_single_grasp');
					});
				}
			};
		}
		const btnWpLoopStop = getById('btn-wp-loop-stop');
		if (btnWpLoopStop) {
			btnWpLoopStop.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: 循环抓取停止');
				callSetBool(getSetting('svc-loop-grasp-control'), false);
				log('停循环');
			};
		}
		const btnGnCapStart = getById('btn-gn-cap-start');
		if (btnGnCapStart) {
			btnGnCapStart.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: GraspNet 采集启动');
				callSetBool(getSetting('svc-graspnet-capture'), true);
			};
		}
		const btnGnCapStop = getById('btn-gn-cap-stop');
		if (btnGnCapStop) {
			btnGnCapStop.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: GraspNet 采集停止');
				callSetBool(getSetting('svc-graspnet-capture'), false);
			};
		}
		const btnGnLoopStart = getById('btn-gn-loop-start');
		if (btnGnLoopStart) {
			btnGnLoopStart.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: GraspNet 循环启动');
				callSetBool(getSetting('svc-publish-grasps-loop'), true);
			};
		}
		const btnGnLoopStop = getById('btn-gn-loop-stop');
		if (btnGnLoopStop) {
			btnGnLoopStop.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: GraspNet 循环停止');
				callSetBool(getSetting('svc-publish-grasps-loop'), false);
			};
		}
		const btnQuickSwap0 = getById('btn-quick-swap-0');
		if (btnQuickSwap0) {
			btnQuickSwap0.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: 快换 → gripper0');
				callGripperSwap('gripper0');
			};
		}
		const btnQuickSwap1 = getById('btn-quick-swap-1');
		if (btnQuickSwap1) {
			btnQuickSwap1.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: 快换 → gripper1');
				callGripperSwap('gripper1');
			};
		}
		const btnQuickSwap2 = getById('btn-quick-swap-2');
		if (btnQuickSwap2) {
			btnQuickSwap2.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: 快换 → gripper2');
				callGripperSwap('gripper2');
			};
		}
		const btnDbgMoveXYZ = getById("btn-dbg-move-xyz");
		if (btnDbgMoveXYZ) {
			btnDbgMoveXYZ.onclick = () => {
				logBus.addLog('info', 'service', '按钮点击: Debug Move XYZ');
				callDebugMoveXYZ();
			};
		}
	}
		function callDebugMoveXYZ() {
			const x = parseFloat((getById("dbg-xyz-x") || {}).value) || 0;
			const y = parseFloat((getById("dbg-xyz-y") || {}).value) || 0;
			const z = parseFloat((getById("dbg-xyz-z") || {}).value) || 0;
			const vel = parseFloat((getById("dbg-xyz-vel") || {}).value) || 0.3;
			const acc = parseFloat((getById("dbg-xyz-acc") || {}).value) || 0.2;
			log("Move XYZ -> (" + x.toFixed(3) + ", " + y.toFixed(3) + ", " + z.toFixed(3) + ") v=" + vel + " a=" + acc);
			logBus.addLog('info', 'service', '\u5f00\u59cb: /debug/move_to_xyz (' + x.toFixed(3) + ', ' + y.toFixed(3) + ', ' + z.toFixed(3) + ')', { phase: 'start', service: '/debug/move_to_xyz' });
			transport.callService({
				service: "/debug/move_to_xyz",
				type: "demo_interface/srv/MoveToPose",
				request: {
					target_pose: { position: { x: x, y: y, z: z }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
					target_joints: [0,0,0,0,0,0],
					use_joints: false,
					velocity_factor: vel,
					acceleration_factor: acc
				}
			}).then(function (r) {
				log("Move XYZ -> success=" + r.success + " " + (r.message || ""));
				logBus.addLog('info', 'service', '\u2713 /debug/move_to_xyz \u2192 success=' + r.success, { phase: 'completed', service: '/debug/move_to_xyz', success: r.success });
			}).catch(function (e) {
				log("Move XYZ \u9519\u8bef: " + e);
				logBus.addLog('error', 'service', '\u2717 /debug/move_to_xyz \u9519\u8bef: ' + e, { phase: 'failed', service: '/debug/move_to_xyz', error: String(e) });
			});
		}
	return {
		callSetBool,
		callExecuteGrasp,
		callGripperSwap,
		setCurrentToolId,
			callDebugMoveXYZ,
		bindControlButtons
	};
}
export { createVisionServiceActions };
