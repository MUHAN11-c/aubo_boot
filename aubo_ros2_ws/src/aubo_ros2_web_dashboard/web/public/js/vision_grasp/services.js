// services.js — ROS service call bindings for grasp/vacuum/gripper
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
		transport
			.callService({
				service: serviceName,
				type: setBoolServiceType,
				request: { data: !!data }
			})
			.then(r => {
				log(`${serviceName} → success=${r.success} ${r.message || ''}`);
				if (typeof done === 'function') done(null, r);
			})
			.catch(e => {
				log(`${serviceName} 错误: ${e}`);
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
		transport
			.callService({
				service: executeSingleService,
				type: fixedServiceTypes['execute-single-grasp'] || 'ivg_interfaces/srv/ExecuteGraspPose',
				request: { object_id: oid, use_visual_estimation: !!useVisual }
			})
			.then(r => {
				log(`${executeSingleService} → success=${r.success} ${r.message || ''}`);
				if (typeof done === 'function') done(null, r);
			})
			.catch(e => {
				log(`${executeSingleService} 错误: ${e}`);
				if (typeof done === 'function') done(e);
			});
	}
	function callGripperSwap(direction) {
		if (!transport.isConnected()) {
			log('未连接，无法执行夹爪快换');
			return;
		}
		const svcName = getSetting('svc-gripper-swap');
		if (!String(svcName || '').trim()) {
			log('服务名为空，无法调用夹爪快换');
			return;
		}
		transport
			.callService({
				service: svcName,
				type: serviceTypeMap['svc-gripper-swap'] || 'ivg_interfaces/srv/RunGripperSwap',
				request: { direction: direction || 'gripper0_to_gripper2' }
			})
			.then(r => {
				log(`${svcName} → success=${r.success} ${r.message || ''}`);
			})
			.catch(e => {
				var reason = String(e);
				if (reason.indexOf('does not exist') !== -1) {
					reason += ' (仿真模式下该服务由真实硬件提供，当前不可用)';
				}
				log(`${svcName} 错误: ${reason}`);
			});
	}
	function bindControlButtons() {
		const btnWpSingleStart = getById('btn-wp-single-start');
		if (btnWpSingleStart) {
			btnWpSingleStart.onclick = () => {
				callExecuteGrasp(useVisualFromMode());
			};
		}
		const btnWpSingleStop = getById('btn-wp-single-stop');
		if (btnWpSingleStop) {
			btnWpSingleStop.onclick = () => {
				callSetBool(getSetting('svc-loop-grasp-control'), false);
				log('已停止');
			};
		}
		const btnWpLoopStart = getById('btn-wp-loop-start');
		if (btnWpLoopStart) {
			btnWpLoopStart.onclick = () => {
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
				callSetBool(getSetting('svc-loop-grasp-control'), false);
				log('停循环');
			};
		}
		const btnGnCapStart = getById('btn-gn-cap-start');
		if (btnGnCapStart) {
			btnGnCapStart.onclick = () => {
				callSetBool(getSetting('svc-graspnet-capture'), true);
			};
		}
		const btnGnCapStop = getById('btn-gn-cap-stop');
		if (btnGnCapStop) {
			btnGnCapStop.onclick = () => {
				callSetBool(getSetting('svc-graspnet-capture'), false);
			};
		}
		const btnGnLoopStart = getById('btn-gn-loop-start');
		if (btnGnLoopStart) {
			btnGnLoopStart.onclick = () => {
				callSetBool(getSetting('svc-publish-grasps-loop'), true);
			};
		}
		const btnGnLoopStop = getById('btn-gn-loop-stop');
		if (btnGnLoopStop) {
			btnGnLoopStop.onclick = () => {
				callSetBool(getSetting('svc-publish-grasps-loop'), false);
			};
		}
		const btnQuickSwap0 = getById('btn-quick-swap-0');
		if (btnQuickSwap0) {
			btnQuickSwap0.onclick = () => {
				callGripperSwap('gripper2_to_gripper0');
			};
		}
		const btnQuickSwap = getById('btn-quick-swap');
		if (btnQuickSwap) {
			btnQuickSwap.onclick = () => {
				callGripperSwap('gripper0_to_gripper2');
			};
		}
		const btnDbgMoveXYZ = getById("btn-dbg-move-xyz");
		if (btnDbgMoveXYZ) {
			btnDbgMoveXYZ.onclick = () => { callDebugMoveXYZ(); };
		}
	}
		function callDebugMoveXYZ() {
			const x = parseFloat((getById("dbg-xyz-x") || {}).value) || 0;
			const y = parseFloat((getById("dbg-xyz-y") || {}).value) || 0;
			const z = parseFloat((getById("dbg-xyz-z") || {}).value) || 0;
			const vel = parseFloat((getById("dbg-xyz-vel") || {}).value) || 0.3;
			const acc = parseFloat((getById("dbg-xyz-acc") || {}).value) || 0.2;
			log("Move XYZ -> (" + x.toFixed(3) + ", " + y.toFixed(3) + ", " + z.toFixed(3) + ") v=" + vel + " a=" + acc);
			transport.callService({
				service: "/debug/move_to_xyz",
				type: "ivg_interfaces/srv/MoveToPose",
				request: {
					target_pose: { position: { x: x, y: y, z: z }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
					target_joints: [0,0,0,0,0,0],
					use_joints: false,
					velocity_factor: vel,
					acceleration_factor: acc
				}
			}).then(function (r) {
				log("Move XYZ -> success=" + r.success + " " + (r.message || ""));
			}).catch(function (e) {
				log("Move XYZ \u9519\u8bef: " + e);
			});
		}
	return {
		callSetBool,
		callExecuteGrasp,
		callGripperSwap,
			callDebugMoveXYZ,
		bindControlButtons
	};
}
export { createVisionServiceActions };
