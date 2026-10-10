// urdf_panel.js — left-panel URDF viewer lifecycle (start/stop/layout)
// 使用主 ivgTransport.ros 连接，不创建独立 WebSocket
import * as ROSLIB from 'roslib';
import { IvgRos3dView3dSession } from '../view3d/session.js';
import { logBus } from '../core/log-bus.js';

function createVisionUrdfPanel(opts) {
	var options = opts || {};
	var SessionCtor = options.SessionCtor || IvgRos3dView3dSession;
	var doc = options.documentRef || document;
	var visionUrdfSession = null;
	var visionUrdfResizeObs = null;
	var toolStatusSub = null;
	var _currentToolId = '';
	var _initialized = false;
	var LS_KEY = 'ivg_last_tool_id';

	function stop() {
		if (toolStatusSub) {
			try { toolStatusSub.unsubscribe(); } catch (e) {}
			toolStatusSub = null;
		}
		if (visionUrdfResizeObs) {
			try { visionUrdfResizeObs.disconnect(); } catch (e) {}
			visionUrdfResizeObs = null;
		}
		if (visionUrdfSession) {
			try { visionUrdfSession.stop(); } catch (e) {}
			visionUrdfSession = null;
		}
		_currentToolId = '';
		_initialized = false;
		// ros 连接由 ivgTransport 统一管理，此处不关闭
	}

	function layout() {
		if (!visionUrdfSession || !visionUrdfSession.viewer3d) return;
		var host = doc.getElementById('vision-urdf-host');
		if (!host || typeof visionUrdfSession.viewer3d.resize !== 'function') return;
		var w = Math.max(120, host.clientWidth || 400);
		var h = Math.max(240, host.clientHeight || Math.round(w * 0.72));
		try { visionUrdfSession.viewer3d.resize(w, h); } catch (e) {}
	}

	// ros: 主 ivgTransport.ros 连接（已建立）
	function start(getById, ros) {
		var host = doc.getElementById('vision-urdf-host');
		if (!host || typeof SessionCtor !== 'function') return;
		if (!ros || !ros.isConnected) {
			logBus.addLog('warn', 'view3d', 'ros 未连接，跳过 3D 启动');
			return;
		}
		stop();
		try {
		visionUrdfSession = new SessionCtor(ros, getById, {
			view3dHostId: 'vision-urdf-host',
			viewerInnerId: 'vision-urdf-inner',
			forceUrdfOnly: true
		});
		visionUrdfSession.start();
		requestAnimationFrame(function () { layout(); });
		if (typeof ResizeObserver !== 'undefined') {
			visionUrdfResizeObs = new ResizeObserver(function () { layout(); });
			visionUrdfResizeObs.observe(host);
		}

		// 初始状态：优先 localStorage（ROS 状态有最多 5s 延迟）喵~
		var stored = '';
		try { stored = localStorage.getItem(LS_KEY) || ''; } catch (e) {}
		if (stored) {
			_currentToolId = stored;
			_initialized = true;
		}

		// 订阅工具状态 → tool_id 变化 → reloadUrdf 显示新夹爪喵~
		toolStatusSub = new ROSLIB.Topic({
			ros: ros,
			name: '/tool_changer_status',
			messageType: 'ivg_interfaces/msg/ToolChangerStatus',
			throttle_rate: 500
		});
	toolStatusSub.subscribe(function (msg) {
		if (!msg) return;
		var newId = String(msg.tool_id || '');
		var connected = msg.is_connected !== false;

		if (!_initialized) {
			// 首条消息：以 ROS 真实状态为准喵~
			_initialized = true;
			_currentToolId = newId;
			try { localStorage.setItem(LS_KEY, newId); } catch (e) {}
			return;
		}

		if (connected && newId !== _currentToolId) {
			_currentToolId = newId;
			try { localStorage.setItem(LS_KEY, newId); } catch (e) {}
			// 延迟等后端 URDF 参数生效后重载喵~
			setTimeout(function () {
				if (visionUrdfSession &&
				    typeof visionUrdfSession.reloadUrdf === 'function') {
					visionUrdfSession.reloadUrdf();
				}
			}, 500);
		}
	});
		} catch (e) {
			logBus.addLog('warn', 'view3d', '左栏 URDF 3D 未启动: ' + (e.message || e));
		}
	}

	return { stop: stop, start: start, layout: layout };
}

var IVGVisionUrdfPanel = { createVisionUrdfPanel: createVisionUrdfPanel };
globalThis.IVGVisionUrdfPanel = IVGVisionUrdfPanel;
export { createVisionUrdfPanel, IVGVisionUrdfPanel };
