// urdf_panel.js — left-panel URDF viewer lifecycle (start/stop/layout)
// 2026-10-10 渲染新岛：默认会话切 IvgIsland3dSession（现代 three + urdf-loader + foxglove 数据），
// 工具状态订阅走 ros_client；不再依赖 rosbridge 连接（数据面自管理，惰性建连）。
import { IvgIsland3dSession } from '../view3d/island_session.js';
import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';
import { logBus } from '../core/log-bus.js';

function createVisionUrdfPanel(opts) {
	var options = opts || {};
	var SessionCtor = options.SessionCtor || IvgIsland3dSession;
	var doc = options.documentRef || document;
	var visionUrdfSession = null;
	var visionUrdfResizeObs = null;
	var toolStatusSub = null;
	var _currentToolId = '';
	var _initialized = false;
	var LS_KEY = 'ivg_last_tool_id';

	function stop() {
		if (toolStatusSub) {
			try { toolStatusSub.cancel(); } catch (e) {}
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
	}

	function layout() {
		if (!visionUrdfSession || !visionUrdfSession.viewer3d) return;
		var host = doc.getElementById('vision-urdf-host');
		if (!host || typeof visionUrdfSession.viewer3d.resize !== 'function') return;
		var w = Math.max(120, host.clientWidth || 400);
		var h = Math.max(240, host.clientHeight || Math.round(w * 0.72));
		try { visionUrdfSession.viewer3d.resize(w, h); } catch (e) {}
	}

	// ros 参数保留签名兼容；现代岛数据自管理（foxglove 连接惰性建立）
	function start(getById, ros) {
		var host = doc.getElementById('vision-urdf-host');
		if (!host || typeof SessionCtor !== 'function') return;
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

			// 订阅工具状态 → tool_id 变化 → reloadUrdf 显示新夹爪（经 foxglove；URDF 重载由
			// robot_description latched 重发驱动，island 会话自动响应）
			var client = foxgloveRosClient();
			if (!client.isConnected) client.connect().catch(function () {});
			toolStatusSub = client.subscribe('/tool_changer_status', 'ivg_interfaces/msg/ToolChangerStatus', function (m) {
				if (!m || !m.data) return;
				var newId = String(m.data.tool_id || '');
				var connected = m.data.is_connected !== false;

				if (!_initialized) {
					// 首条消息：以 ROS 真实状态为准喵~
					_initialized = true;
					_currentToolId = newId;
					try { localStorage.setItem(LS_KEY, newId); } catch (e) {}
					return;
				}

				if (connected && newId && newId !== _currentToolId) {
					_currentToolId = newId;
					try { localStorage.setItem(LS_KEY, newId); } catch (e) {}
				}
			}, { dispatchMode: 'latest' });
		} catch (e) {
			logBus.addLog('warn', 'view3d', '左栏 URDF 3D 未启动: ' + (e.message || e));
		}
	}

	return { stop: stop, start: start, layout: layout, getSession: function () { return visionUrdfSession; } };
}

var IVGVisionUrdfPanel = { createVisionUrdfPanel: createVisionUrdfPanel };
globalThis.IVGVisionUrdfPanel = IVGVisionUrdfPanel;
export { createVisionUrdfPanel, IVGVisionUrdfPanel };
