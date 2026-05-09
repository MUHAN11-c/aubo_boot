// urdf_panel.js — left-panel URDF viewer lifecycle (start/stop/layout)
// 使用主 ivgTransport.ros 连接，不创建独立 WebSocket
import * as ROSLIB from 'roslib';
import { IvgRos3dView3dSession } from '../view3d/session.js';

function createVisionUrdfPanel(opts) {
	var options = opts || {};
	var SessionCtor = options.SessionCtor || IvgRos3dView3dSession;
	var doc = options.documentRef || document;
	var visionUrdfSession = null;
	var visionUrdfResizeObs = null;
	var toolStatusSub = null;

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
			console.warn('[urdf_panel] ros 未连接，跳过 3D 启动');
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
		// 订阅工具状态 → tool_id 变化 → 重载 URDF 显示新夹爪
		var currentToolId = null;
		toolStatusSub = new ROSLIB.Topic({
			ros: ros,
			name: '/tool_changer_status',
			messageType: 'tool_changer_interface/msg/ToolChangerStatus',
			throttle_rate: 500
		});
	toolStatusSub.subscribe(function (msg) {
		var newId = msg && msg.tool_id;
		if (!newId) return;
		if (currentToolId === null) {
			// 首条消息：记录当前 tool_id，不触发重载
			currentToolId = newId;
		} else if (newId !== currentToolId) {
			// tool_id 变化 → 重载 URDF 显示新夹爪
			currentToolId = newId;
			setTimeout(function () {
				if (visionUrdfSession &&
				    typeof visionUrdfSession.reloadUrdf === 'function') {
					visionUrdfSession.reloadUrdf();
				}
			}, 200);
		}
	});
		} catch (e) {
			console.warn('[ivg/vision] 左栏 URDF 3D 未启动:', e);
		}
	}

	return { stop: stop, start: start, layout: layout };
}

var IVGVisionUrdfPanel = { createVisionUrdfPanel: createVisionUrdfPanel };
globalThis.IVGVisionUrdfPanel = IVGVisionUrdfPanel;
export { createVisionUrdfPanel, IVGVisionUrdfPanel };
