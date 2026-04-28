// urdf_panel.js — left-panel URDF viewer lifecycle (start/stop/layout)
import * as ROSLIB from 'roslib';
import { IvgRos3dView3dSession } from '../view3d/session.js';
function createVisionUrdfPanel(opts) {
	const options = opts || {};
	const ports = options.ports;
	const SessionCtor = options.SessionCtor || IvgRos3dView3dSession;
	const doc = options.documentRef || document;
	let visionUrdfRos = null;
	let visionUrdfSession = null;
	let visionUrdfResizeObs = null;
		function stop() {
			if (visionUrdfResizeObs) {
				try { visionUrdfResizeObs.disconnect(); } catch (e) {  }
				visionUrdfResizeObs = null;
			}
			if (visionUrdfSession) {
				try { visionUrdfSession.stop(); } catch (e) {  }
				visionUrdfSession = null;
			}
			if (visionUrdfRos) {
				try { visionUrdfRos.close(); } catch (e) {  }
				visionUrdfRos = null;
			}
		}
		function ensureRos() {
			return new Promise((resolve, reject) => {
				if (visionUrdfRos && visionUrdfRos.isConnected) {
					resolve(visionUrdfRos);
					return;
				}
				if (typeof ROSLIB === 'undefined' || typeof ROSLIB.Ros !== 'function') {
					reject(new Error('ROSLIB 未加载'));
					return;
				}
				const url = ports.rosbridgeWebSocketUrl();
				const r = new ROSLIB.Ros({ url });
				const t = setTimeout(() => {
					try { r.close(); } catch (e1) {  }
					reject(new Error('3D 连接超时'));
				}, 20000);
				r.on('connection', () => {
					clearTimeout(t);
					visionUrdfRos = r;
					resolve(visionUrdfRos);
				});
				r.on('error', () => {
					clearTimeout(t);
					reject(new Error('3D 连接错误'));
				});
			});
		}
		function layout() {
			if (!visionUrdfSession || !visionUrdfSession.viewer3d) return;
			const host = doc.getElementById('vision-urdf-host');
			if (!host || typeof visionUrdfSession.viewer3d.resize !== 'function') return;
			const w = Math.max(120, host.clientWidth || 400);
			const h = Math.max(240, host.clientHeight || Math.round(w * 0.72));
			try {
				visionUrdfSession.viewer3d.resize(w, h);
			} catch (e) {
			}
		}
		function start(getById) {
			const host = doc.getElementById('vision-urdf-host');
			if (!host || typeof SessionCtor !== 'function') return;
			stop();
			void (async () => {
				try {
					const ros = await ensureRos();
					visionUrdfSession = new SessionCtor(ros, getById, {
						view3dHostId: 'vision-urdf-host',
						viewerInnerId: 'vision-urdf-inner',
						forceUrdfOnly: true
					});
					visionUrdfSession.start();
					requestAnimationFrame(() => layout());
					if (typeof ResizeObserver !== 'undefined') {
						visionUrdfResizeObs = new ResizeObserver(() => layout());
						visionUrdfResizeObs.observe(host);
					}
				} catch (e) {
					console.warn('[ivg/vision] 左栏 URDF 3D 未启动:', e);
				}
			})();
		}
	return {
		stop,
		start,
		layout
	};
}
const IVGVisionUrdfPanel = {
	createVisionUrdfPanel
};
globalThis.IVGVisionUrdfPanel = IVGVisionUrdfPanel;
export { createVisionUrdfPanel, IVGVisionUrdfPanel };
