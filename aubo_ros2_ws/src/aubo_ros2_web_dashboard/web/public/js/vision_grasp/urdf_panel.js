/**
 * 视觉抓取左栏 URDF 面板控制器：
 * - 负责独立 rosbridge 连接、3D 会话创建、ResizeObserver 自适应。
 * - 让 `vision_grasp_panel.js` 不再直接管理这部分资源生命周期。
 */
(function (global) {
	'use strict';

	function createVisionUrdfPanel(opts) {
		const options = opts || {};
		const ports = options.ports;
		const SessionCtor = options.SessionCtor || global.IvgRos3dView3dSession;
		const doc = options.documentRef || document;
		let visionUrdfRos = null;
		let visionUrdfSession = null;
		let visionUrdfResizeObs = null;

		function stop() {
			if (visionUrdfResizeObs) {
				try { visionUrdfResizeObs.disconnect(); } catch (e) { /* ignore */ }
				visionUrdfResizeObs = null;
			}
			if (visionUrdfSession) {
				try { visionUrdfSession.stop(); } catch (e) { /* ignore */ }
				visionUrdfSession = null;
			}
			if (visionUrdfRos) {
				try { visionUrdfRos.close(); } catch (e) { /* ignore */ }
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
				const r = new ROSLIB.Ros();
				const t = setTimeout(() => {
					try { r.close(); } catch (e1) { /* ignore */ }
					reject(new Error('rosbridge 连接超时'));
				}, 20000);
				r.on('connection', () => {
					clearTimeout(t);
					visionUrdfRos = r;
					resolve(visionUrdfRos);
				});
				r.on('error', () => {
					clearTimeout(t);
					reject(new Error('rosbridge 连接错误'));
				});
				void r.connect(url).catch(() => {
					clearTimeout(t);
					reject(new Error('rosbridge connect() 失败'));
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
				/* ignore */
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
						viewerInnerId: 'vision-urdf-inner'
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

	global.IVGVisionUrdfPanel = {
		createVisionUrdfPanel
	};
})(typeof window !== 'undefined' ? window : globalThis);
