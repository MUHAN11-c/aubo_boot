/**
 * topics_lab 视图会话控制器：
 * - 负责 2D / 3D viewer 的启动与停止。
 * - 让 `ros_console.js` 只保留页面编排，不再直接持有 viewer 创建细节。
 */
(function (global) {
	'use strict';

	function createTopicsLabViewSessions(opts) {
		const options = opts || {};
		const ros2dBackground = options.ros2dBackground || '#eef1f6';
		let viewer2d = null;
		let mapGridClient = null;
		let view3dSession = null;

		function stop2d($) {
			const h = $('view2d-host');
			if (h) h.innerHTML = '';
			viewer2d = null;
			mapGridClient = null;
		}

		function start2d(ros, $) {
			stop2d($);
			if (typeof ROS2D === 'undefined' || typeof createjs === 'undefined') {
				alert('ROS2D / EaselJS 未加载');
				return;
			}
			const host = $('view2d-host');
			const inner = document.createElement('div');
			inner.id = 'ros2d-inner';
			host.appendChild(inner);
			viewer2d = new ROS2D.Viewer({ divID: 'ros2d-inner', width: 800, height: 500, background: ros2dBackground });
			const mapTopic = ($('map-topic').value || '/map').trim();
			mapGridClient = new ROS2D.OccupancyGridClient({
				ros,
				topic: mapTopic,
				rootObject: viewer2d.scene,
				continuous: true
			});
			mapGridClient.on('change', () => {
				const mc = mapGridClient;
				const v = viewer2d;
				if (!mc || !v) return;
				const cg = mc.currentGrid;
				if (!cg || !cg.pose) return;
				v.scaleToDimensions(cg.width, cg.height);
				v.shift(cg.pose.position.x, cg.pose.position.y);
			});
		}

		function stop3d() {
			if (view3dSession) {
				try {
					view3dSession.stop();
				} catch (e0) {
					/* ignore */
				}
				view3dSession = null;
			}
		}

		function start3d(ros, $) {
			if (!ros || !ros.isConnected) {
				alert('未连接 rosbridge，请先重连');
				return;
			}
			if (typeof IvgRos3dView3dSession !== 'function') {
				alert('未加载 view3d/session.js 或 ros3d 依赖');
				return;
			}
			stop3d();
			view3dSession = new IvgRos3dView3dSession(ros, $, {
				showUrdf: false
			});
			view3dSession.start();
		}

		return {
			stop2d,
			start2d,
			stop3d,
			start3d,
			getView3dSession() {
				return view3dSession;
			}
		};
	}

	global.IVGTopicsLabViewSessions = {
		createTopicsLabViewSessions
	};
})(typeof window !== 'undefined' ? window : globalThis);
