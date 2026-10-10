// urdf_island.js — 渲染新岛：现代 three(0.164) + urdf-loader 的 URDF 查看器
//
// 与 view3d/urdf-viewer.js 的 createUrdfViewer 同契约（start/stop/layout/getSession/
// addTrajectoryLine/clearTrajectoryLines/getCurrentToolId），数据全部走自研 foxglove 栈：
//   /robot_description（latched 订阅，工具换装时 RSP 重发→自动重载）
//   /joint_states → robot.setJointValues()（按名映射，mimic 关节自动联动）
//   /tool_changer_status → 仅记录 tool_id（URDF 重载由 robot_description 重发驱动）
// mesh 解析：URDFLoader.packages 函数 → 网关 /api/ivg/robot-mesh/<pkg>/<path>。
// 参考：gkjohnson/urdf-loaders 官方示例（javascript/example）的 viewer 结构。
//
// 阶段定位（2a）：latte 面板先行；vision 面板（URDF+Marker+点云混合场景）待 2b/2c 整体迁移。

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoaderDefault from 'urdf-loader';
import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';
import { logBus } from '../core/log-bus.js';

const LS_KEY = 'ivg_last_tool_id';

function meshUrlForPackage(pkg) {
	// 网关 robot_mesh 路由：share 目录内 mesh 文件（大小写不敏感、防穿越）。
	// 注意：urdf-loader 会在返回值后拼接 '/'+relPath，故不带尾斜杠。
	return `/api/ivg/robot-mesh/${pkg}`;
}

export function createUrdfViewerModern(opts = {}) {
	const $ = opts.getById || (id => document.getElementById(id));
	const hostId = opts.hostId || 'vision-urdf-host';
	const innerId = opts.innerId || (hostId + '-inner');

	const client = foxgloveRosClient();

	let _renderer = null;
	let _scene = null;
	let _camera = null;
	let _controls = null;
	let _robot = null;
	let _robotGroup = null;
	let _trajGroup = null;
	let _raf = 0;
	let _resizeObs = null;
	let _subHandles = [];
	let _currentToolId = '';
	let _initialized = false;
	let _urdfText = '';
	let _loadingUrdf = false;
	const _loader = new URDFLoaderDefault();

	_loader.packages = meshUrlForPackage;

	function _ensureConnected() {
		if (!client.isConnected) {
			// urdf-viewer 旧路径要求 ros 已连接；新岛数据自管理，连接惰性建立
			client.connect().catch((e) => {
				logBus.addLog('warn', 'view3d', '[urdf-island] foxglove 连接失败: ' + (e.message || e));
			});
		}
	}

	function _layout() {
		const host = $(hostId);
		if (!host || !_renderer) return;
		const w = Math.max(120, host.clientWidth || 400);
		const h = Math.max(240, host.clientHeight || Math.round(w * 0.72));
		_renderer.setSize(w, h, false);
		_camera.aspect = w / h;
		_camera.updateProjectionMatrix();
	}

	function _loadUrdf(text) {
		if (_loadingUrdf || text === _urdfText) return;
		_urdfText = text;
		_loadingUrdf = true;
		try {
			const robot = _loader.parse(text);
			if (!robot) throw new Error('URDF parse 返回空');
			if (_robot) {
				_robotGroup.remove(_robot);
				_disposeObject(_robot);
			}
			_robot = robot;
			_robot.rotation.x = -Math.PI / 2; // URDF z-up → three y-up（查看用）
			_robotGroup.add(_robot);
			logBus.addLog('info', 'view3d', 'URDF 已加载（现代渲染岛）: ' +
				(robot.joints ? Object.keys(robot.joints).length : '?') + ' 关节');
		} catch (e) {
			logBus.addLog('warn', 'view3d', 'URDF 解析失败: ' + (e.message || e));
		} finally {
			_loadingUrdf = false;
		}
	}

	function _disposeObject(obj) {
		obj.traverse?.((child) => {
			if (child.geometry) child.geometry.dispose?.();
			if (child.material) {
				const mats = Array.isArray(child.material) ? child.material : [child.material];
				for (const m of mats) m.dispose?.();
			}
		});
	}

	function _applyJointStates(msg) {
		if (!_robot || !msg || !Array.isArray(msg.name)) return;
		const values = {};
		for (let i = 0; i < msg.name.length; i++) {
			values[msg.name[i]] = (msg.position && msg.position[i] !== undefined) ? Number(msg.position[i]) : 0;
		}
		try { _robot.setJointValues(values); } catch (_) { }
	}

	function start() {
		const host = $(hostId);
		if (!host) return;
		stop();

		let inner = document.getElementById(innerId);
		if (!inner) {
			inner = document.createElement('div');
			inner.id = innerId;
			host.appendChild(inner);
		}
		host.setAttribute('data-ivg-hide-hints', '1');

		_scene = new THREE.Scene();
		_scene.background = new THREE.Color('#f5f3f0');
		_camera = new THREE.PerspectiveCamera(50, 4 / 3, 0.05, 100);
		_camera.position.set(2.2, 1.8, 2.2);
		_camera.lookAt(0, 0.3, 0);
		try {
			_renderer = new THREE.WebGLRenderer({ antialias: true });
			_renderer.setPixelRatio(Math.min(2, globalThis.devicePixelRatio || 1));
			inner.appendChild(_renderer.domElement);
		} catch (e) {
			// 无 WebGL 环境（如无 GPU 的自动化浏览器）：继续跑数据链路（URDF 解析/关节驱动），
			// 仅跳过渲染——可观测性经 logBus/console 保留
			_renderer = null;
			logBus.addLog('warn', 'view3d', '[urdf-island] WebGL 不可用，仅运行数据链路: ' + (e.message || e));
		}

		if (_renderer) {
			_controls = new OrbitControls(_camera, _renderer.domElement);
			_controls.enableDamping = true;
			_controls.target.set(0, 0.3, 0);
		}

		_scene.add(new THREE.AmbientLight(0xffffff, 0.55));
		const dir = new THREE.DirectionalLight(0xffffff, 1.1);
		dir.position.set(3, 5, 2);
		_scene.add(dir);
		const dir2 = new THREE.DirectionalLight(0xffffff, 0.4);
		dir2.position.set(-3, 2, -2);
		_scene.add(dir2);

		// 网格在 y-up 的 xz 平面（对应 URDF z-up 的 xy 平面）
		const grid = new THREE.GridHelper(4, 40, 0xbbbbbb, 0xdddddd);
		grid.position.y = 0.001;
		_scene.add(grid);

		_robotGroup = new THREE.Group();
		_scene.add(_robotGroup);
		_trajGroup = new THREE.Group();
		_scene.add(_trajGroup);

		const loop = () => {
			_raf = requestAnimationFrame(loop);
			if (_renderer) {
				_controls.update();
				_renderer.render(_scene, _camera);
			}
		};
		if (_renderer) loop();

		requestAnimationFrame(_layout);
		if (typeof ResizeObserver !== 'undefined') {
			_resizeObs = new ResizeObserver(() => _layout());
			_resizeObs.observe(host);
		}

		_ensureConnected();
		_subHandles.push(client.subscribe('/robot_description', 'std_msgs/msg/String', (m) => {
			if (m.data && typeof m.data.data === 'string') _loadUrdf(m.data.data);
		}, { dispatchMode: 'immediate' }));
		_subHandles.push(client.subscribe('/joint_states', 'sensor_msgs/msg/JointState', (m) => {
			_applyJointStates(m.data);
		}, { dispatchMode: 'immediate' }));
		_subHandles.push(client.subscribe('/tool_changer_status', 'ivg_interfaces/msg/ToolChangerStatus', (m) => {
			const d = m.data || {};
			const newId = String(d.tool_id || '');
			if (!_initialized) {
				_initialized = true;
				_currentToolId = newId;
				try { localStorage.setItem(LS_KEY, newId); } catch (_) { }
				return;
			}
			if (d.is_connected !== false && newId && newId !== _currentToolId) {
				_currentToolId = newId;
				try { localStorage.setItem(LS_KEY, newId); } catch (_) { }
			}
		}, { dispatchMode: 'latest' }));
		try {
			const stored = localStorage.getItem(LS_KEY) || '';
			if (stored) {
				_currentToolId = stored;
				_initialized = true;
			}
		} catch (_) { }
	}

	function stop() {
		if (_raf) {
			cancelAnimationFrame(_raf);
			_raf = 0;
		}
		for (const h of _subHandles) {
			try { h.cancel(); } catch (_) { }
		}
		_subHandles = [];
		if (_resizeObs) {
			try { _resizeObs.disconnect(); } catch (_) { }
			_resizeObs = null;
		}
		if (_controls) {
			try { _controls.dispose(); } catch (_) { }
			_controls = null;
		}
		if (_renderer) {
			try { _renderer.dispose(); } catch (_) { }
			_renderer.domElement?.remove();
			_renderer = null;
		}
		if (_robot) {
			_disposeObject(_robot);
			_robot = null;
		}
		_robotGroup = null;
		_trajGroup = null;
		_scene = null;
		_urdfText = '';
		_currentToolId = '';
		_initialized = false;
	}

	function reloadUrdf() {
		// robot_description 为 latched 话题：重取最新值即可（重新订阅一拍）
		const text = _urdfText;
		_urdfText = '';
		_loadUrdf(text);
	}

	function addTrajectoryLine(waypoints, colorHex) {
		if (!_trajGroup || !Array.isArray(waypoints) || waypoints.length < 2) return;
		const pts = [];
		for (const wp of waypoints) {
			const p = wp.position || wp;
			// waypoints 约定 base 系（URDF z-up）→ 岛内 y-up
			pts.push(new THREE.Vector3(Number(p.x) || 0, Number(p.z) || 0, -Number(p.y) || 0));
		}
		const geo = new THREE.BufferGeometry().setFromPoints(pts);
		const mat = new THREE.LineBasicMaterial({ color: colorHex || '#ff6b6b' });
		_trajGroup.add(new THREE.Line(geo, mat));
	}

	function clearTrajectoryLines() {
		if (!_trajGroup) return;
		for (const child of [..._trajGroup.children]) {
			_trajGroup.remove(child);
			child.geometry?.dispose?.();
			child.material?.dispose?.();
		}
	}

	function getSession() {
		// 与 latte/main.js 消费面兼容的最小 shim
		return {
			isModern: true,
			viewer3d: { resize: _layout },
			reloadUrdf,
			addTrajectoryLine,
			clearTrajectoryLines,
		};
	}

	function getCurrentToolId() {
		return _currentToolId;
	}

	function debugState() {
		// 无 WebGL 环境（自动化浏览器）下的数据链路验证钩子
		const joints = {};
		if (_robot && _robot.joints) {
			for (const [name, j] of Object.entries(_robot.joints)) {
				joints[name] = j.angle ?? null;
			}
		}
		return {
			hasRobot: !!_robot,
			hasRenderer: !!_renderer,
			jointCount: Object.keys(joints).length,
			joints,
		};
	}

	return { start, stop, layout: _layout, getSession, getCurrentToolId, addTrajectoryLine, clearTrajectoryLines, reloadUrdf, debugState };
}
