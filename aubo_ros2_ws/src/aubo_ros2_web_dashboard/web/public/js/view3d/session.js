// session.js — IvgRos3dView3dSession: 3D viewer, TF, URDF, markers
import * as ROSLIB from 'roslib';
import * as ROS3D from 'ros3d';
import { ivgRos3dEmbeddedObject3DClass, installIvgRos3dEmbeddedThreeSafeAddPatch } from './patches.js';
import { IvgRos3dTfClient } from './tf_clients.js';
import { removeView3dUrdfHint, showView3dUrdfHint } from './hints.js';
const IVG_VIEW3D_DESKTOP_PIXEL_RATIO_MAX = 1.5; // 桌面像素比最大值
const IVG_VIEW3D_AXES_SCALE = 0.35; // 坐标轴缩放比例
const IVG_VIEW3D_GRID_COLOR = '#cbd5e1'; // 网格颜色
const IVG_VIEW3D_GRID_CELLS = 10; // 网格单元格数
	function IvgRos3dView3dSession(ros, $, opts) {
		this.ros = ros;
		this.$ = $;
		this.opts = opts || {};
		this._view3dHostId = this.opts.view3dHostId || 'view3d-host';
		this._viewerInnerId = this.opts.viewerInnerId || 'view3d-inner';
		const vh = this.opts.viewerHeight;
		this._viewerHeight = typeof vh === 'number' && vh > 0 ? vh : null;
		this.viewer3d = null;
		this.tfClient3d = null;
		this.ros3dLaserScan = null;
		this.ros3dMarkerClient = null;
		this.ros3dMarkerRoot = null;
		this.ros3dAxes = null;
		this.ros3dGrid = null;
		this.ros3dUrdfRoot = null;
		this._deferredStartTimers = [];
		this._markersStarted = false;
		this._urdfStarted = false;
		this._markerFocusTimer = null;
		this._markerCameraPrimed = false;
		this._Obj3DClass = null;
		this._urdfFocusTimer = null;
		this._urdfCameraPrimed = false;
		this.ros3dUrdfClient = null;
	}
	function ivgComputeViewerPixelRatio(globalObj, coarsePointer) {
		if (coarsePointer) return 1;
		const dpr = Number(globalObj && globalObj.devicePixelRatio) || 1;
		return Math.max(1, Math.min(dpr, IVG_VIEW3D_DESKTOP_PIXEL_RATIO_MAX));
	}
	function ivgApplyAxesScale(axesObj) {
		if (!axesObj || !axesObj.scale || typeof axesObj.scale.set !== 'function') return;
		axesObj.scale.set(IVG_VIEW3D_AXES_SCALE, IVG_VIEW3D_AXES_SCALE, IVG_VIEW3D_AXES_SCALE);
	}
	IvgRos3dView3dSession.prototype._focusViewerOnObject = function (obj3d, opts) {
		if (!this.viewer3d || !this.viewer3d.camera || !obj3d) return;
		const options = opts || {};
		const cam = this.viewer3d.camera;
		const Vector3Emb = cam.position.constructor;
		const center = new Vector3Emb();
		let radius = 0.25;
		try {
			if (typeof obj3d.getWorldPosition === 'function') {
				obj3d.getWorldPosition(center);
			}
			if (typeof obj3d.traverse === 'function') {
				obj3d.traverse(node => {
					if (!node || !node.geometry || !node.geometry.boundingSphere) return;
					const bs = node.geometry.boundingSphere;
					if (node.localToWorld && bs.center) {
						const c = new Vector3Emb(bs.center.x, bs.center.y, bs.center.z);
						node.localToWorld(c);
						center.copy(c);
					}
					radius = Math.max(radius, Number(bs.radius) || 0);
				});
			}
		} catch (e) {
			if (typeof obj3d.getWorldPosition === 'function') obj3d.getWorldPosition(center);
		}
		const camera = this.viewer3d.camera;
		const distanceFactor = Math.max(1.2, Number(options.distanceFactor) || 2.6);
		const dist = Math.max(Number(options.minDistance) || 0.45, radius * distanceFactor);
		camera.position.set(center.x + dist, center.y + dist * 0.72, center.z + Math.max(dist * 0.52, radius * 1.8));
		camera.lookAt(center);
		if (this.viewer3d.cameraControls) {
			this.viewer3d.cameraControls.center = center.clone();
			if (typeof this.viewer3d.cameraControls.update === 'function') this.viewer3d.cameraControls.update();
		}
	};
	IvgRos3dView3dSession.prototype._defer = function (fn, delayMs) {
		const tid = setTimeout(() => {
			this._deferredStartTimers = this._deferredStartTimers.filter(x => x !== tid);
			try {
				fn();
			} catch (e) {
				console.error('[ivg/view3d] deferred stage failed:', e);
			}
		}, delayMs);
		this._deferredStartTimers.push(tid);
	};
	IvgRos3dView3dSession.prototype._startMarkersStage = function (mk) {
		if (this._markersStarted || !mk) return;
		const OC = this._Obj3DClass || null;
		if (!OC) return;
		this._markersStarted = true;
		this.ros3dMarkerRoot = new OC();
		this.viewer3d.addObject(this.ros3dMarkerRoot);
		const MarkerCtor =
			typeof ROS3D.MarkerArrayClient === 'function'
				? ROS3D.MarkerArrayClient
				: ROS3D.MarkerClient;
		const markerOpts = {
			ros: this.ros,
			tfClient: this.tfClient3d,
			topic: mk,
			rootObject: this.ros3dMarkerRoot
		};
		if (MarkerCtor === ROS3D.MarkerClient) markerOpts.lifetime = 0;
		this.ros3dMarkerClient = new MarkerCtor(markerOpts);
		this._markerFocusTimer = setInterval(() => {
			if (!this.ros3dMarkerRoot) return;
			if (this.ros3dMarkerRoot.children && this.ros3dMarkerRoot.children.length > 0 && !this._markerCameraPrimed) {
				this._focusViewerOnObject(this.ros3dMarkerRoot, { distanceFactor: 3.2, minDistance: 0.72 });
				this._markerCameraPrimed = true;
				clearInterval(this._markerFocusTimer);
				this._markerFocusTimer = null;
			}
		}, 250);
	};
	IvgRos3dView3dSession.prototype._startUrdfStage = function ($, host, fixedFrame) {
		if (this._urdfStarted) return;
		const OC = this._Obj3DClass || null;
		if (!OC) return;
		this._urdfStarted = true;
		this.ros3dUrdfRoot = new OC();
		this.viewer3d.addObject(this.ros3dUrdfRoot);
		const rawParam = (($('urdf-param') && $('urdf-param').value) || '').trim();
		let pName = '/robot_state_publisher:robot_description';
		if (rawParam) {
			if (rawParam.includes(':')) pName = rawParam;
			else pName = `/robot_state_publisher:${rawParam.replace(/^\/+/, '')}`;
		}
		this._urdfParamName = pName;
		const meshBase = `${globalThis.location.origin}/api/ivg/robot-mesh/`;
		showView3dUrdfHint(host, '<strong>机械臂 URDF</strong>：使用 ROS3D.UrdfClient 官方链路加载中。');
		try {
			this.ros3dUrdfClient = new ROS3D.UrdfClient({
				ros: this.ros,
				tfClient: this.tfClient3d,
				path: meshBase,
				rootObject: this.ros3dUrdfRoot,
				param: pName
			});
		} catch (err0) {
			console.error('URDF:', err0);
			showView3dUrdfHint(
				host,
				`<strong>机械臂加载失败</strong>：${String(err0)}。请检查参数 <code>${pName}</code>、TF 固定坐标 <code>${fixedFrame}</code> 与网格资源路径。`
			);
			return;
		}
		const selfUrdf = this;
		let urdfFocusTicks = 0;
		selfUrdf._urdfFocusTimer = setInterval(() => {
			urdfFocusTicks += 1;
			if (!selfUrdf.ros3dUrdfRoot || !selfUrdf.viewer3d) {
				clearInterval(selfUrdf._urdfFocusTimer);
				selfUrdf._urdfFocusTimer = null;
				return;
			}
			const ch = selfUrdf.ros3dUrdfRoot.children && selfUrdf.ros3dUrdfRoot.children.length;
			if (ch > 0 && !selfUrdf._urdfCameraPrimed) {
				try {
					selfUrdf.ros3dUrdfRoot.updateMatrixWorld(true);
					selfUrdf._focusViewerOnObject(selfUrdf.ros3dUrdfRoot, { distanceFactor: 2.8, minDistance: 0.48 });
				} catch (e0) {
				}
				selfUrdf._urdfCameraPrimed = true;
				clearInterval(selfUrdf._urdfFocusTimer);
				selfUrdf._urdfFocusTimer = null;
				return;
			}
			if (urdfFocusTicks > 48) {
				clearInterval(selfUrdf._urdfFocusTimer);
				selfUrdf._urdfFocusTimer = null;
				if (!selfUrdf._urdfCameraPrimed && selfUrdf.viewer3d && selfUrdf.viewer3d.camera) {
					try {
						selfUrdf._focusViewerOnObject(selfUrdf.ros3dUrdfRoot, { distanceFactor: 3, minDistance: 0.55 });
					} catch (e1) {
					}
				}
			}
		}, 250);
	};
	IvgRos3dView3dSession.prototype._countMeshes = function (root) {
		var count = 0;
		root.traverse(function (n) { if (n.isMesh) count++; });
		return count;
	};
	IvgRos3dView3dSession.prototype.reloadUrdf = function () {
		if (!this._urdfStarted) return;
		var self = this;
		var OC = this._Obj3DClass || null;
		if (!OC) return;
		// 后台创建新模型，旧模型保持可见，避免画面闪烁
		var newRoot = new OC();
		var oldRoot = this.ros3dUrdfRoot;
		var oldClient = this.ros3dUrdfClient;
		if (this._urdfFocusTimer) {
			clearInterval(this._urdfFocusTimer);
			this._urdfFocusTimer = null;
		}
		// 统计旧模型中的 Mesh 数量作为加载完成目标
		var targetMeshCount = this._countMeshes(oldRoot) || 6;
		var meshBase = (typeof globalThis !== 'undefined' && globalThis.location)
			? globalThis.location.origin + '/api/ivg/robot-mesh/'
			: '/api/ivg/robot-mesh/';
		var newClient = new ROS3D.UrdfClient({
			ros: this.ros,
			tfClient: this.tfClient3d,
			path: meshBase,
			rootObject: newRoot,
			param: this._urdfParamName
		});
		// 递归劫持 newRoot 树中所有 Object3D.add，
		// 每个 STL mesh 异步加载完成 → meshRes.add(mesh) → 计数
		var meshLoaded = 0;
		var ready = false;
		function _wrapAdd(node) {
			if (node.__ivgHijacked) return;
			node.__ivgHijacked = true;
			var orig = node.add.bind(node);
			node.add = function (obj) {
				var r = orig(obj);
				if (!ready && obj && obj.traverse) {
					obj.traverse(function (n) {
						if (n.isMesh) meshLoaded++;
						_wrapAdd(n);
					});
				}
				if (!ready && meshLoaded >= targetMeshCount) {
					ready = true;
					// 先加新模型 → 再移旧模型 → 保证场景从不空白
					if (self.viewer3d && newRoot) {
						try { self.viewer3d.addObject(newRoot); } catch (e) {}
					}
					if (self.viewer3d && oldRoot) {
						try { self.viewer3d.scene.remove(oldRoot); } catch (e) {}
					}
					if (oldClient) {
						try {
							var u = oldClient.urdf;
							if (u && typeof u.unsubscribeTf === 'function') u.unsubscribeTf();
						} catch (e) {}
					}
					self.ros3dUrdfRoot = newRoot;
					self.ros3dUrdfClient = newClient;
				}
				return r;
			};
		}
		// 劫持 newRoot.add → 当 UrdfClient getParam 回调触发
		// newRoot.add(urdfObj) 时，递归劫持整棵 Urdf 子树
		{
			var _origNewRootAdd = newRoot.add.bind(newRoot);
			newRoot.add = function (obj) {
				var r = _origNewRootAdd(obj);
				if (obj && obj.traverse) {
					obj.traverse(function (n) { _wrapAdd(n); });
				}
				return r;
			};
		}
		// 安全兜底: 2s 后若仍未就绪则强制 swap
		setTimeout(function () {
			if (!ready && self.viewer3d && newRoot) {
				ready = true;
				if (self.viewer3d && newRoot) {
					try { self.viewer3d.addObject(newRoot); } catch (e) {}
				}
				if (self.viewer3d && oldRoot) {
					try { self.viewer3d.scene.remove(oldRoot); } catch (e) {}
				}
				if (oldClient) {
					try {
						var u = oldClient.urdf;
						if (u && typeof u.unsubscribeTf === 'function') u.unsubscribeTf();
					} catch (e) {}
				}
				self.ros3dUrdfRoot = newRoot;
				self.ros3dUrdfClient = newClient;
			}
		}, 2000);			};
	IvgRos3dView3dSession.prototype.stop = function () {
		this._deferredStartTimers.forEach(t => clearTimeout(t));
		this._deferredStartTimers.length = 0;
		this._markersStarted = false;
		this._urdfStarted = false;
		this._markerCameraPrimed = false;
		this._urdfCameraPrimed = false;
		if (this._urdfFocusTimer) {
			clearInterval(this._urdfFocusTimer);
			this._urdfFocusTimer = null;
		}
		if (this._markerFocusTimer) {
			clearInterval(this._markerFocusTimer);
			this._markerFocusTimer = null;
		}
		var _rosAlive = this.ros && this.ros.isConnected;
		if (this.ros3dUrdfClient) {
			if (_rosAlive) {
				try {
					const u = this.ros3dUrdfClient.urdf;
					if (u && typeof u.unsubscribeTf === 'function') u.unsubscribeTf();
				} catch (eUrdf) {}
			}
			this.ros3dUrdfClient = null;
		}
		if (this.ros3dMarkerClient) {
			if (_rosAlive) {
				try { this.ros3dMarkerClient.unsubscribe(); } catch (e0) {}
			}
			this.ros3dMarkerClient = null;
		}
		if (this.ros3dLaserScan) {
			if (_rosAlive) {
				try { this.ros3dLaserScan.unsubscribe(); } catch (e2) {}
			}
			this.ros3dLaserScan = null;
		}
		if (this.viewer3d) {
			try {
				if (this.ros3dUrdfRoot) {
					this.viewer3d.scene.remove(this.ros3dUrdfRoot);
					this.ros3dUrdfRoot = null;
				}
				if (this.ros3dMarkerRoot) {
					this.viewer3d.scene.remove(this.ros3dMarkerRoot);
					this.ros3dMarkerRoot = null;
				}
				if (this.ros3dAxes) {
					this.viewer3d.scene.remove(this.ros3dAxes);
					this.ros3dAxes = null;
				}
				if (this.ros3dGrid) {
					this.viewer3d.scene.remove(this.ros3dGrid);
					this.ros3dGrid = null;
				}
			} catch (e3a) {
			}
			try { this.viewer3d.stop(); } catch (e4) {  }
			this.viewer3d = null;
			this._Obj3DClass = null;
		} else {
			this.ros3dMarkerRoot = null;
			this.ros3dUrdfRoot = null;
			this.ros3dAxes = null;
			this.ros3dGrid = null;
			this._Obj3DClass = null;
		}
		if (this.tfClient3d) {
			if (_rosAlive) {
				try { this.tfClient3d.dispose(); } catch (e3) {}
			}
			this.tfClient3d = null;
		}
		const host3 = this.$(this._view3dHostId);
		removeView3dUrdfHint(host3);
		if (host3) host3.innerHTML = '';
	};
	IvgRos3dView3dSession.prototype.start = function () {
		const $ = this.$;
		const ros = this.ros;
		const sn = (($('scan3-topic') && $('scan3-topic').value) || '').trim();
		const mk = (($('marker3-topic') && $('marker3-topic').value) || '').trim();
		const urdfEl = $('view3d-show-urdf');
		const wantUrdf = this.opts && this.opts.forceUrdfOnly ? true : (!urdfEl || urdfEl.checked);
		if (!sn && !mk && !wantUrdf) {
			alert('请至少填写「雷达」或「Marker」话题之一，或勾选「显示机械臂 URDF」');
			return;
		}
		if (mk && typeof ROS3D.MarkerArrayClient === 'undefined' && typeof ROS3D.MarkerClient === 'undefined') {
			alert('已填写 Marker 话题但未加载 ROS3D.MarkerArrayClient / MarkerClient（需 ros3d）');
			return;
		}
		if (wantUrdf && typeof ROS3D.UrdfClient !== 'function') {
			alert('当前脚本缺少 ROS3D.UrdfClient，无法显示机械臂；请更新 ros3d。');
			return;
		}
		this.stop();
		if (typeof ROS3D === 'undefined' || typeof ROSLIB === 'undefined') {
			alert('ROS3D 或 ROSLIB 未加载');
			return;
		}
		const host = $(this._view3dHostId);
		const innerId = this._viewerInnerId;
		const inner = document.createElement('div');
		inner.id = innerId;
		if (host) host.appendChild(inner);
		if (host) host.setAttribute('data-ivg-hide-hints', '1');
		const coarsePointer = typeof globalThis.matchMedia === 'function' && globalThis.matchMedia('(pointer: coarse)').matches;
		const w = (host && host.clientWidth) || 800;
		let h = 800;
		if (this._viewerHeight != null) h = this._viewerHeight;
		else if (host) h = Math.max(240, host.clientHeight || Math.round(w * 0.75));
		this.viewer3d = new ROS3D.Viewer({
			divID: innerId,
			width: w,
			height: h,
			antialias: !coarsePointer,
			background: '#1a1a2e',
				intensity: 0.85,
			cameraPose: { x: 3, y: 3, z: 3 },
			cameraZoomSpeed: 0.5
		});
		installIvgRos3dEmbeddedThreeSafeAddPatch(this.viewer3d);
		this._Obj3DClass = ivgRos3dEmbeddedObject3DClass(this.viewer3d);
		if (this.viewer3d && this.viewer3d.renderer && typeof this.viewer3d.renderer.setPixelRatio === 'function') {
			this.viewer3d.renderer.setPixelRatio(ivgComputeViewerPixelRatio(globalThis, coarsePointer));
		}
		const axEl = $('view3d-show-axes');
		const grEl = $('view3d-show-grid');
		if (!axEl || axEl.checked) {
			this.ros3dAxes = new ROS3D.Axes();
			ivgApplyAxesScale(this.ros3dAxes);
			this.viewer3d.addObject(this.ros3dAxes);
		}
		if (!grEl || grEl.checked) {
			this.ros3dGrid = new ROS3D.Grid({ num_cells: IVG_VIEW3D_GRID_CELLS, cellSize: 1, color: IVG_VIEW3D_GRID_COLOR });
			this.viewer3d.addObject(this.ros3dGrid);
		}
		let fixedFrame = (($('tf-fixed-frame') && $('tf-fixed-frame').value) || 'base_link').trim().replace(/^\/+/, '');
		if (fixedFrame === '') fixedFrame = 'base_link';
		this._fixedFrame = fixedFrame;
		const tf3dOpts = (this.opts && this.opts.tf3d) || {};
		this.tfClient3d = new IvgRos3dTfClient(ros, fixedFrame, tf3dOpts);
		const pszEl = $('view3d-point-size');
		const ptSize = Math.max(0.01, parseFloat((pszEl && pszEl.value) || '0.05') || 0.05);
		const maxPts = coarsePointer ? 4500 : 32000;
		if (sn) {
			this.ros3dLaserScan = new ROS3D.LaserScan({
				ros,
				tfClient: this.tfClient3d,
				rootObject: this.viewer3d.scene,
				topic: sn,
				max_pts: maxPts,
				material: { size: Math.max(0.01, ptSize * 0.85), color: 0xff4444 }
			});
		}
		if (mk) this._defer(() => this._startMarkersStage(mk), 1200);
		if (wantUrdf && typeof ROS3D.UrdfClient === 'function') {
			const urdfDelayMs = this.opts && this.opts.forceUrdfOnly ? 0 : 3500;
			this._defer(() => this._startUrdfStage($, host, fixedFrame), urdfDelayMs);
		}
	};
const IVGView3DSession = {
	IvgRos3dView3dSession
};
globalThis.IVGView3DSession = IVGView3DSession;
globalThis.IvgRos3dView3dSession = IvgRos3dView3dSession;
export {
	ivgComputeViewerPixelRatio,
	ivgApplyAxesScale,
	IvgRos3dView3dSession,
	IVGView3DSession
};
