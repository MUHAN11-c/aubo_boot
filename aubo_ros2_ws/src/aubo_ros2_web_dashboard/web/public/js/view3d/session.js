/**
 * 3D 会话编排层：
 * - 负责把补丁、TF、点云、雷达、Marker、URDF 装配成一个页面可复用的 viewer 会话。
 * - 继续对外保留 `IvgRos3dView3dSession`，让 topics_lab / vision_grasp 无需知道内部拆分细节。
 */
(function (global) {
	'use strict';

	const patches = global.IVGView3DPatches;
	const tfApi = global.IVGView3DTf;
	const hints = global.IVGView3DHints;
	const pcApi = global.IVGView3DPointCloud;
	const urdfApi = global.IVGView3DUrdf;
	if (!patches || !tfApi || !hints || !pcApi || !urdfApi) {
		throw new Error('view3d session 依赖未按顺序加载');
	}

	const normalizeFrameId = tfApi.normalizeFrameId;
	const IvgRos3dTfClient = tfApi.IvgRos3dTfClient;
	const IvgRosPointCloudClient = pcApi.IvgRosPointCloudClient;
	const removeView3dPc2Hint = hints.removeView3dPc2Hint;
	const showView3dPc2Hint = hints.showView3dPc2Hint;
	const removeView3dUrdfHint = hints.removeView3dUrdfHint;
	const showView3dUrdfHint = hints.showView3dUrdfHint;
	const ivgAttachUrdfFromRosParam = urdfApi.ivgAttachUrdfFromRosParam;
	const IVG_VIEW3D_DESKTOP_PIXEL_RATIO_MAX = 1.5;
	const IVG_VIEW3D_AXES_SCALE = 0.35;
	const IVG_VIEW3D_GRID_COLOR = '#cbd5e1';
	const IVG_VIEW3D_GRID_CELLS = 10;

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
		this.ros3dPointCloud2 = null;
		this.ros3dLaserScan = null;
		this.ros3dMarkerClient = null;
		this.ros3dMarkerRoot = null;
		this.ros3dAxes = null;
		this.ros3dGrid = null;
		this.ros3dUrdfRoot = null;
		this._pcAutoFrameFallbackTimer = null;
		this._pcAutoFrameFallbackUsed = false;
		this._pcCameraPrimed = false;
		this._deferredStartTimers = [];
		this._markersStarted = false;
		this._urdfStarted = false;
		this._markerFocusTimer = null;
		this._markerCameraPrimed = false;
		this._Obj3DClass = null;
		this._urdfFocusTimer = null;
		this._urdfCameraPrimed = false;
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
			if (typeof THREE !== 'undefined' && THREE.Box3 && THREE.Sphere) {
				const box = new THREE.Box3().setFromObject(obj3d);
				if (!box.isEmpty()) {
					box.getCenter(center);
					const sphere = box.getBoundingSphere(new THREE.Sphere());
					radius = Math.max(0.08, Number(sphere && sphere.radius) || 0.25);
				} else if (typeof obj3d.getWorldPosition === 'function') {
					obj3d.getWorldPosition(center);
				}
			} else if (typeof obj3d.getWorldPosition === 'function') {
				obj3d.getWorldPosition(center);
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
		const OC = this._Obj3DClass || (typeof THREE !== 'undefined' ? THREE.Object3D : null);
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
		const OC = this._Obj3DClass || (typeof THREE !== 'undefined' ? THREE.Object3D : null);
		if (!OC) return;
		this._urdfStarted = true;
		this.ros3dUrdfRoot = new OC();
		this.viewer3d.addObject(this.ros3dUrdfRoot);
		const pName = (($('urdf-param') && $('urdf-param').value) || '/robot_state_publisher:robot_description').trim();
		const meshBase = `${global.location.origin}/api/ivg/robot-mesh/`;
		showView3dUrdfHint(host, '<strong>机械臂 URDF</strong>：阶段式加载中；若仍失败，不会再阻断点云与 Marker 主视图。');
		const selfUrdf = this;
		ivgAttachUrdfFromRosParam(this.ros, pName, meshBase, this.tfClient3d, this.ros3dUrdfRoot, $, err => {
			console.error('URDF:', err);
			showView3dUrdfHint(host, `<strong>机械臂加载失败</strong>：${String(err)}。当前 3D 主视图已继续工作；请单独检查 <code>${fixedFrame}</code> 与网格资源。`);
		}, host);
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
					/* ignore */
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
						/* ignore */
					}
				}
			}
		}, 250);
	};

	IvgRos3dView3dSession.prototype.stop = function () {
		this._deferredStartTimers.forEach(t => clearTimeout(t));
		this._deferredStartTimers.length = 0;
		if (this._pcAutoFrameFallbackTimer) {
			clearInterval(this._pcAutoFrameFallbackTimer);
			this._pcAutoFrameFallbackTimer = null;
		}
		this._pcAutoFrameFallbackUsed = false;
		this._pcCameraPrimed = false;
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
		if (this.ros3dMarkerClient) {
			try { this.ros3dMarkerClient.unsubscribe(); } catch (e0) { /* ignore */ }
			this.ros3dMarkerClient = null;
		}
		if (this.ros3dPointCloud2) {
			try { this.ros3dPointCloud2.unsubscribe(); } catch (e1) { /* ignore */ }
			this.ros3dPointCloud2 = null;
		}
		if (this.ros3dLaserScan) {
			try { this.ros3dLaserScan.unsubscribe(); } catch (e2) { /* ignore */ }
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
				/* ignore */
			}
			try { this.viewer3d.stop(); } catch (e4) { /* ignore */ }
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
			try { this.tfClient3d.dispose(); } catch (e3) { /* ignore */ }
			this.tfClient3d = null;
		}
		const host3 = this.$(this._view3dHostId);
		removeView3dPc2Hint(host3);
		removeView3dUrdfHint(host3);
		if (host3) host3.innerHTML = '';
	};

	IvgRos3dView3dSession.prototype.start = function () {
		const $ = this.$;
		const ros = this.ros;
		const pcn = (($('pc-topic') && $('pc-topic').value) || '').trim();
		const sn = (($('scan3-topic') && $('scan3-topic').value) || '').trim();
		const mk = (($('marker3-topic') && $('marker3-topic').value) || '').trim();
		const urdfEl = $('view3d-show-urdf');
		const wantUrdf = !urdfEl || urdfEl.checked;
		if (!pcn && !sn && !mk && !wantUrdf) {
			alert('请至少填写「点云」「雷达」或「Marker」话题之一，或勾选「显示机械臂 URDF」');
			return;
		}
		if (mk && typeof ROS3D.MarkerArrayClient === 'undefined' && typeof ROS3D.MarkerClient === 'undefined') {
			alert('已填写 Marker 话题但未加载 ROS3D.MarkerArrayClient / MarkerClient（需 ros3d.min.js）');
			return;
		}
		if (wantUrdf && !pcn && !sn && !mk) {
			if (typeof ROS3D.Urdf !== 'function' || typeof ROSLIB.UrdfModel !== 'function') {
				alert('当前脚本缺少 ROS3D.Urdf / ROSLIB.UrdfModel，无法显示机械臂；请更新 ros3d / roslib');
				return;
			}
		}
		this.stop();
		if (typeof ROS3D === 'undefined' || typeof ROSLIB === 'undefined') {
			alert('ROS3D 或 ROSLIB 未加载（需 roslib-2.iife.js + three r89 + ros3d）');
			return;
		}
		patches.installIvgThreeSafeAddPatch();
		patches.ivgInstallMeshLoaderCasePatch();
		const host = $(this._view3dHostId);
		const innerId = this._viewerInnerId;
		const inner = document.createElement('div');
		inner.id = innerId;
		if (host) host.appendChild(inner);
		if (host) host.setAttribute('data-ivg-hide-hints', '1');
		const coarsePointer = typeof global.matchMedia === 'function' && global.matchMedia('(pointer: coarse)').matches;
		const w = (host && host.clientWidth) || 800;
		let h = 800;
		if (this._viewerHeight != null) h = this._viewerHeight;
		else if (host) h = Math.max(240, host.clientHeight || Math.round(w * 0.75));
		this.viewer3d = new ROS3D.Viewer({
			divID: innerId,
			width: w,
			height: h,
			antialias: !coarsePointer,
			background: '#ffffff',
			cameraPose: { x: 3, y: 3, z: 3 },
			cameraZoomSpeed: 0.5
		});
		patches.installIvgRos3dEmbeddedThreeSafeAddPatch(this.viewer3d);
		this._Obj3DClass = patches.ivgRos3dEmbeddedObject3DClass(this.viewer3d);
		if (this.viewer3d && this.viewer3d.renderer && typeof this.viewer3d.renderer.setPixelRatio === 'function') {
			this.viewer3d.renderer.setPixelRatio(ivgComputeViewerPixelRatio(global, coarsePointer));
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
		const topicTfEl = $('view3d-use-topic-tf-only');
		const topicTfOnly = !!(topicTfEl && topicTfEl.checked);
		const tf3dOpts = (this.opts && this.opts.tf3d) || {};
		this.tfClient3d = new IvgRos3dTfClient(ros, fixedFrame, {
			...tf3dOpts,
			useTopicTfOnly: topicTfOnly ? true : tf3dOpts.useTopicTfOnly,
			preferRos2TfClient: topicTfOnly ? false : tf3dOpts.preferRos2TfClient
		});
		const maxPts = Math.max(1000, parseInt(($('pc-max') && $('pc-max').value) || '32000', 10) || 32000);
		const pszEl = $('view3d-point-size');
		const ptSize = Math.max(0.01, parseFloat((pszEl && pszEl.value) || '0.05') || 0.05);
		const pcThrottleEl = $('pc-throttle-ms');
		const pcRatioEl = $('pc-msg-ratio');
		const pcThrottleMsRaw = Math.max(0, parseInt((pcThrottleEl && pcThrottleEl.value) || '120', 10) || 0);
		const pcMsgRatioRaw = Math.max(1, parseInt((pcRatioEl && pcRatioEl.value) || '2', 10) || 1);
		const effectiveMaxPts = coarsePointer ? Math.min(maxPts, 4500) : maxPts;
		const pcThrottleMs = coarsePointer ? Math.max(pcThrottleMsRaw, 220) : pcThrottleMsRaw;
		const pcMsgRatio = coarsePointer ? Math.max(pcMsgRatioRaw, 4) : pcMsgRatioRaw;
		const selfSession = this;
		if (pcn) {
			const basePixelSize = Math.max(2, Math.min(24, Math.round(ptSize * 120)));
			const pcPixelSize = coarsePointer ? 1 : basePixelSize;
			this.ros3dPointCloud2 = new IvgRosPointCloudClient({
				ros,
				tfClient: this.tfClient3d,
				rootObject: this.viewer3d.scene,
				topic: pcn,
				max_pts: effectiveMaxPts,
				messageRatio: pcMsgRatio,
				throttle_rate: pcThrottleMs,
				pointSizePixels: pcPixelSize,
				opacity: coarsePointer ? 0.9 : 0.72,
				hintHost: host
			});
			showView3dPc2Hint(host, `<strong>加载点云</strong>：首帧大图仍可能需数秒。当前 <strong>节流 ${pcThrottleMs} ms</strong>、<strong>隔帧 ${pcMsgRatio}</strong>；若仍慢请调大节流或调低“最大点数”。`);
			this._pcAutoFrameFallbackTimer = setInterval(() => {
				const pc2 = selfSession.ros3dPointCloud2;
				if (!pc2 || !pc2.__ivgPc2Sig || !pc2.__ivgPc2Sig.got) return;
				const pcSceneNode = pc2.points && pc2.points.sn;
				const pcFrame = normalizeFrameId(pc2.__ivgPc2Frame || (pcSceneNode && pcSceneNode.frameID));
				if (!pcFrame) return;
				if (pcSceneNode && !selfSession._pcCameraPrimed && !selfSession._markerCameraPrimed) {
					selfSession._focusViewerOnObject(pcSceneNode, { distanceFactor: 2.4, minDistance: 0.42 });
					selfSession._pcCameraPrimed = true;
				}
				if (pcSceneNode && pcSceneNode.visible === true) {
					clearInterval(selfSession._pcAutoFrameFallbackTimer);
					selfSession._pcAutoFrameFallbackTimer = null;
				} else {
					const firstFrameAt = pc2.__ivgPc2FirstFrameAt || 0;
					if (firstFrameAt && Date.now() - firstFrameAt > 3000) {
						clearInterval(selfSession._pcAutoFrameFallbackTimer);
						selfSession._pcAutoFrameFallbackTimer = null;
					}
				}
			}, 1000);
		}
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
		if (wantUrdf && typeof ROS3D.Urdf === 'function' && typeof ROSLIB.UrdfModel === 'function') {
			this._defer(() => this._startUrdfStage($, host, fixedFrame), 3500);
		}
		if (pcn && ros && ros.isConnected) {
			const pc2FailAfterMs = 45000;
			const hintHost = $(this._view3dHostId);
			setTimeout(() => {
				const got = selfSession.ros3dPointCloud2 && selfSession.ros3dPointCloud2.__ivgPc2Sig && selfSession.ros3dPointCloud2.__ivgPc2Sig.got;
				if (!got) {
					const metaT = new ROSLIB.Topic({
						ros,
						name: pcn,
						messageType: 'sensor_msgs/msg/PointCloud2',
						throttle_rate: 0,
						queue_length: 0,
						queue_size: 100
					});
					metaT.getPublishers(
						pubs => {
							const n = Array.isArray(pubs) ? pubs.length : 0;
							if (!hintHost) return;
							if (n === 0) {
								showView3dPc2Hint(hintHost, `<strong>点云无数据</strong>：<code>${pcn}</code> 上未发现发布者。请确认相机/深度节点已启动，或在侧栏选择实际在发布的 <code>sensor_msgs/msg/PointCloud2</code> 话题后再“启动 3D”。`);
							} else {
								showView3dPc2Hint(hintHost, `<strong>点云无数据</strong>：检测到 <code>${n}</code> 个发布者，但 <strong>${pc2FailAfterMs / 1000} 秒</strong>内浏览器仍未收到可渲染首帧。请查 rosbridge / 网关日志、消息体大小与 QoS；或降低点云分辨率以减轻单帧体积。`);
							}
						},
						() => {
							if (hintHost) {
								showView3dPc2Hint(hintHost, '<strong>点云无数据</strong>：无法查询发布者（rosapi 不可用或调用失败）。请确认 rosbridge 同进程已加载 <code>rosapi</code>，并查看浏览器控制台与 rosbridge 日志。');
							}
						}
					);
				} else {
					const pcSceneNode = selfSession.ros3dPointCloud2 && selfSession.ros3dPointCloud2.points && selfSession.ros3dPointCloud2.points.sn;
					const pcFrame = normalizeFrameId(pcSceneNode && pcSceneNode.frameID);
					if (pcSceneNode && pcSceneNode.visible !== true && pcFrame) {
						const tfOk = selfSession.tfClient3d && typeof selfSession.tfClient3d.hasTransformFor === 'function' && selfSession.tfClient3d.hasTransformFor(pcFrame);
						if (!tfOk && hintHost) {
							showView3dPc2Hint(hintHost, `<strong>点云首帧已到但不可见</strong>：已收到 <code>${pcn}</code> 数据，但浏览器端尚未建立 <code>${fixedFrame}</code> → <code>${pcFrame}</code> 的 TF 链。请确认相机/机械臂 TF 已发布到 <code>/tf</code> 或 <code>/tf_static</code>，或先把固定坐标系改为 <code>${pcFrame}</code> 进行验证。`);
						}
					}
				}
			}, pc2FailAfterMs);
		}
	};

	global.IVGView3DSession = {
		IvgRos3dView3dSession
	};
	global.IvgRos3dView3dSession = IvgRos3dView3dSession;
})(typeof window !== 'undefined' ? window : globalThis);
