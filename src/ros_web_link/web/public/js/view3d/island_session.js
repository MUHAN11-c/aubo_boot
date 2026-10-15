// island_session.js — 渲染新岛会话（three 0.164 + urdf-loader + foxglove 数据）
//
// IvgRos3dView3dSession 的现代替代（构造形状兼容：new SessionCtor(ros, getById, opts)）。
// 能力：URDF（/robot_description + /joint_states）、Marker/MarkerArray（按 ns/id 生命周期，
// pose 经 tf_buffer 挂到 fixedFrame）、PointCloud2、LaserScan、轨迹线。
// 全部数据走自研 foxglove 栈（ros_client + tf_buffer）；不依赖 rosbridge/rosapi/ros3d。
//
// 坐标约定：内容整体挂在 -90°x 旋转的 contentGroup（URDF z-up → three y-up 显示），
// marker/点云/URDF 的位姿计算全部在 z-up 语义内完成后放入 contentGroup。
//
// Marker 支持类型：ARROW/CUBE/SPHERE/CYLINDER/LINE_STRIP/LINE_LIST/CUBE_LIST/SPHERE_LIST/
// POINTS/TRIANGLE_LIST；TEXT/MESH 未用（warn 一次跳过）。action：ADD(0 修改)/DELETE(1)/
// DELETEALL(2)；lifetime>0 到期自动删除。

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoaderDefault from 'urdf-loader';
import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';
import { IvgFoxgloveTfClient } from './tf_buffer.js';
import { createModernPointCloudViewer } from './pointcloud_island.js';
import { logBus } from '../core/log-bus.js';

const MARKER_TYPE = {
	ARROW: 0, CUBE: 1, SPHERE: 2, CYLINDER: 3, LINE_STRIP: 4, LINE_LIST: 5,
	CUBE_LIST: 6, SPHERE_LIST: 7, POINTS: 8, TEXT: 9, MESH: 10, TRIANGLE_LIST: 11,
};
const MARKER_ACTION = { ADD: 0, DELETE: 1, DELETE_ALL: 2 };

function num(v, dflt) { return (typeof v === 'number' && isFinite(v)) ? v : dflt; }
function colorOf(c, dflt) {
	if (!c) return dflt;
	return new THREE.Color(num(c.r, 0), num(c.g, 0), num(c.b, 0));
}
function vec3Of(v) { return new THREE.Vector3(num(v.x, 0), num(v.y, 0), num(v.z, 0)); }
function quatOf(q) {
	const x = num(q.x, 0), y = num(q.y, 0), z = num(q.z, 0), w = num(q.w, 1);
	const len = Math.hypot(x, y, z, w) || 1;
	return new THREE.Quaternion(x / len, y / len, z / len, w / len);
}

class IvgIsland3dSession {
	constructor(ros, getById, opts) {
		// ros 参数仅为构造形状兼容（现代岛不需要 ROSLIB 连接）
		this.$ = getById || ((id) => document.getElementById(id));
		this.opts = opts || {};
		this._view3dHostId = this.opts.view3dHostId || 'vision-urdf-host';
		this._viewerInnerId = this.opts.viewerInnerId || (this._view3dHostId + '-inner');
		this.viewer3d = null;
		this.tfClient3d = null;
		this._subs = [];
		this._markers = new Map(); // ns:id → {group, lifetimeMs, expiresAt, frame}
		this._markerWarned = new Set();
		this._raf = 0;
		this._markerTimer = null;
		this._resizeObs = null;
		this._stats = { markerFrames: 0, pcFrames: 0, scanFrames: 0 };
	}

	start() {
		const $ = this.$;
		const host = $(this._view3dHostId);
		if (!host) return;
		this.stop();
		// 调试句柄（无 WebGL 环境的数据链验证 / 人工排查）
		globalThis.__ivgIslandSession = this;

		let inner = document.getElementById(this._viewerInnerId);
		if (!inner) {
			inner = document.createElement('div');
			inner.id = this._viewerInnerId;
			host.appendChild(inner);
		}
		host.setAttribute('data-ivg-hide-hints', '1');

		this._scene = new THREE.Scene();
		this._scene.background = new THREE.Color(this.opts.background || '#f5f3f0');
		this._camera = new THREE.PerspectiveCamera(50, 4 / 3, 0.02, 100);
		this._camera.position.set(2.2, 1.8, 2.2);
		this._camera.lookAt(0, 0.3, 0);
		try {
			this._renderer = new THREE.WebGLRenderer({ antialias: true });
			this._renderer.setPixelRatio(Math.min(2, globalThis.devicePixelRatio || 1));
			inner.appendChild(this._renderer.domElement);
		} catch (e) {
			this._renderer = null;
			logBus.addLog('warn', 'view3d', '[island] WebGL 不可用，仅运行数据链路: ' + (e.message || e));
		}
		if (this._renderer) {
			this._controls = new OrbitControls(this._camera, this._renderer.domElement);
			this._controls.enableDamping = true;
			this._controls.target.set(0, 0.3, 0);
		}

		this._scene.add(new THREE.AmbientLight(0xffffff, 0.6));
		const d1 = new THREE.DirectionalLight(0xffffff, 1.0);
		d1.position.set(3, 5, 2);
		this._scene.add(d1);
		const d2 = new THREE.DirectionalLight(0xffffff, 0.4);
		d2.position.set(-3, 2, -2);
		this._scene.add(d2);

		if (this.opts.gridEnabled !== false) {
			this._scene.add(new THREE.GridHelper(4, 40, 0xbbbbbb, 0xdddddd));
		}

		// 内容组：URDF z-up → three y-up（显示旋转）
		this._content = new THREE.Group();
		this._content.rotation.x = -Math.PI / 2;
		this._scene.add(this._content);
		this._markerRoot = new THREE.Group();
		this._content.add(this._markerRoot);
		this._trajGroup = new THREE.Group();
		this._content.add(this._trajGroup);

		this.viewer3d = { resize: (w, h) => this._layout(w, h) };
		this._layout();
		if (typeof ResizeObserver !== 'undefined') {
			this._resizeObs = new ResizeObserver(() => this._layout());
			this._resizeObs.observe(host);
		}
		const loop = () => {
			this._raf = requestAnimationFrame(loop);
			if (this._renderer) {
				this._controls.update();
				this._renderer.render(this._scene, this._camera);
			}
		};
		if (this._renderer) loop();

		// TF 客户端（鸭子类型与旧岛一致）
		let fixedFrame = (($('tf-fixed-frame') && $('tf-fixed-frame').value) || 'base_link').trim().replace(/^\/+/, '');
		if (!fixedFrame) fixedFrame = 'base_link';
		this._fixedFrame = fixedFrame;
		this.tfClient3d = new IvgFoxgloveTfClient(fixedFrame, (this.opts.tf3d || {}));

		const client = foxgloveRosClient();
		if (!client.isConnected) client.connect().catch(() => { });

		// ── URDF 阶段 ──
		this._loader = new URDFLoaderDefault();
		this._loader.packages = (pkg) => `/api/ivg/robot-mesh/${pkg}`;
		this._robot = null;
		this._urdfText = '';
		this._subs.push(client.subscribe('/robot_description', 'std_msgs/msg/String', (m) => {
			if (m.data && typeof m.data.data === 'string') this._loadUrdf(m.data.data);
		}, { dispatchMode: 'immediate' }));
		this._subs.push(client.subscribe('/joint_states', 'sensor_msgs/msg/JointState', (m) => {
			this._applyJoints(m.data);
		}, { dispatchMode: 'immediate' }));

		// ── Marker 阶段（forceUrdfOnly 也保留 marker，与旧 session 行为一致）──
		const mk = ((this.opts.mk) || (($('markers-topic') && $('markers-topic').value) || '/grasp_markers') || '').trim();
		if (mk) this._startMarkers(mk);

		// ── 点云阶段 ──
		const pc = ((this.opts.pc) || (($('pointcloud-topic') && $('pointcloud-topic').value) || '') || '').trim();
		if (pc) this._startPointCloud(pc);

		// ── LaserScan 阶段 ──
		const sn = ((this.opts.sn) || (($('scan3-topic') && $('scan3-topic').value) || '') || '').trim();
		if (sn) this._startLaserScan(sn);
	}

	_layout(wIn, hIn) {
		const host = this.$(this._view3dHostId);
		if (!host) return;
		const w = wIn || Math.max(120, host.clientWidth || 400);
		const h = hIn || Math.max(240, host.clientHeight || Math.round(w * 0.72));
		if (this._renderer) this._renderer.setSize(w, h, false);
		this._camera.aspect = w / h;
		this._camera.updateProjectionMatrix();
	}

	// ── URDF ──────────────────────────────────────────────────────────────

	_loadUrdf(text) {
		if (text === this._urdfText) return;
		this._urdfText = text;
		try {
			const robot = this._loader.parse(text);
			if (!robot) throw new Error('parse 返回空');
			if (this._robot) {
				this._content.remove(this._robot);
				this._disposeTree(this._robot);
			}
			this._robot = robot;
			this._content.add(robot);
			logBus.addLog('info', 'view3d', '[island] URDF 加载: ' + (robot.joints ? Object.keys(robot.joints).length : '?') + ' 关节');
		} catch (e) {
			logBus.addLog('warn', 'view3d', '[island] URDF 解析失败: ' + (e.message || e));
		}
	}

	_applyJoints(msg) {
		if (!this._robot || !msg || !Array.isArray(msg.name)) return;
		const values = {};
		for (let i = 0; i < msg.name.length; i++) {
			values[msg.name[i]] = msg.position && msg.position[i] !== undefined ? Number(msg.position[i]) : 0;
		}
		try { this._robot.setJointValues(values); } catch (_) { }
	}

	reloadUrdf() {
		const text = this._urdfText;
		this._urdfText = '';
		this._loadUrdf(text);
	}

	// ── Markers ───────────────────────────────────────────────────────────

	_startMarkers(topic) {
		const client = foxgloveRosClient();
		const isMarkerArray = !/marker$/.test(topic) || /array/i.test(topic);
		// 订阅两种类型都尝试：MarkerArray 优先（grasp_markers 为 MarkerArray）
		const type = isMarkerArray ? 'visualization_msgs/msg/MarkerArray' : 'visualization_msgs/msg/Marker';
		this._subs.push(client.subscribe(topic, type, (m) => {
			this._stats.markerFrames++;
			if (Array.isArray(m.data.markers)) {
				for (const mk of m.data.markers) this._ingestMarker(mk);
			} else if (m.data && m.data.header) {
				this._ingestMarker(m.data);
			}
		}, { dispatchMode: 'immediate' }));
		// 10Hz 位姿更新（tf 摆位 + lifetime 过期）
		this._markerTimer = setInterval(() => this._pumpMarkers(), 100);
		logBus.addLog('info', 'view3d', '[island] markers: ' + topic + ' (' + type + ')');
	}

	_ingestMarker(mk) {
		if (!mk || !mk.header) return;
		const key = `${num(mk.ns, '')}:${num(mk.id, 0)}`;
		const action = num(mk.action, 0);
		if (action === MARKER_ACTION.DELETE_ALL) {
			this._clearMarkers();
			return;
		}
		if (action === MARKER_ACTION.DELETE) {
			this._removeMarker(key);
			return;
		}
		const obj = this._buildMarkerObject(mk);
		if (!obj) return;
		this._removeMarker(key);
		const group = new THREE.Group();
		group.add(obj);
		this._markerRoot.add(group);
		const lifetimeSec = num(mk.lifetime, 0);
		this._markers.set(key, {
			group,
			frame: mk.header.frame_id || '',
			expiresAt: lifetimeSec > 0 ? Date.now() + lifetimeSec * 1000 : 0,
			pose: { p: vec3Of(mk.pose && mk.pose.position), q: quatOf(mk.pose && mk.pose.orientation) },
		});
		this._poseMarker(this._markers.get(key));
	}

	_poseMarker(entry) {
		if (!entry.frame || entry.frame === this._fixedFrame) {
			entry.group.position.copy(entry.pose.p);
			entry.group.quaternion.copy(entry.pose.q);
			return true;
		}
		const tf = this.tfClient3d.getTransform(entry.frame);
		if (!tf) return false;
		const t = tf.translation, r = tf.rotation;
		entry.group.position.set(t.x, t.y, t.z);
		entry.group.quaternion.set(r.x, r.y, r.z, r.w);
		// marker 自身位姿相对其 frame：tf 变换 ∘ marker pose
		const qT = new THREE.Quaternion(r.x, r.y, r.z, r.w);
		const pOff = entry.pose.p.clone().applyQuaternion(qT);
		entry.group.position.add(pOff);
		entry.group.quaternion.multiply(entry.pose.q);
		return true;
	}

	_pumpMarkers() {
		const now = Date.now();
		for (const [key, entry] of [...this._markers]) {
			if (entry.expiresAt && entry.expiresAt < now) {
				this._removeMarker(key);
				continue;
			}
			this._poseMarker(entry);
		}
	}

	_removeMarker(key) {
		const entry = this._markers.get(key);
		if (!entry) return;
		this._markerRoot.remove(entry.group);
		this._disposeTree(entry.group);
		this._markers.delete(key);
	}

	_clearMarkers() {
		for (const key of [...this._markers.keys()]) this._removeMarker(key);
	}

	_warnTypeOnce(type) {
		if (!this._markerWarned.has(type)) {
			this._markerWarned.add(type);
			logBus.addLog('warn', 'view3d', `[island] marker type ${type} 未实现，跳过`);
		}
	}

	_buildMarkerObject(mk) {
		const type = num(mk.type, -1);
		const color = colorOf(mk.color, new THREE.Color(0.5, 0.5, 0.5));
		const scale = vec3Of(mk.scale);
		const opacity = num(mk.color && mk.color.a, 1);
		const mat = () => new THREE.MeshStandardMaterial({ color, transparent: opacity < 1, opacity });
		const lineMat = () => new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity });

		switch (type) {
			case MARKER_TYPE.CUBE: {
				const g = new THREE.BoxGeometry(scale.x || 0.05, scale.y || 0.05, scale.z || 0.05);
				return new THREE.Mesh(g, mat());
			}
			case MARKER_TYPE.SPHERE: {
				const s = (scale.x || scale.y || scale.z || 0.05) / 2;
				return new THREE.Mesh(new THREE.SphereGeometry(s || 0.025, 20, 14), mat());
			}
			case MARKER_TYPE.CYLINDER: {
				const r = (scale.x || scale.y || 0.05) / 2;
				const h = scale.z || 0.1;
				const mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 24), mat());
				// ROS marker cylinder 沿 z 轴；three CylinderGeometry 沿 y 轴
				mesh.rotation.x = Math.PI / 2;
				return mesh;
			}
			case MARKER_TYPE.ARROW: {
				// 简化箭头：圆柱杆 + 圆锥头（沿 +x，符合 RViz 语义）
				const shaftR = (scale.y || 0.02) / 2;
				const headR = (scale.z || 0.04) / 2;
				const len = scale.x || 0.15;
				const headL = Math.min(len * 0.3, headR * 3);
				const group = new THREE.Group();
				const shaft = new THREE.Mesh(new THREE.CylinderGeometry(shaftR, shaftR, Math.max(1e-4, len - headL), 12), mat());
				shaft.rotation.z = -Math.PI / 2;
				shaft.position.x = (len - headL) / 2;
				const head = new THREE.Mesh(new THREE.ConeGeometry(headR, headL, 16), mat());
				head.rotation.z = -Math.PI / 2;
				head.position.x = len - headL / 2;
				group.add(shaft, head);
				return group;
			}
			case MARKER_TYPE.LINE_STRIP:
			case MARKER_TYPE.LINE_LIST: {
				const pts = (mk.points || []).map((p) => vec3Of(p));
				if (pts.length < 2) return null;
				if (type === MARKER_TYPE.LINE_STRIP) {
					const geo = new THREE.BufferGeometry().setFromPoints(pts);
					return new THREE.Line(geo, lineMat());
				}
				const pos = [];
				for (let i = 0; i + 1 < pts.length; i += 2) {
					pos.push(pts[i], pts[i + 1]);
				}
				if (!pos.length) return null;
				const geo = new THREE.BufferGeometry().setFromPoints(pos);
				return new THREE.LineSegments(geo, lineMat());
			}
			case MARKER_TYPE.SPHERE_LIST:
			case MARKER_TYPE.CUBE_LIST: {
				const group = new THREE.Group();
				const r = ((scale.x || scale.y || scale.z || 0.04) / 2) || 0.02;
				let i = 0;
				for (const p of (mk.points || [])) {
					let obj;
					if (type === MARKER_TYPE.SPHERE_LIST) {
						obj = new THREE.Mesh(new THREE.SphereGeometry(r, 12, 8), mat());
					} else {
						obj = new THREE.Mesh(new THREE.BoxGeometry(scale.x || 0.04, scale.y || 0.04, scale.z || 0.04), mat());
					}
					obj.position.copy(vec3Of(p));
					const c = mk.colors && mk.colors[i];
					if (c) obj.material.color = colorOf(c, color);
					group.add(obj);
					i++;
				}
				return group.children.length ? group : null;
			}
			case MARKER_TYPE.POINTS: {
				const pts = (mk.points || []).map((p) => vec3Of(p));
				if (!pts.length) return null;
				const geo = new THREE.BufferGeometry().setFromPoints(pts);
				const m = new THREE.PointsMaterial({ size: scale.x || 0.02, vertexColors: !!mk.colors && mk.colors.length > 0, color });
				if (mk.colors && mk.colors.length) {
					const colArr = new Float32Array(pts.length * 3);
					mk.colors.forEach((c, i) => {
						const cc = colorOf(c, color);
						colArr[i * 3] = cc.r; colArr[i * 3 + 1] = cc.g; colArr[i * 3 + 2] = cc.b;
					});
					geo.setAttribute('color', new THREE.BufferAttribute(colArr, 3));
				}
				return new THREE.Points(geo, m);
			}
			case MARKER_TYPE.TRIANGLE_LIST: {
				const ts = mk.points || [];
				if (ts.length < 3) return null;
				const pos = new Float32Array(ts.length * 3);
				const cols = (mk.colors && mk.colors.length === ts.length) ? new Float32Array(ts.length * 3) : null;
				for (let i = 0; i < ts.length; i++) {
					const p = vec3Of(ts[i]);
					pos[i * 3] = p.x; pos[i * 3 + 1] = p.y; pos[i * 3 + 2] = p.z;
					if (cols) {
						const cc = colorOf(mk.colors[i], color);
						cols[i * 3] = cc.r; cols[i * 3 + 1] = cc.g; cols[i * 3 + 2] = cc.b;
					}
				}
				const geo = new THREE.BufferGeometry();
				geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
				if (cols) geo.setAttribute('color', new THREE.BufferAttribute(cols, 3));
				const m = cols
					? new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide, transparent: opacity < 1, opacity })
					: new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, transparent: opacity < 1, opacity });
				return new THREE.Mesh(geo, m);
			}
			case MARKER_TYPE.TEXT:
			case MARKER_TYPE.MESH:
			default:
				this._warnTypeOnce(type);
				return null;
		}
	}

	// ── 点云 / LaserScan ──────────────────────────────────────────────────

	_startPointCloud(topic) {
		const client = foxgloveRosClient();
		const maxPts = 32000;
		this._pcViewer = createModernPointCloudViewer({ scene: this._content, maxPoints: maxPts, pointSize: 0.005 });
		this._pcFrame = '';
		this._subs.push(client.subscribe(topic, 'sensor_msgs/msg/PointCloud2', (m) => {
			this._stats.pcFrames++;
			this._pcFrame = (m.data.header && m.data.header.frame_id) || '';
			this._pcViewer.updatePoints(m.data);
			this._posePc();
		}, { dispatchMode: 'latest' }));
		logBus.addLog('info', 'view3d', '[island] pointcloud: ' + topic);
	}

	_posePc() {
		if (!this._pcViewer || !this._pcFrame) return;
		const tf = this._pcFrame === this._fixedFrame ? null : this.tfClient3d.getTransform(this._pcFrame);
		const pts = this._pcViewer.points;
		if (!tf) {
			pts.position.set(0, 0, 0);
			pts.quaternion.identity();
		} else {
			const t = tf.translation, r = tf.rotation;
			pts.position.set(t.x, t.y, t.z);
			pts.quaternion.set(r.x, r.y, r.z, r.w);
		}
	}

	_startLaserScan(topic) {
		const client = foxgloveRosClient();
		this._scanViewer = createModernPointCloudViewer({ scene: this._content, maxPoints: 8192, pointSize: 0.01 });
		this._scanFrame = '';
		this._subs.push(client.subscribe(topic, 'sensor_msgs/msg/LaserScan', (m) => {
			this._stats.scanFrames++;
			const d = m.data;
			this._scanFrame = (d.header && d.header.frame_id) || '';
			const ranges = d.ranges || [];
			const aMin = num(d.angle_min, -Math.PI / 2);
			const aInc = num(d.angle_increment, Math.PI / 360);
			const rMax = num(d.range_max, 10);
			const pts = [];
			for (let i = 0; i < ranges.length; i++) {
				const r = Number(ranges[i]);
				if (!isFinite(r) || r <= num(d.range_min, 0) || r > rMax) continue;
				const a = aMin + i * aInc;
				pts.push(new THREE.Vector3(r * Math.cos(a), r * Math.sin(a), 0));
			}
			const geo = new THREE.BufferGeometry().setFromPoints(pts);
			// 复用渲染器接口：直接替换内部几何太粗暴，这里走一次性 Points 由生命周期管理
			if (this._scanObj) {
				this._content.remove(this._scanObj);
				this._scanObj.geometry.dispose();
			}
			this._scanObj = new THREE.Points(geo, new THREE.PointsMaterial({ color: 0xff4444, size: 0.01, sizeAttenuation: true }));
			this._content.add(this._scanObj);
			const tf = this._scanFrame === this._fixedFrame ? null : this.tfClient3d.getTransform(this._scanFrame);
			if (tf) {
				const t = tf.translation, r = tf.rotation;
				this._scanObj.position.set(t.x, t.y, t.z);
				this._scanObj.quaternion.set(r.x, r.y, r.z, r.w);
			}
		}, { dispatchMode: 'latest' }));
		logBus.addLog('info', 'view3d', '[island] laserscan: ' + topic);
	}

	// ── 轨迹线（latte 用）────────────────────────────────────────────────

	addTrajectoryLine(waypoints, colorHex) {
		if (!this._trajGroup || !Array.isArray(waypoints) || waypoints.length < 2) return;
		const pts = waypoints.map((wp) => {
			const p = wp.position || wp;
			return new THREE.Vector3(num(p.x, 0), num(p.y, 0), num(p.z, 0));
		});
		const geo = new THREE.BufferGeometry().setFromPoints(pts);
		this._trajGroup.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color: colorHex || '#ff6b6b' })));
	}

	clearTrajectoryLines() {
		if (!this._trajGroup) return;
		for (const child of [...this._trajGroup.children]) {
			this._trajGroup.remove(child);
			child.geometry?.dispose?.();
			child.material?.dispose?.();
		}
	}

	// ── 生命周期 ──────────────────────────────────────────────────────────

	_disposeTree(obj) {
		obj.traverse?.((child) => {
			child.geometry?.dispose?.();
			if (child.material) {
				const mats = Array.isArray(child.material) ? child.material : [child.material];
				for (const m of mats) m.dispose?.();
			}
		});
	}

	stop() {
		if (this._raf) {
			cancelAnimationFrame(this._raf);
			this._raf = 0;
		}
		if (this._markerTimer) {
			clearInterval(this._markerTimer);
			this._markerTimer = null;
		}
		for (const h of this._subs) {
			try { h.cancel(); } catch (_) { }
		}
		this._subs = [];
		if (this._resizeObs) {
			try { this._resizeObs.disconnect(); } catch (_) { }
			this._resizeObs = null;
		}
		if (this.tfClient3d) {
			try { this.tfClient3d.dispose(); } catch (_) { }
			this.tfClient3d = null;
		}
		if (this._controls) {
			try { this._controls.dispose(); } catch (_) { }
			this._controls = null;
		}
		this._clearMarkers();
		if (this._pcViewer) {
			try { this._pcViewer.dispose(); } catch (_) { }
			this._pcViewer = null;
		}
		if (this._scanViewer) {
			try { this._scanViewer.dispose(); } catch (_) { }
			this._scanViewer = null;
		}
		if (this._scanObj) {
			this._scanObj.geometry?.dispose?.();
			this._scanObj = null;
		}
		if (this._robot) {
			this._disposeTree(this._robot);
			this._robot = null;
		}
		this.clearTrajectoryLines();
		if (this._renderer) {
			try { this._renderer.dispose(); } catch (_) { }
			this._renderer.domElement?.remove();
			this._renderer = null;
		}
		this._scene = null;
		this.viewer3d = null;
		this._urdfText = '';
	}

	debugState() {
		const joints = {};
		if (this._robot && this._robot.joints) {
			for (const [name, j] of Object.entries(this._robot.joints)) joints[name] = j.angle ?? null;
		}
		return {
			hasRobot: !!this._robot,
			hasRenderer: !!this._renderer,
			fixedFrame: this._fixedFrame,
			jointCount: Object.keys(joints).length,
			joints,
			markerCount: this._markers.size,
			pointCount: this._pcViewer ? this._pcViewer.pointCount : 0,
			stats: { ...this._stats },
		};
	}
}

const IVGIsland3dSession = { IvgIsland3dSession };
globalThis.IVGIsland3dSession = IVGIsland3dSession;
globalThis.IvgIsland3dSession = IvgIsland3dSession;
export { IvgIsland3dSession, IVGIsland3dSession };
