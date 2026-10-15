// console/teach.js — TCP 拖拽示教（Rebot-Arm rebot-sim.js:963-1390 同构移植，three 侧）
//
// 组成：屏幕拖拽标记（DOM 环）+ 相机朝向拖拽平面（一次构造）+ 半球夹紧 +
// DLS IK 伺服（ikcore 纯核）+ 航点录制/导出。与 viz 面板的接线：
//   VizPanel 提供 robot（URDFRobot）、robotFrame（z-up 容器）、camera、scene、
//   bus 命令总线；本模块产出 joint 写回（经 bus，source='drag'/'drag-settle'）。
// 坐标约定：容器内直接使用 ROS z-up 坐标（threeToRos 由容器旋转承担）。

import * as THREE from 'three';

import {
  clampToWorkspaceEnvelope,
  createIKSolver,
  createTeachRecorder,
  estimateWorkspaceEnvelope,
  SETTLE_TIMEOUT_MS,
  SETTLE_TARGET_ERROR,
} from './ikcore.js';
import { emitCommand } from './bus.js';

const MARKER_SIZE = 22;

class TeachController {
  /**
   * @param {{robot: Object, robotFrame: Object, camera: Object, scene: Object,
   *          canvas: Object, armJointNames: string[], enabledGuard: function}} deps
   */
  constructor(deps) {
    this.deps = deps;
    this.enabled = false;
    this.recording = false;
    this.draggingTcp = false;
    this.dragSettling = false;
    this._dragPlane = null;
    this._dragTarget = new THREE.Vector3();
    this._dragTargetClamped = false;
    this._dragLastTime = 0;
    this._settleStart = 0;
    this._ray = new THREE.Raycaster();
    this._recorder = createTeachRecorder();
    this._solver = null;
    this._envelope = null;
    this._angles = {};          // 当前关节角（名字→弧度）
    this._marker = null;
    this._targetGhost = null;   // 目标球
    this._errorLine = null;
    this._loopRunning = false;
    this.stats = { waypoints: 0, dragError: null, clamped: false };
    this._buildMarker();
  }

  // ── 关节/包络准备 ─────────────────────────────────────────────────────

  prepare() {
    const { robot, armJointNames } = this.deps;
    const joints = [];
    for (const name of armJointNames) {
      const j = robot?.joints?.[name]
        || robot?.getObjectByName?.(name);   // clone 后 joints 失效回退
      const lo = Number(j?.limit?.lower ?? -Math.PI);
      const hi = Number(j?.limit?.upper ?? Math.PI);
      joints.push({ name, lower: lo, upper: hi });
      this._angles[name] = Number(j?.angle ?? 0) || 0;
    }
    this._solver = createIKSolver({
      fk: (angles) => this.getTcp(angles),
      joints,
    });
    this._envelope = estimateWorkspaceEnvelope({
      fk: (angles) => this.getTcp(angles),
      joints,
    });
    this._buildEnvelope();
  }

  /**
   * 工作空间包络线框（rebot-sim.js:352-393 同构）：5 条水平圆环（高度比
   * 0/0.25/0.5/0.75/0.95，半径 R·√(1-r²)）+ 12 条经线弧（每 30°，主/辅透明度）。
   * 坐标：容器内 three（x=ROS x，y=ROS z 高度，z=-ROS y）。
   */
  _buildEnvelope() {
    const { scene } = this.deps;
    const { radius: R, heightLimit: h } = this._envelope;
    if (!scene || !R || !h) return;
    if (this._envelopeGroup) {
      scene.remove(this._envelopeGroup);
      for (const c of this._envelopeGroup.children) {
        c.geometry?.dispose?.();
        c.material?.dispose?.();
      }
    }
    const group = new THREE.Group();
    const ringPts = (rr, y) => {
      const pts = [];
      for (let i = 0; i <= 64; i++) {
        const a = (i / 64) * Math.PI * 2;
        pts.push(new THREE.Vector3(Math.cos(a) * rr, y, Math.sin(a) * rr));
      }
      return pts;
    };
    for (const r of [0, 0.25, 0.5, 0.75, 0.95]) {
      const rr = R * Math.sqrt(Math.max(0, 1 - r * r));
      group.add(new THREE.LineLoop(
        new THREE.BufferGeometry().setFromPoints(ringPts(rr, r * h)),
        new THREE.LineBasicMaterial({ color: 0x33d6b0, transparent: true, opacity: 0.32 })));
    }
    for (let k = 0; k < 12; k++) {
      const az = (k / 12) * Math.PI * 2;
      const pts = [];
      for (let i = 0; i <= 24; i++) {
        const r = i / 24;
        const pr = R * Math.sqrt(Math.max(0, 1 - r * r));
        pts.push(new THREE.Vector3(Math.cos(az) * pr, r * h, Math.sin(az) * pr));
      }
      group.add(new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color: 0x33d6b0,
          transparent: true, opacity: k % 3 === 0 ? 0.3 : 0.16 })));
    }
    group.visible = false;
    this._envelopeGroup = group;
    scene.add(group);
  }

  get showEnvelope() { return this._showEnvelope !== false; }
  set showEnvelope(v) {
    this._showEnvelope = !!v;
    this._syncEnvelopeVisible();
  }

  _syncEnvelopeVisible() {
    if (this._envelopeGroup) {
      this._envelopeGroup.visible = this.enabled && this.showEnvelope;
    }
  }

  /** 当前 TCP（ROS z-up 米制；angles 省略用现值） */
  getTcp(angles) {
    const { robot } = this.deps;
    if (!robot) return { x: 0, y: 0, z: 0 };
    const names = this._solver ? this._solver.names : Object.keys(this._angles);
    const apply = angles || this._angles;
    for (const n of names) {
      const j = robot.joints?.[n] || robot.getObjectByName?.(n);
      try { j?.setJointValue?.(apply[n] ?? 0); } catch (_) { /* keep */ }
    }
    robot.updateMatrixWorld(true);
    const tcpObj = robot.getObjectByName('panda_link8')
      || robot.getObjectByName('end_link')
      || robot.getObjectByName('link6')
      || robot.children[robot.children.length - 1];
    const p = new THREE.Vector3();
    if (tcpObj) tcpObj.getWorldPosition(p);
    // robotFrame 在父级，容器旋转承担 z-up：世界 y 即 ROS z
    return { x: p.x, y: -p.z, z: p.y };
  }

  /** 写回关节（经总线；source 标记） */
  _applyAngles(angles, source) {
    Object.assign(this._angles, angles);
    const { robot } = this.deps;
    for (const [name, v] of Object.entries(angles)) {
      const j = robot?.joints?.[name] || robot?.getObjectByName?.(name);
      try { j?.setJointValue?.(v); } catch (_) { /* keep */ }
    }
    emitCommand({ type: 'joint', name: null, value: null, joints: { ...angles },
      source, label: 'tcp-drag' });
  }

  // ── 标记 / 目标球 / 误差线 ────────────────────────────────────────────

  _buildMarker() {
    const host = this.deps.canvas.parentElement;
    const el = document.createElement('div');
    el.id = 'tcp-drag-marker';
    el.style.cssText = `position:absolute;width:${MARKER_SIZE}px;height:${MARKER_SIZE}px;` +
      'border-radius:50%;pointer-events:auto;touch-action:none;display:none;z-index:5;' +
      'border:2px solid var(--teal);box-shadow:0 0 10px rgba(51,214,176,.55);' +
      'background:radial-gradient(circle,rgba(51,214,176,.28) 0%,transparent 70%);' +
      'cursor:grab;transform:translate(-50%,-50%);';
    el.addEventListener('pointerdown', (e) => this._startDrag(e));
    el.addEventListener('pointermove', (e) => this._moveDrag(e));
    el.addEventListener('pointerup', (e) => this._endDrag(e));
    el.addEventListener('pointercancel', (e) => this._endDrag(e));
    host.appendChild(el);
    this._marker = el;

    const { scene } = this.deps;
    this._targetGhost = new THREE.Mesh(
      new THREE.SphereGeometry(0.018, 12, 10),
      new THREE.MeshBasicMaterial({ color: 0xf2a541, transparent: true, opacity: 0.85 }));
    this._targetGhost.visible = false;
    scene.add(this._targetGhost);
    this._errorLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineBasicMaterial({ color: 0xff6b5f, transparent: true, opacity: 0 }));
    this._errorLine.visible = false;
    scene.add(this._errorLine);
  }

  _updateMarkerScreen() {
    if (!this.enabled) return;
    const { camera, canvas } = this.deps;
    const tcp = this.getTcp();
    // TCP 读数 HUD（rebot-sim.js:1767-1779 同构：ROS 系毫米 + 夹紧超限变红；100ms 节流）
    const now = performance.now();
    if (now - (this._lastHudAt || 0) > 100) {
      this._lastHudAt = now;
      this.stats.tcp = { x: +tcp.x.toFixed(4), y: +tcp.y.toFixed(4), z: +tcp.z.toFixed(4) };
      const hud = document.getElementById('hud-tcp');
      if (hud) {
        hud.textContent = `TCP ${(tcp.x * 1000).toFixed(0)},${(tcp.y * 1000).toFixed(0)},${(tcp.z * 1000).toFixed(0)} mm`;
        hud.style.color = this.stats.clamped ? 'var(--red)' : 'var(--muted)';
      }
    }
    const world = new THREE.Vector3(tcp.x, tcp.z, -tcp.y);  // ROS→容器内 three
    const projected = world.clone().project(camera);
    const r = canvas.getBoundingClientRect();
    const host = this._marker.parentElement.getBoundingClientRect();
    const x = host.left - r.left + ((projected.x + 1) / 2) * r.width;
    const y = host.top - r.top + ((1 - projected.y) / 2) * r.height;
    this._marker.style.left = x + 'px';
    this._marker.style.top = y + 'px';
    this._marker.style.display = projected.z < 1 ? 'block' : 'none';
    if (this._targetGhost.visible) {
      const t = this._dragTarget;
      const tw = new THREE.Vector3(t.x, t.z, -t.y);
      this._targetGhost.position.copy(tw);
      const mat = this._targetGhost.material;
      mat.color.setHex(this._dragTargetClamped ? 0xff6b5f : 0xf2a541);
      mat.opacity = this._dragTargetClamped ? 0.95 : 0.85;
    }
  }

  // ── 开关 ───────────────────────────────────────────────────────────────

  toggle(on) {
    this.enabled = on;
    this._marker.style.display = on ? 'block' : 'none';
    if (!on) {
      this._targetGhost.visible = false;
      this._errorLine.visible = false;
      this.draggingTcp = false;
      this.dragSettling = false;
    } else if (!this._solver) {
      this.prepare();
    }
    this._syncEnvelopeVisible();
  }

  toggleRecording(on) {
    this.recording = on;
    if (on) {
      this._recorder.reset();
      if (!this.enabled) this.toggle(true);
      this._recorder.record({ joints: { ...this._angles }, tcp: this.getTcp() }, true);
    }
    this.stats.waypoints = this._recorder.waypoints.length;
  }

  // ── 拖拽（参考 startTcpDrag/moveTcpDrag/endTcpDrag） ───────────────────

  _startDrag(event) {
    if (!this.enabled) return;
    event.preventDefault();
    event.stopPropagation();
    this._marker.setPointerCapture(event.pointerId);
    this.draggingTcp = true;
    this._marker.classList.add('dragging');
    // 平面一次构造：法向=相机前向，过当前 TCP
    const tcp = this.getTcp();
    const world = new THREE.Vector3(tcp.x, tcp.z, -tcp.y);
    const normal = new THREE.Vector3();
    this.deps.camera.getWorldDirection(normal);
    this._dragPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, world);
    this._dragTarget.copy(world);
    this._dragTargetClamped = false;
    this._dragLastTime = performance.now();
    this._targetGhost.visible = true;
    this._errorLine.visible = true;
    this._recorder.record({ joints: { ...this._angles }, tcp }, true);
    this._ensureLoop();
    this._moveDrag(event);
  }

  _planeHit(event) {
    const { camera, canvas } = this.deps;
    const r = canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((event.clientX - r.left) / r.width) * 2 - 1,
      -((event.clientY - r.top) / r.height) * 2 + 1);
    this._ray.setFromCamera(ndc, camera);
    const hit = new THREE.Vector3();
    return this._ray.ray.intersectPlane(this._dragPlane, hit) ? hit : null;
  }

  _moveDrag(event) {
    if (!this.draggingTcp || !this._solver) return;
    const hit = this._planeHit(event);
    if (!hit) return;
    // 容器内 three → ROS z-up，再半球夹紧
    const ros = { x: hit.x, y: -hit.z, z: hit.y };
    const bounded = clampToWorkspaceEnvelope(ros, this._envelope);
    this._dragTargetClamped = bounded.clamped;
    this._dragTarget.set(bounded.point.x, bounded.point.z, -bounded.point.y);
    this.stats.clamped = bounded.clamped;

    const now = performance.now();
    const dt = Math.min(0.05, Math.max(0.012, (now - this._dragLastTime) / 1000 || 0.016));
    this._dragLastTime = now;
    const substeps = Math.max(1, Math.ceil(dt / 0.016));
    let result = null;
    for (let i = 0; i < substeps; i++) {
      result = this._solver.servoStep(this._angles, bounded.point, dt / substeps);
      this._applyAngles(result.angles, 'drag');
    }
    if (result) this.stats.dragError = result.error;
    this._updateErrorLine();
    if (this.recording) {
      this._recorder.record({ joints: { ...this._angles }, tcp: this.getTcp() });
      this.stats.waypoints = this._recorder.waypoints.length;
    }
  }

  _endDrag(event) {
    if (!this.draggingTcp) return;
    this.draggingTcp = false;
    this._marker.classList.remove('dragging');
    const tcp = this.getTcp();
    const t = this._dragTarget;
    const rosT = { x: t.x, y: -t.z, z: t.y };
    const dist = Math.hypot(rosT.x - tcp.x, rosT.y - tcp.y, rosT.z - tcp.z);
    if (dist > SETTLE_TARGET_ERROR) {
      this.dragSettling = true;
      this._settleStart = performance.now();
      this._settleTarget = rosT;
      this._ensureLoop();
    } else {
      this._finishDrag();
    }
    if (this.recording) {
      this._recorder.record({ joints: { ...this._angles }, tcp }, true);
      this.stats.waypoints = this._recorder.waypoints.length;
    }
  }

  _finishDrag() {
    this.dragSettling = false;
    this._targetGhost.visible = false;
    this._errorLine.visible = false;
  }

  _updateErrorLine() {
    const tcp = this.getTcp();
    const t = this._dragTarget;
    const rosT = { x: t.x, y: -t.z, z: t.y };
    const err = Math.hypot(rosT.x - tcp.x, rosT.y - tcp.y, rosT.z - tcp.z);
    this._errorLine.geometry.setFromPoints([
      new THREE.Vector3(tcp.x, tcp.z, -tcp.y),
      new THREE.Vector3(t.x, t.y, t.z),
    ]);
    this._errorLine.material.opacity = Math.min(0.9, Math.max(0.28, err / 0.08));
    this._errorLine.visible = err > 0.001;
  }

  // ── 帧循环（settle + 标记投影） ───────────────────────────────────────

  _ensureLoop() {
    if (this._loopRunning) return;
    this._loopRunning = true;
    const loop = () => {
      if (!this.enabled && !this.dragSettling) {
        this._loopRunning = false;
        return;
      }
      requestAnimationFrame(loop);
      if (this.dragSettling && this._solver) {
        const now = performance.now();
        const elapsed = now - this._settleStart;
        const r = this._solver.servoStep(this._angles, this._settleTarget, 0.016);
        this._applyAngles(r.angles, 'drag-settle');
        this.stats.dragError = r.error;
        this._updateErrorLine();
        if (r.reached || elapsed >= SETTLE_TIMEOUT_MS) {
          this._recorder.record(
            { joints: { ...this._angles }, tcp: this.getTcp() }, true);
          this.stats.waypoints = this._recorder.waypoints.length;
          this._finishDrag();
        }
      }
      this._updateMarkerScreen();
    };
    requestAnimationFrame(loop);
  }

  // ── 导出（waypoints_v1） ──────────────────────────────────────────────

  exportWaypoints(jointNames) {
    const wps = this._recorder.waypoints;
    if (!wps.length) return null;
    return {
      format: 'rwl_teach_waypoints_v1',
      frame_id: 'base',
      joint_names: jointNames,
      count: wps.length,
      waypoints: wps.map((w) => ({
        time_from_start: { sec: Math.floor(w.t / 1000), nanosec: Math.round(w.t % 1000 * 1e6) },
        positions: jointNames.map((n) => Number(w.joints[n] || 0)),
        tcp: { x: +w.tcp.x.toFixed(4), y: +w.tcp.y.toFixed(4), z: +w.tcp.z.toFixed(4) },
      })),
    };
  }

  get recorder() { return this._recorder; }
}

export { TeachController };
