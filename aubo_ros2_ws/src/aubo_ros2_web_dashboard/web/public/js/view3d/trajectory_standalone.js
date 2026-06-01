// trajectory_standalone.js — 纯 Three.js 轨迹 3D 查看器 (无模块依赖)
// 依赖: THREE 全局变量 (由 <script src="three.js"> 加载)
// 坐标映射: ROS (X-fwd, Y-left, Z-up) → Three.js (X-right, Y-up, Z-toward)
//   data.x → Three.js X,  data.z → Three.js Y (高度),  data.y → Three.js Z (深度)
var TrajectoryStandaloneViewer = (function() {
var THREE = window.THREE;

var DEEP_BG = new THREE.Color('#14141e');

class TrajectoryStandaloneViewer {
  constructor() {
    this._scene = null;
    this._camera = null;
    this._renderer = null;
    this._objects = [];       // persistent + trajectory
    this._ambient = [];       // always-present (grid, stars, ring, lights — never cleared)
    this._container = null;
    this._resizeObs = null;
    this._animateId = null;
    this._clock = new THREE.Clock();
    this._ring = null;        // base ring
    this._scanLine = null;    // scanning plane
  }

  init(containerId) {
    var el = document.getElementById(containerId);
    if (!el) return;
    this._container = el;
    var w = el.clientWidth || 600;
    var h = el.clientHeight || 360;

    // ── Scene ──────────────────────────────────
    this._scene = new THREE.Scene();
    this._scene.background = DEEP_BG;
    this._scene.fog = new THREE.Fog(DEEP_BG, 2.2, 8.5);

    // ── Camera ─────────────────────────────────
    this._camera = new THREE.PerspectiveCamera(48, w / Math.max(h, 1), 0.02, 15);
    this._camera.position.set(1.0, 0.75, 1.35);
    this._camera.lookAt(-0.15, 0.45, -0.12);

    // ── Renderer ───────────────────────────────
    this._renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, preserveDrawingBuffer: false });
    this._renderer.setSize(w, h);
    this._renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this._renderer.toneMapping = THREE.ACESFilmicToneMapping || 0;
    this._renderer.toneMappingExposure = 1.05;
    el.appendChild(this._renderer.domElement);

    // ── Lights ─────────────────────────────────
    var amb = new THREE.AmbientLight(0x7788aa, 0.45);
    this._scene.add(amb); this._ambient.push(amb);

    var key = new THREE.DirectionalLight(0xffeedd, 0.5);
    key.position.set(0.8, 1.5, 0.6);
    this._scene.add(key); this._ambient.push(key);

    var rim = new THREE.DirectionalLight(0x5577bb, 0.25);
    rim.position.set(-0.5, 0.5, -1);
    this._scene.add(rim); this._ambient.push(rim);

    var fill = new THREE.DirectionalLight(0x334466, 0.2);
    fill.position.set(0.3, -0.2, 0.8);
    this._scene.add(fill); this._ambient.push(fill);

    // ── Ground grid (holographic) ─────────────
    this._makeGrid();

    // ── Base ring (radar-style) ────────────────
    this._makeBaseRing();

    // ── Starfield depth particles ──────────────
    this._makeStarfield();

    // ── Scanning plane (subtle moving) ─────────
    this._makeScanPlane();

    // ── Origin marker ──────────────────────────
    var ax = new THREE.AxesHelper(0.16);
    this._scene.add(ax); this._ambient.push(ax);

    // ── Controls ───────────────────────────────
    this._setupControls();

    // ── Resize / animate ───────────────────────
    var self = this;
    this._resizeObs = new ResizeObserver(function() { self._resize(); });
    this._resizeObs.observe(el);
    this._animate();
  }

  /* ── ambient helpers ───────────────────────────────────── */

  _makeGrid() {
    // Primary grid — fine divisions
    var g1 = new THREE.GridHelper(2.2, 30, new THREE.Color('#2a2a3c'), new THREE.Color('#1a1a28'));
    g1.position.y = 0.001;
    this._scene.add(g1); this._ambient.push(g1);

    // Secondary — major divisions only (faint)
    var g2 = new THREE.GridHelper(2.2, 6, new THREE.Color('#3a3a50'), new THREE.Color('#1a1a28'));
    g2.position.y = 0.002;
    g2.material.opacity = 0.4; g2.material.transparent = true;
    this._scene.add(g2); this._ambient.push(g2);
  }

  _makeBaseRing() {
    var ringGeom = new THREE.TorusGeometry(0.55, 0.003, 16, 80);
    var ringMat  = new THREE.MeshStandardMaterial({
      color: 0x335577, roughness: 0.5, metalness: 0.7,
      emissive: 0x112233, emissiveIntensity: 0.5,
      transparent: true, opacity: 0.55, depthTest: true
    });
    this._ring = new THREE.Mesh(ringGeom, ringMat);
    this._ring.rotation.x = -Math.PI / 2;
    this._ring.position.y = 0.004;
    this._scene.add(this._ring); this._ambient.push(this._ring);

    // Inner ring
    var irGeom = new THREE.TorusGeometry(0.25, 0.002, 12, 48);
    var irMat  = new THREE.MeshStandardMaterial({
      color: 0x224466, roughness: 0.5, metalness: 0.7,
      emissive: 0x0a1a2a, emissiveIntensity: 0.4,
      transparent: true, opacity: 0.35, depthTest: true
    });
    var ir = new THREE.Mesh(irGeom, irMat);
    ir.rotation.x = -Math.PI / 2;
    ir.position.y = 0.003;
    this._scene.add(ir); this._ambient.push(ir);
  }

  _makeStarfield() {
    var n = 200;
    var geom = new THREE.BufferGeometry();
    var pos = new Float32Array(n * 3);
    for (var i = 0; i < n; i++) {
      pos[i*3]   = (Math.random() - 0.5) * 3.5;
      pos[i*3+1] = Math.random() * 2.2;
      pos[i*3+2] = (Math.random() - 0.5) * 3.5;
    }
    geom.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    var mat = new THREE.PointsMaterial({
      color: 0x445566, size: 0.005,
      depthTest: false, transparent: true, opacity: 0.5
    });
    var stars = new THREE.Points(geom, mat);
    this._scene.add(stars); this._ambient.push(stars);
  }

  _makeScanPlane() {
    // Subtle semi-transparent ground plane for depth reference
    this._scanLine = new THREE.Mesh(
      new THREE.PlaneGeometry(1.6, 1.6),
      new THREE.MeshBasicMaterial({
        color: 0x334466,
        transparent: true,
        opacity: 0.03,
        side: THREE.DoubleSide,
        depthTest: false,
        depthWrite: false
      })
    );
    this._scanLine.rotation.x = -Math.PI / 2;
    this._scanLine.position.y = 0.005;
    this._scanLine.renderOrder = 999;
    this._scene.add(this._scanLine);
    this._ambient.push(this._scanLine);
  }

  /* ── coordinate mapping ────────────────────────────────── */

  _rosToThree(x, y, z) {
    return new THREE.Vector3(x, z, y);
  }

  /* ── orbit controls (pointer) ─────────────────────────── */

  _setupControls() {
    var self = this;
    var cam = this._camera;
    var el = this._renderer.domElement;
    var isDown = false, lastX = 0, lastY = 0;
    // Store orbit target as instance property so _fitCamera can sync it
    this._orbitTarget = new THREE.Vector3(-0.15, 0.45, -0.12);
    var target = this._orbitTarget;

    var onDown = function(e) {
      isDown = true; lastX = e.clientX; lastY = e.clientY;
      el.setPointerCapture(e.pointerId);
      el.style.cursor = 'grabbing';
    };
    var onMove = function(e) {
      if (!isDown) return;
      var dx = e.clientX - lastX, dy = e.clientY - lastY;
      lastX = e.clientX; lastY = e.clientY;
      var rel = cam.position.clone().sub(target);
      rel.applyAxisAngle(new THREE.Vector3(0, 1, 0), -dx * 0.005);
      var right = new THREE.Vector3().crossVectors(rel.clone().normalize(), new THREE.Vector3(0, 1, 0)).normalize();
      rel.applyAxisAngle(right, -dy * 0.005);
      cam.position.copy(target.clone().add(rel));
      cam.lookAt(target);
    };
    var onUp = function() {
      isDown = false; el.style.cursor = 'grab';
    };
    var onWheel = function(e) {
      e.preventDefault();
      var f = e.deltaY > 0 ? 1.12 : 0.88;
      var rel = cam.position.clone().sub(target);
      var d = rel.length() * f;
      if (d > 0.06 && d < 12) cam.position.copy(target.clone().add(rel.normalize().multiplyScalar(d)));
    };

    // Ref: https://developer.mozilla.org/en-US/docs/Web/API/Element/pointercancel_event
    el.addEventListener('pointerdown', onDown);
    el.addEventListener('pointermove', onMove);
    el.addEventListener('pointerup', onUp);
    el.addEventListener('pointerleave', onUp);
    el.addEventListener('pointercancel', onUp);
    el.addEventListener('wheel', onWheel, { passive: false });
    el.style.cursor = 'grab';

    // Store for cleanup
    this._ctrlCleanup = function() {
      el.removeEventListener('pointerdown', onDown);
      el.removeEventListener('pointermove', onMove);
      el.removeEventListener('pointerup', onUp);
      el.removeEventListener('pointerleave', onUp);
      el.removeEventListener('pointercancel', onUp);
      el.removeEventListener('wheel', onWheel);
    };
  }

  /* ── resize ───────────────────────────────────────────── */

  _resize() {
    if (!this._container || !this._renderer || !this._camera) return;
    var w = this._container.clientWidth;
    var h = this._container.clientHeight;
    this._renderer.setSize(w, h);
    this._camera.aspect = w / Math.max(h, 1);
    this._camera.updateProjectionMatrix();
  }

  /* ── animate ──────────────────────────────────────────── */

  _animate() {
    var self = this;
    this._animateId = requestAnimationFrame(function() { self._animate(); });
    if (!this._renderer || !this._scene || !this._camera) return;

    // Subtle ring pulse
    var t = this._clock.getElapsedTime();
    if (this._ring) {
      var s = 1 + Math.sin(t * 0.7) * 0.015;
      this._ring.scale.setScalar(s);
      this._ring.material.opacity = 0.45 + Math.sin(t * 0.7) * 0.1;
    }
    // Subtle scan plane bob
    if (this._scanLine) {
      this._scanLine.position.y = 0.005 + Math.sin(t * 0.4) * 0.03;
      this._scanLine.material.opacity = 0.025 + Math.sin(t * 0.4) * 0.01;
    }

    this._renderer.render(this._scene, this._camera);
  }

  /* ── public API ──────────────────────────────────────── */

  /**
   * Load full trajectory with glow effect.
   * @param {Object} data — {x:[], y:[], z:[], ...}
   * @param {string} color — hex
   * @param {number} [glowWidth=4] — multiplier for glow line thickness
   */
  loadTrajectory(data, color) {
    this.clearTrajectories();
    color = color || '#4d96ff';
    if (!data || !data.x || data.x.length < 2) return;

    var pts = [];
    var step = Math.max(1, Math.floor(data.x.length / 2000)); // decimate for perf
    for (var i = 0; i < data.x.length; i += step)
      pts.push(this._rosToThree(data.x[i], data.y[i], data.z[i]));
    // Always include last point
    var last = this._rosToThree(data.x[data.x.length-1], data.y[data.y.length-1], data.z[data.z.length-1]);
    if (pts[pts.length-1].distanceToSquared(last) > 1e-8) pts.push(last);

    var geom = new THREE.BufferGeometry().setFromPoints(pts);

    // Glow layer — wide, transparent
    var glowMat = new THREE.LineBasicMaterial({
      color: color, linewidth: 1,
      depthTest: true, transparent: true, opacity: 0.18,
      depthWrite: false
    });
    // Clone geometry for glow so each Line owns its geometry (safe dispose)
    var glowLine = new THREE.Line(geom.clone(), glowMat);
    glowLine.renderOrder = 1;
    this._scene.add(glowLine);
    this._objects.push(glowLine);

    // Core line — bright, sharp
    var coreMat = new THREE.LineBasicMaterial({
      color: color, linewidth: 1,
      depthTest: true, transparent: true, opacity: 0.82
    });
    var coreLine = new THREE.Line(geom, coreMat);
    coreLine.renderOrder = 0;
    this._scene.add(coreLine);
    this._objects.push(coreLine);

    // Trail particles — sparse glow dots along trajectory
    var trailN = Math.min(pts.length, 120);
    var trailStep = Math.max(1, Math.floor(pts.length / trailN));
    var trailPositions = [];
    for (var i = 0; i < pts.length; i += trailStep)
      trailPositions.push(pts[i].x, pts[i].y, pts[i].z);
    var trailGeom = new THREE.BufferGeometry();
    trailGeom.setAttribute('position', new THREE.Float32BufferAttribute(trailPositions, 3));
    var trailMat = new THREE.PointsMaterial({
      color: color, size: 0.008,
      depthTest: true, transparent: true, opacity: 0.55,
      depthWrite: false, blending: THREE.AdditiveBlending
    });
    var trailDots = new THREE.Points(trailGeom, trailMat);
    trailDots.renderOrder = 2;
    this._scene.add(trailDots);
    this._objects.push(trailDots);

    // Start marker — green emissive
    var sd = new THREE.Mesh(
      new THREE.SphereGeometry(0.008, 12, 12),
      new THREE.MeshStandardMaterial({
        color: 0x22dd66, roughness: 0.3, metalness: 0.2,
        emissive: 0x22dd66, emissiveIntensity: 0.5, depthTest: true
      })
    );
    sd.position.copy(pts[0]);
    this._scene.add(sd); this._objects.push(sd);

    // Start halo ring
    var haloGeom = new THREE.TorusGeometry(0.014, 0.002, 8, 16);
    var haloMat  = new THREE.MeshBasicMaterial({ color: 0x22dd66, transparent: true, opacity: 0.5, depthTest: true, depthWrite: false });
    var halo = new THREE.Mesh(haloGeom, haloMat);
    halo.position.copy(pts[0]);
    this._scene.add(halo); this._objects.push(halo);

    // End marker — warm red emissive
    var ed = new THREE.Mesh(
      new THREE.SphereGeometry(0.01, 12, 12),
      new THREE.MeshStandardMaterial({
        color: 0xff5544, roughness: 0.3, metalness: 0.2,
        emissive: 0xff5544, emissiveIntensity: 0.5, depthTest: true
      })
    );
    ed.position.copy(pts[pts.length - 1]);
    this._scene.add(ed); this._objects.push(ed);

    this._fitCamera(pts);
  }

  /**
   * Load latte art segments with distinct colors.
   */
  loadLatteSegments(segments, colors) {
    this.clearLatteOnly();
    if (!segments || !segments.length) return;
    var self = this;

    segments.forEach(function(seg, i) {
      if (!seg.x || seg.x.length < 2) return;
      var pts = [];
      var step = Math.max(1, Math.floor(seg.x.length / 800));
      for (var j = 0; j < seg.x.length; j += step)
        pts.push(self._rosToThree(seg.x[j], seg.y[j], seg.z[j]));
      var lastP = self._rosToThree(seg.x[seg.x.length-1], seg.y[seg.y.length-1], seg.z[seg.z.length-1]);
      if (pts[pts.length-1].distanceToSquared(lastP) > 1e-8) pts.push(lastP);

      var c = (colors && colors[i]) || '#ff6b6b';
      var geom = new THREE.BufferGeometry().setFromPoints(pts);

      // Glow
      var gMat = new THREE.LineBasicMaterial({
        color: c, linewidth: 1, depthTest: true,
        transparent: true, opacity: 0.22, depthWrite: false
      });
      var gLine = new THREE.Line(geom.clone(), gMat);
      gLine._isLatte = true; gLine.renderOrder = 1;
      self._scene.add(gLine); self._objects.push(gLine);

      // Core
      var cMat = new THREE.LineBasicMaterial({
        color: c, linewidth: 1, depthTest: true,
        transparent: true, opacity: 0.9
      });
      var cLine = new THREE.Line(geom, cMat);
      cLine._isLatte = true; cLine.renderOrder = 0;
      self._scene.add(cLine); self._objects.push(cLine);

      // Segment start dot
      var dot = new THREE.Mesh(
        new THREE.SphereGeometry(0.007, 10, 10),
        new THREE.MeshStandardMaterial({
          color: c, roughness: 0.25, metalness: 0.15,
          emissive: c, emissiveIntensity: 0.5, depthTest: true
        })
      );
      dot.position.copy(pts[0]);
      dot._isLatte = true;
      self._scene.add(dot); self._objects.push(dot);
    });
  }

  /* ── cleanup ──────────────────────────────────────────── */

  clearLatteOnly() {
    var self = this;
    var toRemove = this._objects.filter(function(o) { return o._isLatte; });
    toRemove.forEach(function(o) {
      self._scene.remove(o);
      if (o.geometry) o.geometry.dispose();
      if (o.material) o.material.dispose();
    });
    this._objects = this._objects.filter(function(o) { return !o._isLatte; });
  }

  clearTrajectories() {
    this.clearLatteOnly();
    var self = this;
    this._objects.forEach(function(o) {
      self._scene.remove(o);
      if (o.geometry) o.geometry.dispose();
      if (o.material) {
        if (Array.isArray(o.material)) o.material.forEach(function(m) { m.dispose(); });
        else o.material.dispose();
      }
    });
    this._objects = [];
  }

  /* ── camera ───────────────────────────────────────────── */

  _fitCamera(pts) {
    if (!pts || !pts.length) return;
    var box = new THREE.Box3().setFromPoints(pts);
    var center = new THREE.Vector3(); box.getCenter(center);
    var size = new THREE.Vector3(); box.getSize(size);
    var maxDim = Math.max(size.x, size.y, size.z, 0.03);
    var dist = maxDim * 3.2;
    this._camera.position.set(
      center.x + dist * 0.55,
      center.y + dist * 0.5,
      center.z + dist * 0.7
    );
    this._camera.lookAt(center);
    // Sync orbit rotation center so drag-rotate stays around trajectory center
    if (this._orbitTarget) this._orbitTarget.copy(center);
  }

  /* ── destroy ──────────────────────────────────────────── */

  destroy() {
    if (this._animateId) cancelAnimationFrame(this._animateId);
    if (this._resizeObs) this._resizeObs.disconnect();
    if (this._ctrlCleanup) this._ctrlCleanup();
    this.clearTrajectories();

    // Dispose ambient objects
    var all = (this._ambient || []).concat(this._objects || []);
    var self = this;
    all.forEach(function(o) {
      if (o && o.geometry) o.geometry.dispose();
      if (o && o.material) {
        if (Array.isArray(o.material)) o.material.forEach(function(m) { m.dispose(); });
        else o.material.dispose();
      }
      if (o) self._scene.remove(o);
    });
    this._ambient = []; this._objects = [];

    if (this._renderer) {
      this._renderer.dispose();
      if (this._renderer.domElement && this._renderer.domElement.parentNode)
        this._renderer.domElement.parentNode.removeChild(this._renderer.domElement);
    }
    this._scene = null; this._camera = null; this._renderer = null;
  }
}

return TrajectoryStandaloneViewer;
})();
