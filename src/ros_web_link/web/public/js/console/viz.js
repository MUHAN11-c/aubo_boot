// console/viz.js — 常用可视化内容实时渲染（RViz2 风格 Displays + 工具栏）
//
// 参考 RViz2 的 Displays 面板（每显示项：启用开关 / 话题选择 / 状态 / 属性）与
// 工具栏（测距、Publish Point、视角保存恢复），rqt 的多插件观测思路：
//   3D：Grid、TF（轴+名签可开关）、PointCloud2（点大小属性）、LaserScan 3D 投影、
//       Marker(SPHERE_LIST/CUBE/TEXT)、Path、PoseArray/PoseStamped
//   2D：Image(mono8/rgb8/bgr8)、LaserScan 扇形、JointState 条
//   工具：测距（两点击）、发布点（发布 PointStamped，走 clientPublish 链路）、
//         视角保存/恢复（localStorage）+ 预设机位；ROS Time（各流 header 最新）
// 渲染优化：three 脏标记 RAF；图像/激光节流 100ms；点云 BufferGeometry 复用。
// 快照 window.__rwl_viz_stats()；显示配置持久化 localStorage('rwl-viz-cfg-v2')。

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoaderDefault from 'urdf-loader';

import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';
import { emitCommand, onCommand } from './bus.js';
import { TeachController } from './teach.js';
import { foxgloveTfTree, lookupLatest } from '../view3d/tf_buffer.js';
import { parsePointCloud2 } from '../view3d/pointcloud_parse.js';
import { covarianceEllipse, createJointMotion, createOdomTrail } from './viz_core.js';

const $ = (id) => document.getElementById(id);
const MAX_POINTS = 4096;
const CFG_KEY = 'rwl-viz-cfg-v2';

function makeTextSprite(text, color = '#8b93a7', size = 42) {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 64;
  const ctx = c.getContext('2d');
  ctx.font = `${size}px ui-monospace, monospace`;
  ctx.fillStyle = color;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, 128, 32);
  const tex = new THREE.CanvasTexture(c);
  tex.minFilter = THREE.LinearFilter;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true }));
  sp.scale.set(0.45, 0.11, 1);
  return sp;
}

/** 默认 Displays 配置（localStorage 覆盖） */
function defaultCfg() {
  return {
    grid: { on: true },
    robot: { on: true, topic: '/robot_description', ghost: false },
    map: { on: false, topic: '/map' },
    tf: { on: true, labels: true, topic: '/tf' },
    cloud: { on: true, topic: '/rwl_viz/points', size: 0.035 },
    scan3d: { on: true, topic: '/rwl_viz/scan' },
    markers: { on: true, topic: '/rwl_viz/markers' },
    path: { on: true, topic: '/rwl_viz/path' },
    poses: { on: true, topic: '/rwl_viz/pose_array' },
    odom: { on: true, topic: '/rwl_viz/odom', trail: true },
    range: { on: true, topic: '/rwl_viz/range' },
    wrench: { on: true, topic: '/rwl_viz/wrench', scale: 0.2 },
    posecov: { on: true, topic: '/rwl_viz/pose_cov' },
    image: { on: true, topic: '/rwl_viz/image' },
    imagec: { on: false, topic: '/rwl_viz/image_compressed' },
  };
}

class VizPanel {
  constructor() {
    this.cfg = { ...defaultCfg(), ...JSON.parse(localStorage.getItem(CFG_KEY) || '{}') };
    this.stats = {
      connectedAt: null, rosTimeNs: null,
      fps: null, objectCount: null,
      cloud: { points: 0, lastAt: null },
      image: { w: 0, h: 0, enc: '', frames: 0, lastAt: null },
      scan: { beams: 0, frames: 0, lastAt: null },
      joints: { names: [], position: [], frames: 0, lastAt: null },
      jointCmdSent: 0,
      // 镜像防环 v2（rebot-ros-ui.js:190-220 精确语义）：一次性 hold——到位即删
      _mirrorHoldUntil: new Map(),   // joint → 撤销时刻（performance.now() ms）
      _mirrorTargets: new Map(),     // joint → 最近命令目标弧度
      trackingError: null,           // {maxDeg, rmsDeg, worst}
      tf: { frames: [], lastAt: null },
      markers: { count: 0, lastAt: null },
      path: { poses: 0, lastAt: null },
      odom: { points: 0, frames: 0, lastAt: null },
      range: { value: null, fov: null, frames: 0, lastAt: null },
      wrench: { force: null, torque: null, frames: 0, lastAt: null },
      posecov: { a: null, b: null, angle: null, frames: 0, lastAt: null },
      imagec: { frames: 0, lastAt: null },
      tool: null, lastMeasure: null, lastPublishPoint: null, lastGoalPose: null,
      lastInitialPose: null,
    };
    this._handles = {};          // displayKey → 订阅句柄
    this._msgs = {};             // displayKey → 计数
    this._dirty = true;
    this._started = false;
    this._tool = null;           // null | 'measure' | 'point' | 'goal' | 'poseest'
    this._measurePts = [];
    // 里程计轨迹（0.1m 位移阈值 + 100 点环形上限，RVIZ-RQT Scene3D:1807-1817）
    this._odomTrail = createOdomTrail(100, 0.1);
    // 关节预设缓动（moveToAtoms 语义：easeInOutCubic；viz_core 纯核）
    this._motion = createJointMotion();
    this._motionStart = 0;
    this._motionLabel = '';
    this._lastMotionPublish = 0;
    this._envShow = localStorage.getItem('rwl-env-show') !== '0';
    this._init3d();
    this._init2d();
    this._initToolbar();
    this._renderDisplaysPanel();
    setInterval(() => this._refreshStatus(), 1000);
    this._client = foxgloveRosClient();
    this._client.onStateChange((s) => {
      if (s === 'connected' && !this._started) this.start();
    });
    if (this._client.isConnected && !this._started) this.start();
  }

  // ── 3D 场景 ─────────────────────────────────────────────────────────────

  _init3d() {
    const wrap = $('viz-3d');
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.domElement.style.width = '100%';
    renderer.domElement.style.height = '100%';
    renderer.domElement.style.display = 'block';
    wrap.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(
      getComputedStyle(document.body).getPropertyValue('--viz-bg').trim() || '#eceef0');
    window.addEventListener('rwl-theme', () => {
      scene.background.set(
        getComputedStyle(document.body).getPropertyValue('--viz-bg').trim() || '#eceef0');
      this._dirty = true;
    });
    this.grid = new THREE.GridHelper(4, 8, 0x9aa5a0, 0xc5ccc7);
    this.grid.receiveShadow = true;
    scene.add(this.grid);
    scene.add(new THREE.AxesHelper(0.6));
    const rootLabel = makeTextSprite('root', '#8b93a7');
    rootLabel.position.set(0, 0.1, 0);
    scene.add(rootLabel);

    const camera = new THREE.PerspectiveCamera(55, 1, 0.05, 100);
    camera.position.set(1.6, 1.4, 2.2);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0.4, 0);
    controls.enableDamping = true;
    controls.addEventListener('change', () => { this._dirty = true; });

    // 光照升级（RViz 风格柔和三点光 + 阴影）
    scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 0.9));
    const dir = new THREE.DirectionalLight(0xffffff, 1.2);
    dir.position.set(2, 4, 1.5);
    dir.castShadow = true;
    dir.shadow.mapSize.set(1024, 1024);
    dir.shadow.camera.left = -3; dir.shadow.camera.right = 3;
    dir.shadow.camera.top = 3; dir.shadow.camera.bottom = -3;
    scene.add(dir);
    const fill = new THREE.DirectionalLight(0xddeeff, 0.4);
    fill.position.set(-2, 2, -1);
    scene.add(fill);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.groups = {
      tf: new THREE.Group(), robot: new THREE.Group(), cloud: new THREE.Group(),
      scan: new THREE.Group(), markers: new THREE.Group(), path: new THREE.Group(),
      poses: new THREE.Group(), map: new THREE.Group(), tools: new THREE.Group(),
      odom: new THREE.Group(), range: new THREE.Group(), wrench: new THREE.Group(),
      posecov: new THREE.Group(),
    };
    // 统一 ROS Z-up 世界：TF 轴（_tfFrame）与 URDF（_robotFrame）各自已有 Z-up 容器，
    // 数据组（cloud/scan/markers/path/poses/map）共享 _rosFrame——消息里的 ROS 坐标
    // 直接写入组内，header.frame_id → root 的 TF 变换落在组上（RViz 语义）。
    // tools 组除外（测距/发布点是 three 场景坐标交互产物）
    this._tfFrame = new THREE.Group();
    this._tfFrame.rotation.x = -Math.PI / 2;
    this.groups.tf.add(this._tfFrame);
    this._rosFrame = new THREE.Group();
    this._rosFrame.rotation.x = -Math.PI / 2;
    scene.add(this._rosFrame);
    for (const key of ['cloud', 'scan', 'markers', 'path', 'poses', 'map',
      'odom', 'range', 'wrench', 'posecov']) {
      this._rosFrame.add(this.groups[key]);
    }
    this._tfRootFrame = null;   // TF 树根帧（每次 TF 更新重解析）
    this._urdfLoader = new URDFLoaderDefault();
    // mesh 解析链（对齐 dashboard urdf_island / Rebot-Arm server 的 /api/meshes 模式）：
    // packages 回调 → 网关 /api/ivg/robot-mesh/<pkg>/<rel>；file:// 绝对路径
    // （MoveItConfigsBuilder 展开 $(find) 产物）先重写为 package:// 再进 loader
    this._urdfLoader.packages = (pkg) => `/api/ivg/robot-mesh/${pkg}`;
    // LoadingManager：DAE/STL 经 packages 回调异步加载——parse() 返回时骨架就绪
    // 但网格数据未到；onLoad 后再设 castShadow/receiveShadow 才能真正生效
    this._meshManager = new THREE.LoadingManager();
    this._meshManager.onLoad = () => {
      if (!this._urdfRobot) return;
      this._urdfRobot.traverse((o) => {
        if (o.isMesh) {
          o.castShadow = true;
          o.receiveShadow = true;
          if (o.material && o.material.isMeshStandardMaterial) {
            o.material.roughness = Math.min(o.material.roughness ?? 0.6, 0.45);
            o.material.metalness = Math.max(o.material.metalness ?? 0, 0.15);
          }
        }
      });
      // 全部 mesh 就绪后再对准相机（骨架包围盒极小 → 加载后才能正确对准）
      this._fitCameraToRobot();
      this._dirty = true;
    };
    this._urdfLoader.manager = this._meshManager;
    this._urdfRobot = null;
    this._urdfText = null;
    Object.values(this.groups).forEach((g) => scene.add(g));
    this._tfObjects = new Map();

    const posArr = new Float32Array(MAX_POINTS * 3);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(posArr, 3));
    geo.setDrawRange(0, 0);
    this._cloudPos = posArr;
    this.cloudPoints = new THREE.Points(geo,
      new THREE.PointsMaterial({ color: 0x4f8cff, size: this.cfg.cloud.size }));
    this.groups.cloud.add(this.cloudPoints);

    // 3D 激光：预分配（扇形最大 720 波束）
    this._scanPos = new Float32Array(720 * 3);
    const sgeo = new THREE.BufferGeometry();
    sgeo.setAttribute('position', new THREE.BufferAttribute(this._scanPos, 3));
    sgeo.setDrawRange(0, 0);
    this.scanPoints = new THREE.Points(sgeo,
      new THREE.PointsMaterial({ color: 0x2ecc71, size: 0.03 }));
    this.groups.scan.add(this.scanPoints);

    this.renderer = renderer;
    this.scene = scene;
    this.camera = camera;
    this.controls = controls;

    const resize = () => {
      const w = wrap.clientWidth, h = wrap.clientHeight;
      if (!w || !h) return;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      this._dirty = true;
    };
    new ResizeObserver(resize).observe(wrap);
    resize();

    // 指针工具（测距/发布点/Goal Pose）：与 OrbitControls 共存——工具激活时左键截获
    this._ray = new THREE.Raycaster();
    this._plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);  // y=0 地面
    const el = renderer.domElement;
    const planePt = (ev) => {
      const r = el.getBoundingClientRect();
      const ndc = new THREE.Vector2(
        ((ev.clientX - r.left) / r.width) * 2 - 1,
        -((ev.clientY - r.top) / r.height) * 2 + 1);
      this._ray.setFromCamera(ndc, camera);
      const pt = new THREE.Vector3();
      return this._ray.ray.intersectPlane(this._plane, pt) ? pt : null;
    };
    el.addEventListener('pointerdown', (ev) => {
      if (!this._tool || ev.button !== 0) return;
      const pt = planePt(ev);
      if (!pt) return;
      ev.stopImmediatePropagation();
      if (this._tool === 'goal' || this._tool === 'poseest') {
        // 2D Goal Pose / 2D 初始位姿：按下→拖拽定航向→松开发布（RViz 工具语义；
        // poseest 发 PoseWithCovarianceStamped 到 /initialpose，协方差取 RViz 标准初值）
        const isInit = this._tool === 'poseest';
        this._goalStart = pt.clone();
        this._goalArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), pt, 0.5,
          isInit ? 0x4dabf7 : 0x9b59b6, 0.1, 0.05);
        this.groups.tools.add(this._goalArrow);
        this._dirty = true;
        const onMove = (mv) => {
          const p2 = planePt(mv);
          if (!p2) return;
          const dir = p2.clone().sub(this._goalStart);
          if (dir.length() > 0.02) {
            this._goalArrow.setDirection(dir.clone().normalize());
          }
        };
        const onUp = (up) => {
          el.removeEventListener('pointermove', onMove);
          el.removeEventListener('pointerup', onUp);
          const p2 = planePt(up);
          const dir = p2 ? p2.clone().sub(this._goalStart) : new THREE.Vector3();
          // 场景地面（y=0）上：ROS x = scene x，ROS y = -scene z（Z-up 容器约定）
          const yaw = dir.length() > 0.05
            ? Math.atan2(-dir.z, dir.x)
            : 0;
          const pose = {
            position: { x: this._goalStart.x, y: -this._goalStart.z, z: 0 },
            orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
          };
          const frameId = this._fixedFrame();
          let ok;
          if (isInit) {
            // RViz 2D Pose Estimate 标准协方差（x/y 0.25，yaw 0.0685…）
            const cov = new Array(36).fill(0);
            cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.06853891909;
            ok = this._client.publish('/initialpose',
              'geometry_msgs/msg/PoseWithCovarianceStamped',
              { header: { frame_id: frameId }, pose: { pose, covariance: cov } });
            this.stats.lastInitialPose = {
              frame: frameId,
              x: +this._goalStart.x.toFixed(2), y: +(-this._goalStart.z).toFixed(2),
              yaw: +yaw.toFixed(3), ok,
            };
          } else {
            ok = this._client.publish('/goal_pose', 'geometry_msgs/msg/PoseStamped',
              { header: { frame_id: frameId }, pose });
            this.stats.lastGoalPose = {
              frame: frameId,
              x: +this._goalStart.x.toFixed(2), y: +(-this._goalStart.z).toFixed(2),
              yaw: +yaw.toFixed(3), ok,
            };
          }
          const topic = isInit ? '/initialpose' : '/goal_pose';
          $('hud-tool').textContent =
            `已发布 ${topic} (${this._goalStart.x.toFixed(2)}, ${(-this._goalStart.z).toFixed(2)}) yaw=${yaw.toFixed(2)} → ${ok ? '✓' : '✗'}`;
          setTimeout(() => {
            this.groups.tools.remove(this._goalArrow); this._dirty = true;
          }, 2000);
        };
        el.addEventListener('pointermove', onMove);
        el.addEventListener('pointerup', onUp);
        return;
      }
      this._onToolClick(pt);
    }, true);

    const loop = () => {
      requestAnimationFrame(loop);
      // FPS EMA（α=0.1，RAF 实际帧间隔）
      const now = performance.now();
      if (this._lastTick) {
        const inst = 1000 / Math.max(now - this._lastTick, 0.1);
        this._fpsEma = this._fpsEma == null ? inst : this._fpsEma * 0.9 + inst * 0.1;
      }
      this._lastTick = now;
      controls.update();
      this._stepMotion(now);
      if (!this._dirty) return;
      this._dirty = false;
      renderer.render(scene, camera);
    };
    loop();
    // HUD：FPS + 场景对象数（1s 节流，scene.traverse 计数）
    setInterval(() => {
      let objs = 0;
      scene.traverse(() => { objs++; });
      this.stats.objectCount = objs;
      this.stats.fps = this._fpsEma == null ? null : Math.round(this._fpsEma);
      const f = $('hud-fps'), o = $('hud-objs');
      if (f) f.textContent = this.stats.fps ?? '—';
      if (o) o.textContent = objs;
    }, 1000);
  }

  _init2d() {
    this._imgCtx = $('viz-image').getContext('2d');
    this._scanCtx = $('viz-scan').getContext('2d');
    this._imgLastDraw = 0;
    this._scanLastDraw = 0;
  }

  // ── 工具栏（RViz 风格） ─────────────────────────────────────────────────

  _initToolbar() {
    document.querySelectorAll('[data-tool]').forEach((b) => {
      b.addEventListener('click', () => {
        const t = b.dataset.tool;
        this._tool = this._tool === t ? null : t;
        this.stats.tool = this._tool;
        document.querySelectorAll('[data-tool]').forEach((x) =>
          x.classList.toggle('active', x.dataset.tool === this._tool));
        $('hud-tool').textContent = this._tool === 'measure'
          ? '测距：在 3D 中点击两个点'
          : this._tool === 'point' ? '发布点：在 3D 中点击一个点'
          : this._tool === 'goal' ? '2D Goal Pose：按下拖拽定航向，松开发布 /goal_pose'
          : this._tool === 'poseest' ? '2D 初始位姿：按下拖拽定航向，松开发布 /initialpose'
          : '';
        if (!this._tool) this._clearTools();
      });
    });
    const VIEWS = {
      iso: { p: [3.0, 2.4, 3.2], t: [0.4, 0.4, 0] },
      top: { p: [0.5, 6.5, 0.01], t: [0.5, 0, 0] },
      side: { p: [0.5, 0.8, 5.5], t: [0.5, 0.5, 0] },
    };
    const applyView = (v) => {
      this.camera.position.set(...v.p);
      this.controls.target.set(...v.t);
      this._dirty = true;
    };
    document.querySelectorAll('[data-view]').forEach((b) => {
      b.addEventListener('click', () => applyView(VIEWS[b.dataset.view]));
    });
    $('btn-view-save').addEventListener('click', () => {
      localStorage.setItem('rwl-viz-view', JSON.stringify({
        p: this.camera.position.toArray(), t: this.controls.target.toArray() }));
      $('hud-tool').textContent = '视角已保存';
    });
    $('btn-view-restore').addEventListener('click', () => {
      const v = JSON.parse(localStorage.getItem('rwl-viz-view') || 'null');
      if (v) applyView({ p: v.p, t: v.t });
    });
  }

  _clearTools() {
    VizPanel._clearGroup(this.groups.tools);
    this._measurePts = [];
  }

  /** 工具发布坐标系：TF 树根（RViz fixed-frame 语义；Nav2 场景=map），缺省 rwl_viz */
  _fixedFrame() {
    return this._tfRoot() || 'rwl_viz';
  }

  _onToolClick(pt) {
    if (this._tool === 'point') {
      // Publish Point（RViz 工具语义）：发布 PointStamped + 场景闪现标记
      //（点击发生在 y=0 地面：ROS x = scene x，ROS y = -scene z）
      const ok = this._client.publish('/rwl_viz/clicked_point',
        'geometry_msgs/msg/PointStamped', {
          header: { frame_id: this._fixedFrame() },
          point: { x: pt.x, y: -pt.z, z: 0 },
        });
      this.stats.lastPublishPoint = { x: +pt.x.toFixed(3), y: +pt.y.toFixed(3), ok };
      const m = new THREE.Mesh(new THREE.SphereGeometry(0.04, 10, 8),
        new THREE.MeshBasicMaterial({ color: 0xf39c12 }));
      m.position.copy(pt);
      this.groups.tools.add(m);
      setTimeout(() => { this.groups.tools.remove(m); this._dirty = true; }, 1500);
      this._dirty = true;
      $('hud-tool').textContent =
        `已发布 clicked_point (${pt.x.toFixed(2)}, ${pt.y.toFixed(2)}) → ${ok ? '✓' : '✗'}`;
      return;
    }
    // measure
    this._measurePts.push(pt.clone());
    const dot = new THREE.Mesh(new THREE.SphereGeometry(0.025, 8, 6),
      new THREE.MeshBasicMaterial({ color: 0xe74c3c }));
    dot.position.copy(pt);
    this.groups.tools.add(dot);
    if (this._measurePts.length === 2) {
      const [a, b] = this._measurePts;
      this.groups.tools.add(new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([a, b]),
        new THREE.LineBasicMaterial({ color: 0xe74c3c })));
      const d = a.distanceTo(b);
      const label = makeTextSprite(d.toFixed(3) + ' m', '#e74c3c');
      label.position.copy(a.clone().add(b).multiplyScalar(0.5));
      label.position.y += 0.08;
      this.groups.tools.add(label);
      this.stats.lastMeasure = +d.toFixed(3);
      $('hud-tool').textContent = `距离 = ${d.toFixed(3)} m`;
      this._measurePts = [];
    }
    this._dirty = true;
  }

  // ── Displays 面板（RViz 风格） ──────────────────────────────────────────

  _renderDisplaysPanel() {
    const rows = [
      ['grid', '网格 Grid', null, []],
      ['robot', 'RobotModel URDF', null, [['ghost', '幽灵']]],
      ['map', '地图 OccupancyGrid', this.cfg.map.topic, []],
      ['tf', 'TF 坐标轴', null, [['labels', '名签']]],
      ['cloud', '点云 PointCloud2', this.cfg.cloud.topic, [['size', '点大小']]],
      ['scan3d', '激光 3D LaserScan', this.cfg.scan3d.topic, []],
      ['markers', '标记 MarkerArray', this.cfg.markers.topic, []],
      ['path', '路径 Path', this.cfg.path.topic, []],
      ['poses', '位姿数组 PoseArray', this.cfg.poses.topic, []],
      ['odom', '里程 Odometry', this.cfg.odom.topic, [['trail', '轨迹']]],
      ['range', '测距 Range', this.cfg.range.topic, []],
      ['wrench', '力/力矩 Wrench', this.cfg.wrench.topic, [['scale', '缩放']]],
      ['posecov', '协方差位姿 PoseCov', this.cfg.posecov.topic, []],
      ['image', '相机 Image', this.cfg.image.topic, []],
      ['imagec', '压缩图 CompressedImage', this.cfg.imagec.topic, []],
    ];
    const tb = $('viz-displays').querySelector('tbody');
    tb.innerHTML = rows.map(([key, name, topic, props]) => `
      <tr data-display="${key}">
        <td><input type="checkbox" data-cfg="on" ${this.cfg[key].on ? 'checked' : ''}></td>
        <td>${name}</td>
        <td>${topic === null
          ? '<span style="color:var(--muted)">' + (key === 'tf' ? '/tf（固定）' : key === 'robot' ? '/robot_description（固定）' : '内置') + '</span>'
          : `<input class="dtopic" list="rwl-topics" value="${this.cfg[key].topic}">`}</td>
        <td class="props">${props.map(([p, label]) => p === 'size'
          ? `<label class="mini">${label} <input type="range" min="0.01" max="0.12" step="0.005" data-cfg="size" value="${this.cfg.cloud.size}"></label>`
          : p === 'scale'
          ? `<label class="mini">${label} <input type="range" min="0.05" max="1" step="0.05" data-cfg="scale" value="${this.cfg.wrench.scale}"></label>`
          : `<label class="mini"><input type="checkbox" data-cfg="${p}" ${this.cfg[key][p] ? 'checked' : ''}>${label}</label>`).join('')}</td>
        <td class="dst" id="viz-st-${key}">—</td>
      </tr>`).join('');
    tb.querySelectorAll('tr[data-display]').forEach((tr) => {
      const key = tr.dataset.display;
      tr.querySelectorAll('[data-cfg]').forEach((el) => {
        el.addEventListener(el.type === 'range' ? 'input' : 'change', () => {
          const p = el.dataset.cfg;
          this.cfg[key][p] = el.type === 'checkbox' ? el.checked : parseFloat(el.value);
          localStorage.setItem(CFG_KEY, JSON.stringify(this.cfg));
          this._applyDisplayCfg(key);
        });
      });
      const topicEl = tr.querySelector('.dtopic');
      if (topicEl) {
        topicEl.addEventListener('change', () => {
          this.cfg[key].topic = topicEl.value.trim();
          localStorage.setItem(CFG_KEY, JSON.stringify(this.cfg));
          this._applyDisplayCfg(key);
        });
      }
    });
    // 话题候选（datalist）周期刷新自图谱
    const refreshTopics = () => {
      const chans = this._client?.player?.channels;
      if (!chans) return;
      $('rwl-topics').innerHTML = [...chans.keys()]
        .map((t) => `<option value="${t}">`).join('');
    };
    setInterval(refreshTopics, 2000);
    refreshTopics();
  }

  _applyDisplayCfg(key) {
    const c = this.cfg[key];
    if (key === 'grid') { this.grid.visible = c.on; this._dirty = true; return; }
    const g = this.groups[key === 'scan3d' ? 'scan' : key];
    if (g) g.visible = c.on;
    if (key === 'cloud' && c.size) this.cloudPoints.material.size = c.size;
    if (key === 'robot' && this._ghostRobot) {
      this._ghostRobot.visible = !!c.ghost;
      this._dirty = true;
    }
    if (key === 'tf' && this._tfObjects.size) {
      for (const obj of this._tfObjects.values()) obj.children[1].visible = c.labels;
    }
    this._subscribeDisplay(key, true);
    this._dirty = true;
  }

  _subscribeDisplay(key, force = false) {
    const c = this.cfg[key];
    if (!this._client?.isConnected) return;
    if (this._handles[key]) {
      if (!force) return;
      try { this._handles[key].cancel(); } catch (_) {}
      this._handles[key] = null;
    }
    if (!c.on || !c.topic) return;
    const handler = {
      robot: (m) => this._onRobotDescription(m.data),
      map: (m) => this._onOccupancyGrid(m.data),
      tf: () => this._onTf(),
      cloud: (m) => this._onCloud(m.data),
      scan3d: (m) => { this._onScan(m.data); this._onScan3d(m.data); },
      markers: (m) => this._onMarkers(m.data.markers || [m.data]),
      path: (m) => this._onPath(m.data),
      poses: (m) => this._onPoseArray(m.data),
      odom: (m) => this._onOdom(m.data),
      range: (m) => this._onRange(m.data),
      wrench: (m) => this._onWrench(m.data),
      posecov: (m) => this._onPoseCov(m.data),
      image: (m) => this._onImage(m.data),
      imagec: (m) => this._onCompressedImage(m.data),
    }[key];
    if (!handler) return;
    if (key === 'odom') this._odomTrail.clear();   // 换话题/重订时轨迹清零
    this._msgs[key] = 0;
    this._handles[key] = this._client.subscribe(c.topic, '', (m) => {
      this._msgs[key]++;
      this._stamp(m.data?.header);
      handler(m);
    }, { dispatchMode: 'latest' });
  }

  /** 用各流 header.stamp 维护 ROS Time（RViz 时间面板语义） */
  _stamp(h) {
    if (!h?.stamp) return;
    const ns = Number(h.stamp.sec) * 1e9 + Number(h.stamp.nanosec);
    if (ns > (this.stats.rosTimeNs ?? 0)) this.stats.rosTimeNs = ns;
  }

  // ── 订阅入口 ────────────────────────────────────────────────────────────

  start() {
    if (this._started) return;
    this._started = true;
    this.stats.connectedAt = new Date().toISOString();
    // JointState 固定流（关节条形图）+ PoseStamped（单箭头并入 poses 组，限量防堆积）
    for (const jsTopic of ['/rwl_viz/joint_states', '/joint_states']) {
      this._client.subscribe(jsTopic, '', (m) => {
        this._stamp(m.data?.header);
        this._onJoints(m.data);
        this._onJointStatesForRobot(m.data);  // RobotModel 关节动画
      }, { dispatchMode: 'latest' });
    }
    this._client.subscribe('/rwl_viz/pose', '', (m) => {
      this._stamp(m.data?.header);
      if (this.groups.poses.children.length > 16) VizPanel._clearGroup(this.groups.poses);
      const arrow = this._arrow(m.data.pose, 0xe74c3c);
      const frameTf = this._frameToRoot(m.data?.header?.frame_id);
      if (frameTf) arrow.applyMatrix4(frameTf);
      this.groups.poses.add(arrow);
      this._dirty = true;
    }, { dispatchMode: 'latest' });
    for (const key of Object.keys(this.cfg)) {
      if (key !== 'grid' && key !== 'tf') this._subscribeDisplay(key, true);
    }
    this._subscribeDisplay('tf', true);
    // TF 树单源：foxgloveTfTree() 自订 /tf（latest）+ /tf_static（immediate，
    // TRANSIENT_LOCAL 晚连接取最后一帧）并做全精度链路累积；此处再订 /tf_static
    // 仅作静态变换到达后的重绘触发（旧代码传 m 而非 m.data，静态 TF 从未进树）
    this._tree = foxgloveTfTree();
    this._client.subscribe('/tf_static', 'tf2_msgs/msg/TFMessage', () => {
      this._onTf();
    }, { dispatchMode: 'immediate' });
    $('viz-status').textContent = '可视化流已订阅（Displays 面板可调）';
  }

  _refreshStatus() {
    if (!this._started) return;
    const st = this.stats.rosTimeNs;
    $('viz-rostime').textContent = st ? new Date(st / 1e6).toISOString().slice(11, 23) : '—';
    for (const key of Object.keys(this._msgs)) {
      const el = $('viz-st-' + key);
      if (!el) continue;
      const n = this._msgs[key];
      el.textContent = this.cfg[key].on ? (n > 0 ? `✓ ${n}` : '…') : '关';
      el.className = 'dst ' + (!this.cfg[key].on ? '' : n > 0 ? 'ok' : 'warn');
    }
    const i = this.stats.image;
    $('viz-img-cap').textContent = i.frames ? `${i.w}×${i.h} ${i.enc} · ${i.frames} 帧` : '—';
    const s = this.stats.scan;
    $('viz-scan-cap').textContent = s.frames ? `${s.beams} 波束 · ${s.frames} 帧` : '—';
    $('viz-joints-cap').textContent = this.stats.joints.frames
      ? `${this.stats.joints.names.length} 关节 · ${this.stats.joints.frames} 帧` : '—';
  }

  // ── 各显示项渲染 ────────────────────────────────────────────────────────

  /** RobotModel（RViz 招牌显示）：/robot_description URDF + joint_states 驱动 */
  _onRobotDescription(data) {
    let text = data?.data;
    if (!text || text === this._urdfText) return;
    // file://<ws>/install/<pkg>/share/<pkg>/… → package://<pkg>/…（mesh 走 robot-mesh 路由）
    text = text.replace(
      /file:\/\/\S*?\/install\/([\w-]+)\/share\/\1\//g, 'package://$1/');
    this._urdfText = text;
    VizPanel._clearGroup(this.groups.robot);
    const mask = document.getElementById('robot-loading-mask');
    const maskText = document.getElementById('robot-loading-text');
    if (mask) mask.classList.remove('hidden');
    try {
      this._urdfRobot = this._urdfLoader.parse(text);
      // ROS Z-up 容器（rebot-sim.js:251 同构）：ROS z→three y，网格/灯光保持默认
      this._robotFrame = new THREE.Group();
      this._robotFrame.rotation.x = -Math.PI / 2;
      this._robotFrame.add(this._urdfRobot);
      this.groups.robot.add(this._robotFrame);
      this.stats.robotModel = {
        links: Object.keys(this._urdfRobot.links || {}).length,
        joints: Object.keys(this._urdfRobot.joints || {}).length,
      };
      this._createGhost();
      this._initTeach();
      // 阴影/材质/相机对准由 LoadingManager.onLoad 回调处理（mesh 异步就绪后）
      // 此处不再调 _fitCameraToRobot（骨架包围盒极小 → 对不准）
      if (maskText) maskText.textContent = '模型就绪';
      if (mask) setTimeout(() => mask.classList.add('hidden'), 400);
    } catch (e) {
      this.stats.robotModel = { error: String(e.message || e) };
      if (maskText) maskText.textContent = '模型加载失败: ' + (e.message || e);
      // 遮罩兼作错误面板（参考 failLoad 模式：不自动隐藏）
    }
    this._dirty = true;
  }

  /** 相机自适应机器人包围盒（首次加载后对准） */
  _fitCameraToRobot() {
    if (!this._urdfRobot) return;
    // 确保 matrixWorld 最新（mesh 异步加载后需手动刷新）
    const frame = this._robotFrame || this._urdfRobot;
    frame.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(frame);
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    // 距离：取包围球直径 × 1.5，最少 1.2m（避免细长臂近距离只有一截）
    const radius = Math.max(size.x, size.y, size.z) / 2;
    const dist = Math.max(1.2, radius * 3);
    // 相机放斜 45° 俯角 30°（RViz 默认机位风格）
    this.camera.position.set(
      center.x + dist * 0.55,
      center.y + dist * 0.45,
      center.z + dist * 0.7);
    this.controls.target.copy(center);
    this.controls.update();
    this._dirty = true;
  }

  /** 幽灵预览机器人（rebot-sim.js:590-597 同构：clone + getObjectByName 回退） */
  _createGhost() {
    if (!this._urdfRobot || this._ghostRobot) return;
    this._ghostRobot = this._urdfRobot.clone(true);
    this._ghostRobot.traverse((o) => {
      if (o.isMesh) {
        o.material = new THREE.MeshStandardMaterial({
          color: 0x33d6b0, roughness: 0.28, metalness: 0.05,
          transparent: true, opacity: 0.22, side: THREE.DoubleSide,
          depthWrite: false,
        });
        o.castShadow = o.receiveShadow = false;
      }
    });
    this._ghostRobot.visible = false;
    this._robotFrame.add(this._ghostRobot);
  }

  _syncGhostToRobot() {
    if (!this._ghostRobot || !this._ghostRobot.visible) return;
    for (const [name, j] of Object.entries(this._urdfRobot.joints || {})) {
      const gj = this._ghostRobot.joints?.[name]
        || this._ghostRobot.getObjectByName(name);
      try { gj?.setJointValue?.(j.angle ?? 0); } catch (_) { /* keep */ }
    }
  }

  /** 示教控制器挂载（arm 关节动态发现：revolute 类型枚举，排除手指） */
  _initTeach() {
    if (this._teach || !this._urdfRobot) return;
    const armJoints = Object.entries(this._urdfRobot.joints || {})
      .filter(([, j]) => (j.jointType === 'revolute' || j.jointType === 1
        || j.type === 1))
      .map(([name]) => name)
      .filter((n) => !/finger/.test(n));
    if (!armJoints.length) return;
    this._armJointNames = armJoints;
    this._teach = new TeachController({
      robot: this._urdfRobot,
      robotFrame: this._robotFrame,
      camera: this.camera,
      scene: this.scene,
      canvas: this.renderer.domElement,
      armJointNames: armJoints,
    });
    // 总线订阅：drag/drag-settle/trajectory 写回 URDF 关节 + 幽灵同步
    this._teachUnsub = onCommand((cmd) => {
      if (cmd.type !== 'joint' || !cmd.joints) return;
      for (const [name, v] of Object.entries(cmd.joints)) {
        const j = this._urdfRobot?.joints?.[name]
          || this._urdfRobot?.getObjectByName?.(name);
        try { j?.setJointValue?.(v); } catch (_) { /* keep */ }
      }
      this._dirty = true;
    });
  }

  _onJointStatesForRobot(data) {
    const robot = this._urdfRobot;
    if (!robot) return;
    const values = {};
    (data.name || []).forEach((n, i) => {
      if (robot.joints?.[n]) values[n] = Number(data.position?.[i] ?? 0);
    });
    try { robot.setJointValues(values); this._dirty = true; } catch (_) {}
  }

  // ── TF（全精度链路累积，view3d/tf_buffer 单源）──────────────────────────

  /** TF 树根帧 id（单树场景取 isRoot 帧；每次 TF 更新重解析，会话切换自动跟随） */
  _tfRoot() {
    if (!this._tree) return null;
    let root = this._tfRootFrame;
    if (!root || !this._tree.hasFrame(root)) {
      root = null;
      for (const f of this._tree.frames().values()) {
        if (f.isRoot()) { root = f.id; break; }
      }
      this._tfRootFrame = root;
    }
    return root;
  }

  /** frame → root 的最新变换（THREE.Matrix4）；frame 缺省/不可解析 → null */
  _frameToRoot(frameId) {
    if (!frameId) return null;
    const root = this._tfRoot();
    if (!root || frameId === root) return null;
    if (!this._tree.hasFrame(frameId)) return null;
    const tfm = lookupLatest(this._tree, frameId, root);
    if (!tfm) return null;
    const m = new THREE.Matrix4();
    const q = tfm.rotation;
    m.makeRotationFromQuaternion(new THREE.Quaternion(
      Number(q.x), Number(q.y), Number(q.z), Number(q.w)));
    m.setPosition(tfm.translation.x, tfm.translation.y, tfm.translation.z);
    return m;
  }

  /** 把消息坐标系的组摆到 root 世界（RViz 帧语义）；不可解析时归位 + 一次性 warn */
  _applyFrameToGroup(group, frameId, context) {
    const t = this._frameToRoot(frameId);
    if (t) {
      t.decompose(group.position, group.quaternion, new THREE.Vector3());
    } else {
      if (frameId && this._tree && !this._tree.hasFrame(frameId)) {
        const key = context + ':' + frameId;
        if (!this._frameWarned?.has(key)) {
          (this._frameWarned ??= new Set()).add(key);
          console.warn(`[viz] ${context}: TF 无 ${frameId} 帧，按 root 原位显示`);
        }
      }
      group.position.set(0, 0, 0);
      group.quaternion.identity();
    }
  }

  _onTf() {
    // TF 树由 foxgloveTfTree() 单源维护（/tf + /tf_static 全精度累积）；
    // 此处只做显示：遍历树帧，lookupLatest(frame, root) 取世界位姿（平移+旋转）。
    // 修复记录：旧实现 (1) 只叠平移、四元数取本条局部旋转（中间帧姿态错）
    // (2) /tf_static 从未进树（回调传参 bug）
    if (!this._tree) this._tree = foxgloveTfTree();
    const rootId = this._tfRoot();
    if (!rootId) return;

    for (const f of this._tree.frames().values()) {
      const name = f.id;
      let obj = this._tfObjects.get(name);
      if (!obj) {
        obj = new THREE.Group();
        obj.add(new THREE.AxesHelper(0.22));
        const label = makeTextSprite(name, '#2ecc71');
        label.position.set(0, 0, 0.14);  // Z-up 容器内"上"= Z
        label.visible = this.cfg.tf.labels;
        obj.add(label);
        this._tfFrame.add(obj);  // Z-up 容器
        this._tfObjects.set(name, obj);
      }
      if (name === rootId) {
        obj.position.set(0, 0, 0);
        obj.quaternion.identity();
        continue;
      }
      const tfm = lookupLatest(this._tree, name, rootId);
      if (!tfm) continue;
      obj.position.set(tfm.translation.x, tfm.translation.y, tfm.translation.z);
      const q = tfm.rotation;
      obj.quaternion.set(Number(q.x), Number(q.y), Number(q.z), Number(q.w));
    }
    this.stats.tf = { frames: [...this._tfObjects.keys()], lastAt: Date.now() };
    this._dirty = true;
  }

  _onCloud(data) {
    // 解析单源 pointcloud_parse.js：fields[].offset + DataView 读 float32
    //（旧实现逐字节读 data → 渲染出的是字节值而非坐标）
    const { count, error } = parsePointCloud2(data,
      { positions: this._cloudPos, maxPoints: MAX_POINTS });
    if (error) {
      if (!this._cloudWarned) {
        this._cloudWarned = true;
        console.warn('[viz] PointCloud2 解析失败:', error === 'data' ? 'data 字段不可解析' : '无 xyz 字段');
      }
      this.cloudPoints.geometry.setDrawRange(0, 0);
      return;
    }
    this.cloudPoints.geometry.attributes.position.needsUpdate = true;
    this.cloudPoints.geometry.setDrawRange(0, count);
    this._applyFrameToGroup(this.groups.cloud, data?.header?.frame_id, 'cloud');
    this.stats.cloud = { points: count, lastAt: Date.now() };
    this._dirty = true;
  }

  _onScan3d(data) {
    // RViz LaserScan 3D 语义：ROS 坐标（z-up 容器内）x=cos·r 前向、y=sin·r 左向
    const ranges = data.ranges || [];
    const n = Math.min(ranges.length, 720);
    let i3 = 0;
    for (let i = 0; i < n; i++) {
      const r = ranges[i];
      if (isFinite(r)) {
        const a = (data.angle_min || 0) + i * (data.angle_increment || 0);
        this._scanPos[i3++] = Math.cos(a) * r;
        this._scanPos[i3++] = Math.sin(a) * r;
        this._scanPos[i3++] = 0.02;
      } else {
        this._scanPos[i3] = 0; this._scanPos[i3 + 1] = 0; this._scanPos[i3 + 2] = 0;
        i3 += 3;
      }
    }
    this.scanPoints.geometry.attributes.position.needsUpdate = true;
    this.scanPoints.geometry.setDrawRange(0, n);
    this._applyFrameToGroup(this.groups.scan, data?.header?.frame_id, 'scan3d');
    this._dirty = true;
  }

  _onMarkers(markers) {
    const g = this.groups.markers;
    VizPanel._clearGroup(g);
    let count = 0;
    for (const mk of markers) {
      if (Number(mk.action) === 2) continue;
      const type = Number(mk.type);
      const colorOf = (c) => new THREE.Color(c?.r ?? 0.8, c?.g ?? 0.6, c?.b ?? 0.3);
      if (type === 7) {
        const mat = new THREE.MeshStandardMaterial({
          color: mk.colors?.[0] ? colorOf(mk.colors[0]) : 0x4f8cff });
        const geo = new THREE.SphereGeometry(Math.max(mk.scale?.x || 0.05, 0.01), 12, 8);
        (mk.points || []).forEach((p, i) => {
          const mesh = new THREE.Mesh(geo, mat);
          mesh.position.set(p.x, p.y, p.z);
          if (mk.colors?.[i]) mesh.material = new THREE.MeshStandardMaterial({
            color: colorOf(mk.colors[i]) });
          g.add(mesh);
          count++;
        });
      } else if (type === 1) {
        const mesh = new THREE.Mesh(
          new THREE.BoxGeometry(mk.scale?.x || 0.2, mk.scale?.y || 0.2, mk.scale?.z || 0.2),
          new THREE.MeshStandardMaterial({ color: colorOf(mk.color) }));
        mesh.position.set(mk.pose?.position?.x || 0, mk.pose?.position?.y || 0,
          mk.pose?.position?.z || 0);
        g.add(mesh);
        count++;
      } else if (type === 9 || type === 10) {
        const sp = makeTextSprite(mk.text || mk.mesh_resource || '?', '#f39c12');
        sp.position.set(mk.pose?.position?.x || 0, mk.pose?.position?.y || 0,
          (mk.pose?.position?.z || 0) + 0.05);
        if (mk.scale?.z) sp.scale.set(mk.scale.z * 2.2, mk.scale.z * 0.55, 1);
        g.add(sp);
        count++;
      }
    }
    // 组级帧放置（MarkerArray 无顶层 header，取首条 marker 的 frame_id；
    // 混帧数组的少见场景按首帧处理——RViz 是逐条，此处组级近似）
    this._applyFrameToGroup(g, markers[0]?.header?.frame_id, 'markers');
    this.stats.markers = { count, lastAt: Date.now() };
    this._dirty = true;
  }

  _onPath(data) {
    const g = this.groups.path;
    VizPanel._clearGroup(g);
    const frameTf = this._frameToRoot(data?.header?.frame_id);
    const pts = (data.poses || []).map((p) => {
      const v = new THREE.Vector3(p.pose.position.x, p.pose.position.y, p.pose.position.z);
      if (frameTf) v.applyMatrix4(frameTf);
      return v;
    });
    if (pts.length >= 2) {
      g.add(new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color: 0x2ecc71 })));
    }
    this.stats.path = { poses: pts.length, lastAt: Date.now() };
    this._dirty = true;
  }

  _arrow(pose, color) {
    // geometry_msgs/Pose 字段是 orientation（修复记录：旧代码读 .rotation，
    // 所有调用方传的都是 Pose → 箭头显示一直静默失败）
    const p = pose.position, q = pose.orientation ?? pose.rotation;
    const dir = new THREE.Vector3(0, 1, 0).applyQuaternion(
      new THREE.Quaternion(q.x, q.y, q.z, q.w));
    return new THREE.ArrowHelper(dir, new THREE.Vector3(p.x, p.y, p.z), 0.25, color, 0.08, 0.04);
  }

  _onPoseArray(data) {
    const g = this.groups.poses;
    VizPanel._clearGroup(g);
    const frameTf = this._frameToRoot(data?.header?.frame_id);
    (data.poses || []).forEach((p) => {
      const arrow = this._arrow(p, 0xf39c12);
      if (frameTf) arrow.applyMatrix4(frameTf);
      g.add(arrow);
    });
    this._dirty = true;
  }

  /** Odometry（RViz 语义）：位姿箭头 + 轨迹（0.1m 阈值 + 100 点环形上限） */
  _onOdom(data) {
    const g = this.groups.odom;
    VizPanel._clearGroup(g);
    const pose = data?.pose?.pose;
    if (!pose) return;
    g.add(this._arrow(pose, 0x00b4d8));
    const p = pose.position;
    this._odomTrail.push(Number(p.x), Number(p.y), Number(p.z));
    if (this.cfg.odom.trail && this._odomTrail.points.length >= 2) {
      const pts = this._odomTrail.points.map(([x, y, z]) => new THREE.Vector3(x, y, z));
      g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color: 0x00b4d8, transparent: true, opacity: 0.55 })));
    }
    this._applyFrameToGroup(g, data?.header?.frame_id, 'odom');
    this.stats.odom = { points: this._odomTrail.points.length,
      frames: (this.stats.odom?.frames || 0) + 1, lastAt: Date.now() };
    this._dirty = true;
  }

  /** Range（RViz 语义）：传感器原点扇形（±fov/2）+ 距离弧 + 端点球，x 前向 */
  _onRange(data) {
    const g = this.groups.range;
    VizPanel._clearGroup(g);
    const r = Number(data?.range);
    if (!Number.isFinite(r) || r <= 0) return;
    const fov = Math.min(Math.PI, Number(data?.field_of_view) || 0.5);
    const arc = [];
    const segs = 24;
    for (let i = 0; i <= segs; i++) {
      const a = -fov / 2 + (fov * i) / segs;
      arc.push(new THREE.Vector3(Math.cos(a) * r, Math.sin(a) * r, 0));
    }
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(arc),
      new THREE.LineBasicMaterial({ color: 0x20c9d6, transparent: true, opacity: 0.85 })));
    for (const a of [-fov / 2, fov / 2]) {
      g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(
        [new THREE.Vector3(0, 0, 0), new THREE.Vector3(Math.cos(a) * r, Math.sin(a) * r, 0)]),
        new THREE.LineBasicMaterial({ color: 0x20c9d6, transparent: true, opacity: 0.4 })));
    }
    const tip = new THREE.Mesh(new THREE.SphereGeometry(0.03, 10, 8),
      new THREE.MeshBasicMaterial({ color: 0x20c9d6 }));
    tip.position.set(r, 0, 0);
    g.add(tip);
    this._applyFrameToGroup(g, data?.header?.frame_id, 'range');
    this.stats.range = { value: +r.toFixed(3), fov: +fov.toFixed(2),
      frames: (this.stats.range?.frames || 0) + 1, lastAt: Date.now() };
    this._dirty = true;
  }

  /** Wrench（RViz 语义）：force 红箭头 + torque 蓝箭头（scale 属性缩放，长度钳 0.06~1.2m） */
  _onWrench(data) {
    const g = this.groups.wrench;
    VizPanel._clearGroup(g);
    const w = data?.wrench;
    if (!w) return;
    const scale = this.cfg.wrench.scale || 0.2;
    const mkArrow = (v, color, k) => {
      const mag = Math.hypot(Number(v?.x) || 0, Number(v?.y) || 0, Number(v?.z) || 0);
      if (mag < 1e-9) return null;
      const len = Math.min(1.2, Math.max(0.06, mag * scale * k));
      const dir = new THREE.Vector3(Number(v.x), Number(v.y), Number(v.z)).normalize();
      return new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0, 0), len, color,
        Math.min(0.1, len * 0.3), Math.min(0.05, len * 0.15));
    };
    const f = mkArrow(w.force, 0xe74c3c, 1);
    const tq = mkArrow(w.torque, 0x3498db, 2.5);
    if (f) g.add(f);
    if (tq) g.add(tq);
    this._applyFrameToGroup(g, data?.header?.frame_id, 'wrench');
    this.stats.wrench = {
      force: [w.force?.x, w.force?.y, w.force?.z].map((v) => +Number(v || 0).toFixed(3)),
      torque: [w.torque?.x, w.torque?.y, w.torque?.z].map((v) => +Number(v || 0).toFixed(3)),
      frames: (this.stats.wrench?.frames || 0) + 1, lastAt: Date.now(),
    };
    this._dirty = true;
  }

  /** PoseWithCovarianceStamped（AMCL 语义）：位姿箭头 + XY 95% 协方差椭圆 */
  _onPoseCov(data) {
    const g = this.groups.posecov;
    VizPanel._clearGroup(g);
    const pose = data?.pose?.pose;
    if (!pose) return;
    g.add(this._arrow(pose, 0x9b59b6));
    const cov = data.pose.covariance;
    // 定长 float64[36] CDR 解码为 Float64Array（非 Array）——按 length 判定
    if (cov && cov.length === 36) {
      const { a, b, angle } = covarianceEllipse(cov);
      if (a > 1e-6) {
        const pts2 = new THREE.EllipseCurve(0, 0, a, b, 0, 2 * Math.PI)
          .getPoints(48).map((p) => new THREE.Vector3(p.x, p.y, 0));
        const ellipse = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(pts2),
          new THREE.LineBasicMaterial({ color: 0x9b59b6, transparent: true, opacity: 0.75 }));
        ellipse.rotation.z = angle;
        ellipse.position.set(pose.position.x, pose.position.y, (pose.position.z || 0) + 0.01);
        g.add(ellipse);
        this.stats.posecov = { a: +a.toFixed(4), b: +b.toFixed(4), angle: +angle.toFixed(3),
          frames: (this.stats.posecov?.frames || 0) + 1, lastAt: Date.now() };
      }
    }
    this._applyFrameToGroup(g, data?.header?.frame_id, 'posecov');
    this._dirty = true;
  }

  /** CompressedImage（jpeg/png）：createImageBitmap 解码到 2D 图像卡（与 raw Image 共卡，后到者显示）
   *  独立 100ms 节流（与 raw 共用时间戳时，同拍到达的压缩帧会被 raw 永久饿死） */
  async _onCompressedImage(data) {
    const now = performance.now();
    if (now - (this._imgcLastDraw || 0) < 100) return;
    this._imgcLastDraw = now;
    const raw = data?.data;
    if (raw == null) return;
    const fmt = String(data?.format || 'jpeg').split(';')[0].toLowerCase();
    const bytes = raw instanceof Uint8Array ? raw
      : raw instanceof ArrayBuffer ? new Uint8Array(raw)
      : Array.isArray(raw) ? new Uint8Array(raw) : null;
    if (!bytes || !bytes.length) return;
    try {
      const blob = new Blob([bytes], { type: 'image/' + (fmt === 'jpeg' ? 'jpeg' : fmt) });
      const bmp = await createImageBitmap(blob);
      const w = bmp.width, h = bmp.height;
      const cv = $('viz-image');
      if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
      this._imgCtx.drawImage(bmp, 0, 0);
      bmp.close?.();
      this.stats.image = { w, h, enc: fmt,
        frames: this.stats.image.frames + 1, lastAt: Date.now() };
      this.stats.imagec = { frames: (this.stats.imagec?.frames || 0) + 1, lastAt: Date.now() };
    } catch (e) {
      if (!this._imagecWarned) {
        this._imagecWarned = true;
        console.warn('[viz] CompressedImage 解码失败（format=' + fmt + '）:', e?.message || e);
      }
    }
  }

  _onImage(data) {
    const now = performance.now();
    if (now - this._imgLastDraw < 100) return;
    this._imgLastDraw = now;
    const cv = $('viz-image');
    const w = data.width, h = data.height;
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    const raw = data.data;
    const img = this._imgCtx.createImageData(w, h);
    const px = img.data;
    const enc = data.encoding;
    if (enc === 'mono8') {
      for (let i = 0, j = 0; i < w * h; i++, j += 4) {
        const v = raw[i];
        px[j] = px[j + 1] = px[j + 2] = v; px[j + 3] = 255;
      }
    } else if (enc === 'rgb8' || enc === 'bgr8') {
      const [R, G, B] = enc === 'rgb8' ? [0, 1, 2] : [2, 1, 0];
      for (let i = 0, j = 0; i < w * h; i++, j += 4) {
        const o = i * 3;
        px[j] = raw[o + R]; px[j + 1] = raw[o + G]; px[j + 2] = raw[o + B]; px[j + 3] = 255;
      }
    } else {
      return;
    }
    this._imgCtx.putImageData(img, 0, 0);
    this.stats.image = { w, h, enc, frames: this.stats.image.frames + 1, lastAt: Date.now() };
  }

  _onScan(data) {
    const now = performance.now();
    if (now - this._scanLastDraw < 100) return;
    this._scanLastDraw = now;
    const cv = $('viz-scan');
    const ctx = this._scanCtx;
    const W = cv.width, H = cv.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = '#0c0e12';
    ctx.fillRect(0, 0, W, H);
    const cx = W / 2, cy = H - 18;
    const maxR = data.range_max || 10;
    const s = (H - 40) / maxR;
    ctx.strokeStyle = '#1d2230';
    for (const r of [maxR * 0.33, maxR * 0.66, maxR]) {
      ctx.beginPath();
      ctx.arc(cx, cy, r * s, -Math.PI / 2 + data.angle_min, -Math.PI / 2 + data.angle_max);
      ctx.stroke();
    }
    ctx.fillStyle = '#8b93a7';
    ctx.fillRect(cx - 2, cy - 2, 4, 4);
    ctx.fillStyle = '#2ecc71';
    const ranges = data.ranges || [];
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (!isFinite(r) || r < (data.range_min || 0)) continue;
      const a = data.angle_min + i * (data.angle_increment || 0);
      ctx.fillRect(cx + Math.sin(a) * r * s - 1, cy - Math.cos(a) * r * s - 1, 2.5, 2.5);
    }
    this.stats.scan = { beams: ranges.length, frames: this.stats.scan.frames + 1, lastAt: Date.now() };
  }

  _onJoints(data) {
    const box = $('viz-joints');
    const names = data.name || [];
    const armed = document.body.classList.contains('armed');
    if (box.children.length !== names.length) {
      // 每关节两态：只读 jbar（含 i）+ armed 态 range；CSS 按交互锁切换显示
      // 滑块范围动态化：URDF 限位可用时取之（默认 ±3.2）
      const lim = (n) => {
        const j = this._urdfRobot?.joints?.[n];
        const hi = Math.min(3.2, Number(j?.limit?.upper ?? 3.2));
        const lo = Math.max(-3.2, Number(j?.limit?.lower ?? -3.2));
        return [lo, hi];
      };
      box.innerHTML = names.map((n) => {
        const [lo, hi] = lim(n);
        return `
        <div class="joint-row" data-joint="${n}">
          <span class="jn">${n}</span>
          <span class="jbar"><i></i></span>
          <input type="range" min="${lo.toFixed(2)}" max="${hi.toFixed(2)}" step="0.01">
          <span class="jv"></span>
        </div>`;
      }).join('');
      box.querySelectorAll('.joint-row').forEach((row) => {
        const name = row.dataset.joint;
        const slider = row.querySelector('input[type="range"]');
        let lastSent = 0;
        slider.addEventListener('input', () => {
          const v = parseFloat(slider.value);
          row.querySelector('.jv').textContent = v.toFixed(3);
          const now = performance.now();
          if (now - lastSent < 45) return;  // 命令节流 45ms（COMMAND_INTERVAL_MS）
          lastSent = now;
          // 镜像防环 v2：武装一次性 hold（1800ms 或到位 0.025 rad 即删）
          this.stats._mirrorHoldUntil.set(name, now + 1800);
          this.stats._mirrorTargets.set(name, v);
          const ok = this._client.publish('/rwl_viz/joint_cmd', 'sensor_msgs/msg/JointState', {
            name: names, position: names.map((n2) =>
              n2 === name ? v : (parseFloat(
                box.querySelector(`[data-joint="${n2}"] input`)?.value ?? '0') || 0)),
          });
          if (ok) this.stats.jointCmdSent++;
        });
      });
    }
    const rows = box.children;
    let maxErr = 0, worst = '', sumSq = 0, cnt = 0;
    const now = performance.now();
    names.forEach((n, i) => {
      const row = rows[i];
      const p = Number(data.position?.[i] ?? 0);
      // 镜像防环 v2：到位（|target-value|<0.025 rad）或超时 → 删 hold，放行镜像
      const hold = this.stats._mirrorHoldUntil.get(n);
      if (hold !== undefined) {
        const target = this.stats._mirrorTargets.get(n);
        const reached = typeof target === 'number' && Math.abs(target - p) < 0.025;
        if (reached || now > hold) this.stats._mirrorHoldUntil.delete(n);
      }
      const mirroring = armed && this.stats._mirrorHoldUntil.has(n);
      if (!mirroring) {
        const bar = row.querySelector('.jbar i');
        const half = Math.min(Math.max(p, -3.2), 3.2) / 3.2;
        bar.style.transform =
          `translateX(${half >= 0 ? 50 : 50 + half * 50}%) scaleX(${Math.abs(half)})`;
        if (!armed || document.activeElement !== row.querySelector('input')) {
          row.querySelector('.jv').textContent = p.toFixed(3);
          if (armed) row.querySelector('input').value = p;
        }
      }
      // 跟踪误差（rebot-ros-ui.js:676-701）：目标=最近命令值，缺省当前位
      const target = this.stats._mirrorTargets.get(n) ?? p;
      const err = Math.abs(target - p);
      if (err > maxErr) { maxErr = err; worst = n; }
      sumSq += err * err; cnt++;
    });
    if (cnt) {
      const rms = Math.sqrt(sumSq / cnt);
      this.stats.trackingError = {
        maxDeg: +(maxErr * 180 / Math.PI).toFixed(2),
        rmsDeg: +(rms * 180 / Math.PI).toFixed(2),
        worst,
      };
      const cap = $('tracking-error');
      if (cap) {
        cap.textContent = `最大 ${this.stats.trackingError.maxDeg}° ${worst} / RMS ${this.stats.trackingError.rmsDeg}°`;
        cap.style.color = maxErr < 0.035 ? 'var(--green)'
          : maxErr < 0.12 ? 'var(--amber)' : 'var(--red)';
      }
    }
    this.stats.joints = {
      names, position: (data.position || []).map(Number),
      frames: this.stats.joints.frames + 1, lastAt: Date.now(),
    };
    this._syncGhostToRobot();
  }

  /** OccupancyGrid 话题显示（RVIZ-RQT Scene3D:2976-3173 定位数学照搬） */
  _onOccupancyGrid(data) {
    const w = Number(data?.info?.width ?? 0), h = Number(data?.info?.height ?? 0);
    const res = Number(data?.info?.resolution ?? 0.05);
    if (!w || !h) return;
    const raw = data.data;
    const cv = document.createElement('canvas');
    cv.width = w; cv.height = h;
    const ctx = cv.getContext('2d');
    const img = ctx.createImageData(w, h);
    for (let i = 0; i < w * h; i++) {
      const v = Number(raw[i]);
      const j = i * 4;
      if (v < 0) { img.data[j] = 128; img.data[j + 1] = 128; img.data[j + 2] = 128; img.data[j + 3] = 120; }
      else if (v === 0) { img.data[j] = 255; img.data[j + 1] = 255; img.data[j + 2] = 255; img.data[j + 3] = 40; }
      else if (v >= 65) { img.data[j] = 20; img.data[j + 1] = 20; img.data[j + 2] = 20; img.data[j + 3] = 255; }
      else { img.data[j] = 160; img.data[j + 1] = 160; img.data[j + 2] = 160; img.data[j + 3] = 200; }
    }
    ctx.putImageData(img, 0, 0);
    const tex = new THREE.CanvasTexture(cv);
    tex.flipY = true;
    tex.minFilter = THREE.LinearFilter;
    tex.wrapS = tex.wrapT = THREE.ClampToEdgeWrapping;
    // groups.map 已在 _init3d 预建并挂 _rosFrame（Z-up 容器，ROS XY 水平铺开）
    VizPanel._clearGroup(this.groups.map);
    const mesh = new THREE.Mesh(
      new THREE.PlaneGeometry(w * res, h * res),
      new THREE.MeshBasicMaterial({
        map: tex, transparent: true, opacity: 0.8, side: THREE.DoubleSide }));
    const o = data.info.origin?.position;
    // ROS XY 平面（z-up 容器内即默认平面），中心 = origin + size/2
    mesh.position.set(
      (o?.x ?? 0) + w * res / 2, (o?.y ?? 0) + h * res / 2, o?.z ?? 0.001);
    this.groups.map.add(mesh);
    this.stats.occupancy = { w, h, res, lastAt: Date.now() };
    this._dirty = true;
  }

  // ── 关节预设缓动（moveToAngles 语义，rebot-sim.js:876-903） ──────────────

  /** RAF 帧驱动：本地插值写 URDF（bus source='trajectory'）+ 45ms 节流发布 joint_cmd */
  _stepMotion(now) {
    const m = this._motion.sample(now - this._motionStart);
    if (!m) return;
    emitCommand({ type: 'joint', joints: m.angles,
      source: 'trajectory', label: this._motionLabel || 'motion' });
    if (now - this._lastMotionPublish >= 45) {
      this._lastMotionPublish = now;
      this._publishJointCmd(m.angles);
    }
    if (m.done) {
      this._publishJointCmd(this._motion.target);
    }
  }

  /** 启动一次关节缓动（easeInOutCubic）；armed 时动画帧同步节流写 /rwl_viz/joint_cmd */
  _animateTo(target, durationMs = 850, label = 'preset') {
    if (!this._armJointNames?.length) return false;
    const current = {};
    const tgt = {};
    for (const n of this._armJointNames) {
      const j = this._urdfRobot?.joints?.[n] || this._urdfRobot?.getObjectByName?.(n);
      current[n] = Number(j?.angle ?? 0) || 0;
      tgt[n] = Number(target?.[n] ?? current[n]);
    }
    this._motion.start(current, tgt, durationMs);
    this._motionStart = performance.now();
    this._motionLabel = label;
    return true;
  }

  /** 关节命令发布（与滑块写回同链路：镜像 hold 记账 + armed 门） */
  _publishJointCmd(joints) {
    if (!document.body.classList.contains('armed')) return false;
    const names = Object.keys(joints);
    if (!names.length) return false;
    const now = performance.now();
    for (const n of names) {
      this.stats._mirrorHoldUntil.set(n, now + 1800);
      this.stats._mirrorTargets.set(n, joints[n]);
    }
    const ok = this._client.publish('/rwl_viz/joint_cmd', 'sensor_msgs/msg/JointState', {
      name: names, position: names.map((n) => joints[n]),
    });
    if (ok) this.stats.jointCmdSent++;
    return ok;
  }

  static _clearGroup(g) {
    while (g.children.length) {
      const c = g.children.pop();
      c.geometry?.dispose?.();
      // 材质贴图（OccupancyGrid CanvasTexture 等）须随材质一起释放，
      // 否则 /map 流长跑每帧泄漏一张 GPU 纹理
      c.material?.map?.dispose?.();
      c.material?.dispose?.();
    }
  }
}

let _panel = null;

/** 初始化可视化面板（幂等；连接后自动按 Displays 配置订阅） */
export function initViz() {
  if (_panel) return _panel;
  _panel = new VizPanel();
  window.__rwl_viz_stats = () => JSON.parse(JSON.stringify(_panel.stats,
    (_k, v) => (typeof v === 'bigint' ? Number(v) : v)));
  window.__rwl_viz_panel = _panel;
  return _panel;
}
