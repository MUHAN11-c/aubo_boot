// console/viz_core.js — 可视化/动画纯核（three/DOM 零耦合，vitest 可测）
//
// 本轮深读补移植（2026-10-15，参考行号映射见 doc/io.md）：
//   easeInOutCubic + 关节运动动画器  ← rebot-sim.js:876-903 moveToAngles
//   easeInOutQuad + 回放段采样       ← rebot-sim.js:1317-1352 updateTeachingPlayback
//   XY 协方差椭圆（95% 置信）        ← RVIZ-RQT Scene3D.vue:1956-1993
//   里程计轨迹（位移阈值+环形上限）  ← RVIZ-RQT Scene3D.vue:1807-1817
//   姿态预设存取（localStorage）     ← rebot-sim.js:50-57 presets 的通用化
//（关节名不硬编码：任意机器人 URDF 关节集皆可存取）

/** easeInOutCubic（moveToAngles 预设位姿动画缓动） */
export function easeInOutCubic(t) {
  return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}

/** easeInOutQuad（示教回放段/夹爪类短动画缓动） */
export function easeInOutQuad(t) {
  return t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
}

/**
 * 关节运动动画器（moveToAngles 核心状态机）。
 * start() 记录起点/目标/时长；sample(elapsedMs) 返回插值关节角。
 * 时间由调用方提供（RAF 或测试时钟），不持有定时器。
 */
export function createJointMotion() {
  let from = null;
  let to = null;
  let dur = 1;

  return {
    /** @param {Object<string,number>} current @param {Object<string,number>} target */
    start(current, target, durationMs = 850) {
      from = { ...current };
      to = { ...target };
      dur = Math.max(durationMs || 850, 1);
      return this;
    },
    /** @returns {{angles:Object<string,number>, done:boolean, t:number}|null */
    sample(elapsedMs) {
      if (!to) return null;
      const t = Math.min(1, Math.max(0, elapsedMs / dur));
      const e = easeInOutCubic(t);
      const angles = {};
      for (const k of Object.keys(to)) {
        const a = from?.[k] ?? 0;
        angles[k] = a + (to[k] - a) * e;
      }
      return { angles, done: t >= 1, t };
    },
    get target() { return to; },
    get duration() { return dur; },
    cancel() { to = null; },
  };
}

/**
 * XY 协方差椭圆（PoseWithCovarianceStamped.pose.covariance[36] → 95% 置信椭圆）。
 * @returns {{a:number, b:number, angle:number}} 半轴（米）与主轴角（弧度，XY 平面内）
 */
export function covarianceEllipse(cov36) {
  const cxx = Number(cov36?.[0] ?? 0);
  const cyy = Number(cov36?.[7] ?? 0);
  const cxy = Number(cov36?.[1] ?? 0);
  const trace = cxx + cyy;
  const det = cxx * cyy - cxy * cxy;
  const disc = Math.max(0, (trace * trace) / 4 - det);
  const l1 = trace / 2 + Math.sqrt(disc);
  const l2 = trace / 2 - Math.sqrt(disc);
  return {
    a: 2 * Math.sqrt(Math.max(l1, 0)),
    b: 2 * Math.sqrt(Math.max(l2, 0)),
    angle: 0.5 * Math.atan2(2 * cxy, cxx - cyy),
  };
}

/**
 * 里程计轨迹采样器：位移阈值去抖 + 点数上限（环形淘汰）。
 * @param {number} maxPoints 轨迹点上限（clamp 10~1000）
 * @param {number} minStep 追加所需最小位移（米；参考 0.1）
 */
export function createOdomTrail(maxPoints = 100, minStep = 0.1) {
  const cap = Math.min(1000, Math.max(10, Math.round(maxPoints)));
  const step = Math.max(1e-6, minStep);
  const pts = [];
  let last = null;
  return {
    /** @returns {boolean} 是否追加了点 */
    push(x, y, z) {
      if (last && Math.hypot(x - last[0], y - last[1], z - last[2]) < step) return false;
      last = [x, y, z];
      pts.push([x, y, z]);
      if (pts.length > cap) pts.shift();
      return true;
    },
    get points() { return pts; },
    get capacity() { return cap; },
    clear() { pts.length = 0; last = null; },
  };
}

/** 回放段时长钳制（80~900ms，按录制原始间隔） */
export function replaySegmentDuration(prevT, nextT, minMs = 80, maxMs = 900) {
  return Math.min(maxMs, Math.max(minMs, nextT - prevT));
}

/** 回放段插值采样：prev→next 关节角，t01 线性比例（内部再过缓动） */
export function sampleSegment(prev, next, t01, ease = easeInOutQuad) {
  const e = ease(Math.min(1, Math.max(0, t01)));
  const out = {};
  for (const k of Object.keys(next)) {
    const a = prev?.[k] ?? 0;
    out[k] = a + (next[k] - a) * e;
  }
  return out;
}

/**
 * 姿态预设存取（localStorage 持久化；名字→关节角表）。
 * storage 参数可注入（测试用内存 stub）。
 */
export function createPresetStore(storage) {
  const KEY = 'rwl-joint-presets-v1';
  const ok = storage && typeof storage.getItem === 'function';
  const load = () => {
    if (!ok) return {};
    try { return JSON.parse(storage.getItem(KEY) || '{}'); } catch (_) { return {}; }
  };
  const persist = (map) => {
    if (ok) {
      try { storage.setItem(KEY, JSON.stringify(map)); } catch (_) { /* 满/隐私模式 */ }
    }
  };
  return {
    /** @returns {Array<{name:string, joints:Object<string,number>}>} */
    list() {
      return Object.entries(load())
        .filter(([, v]) => v && typeof v === 'object')
        .map(([name, joints]) => ({ name, joints: { ...joints } }));
    },
    save(name, joints) {
      if (!name) return false;
      const map = load();
      map[name] = { ...joints };
      persist(map);
      return true;
    },
    remove(name) {
      const map = load();
      const had = name in map;
      delete map[name];
      persist(map);
      return had;
    },
  };
}

// ── Displays 实例配置（RViz Add-Display 语义；sim_demo 官方数据面默认值） ───

/** 显示类型注册表：singleton=单实例不可增删；topic=官方数据面默认话题 */
export const DISPLAY_TYPES = {
  grid:    { label: '网格 Grid', singleton: true },
  robot:   { label: 'RobotModel URDF', singleton: true, topic: '/robot_description', props: ['ghost'] },
  tf:      { label: 'TF 坐标轴', singleton: true, topic: '/tf', props: ['labels'] },
  map:     { label: '地图 OccupancyGrid', topic: '/map', props: ['opacity'] },
  costmap: { label: '代价地图 Costmap', topic: '/global_costmap/costmap', props: ['opacity'] },
  cloud:   { label: '点云 PointCloud2', topic: '/test_cloud', props: ['size'] },
  scan3d:  { label: '激光 3D LaserScan', topic: '/scan' },
  marker:  { label: '标记 MarkerArray', topic: '' },
  path:    { label: '路径 Path', topic: '/plan' },
  poses:   { label: '位姿数组 PoseArray', topic: '/particle_cloud' },
  odom:    { label: '里程 Odometry', topic: '/odom', props: ['trail', 'trailLen'] },
  range:   { label: '测距 Range', topic: '' },
  wrench:  { label: '力/力矩 Wrench', topic: '', props: ['scale'] },
  posecov: { label: '协方差位姿 PoseCov', topic: '/amcl_pose' },
  image:   { label: '相机 Image', topic: '/camera/image_raw' },
  imagec:  { label: '压缩图 CompressedImage', topic: '/camera/image_raw/compressed' },
};

const INSTANCE_DEFAULT_PROPS = {
  robot: { ghost: false },
  tf: { labels: true },
  map: { opacity: 0.85 },
  costmap: { opacity: 0.5 },
  cloud: { size: 0.035 },
  odom: { trail: true, trailLen: 300 },
  wrench: { scale: 0.2 },
};

export function makeInstance(id, type, name, overrides = {}) {
  const spec = DISPLAY_TYPES[type];
  return { id, type, name, on: true, topic: spec.topic || '',
    ...(INSTANCE_DEFAULT_PROPS[type] || {}), ...overrides };
}

export function defaultInstances() {
  return [
    makeInstance('grid', 'grid', '网格'),
    makeInstance('robot', 'robot', 'RobotModel'),
    makeInstance('tf', 'tf', 'TF'),
    makeInstance('map1', 'map', '地图'),
    makeInstance('gcm1', 'costmap', '全局代价地图'),
    makeInstance('lcm1', 'costmap', '局部代价地图',
      { topic: '/local_costmap/costmap' }),
    makeInstance('cloud1', 'cloud', '点云'),
    makeInstance('scan1', 'scan3d', '激光'),
    makeInstance('path1', 'path', '路径'),
    makeInstance('poses1', 'poses', '粒子云'),
    makeInstance('odom1', 'odom', '里程'),
    makeInstance('posecov1', 'posecov', 'AMCL 位姿'),
    makeInstance('image1', 'image', '相机'),
  ];
}

export function defaultGlobal() {
  // fixedFrame ''=自动（TF 树根）；follow ''=自由视角，否则跟随 TF 帧（Foxglove 语义）
  return { fixedFrame: '', bg: '', follow: '' };
}

export function defaultCfg() {
  return { global: defaultGlobal(), instances: defaultInstances() };
}

/** v3 固定键配置 → v4 实例数组迁移（保留用户自定义 topic/on/属性） */
export function migrateCfg(raw) {
  const base = defaultCfg();
  if (raw && Array.isArray(raw.instances)) {
    return { global: { ...defaultGlobal(), ...(raw.global || {}) },
      instances: raw.instances };
  }
  if (raw) {
    for (const inst of base.instances) {
      const old = raw[inst.type];
      if (!old) continue;
      inst.topic = old.topic || inst.topic;
      inst.on = old.on !== false;
      for (const k of Object.keys(INSTANCE_DEFAULT_PROPS[inst.type] || {})) {
        if (old[k] !== undefined) inst[k] = old[k];
      }
    }
  }
  return base;
}

/**
 * OccupancyGrid 地图语义像素（RViz2 Map 显示色板）。
 * -1=未知(灰) / 0=自由(白,低α) / ≥65=占据(黑) / 其余=灰。
 * 返回 [r, g, b, a]。
 */
export function mapGridPixel(v) {
  if (v < 0) return [127, 127, 127, 120];
  if (v === 0) return [255, 255, 255, 40];
  if (v >= 65) return [10, 10, 10, 255];
  return [127, 127, 127, 200];
}

/**
 * Costmap 代价语义像素（RViz2 Costmap 显示色带）：
 * -1=未知(暗灰) / 0=自由(淡蓝,极低α) / 1..100=代价蓝→红渐变(α 随代价升高)。
 */
export function costmapPixel(v) {
  if (v < 0) return [80, 80, 80, 90];
  if (v === 0) return [70, 130, 220, 16];
  const t = Math.min(1, v / 100);
  return [
    Math.round(30 + 225 * t),
    Math.round(90 - 50 * t),
    Math.round(255 - 225 * t),
    Math.round(80 + 160 * t),
  ];
}

/**
 * Teleop 命令混合器（键盘集合 → Twist 速度；纯函数，vitest 可测）。
 * keys: Set<string>（'w'/'a'/'s'/'d'/'arrowup'…小写）；maxLin m/s、maxAng rad/s。
 * 松键（空集）= 零速急停。
 */
export function mixTeleopCommand(keys, maxLin, maxAng) {
  let x = 0;
  let w = 0;
  if (keys.has('w') || keys.has('arrowup')) x += maxLin;
  if (keys.has('s') || keys.has('arrowdown')) x -= maxLin * 0.6;
  if (keys.has('a') || keys.has('arrowleft')) w += maxAng;
  if (keys.has('d') || keys.has('arrowright')) w -= maxAng;
  if (keys.has('q')) w += maxAng;
  if (keys.has('e')) w -= maxAng;
  const r3 = (v) => Math.round(v * 1000) / 1000;
  return {
    active: x !== 0 || w !== 0,
    twist: {
      linear: { x: r3(x), y: 0, z: 0 },
      angular: { x: 0, y: 0, z: r3(w) },
    },
  };
}
