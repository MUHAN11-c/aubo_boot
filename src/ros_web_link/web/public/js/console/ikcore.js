// console/ikcore.js — DLS IK 伺服纯核（Rebot-Arm rebot-sim.js:1455-1676 同构移植）
//
// 与 three/DOM 零耦合（可 vitest）：调用方提供 fk(angles)→{x,y,z}（ROS z-up 米制）
// 与关节限位表。算法逐参对齐参考实现：
//   gain 12 / damping λ 0.035（λ²=0.001225）/ maxJointSpeed 2.8 rad·s⁻¹
//   雅可比中心差分 eps 0.004 rad；Δθ = Jᵀ(JJᵀ+λ²I)⁻¹e（3×3 Cramer）
//   子步 16ms；到位阈值 1.5mm；settle 超时 1400ms / 目标误差 2mm

const GAIN = 12;
const DAMPING = 0.035;
const MAX_JOINT_SPEED = 2.8;     // rad/s
const JACOBIAN_EPS = 0.004;      // rad
const REACHED_TOLERANCE = 0.0015; // m
export const SETTLE_TIMEOUT_MS = 1400;
export const SETTLE_TARGET_ERROR = 0.002; // m
export const TEACH_SAMPLE_INTERVAL_MS = 90;
export const TEACH_MIN_TCP_STEP = 0.004;  // m

/** 3×3 线性解（Cramer；奇异返回 null） */
function solve3x3(a, b) {
  const [a00, a01, a02, a10, a11, a12, a20, a21, a22] = a;
  const det = a00 * (a11 * a22 - a12 * a21)
    - a01 * (a10 * a22 - a12 * a20)
    + a02 * (a10 * a21 - a11 * a20);
  if (Math.abs(det) < 1e-9) return null;
  const inv = 1 / det;
  return [
    (b[0] * (a11 * a22 - a12 * a21) - a01 * (b[1] * a22 - a12 * b[2]) + a02 * (b[1] * a21 - a11 * b[2])) * inv,
    (a00 * (b[1] * a22 - a12 * b[2]) - b[0] * (a10 * a22 - a12 * a20) + a02 * (a10 * b[2] - b[1] * a20)) * inv,
    (a00 * (a11 * b[2] - b[1] * a21) - a01 * (a10 * b[2] - b[1] * a20) + b[0] * (a10 * a21 - a11 * a20)) * inv,
  ];
}

/**
 * 阻尼最小二乘 IK 伺服器。
 * @param {{fk:function(Object<string,number>):{x:number,y:number,z:number},
 *          joints: Array<{name:string, lower:number, upper:number}>}} opts
 */
function createIKSolver(opts) {
  const { fk, joints } = opts;
  const names = joints.map((j) => j.name);
  const limits = new Map(joints.map((j) => [j.name, j]));

  function errorTo(current, target, out) {
    const p = fk(current);
    out[0] = target.x - p.x; out[1] = target.y - p.y; out[2] = target.z - p.z;
    return Math.hypot(out[0], out[1], out[2]);
  }

  /** 3×N 中心差分雅可比（两列 FK/关节） */
  function computeJacobian(base) {
    const n = names.length;
    const j = new Array(3 * n);
    const angles = { ...base };
    for (let c = 0; c < n; c++) {
      const name = names[c];
      const orig = angles[name];
      angles[name] = orig + JACOBIAN_EPS;
      const pPlus = fk(angles);
      angles[name] = orig - JACOBIAN_EPS;
      const pMinus = fk(angles);
      angles[name] = orig;
      j[c] = (pPlus.x - pMinus.x) / (2 * JACOBIAN_EPS);
      j[n + c] = (pPlus.y - pMinus.y) / (2 * JACOBIAN_EPS);
      j[2 * n + c] = (pPlus.z - pMinus.z) / (2 * JACOBIAN_EPS);
    }
    return j;
  }

  /** Δθ = Jᵀ(JJᵀ+λ²I)⁻¹e */
  function solveDampedLeastSquares(j, e) {
    const n = names.length;
    const lam2 = DAMPING * DAMPING;
    // A = JJᵀ + λ²I（3×3 行主序）
    const a = new Array(9);
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        let s = 0;
        for (let k = 0; k < n; k++) s += j[r * n + k] * j[c * n + k];
        a[r * 3 + c] = s + (r === c ? lam2 : 0);
      }
    }
    const y = solve3x3(a, e);
    if (!y) return null;
    const dtheta = new Array(n);
    for (let k = 0; k < n; k++) {
      dtheta[k] = j[k] * y[0] + j[n + k] * y[1] + j[2 * n + k] * y[2];
    }
    return dtheta;
  }

  const _err = [0, 0, 0];

  /**
   * 单步伺服（16ms 语义）。返回 {error, reached, angles}；angles 为新关节角。
   * @param {Object<string,number>} current
   * @param {{x,y,z}} target
   * @param {number} dt 秒
   */
  function servoStep(current, target, dt) {
    const errNorm = errorTo(current, target, _err);
    if (errNorm < REACHED_TOLERANCE) {
      return { error: errNorm, reached: true, angles: { ...current } };
    }
    // 误差步长钳制（0.08~0.65 × gain·dt）
    const scale = Math.min(0.65, Math.max(0.08, GAIN * dt));
    const e = [_err[0] * scale, _err[1] * scale, _err[2] * scale];
    const j = computeJacobian(current);
    const delta = solveDampedLeastSquares(j, e);
    const angles = { ...current };
    if (delta) {
      const dMax = MAX_JOINT_SPEED * dt;
      for (let i = 0; i < names.length; i++) {
        const name = names[i];
        const lim = limits.get(name);
        const d = Math.max(-dMax, Math.min(dMax, delta[i]));
        const lo = lim ? lim.lower : -Infinity;
        const hi = lim ? lim.upper : Infinity;
        angles[name] = Math.max(lo, Math.min(hi, angles[name] + d));
      }
    }
    const newErr = errorTo(angles, target, _err);
    return { error: newErr, reached: false, angles };
  }

  /** 离线求解（点击移动/规划预览）：迭代 servoStep，失败回滚 */
  function solveTarget(target, startAngles, maxIter = 240, timeoutMs = 900) {
    let angles = { ...startAngles };
    const t0 = (typeof performance !== 'undefined' ? performance.now() : Date.now());
    for (let i = 0; i < maxIter; i++) {
      const r = servoStep(angles, target, 0.016);
      angles = r.angles;
      if (r.reached) return { ok: true, angles, iterations: i + 1 };
      const now = (typeof performance !== 'undefined' ? performance.now() : Date.now());
      if (now - t0 > timeoutMs) break;
    }
    const finalErr = errorTo(angles, target, _err);
    return { ok: finalErr < 0.01, angles, iterations: maxIter, error: finalErr };
  }

  return { servoStep, solveTarget, names };
}

/**
 * 工作空间半球包络（estimateWorkspaceEnvelope 确定性采样 + clamp 合并，
 * 参考 rebot-sim.js:512-542/1173-1200）。
 * @param {{fk, joints, nominalReach?:number}} opts
 */
function estimateWorkspaceEnvelope(opts) {
  const { fk, joints, nominalReach = 0.65 } = opts;
  const names = joints.map((j) => j.name);
  const limits = new Map(joints.map((j) => [j.name, j]));
  // 确定性伪随机（sine-hash，参考 seededUnit）
  let seed = 1;
  const unit = () => {
    seed = (seed * 1103515245 + 12345) & 0x7fffffff;
    return seed / 0x7fffffff;
  };
  let maxPlanar = 0;
  let maxHeight = 0;
  const N = 960;
  for (let s = 0; s < N; s++) {
    const angles = {};
    for (const name of names) {
      const lim = limits.get(name);
      const lo = lim ? lim.lower : -Math.PI;
      const hi = lim ? lim.upper : Math.PI;
      angles[name] = lo + unit() * (hi - lo);
    }
    const p = fk(angles);
    const planar = Math.hypot(p.x, p.y);
    if (planar > maxPlanar) maxPlanar = planar;
    if (p.z > maxHeight) maxHeight = p.z;
  }
  const radius = Math.min(1.2, Math.max(nominalReach, Math.ceil(maxPlanar * 100) / 100));
  const heightLimit = Math.min(1.5, Math.max(0.3, Math.ceil(maxHeight * 100) / 100));
  return { radius, heightLimit };
}

/** 半球夹紧：越高水平半径越小（参考 clampToWorkspaceEnvelope） */
function clampToWorkspaceEnvelope(pos, env) {
  const { radius, heightLimit } = env;
  const point = { x: pos.x, y: pos.y, z: pos.z };
  let clamped = false;
  if (point.z < 0) { point.z = 0; clamped = true; }
  if (point.z > heightLimit) { point.z = heightLimit; clamped = true; }
  const verticalRatio = Math.min(1, Math.max(0, point.z / heightLimit));
  const planarLimit = Math.max(0.03,
    radius * Math.sqrt(Math.max(0, 1 - verticalRatio * verticalRatio)));
  const planar = Math.hypot(point.x, point.y);
  if (planar > planarLimit) {
    const s = planarLimit / planar;
    point.x *= s; point.y *= s;
    clamped = true;
  }
  return { point, clamped };
}

/** 航点录制闸（90ms + 4mm 双闸，参考 recordTeachingWaypoint） */
function createTeachRecorder() {
  const waypoints = [];
  let lastSampleAt = 0;
  let startedAt = null;
  let lastTcp = null;

  /**
   * @param {{t?:number, joints:Object<string,number>, tcp:{x,y,z}}} candidate
   * @param {boolean} force 起终点强制记录
   */
  function record(candidate, force = false) {
    const now = performance.now();
    if (startedAt === null) startedAt = now;
    if (!force) {
      if (now - lastSampleAt < TEACH_SAMPLE_INTERVAL_MS) return false;
      if (lastTcp) {
        const d = Math.hypot(
          candidate.tcp.x - lastTcp.x, candidate.tcp.y - lastTcp.y,
          candidate.tcp.z - lastTcp.z);
        if (d < TEACH_MIN_TCP_STEP) return false;
      }
    }
    lastSampleAt = now;
    lastTcp = { ...candidate.tcp };
    waypoints.push({
      t: now - startedAt,
      joints: { ...candidate.joints },
      tcp: { ...candidate.tcp },
    });
    return true;
  }

  return {
    record,
    reset() { waypoints.length = 0; lastSampleAt = 0; startedAt = null; lastTcp = null; },
    get waypoints() { return waypoints; },
  };
}

/** 航点 → 轨迹点（buildTrajectoryPoints 同构：首点=当前@50ms，比例重排，逐点≥0.2s） */
function buildTrajectoryPoints(recorderWaypoints, currentJoints, jointNames, totalDuration) {
  const points = [{
    positions: jointNames.map((n) => Number(currentJoints[n] || 0)),
    seconds: 0.05,
  }];
  const wps = recorderWaypoints;
  if (!wps.length) return points;
  const firstT = wps[0].t;
  const span = Math.max(1, wps[wps.length - 1].t - firstT);
  wps.forEach((wp, index) => {
    const ratio = Math.max(0, (wp.t - firstT) / span);
    const seconds = Math.max(0.2,
      index === wps.length - 1 ? totalDuration : ratio * totalDuration);
    points.push({
      positions: jointNames.map((n) => Number(wp.joints[n] || 0)),
      seconds,
    });
  });
  return points;
}

export {
  createIKSolver,
  estimateWorkspaceEnvelope,
  clampToWorkspaceEnvelope,
  createTeachRecorder,
  buildTrajectoryPoints,
  solve3x3,
  GAIN,
  DAMPING,
};
