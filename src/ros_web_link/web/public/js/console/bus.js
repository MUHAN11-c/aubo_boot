// console/bus.js — 命令总线（Rebot-Arm rebot-sim.js:1708-1734 同构移植）
//
// 三条设计规则（照搬参考实现）：
//   1. 浅拷贝分发 + 逐监听者 try/catch——单个坏订阅者不破坏循环
//   2. 每条信封带 stamp + source——下游可按 45ms 节流、武装镜像 hold、过滤回环
//   3. 每个动作只发一个 joint-batch；插值帧用 emit:false 静默
//
// source 约定：slider / drag / drag-settle / solver / ros / trajectory /
//              trajectory-target / teach-replay / user / sim
// 'ros' 源在发布侧订阅者中过滤（镜像→发布回环在源头掐断）。

const listeners = new Set();

/** 发布命令（浅拷贝分发）。cmd = {type, name?, value?, joints?, source, stamp?} */
function emitCommand(cmd) {
  const envelope = {
    stamp: performance.now(),
    ...cmd,
  };
  for (const fn of [...listeners]) {
    try { fn(envelope); } catch (e) { console.warn('[bus] listener failed:', e); }
  }
}

/** 过滤到数值关节角的批量命令（jointDefs 键序由调用方提供） */
function emitJointBatch(angles, jointNames, source, label) {
  const joints = {};
  let n = 0;
  for (const name of jointNames) {
    const v = angles[name];
    if (typeof v === 'number' && Number.isFinite(v)) { joints[name] = v; n++; }
  }
  if (!n) return;
  emitCommand({ type: 'joint-batch', joints, source, label: label || '' });
}

function onCommand(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

/** 测试辅助：清空监听者 */
function _resetBus() { listeners.clear(); }

export { emitCommand, emitJointBatch, onCommand, _resetBus };
