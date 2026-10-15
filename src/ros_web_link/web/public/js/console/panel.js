// console/panel.js — 右栏新面板接线（示教/轨迹 + 拓扑图 + 分割条）
//
// 依赖 VizPanel（含 _teach/_armJointNames/_urdfRobot）。轨迹发送：
// FollowJointTrajectory action 真执行（经 ros_client action 组合），
// 失败（无 schema/超时）提示控制器未在环（viz_fixture 删除后无低级回退；Rebot-Arm
// replayTrajectoryLowLevel 同构）。guardedCall：交互锁 + 确认弹窗。

import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';
import { createActionClient } from '../transport/foxglove/action_client.js';
import { buildTrajectoryPoints } from './ikcore.js';
import { createPresetStore, replaySegmentDuration, sampleSegment } from './viz_core.js';
import { emitCommand } from './bus.js';
import { TopologyGraph } from './topo.js';
import { logBus } from '../core/log-bus.js';

const $ = (id) => document.getElementById(id);

const client = foxgloveRosClient();

/** guardedCall（rebot-ros-ui.js:573-596 同构）：锁 + 确认 + 乐观状态 + 错误回读 */
function guardedCall(action, optimisticMsg) {
  if (!client.isConnected) { alert('先连接'); return null; }
  if (!document.body.classList.contains('armed')) {
    alert('交互锁未开启（头部「交互锁」）——轨迹执行需先解锁');
    return null;
  }
  if (!window.confirm('确认发送这条轨迹到机械臂？（mock 演示闭环）')) return null;
  const state = $('traj-state');
  state.textContent = optimisticMsg;
  state.style.color = 'var(--amber)';
  return action().then((r) => {
    state.textContent = '✓ 完成';
    state.style.color = 'var(--green)';
    return r;
  }).catch((e) => {
    state.textContent = '✗ ' + String(e.message || e).slice(0, 60);
    state.style.color = 'var(--red)';
    throw e;
  });
}

function secsToRosTime(s) {
  return { sec: Math.floor(s), nanosec: Math.round((s % 1) * 1e9) };
}

let lowLevelCancel = false;

function initTrajectoryPanel(viz) {
  const refreshStats = () => {
    const t = viz._teach;
    $('traj-waypoints').textContent = t ? t.stats.waypoints : 0;
    const e = t && t.stats.dragError != null ? (t.stats.dragError * 1000).toFixed(0) + ' mm' : '—';
    $('traj-drag-error').textContent = e;
  };
  setInterval(refreshStats, 500);

  $('btn-teach-toggle').addEventListener('click', () => {
    const t = viz._teach;
    if (!t) return alert('RobotModel 加载后可用（需 URDF 含 revolute 关节）');
    t.toggle(!t.enabled);
    $('btn-teach-toggle').classList.toggle('active-tool', t.enabled);
    $('traj-state').textContent = t.enabled ? '拖拽 TCP 环' : '待示教';
  });
  $('btn-teach-record').addEventListener('click', () => {
    const t = viz._teach;
    if (!t) return;
    t.toggleRecording(!t.recording);
    $('btn-teach-record').classList.toggle('active-tool', t.recording);
    $('traj-state').textContent = t.recording ? '● 录制中' : '录制暂停';
  });
  $('btn-teach-clear').addEventListener('click', () => {
    viz._teach?._recorder.reset();
    if (viz._teach) viz._teach.stats.waypoints = 0;
    $('traj-out').textContent = '—';
  });
  $('btn-teach-export').addEventListener('click', () => {
    const t = viz._teach;
    if (!t) return;
    const data = t.exportWaypoints(viz._armJointNames || []);
    if (!data) return alert('无航点');
    $('traj-out').textContent = JSON.stringify(data, null, 1);
    navigator.clipboard?.writeText(JSON.stringify(data)).catch(() => {});
  });

  $('btn-traj-send').addEventListener('click', () => {
    const t = viz._teach;
    if (!t || !t.recorder.waypoints.length) return alert('先录制航点');
    const jointNames = viz._armJointNames;
    const current = {};
    for (const n of jointNames) {
      const j = viz._urdfRobot?.joints?.[n] || viz._urdfRobot?.getObjectByName?.(n);
      current[n] = Number(j?.angle ?? 0) || 0;
    }
    const duration = Math.min(30, Math.max(1, parseFloat($('traj-duration').value) || 6));
    const points = buildTrajectoryPoints(t.recorder.waypoints, current, jointNames, duration);

    const actionName = $('traj-action').value.trim()
      || '/panda_arm_controller/follow_joint_trajectory';
    const send = () => guardedCall(async () => {
      const ac = createActionClient(actionName);
      const { status, statusText, result } = await ac.sendGoal({
        trajectory: {
          joint_names: jointNames,
          points: points.map((p) => ({
            positions: p.positions, velocities: [], accelerations: [],
            time_from_start: secsToRosTime(p.seconds),
          })),
        },
        goal_tolerance: [], path_tolerance: [], goal_time_tolerance: secsToRosTime(0),
      }, { resultTimeoutMs: duration * 1000 + 15000 });
      ac.dispose();
      if (status !== 4) throw new Error(`终态 ${statusText}`);
      $('traj-out').textContent = `FJT ✓ ${points.length} 点 · ${duration}s\n`
        + `误差码 ${result?.error_code ?? '—'}`;
      return result;
    }, 'FJT 执行中…');
    const p = send();
    if (!p) return;
    p.catch((e) => {
      // viz_fixture 删除后无 joint_cmd 低级回退：FJT 需官方控制器在环
      // （TB3: /joint_trajectory_controller；panda: /panda_arm_controller）
      $('traj-state').textContent = '✗ FJT 失败（控制器在环？）' + String(e.message || e).slice(0, 40);
      $('traj-state').style.color = 'var(--red)';
    });
  });
  $('btn-traj-cancel').addEventListener('click', () => { lowLevelCancel = true; });

  // ── 姿态预设（rebot-sim.js:50-57/745-759 通用化：零位内置 + 用户存档） ──
  const presetSel = $('preset-select');
  const store = createPresetStore(window.localStorage);
  const refreshPresets = () => {
    presetSel.innerHTML = '<option value="">— 选择预设 —</option>' +
      '<option value="__zero__">零位 Zero（全 0）</option>' +
      store.list().map((p) => `<option value="${p.name}">${p.name}</option>`).join('');
  };
  refreshPresets();
  const applyPreset = () => {
    const v = presetSel.value;
    if (!v) return;
    if (!viz._armJointNames?.length) {
      return alert('RobotModel 加载后可用（需 URDF 含 revolute 关节）');
    }
    const saved = v === '__zero__' ? null : store.list().find((p) => p.name === v);
    if (v !== '__zero__' && !saved) return;
    const target = {};
    for (const n of viz._armJointNames) target[n] = saved ? (saved.joints[n] ?? 0) : 0;
    viz._animateTo(target, 850, 'preset:' + (v === '__zero__' ? 'zero' : v));
    const state = $('traj-state');
    state.textContent = v === '__zero__' ? '→ 零位（850ms 缓动）' : `→ 预设 ${v}`;
    state.style.color = 'var(--amber)';
  };
  $('btn-preset-apply').addEventListener('click', applyPreset);
  presetSel.addEventListener('change', applyPreset);
  $('btn-preset-save').addEventListener('click', () => {
    if (!viz._armJointNames?.length) return alert('RobotModel 加载后可用');
    const name = prompt('预设名称：');
    if (!name) return;
    const joints = {};
    for (const n of viz._armJointNames) {
      const j = viz._urdfRobot?.joints?.[n] || viz._urdfRobot?.getObjectByName?.(n);
      joints[n] = Number(j?.angle ?? 0) || 0;
    }
    store.save(name, joints);
    refreshPresets();
    presetSel.value = name;
  });
  $('btn-preset-del').addEventListener('click', () => {
    const v = presetSel.value;
    if (!v || v === '__zero__') return;
    store.remove(v);
    refreshPresets();
  });

  // ── 示教本地回放（rebot-sim.js:1298-1352 同构；纯前端可视化验证，不发 ROS） ──
  $('btn-teach-replay').addEventListener('click', async () => {
    const t = viz._teach;
    const wps = t?.recorder?.waypoints || [];
    if (wps.length < 2) return alert('至少 2 个航点（先录制）');
    const btn = $('btn-teach-replay');
    btn.disabled = true;
    const state = $('traj-state');
    state.textContent = '▶ 本地回放中…';
    state.style.color = 'var(--amber)';
    try {
      for (let i = 1; i < wps.length; i++) {
        // 段时长按录制原始间隔（80~900ms 钳制），段内 easeInOutQuad
        const dur = replaySegmentDuration(wps[i - 1].t, wps[i].t);
        const t0 = performance.now();
        await new Promise((resolve) => {
          const step = () => {
            const k = Math.min(1, (performance.now() - t0) / dur);
            const joints = sampleSegment(wps[i - 1].joints, wps[i].joints, k);
            emitCommand({ type: 'joint', joints, source: 'teach-replay', label: 'replay' });
            if (k < 1) requestAnimationFrame(step);
            else resolve();
          };
          requestAnimationFrame(step);
        });
      }
      state.textContent = '✓ 回放完成';
      state.style.color = 'var(--green)';
    } finally {
      btn.disabled = false;
      setTimeout(() => { state.style.color = ''; }, 1500);
    }
  });

  // ── 工作空间包络线框开关（TeachController 内渲染，viz 记忆偏好） ─────────
  const envBox = $('env-show');
  if (envBox) {
    envBox.checked = viz._envShow !== false;
    envBox.addEventListener('change', () => {
      viz._envShow = envBox.checked;
      localStorage.setItem('rwl-env-show', envBox.checked ? '1' : '0');
      if (viz._teach) viz._teach.showEnvelope = envBox.checked;
    });
  }
}

function initTopology(viz, topicsCard) {
  const container = $('topo-container');
  if (!container) return;
  const graph = new TopologyGraph({
    container,
    onTopicClick: (topic) => {
      // 点击话题 → 填入回显输入 + 顺手订阅（话题卡 toggle）
      $('sub-topic').value = topic;
      topicsCard?.toggle(topic, true);
    },
    hzOf: (topic) => {
      const snap = window.__rwl_topics ? window.__rwl_topics() : [];
      const row = snap.find((r) => r.topic === topic);
      return row ? row.hz : null;
    },
  });
  setInterval(() => {
    if (!client.isConnected) return;
    try {
      let g = client.getGraph();
      // rqt_graph 语义过滤：服务显隐 + 名称搜索（拓扑图 SVG 内按文本淡化）
      const svc = $('topo-show-svc');
      if (svc && !svc.checked && g) {
        g = { ...g, advertisedServices: {} };
      }
      graph.update(g);
      const q = ($('topo-search')?.value || '').trim().toLowerCase();
      const svg = document.querySelector('#topo-container svg');
      if (svg) {
        svg.querySelectorAll('text').forEach((t) => {
          t.setAttribute('opacity',
            !q || (t.textContent || '').toLowerCase().includes(q) ? '1' : '0.15');
        });
      }
    } catch (_) { /* 无 capability */ }
  }, 3000);
}

/** 列间分割条（MainLayout.vue:826-867 同构：30-80% 钳制） */
function initSplitter() {
  const splitter = $('col-splitter');
  const grid = document.querySelector('main.app-grid');
  if (!splitter || !grid) return;
  // 分割条位置持久化 + 布局快照钩子（__rwl_getSplit/__rwl_setSplit）
  window.__rwl_getSplit = () => {
    const right = parseFloat(
      getComputedStyle(grid).gridTemplateColumns.split(' ')[2]) || 380;
    return Math.round((1 - right / window.innerWidth) * 100);
  };
  window.__rwl_setSplit = (pct) => {
    const vw = window.innerWidth || 1600;
    const right = Math.max(240, vw * (1 - Math.min(80, Math.max(30, pct)) / 100));
    grid.style.gridTemplateColumns = `minmax(0, ${pct}fr) 6px ${right}px`;
  };
  const saved = parseFloat(localStorage.getItem('rwl-split') || '');
  if (!Number.isNaN(saved)) window.__rwl_setSplit(saved);
  splitter.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    splitter.setPointerCapture(e.pointerId);
    splitter.classList.add('active');
    document.body.style.userSelect = 'none';
    document.body.style.cursor = 'col-resize';
    const startX = e.clientX;
    const startRight = parseFloat(
      getComputedStyle(grid).gridTemplateColumns.split(' ')[2]) || 380;
    const move = (ev) => {
      const delta = (ev.clientX - startX);
      const vw = window.innerWidth;
      const pct = Math.min(80, Math.max(30, (1 - (startRight - delta) / vw) * 100));
      grid.style.gridTemplateColumns = `minmax(0, ${pct}fr) 6px ${
        Math.max(240, startRight - delta)}px`;
    };
    const up = () => {
      const pct = window.__rwl_getSplit();
      try { localStorage.setItem('rwl-split', String(pct)); } catch (_) {}
      splitter.releasePointerCapture(e.pointerId);
      splitter.classList.remove('active');
      document.body.style.userSelect = '';
      document.body.style.cursor = '';
      splitter.removeEventListener('pointermove', move);
      splitter.removeEventListener('pointerup', up);
    };
    splitter.addEventListener('pointermove', move);
    splitter.addEventListener('pointerup', up);
  });
}

export function initPanels(viz, topicsCard) {
  initTrajectoryPanel(viz);
  initTopology(viz, topicsCard);
  initSplitter();
}
