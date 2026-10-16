// console/app.js — 传输诊断台交互层
//
// 复用通信层单例（与 IVG 面板同一 foxglove ros_client），把 ROS 生态各通信原语
// 暴露成可操作的表单：话题回显/发布、服务调用、action goal/feedback/取消、
// 参数读写、/tf 浏览、图谱浏览，外加一键全链路自检（对照 E2E 测试矩阵）。
// 自检结果写 window.__rwl_selftest 供浏览器自动化读取。
//
// 依赖：/js/transport/foxglove/ros_client.js、action_client.js（本包 share 静态）。

import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';
import { createActionClient } from '../transport/foxglove/action_client.js';
import { initViz, DISPLAY_TYPES } from './viz.js';
import { initPlot } from './plot.js';
import { initTopics } from './topics.js';
import { initPanels } from './panel.js';
import { initMonPanels } from './monpanels.js';
import { initTeleop } from './teleop.js';

const $ = (id) => document.getElementById(id);
const fmt = (v) => {
  try {
    return JSON.stringify(v, (_k, val) => (val instanceof Uint8Array ? Array.from(val) : val), 2);
  } catch (_) {
    return String(v);
  }
};

const client = foxgloveRosClient();
let currentActionClient = null;

// ── 连接 ────────────────────────────────────────────────────────────────────

function defaultWsUrl() {
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${proto}//${location.host}/ws/foxglove`;
}

function setConnPill(state, text) {
  const pill = $('conn-pill');
  pill.className = 'pill ' + (state || '');
  $('conn-pill-text').textContent = text;
  // ConnectionStatus 模式（RVIZ-RQT）：pill 即开关，title 展开详情
  const st = client.stats;
  const subs = client.player?._topicSubs?.size ?? 0;
  pill.title = `${text} · session: ${st.sessionId ?? '—'} · 订阅: ${subs} · 重连: ${st.reconnects}` +
    `\n点击${state === 'on' ? '断开' : '连接'}`;
}

function refreshConnInfo() {
  const st = client.stats;
  const brief = `session ${st.sessionId ?? '—'} · ${st.messagesReceived} msg · ${st.reconnects} 重连`;
  $('conn-detail').textContent = brief;
  $('conn-detail-2').textContent =
    `session: ${st.sessionId ?? '—'} · 消息: ${st.messagesReceived} · ` +
    `字节: ${st.bytesReceived} · 服务调用: ${st.serviceCalls} · 重连: ${st.reconnects}`;
  const caps = client.isConnected ? (client.player?.capabilities || []) : [];
  $('caps-pill').textContent = 'caps: ' + (caps.length ? caps.join(', ') : '—');
}

async function connect() {
  const url = $('ws-url').value.trim() || defaultWsUrl();
  setConnPill('warn', '连接中…');
  try {
    await client.connect(url);
  } catch (e) {
    // ros_client 首连失败后会自行退避重连——这里只反映瞬时状态
    setConnPill('err', '连接失败（重试中）');
    console.warn('[console] connect:', e);
    return;
  }
}

// 连接 pill 可点：connected→断开；否则→连接
$('conn-pill').addEventListener('click', () => {
  if (client.isConnected) client.disconnect();
  else connect();
});

client.onStateChange((s) => {
  if (s === 'connected') {
    setConnPill('on', '已连接');
    $('btn-connect').disabled = true;
    $('btn-disconnect').disabled = false;
    refreshGraph();
  } else if (s === 'closed') {
    setConnPill('err', '已断开');
    $('btn-connect').disabled = false;
    $('btn-disconnect').disabled = true;
  } else {
    setConnPill('warn', s);
  }
});
setInterval(refreshConnInfo, 1000);

// ── 交互锁（Rebot-Arm control-lock 模式；armed 时关节滑块写回） ───────────
const lockRow = $('lock-row');
const lockBox = $('interact-lock');
lockRow.addEventListener('click', (e) => {
  if (e.target !== lockBox) lockBox.checked = !lockBox.checked;
  const armed = lockBox.checked;
  document.body.classList.toggle('armed', armed);
  lockRow.classList.toggle('armed', armed);
  $('joints-mode').textContent = armed ? '(写回 ON)' : '(只读)';
  $('joints-mode').style.color = armed ? 'var(--amber)' : 'var(--muted)';
});
$('btn-selftest-mini').addEventListener('click', () => {
  document.getElementById('sec-selftest').open = true;
  runSelfTest();
  document.getElementById('sec-selftest').scrollIntoView({ block: 'nearest' });
});

// ── 卡片全屏（RVIZ-RQT per-panel fullscreen 模式） ─────────────────────────
document.querySelectorAll('[data-fs]').forEach((btn) => {
  btn.addEventListener('click', () => {
    const card = btn.closest('section.card, details.card');
    if (!card) return;
    if (document.fullscreenElement) document.exitFullscreen();
    else card.requestFullscreen().catch(() => {});
  });
});

// ── 图谱 ────────────────────────────────────────────────────────────────────

let graphTab = 'topics';
let graphTimer = null;

document.querySelectorAll('.tabs button').forEach((btn) => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.tabs button').forEach((b) => b.classList.remove('active'));
    btn.classList.add('active');
    graphTab = btn.dataset.tab;
    refreshGraph();
  });
});

function refreshGraph() {
  if (!client.isConnected) return;
  let snap;
  try {
    snap = client.getGraph();
  } catch (_) {
    return; // 服务器无 connectionGraph 能力
  }
  const list = $('graph-list');
  list.innerHTML = '';
  let items = [];
  if (graphTab === 'topics') items = snap.topics.map((t) => [t, '']);
  else if (graphTab === 'services') items = snap.services.map((s) => [s, '']);
  else if (graphTab === 'actions') {
    // 动作名从 action 内部话题推导（rqt 动作列表语义；bridge 图谱无独立动作实体）
    const acts = new Set(snap.topics
      .filter((t) => t.includes('/_action/'))
      .map((t) => t.split('/_action/')[0]));
    items = [...acts].map((a) => [a, '']);
  } else items = snap.nodes.map((n) => [n, '']);
  $('graph-hint').textContent = `${items.length} 项 · 点击带入表单`;
  for (const [name] of items) {
    const li = document.createElement('li');
    li.innerHTML = `<span>${name}</span>`;
    li.addEventListener('click', () => {
      if (graphTab === 'topics') {
        $('sub-topic').value = name;
        const ch = client.player.channels.get(name);
        if (ch) {
          $('pub-topic').value = name;
          $('pub-type').value = ch.schemaName || '';
        }
      } else if (graphTab === 'services') {
        $('svc-name').value = name;
      } else {
        $('param-node').value = name;
      }
    });
    list.appendChild(li);
  }
}

// ── 话题回显 ────────────────────────────────────────────────────────────────

let subHandle = null;
let subCount = 0;
let subLast = 0;
let subEchoPending = null;   // 回显渲染节流：最多 400ms 一帧（原逐帧渲染是大消息时的 CPU 热点）

$('btn-sub').addEventListener('click', () => {
  if (!client.isConnected) return alert('先连接');
  const topic = $('sub-topic').value.trim();
  if (subHandle) { try { subHandle.cancel(); } catch (_) {} }
  subCount = 0;
  subHandle = client.subscribe(topic, '', (m) => {
    subCount++;
    const now = performance.now();
    const dt = now - subLast;
    subLast = now;
    const hz = dt > 0 && subCount > 1 ? (1000 / dt).toFixed(1) : '—';
    $('sub-hz').textContent = `${subCount} 条 · ${hz} Hz`;
    subEchoPending = m;
  }, { dispatchMode: 'latest' });
  $('btn-unsub').disabled = false;
  $('sub-out').textContent = '等待消息…';
});

setInterval(() => {  // 回显渲染节流（400ms）
  if (!subEchoPending) return;
  const m = subEchoPending;
  subEchoPending = null;
  $('sub-out').textContent = `[${m.type || '?'}]\n${fmt(m.data)}`;
}, 400);

$('btn-unsub').addEventListener('click', () => {
  if (subHandle) { try { subHandle.cancel(); } catch (_) {} subHandle = null; }
  $('btn-unsub').disabled = true;
});

// ── 话题发布 ────────────────────────────────────────────────────────────────

$('btn-pub').addEventListener('click', () => {
  if (!client.isConnected) return alert('先连接');
  const topic = $('pub-topic').value.trim();
  const type = $('pub-type').value.trim();
  let msg;
  try {
    msg = JSON.parse($('pub-json').value);
  } catch (e) {
    return ($('pub-out').textContent = 'JSON 解析失败: ' + e.message);
  }
  // 通道 schema 未必已到（advertise 首帧竞态）：短等重试
  let tries = 0;
  const attempt = () => {
    const ok = client.publish(topic, type, msg);
    if (ok) {
      $('pub-out').textContent = `已发布 → ${topic} (${type})\n${fmt(msg)}`;
    } else if (++tries < 10) {
      setTimeout(attempt, 400);
    } else {
      $('pub-out').textContent =
        `发布失败：${topic} 不在图上或 schema 未就绪（客户端发布需图上已有该话题）`;
    }
  };
  attempt();
});

// ── 服务调用 ────────────────────────────────────────────────────────────────

// ── 服务调用（历史可重发，rqt_service_caller 语义） ─────────────────────────

const SVC_HIST_KEY = 'rwl-svc-hist-v1';
function svcHist() {
  try { return JSON.parse(localStorage.getItem(SVC_HIST_KEY) || '[]'); }
  catch (_) { return []; }
}
function svcHistPush(entry) {
  const all = svcHist().filter((h) => h.name !== entry.name);
  all.unshift(entry);
  try { localStorage.setItem(SVC_HIST_KEY, JSON.stringify(all.slice(0, 20))); } catch (_) {}
  refreshSvcHist();
}
function refreshSvcHist() {
  const sel = $('svc-hist');
  if (!sel) return;
  const all = svcHist();
  sel.innerHTML = `<option value="">历史（${all.length}）</option>`
    + all.map((h, i) => `<option value="${i}">${h.name}</option>`).join('');
}
refreshSvcHist();
$('svc-hist')?.addEventListener('change', (e) => {
  const h = svcHist()[+e.target.value];
  if (!h) return;
  $('svc-name').value = h.name;
  $('svc-json').value = JSON.stringify(h.req, null, 1);
});

$('btn-svc').addEventListener('click', async () => {
  if (!client.isConnected) return alert('先连接');
  const name = $('svc-name').value.trim();
  let req;
  try {
    req = JSON.parse($('svc-json').value || '{}');
  } catch (e) {
    return ($('svc-out').textContent = 'JSON 解析失败: ' + e.message);
  }
  $('svc-out').textContent = '调用中…';
  try {
    const resp = await client.callService(name, req, { timeoutMs: 15000 });
    $('svc-out').textContent = fmt(resp);
    svcHistPush({ name, req, at: new Date().toISOString() });
  } catch (e) {
    $('svc-out').textContent = '失败: ' + (e.message || e);
  }
});

// ── Action ──────────────────────────────────────────────────────────────────

$('btn-act-send').addEventListener('click', async () => {
  if (!client.isConnected) return alert('先连接');
  const name = $('act-name').value.trim();
  let goal;
  try {
    goal = JSON.parse($('act-json').value || '{}');
  } catch (e) {
    return ($('act-out').textContent = 'JSON 解析失败: ' + e.message);
  }
  if (currentActionClient) { try { currentActionClient.dispose(); } catch (_) {} }
  currentActionClient = createActionClient(name);
  $('act-out').textContent = 'goal 已提交，等待…';
  $('btn-act-cancel').disabled = false;
  try {
    const { status, statusText, result } = await currentActionClient.sendGoal(goal, {
      onFeedback: (fb) => {
        $('act-out').textContent += `\n[feedback] ${fmt(fb)}`;
        $('act-out').scrollTop = $('act-out').scrollHeight;
      },
    });
    $('act-out').textContent += `\n[终态 ${status} ${statusText}] ${fmt(result)}`;
  } catch (e) {
    $('act-out').textContent += `\n[失败] ${e.message || e}`;
  } finally {
    $('btn-act-cancel').disabled = true;
  }
});

$('btn-act-cancel').addEventListener('click', async () => {
  if (!currentActionClient) return;
  try {
    const r = await currentActionClient.cancelAll();
    $('act-out').textContent += `\n[cancel] return=${r.returnCode} canceling=${r.goalsCanceling.length}`;
  } catch (e) {
    $('act-out').textContent += `\n[cancel 失败] ${e.message || e}`;
  }
});

// ── 参数 ────────────────────────────────────────────────────────────────────

$('btn-param-get').addEventListener('click', async () => {
  if (!client.isConnected) return alert('先连接');
  const node = $('param-node').value.trim();
  const pname = $('param-name').value.trim();
  try {
    const values = await client.getParams(node, pname ? [pname] : []);
    $('param-out').textContent = fmt(values);
  } catch (e) {
    $('param-out').textContent = '失败: ' + (e.message || e);
  }
});

$('btn-param-set').addEventListener('click', async () => {
  if (!client.isConnected) return alert('先连接');
  const node = $('param-node').value.trim();
  const pname = $('param-name').value.trim();
  if (!pname) return ($('param-out').textContent = '设置需指定参数名');
  let raw = $('param-value').value;
  if (/^-?\d+(\.\d+)?$/.test(raw)) raw = Number(raw);
  else if (raw === 'true' || raw === 'false') raw = raw === 'true';
  try {
    const r = await client.setParam(node, pname, raw);
    $('param-out').textContent = r.successful ? `已设置 ${pname} = ${fmt(raw)}` : `被拒: ${r.reason}`;
  } catch (e) {
    $('param-out').textContent = '失败: ' + (e.message || e);
  }
});

// ── TF ──────────────────────────────────────────────────────────────────────

let tfHandle = null;
$('btn-tf').addEventListener('click', () => {
  if (!client.isConnected) return alert('先连接');
  if (tfHandle) return;
  tfHandle = client.subscribe('/tf', 'tf2_msgs/msg/TFMessage', (m) => {
    const list = $('tf-list');
    list.innerHTML = '';
    for (const t of m.data.transforms || []) {
      const li = document.createElement('li');
      li.innerHTML = `<span>${t.header.frame_id} → ${t.child_frame_id}</span>` +
        `<span class="t">${new Date(Math.floor(m.timestampMs)).toLocaleTimeString()}</span>`;
      list.appendChild(li);
    }
  }, { dispatchMode: 'latest' });
  $('btn-tf').disabled = true;
  $('btn-tf-stop').disabled = false;
});

$('btn-tf-stop').addEventListener('click', () => {
  if (tfHandle) { try { tfHandle.cancel(); } catch (_) {} tfHandle = null; }
  $('btn-tf').disabled = false;
  $('btn-tf-stop').disabled = true;
});

// ── 全链路自检 ──────────────────────────────────────────────────────────────
// 对照 test/test_e2e_comm_matrix.py：同一矩阵的浏览器版。fixture 缺席时相关项标 SKIP。

const SELFTEST_STEPS = [
  {
    name: '连接 + serverInfo 能力',
    run: async () => {
      if (!client.isConnected) await client.connect();
      const caps = client.player?.capabilities || [];
      for (const c of ['clientPublish', 'services', 'connectionGraph']) {
        if (!caps.includes(c)) throw new Error(`缺能力 ${c}`);
      }
      return `caps=${caps.join(',')}`;
    },
  },
  {
    name: '话题订阅 /rwl_demo/chatter',
    fixture: true,
    run: () => new Promise((resolve, reject) => {
      const h = client.subscribe('/rwl_demo/chatter', '', (m) => {
        h.cancel();
        resolve(m.data?.data);
      });
      setTimeout(() => { try { h.cancel(); } catch (_) {} reject(new Error('8s 无消息')); }, 8000);
    }),
  },
  {
    name: '客户端发布 roundtrip echo→echo_reply',
    fixture: true,
    run: () => new Promise((resolve, reject) => {
      const token = 'selftest-' + Date.now();
      const h = client.subscribe('/rwl_demo/echo_reply', '', (m) => {
        if (m.data?.data === 'echo:' + token) {
          h.cancel();
          resolve(m.data.data);
        }
      });
      let tries = 0;
      const send = () => {
        if (tries++ > 20) { h.cancel(); return reject(new Error('无回音')); }
        client.publish('/rwl_demo/echo', 'std_msgs/msg/String', { data: token });
        setTimeout(send, 450);
      };
      setTimeout(send, 300);
    }),
  },
  {
    name: '服务调用 add_two_ints(20,22)',
    fixture: true,
    run: async () => {
      const r = await client.callService('/rwl_demo/add_two_ints', { a: 20, b: 22 }, { timeoutMs: 10000 });
      // CDR 解码的整型是 BigInt（@foxglove MessageReader 语义），比较前先 Number()
      if (Number(r.sum) !== 42) throw new Error('sum=' + r.sum);
      return 'sum=42';
    },
  },
  {
    name: '参数 get/set/get（comm_fixture.rwl.answer）',
    fixture: true,
    run: async () => {
      const before = await client.getParams('/comm_fixture', ['rwl.answer']);
      const next = (Number(before['rwl.answer']) || 0) + 1;
      const set = await client.setParam('/comm_fixture', 'rwl.answer', next);
      if (!set.successful) throw new Error('set 被拒');
      const after = await client.getParams('/comm_fixture', ['rwl.answer']);
      if (after['rwl.answer'] !== next) throw new Error(`回读 ${after['rwl.answer']} ≠ ${next}`);
      await client.setParam('/comm_fixture', 'rwl.answer', before['rwl.answer']);
      return `→ ${next} → 还原`;
    },
  },
  {
    name: 'Action fibonacci(8)：feedback + result',
    fixture: true,
    run: () => new Promise((resolve, reject) => {
      const ac = createActionClient('/rwl_demo/fibonacci');
      let fbCount = 0;
      const timer = setTimeout(() => { try { ac.dispose(); } catch (_) {} reject(new Error('25s 超时')); }, 25000);
      ac.sendGoal({ order: 8 }, {
        onFeedback: () => { fbCount++; },
      }).then(({ status, statusText, result }) => {
        clearTimeout(timer);
        ac.dispose();
        if (status !== 4) throw new Error(`终态 ${statusText}`);
        if (!fbCount) throw new Error('未收到 feedback');
        resolve(`${fbCount} 条 feedback · 序列 ${result.sequence?.length} 项`);
      }).catch((e) => { clearTimeout(timer); reject(e); });
    }),
  },
  {
    name: 'Action 取消（order=60 中途 cancel）',
    fixture: true,
    run: () => new Promise((resolve, reject) => {
      const ac = createActionClient('/rwl_demo/fibonacci');
      let fbCount = 0;
      const timer = setTimeout(() => { try { ac.dispose(); } catch (_) {} reject(new Error('30s 超时')); }, 30000);
      ac.sendGoal({ order: 60 }, {
        onFeedback: (fb, goalId) => {
          fbCount++;
          if (fbCount === 2 && goalId) {
            // 按 goalId 精确取消（与 E2E 同语义；zero-uuid 批量取消在 rclpy
            // 服务端有死锁风险——见 doc/architecture.md 已知问题）
            ac.cancelGoal(goalId).catch(() => {});
          }
        },
      }).then(({ status, statusText }) => {
        clearTimeout(timer);
        ac.dispose();
        if (status !== 5) throw new Error(`终态 ${statusText}（期望 CANCELED）`);
        resolve('CANCELED ✓');
      }).catch((e) => { clearTimeout(timer); reject(e); });
    }),
  },
  {
    name: 'TF 流（rwl_demo_base → rwl_demo_marker）',
    fixture: true,
    run: () => new Promise((resolve, reject) => {
      const h = client.subscribe('/tf', 'tf2_msgs/msg/TFMessage', (m) => {
        const hit = (m.data.transforms || []).find(
          (t) => t.child_frame_id === 'rwl_demo_marker');
        if (hit) {
          h.cancel();
          resolve(`${hit.header.frame_id} → ${hit.child_frame_id}`);
        }
      }, { dispatchMode: 'latest' });
      setTimeout(() => { try { h.cancel(); } catch (_) {} reject(new Error('8s 无 /tf')); }, 8000);
    }),
  },
  {
    name: '连接图谱含 /rwl_demo/*',
    fixture: true,
    run: async () => {
      let snap = client.getGraph();
      for (let i = 0; i < 10 && !snap.services.includes('/rwl_demo/add_two_ints'); i++) {
        await new Promise((r) => setTimeout(r, 500));
        snap = client.getGraph();
      }
      if (!snap.topics.includes('/rwl_demo/chatter')) throw new Error('chatter 不在图谱');
      if (!snap.services.includes('/rwl_demo/add_two_ints')) throw new Error('服务不在图谱');
      return `${snap.topics.length} 话题 · ${snap.services.length} 服务 · ${snap.nodes.length} 节点`;
    },
  },
];

async function fixtureOnline() {
  const deadline = Date.now() + 5000;
  while (Date.now() < deadline) {
    if (client.player?.channels?.get('/rwl_demo/chatter')) return true;
    await new Promise((r) => setTimeout(r, 400));
  }
  return false;
}

async function runSelfTest() {
  const btn = $('btn-selftest');
  btn.disabled = true;
  const report = $('selftest-report');
  report.innerHTML = '';
  const results = [];
  const hasFixture = await fixtureOnline();
  window.__rwl_selftest = { startedAt: new Date().toISOString(), results, fixture: hasFixture };

  const addRow = (name) => {
    const row = document.createElement('div');
    row.className = 'st-row';
    row.innerHTML = `<span class="mark">…</span><span>${name}</span><span class="ms"></span>`;
    report.appendChild(row);
    return row;
  };

  for (const step of SELFTEST_STEPS) {
    const row = addRow(step.name);
    const t0 = performance.now();
    const entry = { name: step.name, status: 'pass', detail: '' };
    results.push(entry);
    try {
      if (step.fixture && !hasFixture) {
        entry.status = 'skip';
        entry.detail = 'rwl_comm_fixture 未运行';
        row.className = 'st-row skip';
        row.querySelector('.mark').textContent = '⚠';
        row.querySelector('.ms').textContent = 'SKIP';
        row.insertAdjacentHTML('beforeend',
          '<span class="why">先 ros2 run ros_web_link rwl_comm_fixture</span>');
        continue;
      }
      const detail = await step.run();
      entry.detail = String(detail);
      row.className = 'st-row pass';
      row.querySelector('.mark').textContent = '✓';
      row.insertAdjacentHTML('beforeend', `<span class="why">${entry.detail}</span>`);
    } catch (e) {
      entry.status = 'fail';
      entry.detail = e.message || String(e);
      row.className = 'st-row fail';
      row.querySelector('.mark').textContent = '✗';
      row.insertAdjacentHTML('beforeend', `<span class="why">${entry.detail}</span>`);
    } finally {
      row.querySelector('.ms').textContent =
        Math.round(performance.now() - t0) + ' ms';
      entry.ms = Math.round(performance.now() - t0);
    }
  }

  const pass = results.filter((r) => r.status === 'pass').length;
  const fail = results.filter((r) => r.status === 'fail').length;
  const skip = results.filter((r) => r.status === 'skip').length;
  window.__rwl_selftest.finishedAt = new Date().toISOString();
  window.__rwl_selftest.summary = { pass, fail, skip };
  report.insertAdjacentHTML('afterbegin',
    `<div class="kv" style="margin-bottom:6px">结果：${pass} 通过 · ${fail} 失败 · ${skip} 跳过` +
    `${fail === 0 ? '' : '（见 ✗ 行详情）'}</div>`);
  btn.disabled = false;
}

$('btn-selftest').addEventListener('click', runSelfTest);
window.__rwlSelfTestRun = runSelfTest; // 浏览器自动化入口

// ── 传输监控 Transport Monitor ──────────────────────────────────────────────
// 数据源：player.stats 聚合差分（消息/字节/服务/重连/解码失败）、heartbeat RTT、
// 活跃订阅级差分（player._topicSubs.stats，本包私有结构，诊断台特权读取）。
// 快照导出 window.__rwl_monitor() 供自动化。

const MON_WINDOW = 60;          // 60 × 1s 采样
const mon = {
  hz: [], kb: [],               // 滚动序列
  last: null,                   // 上秒聚合快照
  topicLast: new Map(),         // topic → {messages, bytes}
  topicLastAt: new Map(),       // topic → 最近一帧时刻（subscribe 包装记录）
  inter: [],                    // 交互日志 [{t, kind, name, ok, ms, note}]
  events: [],                   // 事件流 [{t, kind, text}]
  publishes: 0,
  connectedAt: null,
};

function monEvent(kind, text) {
  mon.events.unshift({ t: new Date().toISOString().slice(11, 23), kind, text });
  if (mon.events.length > 80) mon.events.length = 80;
}
function monInter(kind, name, ok, ms, note = '') {
  mon.inter.unshift({ t: new Date().toISOString().slice(11, 23), kind, name, ok, ms, note });
  if (mon.inter.length > 80) mon.inter.length = 80;
}

// 连接/状态事件
client.onStateChange((s) => {
  monEvent(s === 'connected' ? 'ok' : 'warn', `连接状态 → ${s}`);
  if (s === 'connected') mon.connectedAt = Date.now();
});
client.onStatus((msg) => {
  monEvent('warn', `status: ${(msg.message || '').slice(0, 90)}`);
});

// 交互面包装：服务调用（含 action 的 send_goal/get_result/cancel_goal）与发布
const _origCallService = client.callService.bind(client);
client.callService = async (name, req, opts = {}) => {
  const t0 = performance.now();
  try {
    const r = await _origCallService(name, req, opts);
    monInter('svc', name, true, Math.round(performance.now() - t0));
    return r;
  } catch (e) {
    monInter('svc', name, false, Math.round(performance.now() - t0),
      String(e.message || e).slice(0, 60));
    throw e;
  }
};
const _origPublish = client.publish.bind(client);
client.publish = (topic, type, message) => {
  const ok = _origPublish(topic, type, message);
  mon.publishes += ok ? 1 : 0;
  monInter('pub', topic, !!ok, 0, ok ? '' : '图上无该话题/schema');
  return ok;
};

// 订阅包装：记录每话题最近一帧时刻（监控表“最近一帧”列）
const _origSubscribe = client.subscribe.bind(client);
client.subscribe = (topic, type, cb, opts = {}) =>
  _origSubscribe(topic, type, (m) => {
    mon.topicLastAt.set(topic, Date.now());
    cb(m);
  }, opts);

// 话题卡的订阅入口已由 topics.js 承载（checkbox/可视化）；此处无操作。

function fmtBytes(n) {
  if (n >= 1024 * 1024) return (n / 1024 / 1024).toFixed(1) + ' MB';
  if (n >= 1024) return (n / 1024).toFixed(1) + ' KB';
  return n + ' B';
}
function drawSpark(canvas, series, color) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 300;
  const h = canvas.clientHeight || 90;
  if (canvas.width !== w * dpr) { canvas.width = w * dpr; canvas.height = h * dpr; }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = '#262b36';
  ctx.beginPath(); ctx.moveTo(0, h - 0.5); ctx.lineTo(w, h - 0.5); ctx.stroke();
  const data = series.slice(-MON_WINDOW);
  if (!data.length) return;
  const max = Math.max(...data, 1e-9);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  data.forEach((v, i) => {
    const x = (i / Math.max(data.length - 1, 1)) * w;
    const y = h - 4 - (v / max) * (h - 10);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

setInterval(() => {
  const st = client.stats;
  const hb = client.player?.heartbeat || {};
  // 聚合差分
  const prev = mon.last;
  mon.last = {
    m: st.messagesReceived, b: st.bytesReceived,
    svc: st.serviceCalls + st.serviceFailures,
    reconnects: st.reconnects, decodeErrors: st.decodeErrors,
  };
  if (prev) {
    // 聚合速率 EMA（α=0.3，ChartPanel 模式）
    mon.hzEma = (mon.hzEma ?? 0) * 0.7 + (mon.last.m - prev.m) * 0.3;
    mon.kbEma = (mon.kbEma ?? 0) * 0.7 + ((mon.last.b - prev.b) / 1024) * 0.3;
    mon.hz.push(mon.last.m - prev.m);
    mon.kb.push((mon.last.b - prev.b) / 1024);
    if (mon.hz.length > MON_WINDOW) mon.hz.shift();
    if (mon.kb.length > MON_WINDOW) mon.kb.shift();
    if (mon.last.reconnects > prev.reconnects) monEvent('bad', `重连 #${mon.last.reconnects}`);
    if (mon.last.decodeErrors > prev.decodeErrors) {
      monEvent('bad', `CDR 解码失败 +${mon.last.decodeErrors - prev.decodeErrors}`);
    }
  }
  // 诊断卡（Rebot-Arm diag-grid 模式；状态/重连/解码失败已拆分）
  const rttEl = $('mon-rtt');
  rttEl.textContent = hb.rttMs != null ? hb.rttMs + ' ms' : (client.isConnected ? '…' : '—');
  rttEl.className = 'v ' + (!client.isConnected ? '' : (hb.alive === false ? 'bad' : 'ok'));
  $('mon-hz').textContent = mon.hzEma != null ? mon.hzEma.toFixed(0) : '—';
  $('mon-kbps').textContent = mon.kbEma != null ? mon.kbEma.toFixed(1) : '—';
  $('mon-msgs').textContent = st.messagesReceived;
  $('mon-bytes').textContent = fmtBytes(st.bytesReceived);
  $('mon-svc').textContent = `${st.serviceCalls}/${st.serviceFailures || 0}`;
  $('mon-reconn').textContent = st.reconnects;
  $('mon-reconn').className = 'v ' + (st.reconnects ? 'warn' : '');
  $('mon-decode').textContent = st.decodeErrors;
  $('mon-decode').className = 'v ' + (st.decodeErrors ? 'bad' : '');
  $('mon-hz-now').textContent = (mon.hz[mon.hz.length - 1] ?? 0).toFixed(0) + ' msg/s';
  $('mon-kb-now').textContent = (mon.kb[mon.kb.length - 1] ?? 0).toFixed(1) + ' KB/s';
  drawSpark($('mon-chart-hz'), mon.hz, '#33d6b0');
  drawSpark($('mon-chart-kb'), mon.kb, '#5fa8ff');

  // 话题级速率差分（供 __rwl_monitor 快照；表格渲染移交话题卡 topics.js）
  const subs = client.player?._topicSubs;
  const now = Date.now();
  if (subs) {
    for (const [topic, s] of subs) {
      const prevT = mon.topicLast.get(topic) || { messages: 0, bytes: 0 };
      const dHz = s.stats.messages - prevT.messages;
      const dKb = (s.stats.bytes - prevT.bytes) / 1024;
      mon.topicLast.set(topic, { messages: s.stats.messages, bytes: s.stats.bytes });
      const lastAt = mon.topicLastAt.get(topic);
      (mon.topicRate ??= new Map()).set(topic, { hz: dHz, kbps: dKb, lastAt: lastAt || null });
    }
  }

  // 日志/事件渲染
  $('mon-inter').innerHTML = mon.inter.slice(0, 30).map((e) =>
    `<div><span class="t">${e.t}</span>` +
    `<span class="${e.ok ? 'ok' : 'bad'}">${e.ok ? '✓' : '✗'}</span> ` +
    `${e.kind} ${e.name} ${e.ms ? e.ms + 'ms' : ''} ${e.note || ''}</div>`).join('') ||
    '<div style="color:var(--muted)">调用服务/发布/action 后此处出现记录</div>';
  $('mon-events').innerHTML = mon.events.slice(0, 30).map((e) =>
    `<div><span class="t">${e.t}</span><span class="${e.kind}">●</span> ${e.text}</div>`).join('') ||
    '<div style="color:var(--muted)">连接状态与告警事件流</div>';
}, 1000);

// 自动化快照
window.__rwl_monitor = () => {
  const st = client.stats;
  const topics = [];
  if (client.player?._topicSubs) {
    for (const [topic, s] of client.player._topicSubs) {
      const r = (mon.topicRate || new Map()).get(topic) || {};
      topics.push({
        topic, messages: s.stats.messages, bytes: s.stats.bytes,
        hz: r.hz ?? null, kbps: r.kbps ?? null, lastAt: r.lastAt ?? null,
      });
    }
  }
  return {
    state: client.state, connectedAt: mon.connectedAt,
    heartbeat: client.player?.heartbeat || null,
    aggregate: st, publishes: mon.publishes,
    hzSeries: [...mon.hz], kbSeries: [...mon.kb],
    interactions: mon.inter.slice(0, 20), events: mon.events.slice(0, 20),
    topics,
  };
};

// ── 主题切换（默认白色，localStorage 记忆） ─────────────────────────────────
(function themeInit() {
  const saved = localStorage.getItem('rwl-theme') || 'light';
  document.body.dataset.theme = saved;
  const btn = document.getElementById('theme-toggle');
  const sync = () => {
    btn.textContent = document.body.dataset.theme === 'dark' ? '☀️ 浅色' : '🌙 深色';
  };
  btn.addEventListener('click', () => {
    const next = document.body.dataset.theme === 'dark' ? 'light' : 'dark';
    document.body.dataset.theme = next;
    localStorage.setItem('rwl-theme', next);
    window.dispatchEvent(new CustomEvent('rwl-theme', { detail: next }));
    sync();
  });
  sync();
})();

// ── 初始化 ──────────────────────────────────────────────────────────────────

$('ws-url').value = defaultWsUrl();
refreshConnInfo();
const vizPanel = initViz();     // RViz 风格可视化（Displays/工具栏/3D/2D/关节写回）
initPlot();                     // rqt_plot 风格字段绘图
const topicsCard = initTopics(vizPanel);   // 话题卡（订阅树 + 一键可视化）
initPanels(vizPanel, topicsCard);          // 示教/轨迹/拓扑图/分割条
initMonPanels();                // 诊断（runtime_monitor）+ ROS 日志（rqt_console）
window.__rwl_viz = vizPanel;    // 浏览器自动化入口（addOrEnable/快照；同 __rwl_topics 模式）
// ── 顶部导航栏（RViz 菜单栏语义：按工作区切换成员卡片，主网格扁平重排） ──────
(function navInit() {
  const bar = document.getElementById('top-nav');
  const main = document.querySelector('main.app-grid');
  if (!bar || !main) return;
  const MEMBERS = {
    viz: ['sec-viz', 'sec-displays', 'sec-topics', 'sec-teleop'],
    data: ['sec-2d', 'sec-plot'],
    monitor: ['sec-mon', 'sec-diag', 'sec-roslog'],
    tools: ['sec-traj', 'sec-params', 'sec-ops', 'sec-selftest'],
    conn: ['sec-conn', 'sec-graph', 'sec-topo'],
  };
  const apply = (name) => {
    main.dataset.nav = name;
    main.classList.toggle('col-flat', name !== 'viz');
    for (const ids of Object.values(MEMBERS)) {
      for (const id of ids) {
        document.getElementById(id)?.classList.remove('in-view');
      }
    }
    for (const id of MEMBERS[name] || []) {
      document.getElementById(id)?.classList.add('in-view');
    }
    localStorage.setItem('rwl-nav', name);
  };
  bar.querySelectorAll('button[data-nav]').forEach((b) => {
    b.addEventListener('click', () => apply(b.dataset.nav));
  });
  apply(localStorage.getItem('rwl-nav') || 'viz');
})();

// ── RViz2/Foxglove 升级轮：Displays 添加 / 布局快照 / Teleop / 参数面板 ─────
(function extendedPanels() {
  // ＋添加显示项（RViz Add Display）
  const addSel = document.getElementById('disp-add-type');
  if (addSel) {
    addSel.innerHTML = '<option value="">＋ 添加显示项…</option>'
      + Object.entries(DISPLAY_TYPES)
        .filter(([, s]) => !s.singleton)
        .map(([k, s]) => `<option value="${k}">${s.label}</option>`).join('');
    addSel.addEventListener('change', () => {
      if (addSel.value) { vizPanel.addInstance(addSel.value); addSel.value = ''; }
    });
  }

  // 布局命名快照（保存/恢复/删除/导入导出）
  const layoutSel = document.getElementById('layout-select');
  const refreshLayouts = () => {
    if (!layoutSel) return;
    const all = vizPanel.listLayouts();
    layoutSel.innerHTML = '<option value="">布局：默认</option>'
      + Object.keys(all).map((n) => `<option value="${n}">${n}</option>`).join('');
  };
  refreshLayouts();
  layoutSel?.addEventListener('change', () => {
    const name = layoutSel.value;
    if (!name) return;
    const snap = vizPanel.listLayouts()[name];
    if (snap && vizPanel.applyLayout(snap)) {
      const h = document.getElementById('hud-tool');
      if (h) h.textContent = `布局「${name}」已恢复`;
    }
  });
  document.getElementById('layout-save')?.addEventListener('click', () => {
    const fallback = '布局 ' + new Date().toLocaleString();
    const name = window.prompt('布局名称', layoutSel?.value || fallback) || fallback;
    vizPanel.saveLayout(name, vizPanel.snapshotLayout());
    refreshLayouts();
    if (layoutSel) layoutSel.value = name;
  });
  document.getElementById('layout-del')?.addEventListener('click', () => {
    const name = layoutSel?.value;
    if (!name) return;
    vizPanel.deleteLayout(name);
    refreshLayouts();
  });
  document.getElementById('layout-export')?.addEventListener('click', async () => {
    const json = JSON.stringify(vizPanel.snapshotLayout(), null, 1);
    try {
      await navigator.clipboard.writeText(json);
      const h = document.getElementById('hud-tool');
      if (h) h.textContent = '布局 JSON 已复制到剪贴板';
    } catch (_) {
      window.prompt('复制布局 JSON：', json);
    }
  });
  document.getElementById('layout-import')?.addEventListener('click', () => {
    const txt = window.prompt('粘贴布局 JSON：');
    if (!txt) return;
    try {
      if (vizPanel.applyLayout(JSON.parse(txt))) refreshLayouts();
    } catch (e) {
      alert('导入失败: ' + (e.message || e));
    }
  });

  // Teleop（官方 TB3 /cmd_vel；仅限仿真）
  initTeleop(() => window.__rwl_client);

  // 参数面板（rqt_reconfigure 简化版：节点 → 全量参数 → 类型内联编辑 → 写回）
  const esc = (s) => String(s).replace(/[&<>"]/g,
    (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[ch]));
  document.getElementById('btn-param-load')?.addEventListener('click', async () => {
    const node = document.getElementById('param-node').value.trim() || '/';
    const box = document.getElementById('param-list');
    box.textContent = '读取中…';
    try {
      const params = await client.getParams(node);
      const names = Object.keys(params);
      if (!names.length) {
        box.textContent = '（节点无参数或不可达）';
        return;
      }
      box.innerHTML = '';
      for (const name of names) {
        const v = params[name];
        const row = document.createElement('div');
        row.style.cssText = 'display:flex;gap:6px;align-items:center;';
        const nameCell = `<span class="mini" style="width:170px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap" title="${esc(name)}">${esc(name)}</span>`;
        const type = typeof v;
        if (type === 'boolean') {
          row.innerHTML = `${nameCell}
            <input type="checkbox" data-pn="${esc(name)}" ${v ? 'checked' : ''}>
            <span class="mini dst ok">bool</span>`;
          row.querySelector('input').addEventListener('change', async (e) => {
            const r = await client.setParam(node, name, e.target.checked);
            const d = row.querySelector('.dst');
            d.textContent = r.successful ? '✓ 已写' : '✗ ' + (r.reason || '失败');
            d.className = 'mini dst ' + (r.successful ? 'ok' : 'warn');
          });
        } else if (type === 'number' || type === 'bigint') {
          row.innerHTML = `${nameCell}
            <input type="number" step="any" value="${v}" data-pn="${esc(name)}" style="width:130px;">
            <span class="mini dst ok">num</span>`;
          row.querySelector('input').addEventListener('change', async (e) => {
            const r = await client.setParam(node, name, parseFloat(e.target.value));
            const d = row.querySelector('.dst');
            d.textContent = r.successful ? '✓ 已写' : '✗ ' + (r.reason || '失败');
            d.className = 'mini dst ' + (r.successful ? 'ok' : 'warn');
          });
        } else if (type === 'string') {
          row.innerHTML = `${nameCell}
            <input type="text" value="${esc(v)}" data-pn="${esc(name)}" style="flex:1;min-width:0;">
            <span class="mini dst ok">str</span>`;
          row.querySelector('input').addEventListener('change', async (e) => {
            const r = await client.setParam(node, name, e.target.value);
            const d = row.querySelector('.dst');
            d.textContent = r.successful ? '✓ 已写' : '✗ ' + (r.reason || '失败');
            d.className = 'mini dst ' + (r.successful ? 'ok' : 'warn');
          });
        } else {
          row.innerHTML = `${nameCell}
            <span class="mini" style="flex:1;overflow:hidden;text-overflow:ellipsis">${esc(JSON.stringify(v)).slice(0, 70)}</span>
            <span class="mini dst">${type}</span>`;
        }
        box.appendChild(row);
      }
    } catch (e) {
      box.textContent = '读取失败: ' + (e.message || e);
    }
  });
  // 节点 datalist（连接图节点；周期刷新）
  setInterval(() => {
    const dl = document.getElementById('rwl-nodes');
    let graph = null;
    try { graph = client.getGraph(); } catch (_) { return; }
    if (!dl || !graph?.nodes) return;
    dl.innerHTML = [...graph.nodes].map((n) => `<option value="${n}">`).join('');
  }, 4000);
})();

// 页面加载即自动连接（诊断台默认行为）
connect();
