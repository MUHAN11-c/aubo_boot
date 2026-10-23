'use strict';

const $ = (id) => document.getElementById(id);
const safe = (value) => String(value ?? '').replace(/[&<>"']/g, (ch) => ({
  '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
}[ch]));

let state = {};
let paramData = null;
let stopRounds = false;
let roundsBusy = false;
let singleArmed = false;
let singleKey = '';
let lastStateAt = 0;
let pollPending = null;
let selectionEpoch = null;
let sessionGeneration = 0;
let unknownRequest = '';
const picked = new Set();
const clickQueue = [];

async function getJSON(path, opts) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), opts?.method === 'POST' ? 195000 : 6000);
  let response;
  let text;
  try {
    response = await fetch(path, {...opts, signal: controller.signal});
    text = await response.text();
  } finally {
    clearTimeout(timer);
  }
  let data = null;
  try { data = text ? JSON.parse(text) : null; } catch (_) { data = null; }
  if (!response.ok) {
    const message = (data && (data.error || data.message)) || response.status;
    const error = new Error(message);
    error.status = response.status;
    error.body = data;
    throw error;
  }
  if (!data || typeof data !== 'object') throw new Error('响应格式无效');
  if (response.status === 202 || response.status === 207
      || (opts?.method === 'POST' && data.accepted === false)) {
    const message = (data.results || []).filter(row => row.accepted !== true)
      .map(row => `${row.name || ''}: ${row.message || '未写入'}`).join('；');
    const error = new Error(message || data.result?.message || data.message || '请求未接受');
    error.status = response.status;
    error.body = data;
    throw error;
  }
  return data;
}

function requireReady() {
  if (unknownRequest) throw new Error(`请求 ${unknownRequest} 结果未知；先核对状态并取消在途动作`);
  if (!lastStateAt || Date.now() - lastStateAt > 5000) throw new Error('状态已陈旧，请等待连接恢复');
  if (state.debug?.enabled !== true) throw new Error('调试入口未开启');
  if (state.task_executor?.state?.recovery_required === true) throw new Error('等待人工恢复确认');
}

function requireContext(epoch, ids = []) {
  requireReady();
  const targets = state.perception?.targets;
  if (!Number.isInteger(epoch) || epoch <= 0 || targets?.scene_epoch !== epoch
      || targets.target_set_locked !== true) throw new Error('场景已变化或未锁定，请重新选择');
  const current = new Set(observations().map(item => item.target_id));
  if (ids.some(id => !current.has(id))) throw new Error('目标已不属于当前场景，请重新选择');
}

function requireSession(generation) {
  if (generation !== sessionGeneration || stopRounds) throw new Error('本次会话已停止');
}

function findParam(node, name) {
  const group = ((paramData && paramData.groups) || []).find((item) => item.node === node);
  return ((group && group.params) || []).find((item) => item.name === name);
}

function tieEnables(changed) {
  if ((changed === 'ex' || changed === 'gr' || changed === 'tl') && $(changed).checked) {
    $('motion').checked = true;
  }
  if (changed === 'tl' && $('tl').checked) {
    $('gr').checked = true;
    $('ex').checked = true;
  }
  if (changed === 'gr' && $('gr').checked) $('ex').checked = true;
  if (changed === 'gr' && !$('gr').checked) $('tl').checked = false;
  if (changed === 'ex' && !$('ex').checked) {
    $('gr').checked = false;
    $('tl').checked = false;
  }
  if (changed === 'motion' && !$('motion').checked) {
    $('ex').checked = false;
    $('gr').checked = false;
    $('tl').checked = false;
  }
}

async function loadRound() {
  paramData = await getJSON('/api/params');
  const motion = findParam('peach_observability', 'debug.motion_enabled');
  const pregrasp = findParam('peach_supervisor', 'execute_pregrasp_only');
  if (motion) $('motion').checked = motion.value === true;
  if (pregrasp) $('full').checked = pregrasp.value === false;
  const skip = findParam('peach_supervisor', 'skip_reconstruction');
  if (skip) $('skip').checked = skip.value === true;
  const task = (state.task_executor && state.task_executor.state) || {};
  $('ex').checked = task.execution_enabled === true;
  $('gr').checked = task.grasp_enabled === true;
  $('tl').checked = task.tool_enabled === true;
  $('round-note').textContent = '已读取当前开关。';
}

async function applyRound() {
  tieEnables('ex');
  const motion = $('motion').checked;
  const full = $('full').checked;
  const execution = $('ex').checked;
  const grasp = $('gr').checked;
  const tool = $('tl').checked;
  const summary = [
    `运动放行：${motion ? '开' : '关'}`,
    `执行档：${full ? '完整抓取（套入与剪切）' : '停在预抓取'}`,
    `执行 ${execution ? '开' : '关'} · 抓取 ${grasp ? '开' : '关'} · 工具 ${tool ? '开' : '关'}`,
    '打开执行之后，启动批次会动臂。工具打开会走到剪切。',
  ].join('\n');
  if (!window.confirm(summary)) return;
  $('round-note').textContent = '写入中…';
  try {
    if (roundsBusy || singleArmed) throw new Error('先停止当前单轮或多轮会话再改参数');
    await pushSettings();
    $('round-note').textContent = '本轮设置已写入并读回核对';
    await loadParams();
  } catch (error) {
    $('round-note').textContent = `写入失败：${error.message}`;
  }
}

function renderParams() {
  const groups = (paramData && paramData.groups) || [];
  $('param-groups').innerHTML = groups.map((group, index) => {
    const rows = (group.params || []).map((param) => {
      const id = `p-${index}-${param.name.replace(/\./g, '-')}`;
      const badge = (param.motion ? '<span class="badge motion">运动</span>' : '')
        + (param.static ? '<span class="badge static">重启</span>' : '');
      let input;
      if (param.type === 'bool') {
        input = `<select id="${id}"><option value="true"${param.value === true ? ' selected' : ''}>true</option><option value="false"${param.value === false ? ' selected' : ''}>false</option></select>`;
      } else if (param.type === 'string' && param.choices) {
        input = `<select id="${id}">${param.choices.map((choice) => `<option value="${safe(choice)}"${param.value === choice ? ' selected' : ''}>${safe(choice)}</option>`).join('')}</select>`;
      } else {
        input = `<input id="${id}" type="number" step="any" value="${param.value ?? ''}">`;
      }
      const note = !param.node_running
        ? '<div class="note">节点未运行</div>'
        : (!param.present ? `<div class="note">${safe(group.error || '参数未声明')}</div>` : '');
      return `<div class="param-row${param.node_running ? '' : ' unavailable'}" data-node="${safe(group.node)}" data-name="${safe(param.name)}" data-type="${safe(param.type)}" data-orig="${safe(param.value)}"><div><div class="name">${safe(param.name)}</div><div class="label">${safe(param.label)}${badge}</div></div><div>${input}${note}</div><div class="muted">当前 <code>${safe(param.value)}</code></div></div>`;
    }).join('');
    return `<section class="param-group"><div class="param-head"><b>${safe(group.label)}</b><span class="muted">${safe(group.node)}</span><span class="muted">${group.running ? '运行中' : '未运行'}</span></div>${rows}</section>`;
  }).join('');
  document.querySelectorAll('.param-row input, .param-row select').forEach((el) => {
    el.addEventListener('change', () => el.closest('.param-row').classList.add('changed'));
  });
}

async function loadParams() {
  try {
    paramData = await getJSON('/api/params');
    renderParams();
    $('params-note').textContent = paramData.debug_enabled ? '' : '调试入口已关闭：参数只读';
  } catch (error) {
    $('param-groups').innerHTML = `<p class="empty">参数读取失败：${safe(error.message)}</p>`;
  }
}

function collectChanges() {
  const changes = [];
  document.querySelectorAll('.param-row').forEach((row) => {
    const el = row.querySelector('input, select');
    if (!el) return;
    const type = row.dataset.type;
    const value = type === 'bool' ? el.value === 'true'
      : (type === 'string' ? el.value : Number(el.value));
    const same = String(value) === String(row.dataset.orig)
      || (type === 'double' && Number(value) === Number(row.dataset.orig));
    if (!same) changes.push({node: row.dataset.node, name: row.dataset.name, value});
  });
  return changes;
}

async function applyParams() {
  const changes = collectChanges();
  if (!changes.length) {
    $('params-note').textContent = '没有修改';
    return;
  }
  $('params-note').textContent = '写入中…';
  try {
    if (roundsBusy || singleArmed) throw new Error('先停止当前会话再改参数');
    const data = await writeParams(changes);
    const results = data.results || [];
    const rejected = results.filter((row) => !row.accepted);
    $('params-note').textContent = rejected.length
      ? `写入 ${data.accepted_count ?? 0}/${results.length}，${rejected.map((row) => row.message).join('；')}`
      : `写入 ${results.length} 项`;
    await loadParams();
  } catch (error) {
    $('params-note').textContent = `写入失败：${error.message}`;
  }
}

function observations() {
  const list = (state.perception && state.perception.targets
    && state.perception.targets.observations) || [];
  return list.filter((item) => item && item.target_id);
}

function byPriority(list) {
  return [...list].sort((a, b) => (Number(a.priority) || 999) - (Number(b.priority) || 999));
}

let targetKey = '';
function renderTargets() {
  const epoch = state.perception?.targets?.scene_epoch ?? null;
  if (selectionEpoch !== epoch) {
    picked.clear();
    clickQueue.length = 0;
    targetKey = '';
    selectionEpoch = epoch;
  }
  const items = byPriority(observations());
  const key = items.map((item) => `${item.target_id}:${item.priority}`).join('|');
  if (key === targetKey && $('target-picks').querySelector('button')) return;
  targetKey = key;
  const host = $('target-picks');
  if (!items.length) {
    host.innerHTML = '<p class="empty">等待本场目标。开一轮后会重新锁定。</p>';
    return;
  }
  host.innerHTML = items.map((item) => {
    const id = item.target_id;
    const order = Number(item.priority) || 0;
    const checked = picked.has(id) ? ' checked' : '';
    const label = order ? `顺序 ${order}` : id;
    return `<span class="target-pick"><label><input type="checkbox" data-keep="${safe(id)}"${checked}> 点选</label><button type="button" data-target="${safe(id)}">${safe(label)}<small>${safe(id)}</small></button></span>`;
  }).join('');
  host.querySelectorAll('input').forEach((box) => {
    box.addEventListener('change', () => {
      if (box.checked) picked.add(box.dataset.keep);
      else picked.delete(box.dataset.keep);
    });
  });
  host.querySelectorAll('button').forEach((button) => {
    button.addEventListener('click', () => enqueueGrasp([button.dataset.target]));
  });
}

function logRound(text) {
  const item = document.createElement('li');
  item.textContent = text;
  $('round-log').prepend(item);
}

async function postDebug(action, payload) {
  let body;
  try {
    body = await getJSON(`/api/debug/${encodeURIComponent(action)}`, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(payload || {}),
    });
  } catch (error) {
    if (action === 'run_harvest_action' && (!error.status || error.status === 202 || error.status >= 500)) {
      unknownRequest = payload?.request_id || '在途批次';
    }
    throw error;
  }
  if (body.accepted !== true || body.result?.accepted === false || body.result?.success === false) {
    throw new Error(body.result?.message || body.result?.reason || body.message || '操作失败');
  }
  if (['set_enables_service', 'control_service', 'begin_scene_service'].includes(action)
      && body.result?.accepted !== true) throw new Error('服务未确认接受本次请求');
  if (action.endsWith('_action') && Number(body.result?.action_status) !== 4) throw new Error('动作未成功结束');
  return body;
}

async function writeParams(changes) {
  const result = await getJSON('/api/params', {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({changes}),
  });
  if (result.accepted !== true || result.results?.length !== changes.length
      || result.results.some(row => row.accepted !== true)) throw new Error('参数未全部写入');
  paramData = await getJSON('/api/params');
  for (const change of changes) {
    const actual = findParam(change.node, change.name);
    if (!actual?.present || actual.value !== change.value) throw new Error(`参数读回不一致：${change.name}`);
  }
  return result;
}

async function pushSettings() {
  requireReady();
  if (state.task_executor?.state?.action_active === true || state.manipulation?.status?.running === true) {
    throw new Error('批次或臂周期正在运行，不能改本轮设置');
  }
  const generation = sessionGeneration;
  const enables = {execution: $('ex').checked, grasp: $('gr').checked, tool: $('tl').checked};
  const changes = [
    {node: 'peach_observability', name: 'debug.motion_enabled', value: $('motion').checked},
    {node: 'peach_supervisor', name: 'execute_pregrasp_only', value: !$('full').checked},
    {node: 'peach_supervisor', name: 'skip_reconstruction', value: $('skip').checked},
    {node: 'peach_target_reconstruction_node', name: 'capture.auto_mode', value: !$('skip').checked},
    {node: 'peach_arm', name: 'quality.allow_unrefined_geometry', value: $('skip').checked},
  ];
  await writeParams(changes);
  if (generation !== sessionGeneration) throw new Error('设置会话已停止，未应用使能');
  await poll();
  requireReady();
  await postDebug('set_enables_service', {
    ...enables,
    reason: '单步调试',
  });
  const deadline = Date.now() + 5000;
  while (Date.now() < deadline) {
    await poll();
    const task = state.task_executor?.state || {};
    if (Object.entries(enables).every(([name, value]) => task[`${name}_enabled`] === value)) return;
    await new Promise(resolve => setTimeout(resolve, 150));
  }
  throw new Error('使能读回未确认，请核对当前状态');
}

function freshRequest() {
  const now = new Date();
  const pad = (value, width = 2) => String(value).padStart(width, '0');
  const stamp = `${now.getFullYear()}${pad(now.getMonth() + 1)}${pad(now.getDate())}`
    + `_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`
    + `_${pad(now.getMilliseconds(), 3)}`;
  return `grasp_${stamp}`;
}

function requireNoRecovery() {
  const task = (state.task_executor && state.task_executor.state) || {};
  if (task.recovery_required === true) throw new Error('等待人工恢复确认，后续目标已停止');
}

async function finishHarvest(pending, request) {
  let settled = null;
  pending.then(
    (result) => { settled = {ok: true, result}; },
    (error) => { settled = {ok: false, error}; },
  );
  while (settled == null) {
    await poll();
    try { requireNoRecovery(); } catch (error) { unknownRequest = request; throw error; }
    await new Promise((resolve) => setTimeout(resolve, 400));
  }
  if (!settled.ok) throw settled.error;
  return settled.result;
}

async function refreshScene(request, wanted) {
  const opened = await postDebug('begin_scene_service', {
    request_id: request, scene_key: request,
  });
  if (opened.result && opened.result.accepted === false) {
    throw new Error(opened.result.message || '开场景被拒绝');
  }
  const epoch = opened.result && opened.result.scene_epoch;
  if (!Number.isInteger(epoch) || epoch <= 0) throw new Error('开场景响应缺少有效 epoch');
  const deadline = Date.now() + 45000;
  while (Date.now() < deadline) {
    await poll();
    const targets = state.perception && state.perception.targets;
    const sameEpoch = targets?.scene_epoch === epoch;
    const ids = new Set(observations().map((item) => item.target_id));
    const ready = !wanted || wanted.every((id) => ids.has(id));
    if (sameEpoch && targets.target_set_locked === true && ready && lastStateAt) return epoch;
    await new Promise((resolve) => setTimeout(resolve, 500));
  }
  throw new Error('场景锁定超时；请重新核对当前场景');
}

async function setPriority(ordinal) {
  await writeParams([
      {node: 'peach_supervisor', name: 'selection_priority', value: ordinal},
  ]);
}

function requireHarvestSuccess(result) {
  const summary = result?.result;
  if (summary?.success !== true || Number(summary.action_status) !== 4
      || Number(summary.summary_attempted) < 1
      || Number(summary.summary_succeeded) !== Number(summary.summary_attempted)
      || Number(summary.summary_failed) > 0) throw new Error(summary?.message || '批次未全部成功，后续已停止');
}

async function harvestRound(index, mode) {
  const request = freshRequest();
  const generation = sessionGeneration;
  requireReady();
  let ordinal = 0;
  if (mode === 'rotate' || mode === 'first') {
    await refreshScene(request);
    const count = observations().length;
    if (!count) throw new Error('本场没有锁定目标');
    ordinal = mode === 'first' ? 1 : (index % count) + 1;
  } else if (mode !== 'all') {
    throw new Error('目标选择模式无效');
  }
  await setPriority(ordinal);
  requireSession(generation);
  requireReady();
  logRound(`第 ${index + 1} 轮 · ${ordinal ? `顺序 ${ordinal}` : '全部'}`);
  const result = await finishHarvest(postDebug('run_harvest_action', {
    request_id: request, scene_key: request,
    intent: mode === 'all' ? 'PICK_ALL' : 'PICK_SELECTED',
    selection_mode: 'AUTO', target_ids: [],
  }), request);
  requireHarvestSuccess(result);
  logRound(`第 ${index + 1} 轮结束 · 全部成功`);
}

function paintSingle() {
  $('single-panel').classList.toggle('live', singleArmed);
  if (!singleArmed) return;
  const waiting = clickQueue.map((item) => item.ids.join('+'));
  $('single-note').textContent = roundsBusy
    ? `正在抓。${waiting.length ? `接下来 ${waiting.join('，')}` : '抓完继续等下一次点选。'}`
    : (waiting.length ? `接下来 ${waiting.join('，')}` : '单轮保持。点顺序号，或勾选后抓已勾选。');
}

function enqueueGrasp(ids) {
  if (stopRounds && roundsBusy) {
    $('single-note').textContent = '几轮正在跑，单轮先不接';
    return;
  }
  if (!singleArmed) {
    $('single-note').textContent = '先启动单轮点选。启动后点哪颗抓哪颗。';
    return;
  }
  if (!ids.length) {
    $('single-note').textContent = '先勾选要抓的顺序';
    return;
  }
  try { requireReady(); } catch (error) { $('single-note').textContent = error.message; return; }
  clickQueue.push({ids: [...ids], epoch: selectionEpoch});
  paintSingle();
  if (!roundsBusy) drainSingle();
}

async function harvestNamed(ids) {
  const request = freshRequest();
  const epoch = selectionEpoch;
  const generation = sessionGeneration;
  requireReady();
  requireContext(epoch, ids);
  await setPriority(0);
  requireContext(epoch, ids);
  requireSession(generation);
  logRound(`单轮 · ${ids.join(', ')}`);
  const result = await finishHarvest(postDebug('run_harvest_action', {
    request_id: request,
    scene_key: singleKey,
    intent: 'PICK_SELECTED',
    selection_mode: 'MANUAL',
    target_ids: ids,
  }), request);
  requireHarvestSuccess(result);
  const ok = result.result && result.result.success === true;
  logRound(`单轮结束 · ${ids.join(', ')} · ${ok ? '成功' : '见监控页'}`);
}

async function drainSingle() {
  if (roundsBusy || !singleArmed || !clickQueue.length) return;
  const queued = clickQueue.shift();
  roundsBusy = true;
  paintSingle();
  try {
    if (queued.epoch !== selectionEpoch) throw new Error('场景已变化，请重新点选');
    await harvestNamed(queued.ids);
  } catch (error) {
    singleArmed = false;
    clickQueue.length = 0;
    logRound(`单轮未完成：${error.message}`);
    $('single-note').textContent = `未完成：${error.message}`;
  } finally {
    roundsBusy = false;
    if (!singleArmed) {
      clickQueue.length = 0;
      $('single-note').textContent = '单轮已停止';
    } else {
      paintSingle();
      drainSingle();
    }
  }
}

async function startSingle() {
  if (singleArmed) return;
  if (roundsBusy) {
    $('single-note').textContent = '几轮还在跑，等它停或先停止后续轮';
    return;
  }
  if (!window.confirm('启动单轮点选。\n只确认这一次。之后点顺序号就抓那一颗，抓完继续等，直到停止单轮。')) return;
  const generation = ++sessionGeneration;
  stopRounds = false;
  roundsBusy = true;
  try {
    await pushSettings();
    await setPriority(0);
    singleKey = `single_${Date.now().toString(36)}`;
    await refreshScene(singleKey);
    if (generation !== sessionGeneration) return;
    singleArmed = true;
    clickQueue.length = 0;
    paintSingle();
  } catch (error) {
    singleArmed = false;
    $('single-note').textContent = `未启动：${error.message}`;
  } finally {
    roundsBusy = false;
  }
}

async function stopSingle() {
  sessionGeneration += 1;
  singleArmed = false;
  clickQueue.length = 0;
  $('single-panel').classList.remove('live');
  $('single-note').textContent = '单轮已停止';
  if (!roundsBusy && !unknownRequest) return;
  try {
    await postDebug('control_service', {
      command: 'CANCEL_NOW', expected_state_seq: 0, reason: '停止单轮',
    });
  } catch (error) {
    $('single-note').textContent = `停止请求失败：${error.message}`;
  }
}

async function startRounds() {
  if (singleArmed) {
    $('run-note').textContent = '单轮还开着。先停止单轮，再启动这几轮。';
    return;
  }
  if (roundsBusy) return;
  const count = Math.max(1, Math.min(30, parseInt($('rounds').value || '1', 10)));
  const mode = $('round-mode').value;
  if (mode === 'selected') {
    $('run-note').textContent = '已勾选目标只属于当前场景；请用单轮点选，换场后重新选择。';
    return;
  }
  const summary = `启动 ${count} 轮（${$('round-mode').selectedOptions[0].textContent}）。\n每轮重新开场景。真机与仿真用当前这些开关。`;
  if (!window.confirm(summary)) return;
  const generation = ++sessionGeneration;
  roundsBusy = true;
  stopRounds = false;
  try {
    await pushSettings();
    for (let index = 0; index < count; index += 1) {
      if (stopRounds || generation !== sessionGeneration) {
        logRound('后续轮已停止');
        break;
      }
      $('run-note').textContent = `第 ${index + 1} / ${count} 轮`;
      await harvestRound(index, mode);
    }
    $('run-note').textContent = stopRounds ? '已停止' : `${count} 轮已跑完`;
  } catch (error) {
    $('run-note').textContent = `未完成：${error.message}`;
    logRound(`未完成：${error.message}`);
  } finally {
    roundsBusy = false;
    setPriority(0).catch(() => {});
  }
}

async function submitDebug(action, payload, confirmText) {
  if (confirmText && !window.confirm(confirmText)) return null;
  try {
    const body = await postDebug(action, payload);
    return {ok: body.accepted === true, status: 200, body};
  } catch (error) {
    return {ok: false, status: error.status || null, body: error.body || {message: error.message}};
  }
}

['motion', 'full', 'ex', 'gr', 'tl'].forEach((id) => {
  $(id).addEventListener('change', () => tieEnables(id));
});
$('ack-recovery').addEventListener('click', async () => {
  const task = (state.task_executor && state.task_executor.state) || {};
  try {
    await postDebug('control_service', {
      command: 'ACKNOWLEDGE_RECOVERY',
      expected_state_seq: Number(task.state_seq) || 0,
      request_id: freshRequest(),
      reason: '调试页确认恢复',
    });
    $('run-note').textContent = '已确认恢复';
  } catch (error) {
    $('run-note').textContent = `确认失败：${error.message}`;
  }
});
$('round-apply').addEventListener('click', applyRound);
$('round-reload').addEventListener('click', () => loadRound().catch((error) => {
  $('round-note').textContent = `读取失败：${error.message}`;
}));
$('params-reload').addEventListener('click', loadParams);
$('params-apply').addEventListener('click', applyParams);
$('start-single').addEventListener('click', () => {
  startSingle().catch((error) => {
    $('single-note').textContent = `未启动：${error.message}`;
  });
});
$('stop-single').addEventListener('click', () => {
  stopSingle().catch((error) => {
    $('single-note').textContent = `停止失败：${error.message}`;
  });
});
$('grasp-selected').addEventListener('click', () => {
  enqueueGrasp([...picked]);
});
$('start-rounds').addEventListener('click', startRounds);
$('stop-rounds').addEventListener('click', async () => {
  stopRounds = true;
  sessionGeneration += 1;
  $('run-note').textContent = '正在停止后续轮';
  try {
    await postDebug('control_service', {
      command: 'CANCEL_NOW', expected_state_seq: 0, reason: '停止后续轮',
    });
  } catch (error) {
    $('run-note').textContent = `停止请求失败：${error.message}`;
  }
});

async function poll() {
  if (pollPending) return pollPending;
  pollPending = (async () => {
    try {
      const next = await getJSON('/api/state');
      state = next;
      lastStateAt = Date.now();
      renderTargets();
      const task = state.task_executor?.state || {};
      if (unknownRequest && task.run_id === unknownRequest && task.action_active === false
          && !task.recovery_required) unknownRequest = '';
    } catch (_) {
      lastStateAt = 0;
    }
  })();
  try { await pollPending; } finally { pollPending = null; }
}
poll().then(loadRound).catch((error) => {
  $('round-note').textContent = `读取失败：${error.message}`;
});
loadParams();
setInterval(poll, 1000);
