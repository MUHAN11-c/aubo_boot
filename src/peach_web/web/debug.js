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
const picked = new Set();
const clickQueue = [];

async function getJSON(path, opts) {
  const response = await fetch(path, opts);
  const text = await response.text();
  let data = null;
  try { data = text ? JSON.parse(text) : null; } catch (_) { data = null; }
  if (!response.ok) {
    const message = (data && (data.error || data.message)) || response.status;
    const error = new Error(message);
    error.status = response.status;
    error.body = data;
    throw error;
  }
  return data;
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
    const params = await getJSON('/api/params', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({changes: [
        {node: 'peach_observability', name: 'debug.motion_enabled', value: motion},
        {node: 'peach_supervisor', name: 'execute_pregrasp_only', value: !full},
      ]}),
    });
    const rejected = (params.results || []).filter((row) => !row.accepted);
    const enables = await getJSON('/api/debug/set_enables_service', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({
        execution, grasp, tool, reason: '单步调试本轮设置',
      }),
    });
    const enableOk = enables.accepted === true && enables.result?.accepted !== false;
    $('round-note').textContent = rejected.length
      ? `参数有 ${rejected.length} 项未写入`
      : (enableOk ? '本轮设置已写入' : (enables.message || '使能未接受'));
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
    const data = await getJSON('/api/params', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({changes}),
    });
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
  return getJSON(`/api/debug/${encodeURIComponent(action)}`, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(payload || {}),
  });
}

async function pushSettings() {
  const changes = [
    {node: 'peach_observability', name: 'debug.motion_enabled', value: $('motion').checked},
    {node: 'peach_supervisor', name: 'execute_pregrasp_only', value: !$('full').checked},
    {node: 'peach_supervisor', name: 'skip_reconstruction', value: $('skip').checked},
    {node: 'peach_target_reconstruction_node', name: 'capture.auto_mode', value: !$('skip').checked},
    {node: 'peach_arm', name: 'quality.allow_unrefined_geometry', value: $('skip').checked},
  ];
  await getJSON('/api/params', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({changes}),
  });
  await postDebug('set_enables_service', {
    execution: $('ex').checked,
    grasp: $('gr').checked,
    tool: $('tl').checked,
    reason: '单步调试',
  });
}

function freshRequest() {
  const now = new Date();
  const pad = (value, width = 2) => String(value).padStart(width, '0');
  const stamp = `${now.getFullYear()}${pad(now.getMonth() + 1)}${pad(now.getDate())}`
    + `_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`
    + `_${pad(now.getMilliseconds(), 3)}`;
  return `grasp_${stamp}`;
}

async function ackIfNeeded(request) {
  const task = (state.task_executor && state.task_executor.state) || {};
  // 只在批次已停在恢复等待时确认。接触过程中反馈也会带 recovery_required，
  // 那时确认会打断本颗，不能提前清。
  if (task.recovery_required !== true || Number(task.batch_state) !== 7) return;
  await postDebug('control_service', {
    command: 'ACKNOWLEDGE_RECOVERY',
    expected_state_seq: 0,
    request_id: request,
    reason: '下一轮前确认恢复',
  });
}

async function finishHarvest(pending, request) {
  let settled = null;
  pending.then(
    (result) => { settled = {ok: true, result}; },
    (error) => { settled = {ok: false, error}; },
  );
  while (settled == null) {
    await poll();
    await ackIfNeeded(request);
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
  const deadline = Date.now() + 45000;
  while (Date.now() < deadline) {
    await poll();
    const targets = state.perception && state.perception.targets;
    const sameEpoch = epoch == null || !targets || targets.scene_epoch === epoch;
    const ids = new Set(observations().map((item) => item.target_id));
    const ready = !wanted || wanted.every((id) => ids.has(id));
    if (sameEpoch && targets && targets.target_set_locked === true && ready) return;
    await new Promise((resolve) => setTimeout(resolve, 500));
  }
  if (!observations().length) throw new Error('开场景后没有目标');
}

function idsForRound(index) {
  const mode = $('round-mode').value;
  const items = byPriority(observations());
  if (mode === 'all') return null;
  if (mode === 'first') {
    const first = items.find((item) => Number(item.priority) === 1) || items[0];
    return first ? [first.target_id] : [];
  }
  if (mode === 'rotate') {
    const wanted = (index % Math.max(items.length, 1)) + 1;
    const match = items.find((item) => Number(item.priority) === wanted) || items[index % items.length];
    return match ? [match.target_id] : [];
  }
  return items.filter((item) => picked.has(item.target_id)).map((item) => item.target_id);
}

async function setPriority(ordinal) {
  const params = await getJSON('/api/params', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({changes: [
      {node: 'peach_supervisor', name: 'selection_priority', value: ordinal},
    ]}),
  });
  const rejected = (params.results || []).filter((row) => !row.accepted);
  if (rejected.length) throw new Error(rejected.map((row) => row.message).join('；') || '顺序号未写入');
}

async function harvestRound(index, presetIds) {
  const request = freshRequest();
  await ackIfNeeded(request);
  const mode = $('round-mode').value;
  // 按顺序 / 每轮第一颗：顺序号交给本批自己的拍照锁定，不提前点一个会换代的 ID。
  if (presetIds == null && (mode === 'rotate' || mode === 'first')) {
    const ordinal = mode === 'first' ? 1 : index + 1;
    await setPriority(ordinal);
    logRound(`第 ${index + 1} 轮 · 顺序 ${ordinal}`);
    const result = await finishHarvest(postDebug('run_harvest_action', {
      request_id: request, scene_key: request,
      intent: 'PICK_SELECTED', selection_mode: 'AUTO', target_ids: [],
    }), request);
    const ok = result.result && result.result.success === true
      && Number(result.result.summary_succeeded) === 1
      && Number(result.result.summary_attempted) === 1;
    logRound(`第 ${index + 1} 轮结束 · ${ok ? '成功' : '见监控页'}`);
    return;
  }
  await setPriority(0);
  let ids = presetIds;
  if (ids == null && mode === 'all') {
    ids = null;
  } else if (ids == null) {
    await refreshScene(request);
    ids = idsForRound(index);
  } else {
    await refreshScene(request, ids);
    const have = new Set(observations().map((item) => item.target_id));
    ids = (ids || []).filter((id) => have.has(id));
  }
  if (ids === null) {
    logRound(`第 ${index + 1} 轮 · 全部`);
    const result = await finishHarvest(postDebug('run_harvest_action', {
      request_id: request, scene_key: request,
      intent: 'PICK_ALL', selection_mode: 'AUTO', target_ids: [],
    }), request);
    logRound(`第 ${index + 1} 轮结束 · ${result.result && result.result.success ? '成功' : '见监控页'}`);
    return;
  }
  if (!ids.length) throw new Error('没有可抓的 ID');
  logRound(`第 ${index + 1} 轮 · ${ids.join(', ')}`);
  const result = await finishHarvest(postDebug('run_harvest_action', {
    request_id: request, scene_key: request,
    intent: 'PICK_SELECTED', selection_mode: 'MANUAL', target_ids: ids,
  }), request);
  logRound(`第 ${index + 1} 轮结束 · ${result.result && result.result.success ? '成功' : '见监控页'}`);
}

function paintSingle() {
  $('single-panel').classList.toggle('live', singleArmed);
  if (!singleArmed) return;
  const waiting = clickQueue.map((ids) => ids.join('+'));
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
  clickQueue.push(ids);
  paintSingle();
  if (!roundsBusy) drainSingle();
}

async function harvestNamed(ids) {
  const request = freshRequest();
  await ackIfNeeded(request);
  await setPriority(0);
  logRound(`单轮 · ${ids.join(', ')}`);
  const result = await finishHarvest(postDebug('run_harvest_action', {
    request_id: request,
    scene_key: singleKey,
    intent: 'PICK_SELECTED',
    selection_mode: 'MANUAL',
    target_ids: ids,
  }), request);
  const ok = result.result && result.result.success === true;
  logRound(`单轮结束 · ${ids.join(', ')} · ${ok ? '成功' : '见监控页'}`);
}

async function drainSingle() {
  if (roundsBusy || !singleArmed || !clickQueue.length) return;
  const ids = clickQueue.shift();
  roundsBusy = true;
  paintSingle();
  try {
    await harvestNamed(ids);
  } catch (error) {
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
  try {
    await pushSettings();
    await setPriority(0);
    singleKey = `single_${Date.now().toString(36)}`;
    singleArmed = true;
    clickQueue.length = 0;
    paintSingle();
  } catch (error) {
    singleArmed = false;
    $('single-note').textContent = `未启动：${error.message}`;
  }
}

async function stopSingle() {
  singleArmed = false;
  clickQueue.length = 0;
  $('single-panel').classList.remove('live');
  $('single-note').textContent = '单轮已停止';
  if (!roundsBusy) return;
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
  if (mode === 'selected' && !picked.size) {
    $('run-note').textContent = '先点选 ID，或改成每轮第一颗 / 按顺序 / 全部';
    return;
  }
  const summary = `启动 ${count} 轮（${$('round-mode').selectedOptions[0].textContent}）。\n每轮重新开场景。真机与仿真用当前这些开关。`;
  if (!window.confirm(summary)) return;
  roundsBusy = true;
  stopRounds = false;
  try {
    await pushSettings();
    for (let index = 0; index < count; index += 1) {
      if (stopRounds) {
        logRound('后续轮已停止');
        break;
      }
      $('run-note').textContent = `第 ${index + 1} / ${count} 轮`;
      const preset = mode === 'selected' ? [...picked] : null;
      await harvestRound(index, preset);
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
    const body = await getJSON(`/api/debug/${encodeURIComponent(action)}`, {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(payload || {}),
    });
    return {ok: body.accepted === true, status: 200, body};
  } catch (error) {
    return {ok: false, status: error.status || null, body: error.body || {message: error.message}};
  }
}

['motion', 'full', 'ex', 'gr', 'tl'].forEach((id) => {
  $(id).addEventListener('change', () => tieEnables(id));
});
async function fillSuccess() {
  $('motion').checked = true;
  $('full').checked = true;
  $('ex').checked = true;
  $('gr').checked = true;
  $('tl').checked = false;
  $('skip').checked = true;
  $('rounds').value = '4';
  $('round-mode').value = 'rotate';
  $('round-note').textContent = '已填入成功测试，正在写入…';
  const changes = [
    {node: 'peach_observability', name: 'debug.motion_enabled', value: true},
    {node: 'peach_supervisor', name: 'execute_pregrasp_only', value: false},
    {node: 'peach_supervisor', name: 'skip_reconstruction', value: true},
    {node: 'peach_target_reconstruction_node', name: 'capture.auto_mode', value: false},
    {node: 'peach_supervisor', name: 'selection_priority', value: 0},
    {node: 'peach_arm', name: 'quality.allow_unrefined_geometry', value: true},
    {node: 'peach_arm', name: 'timeouts.refined_s', value: 30},
    {node: 'peach_arm', name: 'moveit.velocity_scaling', value: 0.4},
    {node: 'peach_arm', name: 'moveit.acceleration_scaling', value: 0.8},
    {node: 'peach_arm', name: 'moveit.transit_velocity_scaling', value: 0.8},
    {node: 'peach_arm', name: 'moveit.transit_acceleration_scaling', value: 0.8},
    {node: 'peach_arm', name: 'moveit.execute_timeout_s', value: 30},
  ];
  const params = await getJSON('/api/params', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({changes}),
  });
  await postDebug('set_enables_service', {
    execution: true, grasp: true, tool: false, reason: '实验室成功测试',
  });
  const rejected = (params.results || []).filter((row) => !row.accepted);
  $('round-note').textContent = rejected.length
    ? `有 ${rejected.length} 项未写入：${rejected.map((row) => row.name).join(', ')}`
    : '成功测试已写入：4 轮、按顺序、跳过重建、不切刀';
  await loadParams();
}

$('fill-success').addEventListener('click', () => {
  if (!window.confirm('写入实验室成功测试：接触速度 0.4、执行时限 30 秒、跳过重建、套入不切刀、4 轮按顺序。真机接触速度不要用 0.4。')) return;
  fillSuccess().catch((error) => {
    $('round-note').textContent = `写入失败：${error.message}`;
  });
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
  try {
    state = await getJSON('/api/state') || {};
    renderTargets();
  } catch (_) {
    state = state || {};
  }
}
poll().then(loadRound).catch((error) => {
  $('round-note').textContent = `读取失败：${error.message}`;
});
loadParams();
setInterval(poll, 1000);
