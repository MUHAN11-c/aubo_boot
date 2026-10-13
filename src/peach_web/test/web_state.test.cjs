// Browser transport regressions without ROS, hardware or third-party modules.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const source = fs.readFileSync(path.join(__dirname, '../web/app.js'), 'utf8');
const flush = () => new Promise((resolve) => setImmediate(resolve));
const response = (body, status = 200) => ({
  ok: status < 400, status, json: async () => body,
});

test('keyword filtering includes log text without weakening level or node filters', async () => {
  const b = await browser();
  const rows = b.run(`filterLogs([
    {level:40,name:'peach_arm',msg:'staging IK failed'},
    {level:40,name:'peach_arm',msg:'configured'},
    {level:20,name:'peach_arm',msg:'IK candidate'},
    {level:40,name:'camera',msg:'IK marker'}],30,'arm','ik')`);
  assert.equal(rows.length, 1);
  assert.match(rows[0].msg, /staging IK/);
});

test('target context is rechecked after confirmation before sending a command', async () => {
  const b = await browser(); b.requests.length = 0;
  b.handler(async () => response({accepted:true,result:{success:true}}));
  await b.run(`debugPost('build_action',{target_id:'target_0'},'confirm',()=> '场景已变化')`);
  assert.equal(b.requests.length, 0);
  assert.match(b.elements.get('debug-notice').textContent, /场景已变化/);
});

test('trajectory fetch failure invalidates cached tool frames without erasing the historical path', async () => {
  const b = await browser();
  b.run(`trajectoryPayload={xyz:[1,2,3],tool_frames:{tool_axis:{available:true,age_s:0.1}}};
    trajectoryFeed.lastSuccessAt=Date.now(); trajectoryFeed.error='';`);
  assert.equal(b.run('labTrajectory().tool_frames.tool_axis.available'),true);
  b.handler(async () => {throw new Error('lost');});
  await b.run('pollTrajectory()');
  assert.equal(b.run('labTrajectory().tool_frames.tool_axis.available'),false);
  assert.equal(b.run('labTrajectory().xyz.length'),3);
});

async function browser() {
  const elements = new Map();
  const requests = [];
  const timers = new Map();
  const intervals = new Map();
  const events = new Map();
  let timerId = 0;
  const document = {hidden: false, activeElement: null};
  function element(id, dataset = {}) {
    const classes = new Set();
    const attrs = new Map();
    const listeners = {};
    return {
      id, dataset, textContent: id, innerHTML: '', value: '', hidden: false,
      disabled: false, className: '',
      classList: {
        add: (...names) => names.forEach((n) => classes.add(n)),
        remove: (...names) => names.forEach((n) => classes.delete(n)),
        toggle: (name, on) => on ? classes.add(name) : classes.delete(name),
      },
      setAttribute: (name, value) => attrs.set(name, String(value)),
      getAttribute: (name) => attrs.get(name) ?? null,
      removeAttribute: (name) => attrs.delete(name),
      addEventListener: (name, fn) => { listeners[name] = fn; },
      dispatch: (name, event) => listeners[name]?.(event),
      focus() { document.activeElement = this; },
      querySelector() { return document.getElementById(id + '-child'); },
    };
  }
  document.getElementById = (id) => {
    if (!elements.has(id)) elements.set(id, element(id));
    return elements.get(id);
  };
  document.body = element('body');
  document.addEventListener = () => {};
  const tabs = [element('tab-monitor', {view: 'monitor'}), element('tab-debug', {view: 'debug'})];
  const buttons = [
    element('pause', {debug: 'control_service', payload: '{"command":"PAUSE"}'}),
    element('cancel-now', {debug: 'control_service', payload: '{"command":"CANCEL_NOW"}'}),
    element('cancel-action', {debugCancel: 'run_harvest_action'}),
    element('return-photo', {debug: 'photo_pose_service', confirm: '1'}),
  ];
  document.querySelectorAll = (selector) => {
    if (selector === '.view-tabs button') return tabs;
    if (selector === '[data-debug]') return [buttons[0], buttons[1], buttons[3]];
    if (selector === '[data-debug-cancel]') return [buttons[2]];
    if (selector === '[data-debug], [data-debug-cancel]') return buttons;
    return [];
  };
  let handler = async (url) => response(url.includes('/api/state')
    ? {debug: {enabled: true, motion_enabled: false}} : {});
  const setTimer = (callback, delay) => {
    const id = ++timerId; timers.set(id, {callback, delay}); return id;
  };
  const context = vm.createContext({
    document, console, AbortController, Date,
    setInterval: (callback, delay) => {
      const id = ++timerId; intervals.set(id, {callback, delay}); return id;
    },
    clearInterval: (id) => intervals.delete(id),
    setTimeout: setTimer, clearTimeout: (id) => timers.delete(id),
    fetch: (url, options) => {
      requests.push({url, options}); return handler(url, options);
    },
    window: {addEventListener: (name, fn) => events.set(name, fn),
      confirm: () => true, alert: () => {}},
  });
  vm.runInContext(source, context);
  vm.runInContext('const realRenderPlan = renderPlan; const realRenderLedger = renderLedger; const realRenderStatus = renderStatus; renderHardware = () => {};', context);
  // These renderers consume large ROS snapshots; exercise transport and controls here.
  for (const name of ['renderStatus', 'renderFlow', 'renderPlan', 'renderStages',
    'renderLedger', 'renderVision', 'renderSystem', 'renderTrajHud', 'renderTcpMini']) {
    vm.runInContext(`${name} = () => {};`, context);
  }
  await flush();
  return {
    run: (code) => vm.runInContext(code, context),
    elements, requests, buttons, tabs, timers, intervals, events, document,
    handler: (fn) => { handler = fn; },
  };
}

test('initial load requests the trajectory only once', async () => {
  const b = await browser();
  assert.equal(b.requests.filter((r) => r.url.includes('/api/trajectory')).length, 1);
});

test('slow state fetch never overlaps another state fetch', async () => {
  const b = await browser(); b.requests.length = 0;
  let complete;
  b.handler(() => new Promise((resolve) => { complete = resolve; }));
  const first = b.run('pollState()');
  const second = b.run('pollState()');
  assert.equal(b.requests.length, 1);
  complete(response({debug: {enabled: true}}));
  await Promise.all([first, second]);
});

test('accepted Nav2 request with success=false is an operation failure', async () => {
  const b = await browser();
  b.handler(async () => response({accepted: true, result: {success: false}}));
  await b.run('debugPost("manage_nodes_service", {command: "RESET"}, null)');
  assert.match(b.elements.get('debug-results').innerHTML, /debug-result err/);
  assert.match(b.elements.get('debug-results').innerHTML, /操作失败/);
});

test('transport loss records unknown outcome without retrying the POST', async () => {
  const b = await browser(); b.requests.length = 0;
  b.handler(async () => { throw new TypeError('network lost'); });
  await b.run('debugPost("control_service", {command: "PAUSE"}, null)');
  assert.match(b.elements.get('debug-results').innerHTML, /结果未知/);
  assert.equal(b.requests.length, 1);
});

test('duplicate command cannot submit while its previous POST is pending', async () => {
  const b = await browser(); b.requests.length = 0;
  let complete;
  b.handler(() => new Promise((resolve) => { complete = resolve; }));
  const first = b.run('debugPost("control_service", {command: "PAUSE"}, null)');
  const second = b.run('debugPost("control_service", {command: "PAUSE"}, null)');
  assert.equal(b.requests.length, 1);
  complete(response({accepted: true, result: {success: true}}));
  await Promise.all([first, second]);
});

test('state refresh retains pending lock and independent cancellation controls', async () => {
  const b = await browser();
  let complete;
  b.handler(() => new Promise((resolve) => { complete = resolve; }));
  const pending = b.run('debugPost("control_service", {command: "PAUSE"}, null)');
  b.run('renderDebugPanel({enabled: true, motion_enabled: false})');
  assert.equal(b.buttons[0].disabled, true);
  assert.equal(b.buttons[1].disabled, false);
  assert.equal(b.buttons[2].disabled, false);
  complete(response({accepted: true, result: {success: true}}));
  await pending;
  assert.equal(b.buttons[0].disabled, false);
});

test('disconnected state disables ordinary commands but retains cancellation', async () => {
  const b = await browser();
  b.handler(async () => { throw new TypeError('offline'); });
  await b.run('pollState()');
  assert.equal(b.buttons[0].disabled, true);
  assert.equal(b.buttons[2].disabled, false);
});

test('hung state request is aborted and a later poll can recover', async () => {
  const b = await browser();
  b.handler((_url, options) => new Promise((_resolve, reject) => {
    options?.signal?.addEventListener('abort', () => reject(new Error('timeout')));
  }));
  const pending = b.run('pollState()');
  const timeout = [...b.timers.values()].find((timer) => timer.delay > 0 && timer.delay <= 10000);
  assert.ok(timeout, 'GET must have a bounded waiting time');
  timeout.callback();
  await pending;
  b.handler(async () => response({debug: {enabled: true}}));
  await b.run('pollState()');
  assert.match(b.elements.get('connection').querySelector('span').textContent, /已连接/);
});

test('arrow-key tab navigation updates selection, panel and focus', async () => {
  const b = await browser();
  b.tabs[0].dispatch('keydown', {key: 'ArrowRight', preventDefault() {}});
  assert.equal(b.tabs[1].getAttribute('aria-selected'), 'true');
  assert.equal(b.elements.get('view-monitor').hidden, true);
  assert.equal(b.elements.get('view-debug').hidden, false);
  assert.equal(b.document.activeElement, b.tabs[1]);
});

test('backend 202 keeps the running goal visible and preserves its cancel advice', async () => {
  const b = await browser();
  b.run('monitorState.debug.motion_enabled = true;');
  b.handler(async () => response({accepted: false, result: {
    goal_handle_kept: true, message: '仍在执行；可查询状态或取消',
  }}, 202));
  await b.run('debugPost("run_harvest_action", {}, null)');
  const result = b.elements.get('debug-results').innerHTML;
  assert.match(result, /仍在执行/);
  assert.match(result, /可查询状态或取消/);
  assert.doesNotMatch(result, /请求被拒/);
});

test('backend 504 is an unknown execution outcome, not a rejected goal', async () => {
  const b = await browser();
  b.handler(async () => response({accepted: false, result: {message: '服务超时'}}, 504));
  await b.run('debugPost("manage_nodes_service", {}, null)');
  assert.match(b.elements.get('debug-results').innerHTML, /结果未知/);
});

test('BFCache restore restarts polling without duplicating timers', async () => {
  const b = await browser();
  assert.equal(b.intervals.size, 3);
  b.events.get('pagehide')({persisted: true});
  assert.equal(b.intervals.size, 0);
  b.events.get('pageshow')?.({persisted: true});
  await flush();
  assert.equal(b.intervals.size, 3);
  b.events.get('pageshow')?.({persisted: true});
  await flush();
  assert.equal(b.intervals.size, 3);
});

test('log filter keeps the chosen level and above', async () => {
  const b = await browser();
  const kept = b.run(`filterLogs([
    {stamp: 1, level: 10, name: "cam", msg: "dbg"},
    {stamp: 2, level: 20, name: "peach_arm", msg: "info"},
    {stamp: 3, level: 30, name: "peach_supervisor", msg: "warn"},
  ], 20, "")`);
  assert.deepEqual(Array.from(kept, (row) => row.level), [20, 30]);
  const named = b.run(`filterLogs([
    {stamp: 2, level: 20, name: "peach_arm", msg: "info"},
    {stamp: 3, level: 30, name: "peach_supervisor", msg: "warn"},
  ], 10, "supervisor")`);
  assert.equal(named.length, 1);
  assert.equal(named[0].name, "peach_supervisor");
});

test('current stage from the snapshot is written to the header', async () => {
  const b = await browser();
  b.handler(async () => response({
    debug: {enabled: true, motion_enabled: false},
    job: {
      batch_state: 2, target_phase: 5, target_id: "target_0",
      motion: {state: "IDLE"},
    },
    pipeline: {arm: [{current: true, state: "INSERTING", target_id: "target_0"}]},
  }));
  await b.run("pollState()");
  assert.equal(
    b.elements.get("stage-live").textContent,
    "运行中 · 靠近中 · INSERTING · target_0");
});

test('confirming after the snapshot expires cannot send an ordinary POST', async () => {
  const b = await browser(); b.requests.length = 0;
  b.run('window.confirm = () => {stateFeed.lastSuccessAt = Date.now() - 60000; return true;};');
  await b.run('debugPost("execute_action", {}, "confirm")');
  assert.equal(b.requests.length, 0);
});

test('canceled level 2 ledger preserves pregrasp verification and cancellation', async () => {
  const b = await browser();
  b.run(`realRenderLedger({live: {request_id: "r", rows: [{target_id: "target_0",
    outcome_name: "CANCELED", completion_level: 2, reason: "canceled", stages: []}]}})`);
  const html = b.elements.get('ledger-body').innerHTML;
  assert.match(html, /预抓取已验证/);
  assert.match(html, /取消/);
  assert.doesNotMatch(html, /采摘已确认/);
});

test('perception HARVESTED and level 6 never count as confirmed harvest', async () => {
  const b = await browser();
  b.run(`monitorState = {ledger: {live: {request_id: "r", rows: [{target_id: "target_0",
    outcome_name: "SUCCEEDED", completion_level: 6}]}}};
    realRenderPlan({targets: {harvest_run_id: "r", observations: [{target_id: "target_0",
      harvest_status: "HARVESTED", tracking_status: "LOST", candidate: {}}]}}, {state: {run_id: "r"}});`);
  assert.match(b.elements.get('plan-summary-inline').innerHTML, /采摘确认 <b>0/);
  assert.match(b.elements.get('target-list').innerHTML, /撤退已确认/);
  assert.doesNotMatch(b.elements.get('target-list').innerHTML, /HARVESTED/);
});

test('only matching current ledger level 7 counts as confirmed harvest', async () => {
  const b = await browser();
  b.run(`monitorState = {ledger: {live: {request_id: "r", rows: [{target_id: "target_0",
    completion_level: 7}]}}}; realRenderPlan({}, {state: {run_id: "r"}});`);
  assert.match(b.elements.get('plan-summary-inline').innerHTML, /采摘确认 <b>1/);
  b.run('realRenderPlan({}, {state: {run_id: "another_run"}})');
  assert.match(b.elements.get('plan-summary-inline').innerHTML, /采摘确认 <b>0/);
});

test('allowed decision with diagnostic reason is not displayed as a rejection', async () => {
  const b = await browser();
  b.run('realRenderStatus({reconstruction: {grasp_decision: {allowed: true, reason: "radial_budget_negative"}}})');
  assert.match(b.elements.get('status-strip').innerHTML, /许可允许.*诊断/);
  assert.doesNotMatch(b.elements.get('status-strip').innerHTML, /拒绝|未许可/);
});

test('photo return never posts through disabled motion or recovery gates', async () => {
  const b = await browser(); b.requests.length = 0;
  await b.run('debugPost("photo_pose_service", {}, "confirm")');
  assert.equal(b.requests.length, 0);
  b.run('monitorState = {debug: {enabled: true, motion_enabled: true}, task_executor: {state: {recovery_required: true}}}; updateDebugControls();');
  assert.equal(b.buttons[3].disabled, true);
  assert.match(b.buttons[3].title, /恢复/);
  await b.run('debugPost("photo_pose_service", {}, "confirm")');
  assert.equal(b.requests.length, 0);
});

test('photo return uses the existing service after explicit recovery has cleared', async () => {
  const b = await browser(); b.requests.length = 0;
  b.run('monitorState = {debug: {enabled: true, motion_enabled: true}, task_executor: {state: {recovery_required: false, action_active: false}}}; updateDebugControls();');
  b.handler(async () => response({accepted: true, result: {success: true}}));
  await b.run('debugPost("photo_pose_service", {}, "confirm")');
  assert.equal(b.requests.length, 1);
  assert.equal(b.requests[0].url, '/api/debug/photo_pose_service');
});

test('photo return confirmation rechecks the motion gate before submitting', async () => {
  const b = await browser(); b.requests.length = 0;
  b.run('monitorState.debug.motion_enabled = true; window.confirm = () => { monitorState.debug.motion_enabled = false; return true; };');
  await b.run('debugPost("photo_pose_service", {}, "confirm")');
  assert.equal(b.requests.length, 0);
});

test('photo return is disabled during an arm cycle but execution-off still permits planning', async () => {
  const b = await browser(); b.requests.length = 0;
  b.run('monitorState = {debug: {enabled: true, motion_enabled: true}, manipulation: {status: {running: true}}}; updateDebugControls();');
  assert.equal(b.buttons[3].disabled, true);
  b.run('monitorState.manipulation.status = {running: false, execution_enabled: false}; updateDebugControls();');
  assert.equal(b.buttons[3].disabled, false);
  b.handler(async () => response({accepted: true, result: {success: true}}));
  await b.run('debugPost("photo_pose_service", {}, "confirm")');
  assert.equal(b.requests.length, 1);
});

test('main operation area explains PREGRASP_ONLY and independent grasp diagnostic', async () => {
  const b = await browser();
  b.run(`renderOperationContext({params: {"/peach_supervisor": {execute_pregrasp_only: true}},
    task_executor: {state: {execution_enabled: true, grasp_enabled: true, tool_enabled: false}},
    reconstruction: {grasp_decision: {allowed: true, reason: "radial_budget_negative"}}});`);
  assert.match(b.elements.get('operation-mode').textContent, /PREGRASP_ONLY/);
  assert.match(b.elements.get('operation-status').textContent, /预抓取.*不套入.*不剪切/);
  assert.match(b.elements.get('operation-decision').textContent, /许可允许.*诊断：radial_budget_negative/);
  assert.doesNotMatch(b.elements.get('operation-status').textContent, /radial_budget_negative/);
});

test('main operation area shows FULL enable blockers and preserves canceled level 2 evidence', async () => {
  const b = await browser();
  b.run(`renderOperationContext({params: {"/peach_supervisor": {execute_pregrasp_only: false}},
    task_executor: {state: {execution_enabled: true, grasp_enabled: false, tool_enabled: false,
      recovery_required: true, blockers: ["recovery_required"]}},
    job: {completion: {completion_level: 2, outcome_name: "CANCELED"},
      why: "预抓取已验证 · 批次已取消 · 等待人工恢复确认"}});`);
  assert.match(b.elements.get('operation-mode').textContent, /FULL/);
  assert.match(b.elements.get('operation-status').textContent, /抓取关闭.*工具关闭.*恢复/);
  assert.match(b.elements.get('operation-evidence').textContent, /预抓取已验证.*取消/);
  assert.doesNotMatch(b.elements.get('operation-evidence').textContent, /采摘已确认/);
});

test('PREVIEW can submit with motion disabled while OBSERVE_ONLY cannot', async () => {
  const b = await browser(); b.requests.length = 0;
  b.handler(async () => response({accepted: true, result: {success: true}}));
  await b.run('debugPost("execute_action", {mode: "OBSERVE_ONLY"}, null)');
  assert.equal(b.requests.length, 0);
  await b.run('debugPost("execute_action", {mode: "PREVIEW"}, null)');
  assert.equal(b.requests.length, 1);
});

test('return-photo click confirms and submits only the existing empty service request', async () => {
  const b = await browser(); b.requests.length = 0;
  b.run('monitorState.debug.motion_enabled = true;');
  b.run('window.confirm = () => { window.photoConfirmed = true; return true; };');
  b.handler(async () => response({accepted: true, result: {success: true}}));
  await b.buttons[3].dispatch('click');
  assert.equal(b.run('window.photoConfirmed'), true);
  assert.equal(b.requests.length, 1);
  assert.equal(b.requests[0].url, '/api/debug/photo_pose_service');
  assert.equal(b.requests[0].options.body, '{}');
});
