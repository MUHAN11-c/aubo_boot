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
  ];
  document.querySelectorAll = (selector) => {
    if (selector === '.view-tabs button') return tabs;
    if (selector === '[data-debug]') return buttons.slice(0, 2);
    if (selector === '[data-debug-cancel]') return buttons.slice(2);
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
