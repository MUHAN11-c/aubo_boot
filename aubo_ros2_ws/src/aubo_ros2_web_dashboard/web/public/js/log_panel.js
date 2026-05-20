// log_panel.js — 浏览器端综合日志面板
// 捕获: console / 全局错误 / Transport 连接与订阅 / 服务调用 / 驱动模式 / 页面生命周期
import { $, escapeHtml } from './core/utils.js';

const TAG = 'log_panel';

// ── 状态 ───────────────────────────────────────────────────────────────
let logs = [];
let paused = false;
let autoScroll = true;
const MAX_ENTRIES = 800;
const MAX_DOM = 400;

const levelFilters = { log: true, warn: true, error: true, info: true, debug: true, transport: true, service: true, mode: true, lifecycle: true };

const SOURCE_COLORS = {
  console:      'var(--text-muted)',
  global:       'var(--red)',
  promise:      'var(--red)',
  rosbridge:    'var(--green)',
  subscribe:    '#60a5fa',
  unsubscribe:  '#94a3b8',
  service:      '#c084fc',
  service_err:  'var(--red)',
  driver_mode:  'var(--accent)',
  robot_status: '#34d399',
  log_panel:    'var(--orange)',
  lifecycle:    'var(--yellow)',
};

// ── DOM 辅助（$ 已从 utils.js 导入）───────────────────────────────────

function timestamp() {
  const d = new Date();
  const h = String(d.getHours()).padStart(2, '0');
  const m = String(d.getMinutes()).padStart(2, '0');
  const s = String(d.getSeconds()).padStart(2, '0');
  const ms = String(d.getMilliseconds()).padStart(3, '0');
  return h + ':' + m + ':' + s + '.' + ms;
}

// escapeHtml 已从 utils.js 导入

// ── 日志写入 ───────────────────────────────────────────────────────────
function addLog(level, source, msg) {
  if (paused) return;
  if (!levelFilters[level] && !levelFilters[source]) return;

  const entry = { ts: timestamp(), level, source, msg };
  logs.push(entry);
  if (logs.length > MAX_ENTRIES) logs = logs.slice(-MAX_ENTRIES);

  renderLine(entry);
  updateCount();
}

function renderLine(entry) {
  const container = $('log-lines');
  if (!container) return;

  const color = SOURCE_COLORS[entry.source] || 'var(--text-muted)';
  const div = document.createElement('div');
  div.className = 'log-line log-line--' + entry.level;
  div.innerHTML =
    '<span class="log-ts">' + escapeHtml(entry.ts) + '</span>' +
    '<span class="log-src" style="color:' + color + '">[' + escapeHtml(entry.source) + ']</span>' +
    '<span class="log-msg">' + escapeHtml(entry.msg) + '</span>';

  container.appendChild(div);
  while (container.children.length > MAX_DOM) {
    container.removeChild(container.firstChild);
  }
  if (autoScroll) container.scrollTop = container.scrollHeight;
}

function renderAll() {
  const container = $('log-lines');
  if (!container) return;
  container.innerHTML = '';
  const slice = logs.length > MAX_DOM ? logs.slice(-MAX_DOM) : logs;
  for (const e of slice) {
    const color = SOURCE_COLORS[e.source] || 'var(--text-muted)';
    const div = document.createElement('div');
    div.className = 'log-line log-line--' + e.level;
    div.innerHTML =
      '<span class="log-ts">' + escapeHtml(e.ts) + '</span>' +
      '<span class="log-src" style="color:' + color + '">[' + escapeHtml(e.source) + ']</span>' +
      '<span class="log-msg">' + escapeHtml(e.msg) + '</span>';
    container.appendChild(div);
  }
  if (autoScroll && container.lastChild) {
    container.lastChild.scrollIntoView(false);
  }
}

function updateCount() {
  const el = $('log-count');
  if (el) el.textContent = logs.length + ' 条';
}

// ── 全局导出 addLog ────────────────────────────────────────────────────
// 其他模块可通过 window.__ivgLog(level, source, msg) 直接写入日志
window.__ivgLog = addLog;

// ── Console 拦截 ───────────────────────────────────────────────────────
function installConsoleHook() {
  const orig = {};
  ['log', 'warn', 'error', 'info', 'debug'].forEach(function (level) {
    orig[level] = console[level];
    console[level] = function () {
      var args = Array.from(arguments);
      var msg = args.map(function (a) {
        if (a instanceof Error) return a.message || a.toString();
        if (typeof a === 'object') {
          try { return JSON.stringify(a); } catch (e) { return String(a); }
        }
        return String(a);
      }).join(' ');
      addLog(level, 'console', msg);
      orig[level].apply(console, arguments);
    };
  });
}

// ── 全局错误 ───────────────────────────────────────────────────────────
function installErrorHooks() {
  window.addEventListener('error', function (e) {
    var info = e.message || 'Unknown error';
    if (e.filename) info += '  (' + e.filename + ':' + e.lineno + ')';
    addLog('error', 'global', info);
  });
  window.addEventListener('unhandledrejection', function (e) {
    addLog('error', 'promise', (e.reason && e.reason.message) || String(e.reason));
  });
}

// ── Transport 钩子 ─────────────────────────────────────────────────────
function installTransportHook() {
  var timer = setInterval(function () {
    var t = globalThis.ivgTransport;
    if (!t) return;
    if (t._logPanelHooked) { clearInterval(timer); return; }
    t._logPanelHooked = true;

    // ── 连接事件 ──
    var origOnControl = t.onControlJson.bind(t);
    t.onControlJson = function (fn) {
      origOnControl(function (ctrl) {
        if (ctrl && ctrl.op === 'connection') addLog('transport', 'rosbridge', 'WebSocket 已连接');
        if (ctrl && ctrl.op === 'close')     addLog('transport', 'rosbridge', 'WebSocket 连接关闭');
        if (ctrl && ctrl.op === 'error')     addLog('error', 'rosbridge', '错误: ' + (ctrl.message || 'ros_error'));
        fn(ctrl);
      });
    };

    // ── 订阅 ──
    var origSubscribe = t.subscribe.bind(t);
    t.subscribe = function (spec) {
      var topic = spec.topic || '';
      var ok = origSubscribe(spec);
      if (topic) {
        addLog('transport', 'subscribe', (ok ? '✓ ' : '✗ ') + topic + '  [' + (spec.msgType || spec.msg_type || '?') + ']');
      }
      return ok;
    };

    // ── 取消订阅 ──
    var origUnsubscribe = t.unsubscribe.bind(t);
    t.unsubscribe = function (topic) {
      if (topic) addLog('transport', 'unsubscribe', topic);
      return origUnsubscribe(topic);
    };

    // ── 服务调用 ──
    var origCallService = t.callService.bind(t);
    t.callService = function (spec) {
      var svc = spec.service || spec.srvName || '';
      var start = performance.now();
      return origCallService(spec).then(function (r) {
        var ms = (performance.now() - start).toFixed(0);
        var ok = r && r.success !== false;
        addLog(ok ? 'service' : 'service_err', 'service', (ok ? '✓ ' : '✗ ') + svc + '  (' + ms + 'ms)');
        return r;
      }).catch(function (e) {
        var ms = (performance.now() - start).toFixed(0);
        addLog('error', 'service_err', '✗ ' + svc + '  (' + ms + 'ms)  ' + String(e));
        throw e;
      });
    };

    // ── 驱-动模式变更 (轮询) ──
    var lastMode = t._driverMode;
    setInterval(function () {
      if (t._driverMode && t._driverMode !== lastMode) {
        lastMode = t._driverMode;
        addLog('mode', 'driver_mode', '驱动模式切换 → ' + (lastMode === 'real' ? '真实硬件' : '仿真模式'));
      }
    }, 1000);

    addLog('info', 'log_panel', 'Transport 监控钩子已安装');
    clearInterval(timer);
  }, 300);
}

// ── 页面生命周期 ───────────────────────────────────────────────────────
function installLifecycleHooks() {
  document.addEventListener('visibilitychange', function () {
    addLog('lifecycle', 'lifecycle', document.hidden ? '页面隐藏 (后台)' : '页面可见 (前台)');
  });
  window.addEventListener('beforeunload', function () {
    addLog('lifecycle', 'lifecycle', '页面即将关闭');
  });
  window.addEventListener('pagehide', function () {
    addLog('lifecycle', 'lifecycle', '页面已隐藏');
  });
}

// ── 过滤搜索 ───────────────────────────────────────────────────────────
function filterByText(text) {
  var container = $('log-lines');
  if (!container) return;
  var q = String(text || '').trim().toLowerCase();
  var lines = container.children;
  for (var i = 0; i < lines.length; i++) {
    var line = lines[i];
    if (!q) { line.style.display = ''; continue; }
    line.style.display = line.textContent.toLowerCase().indexOf(q) !== -1 ? '' : 'none';
  }
}

// ── 导出日志 ───────────────────────────────────────────────────────────
function exportLogs() {
  if (logs.length === 0) return;
  var text = logs.map(function (e) {
    return e.ts + ' [' + e.source + '] ' + e.level.toUpperCase() + '  ' + e.msg;
  }).join('\n');
  var blob = new Blob([text], { type: 'text/plain' });
  var url = URL.createObjectURL(blob);
  var a = document.createElement('a');
  a.href = url;
  a.download = 'ivg_log_' + new Date().toISOString().slice(0, 10) + '.txt';
  a.click();
  URL.revokeObjectURL(url);
}

// ── 控制按钮绑定 ───────────────────────────────────────────────────────
function bindControls() {
  $('log-btn-clear') && $('log-btn-clear').addEventListener('click', function () {
    logs.length = 0;
    var c = $('log-lines'); if (c) c.innerHTML = '';
    updateCount();
  });

  $('log-btn-pause') && $('log-btn-pause').addEventListener('click', function () {
    paused = !paused;
    this.textContent = paused ? '▶ 恢复' : '⏸ 暂停';
    this.classList.toggle('log-ctrl--active', paused);
  });

  $('log-btn-scroll') && $('log-btn-scroll').addEventListener('click', function () {
    autoScroll = !autoScroll;
    this.textContent = autoScroll ? '⏹ 自动滚动:开' : '⏺ 自动滚动:关';
    this.classList.toggle('log-ctrl--active', !autoScroll);
  });

  $('log-btn-export') && $('log-btn-export').addEventListener('click', exportLogs);

  // 搜索
  var input = $('log-filter-input');
  if (input) {
    input.addEventListener('input', function () { filterByText(this.value); });
  }

  // 来源过滤按钮
  ['log', 'warn', 'error', 'info', 'debug', 'transport', 'service', 'mode', 'lifecycle'].forEach(function (key) {
    var btn = $('log-btn-' + key);
    if (!btn) return;
    btn.addEventListener('click', function () {
      levelFilters[key] = !levelFilters[key];
      btn.classList.toggle('log-ctrl--off', !levelFilters[key]);
      renderAll();
    });
  });
}

// ── 入口 ───────────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', function () {
  installConsoleHook();
  installErrorHooks();
  installLifecycleHooks();
  bindControls();
  installTransportHook();
  addLog('info', 'log_panel', '日志面板已就绪 — 等待事件…');
  addLog('lifecycle', 'lifecycle', '页面加载完成');
});
