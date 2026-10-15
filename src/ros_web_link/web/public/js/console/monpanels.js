// console/monpanels.js — 诊断（rqt_runtime_monitor 风格）与 ROS 日志（rqt_console 风格）
//
// 深读补移植（2026-10-15）：
//   诊断 → 指示灯映射      ← RVIZ-RQT StatusPanel.vue:224-242（全项目唯一真数据消费者）
//   日志骨架（级别过滤/暂停/上限淘汰/新条目置顶）
//                          ← RVIZ-RQT LogViewer.vue:102-160（其数据层是 mock，此处接真 /rosout）
// 面板挂在「监控」标签组；快照 window.__rwl_diag() / window.__rwl_roslog()。

import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';

const $ = (id) => document.getElementById(id);

// diagnostic_msgs/DiagnosticStatus.level：0 OK / 1 WARN / 2 ERROR / 3 STALE
const DIAG_LEVELS = ['ok', 'warn', 'bad', 'warn'];
const DIAG_ICONS = ['✓', '⚠', '✗', '◌'];

class DiagnosticsPanel {
  constructor() {
    this._client = foxgloveRosClient();
    this._handle = null;
    this._rows = new Map();        // name → {level, message, values, stamp}
    this._pendingRender = false;
    this._enabled = false;
    $('diag-enable').addEventListener('change', (e) => this.toggle(e.target.checked));
    $('diag-topic').addEventListener('change', () => {
      if (this._enabled) this.toggle(true);   // 换话题即重订
    });
    this._client.onStateChange((s) => {
      if (s === 'connected' && $('diag-enable').checked) this.toggle(true);
      if (s === 'closed') this._handle = null;
    });
    if (this._client.isConnected && $('diag-enable').checked) this.toggle(true);
  }

  toggle(on) {
    this._enabled = on;
    if (this._handle) {
      try { this._handle.cancel(); } catch (_) { /* keep */ }
      this._handle = null;
    }
    if (!on) {
      $('diag-count').textContent = '';
      return;
    }
    const topic = $('diag-topic').value.trim() || '/diagnostics';
    this._rows.clear();
    this._handle = this._client.subscribe(topic, '', (m) => {
      this._stamp(m.data?.header);
      for (const st of m.data?.status || []) {
        this._rows.set(st.name, {
          // int8 标量 CDR 解码为 number；防御 bytes（rclpy 侧形态）
          level: Number(typeof st.level === 'number' ? st.level
            : Array.isArray(st.level) ? st.level[0]
            : parseInt(st.level, 10) || 0),
          message: String(st.message || ''),
          values: (st.values || []).map((kv) => `${kv.key}=${kv.value}`).join(' '),
          stamp: Date.now(),
        });
      }
      this._scheduleRender();
    }, { dispatchMode: 'latest' });
  }

  _stamp(h) {
    if (!h?.stamp) return;
    const ns = Number(h.stamp.sec) * 1e9 + Number(h.stamp.nanosec);
    if (ns > (this._lastStampNs ?? 0)) this._lastStampNs = ns;
  }

  _scheduleRender() {
    if (this._pendingRender) return;
    this._pendingRender = true;
    setTimeout(() => {
      this._pendingRender = false;
      const rows = [...this._rows.entries()]
        .sort((a, b) => a[0].localeCompare(b[0]));
      $('diag-count').textContent = rows.length ? `${rows.length} 项` : '等待…';
      $('diag-list').innerHTML = rows.map(([name, r]) => {
        const cls = DIAG_LEVELS[r.level] || 'warn';
        return `<div><span class="${cls}">${DIAG_ICONS[r.level] ?? '⚠'}</span> ` +
          `<b>${name}</b> ${escapeHtml(r.message)}` +
          (r.values ? ` <span class="t" style="color:var(--muted)">${escapeHtml(r.values)}</span>` : '') +
          '</div>';
      }).join('') || '<div style="color:var(--muted)">启用后等待 /diagnostics 流…</div>';
    }, 500);
  }

  snapshot() {
    return {
      topic: this._enabled ? ($('diag-topic').value.trim() || '/diagnostics') : null,
      stampNs: this._lastStampNs ?? null,
      rows: [...this._rows.entries()].map(([name, r]) => ({ name, ...r })),
    };
  }
}

// rcl_interfaces/Log.level：10 DEBUG / 20 INFO / 30 WARN / 40 ERROR / 50 FATAL
const LOG_LEVELS = [
  [50, 'FATAL', 'bad'], [40, 'ERROR', 'bad'],
  [30, 'WARN', 'warn'], [20, 'INFO', 'ok'], [10, 'DEBUG', 't'],
];
const logLevelName = (lv) => {
  for (const [v, name] of LOG_LEVELS) if (lv >= v) return name;
  return 'DEBUG';
};
const logLevelClass = (lv) => {
  for (const [v, , cls] of LOG_LEVELS) if (lv >= v) return cls;
  return 't';
};

class RosLogPanel {
  constructor() {
    this._client = foxgloveRosClient();
    this._handle = null;
    this._logs = [];               // {t, level, name, msg}；新在上，上限 500
    this._paused = false;
    this._pendingRender = false;
    $('roslog-level').addEventListener('change', () => this._render());
    $('btn-roslog-pause').addEventListener('click', () => {
      this._paused = !this._paused;
      $('btn-roslog-pause').textContent = this._paused ? '继续' : '暂停';
    });
    $('btn-roslog-clear').addEventListener('click', () => {
      this._logs.length = 0;
      this._render();
    });
    this._client.onStateChange((s) => {
      if (s === 'connected') this._subscribe();
      if (s === 'closed') this._handle = null;
    });
    if (this._client.isConnected) this._subscribe();
  }

  _subscribe() {
    if (this._handle) return;
    this._handle = this._client.subscribe('/rosout', 'rcl_interfaces/msg/Log', (m) => {
      if (this._paused) return;
      const lv = Number(m.data?.level ?? 0);
      this._logs.unshift({
        t: new Date().toISOString().slice(11, 23),
        level: lv,
        name: String(m.data?.name || ''),
        msg: String(m.data?.msg || ''),
      });
      if (this._logs.length > 500) this._logs.length = 500;
      this._scheduleRender();
    }, { dispatchMode: 'latest' });
  }

  _scheduleRender() {
    if (this._pendingRender) return;
    this._pendingRender = true;
    setTimeout(() => {
      this._pendingRender = false;
      this._render();
    }, 500);
  }

  _render() {
    const min = parseInt($('roslog-level').value, 10) || 10;
    const rows = this._logs.filter((l) => l.level >= min);
    $('roslog-count').textContent = `${rows.length}/${this._logs.length} 条`;
    const list = $('roslog-list');
    list.innerHTML = rows.slice(0, 120).map((l) =>
      `<div><span class="t">${l.t}</span>` +
      `<span class="${logLevelClass(l.level)}" style="font-weight:600">${logLevelName(l.level)}</span>` +
      `${l.name ? ' ' + escapeHtml(l.name.split('/').pop()) + ':' : ''} ${escapeHtml(l.msg)}</div>`)
      .join('') || '<div style="color:var(--muted)">等待 /rosout…（节点打日志即出现）</div>';
  }

  snapshot() {
    return {
      paused: this._paused,
      minLevel: parseInt($('roslog-level').value, 10) || 10,
      total: this._logs.length,
      recent: this._logs.slice(0, 20),
    };
  }
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
  })[c]);
}

let _inited = false;

/** 初始化诊断与日志面板（幂等；连接后按开关订阅） */
export function initMonPanels() {
  if (_inited) return;
  _inited = true;
  const diag = new DiagnosticsPanel();
  const roslog = new RosLogPanel();
  window.__rwl_diag = () => diag.snapshot();
  window.__rwl_roslog = () => roslog.snapshot();
}
