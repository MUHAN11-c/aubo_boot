// console/topics.js — 话题卡（RVIZ-RQT-VISUAL 树模式）
//
// 行 = [checkbox 订阅] [topic] [类型 tag] [Hz EMA 徽章] [KB/s] [可视化按钮]。
// 「可视化」按 schemaName 映射 Displays 话题并启用（右键 Visualize 的等价交互）；
// checkbox 直接建/退订阅（复用监控差分）。Hz EMA α=0.3（ChartPanel 模式）。

import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';

const $ = (id) => document.getElementById(id);

// schemaName 后缀 → Displays 类型（可视化映射；v4 实例化语义）
const TYPE_TO_DISPLAY = {
  'PointCloud2': 'cloud', 'LaserScan': 'scan3d', 'Image': 'image',
  'CompressedImage': 'imagec', 'MarkerArray': 'marker', 'Marker': 'marker',
  'Path': 'path', 'PoseArray': 'poses', 'OccupancyGrid': 'map',
  'TFMessage': 'tf', 'Odometry': 'odom', 'Range': 'range',
  'WrenchStamped': 'wrench', 'PoseWithCovarianceStamped': 'posecov',
};

class TopicCard {
  constructor() {
    this._client = foxgloveRosClient();
    this._ema = new Map();     // topic → {hz, kbps}
    this._last = new Map();    // topic → {msgs, bytes}
    this._handles = new Map(); // topic → 订阅句柄
    this._prevGraphSize = -1;
    setInterval(() => this._tick(), 1000);
    this._client.onStateChange((s) => {
      if (s === 'connected') this._render(true);
    });
  }

  setVizPanel(panel) { this._panel = panel; }

  subscribed(topic) { return this._handles.has(topic); }

  toggle(topic, on) {
    if (on && !this._handles.has(topic)) {
      // raw 模式：本卡只做 Hz/流量统计，不做解码——大消息（图像/点云）
      // 全量 CDR 解码纯属浪费（player._dispatchRaw 跳过解码热路径）
      this._handles.set(topic, this._client.subscribe(topic, '', () => {},
        { dispatchMode: 'raw' }));
    } else if (!on && this._handles.has(topic)) {
      try { this._handles.get(topic).cancel(); } catch (_) {}
      this._handles.delete(topic);
      this._ema.delete(topic);
    }
    this._render();
  }

  /** 「可视化」：类型映射 Displays 话题并启用（v4：启用既有实例或新建） */
  visualize(topic) {
    const ch = this._client.player?.channels?.get(topic);
    const short = (ch?.schemaName || '').split('/').pop();
    const key = TYPE_TO_DISPLAY[short];
    if (!key || !this._panel) return false;
    return this._panel.addOrEnable(key, topic);
  }

  _tick() {
    const subs = this._client.player?._topicSubs;
    if (subs) {
      for (const [topic, s] of subs) {
        const prev = this._last.get(topic) || { msgs: 0, bytes: 0 };
        const hz = s.stats.messages - prev.msgs;
        const kbps = (s.stats.bytes - prev.bytes) / 1024;
        this._last.set(topic, { messages: s.stats.messages, bytes: s.stats.bytes });
        const e = this._ema.get(topic) || { hz: 0, kbps: 0 };
        e.hz = e.hz * 0.7 + hz * 0.3;
        e.kbps = e.kbps * 0.7 + kbps * 0.3;
        this._ema.set(topic, e);
      }
    }
    this._render();
  }

  _render(force = false) {
    if (!this._client.isConnected) return;
    const snap = (() => {
      try { return this._client.getGraph(); } catch (_) { return null; }
    })();
    if (!snap) return;
    if (!force && snap.topics.length === this._prevGraphSize && !this._dirty) {
      // 只刷新数值单元格
      this._updateNumbers();
      return;
    }
    this._prevGraphSize = snap.topics.length;
    this._dirty = false;
    const tb = $('topics-table').querySelector('tbody');
    const known = new Set([
      ...snap.topics,
      ...[...this._client.player?.channels?.keys() || []],
    ]);
    const rows = [];
    for (const t of [...known].sort()) {
      const ch = this._client.player?.channels?.get(t);
      const short = (ch?.schemaName || '').split('/').pop();
      const canViz = !!TYPE_TO_DISPLAY[short];
      rows.push(`<tr data-topic="${t}">
        <td><input type="checkbox" data-sub ${this.subscribed(t) ? 'checked' : ''}></td>
        <td style="max-width:150px;overflow:hidden;text-overflow:ellipsis;" title="${t}">${t}</td>
        <td>${short ? `<span class="type-tag">${short}</span>` : '<span style="color:var(--muted)">—</span>'}</td>
        <td class="num" data-cell="hz"></td>
        <td class="num" data-cell="kb"></td>
        <td>${canViz ? `<span class="chip-btn" data-viz>可视化</span>` : ''}</td>
      </tr>`);
    }
    tb.innerHTML = rows.join('');
    $('topics-count').textContent = `${rows.length} 话题`;
    tb.querySelectorAll('tr').forEach((tr) => {
      const t = tr.dataset.topic;
      tr.querySelector('[data-sub]').addEventListener('change', (e) => {
        this.toggle(t, e.target.checked);
      });
      const viz = tr.querySelector('[data-viz]');
      if (viz) viz.addEventListener('click', () => this.visualize(t));
    });
    this._updateNumbers();
  }

  _updateNumbers() {
    const tb = $('topics-table');
    for (const tr of tb.querySelectorAll('tr')) {
      const t = tr.dataset.topic;
      const e = this._ema.get(t);
      const hzCell = tr.querySelector('[data-cell="hz"]');
      const kbCell = tr.querySelector('[data-cell="kb"]');
      if (e && e.hz > 0.05) {
        hzCell.innerHTML = `<span class="hz-badge ${e.hz >= 5 ? 'hi' : 'md'}">${e.hz.toFixed(1)}</span>`;
        kbCell.textContent = e.kbps >= 0.05 ? e.kbps.toFixed(2) : '·';
      } else if (this.subscribed(t)) {
        hzCell.innerHTML = '<span class="hz-badge lo">…</span>';
        kbCell.textContent = '·';
      } else {
        hzCell.textContent = ''; kbCell.textContent = '';
      }
    }
  }

  snapshot() {
    return [...this._ema.entries()].map(([topic, e]) => ({
      topic, hz: +e.hz.toFixed(2), kbps: +e.kbps.toFixed(3),
      subscribed: this.subscribed(topic),
    }));
  }
}

let _card = null;

export function initTopics(panel) {
  if (_card) { _card.setVizPanel(panel); return _card; }
  _card = new TopicCard();
  _card.setVizPanel(panel);
  window.__rwl_topics = () => _card.snapshot();
  return _card;
}
