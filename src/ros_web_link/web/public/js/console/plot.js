// console/plot.js — rqt_plot 风格字段绘图器
//
// 订阅任意话题并按字段路径抽取数值（如 position[0] / ranges[90] / range.range /
// pose.pose.position.x），滚动折线 + 图例 + 自适应量程。最多 4 条序列，
// 每 500ms 采样最新值（rqt_plot 语义），窗口 240 点。
// 快照 window.__rwl_plot()。

import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';

const $ = (id) => document.getElementById(id);
const COLORS = ['#33d6b0', '#5fa8ff', '#f2a541', '#ef5a4d'];
const MAX_SERIES = 4;
let WINDOW = 120;   // 采样窗（500ms × 120 = 60s；下拉可调 60/120/600）

/** 计算字段后缀解析：`ranges_computed_min` → {arrayPath:'ranges', agg:'min'} */
function parseComputed(path) {
  const idx = path.lastIndexOf('_computed_');
  if (idx < 1) return null;
  return { arrayPath: path.slice(0, idx), agg: path.slice(idx + '_computed_'.length) };
}

/** 解析到数组（供计算字段） */
function resolveArray(data, path) {
  let v = data;
  for (const seg of path.replace(/\]/g, '').split(/\.|\[/)) {
    if (!seg) continue;
    v = v?.[seg];
    if (v === undefined || v === null) return null;
  }
  if (Array.isArray(v)) return v;
  if (v && typeof v.length === 'number' && typeof v !== 'string'
    && !(v instanceof Uint8Array) && !(v instanceof Float32Array)) return v;
  return Array.isArray(v) ? v : null;
}

/** 字段路径取数：a.b[2].c → 数值（BigInt→Number；非数值返回 NaN）。
 * 计算字段（ChartPanel 同构）：path 数组路径 → min/max/avg 聚合 */
function extract(data, path) {
  const computed = parseComputed(path);
  if (computed && (computed.agg === 'min' || computed.agg === 'max'
    || computed.agg === 'avg')) {
    const arr = resolveArray(data, computed.arrayPath);
    if (!arr) return NaN;
    const nums = [];
    for (const x of arr) {
      const n = Number(x);
      if (Number.isFinite(n)) nums.push(n);
    }
    if (!nums.length) return NaN;
    if (computed.agg === 'min') return Math.min(...nums);
    if (computed.agg === 'max') return Math.max(...nums);
    let sum = 0;
    for (const n of nums) sum += n;
    return sum / nums.length;
  }
  let v = data;
  for (const seg of path.replace(/\]/g, '').split(/\.|\[/)) {
    if (!seg) continue;
    v = v?.[seg];
    if (v === undefined || v === null) return NaN;
  }
  if (typeof v === 'bigint') return Number(v);
  if (typeof v === 'number') return v;
  const n = Number(v);
  return Number.isFinite(n) ? n : NaN;
}

/** 活体解析字段目录（depth=1；数值数组→[0] 元素 + min/max/avg 计算字段） */
function parseFields(data, maxDepth = 1, prefix = '') {
  const out = [];
  if (!data || typeof data !== 'object') return out;
  for (const [key, val] of Object.entries(data)) {
    const path = prefix ? prefix + '.' + key : key;
    if (typeof val === 'number' || typeof val === 'bigint' || typeof val === 'boolean') {
      out.push({ name: path, path, type: 'number' });
    } else if (val && typeof val.length === 'number' && typeof val !== 'string') {
      out.push({ name: path + '[0]', path: path + '[0]', type: 'number' });
      out.push({ name: path + ' min', path: path + '_computed_min', type: 'computed' });
      out.push({ name: path + ' max', path: path + '_computed_max', type: 'computed' });
      out.push({ name: path + ' avg', path: path + '_computed_avg', type: 'computed' });
    } else if (maxDepth > 0 && typeof val === 'object') {
      const sub = parseFields(val, maxDepth - 1, path);
      for (const f of sub) out.push(f);
    }
  }
  return out;
}

export { extract, resolveArray, parseFields, parseComputed };

class FieldPlot {
  constructor() {
    this.series = [];          // {topic, field, color, last, buf[], hzEma, hzTick}
    this.paused = false;
    this._handles = new Map(); // topic → handle（多序列共享订阅）
    this._client = foxgloveRosClient();
    this._client.onStateChange((s) => {
      if (s === 'connected') this._resubscribeAll();
    });
    $('btn-plot-add').addEventListener('click', () => this.add());
    $('btn-plot-clear').addEventListener('click', () => this.clear());
    $('plot-field').addEventListener('keydown', (e) => { if (e.key === 'Enter') this.add(); });
    $('btn-plot-pause').addEventListener('click', () => {
      this.paused = !this.paused;
      $('btn-plot-pause').textContent = this.paused ? '继续' : '暂停';
      $('btn-plot-pause').classList.toggle('active-tool', this.paused);
    });
    $('plot-window').addEventListener('change', () => {
      WINDOW = parseInt($('plot-window').value, 10);
      for (const s of this.series) {
        while (s.buf.length > WINDOW) s.buf.shift();
      }
    });
    // 话题候选
    setInterval(() => {
      const chans = this._client?.player?.channels;
      if (!chans) return;
      $('plot-topics').innerHTML = [...chans.keys()].map((t) => `<option value="${t}">`).join('');
    }, 2000);
    // 字段 datalist：选话题后一次订阅活体解析字段目录（ChartPanel 模式）
    this._fieldCache = new Map();  // topic → fields[]
    let fieldProbe = null;
    $('plot-topic').addEventListener('change', () => {
      const topic = $('plot-topic').value.trim();
      if (!topic || !this._client.isConnected) return;
      if (this._fieldCache.has(topic)) { this._fillFieldList(topic); return; }
      fieldProbe?.cancel?.();
      fieldProbe = this._client.subscribe(topic, '', (m) => {
        if (this._fieldCache.has(topic)) return;  // 首帧即解析
        const fields = parseFields(m.data, 1);
        if (fields.length) {
          this._fieldCache.set(topic, fields);
          this._fillFieldList(topic);
          fieldProbe?.cancel?.();
        }
      }, { dispatchMode: 'latest' });
    });
    setInterval(() => this._tick(), 500);
    setInterval(() => this._draw(), 500);
    window.__rwl_plot = () => JSON.parse(JSON.stringify(
      this.series.map((s) => ({
        topic: s.topic, field: s.field, last: s.last,
        samples: s.buf.length,
        min: s.buf.length ? Math.min(...s.buf) : null,
        max: s.buf.length ? Math.max(...s.buf) : null,
      }))));
  }

  _fillFieldList(topic) {
    const fields = this._fieldCache.get(topic) || [];
    const dl = document.getElementById('plot-fields');
    if (!dl) {
      const d = document.createElement('datalist');
      d.id = 'plot-fields';
      document.body.appendChild(d);
    }
    document.getElementById('plot-fields').innerHTML =
      fields.map((f) => `<option value="${f.path}">${f.name}</option>`).join('');
  }

  add() {
    if (!this._client.isConnected) return alert('先连接');
    if (this.series.length >= MAX_SERIES) return alert(`最多 ${MAX_SERIES} 条序列`);
    const topic = $('plot-topic').value.trim();
    const field = $('plot-field').value.trim();
    if (!topic || !field) return;
    if (this.series.some((s) => s.topic === topic && s.field === field)) return;
    this.series.push({
      topic, field, color: COLORS[this.series.length % COLORS.length],
      last: null, buf: [], hzEma: 0, hzTick: 0,
    });
    this._resubscribeAll();
    this._legend();
  }

  clear() {
    this.series = [];
    this._resubscribeAll();
    this._legend();
    this._draw(true);
  }

  _resubscribeAll() {
    for (const h of this._handles.values()) { try { h.cancel(); } catch (_) {} }
    this._handles.clear();
    if (!this._client.isConnected) return;
    for (const topic of new Set(this.series.map((s) => s.topic))) {
      this._handles.set(topic, this._client.subscribe(topic, '', (m) => {
        for (const s of this.series) {
          if (s.topic === topic) {
            // EMA Hz（α=0.3，ChartPanel 模式）
            if (s.hzTick) {
              const dt = (Date.now() - s.hzTick) / 1000;
              if (dt > 0.01) s.hzEma = s.hzEma * 0.7 + (1 / dt) * 0.3;
            }
            s.hzTick = Date.now();
            s.last = extract(m.data, s.field);
          }
        }
      }, { dispatchMode: 'latest' }));
    }
  }

  _tick() {
    if (this.paused || !this.series.length) return;
    for (const s of this.series) {
      s.buf.push(s.last);
      if (s.buf.length > WINDOW) s.buf.shift();
    }
  }

  _legend() {
    $('plot-legend').innerHTML = this.series.map((s, i) =>
      `<span class="pl-item"><i style="background:${s.color}"></i>` +
      `${s.topic.split('/').pop() || s.topic}·${s.field}` +
      ` <b>${s.last != null && Number.isFinite(s.last) ? s.last.toFixed(3) : '—'}</b>` +
      ` <span style="color:var(--muted)">${s.hzEma.toFixed(1)}Hz</span>` +
      ` <span class="chip-btn" data-rm="${i}">✕</span></span>`).join(' ');
    $('plot-legend').querySelectorAll('[data-rm]').forEach((el) => {
      el.onclick = () => { this.series.splice(+el.dataset.rm, 1); this._resubscribeAll(); this._legend(); };
    });
  }

  _draw(clear = false) {
    const cv = $('plot-canvas');
    const ctx = cv.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth || 600, h = 140;
    if (cv.width !== w * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#0c0e12';
    ctx.fillRect(0, 0, w, h);
    // 网格
    ctx.strokeStyle = '#1a2029';
    for (let i = 1; i < 4; i++) {
      ctx.beginPath(); ctx.moveTo(0, h * i / 4); ctx.lineTo(w, h * i / 4); ctx.stroke();
    }
    if (clear || !this.series.length) {
      ctx.fillStyle = '#8b93a7';
      ctx.font = '12px ui-monospace, monospace';
      ctx.fillText('添加 话题+字段 开始绘图（如 /rwl_viz/joint_states + position[0]）', 12, h / 2);
      return;
    }
    let min = Infinity, max = -Infinity;
    for (const s of this.series) {
      for (const v of s.buf) {
        if (Number.isFinite(v)) { if (v < min) min = v; if (v > max) max = v; }
      }
    }
    if (!Number.isFinite(min)) return;
    if (max - min < 1e-9) { min -= 0.5; max += 0.5; }
    const pad = (max - min) * 0.08;
    min -= pad; max += pad;
    const y = (v) => h - 6 - ((v - min) / (max - min)) * (h - 16);
    // 零线
    if (min < 0 && max > 0) {
      ctx.strokeStyle = '#2b3345';
      ctx.beginPath(); ctx.moveTo(0, y(0)); ctx.lineTo(w, y(0)); ctx.stroke();
    }
    for (const s of this.series) {
      if (s.buf.length < 2) continue;
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      s.buf.forEach((v, i) => {
        if (!Number.isFinite(v)) return;
        const x = (i / (WINDOW - 1)) * w;
        if (i === 0 || !Number.isFinite(s.buf[i - 1])) ctx.moveTo(x, y(v));
        else ctx.lineTo(x, y(v));
      });
      ctx.stroke();
    }
    // 量程标注
    ctx.fillStyle = '#8b93a7';
    ctx.font = '10px ui-monospace, monospace';
    ctx.fillText(max.toFixed(3), 4, 12);
    ctx.fillText(min.toFixed(3), 4, h - 4);
    this._legend();
  }
}

let _plot = null;

export function initPlot() {
  if (_plot) return _plot;
  _plot = new FieldPlot();
  return _plot;
}
