// console/topo.js — 节点/话题拓扑图（RVIZ-RQT-VISUAL NodeTopicGraph/GraphLayout/
// GraphInteraction 同构移植，纯 vanilla SVG；粒子直接改 cx/cy 属性不重建）
//
// 数据：connectionGraph（publishedTopics/subscribedTopics/advertisedServices）
// + 外部 Hz 表（EMA）。布局：层次（源节点 → 话题层(>6 切 4 列) → 订阅者）。
// 交互：滚轮绕鼠标缩放 / 空白拖拽平移 / fit；话题点击回调（订阅/可视化入口）。

const SVG_NS = 'http://www.w3.org/2000/svg';

// 消息类型色板（GraphLayout.js:796-870 蒸馏，浅色适配白主题）
const TYPE_COLORS = {
  PointCloud2: '#bde7bd', LaserScan: '#a6d4fa', Image: '#f5cba7',
  CompressedImage: '#f5cba7', OccupancyGrid: '#d1c4e9', Odometry: '#c8e6c9',
  Twist: '#ce93d8', TFMessage: '#d3d3d3', String: '#fff9c4',
  JointState: '#b2dfdb', Marker: '#ffcdd2', MarkerArray: '#ffcdd2',
  Imu: '#e1bee7', Path: '#c5cae9', PoseArray: '#c5cae9',
  WrenchStamped: '#d7ccc8', Range: '#ffe0b2', DiagnosticArray: '#ffccbc',
};

function typeColor(schemaName) {
  const short = (schemaName || '').split('/').pop();
  return TYPE_COLORS[short] || '#e8ebee';
}

function el(tag, attrs = {}, parent = null) {
  const node = document.createElementNS(SVG_NS, tag);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  if (parent) parent.appendChild(node);
  return node;
}

class TopologyGraph {
  /**
   * @param {{container: Object, onTopicClick: function, hzOf: function}} opts
   * hzOf(topic) → EMA Hz 或 null
   */
  constructor(opts) {
    this.opts = opts;
    this.nodes = [];      // {id, label, x, y, w, h, kind: 'node'|'topic'|'service'}
    this.connections = []; // {id, from, to, type: 'pub'|'sub'|'srv', path}
    this.transform = { tx: 0, ty: 0, scale: 1 };
    this._particles = [];
    let pseed = 42;  // 确定性伪随机（粒子相位/大小，非加密用途）
    const punit = () => { pseed = (pseed * 1103515245 + 12345) & 0x7fffffff; return pseed / 0x7fffffff; };
    this._raf = 0;
    this._lastT = 0;
    this._fitted = false;
    this.mode = 'all';
    this._lastGraph = null;
    this._filter = '';
    this._nodeEls = new Map();
    this._labelEls = new Map();
    this._hzEls = new Map();
    this._edgeEls = [];
    this._userInteracted = false;
    this._build();
    if (typeof ResizeObserver !== 'undefined') {
      this._ro = new ResizeObserver(() => {
        const w = this.opts.container.clientWidth;
        const h = this.opts.container.clientHeight;
        if (this.nodes.length && w > 0 && h > 0 && !this._userInteracted) this.fit();
      });
      this._ro.observe(this.opts.container);
    }
    this._bindEvents();
    this._startParticles();
  }

  _build() {
    const svg = el('svg', { class: 'topo-svg' });
    this.svg = svg;
    this.opts.container.appendChild(svg);

    const defs = el('defs', {}, svg);
    const marker = el('marker', {
      id: 'topo-arrow', markerWidth: 10, markerHeight: 10, refX: 8, refY: 5,
      orient: 'auto', markerUnits: 'strokeWidth',
    }, defs);
    el('path', { d: 'M0,2 L0,8 L8,5 z', fill: '#8a9490' }, marker);

    this.root = el('g', { class: 'topo-content' }, svg);
    this.layers = {
      connections: el('g', {}, this.root),
      shapes: el('g', {}, this.root),
      labels: el('g', {}, this.root),
      particles: el('g', { 'pointer-events': 'none' }, this.root),
    };

    const bar = document.createElement('div');
    bar.className = 'topo-toolbar';
    bar.innerHTML = '<button data-z="in">＋</button><button data-z="out">－</button>' +
      '<button data-z="fit">适配</button>';
    this.opts.container.appendChild(bar);
    bar.querySelector('[data-z="in"]').onclick = () => this._zoomBy(1.2);
    bar.querySelector('[data-z="out"]').onclick = () => this._zoomBy(0.8);
    bar.querySelector('[data-z="fit"]').onclick = () => this.fit();

    const hint = document.createElement('div');
    hint.className = 'topo-hint';
    hint.textContent = '拖拽平移 · 滚轮缩放 · 点击话题';
    this.opts.container.appendChild(hint);
  }

  // ── 数据 → 图 ─────────────────────────────────────────────────────────

  /** rqt_graph 模式：all=节点+话题/服务；nodes=仅节点（publisher→subscriber 直连） */
  setMode(mode) {
    const next = mode === 'nodes' ? 'nodes' : 'all';
    if (next === this.mode) return;
    this.mode = next;
    this._signature = '';
    if (this._lastGraph) this.update(this._lastGraph);
  }

  /**
   * @param {{publishedTopics: Object, subscribedTopics: Object,
   *          advertisedServices: Object}} graph connectionGraph 快照
   */
  update(graph) {
    if (!graph) return;
    this._lastGraph = graph;
    const stable = (obj) => Object.keys(obj || {}).sort()
      .map((k) => [k, [...(obj[k] || [])].sort()]);
    const sig = JSON.stringify({
      p: stable(graph.publishedTopics),
      s: stable(graph.subscribedTopics),
      a: stable(graph.advertisedServices),
      t: [...new Set([...Object.keys(graph.publishedTopics || {}),
        ...Object.keys(graph.subscribedTopics || {})])].sort()
        .map((t) => [t, this.opts.schemaOf?.(t) || '']),
      mode: this.mode,
    });
    if (sig === this._signature) {
      this._updateHz();
      return;
    }
    this._signature = sig;
    const nodeSet = new Map();  // nodeName → {pubs:Set, subs:Set, srvs:Set}
    const addNode = (name) => {
      if (!nodeSet.has(name)) nodeSet.set(name, { pubs: new Set(), subs: new Set(), srvs: new Set() });
      return nodeSet.get(name);
    };
    for (const [topic, ids] of Object.entries(graph.publishedTopics || {})) {
      for (const n of ids) addNode(n).pubs.add(topic);
    }
    for (const [topic, ids] of Object.entries(graph.subscribedTopics || {})) {
      for (const n of ids) addNode(n).subs.add(topic);
    }
    for (const [svc, ids] of Object.entries(graph.advertisedServices || {})) {
      for (const n of ids) addNode(n).srvs.add(svc);
    }

    const topics = new Set([
      ...Object.keys(graph.publishedTopics || {}),
      ...Object.keys(graph.subscribedTopics || {}),
    ]);
    const services = new Set(Object.keys(graph.advertisedServices || {}));

    // rqt_graph "Nodes only"：隐藏话题/服务节点，把每个 topic 的
    // publisher→subscriber 直接连成边；节点按网格铺开，避免 100+ 话题针尖图。
    if (this.mode === 'nodes') {
      const list = [...nodeSet.keys()].sort();
      const cols = Math.max(3, Math.min(8, Math.ceil(Math.sqrt(list.length || 1))));
      const colW = 180, rowH = 74;
      this.nodes = list.map((id, i) => ({
        id, kind: 'node', x: 60 + (i % cols) * colW,
        y: 55 + Math.floor(i / cols) * rowH, w: 100, h: 42,
        label: id.split('/').pop() || id,
      }));
      const byId = new Map(this.nodes.map((n) => [n.id, n]));
      this.connections = [];
      const seen = new Set();
      let ci = 0;
      for (const [topic, pubs] of Object.entries(graph.publishedTopics || {})) {
        const subs = graph.subscribedTopics?.[topic] || [];
        for (const p of pubs) {
          for (const q of subs) {
            if (p === q) continue;
            const key = p + '\u0000' + q;
            if (seen.has(key)) continue;
            seen.add(key);
            const from = byId.get(p), to = byId.get(q);
            if (from && to) this.connections.push({ id: 'c' + ci++, from, to, type: 'pub' });
          }
        }
      }
      this._render();
      if (!this._fitted) {
        this.fit();
        this._fitted = true;
      }
      return;
    }

    // 布局（层次简化版）：源节点层 → 话题层（>6 按 4 切行）→ 订阅者层
    const sources = [...nodeSet.entries()]
      .filter(([, d]) => d.pubs.size > d.subs.size).map(([n]) => n);
    const sinks = [...nodeSet.entries()]
      .filter(([, d]) => d.subs.size >= d.pubs.size && d.subs.size > 0)
      .map(([n]) => n).filter((n) => !sources.includes(n));
    const others = [...nodeSet.keys()].filter((n) => !sources.includes(n) && !sinks.includes(n));

    const NODE_W = 100, NODE_H = 42, TOPIC_W = 120, TOPIC_H = 26;
    const rows = [];
    const topicList = [...topics].sort();
    // rqt_graph 风格：列数随话题数自适应（3..7），避免固定 4 列在高密度图上
    // 横向过宽 / 纵向过深；fit 会按容器缩放。
    const topicCols = Math.max(3, Math.min(7, Math.ceil(Math.sqrt(topicList.length || 1))));
    const canvasW = Math.max(760, topicCols * 165);
    for (let i = 0; i < topicList.length; i += topicCols) {
      rows.push(topicList.slice(i, i + topicCols));
    }

    this.nodes = [];
    const place = (list, y, kind, w, h) => {
      const spacing = Math.max(w + 30, canvasW / Math.max(list.length, 1));
      list.forEach((id, i) => {
        this.nodes.push({
          id, kind, x: (i + 0.5) * spacing, y, w, h,
          label: id.split('/').pop() || id,
          schema: kind === 'topic' ? this.opts.schemaOf?.(id) : undefined,
        });
      });
    };
    place(sources.length ? sources : others.slice(0, 4), 40, 'node', NODE_W, NODE_H);
    rows.forEach((row, ri) => place(row, 130 + ri * 52, 'topic', TOPIC_W, TOPIC_H));
    const svcList = [...services].slice(0, 8);
    if (svcList.length) place(svcList, 130 + rows.length * 52 + 10, 'service', TOPIC_W, TOPIC_H);
    place(sinks, 130 + rows.length * 52 + (svcList.length ? 62 : 0) + 10, 'node', NODE_W, NODE_H);

    const byId = new Map(this.nodes.map((n) => [n.id, n]));
    this.connections = [];
    let ci = 0;
    for (const [name, data] of nodeSet) {
      const nd = byId.get(name);
      if (!nd) continue;
      for (const t of data.pubs) {
        const td = byId.get(t);
        if (td) this.connections.push({ id: 'c' + ci++, from: nd, to: td, type: 'pub' });
      }
      for (const t of data.subs) {
        const td = byId.get(t);
        if (td) this.connections.push({ id: 'c' + ci++, from: td, to: nd, type: 'sub' });
      }
      for (const s of data.srvs) {
        const sd = byId.get(s);
        if (sd) this.connections.push({ id: 'c' + ci++, from: nd, to: sd, type: 'srv' });
      }
    }
    this._render();
    if (!this._fitted) {
      this.fit();
      this._fitted = true;
    }
  }

  _render() {
    const { connections } = this;   // 数据数组（this.connections）
    const layers = this.layers;      // SVG 层 {connections, shapes, labels, particles}
    layers.connections.innerHTML = '';
    layers.shapes.innerHTML = '';
    layers.labels.innerHTML = '';
    layers.particles.innerHTML = '';
    this._particles = [];
    this._edgeEls = [];
    this._nodeEls = new Map();
    this._labelEls = new Map();
    this._hzEls = new Map();
    let pseed = 42;  // 确定性伪随机（粒子相位/大小，非加密用途）
    const punit = () => { pseed = (pseed * 1103515245 + 12345) & 0x7fffffff; return pseed / 0x7fffffff; };

    for (const c of connections) {
      const line = el('line', {
        x1: c.from.x, y1: c.from.y, x2: c.to.x, y2: c.to.y,
        stroke: c.type === 'pub' ? '#6b7a74' : '#9aa5a0',
        'stroke-width': c.type === 'pub' ? 1.4 : 1,
        opacity: c.type === 'pub' ? 0.7 : 0.5,
        'marker-end': 'url(#topo-arrow)',
      }, layers.connections);
      this._edgeEls.push({ c, el: line });
      for (let i = 0; i < 3; i++) {
        const p = el('circle', {
          r: 2 + punit() * 1.5, fill: c.type === 'pub' ? '#3e9c31' : '#2563c9',
          opacity: 0,
        }, layers.particles);
        this._particles.push({
          el: p, c, progress: (i * 0.33 + punit() * 0.1) % 1,
          speed: 0.5 + punit() * 0.5,
        });
      }
    }

    for (const n of this.nodes) {
      const g = el('g', {
        transform: `translate(${n.x},${n.y})`, cursor: 'pointer',
        'data-id': n.id, 'data-kind': n.kind,
      }, layers.shapes);
      this._nodeEls.set(n.id, g);
      const tip = el('title', {}, g);
      tip.textContent = `${n.id}\n${n.kind}`;
      g.addEventListener('mouseenter', () => this._highlight(n));
      g.addEventListener('mouseleave', () => this._clearHighlight());
      if (n.kind === 'node') {
        el('ellipse', {
          rx: n.w / 2, ry: n.h / 2, fill: 'var(--surface)',
          stroke: '#5a6661', 'stroke-width': 1.6,
        }, g);
      } else {
        el('rect', {
          x: -n.w / 2, y: -n.h / 2, width: n.w, height: n.h, rx: 4,
          fill: n.kind === 'topic' ? typeColor(n.schema) : '#eee0f5',
          stroke: n.kind === 'topic' ? '#67a06b' : '#9c74b8', 'stroke-width': 1,
        }, g);
      }
      const lg = el('g', { transform: `translate(${n.x},${n.y})` }, layers.labels);
      this._labelEls.set(n.id, lg);
      const label = el('text', {
        y: n.kind === 'node' ? 0 : 4, 'text-anchor': 'middle',
        'font-size': n.kind === 'node' ? 10.5 : 9.5,
        'font-weight': n.kind === 'node' ? 500 : 400,
        fill: 'var(--text)',
        'font-family': 'var(--mono)',
      }, lg);
      label.textContent = n.label.length > 16 ? n.label.slice(0, 15) + '…' : n.label;
      if (n.kind === 'topic') {
        const hzText = el('text', {
          y: 15, 'text-anchor': 'middle', 'font-size': 8,
          fill: 'var(--muted)', 'font-family': 'var(--mono)',
        }, lg);
        this._hzEls.set(n.id, hzText);
      }
      g.addEventListener('click', (e) => {
        e.stopPropagation();
        if (n.kind === 'topic') this.opts.onTopicClick?.(n.id);
      });
    }
    this._updateHz();
    this._applyFilter(this._filter);
  }

  _updateHz() {
    if (!this.opts.hzOf) return;
    for (const [id, txt] of this._hzEls) {
      const hz = this.opts.hzOf(id);
      if (hz != null && hz > 0.05) {
        txt.textContent = hz.toFixed(1) + 'Hz';
        txt.setAttribute('fill', hz >= 5 ? '#2e7d32' : hz >= 0.5 ? '#8a6d00' : 'var(--muted)');
      } else {
        txt.textContent = '';
      }
    }
  }

  /** rqt_graph 风格过滤：命中节点/边高亮，未命中淡化（不重建布局） */
  setFilter(q) {
    this._filter = (q || '').trim().toLowerCase();
    this._applyFilter(this._filter);
  }

  _applyFilter(q) {
    if (!q) {
      for (const g of this._nodeEls.values()) g.style.opacity = '1';
      for (const lg of this._labelEls.values()) lg.style.opacity = '1';
      for (const e of this._edgeEls) e.el.style.opacity = e.c.type === 'pub' ? '0.7' : '0.5';
      return;
    }
    const hit = new Set();
    for (const n of this.nodes) {
      const ok = (n.id || '').toLowerCase().includes(q)
        || (n.label || '').toLowerCase().includes(q);
      if (ok) hit.add(n.id);
      this._nodeEls.get(n.id).style.opacity = ok ? '1' : '0.16';
      this._labelEls.get(n.id).style.opacity = ok ? '1' : '0.16';
    }
    for (const e of this._edgeEls) {
      const ok = hit.has(e.c.from.id) || hit.has(e.c.to.id);
      e.el.style.opacity = ok ? '0.9' : '0.06';
    }
  }

  _highlight(n) {
    for (const g of this._nodeEls.values()) g.style.opacity = '0.18';
    for (const lg of this._labelEls.values()) lg.style.opacity = '0.18';
    const related = new Set([n.id]);
    for (const e of this._edgeEls) {
      const ok = e.c.from.id === n.id || e.c.to.id === n.id;
      e.el.style.opacity = ok ? '1' : '0.05';
      if (ok) { related.add(e.c.from.id); related.add(e.c.to.id); }
    }
    for (const id of related) {
      this._nodeEls.get(id).style.opacity = '1';
      this._labelEls.get(id).style.opacity = '1';
    }
  }

  _clearHighlight() {
    this._applyFilter(this._filter);
  }

  // ── 变换与交互（GraphInteraction 核心） ───────────────────────────────

  _applyTransform() {
    const { tx, ty, scale } = this.transform;
    this.root.setAttribute('transform', `translate(${tx},${ty}) scale(${scale})`);
  }

  fit() {
    this._userInteracted = false;
    const svg = this.svg;
    const W = svg.clientWidth || 600, H = svg.clientHeight || 300;
    if (!this.nodes.length) return;
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const n of this.nodes) {
      minX = Math.min(minX, n.x - n.w / 2); maxX = Math.max(maxX, n.x + n.w / 2);
      minY = Math.min(minY, n.y - n.h / 2); maxY = Math.max(maxY, n.y + n.h / 2);
    }
    const contentW = maxX - minX + 60, contentH = maxY - minY + 60;
    const scale = Math.min(1, Math.max(0.15,
      Math.min((W - 40) / contentW, (H - 40) / contentH)));
    this.transform.scale = scale;
    this.transform.tx = W / 2 - (minX + maxX) / 2 * scale;
    this.transform.ty = H / 2 - (minY + maxY) / 2 * scale;
    this._applyTransform();
  }

  _zoomBy(factor, cx = null, cy = null) {
    const svg = this.svg;
    const W = svg.clientWidth, H = svg.clientHeight;
    const x = cx ?? W / 2, y = cy ?? H / 2;
    const { tx, ty, scale } = this.transform;
    const newScale = Math.min(2, Math.max(0.1, scale * factor));
    const ratio = newScale / scale;
    this.transform = {
      scale: newScale,
      tx: x - (x - tx) * ratio,
      ty: y - (y - ty) * ratio,
    };
    this._applyTransform();
  }

  _bindEvents() {
    const svg = this.svg;
    svg.addEventListener('wheel', (e) => {
      e.preventDefault();
      const r = svg.getBoundingClientRect();
      this._userInteracted = true;
      this._zoomBy(e.deltaY > 0 ? 0.9 : 1.1, e.clientX - r.left, e.clientY - r.top);
    }, { passive: false });

    let panning = false, start = null, startT = null;
    svg.addEventListener('mousedown', (e) => {
      this._userInteracted = true;
      panning = true;
      svg.classList.add('panning');
      start = { x: e.clientX, y: e.clientY };
      startT = { ...this.transform };
    });
    window.addEventListener('mousemove', (e) => {
      if (!panning) return;
      this.transform.tx = startT.tx + (e.clientX - start.x);
      this.transform.ty = startT.ty + (e.clientY - start.y);
      this._applyTransform();
    });
    window.addEventListener('mouseup', () => {
      panning = false;
      svg.classList.remove('panning');
    });
  }

  _startParticles() {
    // 粒子动画节流至 ~33ms（原 RAF 60fps × 945 粒子 = 浏览器 CPU 饱和 → 3D 掉帧）
    const INTERVAL = 33;
    const tick = (now) => {
      this._raf = requestAnimationFrame(tick);
      if (now - (this._lastT || 0) < INTERVAL) return;
      const dt = Math.max(INTERVAL, now - (this._lastT || now));
      this._lastT = now;
      for (const p of this._particles) {
        p.progress = (p.progress + p.speed * dt / 3000) % 1;
        const t = p.progress;
        const x = p.c.from.x + (p.c.to.x - p.c.from.x) * t;
        const y = p.c.from.y + (p.c.to.y - p.c.from.y) * t;
        p.el.setAttribute('cx', x);
        p.el.setAttribute('cy', y);
        p.el.setAttribute('opacity',
          t < 0.1 ? t / 0.1 * 0.8 : t > 0.9 ? (1 - t) / 0.1 * 0.8 : 0.8);
      }
    };
    this._raf = requestAnimationFrame(tick);
  }

  destroy() {
    this._ro?.disconnect();
    cancelAnimationFrame(this._raf);
    this.svg.remove();
    this.opts.container.innerHTML = '';
  }
}

export { TopologyGraph, typeColor };
