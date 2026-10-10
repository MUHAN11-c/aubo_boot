// joint-chart.js — 关节角实时监测曲线（轻量 Canvas 2D，零外部依赖）
// 用法:
//   import { createJointChartController } from '../components/joint-chart.js';
//   const chart = createJointChartController({ getById: $, maxSamples: 280 });
//   chart.pushSample(jointNames, positions);

function createJointChartController(opts) {
    const options = opts || {};
    const $ = options.getById || (id => document.getElementById(id));
    const maxSamples = Math.max(60, Number(options.maxSamples) || 280);
    const lineColors = (Array.isArray(options.lineColors) && options.lineColors.length)
        ? options.lineColors.slice()
        : ['#4f6f8a', '#16a34a', '#d97706', '#db2777', '#7c3aed', '#0d9488'];
    const canvasId = options.canvasId || 'joint-chart';
    const legendId = options.legendId || 'joint-legend';

    let state = { names: [], series: [] };
    let drawRaf = null;
    let resizeObs = null;

    function canvasEl() { return $(canvasId); }
    function legendEl() { return document.getElementById(legendId); }

    // ── CSS 变量缓存 ──────────────────────────────────────
    let cssCache = null;
    let cssCacheTs = 0;
    function cssVars() {
        const now = Date.now();
        if (cssCache && now - cssCacheTs < 3000) return cssCache;
        const s = getComputedStyle(document.documentElement);
        cssCache = {
            bg:       s.getPropertyValue('--chart-bg').trim()      || '#f5f3f0',
            grid:     s.getPropertyValue('--chart-grid').trim()    || '#e8e4de',
            axis:     s.getPropertyValue('--chart-axis').trim()    || '#d5d0c8',
            text:     s.getPropertyValue('--chart-text').trim()    || '#8c857b',
            textBold: s.getPropertyValue('--chart-text-bold').trim() || '#5d564e',
        };
        cssCacheTs = now;
        return cssCache;
    }

    // ── 自适应面板高度 ────────────────────────────────────
    function fitCanvas(canvas) {
        const rect = canvas.getBoundingClientRect();
        let w = Math.max(300, Math.floor(rect.width) || 640);
        let h = Math.max(180, Math.floor(rect.height) || 240);
        const panel = canvas.closest('.panel--chart');
        if (panel && panel.clientHeight > 0) {
            const ps = getComputedStyle(panel);
            w = Math.max(280, Math.floor(panel.clientWidth
                - (parseFloat(ps.paddingLeft) || 0)
                - (parseFloat(ps.paddingRight) || 0)));
            let used = (parseFloat(ps.paddingTop) || 0) + (parseFloat(ps.paddingBottom) || 0);
            const h2 = panel.querySelector(':scope > h2');
            const leg = panel.querySelector(':scope > .legend-joint');
            if (h2) used += h2.offsetHeight + 4;
            if (leg) used += leg.offsetHeight + 4;
            h = Math.max(140, Math.floor(panel.clientHeight - used));
        }
        const dpr = Math.min(typeof devicePixelRatio !== 'undefined' ? devicePixelRatio : 1, 2);
        canvas.style.width  = `${w}px`;
        canvas.style.height = `${h}px`;
        canvas.width  = Math.floor(w * dpr);
        canvas.height = Math.floor(h * dpr);
        return { w, h, dpr };
    }

    // ── 绘制 ──────────────────────────────────────────────
    function draw() {
        const canvas = canvasEl();
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        const { w, h, dpr } = fitCanvas(canvas);
        const vars = cssVars();
        const pad = { t: 16, r: 12, b: 24, l: 50 };
        const pw = w - pad.l - pad.r;
        const ph = h - pad.t - pad.b;
        if (pw <= 0 || ph <= 0) return;

        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

        // 背景
        ctx.fillStyle = vars.bg;
        ctx.fillRect(0, 0, w, h);

        // 边框
        ctx.strokeStyle = vars.axis;
        ctx.lineWidth = 1;
        ctx.strokeRect(0.5, 0.5, w - 1, h - 1);

        const series = state.series;
        if (!series.some(a => a.length > 0)) {
            ctx.fillStyle = vars.text;
            ctx.font = '12px ui-monospace, monospace';
            ctx.textAlign = 'center';
            ctx.fillText('-- 等待关节数据 --', w / 2, h / 2);
            return;
        }

        // Y 范围
        let yMin = Infinity, yMax = -Infinity;
        for (const arr of series) {
            for (const v of arr) {
                if (typeof v === 'number' && isFinite(v)) {
                    if (v < yMin) yMin = v; if (v > yMax) yMax = v;
                }
            }
        }
        if (!isFinite(yMin) || !isFinite(yMax)) return;
        if (yMin === yMax) { yMin -= 1; yMax += 1; }
        const yPad = Math.max((yMax - yMin) * 0.08, 0.05);
        yMin -= yPad; yMax += yPad;
        const yr = yMax - yMin;

        const xPos = (t, len) => pad.l + (t / Math.max(1, len - 1)) * pw;
        const yPos = (v)    => pad.t + ph * (1 - (v - yMin) / yr);

        // 水平网格
        ctx.strokeStyle = vars.grid;
        ctx.lineWidth = 0.5;
        const rows = 4;
        for (let i = 0; i <= rows; i++) {
            const y = pad.t + (ph * i) / rows;
            ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(pad.l + pw, y); ctx.stroke();
        }

        // Y 轴刻度
        ctx.fillStyle = vars.text;
        ctx.font = '10px ui-monospace, monospace';
        ctx.textAlign = 'right';
        ctx.textBaseline = 'middle';
        for (let i = 0; i <= rows; i++) {
            ctx.fillText((yMax - (yr * i) / rows).toFixed(2), pad.l - 6, pad.t + (ph * i) / rows);
        }

        // 裁剪
        ctx.save();
        ctx.beginPath(); ctx.rect(pad.l, pad.t, pw, ph); ctx.clip();

        // 折线
        for (let j = 0; j < series.length; j++) {
            const arr = series[j];
            if (arr.length < 2) continue;
            const color = lineColors[j % lineColors.length];
            const len = arr.length;
            ctx.strokeStyle = color;
            ctx.lineWidth = 1.6;
            ctx.lineJoin = 'round';
            ctx.lineCap = 'round';
            ctx.beginPath();
            for (let t = 0; t < len; t++) {
                const x = xPos(t, len), y = yPos(arr[t]);
                t === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
            }
            ctx.stroke();
        }

        ctx.restore();

        // X 轴标签
        ctx.fillStyle = vars.text;
        ctx.font = '10px ui-monospace, monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText('samples', pad.l + pw / 2, h - pad.b + 6);
    }

    function scheduleDraw() {
        if (drawRaf != null) return;
        drawRaf = requestAnimationFrame(() => { drawRaf = null; draw(); });
    }

    // ── Legend ─────────────────────────────────────────────
    function updateLegend() {
        const leg = legendEl();
        if (!leg) return;
        leg.replaceChildren();
        if (!state.names.length) { leg.setAttribute('aria-hidden', 'true'); return; }
        state.names.forEach((n, i) => {
            const row = document.createElement('span');
            row.className = 'legend-j';
            const sw = document.createElement('span');
            sw.className = 'legend-j-swatch';
            sw.style.background = lineColors[i % lineColors.length];
            const lab = document.createElement('span');
            lab.className = 'legend-j-name';
            lab.textContent = n;
            row.appendChild(sw); row.appendChild(lab);
            leg.appendChild(row);
        });
        leg.setAttribute('aria-hidden', 'false');
    }

    // ── API ───────────────────────────────────────────────
    return {
        reset() {
            state = { names: [], series: [] };
            if (drawRaf != null) { cancelAnimationFrame(drawRaf); drawRaf = null; }
            const leg = legendEl();
            if (leg) { leg.replaceChildren(); leg.setAttribute('aria-hidden', 'true'); }
            draw();
        },

        pushSample(rawNames, positions) {
            const n = Math.max(rawNames.length, positions.length);
            if (n === 0) return;
            const normNames = rawNames.map((rn, i) =>
                rn != null && rn !== '' ? String(rn) : `joint_${i}`);
            if (state.series.length !== n || state.names.join('\0') !== normNames.join('\0')) {
                state.names = normNames.slice();
                state.series = Array.from({ length: n }, () => []);
            }
            for (let i = 0; i < n; i++) {
                const v = Number(positions[i]);
                if (!isFinite(v)) continue;
                const row = state.series[i];
                row.push(v);
                while (row.length > maxSamples) row.shift();
            }
            updateLegend();
            scheduleDraw();
        },

        observeResize() {
            const canvas = canvasEl();
            if (!canvas) return;
            if (typeof ResizeObserver !== 'undefined' && canvas.parentElement) {
                if (resizeObs) resizeObs.disconnect();
                resizeObs = new ResizeObserver(() => scheduleDraw());
                resizeObs.observe(canvas.parentElement);
            }
            scheduleDraw();
        },

        getState() { return state; },
    };
}

export { createJointChartController };
