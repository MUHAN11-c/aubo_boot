// joint-chart.js — Canvas 2D 关节角曲线组件（可复用，零外部依赖）
// 从 vision_grasp/joint_chart.js 提取，供 vision-grasp / latte 等各页共用喵~
//
// 用法:
//   import { createJointChartController } from '../components/joint-chart.js';
//   const chart = createJointChartController({ getById: $, maxSamples: 280 });
//   chart.pushSample(jointNames, positions);
//   chart.observeResize();
//   chart.reset();

function createJointChartController(opts) {
    const options = opts || {};
    const $ = options.getById || (id => document.getElementById(id));
    const maxSamples = Number(options.maxSamples) > 0 ? Number(options.maxSamples) : 280;
    const lineColors = Array.isArray(options.lineColors) && options.lineColors.length
        ? options.lineColors.slice()
        : ['#2563eb', '#16a34a', '#d97706', '#db2777', '#7c3aed', '#0d9488'];
    // 可自定义 canvas ID 和 legend ID
    const canvasId = options.canvasId || 'joint-chart';
    const legendId = options.legendId || 'joint-legend';

    let state = { names: [], series: [] };
    let drawRaf = null;
    let resizeObs = null;

    function jointLegendEl() {
        return document.getElementById(legendId);
    }

    function clampChartSizeToPanel(canvas, cssW, cssH) {
        const panel = canvas.closest('.panel--chart');
        if (!panel || panel.clientHeight <= 0) return { w: cssW, h: cssH };
        const pad = getComputedStyle(panel);
        const pl = parseFloat(pad.paddingLeft) || 0;
        const pr = parseFloat(pad.paddingRight) || 0;
        const pt = parseFloat(pad.paddingTop) || 0;
        const pb = parseFloat(pad.paddingBottom) || 0;
        let usedY = pt + pb;
        const h2 = panel.querySelector(':scope > h2');
        const leg = panel.querySelector(':scope > .legend-joint');
        if (h2) usedY += h2.offsetHeight;
        if (leg) usedY += leg.offsetHeight;
        const availH = Math.max(72, Math.floor(panel.clientHeight - usedY));
        const availW = Math.max(200, Math.floor(panel.clientWidth - pl - pr));
        return { w: Math.min(Math.max(200, cssW), availW), h: Math.min(Math.max(72, cssH), availH) };
    }

    function drawImmediate() {
        const canvas = $(canvasId);
        if (!canvas || !canvas.getContext) return;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;
        const rect = canvas.getBoundingClientRect();
        let cssW = Math.max(200, Math.floor(rect.width) || 640);
        let cssH = Math.max(160, Math.floor(rect.height) || 240);
        const capped = clampChartSizeToPanel(canvas, cssW, cssH);
        cssW = capped.w;
        cssH = capped.h;
        const dpr = typeof window !== 'undefined' && window.devicePixelRatio ? window.devicePixelRatio : 1;
        canvas.style.width = `${cssW}px`;
        canvas.style.height = `${cssH}px`;
        canvas.width = Math.floor(cssW * dpr);
        canvas.height = Math.floor(cssH * dpr);
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const W = cssW, H = cssH;
        const padL = 44, padR = 8, padT = 10, padB = 22;
        const plotW = W - padL - padR, plotH = H - padT - padB;

        const style = getComputedStyle(document.documentElement);
        const bg = style.getPropertyValue('--graph-bg').trim() || '#0f172a';
        const border = style.getPropertyValue('--border').trim() || '#334155';
        const muted = style.getPropertyValue('--text-muted').trim() || '#94a3b8';

        ctx.fillStyle = bg;
        ctx.fillRect(0, 0, W, H);
        ctx.strokeStyle = border;
        ctx.lineWidth = 1;
        ctx.strokeRect(0.5, 0.5, W - 1, H - 1);

        const series = state.series;
        const hasData = series.some(arr => arr.length > 0);
        if (!hasData) return;

        let yMin = Infinity, yMax = -Infinity;
        for (let j = 0; j < series.length; j++) {
            for (let t = 0; t < series[j].length; t++) {
                const y = series[j][t];
                if (typeof y === 'number' && isFinite(y)) {
                    if (y < yMin) yMin = y;
                    if (y > yMax) yMax = y;
                }
            }
        }
        if (!isFinite(yMin) || !isFinite(yMax)) return;
        if (yMin === yMax) { yMin -= 1; yMax += 1; }
        const yPad = (yMax - yMin) * 0.08 || 0.05;
        yMin -= yPad; yMax += yPad;

        // 网格线
        ctx.strokeStyle = border;
        ctx.globalAlpha = 0.35;
        ctx.beginPath();
        for (let g = 0; g <= 4; g++) {
            const gy = padT + (plotH * g) / 4;
            ctx.moveTo(padL, gy);
            ctx.lineTo(padL + plotW, gy);
        }
        ctx.stroke();
        ctx.globalAlpha = 1;

        // Y 轴标签
        ctx.fillStyle = muted;
        ctx.font = '10px ui-monospace, monospace';
        ctx.textAlign = 'right';
        ctx.textBaseline = 'middle';
        for (let g = 0; g <= 4; g++) {
            const v = yMax - ((yMax - yMin) * g) / 4;
            const gy = padT + (plotH * g) / 4;
            ctx.fillText(v.toFixed(3), padL - 6, gy);
        }

        // 曲线
        for (let j = 0; j < series.length; j++) {
            const arr = series[j];
            if (arr.length === 0) continue;
            const color = lineColors[j % lineColors.length];
            if (arr.length === 1 && isFinite(arr[0])) {
                const yNorm = (arr[0] - yMin) / (yMax - yMin);
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.arc(padL + plotW / 2, padT + plotH * (1 - yNorm), 3.5, 0, Math.PI * 2);
                ctx.fill();
                continue;
            }
            if (arr.length < 2) continue;
            const denom = Math.max(1, arr.length - 1);
            ctx.strokeStyle = color;
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            for (let t = 0; t < arr.length; t++) {
                const x = padL + (t / denom) * plotW;
                const yNorm = (arr[t] - yMin) / (yMax - yMin);
                const y = padT + plotH * (1 - yNorm);
                if (t === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
            }
            ctx.stroke();
        }

        ctx.fillStyle = muted;
        ctx.font = '10px system-ui, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText('时间 →', padL + plotW / 2, H - padB + 4);
    }

    function scheduleDraw() {
        if (drawRaf != null) return;
        drawRaf = requestAnimationFrame(() => { drawRaf = null; drawImmediate(); });
    }

    function updateLegend() {
        const leg = jointLegendEl();
        if (!leg) return;
        leg.replaceChildren();
        if (!state.names.length) { leg.setAttribute('aria-hidden', 'true'); return; }
        state.names.forEach((n, i) => {
            const row = document.createElement('span');
            row.className = 'legend-j';
            row.setAttribute('role', 'listitem');
            const sw = document.createElement('span');
            sw.className = 'legend-j-swatch';
            sw.style.background = lineColors[i % lineColors.length];
            sw.setAttribute('aria-hidden', 'true');
            const lab = document.createElement('span');
            lab.className = 'legend-j-name';
            lab.textContent = n;
            row.appendChild(sw); row.appendChild(lab);
            leg.appendChild(row);
        });
        leg.setAttribute('aria-hidden', 'false');
    }

    return {
        reset() {
            state = { names: [], series: [] };
            if (drawRaf != null) { cancelAnimationFrame(drawRaf); drawRaf = null; }
            const leg = jointLegendEl();
            if (leg) { leg.replaceChildren(); leg.setAttribute('aria-hidden', 'true'); }
            drawImmediate();
        },
        pushSample(rawNames, positions) {
            const n = Math.max(rawNames.length, positions.length);
            if (n === 0) return;
            const normNames = rawNames.map((rn, i) =>
                rn != null && rn !== '' ? String(rn) : `joint_${i}`);
            const keyNew = normNames.join('\0');
            const keyOld = state.names.join('\0');
            if (state.series.length !== n || keyNew !== keyOld) {
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
            const jc = $(canvasId);
            if (!jc) return;
            if (typeof ResizeObserver !== 'undefined' && jc.parentElement) {
                if (resizeObs) resizeObs.disconnect();
                resizeObs = new ResizeObserver(() => scheduleDraw());
                resizeObs.observe(jc.parentElement);
            }
            scheduleDraw();
        },
        getState() { return state; },
    };
}

export { createJointChartController };
