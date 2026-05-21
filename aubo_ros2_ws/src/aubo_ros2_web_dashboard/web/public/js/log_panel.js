// log_panel.js — 浏览器端综合日志面板 (v2 — 使用统一 logBus)
// 捕获: console / 全局错误 / ros.js 生命周期 / service 响应 / topic 数据 / rosout
import { logBus } from './core/log-bus.js';
import { escapeHtml } from './core/utils.js';

const TAG = 'log_panel';

// ── 渲染状态 ───────────────────────────────────────────────────────────────
let paused = false;
let autoScroll = true;
const MAX_DOM = 200;          // DOM 节点上限（仅可见+缓冲区的 ~2x）
let _lastRenderTs = '';       // 批量阈值: 同一毫秒内的多条合并渲染

const activeFilters = {};     // source → boolean（全部默认 true，动态从 categories 生成）

// ── DOM 辅助 ───────────────────────────────────────────────────────────────
function $(id) { return document.getElementById(id); }

// ── 日志行渲染 ─────────────────────────────────────────────────────────────
function renderLine(entry) {
    if (!entry) return;
    const container = $('log-lines');
    if (!container) return;

    const cats = logBus.getCategories();
    const cat = cats[entry.source] || { color: 'var(--text-muted)', label: entry.source };
    const metaHtml = entry.meta && Object.keys(entry.meta).length
        ? ` <span class="log-meta" title="${escapeHtml(JSON.stringify(entry.meta))}">+</span>`
        : '';

    const div = document.createElement('div');
    div.className = 'log-line log-line--' + entry.level;
    div.setAttribute('data-source', entry.source);
    div.innerHTML =
        '<span class="log-ts">' + escapeHtml(entry.ts) + '</span>' +
        '<span class="log-src" style="color:' + cat.color + '">[' + escapeHtml(cat.label) + ']</span>' +
        '<span class="log-msg">' + escapeHtml(entry.msg) + metaHtml + '</span>';

    // 点击 meta 展开/收起
    if (entry.meta && Object.keys(entry.meta).length) {
        div.querySelector('.log-meta').addEventListener('click', function (e) {
            e.stopPropagation();
            const detail = div.querySelector('.log-detail');
            if (detail) { detail.remove(); return; }
            const pre = document.createElement('pre');
            pre.className = 'log-detail';
            pre.textContent = JSON.stringify(entry.meta, null, 2);
            div.appendChild(pre);
        });
    }

    container.appendChild(div);
    while (container.children.length > MAX_DOM) container.removeChild(container.firstChild);
    if (autoScroll) container.scrollTop = container.scrollHeight;
}

function renderAll() {
    const container = $('log-lines');
    if (!container) return;
    container.innerHTML = '';

    const all = logBus.getLogs();
    const slice = all.slice(-MAX_DOM);
    for (const e of slice) {
        if (!_entryVisible(e)) continue;
        _renderEntryDOM(container, e);
    }
    if (autoScroll && container.lastChild) container.lastChild.scrollIntoView(false);
}

function _entryVisible(e) {
    if (activeFilters[e.source] === false) return false;
    if (activeFilters[e.level] === false) return false;
    return true;
}

function _renderEntryDOM(container, entry) {
    const cats = logBus.getCategories();
    const cat = cats[entry.source] || { color: 'var(--text-muted)', label: entry.source };
    const metaHtml = entry.meta && Object.keys(entry.meta).length
        ? ` <span class="log-meta" title="${escapeHtml(JSON.stringify(entry.meta))}">+</span>`
        : '';
    const div = document.createElement('div');
    div.className = 'log-line log-line--' + entry.level;
    div.setAttribute('data-source', entry.source);
    div.innerHTML =
        '<span class="log-ts">' + escapeHtml(entry.ts) + '</span>' +
        '<span class="log-src" style="color:' + cat.color + '">[' + escapeHtml(cat.label) + ']</span>' +
        '<span class="log-msg">' + escapeHtml(entry.msg) + metaHtml + '</span>';
    if (entry.meta && Object.keys(entry.meta).length) {
        div.querySelector('.log-meta').addEventListener('click', function (e) {
            e.stopPropagation();
            const detail = div.querySelector('.log-detail');
            if (detail) { detail.remove(); return; }
            const pre = document.createElement('pre');
            pre.className = 'log-detail';
            pre.textContent = JSON.stringify(entry.meta, null, 2);
            div.appendChild(pre);
        });
    }
    container.appendChild(div);
}

function updateCount() {
    const el = $('log-count');
    if (el) el.textContent = logBus.count() + ' 条';
}

// ── 日志总线监听 ───────────────────────────────────────────────────────────
logBus.onLog(function (entry) {
    if (!entry) { renderAll(); updateCount(); return; }  // null = 清空信号
    if (paused) return;
    if (!_entryVisible(entry)) return;
    renderLine(entry);
    updateCount();
});

// ── Console 拦截 ───────────────────────────────────────────────────────────
function installConsoleHook() {
    ['log', 'warn', 'error', 'info', 'debug'].forEach(function (level) {
        const orig = console[level];
        console[level] = function () {
            const msg = Array.from(arguments).map(function (a) {
                if (a instanceof Error) return a.message || a.toString();
                if (typeof a === 'object') {
                    try { return JSON.stringify(a); } catch (e) { return String(a); }
                }
                return String(a);
            }).join(' ');
            logBus.addLog(level, 'console', msg);
            orig.apply(console, arguments);
        };
    });
}

// ── 全局错误 ───────────────────────────────────────────────────────────────
function installErrorHooks() {
    window.addEventListener('error', function (e) {
        let info = e.message || 'Unknown error';
        if (e.filename) info += '  (' + e.filename + ':' + e.lineno + ')';
        logBus.addLog('error', 'error', info);
    });
    window.addEventListener('unhandledrejection', function (e) {
        logBus.addLog('error', 'error', (e.reason && e.reason.message) || String(e.reason));
    });
}

// ── 页面生命周期 ───────────────────────────────────────────────────────────
function installLifecycleHooks() {
    document.addEventListener('visibilitychange', function () {
        logBus.addLog('info', 'lifecycle', document.hidden ? '页面隐藏 (后台)' : '页面可见 (前台)');
    });
    window.addEventListener('pagehide', function () {
        logBus.addLog('info', 'lifecycle', '页面已隐藏');
    });
}

// ── 过滤搜索 ───────────────────────────────────────────────────────────────
function filterByText(text) {
    const container = $('log-lines');
    if (!container) return;
    const q = String(text || '').trim().toLowerCase();
    for (let i = 0; i < container.children.length; i++) {
        const line = container.children[i];
        if (!q) { line.style.display = ''; continue; }
        line.style.display = line.textContent.toLowerCase().indexOf(q) !== -1 ? '' : 'none';
    }
}

// ── 导出 ───────────────────────────────────────────────────────────────────
function exportLogs() {
    if (logBus.count() === 0) return;
    const text = logBus.export();
    const blob = new Blob([text], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'ivg_log_' + new Date().toISOString().slice(0, 10) + '.txt';
    a.click();
    URL.revokeObjectURL(url);
}

// ── 动态生成 source 过滤按钮 ──────────────────────────────────────────────
function buildSourceButtons() {
    const toolbar = $('log-toolbar');
    if (!toolbar) return;
    const cats = logBus.getCategories();
    const existingIds = new Set();
    // 收集已有的按钮 id
    toolbar.querySelectorAll('button[id^="log-btn-"]').forEach(b => existingIds.add(b.id));

    for (const [key, cat] of Object.entries(cats)) {
        const btnId = 'log-btn-' + key;
        if (existingIds.has(btnId)) continue;
        activeFilters[key] = true;
        const btn = document.createElement('button');
        btn.className = 'log-ctrl';
        btn.id = btnId;
        btn.textContent = cat.label;
        btn.title = '过滤 ' + cat.label + ' 来源';
        btn.addEventListener('click', function () {
            activeFilters[key] = !activeFilters[key];
            btn.classList.toggle('log-ctrl--off', !activeFilters[key]);
            renderAll();
        });
        const sep = toolbar.querySelector('#log-btn-lifecycle');
        if (sep && sep.parentNode) {
            sep.parentNode.insertBefore(btn, sep.nextSibling);
        } else {
            toolbar.appendChild(btn);
        }
    }
}

// ── 控制按钮绑定 ───────────────────────────────────────────────────────────
function bindControls() {
    $('log-btn-clear') && $('log-btn-clear').addEventListener('click', function () {
        logBus.clear();
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

    const input = $('log-filter-input');
    if (input) input.addEventListener('input', function () { filterByText(this.value); });
}

// ── pre-init: 预设所有分类为可见，避免 restore 时被过滤喵~ ─────────────
(function () {
    const cats = logBus.getCategories();
    for (const key of Object.keys(cats)) {
        activeFilters[key] = true;
    }
})();

// ── 入口 ───────────────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', async function () {
    installConsoleHook();
    installErrorHooks();
    installLifecycleHooks();
    bindControls();
    buildSourceButtons();  // 先生成按钮（设置 activeFilters + DOM）

    // 恢复历史日志（此时 activeFilters 已就绪）
    const restored = await logBus.restore(500);
    updateCount();

    if (restored > 0) {
        logBus.addLog('info', 'system', '已恢复 ' + restored + ' 条历史日志');
    }
    logBus.addLog('info', 'system', '日志面板已就绪 — 等待事件…');
    logBus.addLog('info', 'lifecycle', '页面加载完成');
});
