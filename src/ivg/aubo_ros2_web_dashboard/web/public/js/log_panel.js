// log_panel.js — 浏览器端综合日志面板 (v3 — 频率控制 + rAF 批量渲染)
// 捕获: console / 全局错误 / ros.js 生命周期 / service 响应 / topic 数据 / rosout
import { logBus, FEATURES, LOG_LEVELS } from './core/log-bus.js';
import { escapeHtml } from './core/utils.js';

const TAG = 'log_panel';

// ── 渲染状态 ───────────────────────────────────────────────────────────────
let paused = false;
let autoScroll = true;
const MAX_DOM = 200;            // DOM 节点上限

// rAF 批量渲染
let _renderPending = [];        // 待渲染条目队列
let _rafId = null;
const MAX_PER_FRAME = 20;       // 每帧最多追加 20 条

const activeFilters = {};        // source → boolean
const activeFeatureFilters = {};  // feature → boolean

// 持久化设置
const SETTINGS_KEY = 'ivg_log_panel_settings_v1';

function loadSettings() {
    try {
        const raw = localStorage.getItem(SETTINGS_KEY);
        if (raw) return JSON.parse(raw);
    } catch (_) { /* */ }
    return {};
}

function saveSettings(s) {
    try {
        const current = loadSettings();
        Object.assign(current, s);
        localStorage.setItem(SETTINGS_KEY, JSON.stringify(current));
    } catch (_) { /* */ }
}

// ── DOM 辅助 ───────────────────────────────────────────────────────────────
function $(id) { return document.getElementById(id); }

// ── 日志行渲染 ─────────────────────────────────────────────────────────────
function _renderEntryDOM(container, entry) {
    const cats = logBus.getCategories();
    const cat = cats[entry.source] || { color: 'var(--text-muted)', label: entry.source };
    const feats = logBus.getFeatures();
    const feat = entry.feature ? (feats[entry.feature] || { color: 'var(--text-muted)', label: entry.feature }) : null;
    const featHtml = feat
        ? '<span class="log-feat" style="background:' + feat.color + '20;color:' + feat.color + ';border-color:' + feat.color + '40">' + feat.label + '</span>'
        : '';

    // 折叠计数标记
    const rateCount = (entry.meta && entry.meta._rateCount) ? entry.meta._rateCount : 0;
    const rateHtml = rateCount > 1
        ? '<span class="log-rate-badge" title="' + rateCount + ' 条折叠">x' + rateCount + '</span>'
        : '';

    const metaHtml = entry.meta && Object.keys(entry.meta).length
        ? ' <span class="log-meta" title="' + escapeHtml(JSON.stringify(entry.meta)) + '">+</span>'
        : '';

    const div = document.createElement('div');
    div.className = 'log-line log-line--' + entry.level;
    if (rateCount > 1) div.classList.add('log-line--folded');
    div.setAttribute('data-source', entry.source);
    if (entry.feature) div.setAttribute('data-feature', entry.feature);
    div.innerHTML =
        '<span class="log-ts">' + escapeHtml(entry.ts) + '</span>' +
        '<span class="log-src" style="color:' + cat.color + '">[' + escapeHtml(cat.label) + ']</span>' +
        featHtml +
        rateHtml +
        '<span class="log-msg">' + escapeHtml(entry.msg) + metaHtml + '</span>';

    if (entry.meta && Object.keys(entry.meta).length) {
        const metaBtn = div.querySelector('.log-meta');
        if (metaBtn) {
            metaBtn.addEventListener('click', function (e) {
                e.stopPropagation();
                const detail = div.querySelector('.log-detail');
                if (detail) { detail.remove(); return; }
                const pre = document.createElement('pre');
                pre.className = 'log-detail';
                pre.textContent = JSON.stringify(entry.meta, null, 2);
                div.appendChild(pre);
            });
        }
    }

    container.appendChild(div);
}

// ── rAF 批量渲染 ──────────────────────────────────────────────────────────
function _scheduleRender(entry) {
    _renderPending.push(entry);
    if (!_rafId) {
        _rafId = requestAnimationFrame(_flushRender);
    }
}

function _flushRender() {
    _rafId = null;
    const container = $('log-lines');
    if (!container) { _renderPending = []; return; }

    const batch = _renderPending;
    _renderPending = [];

    // 每帧最多渲染 MAX_PER_FRAME 条
    const toRender = batch.slice(-MAX_PER_FRAME);

    for (const entry of toRender) {
        if (!entry) {
            // null = 清空信号
            container.innerHTML = '';
            return;
        }
        _renderEntryDOM(container, entry);
    }

    // 裁剪 DOM
    while (container.children.length > MAX_DOM) {
        container.removeChild(container.firstChild);
    }

    if (autoScroll) {
        container.scrollTop = container.scrollHeight;
    } else {
        // 即使不自动滚动也提示有新日志
        _updateUnreadBadge(batch.length);
    }
}

let _unreadCount = 0;
function _updateUnreadBadge(increment) {
    _unreadCount += increment;
    // 显示在暂停按钮上作为提示
    const btn = $('log-btn-pause');
    if (btn && !autoScroll && _unreadCount > 0) {
        btn.setAttribute('data-unread', _unreadCount > 99 ? '99+' : String(_unreadCount));
    }
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
    _unreadCount = 0;
}

function _entryVisible(e) {
    if (activeFilters[e.source] === false) return false;
    if (activeFilters[e.level] === false) return false;
    if (e.feature && activeFeatureFilters[e.feature] === false) return false;
    return true;
}

function updateCount() {
    const el = $('log-count');
    if (el) el.textContent = logBus.count() + ' 条';
}

// ── 日志总线监听 (rAF 批量) ───────────────────────────────────────────────
logBus.onLog(function (entry) {
    if (!entry) { renderAll(); updateCount(); return; }  // null = 清空信号
    if (paused) { _unreadCount++; return; }
    if (!_entryVisible(entry)) return;
    _scheduleRender(entry);
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
    toolbar.querySelectorAll('button[id^="log-btn-"]').forEach(b => existingIds.add(b.id));

    for (const [key, cat] of Object.entries(cats)) {
        const btnId = 'log-btn-' + key;
        if (existingIds.has(btnId)) continue;
        activeFilters[key] = true;
        const btn = document.createElement('button');
        btn.className = 'log-ctrl';
        btn.id = btnId;
        btn.textContent = cat.label;
        btn.title = '按来源过滤: ' + cat.label;
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

// ── 动态生成 feature 过滤按钮 ─────────────────────────────────────────────
function buildFeatureButtons() {
    const toolbar = $('log-feature-toolbar');
    if (!toolbar) return;
    toolbar.innerHTML = '<span class="log-ctrl__sep"></span>';
    const feats = logBus.getFeatures();

    for (const [key, f] of Object.entries(feats)) {
        activeFeatureFilters[key] = true;
        const btn = document.createElement('button');
        btn.className = 'log-ctrl';
        btn.id = 'log-feat-' + key;
        btn.textContent = f.label;
        btn.title = '按功能过滤: ' + f.label;
        btn.style.borderColor = f.color + '60';
        btn.style.color = f.color;
        btn.addEventListener('click', function () {
            activeFeatureFilters[key] = !activeFeatureFilters[key];
            btn.classList.toggle('log-ctrl--off', !activeFeatureFilters[key]);
            renderAll();
        });
        toolbar.appendChild(btn);
    }
}

// ── 日志等级选择器 ────────────────────────────────────────────────────────
function buildLevelSelector() {
    const toolbar = $('log-level-toolbar');
    if (!toolbar) return;

    const settings = loadSettings();
    const currentLevel = settings.logLevel != null ? settings.logLevel : LOG_LEVELS.info;

    // 恢复等级设置
    logBus.setLevel(currentLevel);
    const debugOn = settings.debugMode === true;
    logBus.setDebugMode(debugOn);

    // 等级下拉框
    const sel = document.createElement('select');
    sel.className = 'log-search';
    sel.id = 'log-level-select';
    sel.title = '最低日志等级';
    sel.style.minWidth = '80px';
    const levels = [
        { val: LOG_LEVELS.debug, label: 'DEBUG' },
        { val: LOG_LEVELS.info,  label: 'INFO' },
        { val: LOG_LEVELS.warn,  label: 'WARN' },
        { val: LOG_LEVELS.error, label: 'ERROR' },
    ];
    for (const lv of levels) {
        const opt = document.createElement('option');
        opt.value = lv.val;
        opt.textContent = lv.label;
        if (currentLevel === lv.val) opt.selected = true;
        sel.appendChild(opt);
    }
    sel.addEventListener('change', function () {
        const val = parseInt(this.value);
        logBus.setLevel(val);
        saveSettings({ logLevel: val });
        renderAll();
    });

    // 调试模式按钮
    const dbg = document.createElement('button');
    dbg.className = 'log-ctrl';
    dbg.id = 'log-btn-debug-mode';
    dbg.textContent = '🐛 调试';
    dbg.title = '调试模式: 关闭所有频率节流, 全量输出日志';
    if (debugOn) dbg.classList.add('log-ctrl--active');
    dbg.addEventListener('click', function () {
        const next = !logBus.isDebugMode();
        logBus.setDebugMode(next);
        this.classList.toggle('log-ctrl--active', next);
        saveSettings({ debugMode: next });
        // 清空 rate trackers 避免旧摘要干扰
        renderAll();
    });

    const label = document.createElement('span');
    label.className = 'log-feature-label';
    label.textContent = '等级:';

    toolbar.appendChild(label);
    toolbar.appendChild(sel);
    toolbar.appendChild(dbg);

    // 恢复调试模式按钮状态
    if (debugOn) dbg.classList.add('log-ctrl--active');
}

// ── 控制按钮绑定 ───────────────────────────────────────────────────────────
function bindControls() {
    $('log-btn-clear') && $('log-btn-clear').addEventListener('click', function () {
        logBus.clear();
        _unreadCount = 0;
    });

    $('log-btn-pause') && $('log-btn-pause').addEventListener('click', function () {
        paused = !paused;
        this.textContent = paused ? '▶ 恢复' : '⏸ 暂停';
        this.classList.toggle('log-ctrl--active', paused);
        if (!paused) {
            this.removeAttribute('data-unread');
            _unreadCount = 0;
            _flushRender();  // 恢复时立即刷新待渲染条目
            // 补渲染暂停期间错过的
            const pending = _renderPending.length;
            _renderPending = [];
            if (pending > 0) {
                const container = $('log-lines');
                if (container) {
                    const all = logBus.getLogs();
                    const missed = all.slice(-pending);
                    for (const e of missed) {
                        if (!_entryVisible(e)) continue;
                        _renderEntryDOM(container, e);
                    }
                }
            }
            renderAll();  // 完整刷新
        }
    });

    $('log-btn-scroll') && $('log-btn-scroll').addEventListener('click', function () {
        autoScroll = !autoScroll;
        this.textContent = autoScroll ? '⏹ 自动滚动:开' : '⏺ 自动滚动:关';
        this.classList.toggle('log-ctrl--active', !autoScroll);
        if (autoScroll) {
            _unreadCount = 0;
            const btn = $('log-btn-pause');
            if (btn) btn.removeAttribute('data-unread');
        }
    });

    $('log-btn-export') && $('log-btn-export').addEventListener('click', exportLogs);

    const input = $('log-filter-input');
    if (input) input.addEventListener('input', function () { filterByText(this.value); });
}

// ── pre-init: 预设所有过滤为可见 ──────────────────────────────────────────
(function () {
    const cats = logBus.getCategories();
    for (const key of Object.keys(cats)) {
        activeFilters[key] = true;
    }
    const feats = logBus.getFeatures();
    for (const key of Object.keys(feats)) {
        activeFeatureFilters[key] = true;
    }
})();

// ── 入口 ───────────────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', async function () {
    installConsoleHook();
    installErrorHooks();
    installLifecycleHooks();
    bindControls();
    buildLevelSelector();     // 等级选择器 + 调试模式 (需在 restore 前设置)
    buildSourceButtons();     // source 过滤按钮
    buildFeatureButtons();    // feature 过滤按钮

    // 恢复历史日志
    const restored = await logBus.restore(500);
    updateCount();

    if (restored > 0) {
        // 使用内部 _entries 追加，避免触发 rAF 渲染
        logBus.addLog('info', 'system', '已恢复 ' + restored + ' 条历史日志');
    }
    logBus.addLog('info', 'system', '日志面板就绪 — 等待事件…');
    logBus.addLog('info', 'lifecycle', '页面加载完成');

    // 初始渲染
    renderAll();
});
