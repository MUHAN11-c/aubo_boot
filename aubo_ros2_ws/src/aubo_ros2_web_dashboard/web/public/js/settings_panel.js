// settings_panel — IVG 系统设置（公共参数 / 视觉抓取 / 咖啡拉花 三分类）
// 数据流: /api/v1/runtime → 网关状态 + 配置定义 | localStorage ↔ 表单
// 保存键名: ivg_vision_grasp_topics_v3（与视觉抓取/咖啡拉花面板共用）
const STORAGE_KEY = 'ivg_vision_grasp_topics_v3';
const $ = id => document.getElementById(id);

// ── 配置定义（从 BFF 获取，含 label 字段）─────────────────────────────────
let categoryDefs = { common: {}, vision: {}, latte: {} };

async function loadCategoryDefs() {
    try {
        const r = await fetch('/api/v1/runtime', { credentials: 'same-origin' });
        if (!r.ok) return;
        const data = await r.json();
        if (data.settings_categories) categoryDefs = data.settings_categories;

        // 网关信息
        $('gw-info').innerHTML = [
            ['版本', data.version],
            ['rosbridge 端口', data.rosbridge_port],
            ['web_video 端口', data.web_video_port],
            ['WS 路径', data.rosbridge_ws_path],
            ['代理前缀', data.web_video_proxy_prefix],
        ].map(([k, v]) => `<span>${k}: <code>${v != null ? v : '—'}</code></span>`).join('');

        $('gw-status').textContent = '已连接';
        $('gw-status').className = 'gw-status ok';
    } catch (e) {
        $('gw-status').textContent = '不可达';
        $('gw-status').className = 'gw-status err';
    }
}

// ── 工具 ─────────────────────────────────────────────────────────────────
function escapeHtml(s) {
    return String(s || '').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}
function showMsg(text, ok) {
    const el = $('settings-msg');
    if (!el) return;
    el.textContent = text;
    el.className = 'settings-msg ' + (ok ? 'ok' : 'err');
    if (ok) setTimeout(() => { el.textContent = ''; el.className = 'settings-msg'; }, 4000);
}

// ── 构建表单 ──────────────────────────────────────────────────────────────
function buildSection(containerId, items, values, typeLabel) {
    const container = $(containerId);
    if (!container) return;
    if (!items || !items.length) { container.innerHTML = ''; return; }

    const title = typeLabel === 'tf' ? 'TF 话题' : typeLabel === 'service' ? 'ROS 服务' : 'ROS 话题';
    container.innerHTML = `<h3>${title}</h3><div class="settings-grid"></div>`;
    const grid = container.querySelector('.settings-grid');

    items.forEach(d => {
        const val = values[d.id] !== undefined ? values[d.id] : d.default;
        const changed = values[d.id] !== undefined && values[d.id] !== d.default;
        const div = document.createElement('div');
        div.className = 'settings-item' + (changed ? ' changed' : '');
        div.innerHTML = `
            <label for="cfg-${d.id}">${d.label || d.id}
                ${changed ? '<span class="changed-badge">已修改</span>' : ''}
            </label>
            <input type="text" id="cfg-${d.id}" value="${escapeHtml(String(val))}"
                   placeholder="${d.allow_empty ? '留空禁用' : d.default}"
                   data-default="${escapeHtml(d.default)}" />
        `;
        grid.appendChild(div);
        const input = div.querySelector('input');
        input.addEventListener('input', () => {
            const isDefault = input.value.trim() === input.dataset.default;
            div.classList.toggle('changed', !isDefault);
            const badge = div.querySelector('.changed-badge');
            if (badge) badge.style.display = isDefault ? 'none' : '';
        });
    });
}

function buildAllForms(values) {
    const c = categoryDefs;
    // 公共
    buildSection('common-topics', c.common?.topics, values, 'topic');
    buildSection('common-tf',     c.common?.tf_topics, values, 'tf');
    // 视觉抓取
    buildSection('vision-topics',    c.vision?.topics, values, 'topic');
    buildSection('vision-services',  c.vision?.services, values, 'service');
    // 咖啡拉花
    buildSection('latte-topics',     c.latte?.topics, values, 'topic');
    buildSection('latte-services',   c.latte?.services, values, 'service');
}

function readAllInputs() {
    const out = {};
    document.querySelectorAll('.settings-grid input').forEach(el => {
        out[el.id.replace('cfg-', '')] = el.value.trim();
    });
    return out;
}

// ── 保存：API 写入 YAML（优先） + localStorage 兜底 ──────────────────
function loadFromStorage() {
    try { const raw = localStorage.getItem(STORAGE_KEY); return raw ? JSON.parse(raw) || {} : {}; }
    catch (e) { return {}; }
}
function saveToStorage(values) {
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(values)); return true; }
    catch (e) { return false; }
}

async function saveToServer(values) {
    try {
        const r = await fetch('/api/v1/settings', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(values),
            credentials: 'same-origin',
        });
        return r.ok;
    } catch (e) {
        return false;
    }
}

// ── 导出/导入 ─────────────────────────────────────────────────────────────
function exportSettings() {
    const blob = new Blob([JSON.stringify(readAllInputs(), null, 2)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'ivg_settings_' + new Date().toISOString().slice(0,10) + '.json';
    a.click(); URL.revokeObjectURL(a.href);
    showMsg('已导出。', true);
}
function importSettings() {
    const input = document.createElement('input'); input.type = 'file'; input.accept = '.json';
    input.onchange = () => {
        const file = input.files && input.files[0]; if (!file) return;
        const r = new FileReader();
        r.onload = () => {
            try {
                const obj = JSON.parse(r.result);
                if (!obj || typeof obj !== 'object') throw new Error('格式错误');
                buildAllForms(obj);
                showMsg('已导入 ' + Object.keys(obj).length + ' 项，请点击保存。', false);
            } catch (e) { showMsg('导入失败: ' + e.message, false); }
        };
        r.readAsText(file);
    };
    input.click();
}

// ── 标签切换 ──────────────────────────────────────────────────────────────
function initTabs() {
    const tabs = document.querySelectorAll('.settings-tab');
    tabs.forEach(tab => {
        tab.addEventListener('click', () => {
            tabs.forEach(t => t.classList.remove('active'));
            tab.classList.add('active');
            document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
            const target = $('tab-' + tab.dataset.tab);
            if (target) target.classList.add('active');
        });
    });
}

// ── 入口 ─────────────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', async () => {
    await loadCategoryDefs();
    const saved = loadFromStorage();

    buildAllForms(saved);
    initTabs();

    // 刷新链接
    const refreshLink = document.querySelector('#refresh-link');
    if (refreshLink) {
        refreshLink.addEventListener('click', e => {
            e.preventDefault();
            const params = new URLSearchParams({ _: Date.now() });
            // 检测当前来源页面
            const ref = document.referrer;
            const page = ref.includes('vision_grasp') ? 'vision_grasp_panel.html'
                : ref.includes('coffee_latte') ? 'coffee_latte_panel.html'
                : ref.includes('tf_monitor') ? 'tf_monitor_panel.html'
                : 'index.html';
            window.open(page + '?' + params.toString(), '_blank');
        });
    }

    $('btn-save').addEventListener('click', async () => {
        const values = readAllInputs();
        saveToStorage(values);  // 始终写 localStorage

        const ok = await saveToServer(values);
        if (ok) {
            showMsg('已保存到 YAML 配置（重启网关后仍有效）。', true);
        } else {
            showMsg('已保存到浏览器本地（网关不可达，未写入 YAML）。', false);
        }
    });
    $('btn-reset').addEventListener('click', () => {
        if (!confirm('确认恢复所有分类的默认值？')) return;
        buildAllForms({});
        showMsg('已恢复默认值，请点击保存以持久化。', false);
    });
    $('btn-clear').addEventListener('click', () => {
        if (!confirm('清除本地存储？所有面板将使用默认值。')) return;
        try { localStorage.removeItem(STORAGE_KEY); } catch (e) { /* */ }
        buildAllForms({});
        showMsg('已清除。', true);
    });
    $('btn-export').addEventListener('click', exportSettings);
    $('btn-import').addEventListener('click', importSettings);
});
