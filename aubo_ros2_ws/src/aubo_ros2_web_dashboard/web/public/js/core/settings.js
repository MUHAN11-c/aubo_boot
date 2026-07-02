// settings.js — 话题/服务设置管理: localStorage 持久化 + BFF 同步 + 跨标签页事件
//
// 设置层级（优先级从高到低）:
//   1. localStorage 覆盖 (key: ivg_vision_grasp_topics_v3)
//   2. BFF /api/v1/runtime → settings_categories (YAML 默认值)
//   3. 硬编码 fallback (VISION_SETTINGS_DEFAULTS)
//
// 用法:
//   import { loadSettings, getSetting, saveSettings } from '../core/settings.js';
//   await loadSettings();  // 从 localStorage + BFF 加载
//   const topic = getSetting('topic-color');  // → '/camera/color/image_raw'
//   saveSettings({ 'topic-color': '/new_topic' });  // → 写 localStorage + POST BFF

import { canonicalRosTopic } from './utils.js';

const STORAGE_KEY = 'ivg_vision_grasp_topics_v3';
const SETTINGS_CHANGED_EVENT = 'ivg-dashboard-settings-changed';

// 模块级缓存
let _overrides = null;       // localStorage 覆盖值
let _bffSettings = null;     // BFF settings_categories 解析后的默认值
let _allDefaults = {};       // 最终合并的默认值

// ── 加载 ──────────────────────────────────────────────────────────────────────

/** 从 localStorage 加载覆盖值 */
function _loadOverrides() {
    if (_overrides) return _overrides;
    try {
        const raw = localStorage.getItem(STORAGE_KEY);
        _overrides = raw ? JSON.parse(raw) : {};
    } catch (_) {
        _overrides = {};
    }
    if (typeof _overrides !== 'object' || Array.isArray(_overrides)) {
        _overrides = {};
    }
    return _overrides;
}

/** 从 BFF 运行时配置中解析 settings_categories → 展平为 id→defaultValue */
function _parseBffSettings(runtimeConfig) {
    const cats = runtimeConfig?.settings_categories;
    if (!cats || typeof cats !== 'object') return {};
    const out = {};
    for (const [_catName, catVal] of Object.entries(cats)) {
        if (!catVal || typeof catVal !== 'object') continue;
        for (const listKey of ['topics', 'tf_topics', 'services']) {
            const items = catVal[listKey];
            if (!Array.isArray(items)) continue;
            for (const item of items) {
                if (item && item.id && item.default !== undefined) {
                    out[item.id] = item.default;
                }
            }
        }
    }
    return out;
}

/**
 * 加载设置（每个页面初始化时调用一次）
 * @param {Object} hardcodedDefaults - 硬编码默认值 { id: defaultValue }
 * @param {Object} [bffConfig] - BFF /api/v1/runtime 响应（可选，自动从 globalThis 获取）
 * @returns {Object} 合并后的默认值
 */
export async function loadSettings(hardcodedDefaults = {}, bffConfig = null) {
    _allDefaults = { ...hardcodedDefaults };

    // 尝试从 BFF 获取
    const rt = bffConfig || globalThis.__IVG_RUNTIME;
    if (rt) {
        _bffSettings = _parseBffSettings(rt);
        _allDefaults = { ..._allDefaults, ..._bffSettings };
    } else {
        // 运行时未加载，尝试 fetch
        try {
            const resp = await fetch('/api/v1/runtime', { credentials: 'same-origin' });
            if (resp.ok) {
                const data = await resp.json();
                _bffSettings = _parseBffSettings(data);
                _allDefaults = { ..._allDefaults, ..._bffSettings };
            }
        } catch (_) { /* BFF 不可达，仅用硬编码默认值 */ }
    }

    _loadOverrides();
    _listenCrossTab();
    return _allDefaults;
}

// ── 读取 ──────────────────────────────────────────────────────────────────────

/**
 * 获取某个设置的最终值（localStorage 覆盖 > BFF > 硬编码）
 * @param {string} id
 * @param {string} [fallback]
 * @returns {string}
 */
export function getSetting(id, fallback = '') {
    const ov = _loadOverrides();
    if (ov[id] !== undefined && ov[id] !== '') return ov[id];
    if (_allDefaults[id] !== undefined) return _allDefaults[id];
    return fallback;
}

/**
 * 获取规范化的 ROS 话题/服务名（始终以 / 开头）
 */
export function getRosName(id, fallback = '') {
    const raw = getSetting(id, fallback);
    return canonicalRosTopic(raw);
}

/**
 * 获取话题消息类型
 */
export function getTopicType(id, typeMap = {}) {
    return typeMap[id] || '';
}

/**
 * 获取服务类型
 */
export function getServiceType(id, typeMap = {}) {
    return typeMap[id] || '';
}

// ── 写入 ──────────────────────────────────────────────────────────────────────

/** 保存单个设置到 localStorage（不触发 BFF 同步） */
export function setOverride(id, value) {
    const ov = _loadOverrides();
    ov[id] = String(value ?? '');
    try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(ov));
    } catch (_) { /* quota exceeded */ }
}

/** 批量保存到 localStorage + 可选 BFF 同步 */
export async function saveSettings(settings, syncToBff = false) {
    const ov = _loadOverrides();
    for (const [id, value] of Object.entries(settings)) {
        ov[id] = String(value ?? '');
    }
    try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(ov));
    } catch (_) { /* quota exceeded */ }

    // 通知同源其他标签页
    _dispatchSettingsChanged(settings);

    // BFF 持久化
    if (syncToBff) {
        try {
            await fetch('/api/v1/settings', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(settings),
            });
        } catch (_) { /* BFF 不可达，仅本地保存 */ }
    }
}

/** 恢复全部默认值（清除 localStorage 覆盖） */
export function resetSettings() {
    _overrides = {};
    try {
        localStorage.removeItem(STORAGE_KEY);
    } catch (_) { /* */ }
    _dispatchSettingsChanged({});
}

/** 导出全部当前设置（含默认值 + 覆盖）为 JSON */
export function exportSettings() {
    const result = {};
    for (const id of Object.keys(_allDefaults)) {
        result[id] = getSetting(id);
    }
    // 也包含仅有覆盖值的设置
    const ov = _loadOverrides();
    for (const id of Object.keys(ov)) {
        if (!(id in result)) result[id] = ov[id];
    }
    return result;
}

/** 从 JSON 导入设置 */
export async function importSettings(json, syncToBff = false) {
    let data;
    try {
        data = typeof json === 'string' ? JSON.parse(json) : json;
    } catch (_) { return false; }
    if (!data || typeof data !== 'object') return false;
    await saveSettings(data, syncToBff);
    return true;
}

// ── 跨标签页同步 ──────────────────────────────────────────────────────────────

function _dispatchSettingsChanged(settings) {
    try {
        window.dispatchEvent(new CustomEvent(SETTINGS_CHANGED_EVENT, { detail: settings }));
    } catch (_) { /* */ }
}

function _listenCrossTab() {
    if (typeof window === 'undefined') return;
    // 同源其他标签页的 localStorage 变化
    window.addEventListener('storage', (e) => {
        if (e.key === STORAGE_KEY) {
            _overrides = null; // 强制重新读取
            _loadOverrides();
            _dispatchSettingsChanged(_overrides);
        }
    });
}

export { STORAGE_KEY, SETTINGS_CHANGED_EVENT };
