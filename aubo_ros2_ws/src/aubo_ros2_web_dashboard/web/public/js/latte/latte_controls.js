// latte_controls.js — 拉花参数控制: 图案选择 + 杯子配置 + 倾倒参数 + 预览/执行
// 依赖: ros.js (RosManager 单例), logBus (日志总线), fetch API
// 链路:
//   预览: fetch → POST /api/v1/latte/trajectory/preview → JSON waypoints
//   执行: rosbridge → /latte_imitation/replay_trajectory service (ivg_interfaces/srv/ReplayLatteTrajectory)

import { ros } from '../core/ros.js';
import { logBus } from '../core/log-bus.js';

const TAG = '[latte_ctrl]';

// ── 常量 (与 latte.ts 对齐) ──
const LATTE_PREVIEW_API = '/api/v1/latte/trajectory/preview';
const LATTE_RPY_STORAGE_KEY = 'ivg_latte_rpy_v1';
const LATTE_SESSION_STORAGE_KEY = 'ivg_latte_session_v1';
const DEFAULT_SVC = '/latte_imitation/replay_trajectory';
const DEFAULT_SVC_TYPE = 'ivg_interfaces/srv/ReplayLatteTrajectory';

const PATTERN_TYPES = [
    { value: '', label: '录制回放 (Episode)' },
    { value: 'heart', label: '心形 (Heart)' },
    { value: 'rosetta', label: '树叶 (Rosetta)' },
    { value: 'tulip', label: '郁金香 (Tulip)' },
    { value: 'swan', label: '天鹅 (Swan)' },
];

const DEFAULTS = {
    episode: 0, maxEpisode: 39,
    cupX: 0.0, cupY: 0.0, cupZ: 0.15, cupR: 0.04,
    mixH: 0.076, drawH: 0.006, finishH: 0.076,
    wiggleAmp: 0.006, wiggleFreq: 5.0,
    maxVel: 0.05, maxAcc: 0.1, maxJerk: 0.5,
    antiSlosh: true,
    roll: 0, pitch: 0, yaw: 0,
    speedScale: 1.0, toolId: 'default', arm: 'right',
};

// ── 状态 ──
let _state = {
    patternType: '',
    episodeIdx: DEFAULTS.episode,
    tulipLayers: 3,
    cupX: DEFAULTS.cupX, cupY: DEFAULTS.cupY, cupZ: DEFAULTS.cupZ, cupR: DEFAULTS.cupR,
    mixH: DEFAULTS.mixH, drawH: DEFAULTS.drawH, finishH: DEFAULTS.finishH,
    wiggleAmp: DEFAULTS.wiggleAmp, wiggleFreq: DEFAULTS.wiggleFreq,
    maxVel: DEFAULTS.maxVel, maxAcc: DEFAULTS.maxAcc, maxJerk: DEFAULTS.maxJerk,
    antiSlosh: DEFAULTS.antiSlosh,
    roll: DEFAULTS.roll, pitch: DEFAULTS.pitch, yaw: DEFAULTS.yaw,
    speedScale: DEFAULTS.speedScale, toolId: DEFAULTS.toolId,
    previewLoading: false, execExecuting: false,
    message: '', success: null, // null=info, true=ok, false=err
    lastPreview: null, // { tcp_path, spout_path, cup_pose, workspace_bounds }
};

// 执行进度心跳
let _execProgressTimer = null;
let _execStartTime = 0;

// ── localStorage ──
function _loadSession() {
    try {
        const raw = localStorage.getItem(LATTE_SESSION_STORAGE_KEY);
        if (raw) Object.assign(_state, JSON.parse(raw));
    } catch (_) { /* */ }
    try {
        const raw = localStorage.getItem(LATTE_RPY_STORAGE_KEY);
        if (raw) { const r = JSON.parse(raw); if (r.roll != null) _state.roll = r.roll; if (r.pitch != null) _state.pitch = r.pitch; if (r.yaw != null) _state.yaw = r.yaw; }
    } catch (_) { /* */ }
}

function _saveSession() {
    try {
        const { previewLoading, execExecuting, message, success, lastPreview, ...s } = _state;
        localStorage.setItem(LATTE_SESSION_STORAGE_KEY, JSON.stringify(s));
    } catch (_) { /* */ }
}

function _saveRpy() {
    try {
        localStorage.setItem(LATTE_RPY_STORAGE_KEY, JSON.stringify({ roll: _state.roll, pitch: _state.pitch, yaw: _state.yaw }));
    } catch (_) { /* */ }
}

// ── 日志辅助 ──
function _logPatternLabel() {
    const p = PATTERN_TYPES.find(x => x.value === _state.patternType);
    return p ? p.label : '(录制回放 Episode=' + _state.episodeIdx + ')';
}

function _logRequestSummary(req) {
    if (req.pattern_type) {
        return req.pattern_type + ' cup=(' + (req.cup_center_x||0).toFixed(2) + ',' + (req.cup_center_y||0).toFixed(2) + ',' + (req.cup_surface_z||0).toFixed(3) + ')';
    }
    return 'episode=' + (req.episode_idx||0);
}

// ── 请求构建 ──
function _buildRequest(mode) {
    const req = {
        episode_idx: _state.episodeIdx, arm: _state.arm,
        speed_scale: _state.speedScale, mode,
        roll_deg: _state.roll, pitch_deg: _state.pitch, yaw_deg: _state.yaw,
        tool_offset_id: _state.toolId,
    };
    if (_state.patternType) {
        req.pattern_type = _state.patternType;
        req.tulip_layers = _state.tulipLayers;
        req.cup_center_x = _state.cupX; req.cup_center_y = _state.cupY;
        req.cup_surface_z = _state.cupZ; req.cup_radius = _state.cupR;
        req.pour_mix_height_offset = _state.mixH; req.pour_draw_height_offset = _state.drawH;
        req.pour_finish_height_offset = _state.finishH;
        req.pour_wiggle_amplitude = _state.wiggleAmp; req.pour_wiggle_frequency = _state.wiggleFreq;
        req.pour_max_velocity = _state.maxVel; req.pour_max_acceleration = _state.maxAcc;
        req.pour_max_jerk = _state.maxJerk;
        req.enable_anti_sloshing = _state.antiSlosh;
    }
    // 尝试获取当前 EE 位姿
    const pose = _getCurrentEEPose();
    if (pose) req.start_pose = pose;
    return req;
}

function _getCurrentEEPose() {
    try {
        const p = window.__ivgLastEEPose;
        if (p && typeof p.x === 'number') return { x: p.x, y: p.y, z: p.z, qx: p.qx, qy: p.qy, qz: p.qz, qw: p.qw };
    } catch (_) { /* */ }
    return null;
}

// ── 执行进度心跳 ──
function _startProgressHeartbeat(label) {
    _stopProgressHeartbeat();
    _execStartTime = Date.now();
    _execProgressTimer = setInterval(() => {
        const elapsed = ((Date.now() - _execStartTime) / 1000).toFixed(0);
        logBus.addLog('info', 'service', '执行中: ' + label + ' (已等待 ' + elapsed + 's...)', {
            phase: 'in_progress', elapsed_s: parseInt(elapsed),
        });
    }, 5000);
}

function _stopProgressHeartbeat() {
    if (_execProgressTimer) { clearInterval(_execProgressTimer); _execProgressTimer = null; }
    _execStartTime = 0;
}

// ── API 调用 ──
async function _preview() {
    logBus.addLog('info', 'service', '预览开始: ' + _logPatternLabel(), {
        phase: 'start', pattern: _state.patternType, episode: _state.episodeIdx,
    });

    _state.previewLoading = true; _render();
    const t0 = performance.now();

    try {
        const req = _buildRequest('preview');
        const resp = await fetch(LATTE_PREVIEW_API, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(req),
        });
        if (!resp.ok) throw new Error('BFF ' + resp.status + ': ' + (await resp.text().catch(() => '')));
        const data = await resp.json();
        _state.lastPreview = data;
        _state.message = data.message || '轨迹生成成功: ' + data.num_frames + ' 帧, ' + (data.path_length?.toFixed(2) ?? '?') + 'm';
        _state.success = data.success !== false;

        const ms = (performance.now() - t0).toFixed(0);
        logBus.addLog('info', 'service', '预览 ✓ 完成: ' + data.num_frames + ' 帧, ' + (data.path_length?.toFixed(2) ?? '?') + 'm (' + ms + 'ms)', {
            phase: 'completed', num_frames: data.num_frames, path_length: data.path_length, duration_ms: parseInt(ms),
        });

        // 触发 3D 预览叠加
        if (data.tcp_path) _emitPreviewOverlay(data);
    } catch (e) {
        _state.message = '预览失败: ' + String(e.message || e);
        _state.success = false;

        logBus.addLog('error', 'service', '预览 ✗ 失败: ' + String(e.message || e), {
            phase: 'failed', error: String(e.message || e),
        });
    } finally {
        _state.previewLoading = false; _render();
    }
}

async function _execute() {
    const req = _buildRequest('action');
    const label = _logPatternLabel();
    const svc = DEFAULT_SVC;

    logBus.addLog('info', 'service', '执行开始: ' + label + ' → ' + svc, {
        phase: 'start', service: svc, pattern: _state.patternType,
        request_summary: _logRequestSummary(req),
    });

    _state.execExecuting = true; _render();
    _startProgressHeartbeat(label);
    const t0 = performance.now();

    try {
        const result = await ros.callService(svc, DEFAULT_SVC_TYPE, req, 120000);
        _stopProgressHeartbeat();

        const ms = (performance.now() - t0).toFixed(0);
        _state.message = result?.message ?? (result?.success !== false ? '执行完成' : '执行失败');
        _state.success = result?.success ?? true;

        logBus.addLog('info', 'service', '执行 ✓ 完成 (' + ms + 'ms)', {
            phase: 'completed', service: svc, duration_ms: parseInt(ms),
            success: _state.success, response_message: result?.message || '',
        });
    } catch (e) {
        _stopProgressHeartbeat();
        _state.message = '执行失败: ' + String(e.message || e);
        _state.success = false;

        const ms = (performance.now() - t0).toFixed(0);
        logBus.addLog('error', 'service', '执行 ✗ 失败 (' + ms + 'ms): ' + String(e.message || e), {
            phase: 'failed', service: svc, duration_ms: parseInt(ms), error: String(e.message || e),
        });
    } finally {
        _state.execExecuting = false; _render();
    }
}

function _stop() {
    logBus.addLog('info', 'service', '停止请求: ' + DEFAULT_SVC + ' (后端未提供取消接口，此按钮暂不可用)', {
        phase: 'stop', service: DEFAULT_SVC, available: false,
    });
    _stopProgressHeartbeat();
    _state.message = '停止: 后端未提供轨迹取消接口，按钮暂不可用';
    _state.success = null;
    _state.execExecuting = false;
    _render();
}

function _home() {
    logBus.addLog('info', 'service', '回原点: 服务未实现（需 ROS /home 或 /move_to_joints 服务）', {
        phase: 'home', available: false,
    });
    _state.message = '回原点: 服务未实现（需 ROS /home 或 /move_to_joints 服务）';
    _state.success = null;
    _render();
}

// ── 事件触发 ──
const _listeners = { previewData: [], execState: [] };
function on(evt, fn) { if (_listeners[evt]) _listeners[evt].push(fn); }
function _emit(evt, data) { if (_listeners[evt]) _listeners[evt].forEach(fn => fn(data)); }
function _emitPreviewOverlay(data) { _emit('previewData', { tcp_path: data.tcp_path, spout_path: data.spout_path, cup_pose: data.cup_pose, workspace_bounds: data.workspace_bounds }); }

// ── UI 渲染 ──

function _render() {
    const s = _state;
    // 图案选择卡片
    const pc = document.getElementById('latte-pattern-cards');
    if (pc) {
        pc.innerHTML = PATTERN_TYPES.map(p => {
            const sel = s.patternType === p.value ? ' latte-pattern-card--sel' : '';
            return '<button class="latte-pattern-card' + sel + '" data-pattern="' + p.value + '" type="button">' + p.label + '</button>';
        }).join('');
        pc.querySelectorAll('.latte-pattern-card').forEach(card => {
            card.addEventListener('click', () => {
                const old = _state.patternType;
                _state.patternType = card.dataset.pattern;
                logBus.addLog('info', 'service', '图案切换: ' + (_logPatternLabelFor(old)) + ' → ' + _logPatternLabel(), {
                    from: old, to: _state.patternType,
                });
                // 切换 episode 输入显示
                const epRow = document.getElementById('latte-episode-row');
                if (epRow) epRow.style.display = _state.patternType ? 'none' : 'flex';
                _saveSession(); _render();
            });
        });
    }

    // episode
    const epInp = document.getElementById('latte-episode');
    if (epInp) epInp.value = s.episodeIdx;

    // tulip layers
    const tlInp = document.getElementById('latte-tulip-layers');
    if (tlInp) tlInp.value = s.tulipLayers;

    // 杯子
    ['cupX','cupY','cupZ','cupR'].forEach(k => {
        const el = document.getElementById('latte-' + k);
        if (el) el.value = s[k];
    });

    // 倾倒参数
    ['mixH','drawH','finishH','wiggleAmp','wiggleFreq','maxVel','maxAcc','maxJerk'].forEach(k => {
        const el = document.getElementById('latte-' + k);
        if (el) el.value = s[k];
        const ve = document.getElementById('latte-' + k + '_val');
        if (ve) ve.textContent = s[k];
    });
    const asEl = document.getElementById('latte-antiSlosh');
    if (asEl) asEl.checked = s.antiSlosh;

    // 变换
    ['roll','pitch','yaw'].forEach(k => {
        const el = document.getElementById('latte-' + k);
        if (el) el.value = s[k];
    });
    const ssEl = document.getElementById('latte-speedScale');
    if (ssEl) ssEl.value = s.speedScale;

    // 按钮状态
    const btnP = document.getElementById('latte-btn-preview');
    if (btnP) { btnP.disabled = s.previewLoading; btnP.textContent = s.previewLoading ? '预览中…' : '预览'; }
    const btnE = document.getElementById('latte-btn-execute');
    if (btnE) { btnE.disabled = s.execExecuting; btnE.textContent = s.execExecuting ? '执行中…' : '执行'; }

    // 消息
    const msg = document.getElementById('latte-message');
    if (msg) {
        msg.innerHTML = s.message || '';
        msg.style.display = s.message ? 'block' : 'none';
        msg.className = 'latte-msg' + (s.success === false ? ' latte-msg--err' : s.success === true ? ' latte-msg--ok' : '');
    }
}

function _logPatternLabelFor(val) {
    const p = PATTERN_TYPES.find(x => x.value === val);
    return p ? p.label : '(录制回放)';
}

// ── 参数变更日志 ──
function _bindNumberInput(id, key, saveFn) {
    const el = document.getElementById('latte-' + id);
    if (!el) return;
    el.addEventListener('input', () => {
        const old = _state[key];
        _state[key] = parseFloat(el.value) || 0;
        if (saveFn) saveFn();
        if (old !== _state[key]) {
            logBus.addLog('debug', 'service', '参数变更: ' + id + ' ' + old + ' → ' + _state[key], {
                key, from: old, to: _state[key],
            });
        }
    });
}

// ── 事件绑定 ──
function _bindEvents() {
    // Episode
    const epEl = document.getElementById('latte-episode');
    if (epEl) epEl.addEventListener('input', () => {
        const old = _state.episodeIdx;
        _state.episodeIdx = parseInt(epEl.value) || 0; _saveSession();
        if (old !== _state.episodeIdx) {
            logBus.addLog('debug', 'service', '参数变更: episode ' + old + ' → ' + _state.episodeIdx, { key: 'episode', from: old, to: _state.episodeIdx });
        }
    });
    // Tulip layers
    const tlEl = document.getElementById('latte-tulip-layers');
    if (tlEl) tlEl.addEventListener('input', () => {
        const old = _state.tulipLayers;
        _state.tulipLayers = parseInt(tlEl.value) || 3; _saveSession();
        if (old !== _state.tulipLayers) {
            logBus.addLog('debug', 'service', '参数变更: tulipLayers ' + old + ' → ' + _state.tulipLayers, { key: 'tulipLayers', from: old, to: _state.tulipLayers });
        }
    });
    // 杯子
    ['cupX','cupY','cupZ','cupR'].forEach(k => _bindNumberInput(k, k, _saveSession));
    // 倾倒 (含 range 实时渲染)
    ['mixH','drawH','finishH','wiggleAmp','wiggleFreq','maxVel','maxAcc','maxJerk'].forEach(k => _bindNumberInput(k, k, () => { _saveSession(); _render(); }));
    // 防晃
    const asEl = document.getElementById('latte-antiSlosh');
    if (asEl) asEl.addEventListener('change', () => {
        _state.antiSlosh = asEl.checked; _saveSession();
        logBus.addLog('debug', 'service', '参数变更: antiSlosh → ' + _state.antiSlosh, { key: 'antiSlosh', to: _state.antiSlosh });
    });
    // 变换
    ['roll','pitch','yaw'].forEach(k => _bindNumberInput(k, k, _saveRpy));
    // 速度倍率
    const ssEl = document.getElementById('latte-speedScale');
    if (ssEl) ssEl.addEventListener('input', () => { _state.speedScale = parseFloat(ssEl.value) || 1.0; _saveSession(); });
    // 按钮
    const btnP = document.getElementById('latte-btn-preview');
    if (btnP) btnP.addEventListener('click', () => {
        logBus.addLog('info', 'service', '按钮点击: 预览 ' + _logPatternLabel());
        _preview();
    });
    const btnE = document.getElementById('latte-btn-execute');
    if (btnE) btnE.addEventListener('click', () => {
        logBus.addLog('info', 'service', '按钮点击: 执行 ' + _logPatternLabel());
        _execute();
    });
    const btnS = document.getElementById('latte-btn-stop');
    if (btnS) btnS.addEventListener('click', () => {
        logBus.addLog('info', 'service', '按钮点击: 停止');
        _stop();
    });
    const btnH = document.getElementById('latte-btn-home');
    if (btnH) btnH.addEventListener('click', () => {
        logBus.addLog('info', 'service', '按钮点击: 回原点');
        _home();
    });
}

// ── 入口 ──
export function initLatteControls() {
    _loadSession();
    _render();
    _bindEvents();

    logBus.addLog('info', 'system', '拉花参数控制就绪 pattern=' + (_state.patternType || '(episode)') +
        ' cup=(' + _state.cupX + ',' + _state.cupY + ',' + _state.cupZ + ')');
    return { getState: () => _state, on, preview: _preview, execute: _execute };
}
