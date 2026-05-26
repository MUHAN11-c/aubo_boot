// latte_controls.js — 拉花参数控制: 图案选择 + 杯子配置 + 倾倒参数 + 预览/执行
// 依赖: ros.js (RosManager 单例), logBus (日志总线), fetch API
// 链路:
//   预览: rosbridge → /latte_imitation/replay_trajectory service (mode="preview")
//   执行: rosbridge → /latte_imitation/replay_trajectory service (mode="action")

import { ros } from '../core/ros.js';
import { logBus } from '../core/log-bus.js';

const TAG = '[latte_ctrl]';

// ── 常量 (与 latte.ts 对齐) ──
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
    cupX: -0.630,   // 首次默认, 启动后从 ROS2 lizhu_link.x 参数自动获取
    cupY: -0.368,   // 首次默认, 启动后从 ROS2 lizhu_link.y 参数自动获取
    cupZ: 0.04,     // 液面 Z, 需真机微调
    cupR: 0.04,
    mixH: 0.076, drawH: 0.006, finishH: 0.076,
    wiggleAmp: 0.006, wiggleFreq: 5.0,
    maxVel: 0.05, maxAcc: 0.1, maxJerk: 0.5,
    antiSlosh: true,
    roll: 0, pitch: 0, yaw: 0,
    speedScale: 1.0, toolId: 'default', arm: 'right',
    dx: 0.0, dy: 0.0, dz: 0.0, waypointStep: 5,
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
    speedScale: DEFAULTS.speedScale, toolId: DEFAULTS.toolId, arm: DEFAULTS.arm,
    dx: DEFAULTS.dx, dy: DEFAULTS.dy, dz: DEFAULTS.dz,
    waypointStep: DEFAULTS.waypointStep,
    previewLoading: false, execExecuting: false,
    message: '', success: null, // null=info, true=ok, false=err
    lastPreviewWaypoints: null,
    livePreview: true,
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
        const { previewLoading, execExecuting, message, success, lastPreviewWaypoints, ...s } = _state;
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
        translation_x: _state.dx, translation_y: _state.dy, translation_z: _state.dz,
        waypoint_sample_step: _state.waypointStep,
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
    if (pose) {
        req.start_pose = {
            position: {x: pose.x, y: pose.y, z: pose.z},
            orientation: {x: pose.qx, y: pose.qy, z: pose.qz, w: pose.qw},
        };
    }
    return req;
}

function _getCurrentEEPose() {
    if (!window.__ivgPoseReady) return null;
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
        }, 'latte');
    }, 5000);
}

function _stopProgressHeartbeat() {
    if (_execProgressTimer) { clearInterval(_execProgressTimer); _execProgressTimer = null; }
    _execStartTime = 0;
}

// ── 防抖预览 ──
let _debounceTimer = null;
function _debouncedPreview(delayMs = 200) {
    if (_debounceTimer) clearTimeout(_debounceTimer);
    _debounceTimer = setTimeout(() => {
        _debounceTimer = null;
        if (_state.livePreview && !_state.previewLoading && !_state.execExecuting) {
            _preview();
        }
    }, delayMs);
}

// ── API 调用 ──
async function _preview() {
    logBus.addLog('info', 'service', '预览开始: ' + _logPatternLabel() + ' (ROS service)', {
        phase: 'start', pattern: _state.patternType, episode: _state.episodeIdx,
    }, 'latte');

    _state.previewLoading = true; _render();
    const t0 = performance.now();

    try {
        const req = _buildRequest('preview');
        const result = await ros.callService(DEFAULT_SVC, DEFAULT_SVC_TYPE, req, 15000);
        const ms = (performance.now() - t0).toFixed(0);

        _state.lastPreviewWaypoints = result?.waypoints || [];
        const wpCount = _state.lastPreviewWaypoints.length;
        // 构建平移偏移尾部（非零时显示）喵~
        const tx = _state.dx, ty = _state.dy, tz = _state.dz;
        const hasTrans = Math.abs(tx) > 0.0005 || Math.abs(ty) > 0.0005 || Math.abs(tz) > 0.0005;
        const transTail = hasTrans
            ? ' Δ(' + tx.toFixed(3) + ',' + ty.toFixed(3) + ',' + tz.toFixed(3) + ')'
            : '';
        _state.message = result?.message || '预览完成: ' + (result?.num_frames || '?') + ' 帧, ' + ((result?.path_length)?.toFixed(2) ?? '?') + 'm' + transTail;
        _state.success = result?.success !== false;
        if (wpCount === 0 && _state.success) {
            _state.message += ' (轨迹坐标为空，无3D渲染)';
        }

        logBus.addLog('info', 'service', '预览 ✓ 完成 (' + ms + 'ms): ' + (result?.num_frames || '?') + ' 帧', {
            phase: 'completed', num_frames: result?.num_frames, path_length: result?.path_length, duration_ms: parseInt(ms),
        }, 'latte');

        // 触发 3D 轨迹渲染
        document.dispatchEvent(new CustomEvent('latte:preview-ready', {
            detail: {
                waypoints: _state.lastPreviewWaypoints,
                num_frames: result?.num_frames,
                path_length: result?.path_length,
                success: _state.success,
                message: _state.message,
            },
        }));
    } catch (e) {
        _state.message = '预览失败: ' + String(e.message || e);
        _state.success = false;
        _state.lastPreviewWaypoints = [];

        logBus.addLog('error', 'service', '预览 ✗ 失败: ' + String(e.message || e), {
            phase: 'failed', error: String(e.message || e),
        }, 'latte');
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
    }, 'latte');

    _state.execExecuting = true; _render();
    _startProgressHeartbeat(label);
    const t0 = performance.now();

    try {
        const result = await ros.callService(svc, DEFAULT_SVC_TYPE, req, 60000);
        _stopProgressHeartbeat();

        const ms = (performance.now() - t0).toFixed(0);
        const ok = result?.success !== false;
        _state.message = result?.message || (ok ? '执行完成' : '执行失败');
        _state.success = ok;

        logBus.addLog(ok ? 'info' : 'warn', 'service', '执行 ' + (ok ? '✓' : '✗') + ' (' + ms + 'ms)', {
            phase: 'completed', service: svc, duration_ms: parseInt(ms),
            success: ok, response_message: result?.message || '',
        }, 'latte');
    } catch (e) {
        _stopProgressHeartbeat();
        _state.message = '执行失败: ' + String(e.message || e);
        _state.success = false;

        const ms = (performance.now() - t0).toFixed(0);
        logBus.addLog('error', 'service', '执行 ✗ 失败 (' + ms + 'ms): ' + String(e.message || e), {
            phase: 'failed', service: svc, duration_ms: parseInt(ms), error: String(e.message || e),
        }, 'latte');
    } finally {
        _state.execExecuting = false; _render();
    }
}

function _stop() {
    logBus.addLog('info', 'service', '停止请求: ' + DEFAULT_SVC + ' (后端未提供取消接口，此按钮暂不可用)', {
        phase: 'stop', service: DEFAULT_SVC, available: false,
    }, 'latte');
    _stopProgressHeartbeat();
    _state.message = '停止: 后端未提供轨迹取消接口，按钮暂不可用';
    _state.success = null;
    _state.execExecuting = false;
    _render();
}

function _home() {
    logBus.addLog('info', 'service', '回原点: 服务未实现（需 ROS /home 或 /move_to_joints 服务）', {
        phase: 'home', available: false,
    }, 'latte');
    _state.message = '回原点: 服务未实现（需 ROS /home 或 /move_to_joints 服务）';
    _state.success = null;
    _render();
}

// ── 辅助: 安全设置 input.value, 跳过当前聚焦的输入框避免打断编辑喵~
function _safeSetInput(id, value) {
    const el = document.getElementById(id);
    if (el && document.activeElement !== el) {
        el.value = value;
    }
}

// ── UI 渲染 ──

function _render() {
    const s = _state;
    // ── 条件显示: 参数化模式 vs 录制回放 ──
    const isParametric = !!s.patternType;
    const cupGroup = document.getElementById('latte-cup-group');
    const pourGroup = document.getElementById('latte-pour-group');
    const velGroup = document.getElementById('latte-vel-group');
    const epRow = document.getElementById('latte-episode-row');
    const tulipRow = document.getElementById('latte-tulip-row');

    if (cupGroup) cupGroup.style.display = isParametric ? 'block' : 'none';
    if (pourGroup) pourGroup.style.display = isParametric ? 'block' : 'none';
    if (velGroup) velGroup.style.display = isParametric ? 'block' : 'none';
    if (epRow) epRow.style.display = isParametric ? 'none' : 'flex';
    if (tulipRow) tulipRow.style.display = (s.patternType === 'tulip') ? 'flex' : 'none';

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
                }, 'latte');
                // 切换 episode 输入显示
                const epRow = document.getElementById('latte-episode-row');
                if (epRow) epRow.style.display = _state.patternType ? 'none' : 'flex';
                _saveSession(); _render(); _debouncedPreview();
            });
        });
    }

    // episode
    _safeSetInput('latte-episode', s.episodeIdx);

    // tulip layers
    _safeSetInput('latte-tulip-layers', s.tulipLayers);

    // 杯子
    ['cupX','cupY','cupZ','cupR'].forEach(k => {
        _safeSetInput('latte-' + k, s[k]);
    });

    // 倾倒参数 (带实时值显示 #latte-xxx_val)
    ['mixH','drawH','finishH','wiggleAmp','wiggleFreq','maxVel','maxAcc','maxJerk'].forEach(k => {
        _safeSetInput('latte-' + k, s[k]);
        const ve = document.getElementById('latte-' + k + '_val');
        if (ve) ve.textContent = s[k];
    });
    const asEl = document.getElementById('latte-antiSlosh');
    if (asEl) asEl.checked = s.antiSlosh;

    // 变换
    ['roll','pitch','yaw'].forEach(k => {
        _safeSetInput('latte-' + k, s[k]);
    });
    _safeSetInput('latte-speedScale', s.speedScale);

    // 平移偏移 (带实时值显示 #latte-xxx_val)
    ['dx','dy','dz'].forEach(k => {
        _safeSetInput('latte-' + k, s[k]);
        const ve = document.getElementById('latte-' + k + '_val');
        if (ve) ve.textContent = (s[k] * 1000).toFixed(0) + ' mm';
    });
    _safeSetInput('latte-waypointStep', s.waypointStep);

    // 实时预览开关
    const lpEl = document.getElementById('latte-live-preview');
    if (lpEl) lpEl.checked = s.livePreview;

    // 按钮状态 (预览/执行互斥，防止同时发起两个服务调用) 喵~
    const poseReady = window.__ivgPoseReady === true;
    const busy = s.previewLoading || s.execExecuting;
    const btnP = document.getElementById('latte-btn-preview');
    if (btnP) {
        if (!poseReady && !busy) {
            btnP.disabled = true;
            btnP.textContent = '等待末端位姿...';
        } else {
            btnP.disabled = busy;
            btnP.textContent = s.previewLoading ? '预览中…' : (s.execExecuting ? '执行中…' : '预览');
        }
    }
    const btnE = document.getElementById('latte-btn-execute');
    if (btnE) {
        btnE.disabled = busy;
        btnE.textContent = s.execExecuting ? '执行中…' : (s.previewLoading ? '预览中…' : '执行');
    }

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
        _debouncedPreview();
        if (old !== _state[key]) {
            logBus.addLog('debug', 'service', '参数变更: ' + id + ' ' + old + ' → ' + _state[key], {
                key, from: old, to: _state[key],
            }, 'latte');
        }
    });
}

// ── 事件绑定 ──
function _bindEvents() {
    // Episode
    const epEl = document.getElementById('latte-episode');
    if (epEl) epEl.addEventListener('input', () => {
        const old = _state.episodeIdx;
        _state.episodeIdx = parseInt(epEl.value) || 0; _saveSession(); _debouncedPreview();
        if (old !== _state.episodeIdx) {
            logBus.addLog('debug', 'service', '参数变更: episode ' + old + ' → ' + _state.episodeIdx, { key: 'episode', from: old, to: _state.episodeIdx }, 'latte');
        }
    });
    // Tulip layers
    const tlEl = document.getElementById('latte-tulip-layers');
    if (tlEl) tlEl.addEventListener('input', () => {
        const old = _state.tulipLayers;
        _state.tulipLayers = parseInt(tlEl.value) || 3; _saveSession(); _debouncedPreview();
        if (old !== _state.tulipLayers) {
            logBus.addLog('debug', 'service', '参数变更: tulipLayers ' + old + ' → ' + _state.tulipLayers, { key: 'tulipLayers', from: old, to: _state.tulipLayers }, 'latte');
        }
    });
    // 杯子
    ['cupX','cupY','cupZ','cupR'].forEach(k => _bindNumberInput(k, k, _saveSession));
    // 倾倒 (含 range 实时渲染)
    ['mixH','drawH','finishH','wiggleAmp','wiggleFreq','maxVel','maxAcc','maxJerk'].forEach(k => _bindNumberInput(k, k, () => { _saveSession(); _render(); }));
    // 防晃
    const asEl = document.getElementById('latte-antiSlosh');
    if (asEl) asEl.addEventListener('change', () => {
        _state.antiSlosh = asEl.checked; _saveSession(); _debouncedPreview();
        logBus.addLog('debug', 'service', '参数变更: antiSlosh → ' + _state.antiSlosh, { key: 'antiSlosh', to: _state.antiSlosh }, 'latte');
    });
    // 变换
    ['roll','pitch','yaw'].forEach(k => _bindNumberInput(k, k, _saveRpy));
    // 速度倍率
    const ssEl = document.getElementById('latte-speedScale');
    if (ssEl) ssEl.addEventListener('input', () => { _state.speedScale = parseFloat(ssEl.value) || 1.0; _saveSession(); _debouncedPreview(); });
    // 平移偏移
    ['dx','dy','dz'].forEach(k => _bindNumberInput(k, k, _saveSession));
    // 采样步长
    const wsEl2 = document.getElementById('latte-waypointStep');
    if (wsEl2) wsEl2.addEventListener('input', () => {
        const v = parseInt(wsEl2.value) || 5;
        _state.waypointStep = Math.max(1, Math.min(20, v));
        _saveSession(); _debouncedPreview();
    });
    // 实时预览开关
    const lpEl = document.getElementById('latte-live-preview');
    if (lpEl) lpEl.addEventListener('change', () => {
        _state.livePreview = lpEl.checked;
        _saveSession();
        logBus.addLog('info', 'service', '实时预览: ' + (_state.livePreview ? '开启' : '关闭'), {
            key: 'livePreview', to: _state.livePreview,
        }, 'latte');
        if (_state.livePreview) _debouncedPreview(150); // 开启时立即触发一次预览
    });

    // 按钮
    const btnP = document.getElementById('latte-btn-preview');
    if (btnP) btnP.addEventListener('click', () => {
        if (_debounceTimer) { clearTimeout(_debounceTimer); _debounceTimer = null; }
        logBus.addLog('info', 'service', '按钮点击: 预览 ' + _logPatternLabel(), {}, 'latte');
        _preview();
    });
    const btnE = document.getElementById('latte-btn-execute');
    if (btnE) btnE.addEventListener('click', () => {
        logBus.addLog('info', 'service', '按钮点击: 执行 ' + _logPatternLabel(), {}, 'latte');
        _execute();
    });
    const btnS = document.getElementById('latte-btn-stop');
    if (btnS) btnS.addEventListener('click', () => {
        logBus.addLog('info', 'service', '按钮点击: 停止', {}, 'latte');
        _stop();
    });
    const btnH = document.getElementById('latte-btn-home');
    if (btnH) btnH.addEventListener('click', () => {
        logBus.addLog('info', 'service', '按钮点击: 回原点', {}, 'latte');
        _home();
    });

    // 位姿就绪时刷新按钮状态
    document.addEventListener('latte:pose-ready', () => { _render(); });
}

// ── ROS2 参数同步 ──
async function _syncParamsFromRos2() {
    const paramMap = {
        'lizhu_link.x': 'cupX', 'lizhu_link.y': 'cupY',
        'lizhu_link.z': 'cupZ',
    };
    let updated = false;
    for (const [paramName, stateKey] of Object.entries(paramMap)) {
        try {
            const result = await ros.callService(
                '/rosapi/get_param', 'rosapi/GetParam',
                { name: '/latte_imitation/' + paramName }, 3000
            );
            if (result && typeof result.value === 'number') {
                const oldVal = _state[stateKey];
                _state[stateKey] = parseFloat(result.value.toFixed(4));
                if (Math.abs(_state[stateKey] - oldVal) > 0.0001) {
                    updated = true;
                    logBus.addLog('info', 'service',
                        'ROS2 参数同步: ' + paramName + ' = ' + _state[stateKey],
                        { param: paramName, value: _state[stateKey] }, 'latte');
                }
            }
        } catch (_) { /* rosapi 不可用则保持默认值 */ }
    }
    if (updated) { _render(); _debouncedPreview(); }
}

// ── 参考位姿设置 ──
const REF_POSE_LINKS = [
    { key: 'coffee_link', label: 'coffee_Link (取咖啡杯)' },
    { key: 'lizhu_link', label: 'lizhu_Link (放咖啡杯)' },
    { key: 'cup0_link', label: 'cup0_Link (取牛奶杯)' },
    { key: 'reference_pose', label: '参考位姿 (杯口朝上)' },
];
const REF_AXES = [
    { key: 'x', label: 'X', step: 0.001 },
    { key: 'y', label: 'Y', step: 0.001 },
    { key: 'z', label: 'Z', step: 0.001 },
    { key: 'roll', label: 'R°', step: 0.1 },
    { key: 'pitch', label: 'P°', step: 0.1 },
    { key: 'yaw', label: 'Y°', step: 0.1 },
];

function _renderSettingsGrid() {
    const grid = document.getElementById('latte-settings-grid');
    if (!grid) return;
    grid.innerHTML = REF_POSE_LINKS.map(link => {
        const rows = REF_AXES.map(ax => {
            const paramName = link.key + '.' + ax.key;
            return '<label class="latte-cfg-row"><span>' + link.label + ' ' + ax.label + '</span>'
                + '<input type="number" class="latte-num-inp latte-settings-inp"'
                + ' data-param="' + paramName + '" step="' + ax.step + '" />'
                + '</label>';
        }).join('');
        return '<div class="latte-settings-link-group"><h4>' + link.label + '</h4><div class="latte-cfg-grid latte-cfg-grid--3col">' + rows + '</div></div>';
    }).join('');

    // 绑定事件
    grid.querySelectorAll('.latte-settings-inp').forEach(inp => {
        inp.addEventListener('change', async () => {
            const paramName = inp.dataset.param;
            const value = parseFloat(inp.value);
            if (isNaN(value)) return;
            try {
                await ros.callService('/rosapi/set_param', 'rosapi/SetParam', {
                    name: '/latte_imitation/' + paramName, value: String(value),
                }, 3000);
                logBus.addLog('info', 'service', 'ROS2 参数更新: ' + paramName + ' = ' + value,
                    { param: paramName, value: value }, 'latte');
                // 同步更新前端状态
                if (paramName === 'lizhu_link.x') _state.cupX = value;
                if (paramName === 'lizhu_link.y') _state.cupY = value;
                if (paramName === 'lizhu_link.z') _state.cupZ = value;
                _saveSession(); _debouncedPreview();
            } catch (e) {
                logBus.addLog('warn', 'service', 'ROS2 参数设置失败: ' + paramName,
                    { error: String(e.message || e) }, 'latte');
            }
        });
    });

    document.getElementById('latte-settings-refresh')?.addEventListener('click', () => {
        _syncParamsFromRos2().then(() => _fillSettingsInputs());
    });
}

async function _fillSettingsInputs() {
    const grids = document.querySelectorAll('.latte-settings-inp');
    for (const inp of grids) {
        const paramName = inp.dataset.param;
        try {
            const result = await ros.callService(
                '/rosapi/get_param', 'rosapi/GetParam',
                { name: '/latte_imitation/' + paramName }, 3000
            );
            if (result && typeof result.value !== 'undefined') {
                inp.value = typeof result.value === 'number'
                    ? parseFloat(result.value.toFixed(4)) : result.value;
            }
        } catch (_) { /* */ }
    }
}

function _bindSettingsToggle() {
    const btn = document.getElementById('latte-settings-toggle');
    const body = document.getElementById('latte-settings-body');
    if (!btn || !body) return;
    btn.addEventListener('click', () => {
        const expanded = btn.getAttribute('aria-expanded') === 'true';
        btn.setAttribute('aria-expanded', !expanded);
        body.style.display = expanded ? 'none' : 'block';
        if (!expanded) { _fillSettingsInputs(); }
    });
}

// ── 入口 ──
export function initLatteControls() {
    _loadSession();
    _render();
    _bindEvents();
    _bindSettingsToggle();
    _renderSettingsGrid();
    _syncParamsFromRos2().then(() => _fillSettingsInputs());

    logBus.addLog('info', 'system', '拉花参数控制就绪 pattern=' + (_state.patternType || '(episode)') +
        ' cup=(' + _state.cupX + ',' + _state.cupY + ',' + _state.cupZ + ')', {}, 'latte');
    return {
        getState: () => _state, preview: _preview, execute: _execute,
        cleanup: () => {
            if (_debounceTimer) { clearTimeout(_debounceTimer); _debounceTimer = null; }
            _stopProgressHeartbeat();
        },
    };
}
