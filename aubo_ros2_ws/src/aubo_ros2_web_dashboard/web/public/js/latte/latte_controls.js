// latte_controls.js — 拉花参数控制: 图案选择 + 杯子配置 + 倾倒参数 + 预览/执行
// 依赖: ivgTransport (全局单例), 浏览器 fetch API
// 链路:
//   预览: fetch → POST /api/v1/latte/trajectory/preview → JSON waypoints
//   执行: rosbridge → /latte_imitation/replay_trajectory service (ivg_interfaces/srv/ReplayLatteTrajectory)

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
  // 尝试获取当前 EE 位姿 (从全局 rosConnection 或 ivgTransport)
  const pose = _getCurrentEEPose();
  if (pose) req.start_pose = pose;
  return req;
}

function _getCurrentEEPose() {
  // 从 ivgTransport 的 poseStore 或全局变量读取末端位姿
  try {
    const p = window.__ivgLastEEPose;
    if (p && typeof p.x === 'number') return { x: p.x, y: p.y, z: p.z, qx: p.qx, qy: p.qy, qz: p.qz, qw: p.qw };
  } catch (_) { /* */ }
  return null;
}

// ── API 调用 ──
async function _preview() {
  _state.previewLoading = true; _render();
  try {
    const resp = await fetch(LATTE_PREVIEW_API, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(_buildRequest('preview')),
    });
    if (!resp.ok) throw new Error(`BFF ${resp.status}: ${await resp.text().catch(() => '')}`);
    const data = await resp.json();
    _state.lastPreview = data;
    _state.message = data.message || `轨迹生成成功: ${data.num_frames} 帧, ${data.path_length?.toFixed(2) ?? '?'}m`;
    _state.success = data.success !== false;
    // 触发 3D 预览叠加 (由 vision_grasp_panel 监听)
    if (data.tcp_path) _emitPreviewOverlay(data);
  } catch (e) {
    _state.message = '预览失败: ' + String(e.message || e);
    _state.success = false;
  } finally {
    _state.previewLoading = false; _render();
  }
}

async function _execute() {
  _state.execExecuting = true; _render();
  try {
    const req = _buildRequest('action');
    // 通过 ivgTransport rosbridge 调用 ROS 服务
    const svc = (typeof ivgTransport !== 'undefined' && ivgTransport.getSetting) ? ivgTransport.getSetting('latte-replay-svc', DEFAULT_SVC) : DEFAULT_SVC;
    let result;
    if (typeof ivgTransport !== 'undefined' && typeof ivgTransport.callService === 'function') {
      // ivgTransport 封装的服务调用
      result = await ivgTransport.callService(svc, req);
    } else {
      // 回退: 直接用 fetch 调 rosbridge API (V2 协议)
      result = await _rosbridgeServiceCall(svc, req);
    }
    _state.message = result?.message ?? (result?.success ? '执行完成' : '执行失败');
    _state.success = result?.success ?? true;
  } catch (e) {
    _state.message = '执行失败: ' + String(e.message || e);
    _state.success = false;
  } finally {
    _state.execExecuting = false; _render();
  }
}

async function _rosbridgeServiceCall(svc, req) {
  // 回退: 直接通过 rosbridge HTTP API 调用服务
  const resp = await fetch('/api/ivg/proxy/rosbridge/v2/call_service', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ service: svc, args: req }),
  });
  if (!resp.ok) throw new Error(`rosbridge ${resp.status}`);
  const data = await resp.json();
  return { success: data?.result ?? true, message: data?.message ?? '完成' };
}

// ── 事件触发 ──
const _listeners = { previewData: [], execState: [] };
function on(evt, fn) { if (_listeners[evt]) _listeners[evt].push(fn); }
function _emit(evt, data) { if (_listeners[evt]) _listeners[evt].forEach(fn => fn(data)); }
function _emitPreviewOverlay(data) { _emit('previewData', { tcp_path: data.tcp_path, spout_path: data.spout_path, cup_pose: data.cup_pose, workspace_bounds: data.workspace_bounds }); }

// ── UI 渲染 ──

/** 创建设置行: label + input */
function _settingRow(label, id, type, value, opts) {
  opts = opts || {};
  let inp;
  if (type === 'number') {
    inp = `<input id="${id}" type="number" value="${value}" step="${opts.step || 0.001}" min="${opts.min ?? ''}" max="${opts.max ?? ''}" class="latte-num-inp" />`;
  } else if (type === 'checkbox') {
    inp = `<input id="${id}" type="checkbox" ${value ? 'checked' : ''} class="latte-chk" />`;
  } else if (type === 'range') {
    inp = `<input id="${id}" type="range" value="${value}" step="${opts.step || 0.001}" min="${opts.min ?? 0}" max="${opts.max ?? 1}" class="latte-range" /><span class="latte-range-val" id="${id}_val">${value}</span>`;
  }
  return `<label class="latte-cfg-row"><span class="latte-cfg-label">${label}</span>${inp}</label>`;
}

function _render() {
  const s = _state;
  // 图案选择卡片
  const pc = document.getElementById('latte-pattern-cards');
  if (pc) {
    pc.innerHTML = PATTERN_TYPES.map(p => {
      const sel = s.patternType === p.value ? ' latte-pattern-card--sel' : '';
      return `<button class="latte-pattern-card${sel}" data-pattern="${p.value}" type="button">${p.label}</button>`;
    }).join('');
    pc.querySelectorAll('.latte-pattern-card').forEach(card => {
      card.addEventListener('click', () => {
        _state.patternType = card.dataset.pattern;
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

// ── 事件绑定 ──
function _bindEvents() {
  // Episode
  const epEl = document.getElementById('latte-episode');
  if (epEl) epEl.addEventListener('input', () => { _state.episodeIdx = parseInt(epEl.value) || 0; _saveSession(); });
  // Tulip layers
  const tlEl = document.getElementById('latte-tulip-layers');
  if (tlEl) tlEl.addEventListener('input', () => { _state.tulipLayers = parseInt(tlEl.value) || 3; _saveSession(); });
  // 杯子
  ['cupX','cupY','cupZ','cupR'].forEach(k => {
    const el = document.getElementById('latte-' + k);
    if (el) el.addEventListener('input', () => { _state[k] = parseFloat(el.value) || 0; _saveSession(); });
  });
  // 倾倒
  ['mixH','drawH','finishH','wiggleAmp','wiggleFreq','maxVel','maxAcc','maxJerk'].forEach(k => {
    const el = document.getElementById('latte-' + k);
    if (el) el.addEventListener('input', () => { _state[k] = parseFloat(el.value) || 0; _saveSession(); _render(); });
  });
  const asEl = document.getElementById('latte-antiSlosh');
  if (asEl) asEl.addEventListener('change', () => { _state.antiSlosh = asEl.checked; _saveSession(); });
  // 变换
  ['roll','pitch','yaw'].forEach(k => {
    const el = document.getElementById('latte-' + k);
    if (el) el.addEventListener('input', () => { _state[k] = parseFloat(el.value) || 0; _saveRpy(); });
  });
  const ssEl = document.getElementById('latte-speedScale');
  if (ssEl) ssEl.addEventListener('input', () => { _state.speedScale = parseFloat(ssEl.value) || 1.0; _saveSession(); });
  // 按钮
  const btnP = document.getElementById('latte-btn-preview');
  if (btnP) btnP.addEventListener('click', _preview);
  const btnE = document.getElementById('latte-btn-execute');
  if (btnE) btnE.addEventListener('click', _execute);
  const btnS = document.getElementById('latte-btn-stop');
  if (btnS) btnS.addEventListener('click', () => { _state.message = '停止未实现 (需 ROS service)'; _render(); });
  const btnH = document.getElementById('latte-btn-home');
  if (btnH) btnH.addEventListener('click', () => { _state.message = '回原点未实现 (需 ROS service)'; _render(); });
}

// ── 入口 ──
export function initLatteControls() {
  _loadSession();
  _render();
  _bindEvents();
  console.log(TAG, '拉花参数控制就绪 pattern=' + (_state.patternType || '(episode)') + ' cup=(' + _state.cupX + ',' + _state.cupY + ',' + _state.cupZ + ')');
  return { getState: () => _state, on, preview: _preview, execute: _execute };
}
