// debug_panel.js — 调试面板：状态监控 · 运动控制 · IO · IK/FK · 话题/服务
import { ivgTransport } from './ivg_transport.js';
import { loadRecords, saveRecords } from './core/record_store.js';
import { quatToRpyDeg, rpyDegToQuat } from './core/tf-math.js';
import { $, escapeHtml } from './core/utils.js';
import { logBus } from './core/log-bus.js';
import { createPointCloudViewer } from './view3d/pointcloud_viewer.js';

const TAG = '[debug_panel]';

// ── 工具函数 ─────────────────────────────────────────────────────────────────

function ts() { return new Date().toLocaleTimeString('zh-CN', { hour12: false }); }

const _LOG_CSS = { info: '', ok: 'log-ok', err: 'log-err', warn: 'log-warn' };

function _logLine(id, cssClass, msg) {
  const el = $(id);
  if (!el) return;
  const spanCls = cssClass ? ' class="' + cssClass + '"' : '';
  const div = document.createElement('div');
  div.innerHTML = '<span class="log-time">[' + ts() + ']</span> <span' + spanCls + '></span>';
  div.querySelector(spanCls ? 'span.' + cssClass : 'span:last-child').textContent = msg;
  el.insertBefore(div, el.firstChild);
  while (el.children.length > 30) el.removeChild(el.lastChild);
}

function log(id, msg)   { _logLine(id, '', msg); }
function logOk(id, msg)  { _logLine(id, 'log-ok', msg); }
function logErr(id, msg) { _logLine(id, 'log-err', msg); }
function logWarn(id, msg){ _logLine(id, 'log-warn', msg); }

function fmtPose(pos, ori) {
  if (!pos) return '---';
  return 'X:' + (pos.x || 0).toFixed(4) + ' Y:' + (pos.y || 0).toFixed(4) + ' Z:' + (pos.z || 0).toFixed(4);
}

function setStatusDot(elId, val) {
  const el = $(elId);
  if (!el) return;
  el.classList.remove('green', 'red', 'yellow', 'gray');
  el.classList.add(val ? 'green' : 'red');
}

function setTextIfPresent(elId, text) {
  const el = $(elId);
  if (el) el.textContent = text;
}

function updateConnStatus(connected) {
  const el = $('dbg-conn-status');
  if (!el) return;
  el.textContent = connected ? '● 已连接' : '● 未连接';
  el.className = 'conn-status ' + (connected ? 'online' : 'offline');
}

function callSvc(service, type, request, timeoutMs) {
  if (!service || !type) return Promise.reject(new Error('服务名和类型不能为空'));
  return ivgTransport.callService({
    service: service,
    type: type,
    request: request || {},
    timeoutMs: timeoutMs || 15000,
  });
}

// ── 机器人状态监控 ───────────────────────────────────────────────────────────

let _latestRobotStatus = null;
let _robotStatusRetries = 0;
let _robotStatusSubscribed = false;

function setupRobotStatus() {
  if (!ivgTransport.isConnected()) {
    _robotStatusRetries++;
    if (_robotStatusRetries > 30) {
      logBus.addLog('warn', 'system', 'setupRobotStatus 重试超限 (30次), 放弃订阅 /robot_status');
      return;
    }
    setTimeout(setupRobotStatus, 500);
    return;
  }

  _robotStatusRetries = 0;

  // 防止 onRosJson 重复注册
  if (!_robotStatusSubscribed) {
    _robotStatusSubscribed = true;
    ivgTransport.onRosJson('/robot_status', function (msg) {
      _latestRobotStatus = msg;
      updateStatusBar(msg);
    }, 'debug_panel');
  }

  ivgTransport.subscribe({
    topic: '/robot_status',
    msgType: 'ivg_interfaces/msg/RobotStatus',
    maxHz: 10,
  });

  log('debug-log-quick', '已订阅 /robot_status (10Hz)');
}

function updateStatusBar(msg) {
  // online dot
  setStatusDot('dbg-dot-online', msg.is_online);
  setTextIfPresent('dbg-val-online', msg.is_online ? '在线' : '离线');

  // enable dot
  setStatusDot('dbg-dot-enable', msg.enable);
  setTextIfPresent('dbg-val-enable', msg.enable ? '已使能' : '未使能');

  // motion dot
  const motionEl = $('dbg-dot-motion');
  if (motionEl) {
    motionEl.classList.remove('green', 'red', 'yellow', 'gray');
    motionEl.classList.add(msg.in_motion ? 'yellow' : 'gray');
  }
  setTextIfPresent('dbg-val-motion', msg.in_motion ? '运动中' : '静止');

  // planning
  setTextIfPresent('dbg-val-planning', msg.planning_status || '--');

  // TCP pose
  if (msg.cartesian_position && msg.cartesian_position.position) {
    setTextIfPresent('dbg-val-tcp', fmtPose(msg.cartesian_position.position, msg.cartesian_position.orientation));
  }
}

// ── 快速控制 ─────────────────────────────────────────────────────────────────

function setupQuickControls() {
  // 使能
  $('btn-dbg-enable').addEventListener('click', function () {
    log('debug-log-quick', '发送使能请求...');
    callSvc('/set_robot_enable', 'ivg_interfaces/srv/SetRobotEnable', { enable: true })
      .then(function (r) { logOk('debug-log-quick', '使能 OK: ' + (r.message || '')); })
      .catch(function (e) { logErr('debug-log-quick', '使能失败: ' + (e.message || e)); });
  });

  // 去使能
  $('btn-dbg-disable').addEventListener('click', function () {
    log('debug-log-quick', '发送去使能请求...');
    callSvc('/set_robot_enable', 'ivg_interfaces/srv/SetRobotEnable', { enable: false })
      .then(function (r) { logOk('debug-log-quick', '去使能 OK: ' + (r.message || '')); })
      .catch(function (e) { logErr('debug-log-quick', '去使能失败: ' + (e.message || e)); });
  });

  // 停止
  $('btn-dbg-stop').addEventListener('click', function () {
    log('debug-log-quick', '发送停止请求...');
    callSvc('/aubo/stop', 'std_srvs/srv/Trigger', {}, 5000)
      .then(function (r) { logOk('debug-log-quick', '停止 OK'); })
      .catch(function (e) { logWarn('debug-log-quick', '停止失败: ' + (e.message || e)); });
  });

  // 碰撞恢复
  $('btn-dbg-collision-recover').addEventListener('click', function () {
    log('debug-log-quick', '碰撞恢复...');
    callSvc('/aubo/collision_recover', 'std_srvs/srv/Trigger', {}, 5000)
      .then(function (r) { logOk('debug-log-quick', '碰撞恢复 OK'); })
      .catch(function (e) { logErr('debug-log-quick', '碰撞恢复失败: ' + (e.message || e)); });
  });

  // 速度滑动条
  var slider = $('dbg-speed-slider');
  var valEl = $('dbg-speed-val');
  slider.addEventListener('input', function () {
    valEl.textContent = slider.value + '%';
  });

  $('btn-dbg-set-speed').addEventListener('click', function () {
    var frac = parseInt(slider.value) / 100;
    log('debug-log-quick', '设置速度倍率: ' + frac.toFixed(2));
    callSvc('/set_speed_factor', 'ivg_interfaces/srv/SetSpeedFactor', { velocity_factor: frac })
      .then(function (r) { logOk('debug-log-quick', '速度已设为 ' + (frac * 100).toFixed(0) + '%'); })
      .catch(function (e) { logErr('debug-log-quick', '设置速度失败: ' + (e.message || e)); });
  });
}

// ── 笛卡尔直线运动 ───────────────────────────────────────────────────────────

// RPY(deg) → 四元数，同步输入框
function _rpyToQuatInputs() {
  var r = parseFloat($('dbg-xyz-r').value) || 0;
  var p = parseFloat($('dbg-xyz-p').value) || 0;
  var y = parseFloat($('dbg-xyz-yaw').value) || 0;
  var q = rpyDegToQuat(r, p, y);
  $('dbg-xyz-qx').value = q.x.toFixed(6);
  $('dbg-xyz-qy').value = q.y.toFixed(6);
  $('dbg-xyz-qz').value = q.z.toFixed(6);
  $('dbg-xyz-qw').value = q.w.toFixed(6);
}

// 四元数 → RPY(deg)，同步输入框
function _quatToRpyInputs() {
  var q = {
    x: parseFloat($('dbg-xyz-qx').value) || 0,
    y: parseFloat($('dbg-xyz-qy').value) || 0,
    z: parseFloat($('dbg-xyz-qz').value) || 0,
    w: parseFloat($('dbg-xyz-qw').value) || 1,
  };
  var rpy = quatToRpyDeg(q);
  $('dbg-xyz-r').value = rpy.roll.toFixed(2);
  $('dbg-xyz-p').value = rpy.pitch.toFixed(2);
  $('dbg-xyz-yaw').value = rpy.yaw.toFixed(2);
}

var _rpySyncing = false;  // 防递归

function setupCartesian() {
  $('btn-dbg-move-xyz').addEventListener('click', function () {
    var x = parseFloat($('dbg-xyz-x').value) || 0;
    var y = parseFloat($('dbg-xyz-y').value) || 0;
    var z = parseFloat($('dbg-xyz-z').value) || 0;
    var qx = parseFloat($('dbg-xyz-qx').value) || 0;
    var qy = parseFloat($('dbg-xyz-qy').value) || 0;
    var qz = parseFloat($('dbg-xyz-qz').value) || 0;
    var qw = parseFloat($('dbg-xyz-qw').value) || 1;
    var vel = parseFloat($('dbg-xyz-vel').value) || 0.3;
    var acc = parseFloat($('dbg-xyz-acc').value) || 0.2;

    var req = {
      target_pose: {
        position: { x: x, y: y, z: z },
        orientation: { x: qx, y: qy, z: qz, w: qw },
      },
      target_joints: [0, 0, 0, 0, 0, 0],
      use_joints: false,
      velocity_factor: vel,
      acceleration_factor: acc,
    };

    log('debug-log-cartesian', 'MoveL(' + x.toFixed(4) + ', ' + y.toFixed(4) + ', ' + z.toFixed(4) + ') v=' + vel + ' a=' + acc);
    callSvc('/move_to_pose', 'ivg_interfaces/srv/MoveToPose', req, 30000)
      .then(function (r) { logOk('debug-log-cartesian', 'OK: ' + (r.message || '')); })
      .catch(function (e) { logErr('debug-log-cartesian', 'FAIL: ' + (e.message || e)); });
  });

  // RPY 输入 → 自动更新四元数
  ['dbg-xyz-r', 'dbg-xyz-p', 'dbg-xyz-yaw'].forEach(function (id) {
    $(id).addEventListener('input', function () {
      if (_rpySyncing) return;
      _rpySyncing = true;
      _rpyToQuatInputs();
      _rpySyncing = false;
    });
  });

  // 四元数输入 → 自动更新 RPY
  ['dbg-xyz-qx', 'dbg-xyz-qy', 'dbg-xyz-qz', 'dbg-xyz-qw'].forEach(function (id) {
    $(id).addEventListener('input', function () {
      if (_rpySyncing) return;
      _rpySyncing = true;
      _quatToRpyInputs();
      _rpySyncing = false;
    });
  });

  // 从 robot_status 读取当前位置填入 XYZ
  $('btn-dbg-read-xyz').addEventListener('click', function () {
    if (!_latestRobotStatus || !_latestRobotStatus.cartesian_position) {
      logWarn('debug-log-cartesian', '尚未收到 /robot_status，无法读取位姿');
      return;
    }
    var cp = _latestRobotStatus.cartesian_position;
    var pos = cp.position;
    var ori = cp.orientation;
    $('dbg-xyz-x').value = (pos.x || 0).toFixed(4);
    $('dbg-xyz-y').value = (pos.y || 0).toFixed(4);
    $('dbg-xyz-z').value = (pos.z || 0).toFixed(4);
    if (ori) {
      _rpySyncing = true;
      $('dbg-xyz-qx').value = (ori.x || 0).toFixed(4);
      $('dbg-xyz-qy').value = (ori.y || 0).toFixed(4);
      $('dbg-xyz-qz').value = (ori.z || 0).toFixed(4);
      $('dbg-xyz-qw').value = (ori.w || 1).toFixed(4);
      _quatToRpyInputs();
      _rpySyncing = false;
    }
    logOk('debug-log-cartesian', '已从 /robot_status 读取当前位姿');
  });
}

// ── 关节空间运动 ─────────────────────────────────────────────────────────────

function setupJointMotion() {
  $('btn-dbg-move-joints').addEventListener('click', function () {
    var joints = [];
    for (var i = 1; i <= 6; i++) {
      joints.push(parseFloat($('dbg-j' + i).value) || 0);
    }
    var vel = parseFloat($('dbg-jnt-vel').value) || 0.3;

    // /move_to_pose 内部委托 RobotController::moveToJoints (setJointValueTarget → plan → execute)
    var req = {
      target_pose: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
      target_joints: joints,
      use_joints: true,
      velocity_factor: vel,
      acceleration_factor: 0.2,
    };

    log('debug-log-joint', 'MoveJ([' + joints.map(function (v) { return v.toFixed(4); }).join(', ') + ']) v=' + vel);
    callSvc('/move_to_pose', 'ivg_interfaces/srv/MoveToPose', req, 30000)
      .then(function (r) { logOk('debug-log-joint', 'OK: ' + (r.message || '')); })
      .catch(function (e) { logErr('debug-log-joint', 'FAIL: ' + (e.message || e)); });
  });

  $('btn-dbg-read-joints').addEventListener('click', function () {
    if (!_latestRobotStatus || !_latestRobotStatus.joint_position_deg) {
      // fallback: try service call
      log('debug-log-joint', '通过服务读取当前关节角...');
      callSvc('/get_current_state', 'ivg_interfaces/srv/GetCurrentState', {}, 5000)
        .then(function (r) {
          var pos = r.joint_position_rad || [];
          for (var i = 0; i < Math.min(6, pos.length); i++) {
            var inp = $('dbg-j' + (i + 1));
            if (inp) inp.value = pos[i].toFixed(4);
          }
          logOk('debug-log-joint', '当前关节角(rad): [' + pos.map(function (v) { return v.toFixed(4); }).join(', ') + ']');
        })
        .catch(function (e) { logErr('debug-log-joint', '读取失败: ' + (e.message || e)); });
      return;
    }
    var degs = _latestRobotStatus.joint_position_deg;
    // robot_status gives degrees, but joint motion uses radians
    var rads = _latestRobotStatus.joint_position_rad;
    for (var i = 0; i < Math.min(6, (rads || degs).length); i++) {
      var inp = $('dbg-j' + (i + 1));
      if (inp) inp.value = (rads ? rads[i] : degs[i] * Math.PI / 180).toFixed(4);
    }
    logOk('debug-log-joint', '已从 /robot_status 读取当前关节角');
  });
}

// ── IO 控制 ──────────────────────────────────────────────────────────────────

function setupIOControl() {
  // Tab 切换
  var tabs = document.querySelectorAll('#io-tabs .debug-tab');
  tabs.forEach(function (tab) {
    tab.addEventListener('click', function () {
      tabs.forEach(function (t) { t.classList.remove('active'); });
      tab.classList.add('active');
      var panelName = tab.getAttribute('data-tab');
      document.getElementById('io-panel-read').style.display = panelName === 'io-read' ? '' : 'none';
      document.getElementById('io-panel-write').style.display = panelName === 'io-write' ? '' : 'none';
      document.getElementById('io-panel-tool').style.display = panelName === 'io-tool' ? '' : 'none';
    });
  });

  // 读取 IO
  $('btn-dbg-read-io').addEventListener('click', function () {
    var ioType = $('dbg-io-read-type').value;
    var pin = parseInt($('dbg-io-read-pin').value) || 0;
    log('debug-log-io-read', '读取 ' + ioType + ' IO pin=' + pin);
    callSvc('/read_robot_io', 'ivg_interfaces/srv/ReadRobotIO', { io_type: ioType, io_index: pin })
      .then(function (r) {
        logOk('debug-log-io-read', ioType + ' pin=' + pin + ' value=' + r.value + ' (' + (r.message || '') + ')');
      })
      .catch(function (e) { logErr('debug-log-io-read', '读取失败: ' + (e.message || e)); });
  });

  // 设置 DO
  $('btn-dbg-write-io').addEventListener('click', function () {
    var pin = parseInt($('dbg-io-write-pin').value) || 0;
    var val = parseInt($('dbg-io-write-val').value) || 0;
    log('debug-log-io-write', '设置 DO pin=' + pin + ' val=' + val);
    callSvc('/aubo_driver/set_io', 'ivg_interfaces/srv/SetRobotIO', { io_type: 'digital_output', io_index: pin, value: val })
      .then(function (r) {
        logOk('debug-log-io-write', 'DO pin=' + pin + ' → ' + (val ? 'ON' : 'OFF') + ' OK');
      })
      .catch(function (e) { logErr('debug-log-io-write', '设置失败: ' + (e.message || e)); });
  });

  // 设置工具电压
  $('btn-dbg-set-tool-voltage').addEventListener('click', function () {
    var vtype = parseInt($('dbg-tool-voltage').value) || 0;
    var vlabel = {0: '0V', 1: '12V', 2: '24V'}[vtype] || (vtype + 'V');
    log('debug-log-tool-voltage', '设置工具电压: ' + vlabel);
    callSvc('/aubo/set_tool_voltage', 'ivg_interfaces/srv/SetToolVoltage', { voltage_type: vtype })
      .then(function (r) { logOk('debug-log-tool-voltage', '工具电压设为 ' + vlabel + ' OK'); })
      .catch(function (e) { logErr('debug-log-tool-voltage', '设置失败: ' + (e.message || e)); });
  });
}

// ── IK/FK 计算器 ─────────────────────────────────────────────────────────────

function setupIKFK() {
  // Tab 切换
  var tabs = document.querySelectorAll('#ikfk-tabs .debug-tab');
  tabs.forEach(function (tab) {
    tab.addEventListener('click', function () {
      tabs.forEach(function (t) { t.classList.remove('active'); });
      tab.classList.add('active');
      var panelName = tab.getAttribute('data-tab');
      document.getElementById('ikfk-panel-fk').style.display = panelName === 'ikfk-fk' ? '' : 'none';
      document.getElementById('ikfk-panel-ik').style.display = panelName === 'ikfk-ik' ? '' : 'none';
    });
  });

  // FK 计算
  $('btn-dbg-calc-fk').addEventListener('click', function () {
    var joints = [];
    for (var i = 1; i <= 6; i++) {
      joints.push(parseFloat($('dbg-fk-j' + i).value) || 0);
    }
    log('debug-log-fk', 'FK([' + joints.map(function (v) { return v.toFixed(4); }).join(', ') + '])');
    callSvc('/aubo_driver/get_fk', 'ivg_interfaces/srv/GetFK', { joint: joints })
      .then(function (r) {
        var pos = r.pos || [];
        var ori = r.ori || [];  // AUBO SDK: (w,x,y,z)
        var result = 'pos=[' + pos.map(function (v) { return v.toFixed(6); }).join(', ') + '] '
          + 'ori(wxyz)=[' + ori.map(function (v) { return v.toFixed(6); }).join(', ') + ']';
        logOk('debug-log-fk', result);
      })
      .catch(function (e) { logErr('debug-log-fk', 'FK失败: ' + (e.message || e)); });
  });

  // FK 从当前关节填入
  $('btn-dbg-fk-from-current').addEventListener('click', function () {
    var rads = _latestRobotStatus ? _latestRobotStatus.joint_position_rad : null;
    if (!rads || rads.length < 6) {
      logWarn('debug-log-fk', '尚未收到 /robot_status，无法获取当前关节');
      return;
    }
    for (var i = 0; i < 6; i++) {
      var inp = $('dbg-fk-j' + (i + 1));
      if (inp) inp.value = rads[i].toFixed(4);
    }
    logOk('debug-log-fk', '已从 /robot_status 填入当前关节角');
  });

  // IK 计算
  $('btn-dbg-calc-ik').addEventListener('click', function () {
    var refJoints = [];
    for (var i = 1; i <= 6; i++) {
      refJoints.push(parseFloat($('dbg-ik-rj' + i).value) || 0);
    }
    var x = parseFloat($('dbg-ik-x').value) || 0;
    var y = parseFloat($('dbg-ik-y').value) || 0;
    var z = parseFloat($('dbg-ik-z').value) || 0;
    var qw = parseFloat($('dbg-ik-qw').value) || 1;
    var qx = parseFloat($('dbg-ik-qx').value) || 0;
    var qy = parseFloat($('dbg-ik-qy').value) || 0;
    var qz = parseFloat($('dbg-ik-qz').value) || 0;

    var req = {
      ref_joint: refJoints,
      pos: [x, y, z],
      ori: [qw, qx, qy, qz],  // AUBO SDK: (w,x,y,z)
    };

    log('debug-log-ik', 'IK(pos=[' + x + ',' + y + ',' + z + '] ori(wxyz)=[' + qw + ',' + qx + ',' + qy + ',' + qz + '])');
    callSvc('/aubo_driver/get_ik', 'ivg_interfaces/srv/GetIK', req)
      .then(function (r) {
        var joints = r.joint || [];
        if (joints.length >= 6) {
          logOk('debug-log-ik', 'IK解(rad): [' + joints.map(function (v) { return v.toFixed(6); }).join(', ') + ']');
        } else {
          logWarn('debug-log-ik', 'IK返回结果不足6个关节');
        }
      })
      .catch(function (e) { logErr('debug-log-ik', 'IK失败: ' + (e.message || e)); });
  });

  // IK 从当前关节填入参考
  $('btn-dbg-ik-ref-from-current').addEventListener('click', function () {
    var rads = _latestRobotStatus ? _latestRobotStatus.joint_position_rad : null;
    if (!rads || rads.length < 6) {
      logWarn('debug-log-ik', '尚未收到 /robot_status，无法获取当前关节');
      return;
    }
    for (var i = 0; i < 6; i++) {
      var inp = $('dbg-ik-rj' + (i + 1));
      if (inp) inp.value = rads[i].toFixed(4);
    }
    logOk('debug-log-ik', '已从 /robot_status 填入参考关节角');
  });
}

// ── 话题监控 ─────────────────────────────────────────────────────────────────

let _monitorHandlerId = null;
let _monitorTopic = null;
let _monitorCount = 0;

function setupTopicMonitor() {
  $('btn-dbg-sub-start').addEventListener('click', function () {
    var topic = ($('dbg-sub-topic').value || '').trim();
    var type = ($('dbg-sub-type').value || '').trim();
    if (!topic || !type) {
      logErr('debug-log-topic-monitor', '请输入话题名和消息类型');
      return;
    }

    // 取消旧订阅
    if (_monitorHandlerId !== null) {
      ivgTransport.unsubscribe(_monitorTopic);
    }

    _monitorTopic = topic;
    _monitorCount = 0;
    $('dbg-sub-count').textContent = '消息: 0';

    var handler = function (msg) {
      _monitorCount++;
      $('dbg-sub-count').textContent = '消息: ' + _monitorCount;
      var el = $('debug-log-topic-monitor');
      if (!el) return;
      // 移除空状态提示
      var empty = el.querySelector('.debug-empty');
      if (empty) empty.remove();
      var jsonStr = JSON.stringify(msg, null, 2);
      var line = '<div><span class="log-time">[' + ts() + ']</span> <pre style="margin:0;font-size:0.7rem;white-space:pre-wrap;">' + escapeHtml(jsonStr) + '</pre></div>';
      el.innerHTML = line + el.innerHTML;
      while (el.children.length > 100) el.removeChild(el.lastChild);
    };
    _monitorHandlerId = handler;

    // 清除旧监控 handler 并重新注册 /robot_status + 新监控 handler
    ivgTransport.clearRosHandlersByOwner('debug_panel');
    _robotStatusSubscribed = false;
    setupRobotStatus();
    ivgTransport.onRosJson(topic, handler, 'debug_panel');
    ivgTransport.subscribe({ topic: topic, msgType: type, maxHz: 5 });
    logOk('debug-log-topic-monitor', '已订阅: ' + topic + ' (' + type + ')');
  });

  $('btn-dbg-sub-stop').addEventListener('click', function () {
    if (_monitorTopic) {
      ivgTransport.unsubscribe(_monitorTopic);
      logWarn('debug-log-topic-monitor', '已取消订阅: ' + _monitorTopic);
    }
    _monitorHandlerId = null;
    _monitorTopic = null;
    _monitorCount = 0;
    $('dbg-sub-count').textContent = '消息: 0';
    // 重新注册 /robot_status handler（清除监控 handler 后）喵~
    ivgTransport.clearRosHandlersByOwner('debug_panel');
    _robotStatusSubscribed = false;
    setupRobotStatus();
  });
}

// ── 服务调用测试 ─────────────────────────────────────────────────────────────

function setupServiceCall() {
  // 预设选择
  $('dbg-svc-preset').addEventListener('change', function () {
    var val = this.value;
    if (!val) return;
    var parts = val.split('|');
    if (parts.length >= 3) {
      $('dbg-svc-name').value = parts[0];
      $('dbg-svc-type').value = parts[1];
      $('dbg-svc-args').value = parts[2];
    }
  });

  $('btn-dbg-call-svc').addEventListener('click', function () {
    var svc = ($('dbg-svc-name').value || '').trim();
    var type = ($('dbg-svc-type').value || '').trim();
    var argsStr = ($('dbg-svc-args').value || '{}').trim();

    if (!svc || !type) { logErr('debug-log-service', '请输入服务名和类型'); return; }

    var args;
    try { args = JSON.parse(argsStr); }
    catch (e) { logErr('debug-log-service', 'JSON 解析错误: ' + e.message); return; }

    log('debug-log-service', 'call ' + svc + ' (' + type + ') ' + JSON.stringify(args));
    callSvc(svc, type, args, 15000)
      .then(function (r) { logOk('debug-log-service', svc + ' OK: ' + JSON.stringify(r, null, 2)); })
      .catch(function (e) { logErr('debug-log-service', svc + ' FAIL: ' + (e.message || e)); });
  });
}

// ── 话题发布测试 ─────────────────────────────────────────────────────────────

function setupTopicPublish() {
  $('btn-dbg-publish').addEventListener('click', function () {
    var topic = ($('dbg-pub-topic').value || '').trim();
    var type = ($('dbg-pub-type').value || '').trim();
    var msgStr = ($('dbg-pub-msg').value || '{}').trim();

    if (!topic || !type) { logErr('debug-log-topic', '请输入话题名和消息类型'); return; }

    var msg;
    try { msg = JSON.parse(msgStr); }
    catch (e) { logErr('debug-log-topic', 'JSON 解析错误: ' + e.message); return; }

    log('debug-log-topic', 'publish ' + topic + ' (' + type + ') ' + JSON.stringify(msg));
    ivgTransport.publish({ topic: topic, type: type, msg: msg })
      .then(function () { logOk('debug-log-topic', 'OK → 已发布到 ' + topic); })
      .catch(function (e) { logErr('debug-log-topic', 'FAIL: ' + (e.message || e)); });
  });
}

// ── 记录快照加载 ─────────────────────────────────────────────────────────────

let _cachedRecords = [];

function refreshRecordSelect() {
  var sel = $('dbg-record-select');
  if (!sel) return;
  _cachedRecords = loadRecords();
  var curVal = sel.value;
  sel.innerHTML = '<option value="">-- 请选择记录 --</option>';
  if (_cachedRecords.length === 0) {
    sel.innerHTML += '<option value="" disabled>-- 暂无记录 --</option>';
  } else {
    _cachedRecords.forEach(function (r, i) {
      var name = r.name || ('快照 #' + (i + 1));
      var t = (r.timestamp || '').replace('T', ' ').slice(0, 16);
      var opt = document.createElement('option');
      opt.value = String(i);
      opt.textContent = name + ' (' + t + ')';
      sel.appendChild(opt);
    });
  }
  // 恢复先前选中项
  if (curVal && parseInt(curVal) < _cachedRecords.length) {
    sel.value = curVal;
  }
  onSelectRecord();
}

function onSelectRecord() {
  var sel = $('dbg-record-select');
  var detail = $('dbg-record-detail');
  var btnJ = $('btn-dbg-load-joints');
  var btnP = $('btn-dbg-load-pose');
  if (!sel) return;

  var idx = parseInt(sel.value);
  if (isNaN(idx) || idx < 0 || idx >= _cachedRecords.length) {
    if (detail) detail.innerHTML = '<span class="debug-empty">选择记录查看详情</span>';
    if (btnJ) btnJ.disabled = true;
    if (btnP) btnP.disabled = true;
    return;
  }

  var r = _cachedRecords[idx];
  if (btnJ) btnJ.disabled = false;
  if (btnP) btnP.disabled = false;

  if (!detail) return;
  var rads = (r.joints || []).map(function (j) { return j.position_rad != null ? j.position_rad.toFixed(4) : '--'; });
  var pos = r.end_effector && r.end_effector.position_m ? r.end_effector.position_m : {};
  var rpy = r.end_effector && r.end_effector.euler_rpy_deg ? r.end_effector.euler_rpy_deg : {};

  detail.innerHTML =
    '<div class="live-item"><span class="live-key">模式</span><span class="live-val">' + (r.mode || 'unknown') + '</span></div>'
    + '<div class="live-item"><span class="live-key">关节(rad)</span><span class="live-val">' + rads.join(', ') + '</span></div>'
    + '<div class="live-item"><span class="live-key">XYZ(m)</span><span class="live-val">X:' + ((pos.x != null ? pos.x : 0).toFixed(3)) + ' Y:' + ((pos.y != null ? pos.y : 0).toFixed(3)) + ' Z:' + ((pos.z != null ? pos.z : 0).toFixed(3)) + '</span></div>'
    + '<div class="live-item"><span class="live-key">RPY(°)</span><span class="live-val">R:' + ((rpy.roll != null ? rpy.roll : 0).toFixed(1)) + ' P:' + ((rpy.pitch != null ? rpy.pitch : 0).toFixed(1)) + ' Y:' + ((rpy.yaw != null ? rpy.yaw : 0).toFixed(1)) + '</span></div>';
}

function onLoadJoints() {
  var sel = $('dbg-record-select');
  if (!sel) return;
  var idx = parseInt(sel.value);
  if (isNaN(idx) || idx < 0 || idx >= _cachedRecords.length) {
    logWarn('debug-log-record', '请先选择一条记录');
    return;
  }
  var r = _cachedRecords[idx];
  for (var i = 0; i < 6; i++) {
    var inp = $('dbg-j' + (i + 1));
    if (inp && r.joints[i] && r.joints[i].position_rad != null) {
      inp.value = r.joints[i].position_rad.toFixed(4);
    }
  }
  logOk('debug-log-record', '已加载关节角');
}

function onLoadPose() {
  var sel = $('dbg-record-select');
  if (!sel) return;
  var idx = parseInt(sel.value);
  if (isNaN(idx) || idx < 0 || idx >= _cachedRecords.length) {
    logWarn('debug-log-record', '请先选择一条记录');
    return;
  }
  var r = _cachedRecords[idx];
  var pos = r.end_effector && r.end_effector.position_m ? r.end_effector.position_m : {};
  var ori = r.end_effector && r.end_effector.orientation_quaternion ? r.end_effector.orientation_quaternion : {};
  $('dbg-xyz-x').value = pos.x != null ? pos.x.toFixed(4) : '0.0000';
  $('dbg-xyz-y').value = pos.y != null ? pos.y.toFixed(4) : '0.0000';
  $('dbg-xyz-z').value = pos.z != null ? pos.z.toFixed(4) : '0.0000';
  $('dbg-xyz-qx').value = ori.x != null ? ori.x.toFixed(4) : '0.0000';
  $('dbg-xyz-qy').value = ori.y != null ? ori.y.toFixed(4) : '0.0000';
  $('dbg-xyz-qz').value = ori.z != null ? ori.z.toFixed(4) : '0.0000';
  $('dbg-xyz-qw').value = ori.w != null ? ori.w.toFixed(4) : '1.0000';
  _quatToRpyInputs();
  logOk('debug-log-record', '已加载位姿');
}

// 将 waypoint 数组规范化为 record 格式并写入 store，返回导入条数
function _normalizeAndSave(data, baseName) {
  var records = loadRecords();
  var added = 0;
  data.forEach(function (wp) {
    var record = {
      name: wp.name || (baseName + ' #' + (records.length + added + 1)),
      timestamp: wp.timestamp || new Date().toISOString(),
      mode: wp.mode || 'unknown',
      joints: (wp.joints || []).map(function (j) {
        return { name: j.name || '', position_rad: j.position_rad || 0, position_deg: j.position_deg || 0 };
      }),
      end_effector: wp.end_effector || {
        position_m: { x: 0, y: 0, z: 0 },
        orientation_quaternion: { x: 0, y: 0, z: 0, w: 1 },
        euler_rpy_deg: { roll: 0, pitch: 0, yaw: 0 },
      },
    };
    records.push(record);
    added++;
  });
  return { ok: saveRecords(records), added: added };
}

function importJsonFile(file) {
  var reader = new FileReader();
  reader.onload = function (e) {
    var data;
    try { data = JSON.parse(e.target.result); }
    catch (err) { logErr('debug-log-record', 'JSON 解析失败: ' + err.message); return; }

    if (!Array.isArray(data)) {
      logErr('debug-log-record', 'JSON 格式错误: 期望顶层为数组');
      return;
    }

    var result = _normalizeAndSave(data, '导入');
    if (result.ok) {
      logOk('debug-log-record', '已导入 ' + result.added + ' 条记录');
      refreshRecordSelect();
    } else {
      logErr('debug-log-record', '保存失败 (localStorage 可能已满)');
    }
  };
  reader.readAsText(file);
}

function pasteJsonText() {
  var textarea = $('dbg-paste-textarea');
  if (!textarea) return;
  var raw = (textarea.value || '').trim();
  if (!raw) { logWarn('debug-log-record', '请先粘贴 JSON 内容'); return; }

  var data;
  try { data = JSON.parse(raw); }
  catch (err) { logErr('debug-log-record', 'JSON 解析失败: ' + err.message); return; }

  // 统一为数组格式
  var items = Array.isArray(data) ? data : [data];

  var result = _normalizeAndSave(items, '粘贴');
  if (result.ok) {
    logOk('debug-log-record', '已解析并导入 ' + result.added + ' 条记录');
    refreshRecordSelect();
    // 选中第一条新导入的记录
    var sel = $('dbg-record-select');
    if (sel && _cachedRecords.length >= result.added) {
      sel.value = String(_cachedRecords.length - result.added);
      onSelectRecord();
    }
  } else {
    logErr('debug-log-record', '保存失败 (localStorage 可能已满)');
  }
}

function setupRecordLoader() {
  refreshRecordSelect();
  $('dbg-record-select').addEventListener('change', onSelectRecord);
  $('btn-dbg-load-joints').addEventListener('click', onLoadJoints);
  $('btn-dbg-load-pose').addEventListener('click', onLoadPose);

  // ── 位姿导入（顶部工具栏 + 记录卡底部）────────────────────────────────

  // 顶部 toolbar 版本
  var fileInputTop = $('dbg-import-file-input-top');
  if (fileInputTop) {
    $('btn-dbg-import-json-top').addEventListener('click', function () { fileInputTop.click(); });
    fileInputTop.addEventListener('change', function () {
      if (fileInputTop.files && fileInputTop.files.length > 0) {
        log('debug-log-record', '正在导入: ' + fileInputTop.files[0].name + ' ...');
        importJsonFile(fileInputTop.files[0]);
        fileInputTop.value = '';
      }
    });
  }

  var pasteAreaTop = $('dbg-paste-area-top');
  if (pasteAreaTop) {
    $('btn-dbg-toggle-paste-top').addEventListener('click', function () {
      var show = pasteAreaTop.style.display === 'none';
      pasteAreaTop.style.display = show ? '' : 'none';
      this.textContent = show ? '收起粘贴' : '粘贴 JSON';
      if (show) $('dbg-paste-textarea-top').focus();
    });
    $('btn-dbg-paste-parse-top').addEventListener('click', function () {
      // 临时替换 textarea 引用，复用 pasteJsonText
      var origTextarea = $('dbg-paste-textarea');
      var topTextarea = $('dbg-paste-textarea-top');
      // 直接读取顶部 textarea 的值来复用解析逻辑
      var raw = (topTextarea.value || '').trim();
      if (!raw) { logWarn('debug-log-record', '请先粘贴 JSON 内容'); return; }
      var data;
      try { data = JSON.parse(raw); }
      catch (err) { logErr('debug-log-record', 'JSON 解析失败: ' + err.message); return; }
      var items = Array.isArray(data) ? data : [data];
      var result = _normalizeAndSave(items, '粘贴');
      if (result.ok) {
        logOk('debug-log-record', '已解析并导入 ' + result.added + ' 条记录');
        refreshRecordSelect();
        var sel = $('dbg-record-select');
        if (sel && _cachedRecords.length >= result.added) {
          sel.value = String(_cachedRecords.length - result.added);
          onSelectRecord();
        }
      } else {
        logErr('debug-log-record', '保存失败 (localStorage 可能已满)');
      }
    });
    $('btn-dbg-paste-clear-top').addEventListener('click', function () {
      $('dbg-paste-textarea-top').value = '';
      log('debug-log-record', '已清空粘贴内容');
    });
  }

  // 记录卡底部版本
  var fileInput = $('dbg-import-file-input');
  $('btn-dbg-import-json').addEventListener('click', function () {
    fileInput.click();
  });
  fileInput.addEventListener('change', function () {
    if (fileInput.files && fileInput.files.length > 0) {
      log('debug-log-record', '正在导入: ' + fileInput.files[0].name + ' ...');
      importJsonFile(fileInput.files[0]);
      fileInput.value = '';  // 允许重复导入同一文件
    }
  });

  var pasteArea = $('dbg-paste-area');
  $('btn-dbg-toggle-paste').addEventListener('click', function () {
    var show = pasteArea.style.display === 'none';
    pasteArea.style.display = show ? '' : 'none';
    this.textContent = show ? '收起粘贴' : '粘贴 JSON';
    if (show) $('dbg-paste-textarea').focus();
  });
  $('btn-dbg-paste-parse').addEventListener('click', pasteJsonText);
  $('btn-dbg-paste-clear').addEventListener('click', function () {
    $('dbg-paste-textarea').value = '';
    log('debug-log-record', '已清空粘贴内容');
  });

  window.addEventListener('storage', function (e) {
    if (e.key === 'ivg_monitor_records') refreshRecordSelect();
  });
}

// ── 初始化 ───────────────────────────────────────────────────────────────────

async function init() {
  logBus.addLog('info', 'lifecycle', '调试面板初始化中...');

  // 1. 加载运行时 & 连接
  try {
    await ivgTransport.loadRuntime();
    await ivgTransport.connectControl();
    updateConnStatus(true);
    logBus.addLog('info', 'rosbridge', 'rosbridge 已连接 (调试面板)');

    // 连接断开监听
    ivgTransport.onControlJson(function (obj) {
      if (obj.op === 'close') {
        updateConnStatus(false);
        logBus.addLog('warn', 'rosbridge', 'rosbridge 断开 (调试面板)');
      } else if (obj.op === 'connection') {
        updateConnStatus(true);
        setupRobotStatus();
      }
    }, 'debug_panel');
  } catch (e) {
    logBus.addLog('error', 'rosbridge', '连接失败 (调试面板): ' + (e.message || e));
    updateConnStatus(false);
    logErr('debug-log-quick', 'rosbridge 连接失败: ' + (e.message || e));
    // 继续初始化，允许离线查看页面
  }

  // 2. 机器人状态监控
  if (ivgTransport.isConnected()) {
    setupRobotStatus();
  }

  // 3. 各功能模块
  setupQuickControls();
  setupCartesian();
  setupJointMotion();
  setupIOControl();
  setupIKFK();
  setupTopicMonitor();
  setupServiceCall();
  setupTopicPublish();
  setupRecordLoader();

  // 4. 点云 3D 查看 (foxglove_bridge)
  setupPointCloudViewer();

  logBus.addLog('info', 'lifecycle', '调试面板初始化完成');
}


// ── 内嵌最小 OrbitController (离线可用, 零依赖) ──────────────────────────

function _createOrbitController(camera, domElement) {
  const target = new THREE.Vector3();
  const spherical = new THREE.Spherical();
  const panStart = new THREE.Vector2();
  let state = 0; // 0=none, 1=rotate, 2=pan
  let lastX = 0, lastY = 0;
  let rotateSpeed = 0.005, zoomSpeed = 0.08, panSpeed = 0.01;
  let minDist = 0.1, maxDist = 50;

  // 从当前相机位置计算球坐标
  spherical.setFromVector3(camera.position.clone().sub(target));
  camera.lookAt(target);

  function onMouseDown(e) {
    lastX = e.clientX; lastY = e.clientY;
    if (e.button === 0) state = 1;       // 左键: 旋转
    else if (e.button === 2) state = 2;  // 右键: 平移
  }

  function onMouseMove(e) {
    const dx = e.clientX - lastX, dy = e.clientY - lastY;
    lastX = e.clientX; lastY = e.clientY;

    if (state === 1) {
      spherical.theta -= dx * rotateSpeed;
      spherical.phi -= dy * rotateSpeed;
      spherical.phi = Math.max(0.01, Math.min(Math.PI - 0.01, spherical.phi));
    } else if (state === 2) {
      const panX = -dx * panSpeed * spherical.radius * 0.5;
      const panY = dy * panSpeed * spherical.radius * 0.5;
      const right = new THREE.Vector3();
      const up = new THREE.Vector3(0, 1, 0);
      right.crossVectors(camera.getWorldDirection(), up).normalize();
      target.add(right.multiplyScalar(panX));
      target.add(up.clone().multiplyScalar(panY));
    }
    _applySpherical();
  }

  function onMouseUp() { state = 0; }

  function onWheel(e) {
    spherical.radius *= (1 + (e.deltaY > 0 ? 0.05 : -0.05) * zoomSpeed);
    spherical.radius = Math.max(minDist, Math.min(maxDist, spherical.radius));
    _applySpherical();
    e.preventDefault();
  }

  function onContextMenu(e) { e.preventDefault(); }

  function _applySpherical() {
    camera.position.copy(target).add(
      new THREE.Vector3().setFromSpherical(spherical));
    camera.lookAt(target);
  }

  domElement.addEventListener('mousedown', onMouseDown);
  domElement.addEventListener('mousemove', onMouseMove);
  domElement.addEventListener('mouseup', onMouseUp);
  domElement.addEventListener('mouseleave', onMouseUp);
  domElement.addEventListener('wheel', onWheel, { passive: false });
  domElement.addEventListener('contextmenu', onContextMenu);

  return {
    update() {},  // 内嵌实现无需每帧 update
    setTarget(x, y, z) { target.set(x, y, z); _applySpherical(); },
    dispose() {
      domElement.removeEventListener('mousedown', onMouseDown);
      domElement.removeEventListener('mousemove', onMouseMove);
      domElement.removeEventListener('mouseup', onMouseUp);
      domElement.removeEventListener('mouseleave', onMouseUp);
      domElement.removeEventListener('wheel', onWheel);
      domElement.removeEventListener('contextmenu', onContextMenu);
    },
  };
}


// ── 点云 3D 查看 (foxglove_bridge CDR 二进制, 原生 Three.js) ──────────────────

let _pcv = null;           // point cloud viewer 实例
let _pcRenderer = null;    // Three.js WebGLRenderer
let _pcScene = null;       // THREE.Scene
let _pcCamera = null;      // THREE.PerspectiveCamera
let _pcControls = null;    // OrbitControls
let _pcAnimId = null;      // requestAnimationFrame id

function setupPointCloudViewer() {
  const THREE = globalThis.THREE;
  if (!THREE) {
    logWarn('debug-log-pointcloud', 'THREE 未加载, 点云功能不可用');
    return;
  }

  const vp = $('dbg-pointcloud-viewport');
  if (!vp) return;

  // ── 创建 Three.js 场景 ──
  _pcScene = new THREE.Scene();
  _pcScene.background = new THREE.Color(0x1a1a2e);

  // 相机
  const w = vp.clientWidth || 640;
  const h = vp.clientHeight || 420;
  _pcCamera = new THREE.PerspectiveCamera(55, w / h, 0.01, 50);
  _pcCamera.position.set(1.5, 1.0, 2.0);
  _pcCamera.lookAt(0, 0, 0.5);

  // 渲染器
  _pcRenderer = new THREE.WebGLRenderer({ antialias: true });
  _pcRenderer.setSize(w, h);
  _pcRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  _pcRenderer.shadowMap.enabled = false;
  vp.appendChild(_pcRenderer.domElement);

  // ── 内嵌 OrbitController (不依赖外部库) ──
  _pcControls = _createOrbitController(_pcCamera, _pcRenderer.domElement);
  _pcControls.setTarget(0, 0, 0.5);

  // 光照
  _pcScene.add(new THREE.AmbientLight(0x404060, 1.8));
  const dir = new THREE.DirectionalLight(0xffffff, 0.6);
  dir.position.set(0.5, 1, 0.8);
  _pcScene.add(dir);

  // 网格地面
  const grid = new THREE.GridHelper(3, 20, 0x334455, 0x1a1a2e);
  _pcScene.add(grid);

  // 坐标轴
  const axLen = 0.2;
  const axR = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(axLen, 0, 0)]),
    new THREE.LineBasicMaterial({ color: 0xff3333 }));
  const axG = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, axLen, 0)]),
    new THREE.LineBasicMaterial({ color: 0x33ff33 }));
  const axB = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, axLen)]),
    new THREE.LineBasicMaterial({ color: 0x3388ff }));
  _pcScene.add(axR); _pcScene.add(axG); _pcScene.add(axB);

  // 渲染循环
  function animate() {
    _pcAnimId = requestAnimationFrame(animate);
    if (_pcControls) _pcControls.update();
    if (_pcRenderer && _pcScene && _pcCamera) {
      _pcRenderer.render(_pcScene, _pcCamera);
    }
  }
  animate();

  // 窗口大小调整
  function onResize() {
    if (!_pcRenderer || !_pcCamera || !vp) return;
    const rw = vp.clientWidth || 640;
    const rh = vp.clientHeight || 420;
    _pcRenderer.setSize(rw, rh);
    _pcCamera.aspect = rw / rh;
    _pcCamera.updateProjectionMatrix();
  }
  window.addEventListener('resize', onResize);

  // ── 按钮事件 ──
  const btnStart = $('btn-dbg-pc-start');
  const btnStop = $('btn-dbg-pc-stop');
  const topicInput = $('dbg-pc-topic');
  const maxPtsInput = $('dbg-pc-maxpts');
  const ptSizeInput = $('dbg-pc-ptsize');
  const infoEl = $('dbg-pc-info');

  function updateInfo(text, cssClass) {
    if (!infoEl) return;
    infoEl.textContent = text;
    infoEl.style.color = cssClass === 'ok' ? '#4ade80' :
                         cssClass === 'err' ? '#f87171' :
                         cssClass === 'warn' ? '#fbbf24' : '#888';
  }

  btnStart.addEventListener('click', () => {
    if (_pcv) {
      _pcv.stop();
      _pcv.dispose();
      _pcv = null;
    }
    const topic = (topicInput && topicInput.value) || '/camera/depth/color/points';
    const maxPts = parseInt((maxPtsInput && maxPtsInput.value) || '300000', 10);
    const ptSize = parseFloat((ptSizeInput && ptSizeInput.value) || '3') * 0.001;

    try {
      _pcv = createPointCloudViewer({
        scene: _pcScene,
        topic: topic,
        maxPoints: maxPts,
        pointSize: ptSize,
      });
      _pcv.start();
      updateInfo('连接中...', 'warn');
      log('debug-log-pointcloud', '点云连接中 → ' + topic);

      // 定期更新连接状态
      const checkInterval = setInterval(() => {
        if (!_pcv) { clearInterval(checkInterval); return; }
        if (_pcv.isConnected) {
          updateInfo('已连接 | 点数: ' + _pcv.pointCount, 'ok');
          clearInterval(checkInterval);
        }
      }, 500);
      setTimeout(() => { clearInterval(checkInterval); }, 15000);

    } catch (e) {
      updateInfo('错误: ' + (e.message || e), 'err');
      logErr('debug-log-pointcloud', '点云创建失败: ' + (e.message || e));
    }
  });

  btnStop.addEventListener('click', () => {
    if (_pcv) {
      _pcv.stop();
      _pcv.dispose();
      _pcv = null;
    }
    updateInfo('已断开', '');
    log('debug-log-pointcloud', '点云已断开');
  });

  // 清理
  const origCleanup = globalThis._debugPanelCleanup;
  globalThis._debugPanelCleanup = () => {
    if (_pcv) { _pcv.stop(); _pcv.dispose(); _pcv = null; }
    if (_pcAnimId) { cancelAnimationFrame(_pcAnimId); _pcAnimId = null; }
    if (_pcControls) { _pcControls.dispose(); _pcControls = null; }
    window.removeEventListener('resize', onResize);
    if (_pcRenderer) {
      _pcRenderer.dispose();
      if (_pcRenderer.domElement && _pcRenderer.domElement.parentNode) {
        _pcRenderer.domElement.parentNode.removeChild(_pcRenderer.domElement);
      }
      _pcRenderer = null;
    }
    _pcScene = null;
    _pcCamera = null;
    if (typeof origCleanup === 'function') origCleanup();
  };

  updateInfo('未连接', '');
  log('debug-log-pointcloud', '点云查看器就绪, 点击"连接"开始');
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', init);
} else {
  init();
}
