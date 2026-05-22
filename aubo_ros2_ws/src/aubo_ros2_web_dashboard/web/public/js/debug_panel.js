// debug_panel.js — 调试面板：笛卡尔直线/关节运动/服务调用/话题发布
import { ivgTransport } from './ivg_transport.js';

const TAG = '[debug_panel]';

function $(id) { return document.getElementById(id); }

function log(id, msg) {
  const el = $(id);
  if (!el) return;
  var line = '> [' + new Date().toLocaleTimeString() + '] ' + msg;
  el.textContent = line + '\n' + (el.textContent || '');
  // keep last 20 lines
  var lines = el.textContent.split('\n');
  if (lines.length > 20) el.textContent = lines.slice(0, 20).join('\n');
}

function logError(id, msg) {
  var el = $(id);
  if (!el) return;
  el.innerHTML = '<span style="color:#dc2626">> [' + new Date().toLocaleTimeString() + '] ' + msg + '</span>\n' + (el.innerHTML || '');
}

// ===== Cartesian Line Motion =====
function setupCartesian() {
  var btnMove = $('btn-dbg-move-xyz');
  var btnStop = $('btn-dbg-stop-xyz');
  if (!btnMove || !btnStop) return;

  btnMove.addEventListener('click', function () {
    var x = parseFloat($('dbg-xyz-x').value) || 0;
    var y = parseFloat($('dbg-xyz-y').value) || 0;
    var z = parseFloat($('dbg-xyz-z').value) || 0;
    var vel = parseFloat($('dbg-xyz-vel').value) || 0.3;
    var acc = parseFloat($('dbg-xyz-acc').value) || 0.2;

    log('debug-log-cartesian', 'MoveXYZ(x=' + x.toFixed(4) + ', y=' + y.toFixed(4) + ', z=' + z.toFixed(4) + ', vel=' + vel + ', acc=' + acc + ')');

    ivgTransport.callService({
      service: '/move_to_pose',
      args: { x: x, y: y, z: z, vel: vel, acc: acc }
    }).then(function (resp) {
      log('debug-log-cartesian', 'OK: ' + JSON.stringify(resp));
    }).catch(function (err) {
      logError('debug-log-cartesian', 'FAIL: ' + (err.message || err));
    });
  });

  btnStop.addEventListener('click', function () {
    log('debug-log-cartesian', 'STOP 请求');
    ivgTransport.callService({
      service: '/stop_motion',
    }).catch(function () {});
  });
}

// ===== Joint Space Motion =====
function setupJointMotion() {
  var btnMove = $('btn-dbg-move-joints');
  var btnRead = $('btn-dbg-read-joints');
  if (!btnMove) return;

  btnMove.addEventListener('click', function () {
    var joints = [];
    for (var i = 1; i <= 6; i++) {
      joints.push(parseFloat($('dbg-j' + i).value) || 0);
    }
    var vel = parseFloat($('dbg-jnt-vel').value) || 0.3;
    var acc = parseFloat($('dbg-jnt-acc').value) || 0.2;

    log('debug-log-joint', 'MoveJoints([' + joints.map(function(v){return v.toFixed(4);}).join(', ') + '], vel=' + vel + ', acc=' + acc + ')');

    ivgTransport.callService({
      service: '/move_to_joint',
      args: { positions: joints, vel: vel, acc: acc }
    }).then(function (resp) {
      log('debug-log-joint', 'OK: ' + JSON.stringify(resp));
    }).catch(function (err) {
      logError('debug-log-joint', 'FAIL: ' + (err.message || err));
    });
  });

  if (btnRead) {
    btnRead.addEventListener('click', function () {
      log('debug-log-joint', '读取当前关节角...');
      ivgTransport.callService({
        service: '/get_current_joints',
      }).then(function (resp) {
        var positions = resp.values || resp.positions || [];
        for (var i = 0; i < Math.min(6, positions.length); i++) {
          var inp = $('dbg-j' + (i + 1));
          if (inp) inp.value = positions[i].toFixed(4);
        }
        log('debug-log-joint', '当前关节角: [' + positions.map(function(v){return v.toFixed(4);}).join(', ') + ']');
      }).catch(function (err) {
        logError('debug-log-joint', '读取失败: ' + (err.message || err));
      });
    });
  }
}

// ===== Service Call Test =====
function setupServiceCall() {
  var btn = $('btn-dbg-call-svc');
  if (!btn) return;

  btn.addEventListener('click', function () {
    var svc = ($('dbg-svc-name').value || '').trim();
    var argsStr = ($('dbg-svc-args').value || '{}').trim();
    if (!svc) { logError('debug-log-service', '请输入服务名'); return; }

    var args;
    try { args = JSON.parse(argsStr); }
    catch (e) { logError('debug-log-service', 'JSON 解析错误: ' + e.message); return; }

    log('debug-log-service', 'call ' + svc + ' ' + JSON.stringify(args));

    ivgTransport.callService({
      service: svc,
      args: args
    }).then(function (resp) {
      log('debug-log-service', 'OK: ' + JSON.stringify(resp));
    }).catch(function (err) {
      logError('debug-log-service', 'FAIL: ' + (err.message || err));
    });
  });
}

// ===== Topic Publish Test =====
function setupTopicPublish() {
  var btn = $('btn-dbg-publish');
  if (!btn) return;

  btn.addEventListener('click', function () {
    var topic = ($('dbg-pub-topic').value || '').trim();
    var msgStr = ($('dbg-pub-msg').value || '{}').trim();
    if (!topic) { logError('debug-log-topic', '请输入话题名'); return; }

    var msg;
    try { msg = JSON.parse(msgStr); }
    catch (e) { logError('debug-log-topic', 'JSON 解析错误: ' + e.message); return; }

    log('debug-log-topic', 'publish ' + topic + ' ' + JSON.stringify(msg));

    ivgTransport.publish({
      topic: topic,
      msg: msg
    }).then(function () {
      log('debug-log-topic', 'OK → 已发布');
    }).catch(function (err) {
      logError('debug-log-topic', 'FAIL: ' + (err.message || err));
    });
  });
}

// ===== Init =====
function init() {
  setupCartesian();
  setupJointMotion();
  setupServiceCall();
  setupTopicPublish();
  console.log(TAG, '初始化完成');
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', init);
} else {
  init();
}
