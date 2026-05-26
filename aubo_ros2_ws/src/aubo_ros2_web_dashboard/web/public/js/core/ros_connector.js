// core/ros_connector.js — 共享 rosbridge 连接生命周期管理
// 封装 connect/重连/状态展示逻辑，vision_grasp_panel 和 tf_monitor_panel 共用
//
// 用法:
//   import { createRosConnector } from './core/ros_connector.js';
//   const rc = createRosConnector({
//     ivgPorts, ivgTransport,
//     setConnStatus: (text, ok) => { ... },
//     onConnected: () => { /* 订阅话题 */ },
//   });
//   document.addEventListener('DOMContentLoaded', () => rc.connect());

import { ivgPorts } from '../ivg_runtime.js';
import { ivgTransport } from '../ivg_transport.js';

const DEFAULT_RECONNECT_MAX = 12;

export function createRosConnector(opts) {
  const options = opts || {};
  const ports   = options.ivgPorts   || ivgPorts;
  const transport = options.ivgTransport || ivgTransport;
  const owner   = options.owner || 'ros_connector';
  const reconnectMax = options.reconnectMax || DEFAULT_RECONNECT_MAX;
  const setConnStatus = typeof options.setConnStatus === 'function'
    ? options.setConnStatus
    : function () {};

  const rosReconnect = ports.createRosReconnectState();
  let connectInFlight = false;

  function _unsubscribeAll() {
    if (typeof options.onUnsubscribeAll === 'function') {
      options.onUnsubscribeAll();
    }
    transport.clearRosHandlersByOwner(owner);
    transport.unsubscribeAll();
  }

  function _scheduleReconnect() {
    return ports.scheduleRosReconnect(rosReconnect, connect, {
      maxAttempts: reconnectMax,
      onSchedule: function (delayMs, attempt, max) {
        setConnStatus(
          '已断开：' + Math.round(delayMs / 1000) + 's 后自动重连 (' + attempt + '/' + max + ')',
          false
        );
        if (typeof options.onReconnecting === 'function') {
          options.onReconnecting(delayMs, attempt, max);
        }
      },
      onExhausted: function () {
        setConnStatus('已断开（已达自动重连上限，请刷新页面）', false);
      },
    });
  }

  function connect() {
    if (connectInFlight) return;
    connectInFlight = true;
    ports.clearRosReconnectTimer(rosReconnect);
    const myGen = ports.bumpRosReconnectGen(rosReconnect);

    setConnStatus('正在连接…', null);
    // 先清理旧连接上的所有订阅，再关闭，避免 rosbridge 端 WebSocketClosedError
    _unsubscribeAll();
    transport.close();

    void (async () => {
      try {
        await transport.loadRuntime();
        await transport.connectControl();
        if (myGen !== rosReconnect.gen) return;

        rosReconnect.attempts = 0;
        ports.clearRosReconnectTimer(rosReconnect);

        transport.clearControlHandlersByOwner(owner);
        transport.onControlJson(function (ctrl) {
          if (!ctrl || typeof ctrl !== 'object') return;
          if (ctrl.op === 'error') {
            setConnStatus('已连接但发生通信错误，准备重连…', false);
          }
          if (ctrl.op === 'close') {
            setConnStatus('连接已断开，准备重连…', false);
            _unsubscribeAll();
            _scheduleReconnect();
          }
        }, owner);

        setConnStatus('已连接', true);

        if (typeof options.onConnected === 'function') {
          options.onConnected();
        }
      } catch (e) {
        if (myGen !== rosReconnect.gen) return;
        _unsubscribeAll();
        setConnStatus('连接错误', false);
        _scheduleReconnect();
      } finally {
        connectInFlight = false;
      }
    })();
  }

  // 暴露方法给外部（如页面可见性切换）
  return {
    connect: connect,
    rosReconnect: rosReconnect,
    get isConnecting() { return connectInFlight; },
    wireOnlineReconnect: function () {
      ports.wireOnlineRosReconnect(rosReconnect, connect);
    },
  };
}
