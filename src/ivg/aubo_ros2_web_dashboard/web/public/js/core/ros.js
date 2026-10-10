// ros.js — ROSLIB.Ros 单例封装: 连接管理、话题订阅、服务调用、事件分发
// 包装现有 ivgTransport 提供生命周期感知的统一 API 喵~
//
// 所有页面通过此模块与 ROS 通信:
//   import { ros } from '../core/ros.js';
//   await ros.connect(url);
//   ros.subscribe('/topic', 'msg/Type', (msg) => { ... });
//   const result = await ros.callService('/svc', 'srv/Type', { ... });
//
// 参照: Rosboard 模块化 Viewer 模式 + roslibjs event-driven API

import { ivgTransport } from '../ivg_transport.js';
import {
    loadRuntime as _loadPorts,
    rosbridgeWebSocketUrl,
    createRosReconnectState,
    clearRosReconnectTimer,
    bumpRosReconnectGen,
    scheduleRosReconnect,
    wireOnlineRosReconnect,
} from '../ivg_runtime.js';
import { canonicalRosTopic } from './utils.js';
import { logBus } from './log-bus.js';
import { rosDataStore } from '../entities/ros_data_store.js';
import { serviceStore } from '../entities/service_store.js';
import { connectionStore } from '../entities/connection_store.js';

const g = globalThis;

const ROS_RECONNECT_MAX = 12;

class RosManager {
    constructor() {
        this._transport = ivgTransport;
        this._reconnect = createRosReconnectState();
        this._connectInFlight = false;
        this._paused = false;

        // 处理器注册表
        this._rosHandlers = [];
        this._controlHandlers = [];
        this._logHandlers = [];

        // 页面可注册的回调
        this._onStatusChange = null;   // (text, ok) => {}
        this._onConnected = null;       // () => {}
        this._onReconnecting = null;    // (delayMs, attempt, max) => {}
        this._onCleanup = null;         // () => {} 重连前页面级清理

        // 传输层事件 → 内部分发 (rosbridge JSON, fallback 路径)
        this._transport.onRosJson(null, (msg, topic) => {
            this._dispatchRos(topic, msg);
        }, 'ros_manager');
    }

    // ── 回调注册 ──────────────────────────────────────────────────────────

    /** 连接状态变更回调: (statusText, isOk) => {}  */
    onStatusChange(cb) { this._onStatusChange = cb; }

    /** 连接成功后回调: () => {}，用于页面订阅话题等初始化  */
    onConnected(cb) { this._onConnected = cb; }

    /** 重连中回调: (delayMs, attempt, max) => {} */
    onReconnecting(cb) { this._onReconnecting = cb; }

    /** 重连前页面级清理回调: () => {}，用于停止 3D 渲染、清缓存等 */
    onCleanup(cb) { this._onCleanup = cb; }

    // ── 连接生命周期 ────────────────────────────────────────────────────────

    get isConnected() {
        return this._transport.isConnected();
    }

    async loadRuntime() {
        return this._transport.loadRuntime();
    }

    async connect(url) {
        if (this._connectInFlight) return;
        if (this._paused) return;
        this._connectInFlight = true;
        clearRosReconnectTimer(this._reconnect);

        if (this._onStatusChange) this._onStatusChange('正在连接…', null);

        try {
            // 动态加载 TransportHub (统一传输调度, 事件驱动)
            let hub = g.__transportHub;
            if (!hub) {
                try {
                    const mod = await import('../transport/hub.js');
                    hub = mod.transportHub;
                } catch (e) {
                    this._log('warn', 'TransportHub 加载失败: ' + (e.message || e));
                }
            }
            // TransportHub.init() 管理所有适配器连接 (首次连接 + 重连均走此路径)
            if (hub) {
                // 在 init() 前注册 bridgechange 监听, 防止 _reconcile() 事件丢失
                this._wireHubBridgeChange(hub);
                await hub.init();
            } else {
                throw new Error('TransportHub not available');
            }

            this._reconnect.attempts = 0;
            clearRosReconnectTimer(this._reconnect);

            // 监听 ivgTransport 断线事件 (rosbridge 底层关闭)
            this._transport.clearControlHandlersByOwner('ros_manager');
            this._transport.onControlJson((o) => {
                if (!o || typeof o !== 'object') return;
                if (o.op === 'error') {
                    this._log('error', 'rosbridge error: ' + (o.message || 'unknown'));
                    if (this._onStatusChange) this._onStatusChange('已连接但发生通信错误，准备重连…', false);
                }
                if (o.op === 'close') {
                    if (this._paused) return;
                    this._log('transport', 'transport closed, scheduling reconnect...');
                    this._scheduleReconnect();
                }
            }, 'ros_manager');

            // hub.init() 后检查: 若 rosbridge 未连接则触发重连
            if (hub && !hub.activeId) {
                this._log('warn', 'hub.init() 完成但无活跃桥接 (rosbridge 未连接), 启动重连');
                if (this._onStatusChange) this._onStatusChange('rosbridge 未连接，准备重连…', false);
                this._connectInFlight = false;
                this._scheduleReconnect();
                return false;
            }

            if (this._onStatusChange) this._onStatusChange('已连接', true);
            if (this._onConnected) this._onConnected();

            this._log('transport', 'connected via TransportHub');
            return true;
        } catch (e) {
            this._log('error', 'connect failed: ' + (e.message || e));
            if (this._onStatusChange) this._onStatusChange('连接错误', false);
            this._scheduleReconnect();
            throw e;
        } finally {
            this._connectInFlight = false;
        }
    }

    /** 监听 TransportHub bridgechange → 活跃桥接丢失时触发重连 */
    _wireHubBridgeChange(hub) {
        if (hub._rosManagerBridgeWired) return;
        hub._rosManagerBridgeWired = true;

        hub.addEventListener('bridgechange', (e) => {
            connectionStore.setActiveBridge(e.detail.active);
            if (!e.detail.active && !this._paused) {
                this._log('transport', 'all bridges disconnected, scheduling reconnect...');
                this._scheduleReconnect();
            }
        });

        hub.addEventListener('modechange', (e) => {
            connectionStore.setBridgeMode(e.detail.mode);
            connectionStore.setActiveBridge(e.detail.active);
        });

        hub.addEventListener('stats', () => {
            connectionStore.setBridgeStats(hub.getStats());
        });
    }

    disconnect() {
        clearRosReconnectTimer(this._reconnect);
        this._connectInFlight = false;
        this._transport.close();
    }

    // ── 话题订阅 ────────────────────────────────────────────────────────────

    subscribe(topic, msgType, callback) {
        const name = canonicalRosTopic(topic);
        this._rosHandlers.push({ topic: name, fn: callback });

        // TransportHub 可用时委托 (自适应路由: foxglove CDR / rosbridge JSON)
        const hub = g.__transportHub;
        if (hub && hub.initialized) {
            return hub.subscribe(name, msgType, function(typedMsg) {
                // 同步到 RosDataStore (SWR 缓存)
                rosDataStore.set(name, typedMsg.data, typedMsg.bridge || 'unknown');
                try { callback(typedMsg.data); } catch (_) {}
            });
        }
        return this._transport.subscribe({ topic: name, msgType });
    }

    unsubscribe(topic) {
        const name = canonicalRosTopic(topic);
        this._rosHandlers = this._rosHandlers.filter(h => h.topic !== name);
        this._transport.unsubscribe(name);
    }

    unsubscribeAll() {
        this._rosHandlers.length = 0;
        this._transport.unsubscribeAll();
    }

    // ── 服务调用 ────────────────────────────────────────────────────────────

    async callService(service, type, request, timeoutMs = 60000) {
        timeoutMs = timeoutMs || 60000;
        // ServiceStore 注入执行器 (首次调用时)
        if (!serviceStore._executor) {
            const self = this;
            serviceStore.setExecutor(function(name, svcType, svcReq, timeout) {
                const hub = g.__transportHub;
                if (hub && hub.initialized) {
                    return hub.callService(name, svcType, svcReq, timeout);
                }
                return self._transport.callService({
                    service: name, type: svcType, request: svcReq, timeoutMs: timeout,
                });
            });
        }
        return serviceStore.callService(service, type, request, timeoutMs);
    }

    // ── 事件系统 ────────────────────────────────────────────────────────────

    onRosJson(topic, handler) {
        this._rosHandlers.push({ topic: topic ? canonicalRosTopic(topic) : null, fn: handler });
    }

    onControlJson(handler) {
        this._controlHandlers.push(handler);
    }

    onLog(handler) {
        this._logHandlers.push(handler);
    }

    // ── 页面生命周期 ────────────────────────────────────────────────────────

    pause() {
        this._paused = true;
        clearRosReconnectTimer(this._reconnect);
        this._reconnect.attempts = 0;
        this.unsubscribeAll();
        this._transport.clearControlHandlersByOwner('ros_manager');
        this._transport.close();
        this._log('lifecycle', 'page paused');
    }

    resume() {
        if (!this._paused) return;
        this._paused = false;
        this._log('lifecycle', 'page resumed');
        // 不自动重连，由页面入口决定
    }

    /** 监听浏览器 online/offline 事件自动重连 */
    wireOnlineReconnect() {
        wireOnlineRosReconnect(this._reconnect, () => this.connect());
    }

    // ── 内部方法 ────────────────────────────────────────────────────────────

    _scheduleReconnect() {
        if (this._onCleanup) this._onCleanup();
        this.unsubscribeAll();
        if (this._onStatusChange) this._onStatusChange('连接已断开，准备重连…', false);
        scheduleRosReconnect(this._reconnect, () => this.connect(), {
            maxAttempts: ROS_RECONNECT_MAX,
            onSchedule: (delayMs, attempt, max) => {
                this._log('transport',
                    `reconnect in ${Math.round(delayMs / 1000)}s (${attempt}/${max})`);
                if (this._onStatusChange) {
                    this._onStatusChange(
                        `已断开：${Math.round(delayMs / 1000)}s 后自动重连（${attempt}/${max}）`, false);
                }
                if (this._onReconnecting) {
                    this._onReconnecting(delayMs, attempt, max);
                }
            },
            onExhausted: () => {
                this._log('error', 'reconnect exhausted, please refresh page');
                if (this._onStatusChange) {
                    this._onStatusChange('已断开（已达自动重连上限，请刷新页面）', false);
                }
            },
        });
    }

    _dispatchRos(topic, msg) {
        const ct = canonicalRosTopic(topic);
        for (const h of this._rosHandlers) {
            const match = h.topic == null || h.topic === topic || h.topic === ct;
            if (match) {
                try { h.fn(msg, topic); } catch (e) { /* */ }
            }
        }
    }

    _log(level, msg) {
        // 转发到统一日志总线喵~
        logBus.addLog(level, 'ros_manager', msg);
        for (const h of this._logHandlers) {
            try { h(level, msg); } catch (e) { /* */ }
        }
    }
}

// 模块级单例 — 每个页面各自创建，不跨标签页共享
const ros = new RosManager();

// 暴露到 globalThis 供非模块脚本兼容
globalThis.__rosManager = ros;

export { RosManager, ros };
