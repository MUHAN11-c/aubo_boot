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

        // 传输层事件 → 内部分发
        this._transport.onRosJson(null, (msg, topic) => {
            this._dispatchRos(topic, msg);
        });
    }

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

        try {
            const wsUrl = url || rosbridgeWebSocketUrl();
            await this._transport.loadRuntime();
            await this._transport.connectControl();
            this._reconnect.attempts = 0;
            clearRosReconnectTimer(this._reconnect);

            // 连接成功 → 监听后续断线事件
            this._transport.clearControlJsonHandlers();
            this._transport.onControlJson((o) => {
                if (!o || typeof o !== 'object') return;
                if (o.op === 'error') {
                    this._log('error', 'rosbridge error: ' + (o.message || 'unknown'));
                }
                if (o.op === 'close') {
                    if (this._paused) return;
                    this._log('transport', 'rosbridge closed, scheduling reconnect...');
                    this._scheduleReconnect();
                }
            });

            this._log('transport', 'rosbridge connected');
            return true;
        } catch (e) {
            this._log('error', 'connect failed: ' + (e.message || e));
            this._scheduleReconnect();
            throw e;
        } finally {
            this._connectInFlight = false;
        }
    }

    disconnect() {
        clearRosReconnectTimer(this._reconnect);
        this._connectInFlight = false;
        this._transport.close();
    }

    // ── 话题订阅 ────────────────────────────────────────────────────────────

    subscribe(topic, msgType, callback) {
        const name = canonicalRosTopic(topic);
        // 注册处理器 (传输层自动去重 subscribe)
        this._rosHandlers.push({ topic: name, fn: callback });
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
        return this._transport.callService({
            service,
            type,
            request,
            timeoutMs,
        });
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
        this._transport.clearControlJsonHandlers();
        this._transport.close();
        this._log('lifecycle', 'page paused');
    }

    resume() {
        if (!this._paused) return;
        this._paused = false;
        this._log('lifecycle', 'page resumed');
        // 不自动重连，由页面入口决定
    }

    // ── 内部方法 ────────────────────────────────────────────────────────────

    _scheduleReconnect() {
        this.unsubscribeAll();
        scheduleRosReconnect(this._reconnect, () => this.connect(), {
            maxAttempts: ROS_RECONNECT_MAX,
            onSchedule: (delayMs, attempt, max) => {
                this._log('transport',
                    `reconnect in ${Math.round(delayMs / 1000)}s (${attempt}/${max})`);
            },
            onExhausted: () => {
                this._log('error', 'reconnect exhausted, please refresh page');
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
