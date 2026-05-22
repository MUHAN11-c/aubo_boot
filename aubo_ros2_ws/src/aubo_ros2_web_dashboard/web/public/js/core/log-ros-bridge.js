// log-ros-bridge.js — ROS 消息 → 统一日志总线桥接
// 订阅 /rosout，代理服务调用，记录关键话题摘要喵~
//
// 用法: 只需 import 一次（副作用模块），自动检测传输层并注册钩子
//   import './core/log-ros-bridge.js';
//
// 兼容两种传输层访问方式:
//   - ros.js (RosManager 单例, globalThis.__rosManager._transport) — latte 页面
//   - ivgTransport (直接导入, globalThis.ivgTransport) — vision_grasp 页面

import { logBus } from './log-bus.js';

const TAG = '[log-ros-bridge]';

// ── 传输层检测: 同时支持 ros.js 和 ivgTransport ──────────────────────────

function _getTransport() {
    // 优先从 ros.js 单例取（latte/main.js 页面）
    if (globalThis.__rosManager && globalThis.__rosManager._transport) {
        return globalThis.__rosManager._transport;
    }
    // 回退：直接检测全局 ivgTransport 引用（vision_grasp 页面，如果有设置的话）
    if (globalThis.ivgTransport) {
        return globalThis.ivgTransport;
    }
    return null;
}

// ── 去重：相同消息 N 秒内不重复记录 ──────────────────────────────────────
const _dedup = {};  // { key: timestamp }
const DEDUP_WINDOW_MS = 5000;

function _shouldLog(key) {
    const now = Date.now();
    const last = _dedup[key] || 0;
    if (now - last < DEDUP_WINDOW_MS) return false;
    _dedup[key] = now;
    return true;
}

// ── 轮询等待传输层就绪（最长 60s）────────────────────────────────────────
function _waitForTransport(cb) {
    let ticks = 0;
    const timer = setInterval(() => {
        const t = _getTransport();
        if (t && t.ros && t.isConnected()) {
            clearInterval(timer);
            logBus.addLog('info', 'system', 'ROS 日志桥接: 传输层已就绪，注册 /rosout + service 钩子');
            cb(t);
            return;
        }
        if (++ticks > 120) {  // 60s 超时
            clearInterval(timer);
            logBus.addLog('warn', 'system', 'ROS 日志桥接: 等待传输层超时（60s），/rosout 未订阅、service 钩子未注册');
        }
    }, 500);
}

// ── 订阅 /rosout ──────────────────────────────────────────────────────────
function _subscribeRosout(transport) {
    try {
        transport.subscribe({
            topic: '/rosout',
            msgType: 'rcl_interfaces/msg/Log',
            maxHz: 20,
        });
        transport.onRosJson('/rosout', function (msg) {
            if (!msg) return;
            const levelMap = { 10: 'debug', 20: 'info', 30: 'warn', 40: 'error', 50: 'error' };
            const level = levelMap[msg.level] || 'info';
            logBus.addLog(level, 'rosout', msg.msg || '', {
                node: msg.name || '?',
                function: msg.function || '',
                line: msg.line || 0,
                rosLevel: msg.level,
            });
        });
        logBus.addLog('info', 'system', '/rosout 订阅已建立，ROS 2 节点日志将在此显示');
    } catch (e) {
        logBus.addLog('warn', 'system', '/rosout 订阅失败: ' + (e.message || e));
    }
}

// ── 代理服务调用，记录响应负载 ────────────────────────────────────────────
function _hookServiceCalls(transport) {
    if (transport._logBridgeHooked) return;
    transport._logBridgeHooked = true;

    const origCallService = transport.callService.bind(transport);
    transport.callService = function (spec) {
        const svcName = spec.service || spec.srvName || '?';
        const start = performance.now();

        logBus.addLog('info', 'service', '→ ' + svcName + ' (调用中...)', {
            service: svcName, phase: 'in_progress',
        });

        return origCallService(spec).then(function (result) {
            const ms = (performance.now() - start).toFixed(0);
            logBus.addLog('info', 'service', '✓ ' + svcName + ' (' + ms + 'ms)', {
                service: svcName,
                durationMs: parseInt(ms),
                success: result && result.success !== false,
                response: _summarize(result),
            });
            return result;
        }).catch(function (e) {
            const ms = (performance.now() - start).toFixed(0);
            logBus.addLog('error', 'service', '✗ ' + svcName + ' (' + ms + 'ms) ' + String(e), {
                service: svcName,
                durationMs: parseInt(ms),
                error: String(e),
            });
            throw e;
        });
    };

    logBus.addLog('info', 'system', 'service 调用钩子已注册（自动记录所有 ROS 服务调用）');
}

// ── 话题数据摘要记录 ──────────────────────────────────────────────────────
const _topicStats = {};

function _hookTopicData(transport) {
    if (transport._logBridgeTopicHooked) return;
    transport._logBridgeTopicHooked = true;

    // 监听所有话题消息
    transport.onRosJson(null, function (msg, topic) {
        if (!topic) return;
        const tname = String(topic);

        // /rosout 已单独处理，跳过
        if (tname === '/rosout') return;

        // 模式变更（低频）
        if (tname === '/aubo/mode' && msg && msg.data) {
            const mode = String(msg.data);
            if (_shouldLog('mode:' + mode)) {
                logBus.addLog('info', 'topic', '驱动模式: ' + mode, { topic: tname, mode });
            }
            return;
        }

        // 系统状态变更
        if (tname === '/system_status' && msg && msg.data) {
            const status = String(msg.data);
            if (_shouldLog('sys_status:' + status)) {
                logBus.addLog('info', 'topic', '系统状态: ' + status, { topic: tname, status });
            }
            return;
        }

        // 工具快换状态
        if (tname === '/tool_changer_status' && msg && msg.tool_id) {
            const toolId = String(msg.tool_id);
            if (_shouldLog('tool:' + toolId)) {
                logBus.addLog('info', 'topic', '工具快换状态变更: ' + toolId, { topic: tname, tool_id: toolId });
            }
            return;
        }

        // 高频话题：累计统计
        if (tname === '/robot_status' || tname === '/joint_states' || tname === '/grasp_poses_base') {
            if (!_topicStats[tname]) {
                _topicStats[tname] = { count: 0, lastSummary: 0 };
            }
            _topicStats[tname].count++;
            const now = Date.now();
            if (now - _topicStats[tname].lastSummary > 5000) {
                _topicStats[tname].lastSummary = now;
                logBus.addLog('debug', 'topic',
                    tname + ': ' + _topicStats[tname].count + ' 条消息/5s', { topic: tname });
                _topicStats[tname].count = 0;
            }
            return;
        }
    });

    logBus.addLog('info', 'system', '话题摘要钩子已注册（/aubo/mode, /system_status, /tool_changer_status 等）');
}

// ── 响应摘要 ──────────────────────────────────────────────────────────────
function _summarize(obj) {
    if (!obj || typeof obj !== 'object') return String(obj);
    try {
        const s = JSON.stringify(obj);
        return s.length > 256 ? s.slice(0, 256) + '…' : s;
    } catch (_) {
        return String(obj).slice(0, 256);
    }
}

// ── 启动 ───────────────────────────────────────────────────────────────────
logBus.addLog('info', 'system', 'ROS 日志桥接启动中，等待传输层就绪...');
_waitForTransport(function (transport) {
    _subscribeRosout(transport);
    _hookServiceCalls(transport);
    _hookTopicData(transport);
    logBus.addLog('info', 'system', 'ROS 日志桥接完全就绪 (rosout + service + topic 摘要)');
});
