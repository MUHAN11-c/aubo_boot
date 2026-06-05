// ivg_transport — ROSLIB.Ros 封装：连接管理、话题订阅、服务调用、摄像头 URL
// 链路: 浏览器 ←→ IvgTransport ←→ rosbridge (WebSocket)
// 全局单例: ivgTransport
import * as ROSLIB from 'roslib';
import { loadIvgRuntime } from './core/runtime_provider.js';
import { rosbridgeWebSocketUrlFromRuntime } from './ivg_runtime.js';
import { canonicalRosTopic, encodeTopicQueryValue } from './core/utils.js';
import { logBus } from './core/log-bus.js';

const g = globalThis;

// ── 工具 ─────────────────────────────────────────────────────────────────────

// canonicalRosTopic / encodeTopicQueryValue 已从 core/utils.js 导入

// ── 传输层 ──────────────────────────────────────────────────────────────────

function IvgTransport() {
    this.runtime = null;          // 运行时配置 (来自 /api/v1/runtime)
    this.ros = null;              // ROSLIB.Ros 实例
    this._topicSubs = new Map();  // topic → ROSLIB.Topic
    this._topicSpecs = new Map(); // topic → msgType (去重用)
    this._rosHandlers = new Map();    // owner → Set<{topic, fn}> 话题消息处理器
    this._controlHandlers = new Map(); // owner → Set<Function> 控制面 JSON 处理器
    this._connectPromise = null;  // 去重: 同一时间只有一个连接尝试
}


