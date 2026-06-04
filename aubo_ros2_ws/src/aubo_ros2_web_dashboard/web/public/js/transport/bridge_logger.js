// transport/bridge_logger.js — BridgeLogger 全链路统计引擎 (v3.2)
//
// 职责: 维护 per-bridge 累加统计 + 活跃订阅追踪
// 持久化委托给 LogEventBus (不维护自己的日志缓冲)
//
// 3 条输出:
//   1. LogEventBus.addLog('bridge', ...) — 持久化 + BroadcastChannel
//   2. dispatchEvent('bridge:traffic')   — 实时监听器 (status_bar / debug_panel)
//   3. getStats() / getActiveSubs()       — 轮询查询
//
// 用法:
//   import { bridgeLogger } from './transport/bridge_logger.js';
//   bridgeLogger.log({ direction:'sub', topic:'/joint_states', msgType:'sensor_msgs/JointState', bridge:'foxglove', mode:'FOXGLOVE', status:'ok' });
//   bridgeLogger.getStats();         // → { foxglove:{subs,pubs,svcs,msgsRx,msgsTx,errors}, rosbridge:{...} }
//   bridgeLogger.getActiveSubs();    // → [{topic, bridge, msgType}]

const g = globalThis;

function _initStats() {
    return {
        subs: 0, pubs: 0, svcs: 0,
        msgsRx: 0, msgsTx: 0, errors: 0,
    };
}

class BridgeLogger extends EventTarget {
    constructor() {
        super();
        this._stats = {
            foxglove: _initStats(),
            rosbridge: _initStats(),
        };
        this._activeSubs = new Map();  // topic → { bridge, msgType }
    }

    /**
     * 记录一条流量日志
     * @param {{direction:'sub'|'pub'|'svc', topic:string, msgType?:string, bridge:'foxglove'|'rosbridge'|'none', mode:string, status:'ok'|'error'|'degraded'|'restored'}} entry
     */
    log(entry) {
        entry.ts = Date.now();
        entry.status = entry.status || 'ok';

        // 累计统计
        const s = this._stats[entry.bridge];
        if (s) {
            if (entry.direction === 'sub') { s.subs++; s.msgsRx++; }
            else if (entry.direction === 'pub') { s.pubs++; s.msgsTx++; }
            else if (entry.direction === 'svc') { s.svcs++; }
            if (entry.status === 'error') s.errors++;
        }

        // 活跃订阅追踪
        if (entry.direction === 'sub' && entry.status === 'ok') {
            this._activeSubs.set(entry.topic, {
                bridge: entry.bridge,
                msgType: entry.msgType || '?',
            });
        }

        // 实时事件
        this.dispatchEvent(new CustomEvent('bridge:traffic', { detail: entry }));

        // 持久化到 LogEventBus (通过 globalThis 引用，避免循环 import)
        const logBus = g.__logBus;
        if (logBus) {
            const level = entry.status === 'error' ? 'error'
                : (entry.status === 'degraded' ? 'warn' : 'info');
            const meta = Object.assign({}, entry);
            const msg = '[' + entry.bridge + '] ' + entry.direction + ' ' + entry.topic
                + (entry.status !== 'ok' ? ' ' + entry.status : '')
                + (entry.msgType ? ' ' + entry.msgType : '');
            logBus.addLog(level, 'bridge', msg, meta, 'bridge');
        }
    }

    /** 获取累计统计快照 */
    getStats() {
        return {
            foxglove: Object.assign({}, this._stats.foxglove),
            rosbridge: Object.assign({}, this._stats.rosbridge),
        };
    }

    /** 获取当前活跃订阅及其 bridge 归属 */
    getActiveSubs() {
        return Array.from(this._activeSubs.entries()).map(function(_ref) {
            var topic = _ref[0], info = _ref[1];
            return { topic: topic, bridge: info.bridge, msgType: info.msgType };
        });
    }
}

const bridgeLogger = new BridgeLogger();
g.__bridgeLogger = bridgeLogger;

export { BridgeLogger, bridgeLogger };
