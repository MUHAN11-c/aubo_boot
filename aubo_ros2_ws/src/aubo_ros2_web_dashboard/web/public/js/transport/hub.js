// transport/hub.js — TransportHub 统一传输调度中心
//
// 设计原则:
//   - 适配器自注册: adapter 创建后调用 hub.register(adapter)
//   - 事件驱动: 模式/状态变更通过 EventTarget 广播, 消费者订阅事件而非轮询
//   - 零硬编码: 不检查 g.__dataService, 不写死适配器名称
//   - 插件化: 新增 Bridge 类型只需 register() 即可接入
//
// 用法:
//   import { transportHub } from './transport/hub.js';
//   transportHub.register(new FoxgloveAdapter());
//   transportHub.register(new RosbridgeAdapter());
//   await transportHub.init();  // 自动连接所有已注册适配器
//
//   // 任意消费者
//   transportHub.subscribe('/joint_states', 'sensor_msgs/msg/JointState', callback);
//   transportHub.addEventListener('modechange', (e) => {...});

import { BridgeMode } from './adapter.js';
import { foxgloveAdapter } from './foxglove/adapter.js';
import { rosbridgeAdapter } from './rosbridge/adapter.js';
import { topicRouter } from './topic_router.js';
import { SubRegistry } from './sub_registry.js';
import { bridgeLogger } from './bridge_logger.js';
import { foxgloveWebSocketUrl } from '../ivg_runtime.js';

const g = globalThis;
const STORAGE_KEY = 'ivg_bridge_mode';
const STATS_INTERVAL = 2000;

// ── TransportHub ─────────────────────────────────────────────────────────────

class TransportHub extends EventTarget {
  constructor() {
    super();
    this._adapters = new Map();          // id → MessageAdapter
    this._mode = BridgeMode.AUTO;
    this._activeId = null;
    this._router = topicRouter;
    this._subRegistry = new SubRegistry();
    this._initialized = false;
    this._initPromise = null;
    this._statsTimer = null;
  }

  // ── 适配器注册 (插件化入口) ─────────────────────────────────────────────

  /**
   * 注册适配器。适配器在注册时未连接, connect 在 init() 中统一执行。
   * @param {MessageAdapter} adapter
   */
  register(adapter) {
    if (this._adapters.has(adapter.id)) return;
    this._adapters.set(adapter.id, adapter);

    // 监听适配器状态变更 → 自动重评估活跃桥接
    adapter.onStatusChange((status) => {
      if (status.state === 'connected' || status.state === 'disconnected') {
        this._reconcile();
      }
    });

    // 冒泡适配器消息为 hub 事件
    adapter.onMessage((typedMsg) => {
      this.dispatchEvent(new CustomEvent('message', { detail: typedMsg }));
    });
  }

  /** 批量注册 */
  registerAll(adapters) {
    for (const a of adapters) this.register(a);
  }

  // ── 生命周期 ─────────────────────────────────────────────────────────────

  get initialized() { return this._initialized; }
  get mode() { return this._mode; }
  get activeId() { return this._activeId; }
  get activeAdapter() { return this._adapters.get(this._activeId); }

  async init() {
    if (this._initPromise) return this._initPromise;

    const hub = this;

    // 重连: 仅重连已断开的适配器，然后恢复所有活跃订阅
    if (this._initialized) {
      this._initPromise = (async () => {
        const reconnected = [];
        for (const [id, a] of hub._adapters) {
          if (!a.isConnected) {
            try {
              await hub._tryConnect(a);
              reconnected.push(id);
            } catch (_) { /* 重连失败不阻塞其他适配器 */ }
          }
        }
        hub._reconcile();

        // 恢复所有活跃订阅到新连接的适配器
        if (hub._activeId && hub._subRegistry.size > 0) {
          const to = hub.activeAdapter;
          if (to) {
            await hub._subRegistry.migrateAll(to, (adapter, topic, type, cb) => {
              return adapter.subscribe(topic, type, cb);
            });
          }
        }

        hub._initPromise = null;
        console.log(`[TransportHub] reconnect: [${reconnected}]  active: ${hub._activeId}`);
        hub.dispatchEvent(new CustomEvent('ready', {
          detail: { connected: reconnected, failed: [], mode: hub._mode, active: hub._activeId }
        }));
        return { connected: reconnected, failed: [], mode: hub._mode, active: hub._activeId };
      })();
      return this._initPromise;
    }

    // 首次初始化: 并行连接所有已注册适配器
    this._initPromise = (async () => {
      const stored = localStorage.getItem(STORAGE_KEY);
      if (stored && Object.values(BridgeMode).includes(stored)) {
        hub._mode = stored;
      }

      const results = await Promise.allSettled(
        Array.from(hub._adapters.values()).map(a => hub._tryConnect(a))
      );

      const connected = [];
      const failed = [];
      let i = 0;
      for (const [id] of hub._adapters) {
        (results[i++].status === 'fulfilled' ? connected : failed).push(id);
      }

      console.log(`[TransportHub] connected: [${connected}]  failed: [${failed}]  mode: ${hub._mode}`);

      hub._reconcile();

      hub._statsTimer = setInterval(() => {
        hub.dispatchEvent(new CustomEvent('stats'));
      }, STATS_INTERVAL);

      hub._initialized = true;
      hub._initPromise = null;

      hub.dispatchEvent(new CustomEvent('ready', {
        detail: { connected, failed, mode: hub._mode, active: hub._activeId }
      }));

      return { connected, failed, mode: hub._mode, active: hub._activeId };
    })();

    return this._initPromise;
  }

  async _tryConnect(adapter) {
    const url = adapter.id === 'foxglove'
      ? foxgloveWebSocketUrl()
      : this._rosbridgeUrl();
    return adapter.connect(url);
  }

  _rosbridgeUrl() {
    const { rosbridgeWebSocketUrl } = g.ivgPorts || {};
    if (typeof rosbridgeWebSocketUrl === 'function') return rosbridgeWebSocketUrl();
    return `ws://${g.location.host}/ws/rosbridge`;
  }

  // ── 模式切换 (统一入口) ─────────────────────────────────────────────────

  /**
   * 运行时切换 Bridge 模式。触发 'modechange' 事件。
   * @param {BridgeMode} mode
   */
  async setMode(mode) {
    if (mode === this._mode) return;
    const prevActive = this._activeId;
    this._mode = mode;
    localStorage.setItem(STORAGE_KEY, mode);
    this._reconcile();

    // 跨适配器迁移订阅 ("先建后拆")
    if (prevActive !== this._activeId && this._subRegistry.size > 0) {
      const to = this.activeAdapter;
      if (to) {
        await this._subRegistry.migrateAll(to, (adapter, topic, type, cb) => {
          return adapter.subscribe(topic, type, cb);
        });
      }
    }

    this.dispatchEvent(new CustomEvent('modechange', {
      detail: { mode, active: this._activeId, prevActive }
    }));
    console.log(`[TransportHub] mode: ${mode}  active: ${this._activeId}`);
  }

  /** 根据当前 mode + 适配器可用性, 选择活跃桥接 (模式锁定) */
  _reconcile() {
    const prev = this._activeId;
    let nextId = null;
    const fox = this._adapters.get('foxglove');
    const rb = this._adapters.get('rosbridge');
    const wasDegraded = this._wasDegraded;

    if (this._mode === BridgeMode.FOXGLOVE) {
      nextId = (fox && fox.isConnected) ? 'foxglove' : null;
      if (!nextId) {
        bridgeLogger.log({
          direction: 'sub', topic: '(mode)', msgType: '',
          bridge: 'foxglove', mode: this._mode, status: 'error'
        });
        this.dispatchEvent(new CustomEvent('bridgeerror', {
          detail: { mode: this._mode, error: 'foxglove unreachable' }
        }));
      }
    } else if (this._mode === BridgeMode.ROSBRIDGE) {
      nextId = (rb && rb.isConnected) ? 'rosbridge' : null;
      if (!nextId) {
        bridgeLogger.log({
          direction: 'sub', topic: '(mode)', msgType: '',
          bridge: 'rosbridge', mode: this._mode, status: 'error'
        });
        this.dispatchEvent(new CustomEvent('bridgeerror', {
          detail: { mode: this._mode, error: 'rosbridge unreachable' }
        }));
      }
    } else {
      // AUTO: foxglove 优先, 不可用降级 rosbridge (显式)
      if (fox && fox.isConnected) {
        nextId = 'foxglove';
        if (wasDegraded) {
          this._wasDegraded = false;
          bridgeLogger.log({
            direction: 'sub', topic: '(mode)', msgType: '',
            bridge: 'foxglove', mode: this._mode, status: 'restored'
          });
          this.dispatchEvent(new CustomEvent('bridgerestored', {
            detail: { mode: this._mode, active: 'foxglove' }
          }));
        }
      } else if (rb && rb.isConnected) {
        nextId = 'rosbridge';
        if (!wasDegraded) {
          this._wasDegraded = true;
          bridgeLogger.log({
            direction: 'sub', topic: '(mode)', msgType: '',
            bridge: 'rosbridge', mode: this._mode, status: 'degraded'
          });
          this.dispatchEvent(new CustomEvent('bridgedegraded', {
            detail: { mode: this._mode, active: 'rosbridge', reason: 'foxglove unreachable' }
          }));
        }
      } else {
        nextId = null;
      }
    }

    this._activeId = nextId;
    if (prev !== this._activeId) {
      this.dispatchEvent(new CustomEvent('bridgechange', {
        detail: { active: this._activeId, prevActive: prev }
      }));
    }
  }

  // ── 统一数据操作 ─────────────────────────────────────────────────────────

  /**
   * 订阅话题。根据 mode + topic 自动路由到最佳适配器。
   */
  subscribe(topic, msgType, callback) {
    const adapter = this._router.resolve(topic, msgType, this._mode,
      Object.fromEntries(this._adapters));
    if (!adapter) {
      bridgeLogger.log({
        direction: 'sub', topic, msgType, bridge: 'none', mode: this._mode, status: 'error'
      });
      console.warn(`[TransportHub] no adapter for ${topic}`);
      return null;
    }
    this._subRegistry.register(topic, msgType, adapter, callback);
    bridgeLogger.log({
      direction: 'sub', topic, msgType, bridge: adapter.id, mode: this._mode, status: 'ok'
    });
    return adapter.subscribe(topic, msgType, callback);
  }

  unsubscribe(topic, callback) {
    const entry = this._subRegistry.get(topic);
    if (!entry) return;
    entry.callbacks.delete(callback);
    if (entry.callbacks.size === 0) {
      this._subRegistry._active?.delete(topic);
      entry.adapter?.unsubscribe(topic);
    }
  }

  async callService(name, type, request, timeoutMs = 60000) {
    const adapter = this.activeAdapter;
    if (!adapter) {
      bridgeLogger.log({
        direction: 'svc', topic: name, msgType: type, bridge: 'none', mode: this._mode, status: 'error'
      });
      throw new Error('no active adapter for mode: ' + this._mode);
    }
    bridgeLogger.log({
      direction: 'svc', topic: name, msgType: type, bridge: adapter.id, mode: this._mode, status: 'ok'
    });
    try {
      return await adapter.callService(name, type, request, timeoutMs);
    } catch (e) {
      bridgeLogger.log({
        direction: 'svc', topic: name, msgType: type, bridge: adapter.id, mode: this._mode, status: 'error'
      });
      throw e;
    }
  }

  async publish(topic, type, msg) {
    const adapter = this.activeAdapter;
    if (!adapter) {
      bridgeLogger.log({
        direction: 'pub', topic, msgType: type, bridge: 'none', mode: this._mode, status: 'error'
      });
      throw new Error('no_connected_adapter');
    }
    bridgeLogger.log({
      direction: 'pub', topic, msgType: type, bridge: adapter.id, mode: this._mode, status: 'ok'
    });
    return adapter.publish(topic, type, msg);
  }

  // ── 查询 ──────────────────────────────────────────────────────────────────

  getStats() {
    const stats = { mode: this._mode, activeId: this._activeId, adapters: {} };
    for (const [id, a] of this._adapters) {
      stats.adapters[id] = {
        connected: a.isConnected,
        ...a.getStats(),
      };
    }
    return stats;
  }

  getRegisteredIds() {
    return Array.from(this._adapters.keys());
  }

  // ── 便捷事件绑定 ──────────────────────────────────────────────────────────

  /** transportHub.on('modechange', fn) → unsubscribe */
  on(event, fn) {
    this.addEventListener(event, fn);
    return () => this.removeEventListener(event, fn);
  }
}

// ── 单例 (自注册内置适配器) ──────────────────────────────────────────────────

const transportHub = new TransportHub();
transportHub.registerAll([foxgloveAdapter, rosbridgeAdapter]);
g.__transportHub = transportHub;

// 向后兼容: 保留 __dataService 别名
g.__dataService = transportHub;

export { TransportHub, transportHub };
