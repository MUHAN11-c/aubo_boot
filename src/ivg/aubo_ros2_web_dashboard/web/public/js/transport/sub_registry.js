// transport/sub_registry.js — 订阅生命周期管理
//
// 维护所有活跃订阅的注册表, 支持跨 adapter 迁移 ("先建后拆"零丢消息)。
//
// 场景:
//   - 用户切换 Bridge 模式 → migrateAll(toAdapter)
//   - 新 adapter 先建立订阅 → 收到首条消息 → 取消旧 adapter 订阅
//
// 活跃条目: topic → { msgType, adapter, callbacks: Set<callback> }

class SubRegistry {
  constructor() {
    this._active = new Map();  // topic → { msgType, adapter, callbacks }
  }

  /**
   * 注册一个活跃订阅
   */
  register(topic, msgType, adapter, callback) {
    let entry = this._active.get(topic);
    if (!entry) {
      entry = { msgType, adapter, callbacks: new Set() };
      this._active.set(topic, entry);
    }
    entry.callbacks.add(callback);
    return entry;
  }

  /**
   * 取消注册一个话题的 callback
   */
  unregister(topic, callback) {
    const entry = this._active.get(topic);
    if (!entry) return;
    entry.callbacks.delete(callback);
    if (entry.callbacks.size === 0) {
      this._active.delete(topic);
      return null;  // signals: last callback removed, unsubscribe
    }
    return entry;
  }

  /**
   * 获取话题的注册条目
   */
  get(topic) {
    return this._active.get(topic);
  }

  /**
   * 将所有活跃订阅迁移到新 adapter ("先建后拆", 逐 topic 最小化双投窗口)
   *
   * 每 topic 独立迁移: 新 adapter 收到首条消息后立即取消旧 adapter 订阅,
   * 重叠窗口控制在首条消息延迟 (通常 <100ms) 而非全量 Phase 等待时间。
   *
   * @param {MessageAdapter} toAdapter
   * @param {function(string, string, function): void} subscribeFn
   * @returns {Promise<void>}
   */
  async migrateAll(toAdapter, subscribeFn) {
    const entries = Array.from(this._active.entries());

    for (const [topic, entry] of entries) {
      const prevAdapter = entry.adapter;
      if (prevAdapter === toAdapter) continue;

      await new Promise((resolve) => {
        let resolved = false;
        function done() {
          if (resolved) return;
          resolved = true;
          // 取消旧 adapter 订阅 (收到首条消息或超时后)
          if (prevAdapter) prevAdapter.unsubscribe(topic);
          entry.adapter = toAdapter;
          resolve();
        }

        const result = subscribeFn(toAdapter, topic, entry.msgType, (msg) => {
          // 首条消息到达 → 立即切断旧源，消除双投窗口
          done();
          // 转发给所有 callback
          for (const cb of entry.callbacks) {
            try { cb(msg); } catch (_) {}
          }
        });

        if (result && result.then) {
          result.then(() => { /* subscribe 完成, 等待首条消息或超时 */ });
        }

        // 超时保护: 500ms 后强制完成迁移
        setTimeout(() => done(), 500);
      });
    }

    return entries;
  }

  /**
   * 获取所有活跃话题
   */
  get topics() {
    return Array.from(this._active.keys());
  }

  get size() {
    return this._active.size;
  }

  clear() {
    this._active.clear();
  }
}

export { SubRegistry };
