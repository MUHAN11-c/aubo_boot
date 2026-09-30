// transport/adapter.js — MessageAdapter 抽象接口
//
// 定义所有 Bridge 适配器必须实现的统一接口。
// JS 无真实 interface, 使用 JSDoc + 基类 throw Error 约定。
//
// 设计原则:
//   - 依赖倒置: 上层依赖此接口, 不依赖具体 Adapter 实现
//   - 统一输出: 所有 Adapter 输出 TypedMessage 格式
//   - 开闭原则: 新增 Bridge 类型只需实现此接口

/**
 * @interface MessageAdapter
 *
 * TypedMessage 统一消息格式:
 *   { topic: string, type: string, data: unknown, timestamp: number, bridge: 'foxglove'|'rosbridge' }
 *
 * BridgeStatus:
 *   { state: 'disconnected'|'connecting'|'connected'|'error', error?: string, since: number }
 *
 * BridgeStats:
 *   { channelsCount, servicesCount, bytesReceived, messagesReceived, connectedAt }
 */

class MessageAdapter {
  /** @returns {string} 适配器唯一标识 */
  get id() { throw new Error('abstract: id'); }

  /** @returns {boolean} */
  get isConnected() { throw new Error('abstract: isConnected'); }

  /**
   * 建立连接 (等待握手完成)
   * @param {string} url - WebSocket URL
   * @returns {Promise<void>}
   */
  async connect(url) { throw new Error('abstract: connect'); }

  /** 断开连接 */
  disconnect() { throw new Error('abstract: disconnect'); }

  /**
   * 订阅话题, 消息自动解码为 TypedMessage 格式
   * @param {string} topic
   * @param {string} msgType - ROS 2 消息类型 (用于 decoder 选择)
   * @param {function(TypedMessage): void} callback
   * @returns {{cancel: function}} 订阅句柄
   */
  subscribe(topic, msgType, callback) { throw new Error('abstract: subscribe'); }

  /**
   * 取消订阅
   * @param {string} topic
   */
  unsubscribe(topic) { throw new Error('abstract: unsubscribe'); }

  /**
   * 发布消息
   * @param {string} topic
   * @param {string} msgType
   * @param {object} msg
   * @returns {Promise<void>}
   */
  async publish(topic, msgType, msg) { throw new Error('abstract: publish'); }

  /**
   * 调用 ROS 2 服务
   * @param {string} name - 服务名
   * @param {string} type - 服务类型
   * @param {object} request - 请求体
   * @param {number} [timeoutMs=60000]
   * @returns {Promise<object>}
   */
  async callService(name, type, request, timeoutMs = 60000) { throw new Error('abstract: callService'); }

  /**
   * 注册消息回调
   * @param {function(TypedMessage): void} callback
   * @returns {function} unsubscribe 函数
   */
  onMessage(callback) { throw new Error('abstract: onMessage'); }

  /**
   * 注册状态变更回调
   * @param {function(BridgeStatus): void} callback
   * @returns {function} unsubscribe 函数
   */
  onStatusChange(callback) { throw new Error('abstract: onStatusChange'); }

  /** @returns {Set<string>} 能力集合 */
  getCapabilities() { throw new Error('abstract: getCapabilities'); }

  /** @returns {BridgeStats} */
  getStats() { throw new Error('abstract: getStats'); }
}

// ── BridgeMode 枚举 ──────────────────────────────────────────────

const BridgeMode = Object.freeze({
  AUTO: 'auto',
  FOXGLOVE: 'foxglove',
  ROSBRIDGE: 'rosbridge',
});

export { MessageAdapter, BridgeMode };
