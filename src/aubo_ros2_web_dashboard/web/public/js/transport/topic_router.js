// transport/topic_router.js — 话题级路由 (模式锁定 v3.2)
//
// 规则:
//   FOXGLOVE  → FoxgloveAdapter ONLY (不可用返回 null)
//   ROSBRIDGE → RosbridgeAdapter ONLY (不可用返回 null)
//   AUTO      → Foxglove 优先, 不可用降级 Rosbridge
//
// channel 不存在时 warn (不回退，保持模式锁定)

import { BridgeMode } from './adapter.js';

class TopicRouter {
  /**
   * @param {string} topic
   * @param {string} msgType
   * @param {BridgeMode} mode
   * @param {{foxglove: MessageAdapter, rosbridge: MessageAdapter}} adapters
   * @returns {MessageAdapter | null}
   */
  resolve(topic, msgType, mode, adapters) {
    if (mode === BridgeMode.FOXGLOVE) {
      if (!adapters.foxglove?.isConnected) return null;
      // channel 不存在时 warn (保持模式锁定，不回退)
      if (!adapters.foxglove._client?.getChannel(topic)) {
        console.warn('[TopicRouter] ' + topic + ' not in foxglove channels (mode locked)');
      }
      return adapters.foxglove;
    }
    if (mode === BridgeMode.ROSBRIDGE) {
      return adapters.rosbridge?.isConnected ? adapters.rosbridge : null;
    }
    // AUTO: foxglove 优先, 不可用降级 rosbridge
    if (adapters.foxglove?.isConnected) return adapters.foxglove;
    if (adapters.rosbridge?.isConnected) return adapters.rosbridge;
    return null;
  }
}

const topicRouter = new TopicRouter();

export { TopicRouter, topicRouter };
