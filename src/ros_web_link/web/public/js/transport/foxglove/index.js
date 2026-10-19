// transport/foxglove/index.js — Foxglove 协议层 barrel（L0/L1/L2 + 门面 + action）
//
// 应用侧通常只 import /js/ros_web_link.js；需要自定义协议/播放器/编解码时再用这里。
export { FoxgloveProtocolClient, SUBPROTOCOL } from './protocol.js';
export { FoxglovePlayer } from './player.js';
export { CodecCache, buildCodec } from './codec.js';
export {
  foxgloveRosClient,
  FoxgloveRosClient,
  valueFromParameterValue,
  parameterValueFrom,
  PARAM_TYPE,
} from './ros_client.js';
export { createActionClient, GOAL_STATUS, GOAL_STATUS_NAMES } from './action_client.js';
