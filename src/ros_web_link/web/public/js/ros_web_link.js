// ros_web_link.js — 本包浏览器侧稳定入口（barrel）
//
// 应用代码优先从这里导入，而不是直接依赖 transport/foxglove/ 的深路径：
//
//   import { foxgloveRosClient, createActionClient } from '/js/ros_web_link.js';
//
// 门面语义对齐 roslibjs（Topic/Service/Param/Action）；底层 protocol/player/codec
// 仍可从 transport/foxglove/index.js 或各自模块导入，供高级用法使用。
export {
  foxgloveRosClient,
  FoxgloveRosClient,
  valueFromParameterValue,
  parameterValueFrom,
  PARAM_TYPE,
} from './transport/foxglove/ros_client.js';

export {
  createActionClient,
  GOAL_STATUS,
  GOAL_STATUS_NAMES,
} from './transport/foxglove/action_client.js';
