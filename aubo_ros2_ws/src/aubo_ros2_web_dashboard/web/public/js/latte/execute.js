// latte/execute.js — ROS 轨迹执行服务调用
// 通过 rosbridge → /latte_imitation/replay_trajectory (ivg_interfaces/srv/ReplayLatteTrajectory)
// 依赖 core/ros.js 单例喵~

import { ros } from '../core/ros.js';

const DEFAULT_SERVICE = '/latte_imitation/replay_trajectory';
const DEFAULT_SERVICE_TYPE = 'ivg_interfaces/srv/ReplayLatteTrajectory';

/**
 * 执行拉花轨迹
 * @param {Object} requestBody - 请求体 (与 latte_controls.js _buildRequest 一致)
 * @param {string} [serviceName] - ROS 服务名
 * @param {string} [serviceType] - ROS 服务类型
 * @returns {Promise<Object>} 服务响应
 */
export async function executeTrajectory(requestBody, serviceName, serviceType) {
    const svc = serviceName || DEFAULT_SERVICE;
    const type = serviceType || DEFAULT_SERVICE_TYPE;
    return ros.callService(svc, type, requestBody, 120000); // 2 分钟超时（轨迹执行可能较长）
}
