// latte/preview.js — BFF 轨迹预览 API 调用
// POST /api/v1/latte/trajectory/preview → 返回 waypoints (tcp_path, spout_path, cup_pose, workspace_bounds)
// 纯 HTTP，零 ROS 依赖喵~

const LATTE_PREVIEW_API = '/api/v1/latte/trajectory/preview';

/**
 * 调用 BFF 生成拉花轨迹预览
 * @param {Object} requestBody - 请求体 (与 latte_controls.js _buildRequest 格式一致)
 * @returns {Promise<Object>} { tcp_path, spout_path, cup_pose, workspace_bounds, num_frames, path_length, message, success }
 */
export async function fetchTrajectoryPreview(requestBody) {
    const resp = await fetch(LATTE_PREVIEW_API, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(requestBody),
    });
    if (!resp.ok) {
        const text = await resp.text().catch(() => '');
        throw new Error(`BFF ${resp.status}: ${text}`);
    }
    return resp.json();
}
