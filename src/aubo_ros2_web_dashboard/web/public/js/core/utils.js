// core/utils.js — 共享工具函数
// 所有页面 JS 均可 import 使用，消除 $(id) / escapeHtml / canonicalRosTopic 重复定义

/** getElementById 快捷方式 */
export function $(id) {
  return document.getElementById(id);
}

/** HTML 实体转义 */
export function escapeHtml(s) {
  return String(s)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#39;');
}

/** 规范化 ROS 话题名: 确保以 / 开头, 末尾无多余 / */
export function canonicalRosTopic(t) {
  const s = String(t || '').trim();
  if (!s) return '';
  return s.startsWith('/') ? s : '/' + s;
}

/** 将话题名中的 / 编码为 %2F, 用于 URL query 参数 */
export function encodeTopicQueryValue(topic) {
  return String(topic).split('/').map(function (seg) {
    return encodeURIComponent(seg);
  }).join('/');
}

/** 从 ROS 消息中提取数组字段 (兼容 Array / {data:[]} / array-like) 喵~ */
export function rosMsgArrayField(msg, key) {
  if (!msg || typeof msg !== 'object') return [];
  const v = msg[key];
  if (v == null) return [];
  if (Array.isArray(v)) return v;
  if (typeof v === 'object' && Array.isArray(v.data)) return v.data;
  if (typeof v === 'object' && typeof v.length === 'number') {
    try { return Array.prototype.slice.call(v); } catch (e) { return []; }
  }
  return [];
}
