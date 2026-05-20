/**
 * 工具函数 — 项目全局共享的基础工具
 *
 * 设计原则: 纯函数、零依赖、与任何框架无关
 * 来源: 从旧版 core/utils.js 搬移并添加类型标注
 */

/**
 * 规范化 ROS 话题名 — 确保以 / 开头
 * 例如 "joint_states" → "/joint_states"，"/joint_states" → "/joint_states"
 */
export function canonicalRosTopic(t: string): string {
  const s = String(t ?? '').trim()
  if (!s) return ''
  return s.startsWith('/') ? s : `/${s}`
}

/**
 * 将话题名中的 / 编码为 %2F — 用于 URL query 参数
 * 例如 "/camera/color/image_raw" → "%2Fcamera%2Fcolor%2Fimage_raw"
 */
export function encodeTopicQueryValue(topic: string): string {
  return String(topic).split('/').map(seg => encodeURIComponent(seg)).join('/')
}

/** 比较两个 ROS 话题名是否等价 (忽略前导/差异) 喵~ */
export function sameRosTopic(a: string, b: string): boolean {
  return canonicalRosTopic(a) === canonicalRosTopic(b)
}

/** 从 input 事件提取 number 值 喵~ */
export function inputNumberValue(e: Event): number {
  return Number((e.target as HTMLInputElement).value)
}
