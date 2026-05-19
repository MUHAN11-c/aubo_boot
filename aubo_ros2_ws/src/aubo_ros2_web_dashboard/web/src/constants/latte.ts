/**
 * 拉花轨迹调试面板常量
 *
 * 所有拉花相关常量集中定义, 避免分散硬编码喵~
 */

// ═══════════════════════ BFF API ═══════════════════════

/** 拉花轨迹预览 API 端点 */
export const LATTE_PREVIEW_API = '/api/v1/latte/trajectory/preview'

// ═══════════════════════ Episode ═══════════════════════

export const DEFAULT_EPISODE = 0
export const MAX_EPISODE = 39

// ═══════════════════════ RPY 默认值 ═══════════════════════

export const DEFAULT_RPY = { roll: 0, pitch: 0, yaw: 0 }

// ═══════════════════════ 速度倍率 ═══════════════════════

export const DEFAULT_SPEED_SCALE = 1.0
export const MIN_SPEED_SCALE = 0.01
export const MAX_SPEED_SCALE = 10.0

// ═══════════════════════ 工具 ═══════════════════════

export const DEFAULT_TOOL_ID = 'default'

// ═══════════════════════ localStorage ═══════════════════════

/** RPY 角度 localStorage key */
export const LATTE_RPY_STORAGE_KEY = 'ivg_latte_rpy_v1'

// ═══════════════════════ Workspace 默认值 ═══════════════════════

/** AUBO E5 工作空间安全边界 (与 config/workspace_safety.yaml 同步)
 *
 *  基于 AUBO E5 官方工作半径 886.5mm + URDF DH 链审计 喵~
 *  来源: AUBO Robotics Catalog 2025 + aubo_e5_10.urdf 喵~
 */
export const DEFAULT_WORKSPACE = {
  x_min: -0.87, x_max: 0.87,
  y_min: -0.87, y_max: 0.87,
  z_min: -0.85, z_max: 1.10,
}

// ═══════════════════════ 图案类型 ═══════════════════════

export const PATTERN_TYPES = [
  { value: '', label: '录制回放 (Episode)' },
  { value: 'heart', label: '心形 (Heart)' },
  { value: 'rosetta', label: '树叶 (Rosetta)' },
  { value: 'tulip', label: '郁金香 (Tulip)' },
  { value: 'swan', label: '天鹅 (Swan)' },
]

// ═══════════════════════ 杯子参数默认值 ═══════════════════════

export const DEFAULT_CUP = {
  centerX: 0.0,
  centerY: 0.0,
  surfaceZ: 0.15,
  radius: 0.04,
}

// ═══════════════════════ 倾倒参数默认值 ═══════════════════════

export const DEFAULT_POUR = {
  mixHeightOffset: 0.076,
  drawHeightOffset: 0.006,
  finishHeightOffset: 0.076,
  wiggleAmplitude: 0.006,
  wiggleFrequency: 5.0,
  maxVelocity: 0.05,
  maxAcceleration: 0.1,
  maxJerk: 0.5,
  enableAntiSloshing: true,
}
