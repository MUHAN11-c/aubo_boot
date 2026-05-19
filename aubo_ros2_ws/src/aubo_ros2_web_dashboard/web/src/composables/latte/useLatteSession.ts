/**
 * useLatteSession — 拉花轨迹统一状态管理
 *
 * 管理所有拉花相关参数: 轨迹源 / 杯子 / 倾倒 / 变换
 * 持久化 RPY 到 localStorage
 */
import { DEFAULT_EPISODE, MAX_EPISODE, DEFAULT_RPY, DEFAULT_SPEED_SCALE,
         MIN_SPEED_SCALE, MAX_SPEED_SCALE, DEFAULT_TOOL_ID, LATTE_RPY_STORAGE_KEY,
         PATTERN_TYPES, DEFAULT_CUP, DEFAULT_POUR } from '@/constants/latte'

export function useLatteSession() {
  // ── 轨迹源 ──
  const patternType = ref('')
  const episodeIdx = ref(DEFAULT_EPISODE)
  const tulipLayers = ref(3)

  // ── 杯子参数 ──
  const cupCenterX = ref(DEFAULT_CUP.centerX)
  const cupCenterY = ref(DEFAULT_CUP.centerY)
  const cupSurfaceZ = ref(DEFAULT_CUP.surfaceZ)
  const cupRadius = ref(DEFAULT_CUP.radius)

  // ── 倾倒参数 ──
  const pourMixHeight = ref(DEFAULT_POUR.mixHeightOffset)
  const pourDrawHeight = ref(DEFAULT_POUR.drawHeightOffset)
  const pourFinishHeight = ref(DEFAULT_POUR.finishHeightOffset)
  const pourWiggleAmp = ref(DEFAULT_POUR.wiggleAmplitude)
  const pourWiggleFreq = ref(DEFAULT_POUR.wiggleFrequency)
  const pourMaxVel = ref(DEFAULT_POUR.maxVelocity)
  const pourMaxAcc = ref(DEFAULT_POUR.maxAcceleration)
  const pourMaxJerk = ref(DEFAULT_POUR.maxJerk)
  const pourAntiSlosh = ref(DEFAULT_POUR.enableAntiSloshing)
  const pourAdvancedOpen = ref(false)

  // ── 变换参数 ──
  const rpyRoll = ref(DEFAULT_RPY.roll)
  const rpyPitch = ref(DEFAULT_RPY.pitch)
  const rpyYaw = ref(DEFAULT_RPY.yaw)
  const speedScale = ref(DEFAULT_SPEED_SCALE)
  const toolId = ref(DEFAULT_TOOL_ID)
  const execMode = ref<'preview' | 'action'>('preview')

  // ── 结果 ──
  const message = ref('')
  const result = ref<any>(null)

  // ── RPY localStorage ──
  try {
    const saved = localStorage.getItem(LATTE_RPY_STORAGE_KEY)
    if (saved) {
      const r = JSON.parse(saved)
      if (typeof r.roll === 'number') rpyRoll.value = r.roll
      if (typeof r.pitch === 'number') rpyPitch.value = r.pitch
      if (typeof r.yaw === 'number') rpyYaw.value = r.yaw
    }
  } catch { /* ignore */ }

  function saveRpy() {
    try {
      localStorage.setItem(LATTE_RPY_STORAGE_KEY, JSON.stringify({
        roll: rpyRoll.value, pitch: rpyPitch.value, yaw: rpyYaw.value,
      }))
    } catch { /* ignore */ }
  }

  /** 构建统一的请求参数 (同时用于 BFF 预览和 ROS2 执行) */
  function buildRequest(mode: 'preview' | 'action', startPose?: any): Record<string, unknown> {
    const req: Record<string, unknown> = {
      episode_idx: episodeIdx.value,
      arm: 'right',
      speed_scale: speedScale.value,
      mode,
      roll_deg: rpyRoll.value,
      pitch_deg: rpyPitch.value,
      yaw_deg: rpyYaw.value,
      tool_offset_id: toolId.value,
    }

    if (patternType.value) {
      req.pattern_type = patternType.value
      req.tulip_layers = tulipLayers.value
      req.cup_center_x = cupCenterX.value
      req.cup_center_y = cupCenterY.value
      req.cup_surface_z = cupSurfaceZ.value
      req.cup_radius = cupRadius.value
      req.pour_mix_height_offset = pourMixHeight.value
      req.pour_draw_height_offset = pourDrawHeight.value
      req.pour_finish_height_offset = pourFinishHeight.value
      req.pour_wiggle_amplitude = pourWiggleAmp.value
      req.pour_wiggle_frequency = pourWiggleFreq.value
      req.pour_max_velocity = pourMaxVel.value
      req.pour_max_acceleration = pourMaxAcc.value
      req.pour_max_jerk = pourMaxJerk.value
      req.enable_anti_sloshing = pourAntiSlosh.value
    }

    if (startPose) {
      req.start_pose = {
        x: startPose.x, y: startPose.y, z: startPose.z,
        qx: startPose.qx, qy: startPose.qy, qz: startPose.qz, qw: startPose.qw,
      }
    }

    return req
  }

  return {
    patternType, episodeIdx, tulipLayers,
    cupCenterX, cupCenterY, cupSurfaceZ, cupRadius,
    pourMixHeight, pourDrawHeight, pourFinishHeight,
    pourWiggleAmp, pourWiggleFreq,
    pourMaxVel, pourMaxAcc, pourMaxJerk, pourAntiSlosh, pourAdvancedOpen,
    rpyRoll, rpyPitch, rpyYaw, speedScale, toolId, execMode,
    message, result,
    saveRpy, buildRequest,
  }
}
