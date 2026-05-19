/**
 * useLattePreview — 拉花轨迹预览 (非ROS 路径)
 *
 * 调用 FastAPI BFF 端点 POST /api/v1/latte/trajectory/preview。
 * 纯 HTTP fetch，不依赖 roslib / rosbridge。
 *
 * 网关进程无 ROS 2 环境，TF 查询不可靠 → 前端负责传入当前 EE 位姿 (start_pose) 喵~
 *
 * 用法:
 *   const { preview, loading, lastData } = useLattePreview()
 *   const data = await preview({
 *     episode_idx: 0, arm: 'right',
 *     roll_deg: 0, pitch_deg: 0, yaw_deg: 90,
 *     start_pose: { x: 0.5, y: -0.1, z: 0.3, qx: 0, qy: 0, qz: 0, qw: 1 },
 *   })
 *   // data.tcp_path, data.spout_path, data.cup_pose, data.workspace_bounds
 */
import { LATTE_PREVIEW_API } from '@/constants/latte'

/** 轨迹 waypoint */
interface LatteWaypoint {
  x: number; y: number; z: number
  qx: number; qy: number; qz: number; qw: number
}

/** 起点位姿 (前端从 rosbridge 获取的当前 EE 位姿) */
export interface LatteStartPose {
  x: number; y: number; z: number
  qx: number; qy: number; qz: number; qw: number
}

/** 预览请求参数 */
interface LattePreviewParams {
  episode_idx: number
  arm: string
  roll_deg: number
  pitch_deg: number
  yaw_deg: number
  speed_scale?: number
  tool_offset_id?: string
  start_pose?: LatteStartPose
  // 参数化生成 (新增)
  pattern_type?: string
  tulip_layers?: number
  cup_center_x?: number; cup_center_y?: number; cup_surface_z?: number; cup_radius?: number
  pour_mix_height_offset?: number; pour_draw_height_offset?: number; pour_finish_height_offset?: number
  pour_wiggle_amplitude?: number; pour_wiggle_frequency?: number
  pour_max_velocity?: number; pour_max_acceleration?: number; pour_max_jerk?: number
  enable_anti_sloshing?: boolean
}

/** 杯子位姿 */
interface LatteCupPose {
  x: number; y: number; z: number
  qx: number; qy: number; qz: number; qw: number
}

/** 工作空间边界 */
interface LatteWorkspaceBounds {
  x_min: number; x_max: number
  y_min: number; y_max: number
  z_min: number; z_max: number
}

/** 预览响应 */
export interface LattePreviewResponse {
  success: boolean
  num_frames: number
  path_length: number
  tcp_path: LatteWaypoint[]
  spout_path: LatteWaypoint[]
  cup_pose: LatteCupPose
  workspace_bounds: LatteWorkspaceBounds
  message: string
}

export function useLattePreview() {
  const loading = ref(false)
  const error = ref<string | null>(null)
  const lastData = ref<LattePreviewResponse | null>(null)

  /** 调用 BFF 获取轨迹预览数据 喵~ */
  async function preview(params: LattePreviewParams): Promise<LattePreviewResponse> {
    loading.value = true; error.value = null

    const body: Record<string, unknown> = {
      episode_idx: params.episode_idx,
      arm: params.arm ?? 'right',
      roll_deg: params.roll_deg ?? 0.0,
      pitch_deg: params.pitch_deg ?? 0.0,
      yaw_deg: params.yaw_deg ?? 0.0,
      speed_scale: params.speed_scale ?? 1.0,
      tool_offset_id: params.tool_offset_id ?? 'default',
    }

    // 参数化生成字段
    if (params.pattern_type) {
      body.pattern_type = params.pattern_type
      if (params.tulip_layers != null) body.tulip_layers = params.tulip_layers
      if (params.cup_center_x != null) body.cup_center_x = params.cup_center_x
      if (params.cup_center_y != null) body.cup_center_y = params.cup_center_y
      if (params.cup_surface_z != null) body.cup_surface_z = params.cup_surface_z
      if (params.cup_radius != null) body.cup_radius = params.cup_radius
      if (params.pour_mix_height_offset != null) body.pour_mix_height_offset = params.pour_mix_height_offset
      if (params.pour_draw_height_offset != null) body.pour_draw_height_offset = params.pour_draw_height_offset
      if (params.pour_finish_height_offset != null) body.pour_finish_height_offset = params.pour_finish_height_offset
      if (params.pour_wiggle_amplitude != null) body.pour_wiggle_amplitude = params.pour_wiggle_amplitude
      if (params.pour_wiggle_frequency != null) body.pour_wiggle_frequency = params.pour_wiggle_frequency
      if (params.pour_max_velocity != null) body.pour_max_velocity = params.pour_max_velocity
      if (params.pour_max_acceleration != null) body.pour_max_acceleration = params.pour_max_acceleration
      if (params.pour_max_jerk != null) body.pour_max_jerk = params.pour_max_jerk
      if (params.enable_anti_sloshing != null) body.enable_anti_sloshing = params.enable_anti_sloshing
    }

    // 前端传入当前 EE 位姿 (绕过 BFF 的 TF 查询, 网关无 ROS 环境) 喵~
    if (params.start_pose) {
      body.start_pose = {
        x: params.start_pose.x, y: params.start_pose.y, z: params.start_pose.z,
        qx: params.start_pose.qx, qy: params.start_pose.qy,
        qz: params.start_pose.qz, qw: params.start_pose.qw,
      }
    }

    try {
      const resp = await fetch(LATTE_PREVIEW_API, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      })
      if (!resp.ok) {
        const text = await resp.text().catch(() => '')
        throw new Error(`BFF ${resp.status}: ${text || resp.statusText}`)
      }
      const data: LattePreviewResponse = await resp.json()
      lastData.value = data
      return data
    } catch (e: any) {
      const msg = String(e?.message ?? e)
      error.value = msg
      throw e
    } finally {
      loading.value = false
    }
  }

  return {
    preview,
    loading: readonly(loading),
    error: readonly(error),
    lastData: readonly(lastData),
  }
}
