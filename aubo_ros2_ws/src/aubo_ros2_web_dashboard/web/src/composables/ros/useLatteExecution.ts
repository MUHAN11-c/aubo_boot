/**
 * useLatteExecution — 拉花轨迹执行 (ROS 路径)
 *
 * 通过 rosbridge 调用 /latte_imitation/replay_trajectory 服务,
 * 执行 mode="action" 的完整 6 阶段管线喵~
 *
 * 与 useLattePreview (非ROS/BFF) 对称:
 *   - useLattePreview: fetch → BFF → 纯Python计算 → JSON waypoints
 *   - useLatteExecution: rosbridge → ROS Service → MoveIt2 规划+执行
 *
 * 用法:
 *   const { execute, executing } = useLatteExecution()
 *   const result = await execute({
 *     episode_idx: 0, arm: 'right', mode: 'action',
 *     roll_deg: 0, pitch_deg: 0, yaw_deg: 90,
 *     start_pose: { x: 0.5, y: -0.1, z: 0.3, qx: 0, qy: 0, qz: 0, qw: 1 },
 *   })
 */
import { useRosService } from '@/composables/ros/useRosService'
import { useDashboardSettings } from '@/composables/settings/useDashboardSettings'
import type { LatteStartPose } from '@/composables/api/useLattePreview'

/** 执行请求参数 (与 ReplayLatteTrajectory.srv 对齐) */
export interface LatteExecutionParams {
  episode_idx: number
  arm: string
  speed_scale: number
  mode: 'preview' | 'debug' | 'action'
  roll_deg: number; pitch_deg: number; yaw_deg: number
  tool_offset_id: string
  start_pose?: LatteStartPose
  // 参数化生成 (新增)
  pattern_type?: string; tulip_layers?: number
  cup_center_x?: number; cup_center_y?: number; cup_surface_z?: number; cup_radius?: number
  pour_mix_height_offset?: number; pour_draw_height_offset?: number; pour_finish_height_offset?: number
  pour_wiggle_amplitude?: number; pour_wiggle_frequency?: number
  pour_max_velocity?: number; pour_max_acceleration?: number; pour_max_jerk?: number
  enable_anti_sloshing?: boolean
}

/** 执行响应 */
export interface LatteExecutionResult {
  success: boolean
  message: string
  num_frames: number
  path_length: number
}

export function useLatteExecution() {
  const { call: rosCall, calling, lastResult, lastError } = useRosService()
  const settings = useDashboardSettings()

  const executing = calling

  /** 调用 ROS 服务执行拉花轨迹 喵~ */
  async function execute(params: LatteExecutionParams): Promise<LatteExecutionResult> {
    const svc = settings.rosName('svc-latte-replay', '/latte_imitation/replay_trajectory')
    const svcType = settings.serviceType('svc-latte-replay', 'ivg_interfaces/srv/ReplayLatteTrajectory')

    const req: Record<string, unknown> = {
      episode_idx: params.episode_idx,
      arm: params.arm,
      speed_scale: params.speed_scale,
      mode: params.mode,
      roll_deg: params.roll_deg,
      pitch_deg: params.pitch_deg,
      yaw_deg: params.yaw_deg,
      tool_offset_id: params.tool_offset_id,
    }

    // 参数化生成字段
    if (params.pattern_type) {
      req.pattern_type = params.pattern_type
      if (params.tulip_layers != null) req.tulip_layers = params.tulip_layers
      if (params.cup_center_x != null) req.cup_center_x = params.cup_center_x
      if (params.cup_center_y != null) req.cup_center_y = params.cup_center_y
      if (params.cup_surface_z != null) req.cup_surface_z = params.cup_surface_z
      if (params.cup_radius != null) req.cup_radius = params.cup_radius
      if (params.pour_mix_height_offset != null) req.pour_mix_height_offset = params.pour_mix_height_offset
      if (params.pour_draw_height_offset != null) req.pour_draw_height_offset = params.pour_draw_height_offset
      if (params.pour_finish_height_offset != null) req.pour_finish_height_offset = params.pour_finish_height_offset
      if (params.pour_wiggle_amplitude != null) req.pour_wiggle_amplitude = params.pour_wiggle_amplitude
      if (params.pour_wiggle_frequency != null) req.pour_wiggle_frequency = params.pour_wiggle_frequency
      if (params.pour_max_velocity != null) req.pour_max_velocity = params.pour_max_velocity
      if (params.pour_max_acceleration != null) req.pour_max_acceleration = params.pour_max_acceleration
      if (params.pour_max_jerk != null) req.pour_max_jerk = params.pour_max_jerk
      if (params.enable_anti_sloshing != null) req.enable_anti_sloshing = params.enable_anti_sloshing
    }

    // 传入当前 EE 位姿 (后端 ROS 节点可用此值也可自行从 TF 获取) 喵~
    if (params.start_pose) {
      req.start_pose = {
        position: { x: params.start_pose.x, y: params.start_pose.y, z: params.start_pose.z },
        orientation: { x: params.start_pose.qx, y: params.start_pose.qy, z: params.start_pose.qz, w: params.start_pose.qw },
      }
    }

    const result: any = await rosCall(svc, svcType, req)
    return {
      success: result?.success ?? false,
      message: result?.message ?? (result?.success ? '执行完成' : '执行失败'),
      num_frames: result?.num_frames ?? 0,
      path_length: result?.path_length ?? 0.0,
    }
  }

  return {
    execute,
    executing,
    lastResult,
    lastError,
  }
}
