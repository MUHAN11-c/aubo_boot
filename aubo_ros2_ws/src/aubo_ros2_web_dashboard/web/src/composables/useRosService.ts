/**
 * useRosService — ROS 服务调用 composable
 *
 * 替代旧版: services.js (300行) 中的所有 callSetBool/callExecuteGrasp 等
 *
 * 提供常用服务的类型安全快捷方法 + 底层通用 call() 函数
 * 自动管理 calling/error 状态
 *
 * 用法:
 *   const { executeGrasp, changeTool, calling } = useRosService()
 *   const result = await executeGrasp('obj_1', true)
 */
import { useRos } from './useRos'

export function useRosService() {
  const { callService } = useRos()

  const calling = ref(false)
  const lastResult = ref<any>(null)
  const lastError = ref<string | null>(null)

  /** 底层通用服务调用 — 自动管理 calling/error 状态 */
  async function call(service: string, type: string, request?: Record<string, unknown>, timeoutMs?: number) {
    calling.value = true; lastError.value = null; lastResult.value = null
    try { const r = await callService(service, type, request, timeoutMs); lastResult.value = r; return r }
    catch (e: any) { lastError.value = String(e?.message ?? e); throw e }
    finally { calling.value = false }
  }

  // ═══════════════════════ 常用服务快捷方法 ═══════════════════════

  /** SetBool 类型服务 (std_srvs/SetBool) */
  async function callSetBool(service: string, data: boolean) {
    return call(service, 'std_srvs/srv/SetBool', { data })
  }

  /** 执行单次抓取 (工件/AI 模式) — ivg_interfaces/ExecuteGraspPose */
  async function executeGrasp(objectId: string, useVisual: boolean) {
    return call('/execute_single_grasp', 'ivg_interfaces/srv/ExecuteGraspPose', {
      object_id: objectId, use_visual_estimation: useVisual,
    })
  }

  /** 工具快换 — ivg_interfaces/ChangeTool */
  async function changeTool(toolId: string) {
    return call('/change_tool', 'ivg_interfaces/srv/ChangeTool', { tool_id: toolId })
  }

  /** 夹爪快换 (方向) — ivg_interfaces/RunGripperSwap */
  async function runGripperSwap(direction: string) {
    return call('/run_gripper_swap', 'ivg_interfaces/srv/RunGripperSwap', { direction })
  }

  /** 获取当前工具 */
  async function getCurrentTool() {
    return call('/get_current_tool', 'ivg_interfaces/srv/GetCurrentTool')
  }

  /** 咖啡拉花 DO 开关 — std_srvs/SetBool */
  async function setLatteDo(doId: 2 | 4, on: boolean) {
    return call(`/set_latte_do${doId}`, 'std_srvs/srv/SetBool', { data: on })
  }

  /** 循环抓取控制 */
  async function setLoopGraspControl(on: boolean) {
    return call('/loop_grasp_control', 'std_srvs/srv/SetBool', { data: on })
  }

  /** GraspNet 采集控制 */
  async function setGraspnetCapture(on: boolean) {
    return call('/graspnet_capture_control', 'std_srvs/srv/SetBool', { data: on })
  }

  /** GraspNet 循环发布控制 */
  async function setPublishGraspsLoop(on: boolean) {
    return call('/publish_grasps_worker_loop_control', 'std_srvs/srv/SetBool', { data: on })
  }

  return {
    calling: readonly(calling), lastResult: readonly(lastResult), lastError: readonly(lastError),
    call, callSetBool, executeGrasp, changeTool, runGripperSwap, getCurrentTool,
    setLatteDo, setLoopGraspControl, setGraspnetCapture, setPublishGraspsLoop,
  }
}
