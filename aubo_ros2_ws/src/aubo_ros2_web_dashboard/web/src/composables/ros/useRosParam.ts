/**
 * useRosParam — ROS 2 参数操作 composable
 *
 * 通过 rosbridge call_service 调用 ROS 节点的标准参数接口:
 *   /<node>/set_parameters        → rcl_interfaces/srv/SetParameters
 *   /<node>/get_parameters        → rcl_interfaces/srv/GetParameters
 *   /<node>/list_parameters       → rcl_interfaces/srv/ListParameters
 *   /<node>/set_parameters_atomically → rcl_interfaces/srv/SetParametersAtomically
 *
 * 触发 ROS 节点的 on_set_parameters_callback 自动验证喵~
 *
 * 用法:
 *   const { setParam, getParam } = useRosParam()
 *   await setParam('/latte_imitation', 'speed_scale', 0.5)
 *   const val = await getParam('/latte_imitation', 'speed_scale')
 */
import { useRos } from './useRos'

/** ROS 2 参数类型枚举 (rcl_interfaces/msg/ParameterType) */
const PARAM_TYPE = { BOOL: 1, INTEGER: 2, DOUBLE: 3, STRING: 4 } as const

type ParamValue = { type: number; bool_value?: boolean; integer_value?: number; double_value?: number; string_value?: string }

function toParamValue(name: string, value: unknown): { name: string; value: ParamValue } {
  if (typeof value === 'boolean') return { name, value: { type: PARAM_TYPE.BOOL, bool_value: value } }
  if (typeof value === 'number' && Number.isInteger(value)) return { name, value: { type: PARAM_TYPE.INTEGER, integer_value: value } }
  if (typeof value === 'number') return { name, value: { type: PARAM_TYPE.DOUBLE, double_value: value } }
  return { name, value: { type: PARAM_TYPE.STRING, string_value: String(value) } }
}

export function useRosParam() {
  const { callService } = useRos()

  /** 设置参数值 (触发节点 on_set_parameters_callback 验证) 喵~ */
  async function setParam(node: string, name: string, value: unknown, atomically = false): Promise<{ ok: boolean; reason: string }> {
    const svc = atomically ? `/${node}/set_parameters_atomically` : `/${node}/set_parameters`
    const type = atomically ? 'rcl_interfaces/srv/SetParametersAtomically' : 'rcl_interfaces/srv/SetParameters'
    try {
      const pv = toParamValue(name, value)
      const resp: any = await callService(svc, type, { parameters: [pv] })
      const r = resp?.results?.[0] ?? resp
      return { ok: r?.successful ?? false, reason: r?.reason ?? '' }
    } catch (e: any) {
      return { ok: false, reason: String(e?.message ?? e) }
    }
  }

  /** 原子设置多个参数 (任一失败则全部回滚) 喵~ */
  async function setParams(node: string, params: Record<string, unknown>): Promise<{ ok: boolean; reason: string }> {
    try {
      const parameters = Object.entries(params).map(([name, value]) => toParamValue(name, value))
      const resp: any = await callService(`/${node}/set_parameters_atomically`, 'rcl_interfaces/srv/SetParametersAtomically', { parameters })
      return { ok: resp?.result?.successful ?? false, reason: resp?.result?.reason ?? '' }
    } catch (e: any) {
      return { ok: false, reason: String(e?.message ?? e) }
    }
  }

  /** 获取参数值 喵~ */
  async function getParam(node: string, name: string): Promise<unknown> {
    try {
      const resp: any = await callService(`/${node}/get_parameters`, 'rcl_interfaces/srv/GetParameters', { names: [name] })
      const pv = resp?.values?.[0]
      if (!pv) return undefined
      if (pv.type === PARAM_TYPE.BOOL) return pv.bool_value
      if (pv.type === PARAM_TYPE.INTEGER) return pv.integer_value
      if (pv.type === PARAM_TYPE.DOUBLE) return pv.double_value
      return pv.string_value
    } catch {
      return undefined
    }
  }

  return { setParam, setParams, getParam }
}
