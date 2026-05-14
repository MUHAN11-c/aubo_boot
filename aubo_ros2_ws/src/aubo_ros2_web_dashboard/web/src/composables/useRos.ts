/**
 * useRos — ROS 连接管理的全局单例 composable
 *
 * 替代旧版: ivg_transport.js (298行) + ros_connector.js (114行)
 *
 * 核心职责:
 *   1. Ros WebSocket 连接生命周期 (connect/disconnect/重连)
 *   2. 话题订阅/取消订阅 (subscribe/unsubscribe)
 *   3. 服务调用 (callService)
 *   4. 消息分发 (onRosJson / onControlJson)
 *
 * 架构要点:
 *   - 模块级单例 — 所有组件共享同一条 ROS 连接 (rosInstance)
 *   - 通过 rosbridge WebSocket 与 ROS 2 通信
 *   - 话题订阅自动去重 (topicSpecs 缓存)
 *   - 处理程序支持返回取消注册函数 (防止内存泄露)
 *
 * 用法:
 *   const { isConnected, subscribe, callService, onRosJson } = useRos()
 *   await connect()  // 建立连接
 *   subscribe('/joint_states', 'sensor_msgs/msg/JointState', 30)
 *   onRosJson('/joint_states', (msg) => { ... })
 */
import { Ros, Topic, Service } from 'roslib'
import { useRuntime } from './useRuntime'
import { canonicalRosTopic } from '@/lib/utils'

type RosHandler = { topic: string | null; fn: (msg: any, topic: string) => void }
type ControlHandler = (ctrl: { op: string; message?: string }) => void

// ═══════════════════════ 模块级单例状态 ═══════════════════════

const rosInstance = shallowRef<Ros | null>(null)
const connected = ref(false)
const topicSubs = new Map<string, Topic>()   // topic → Topic 实例
const topicSpecs = new Map<string, string>()         // topic → msgType (去重用)
const rosHandlers: RosHandler[] = []                 // 话题消息处理器
const controlHandlers: ControlHandler[] = []         // 控制面消息处理器
let connectPromise: Promise<void> | null = null      // 连接去重

// ═══════════════════════ 导出函数 ═══════════════════════

export function useRos() {
  const { rosbridgeWsUrl, load: loadRuntime } = useRuntime()

  /** 是否已连接到 rosbridge */
  function isConnected(): boolean { return connected.value }

  // ── 连接管理 ──

  /** 建立 rosbridge WebSocket 连接（自动去重，同一时间只有一个连接尝试） */
  async function connect(): Promise<void> {
    if (connectPromise) return connectPromise

    connectPromise = (async () => {
      await loadRuntime()
      const url = rosbridgeWsUrl()
      disconnect()  // 先断开旧连接

      return new Promise<void>((resolve, reject) => {
        let settled = false
        try {
          const ros = new Ros({ url: url as string })
          rosInstance.value = ros

          ros.on('connection', () => {
            if (settled) return; settled = true
            connected.value = true
            dispatchControl({ op: 'connection' })
            resolve()
          })
          ros.on('error', (err: any) => {
            if (settled) return; settled = true
            connected.value = false
            dispatchControl({ op: 'error', message: String(err ?? 'ros_error') })
            reject(new Error('ros_error'))
          })
          ros.on('close', () => {
            connected.value = false
            dispatchControl({ op: 'close', message: 'ros_closed' })
          })
        } catch (e) {
          settled = true; connected.value = false; reject(e)
        }
      })
    })()

    return connectPromise
  }

  /** 断开连接并清理所有订阅 */
  function disconnect() {
    unsubscribeAll()
    connectPromise = null
    try { rosInstance.value?.close() } catch { /* roslib 内部可能抛异常 */ }
    rosInstance.value = null
    connected.value = false
    clearRosHandlers()
    clearControlHandlers()
  }

  // ── 话题订阅 ──

  /**
   * 订阅 ROS 话题
   * @param topic    话题名 (自动规范化)
   * @param msgType  消息类型 (如 'sensor_msgs/msg/JointState')
   * @param maxHz    最大频率 (Hz, 0=不限制)
   * @returns true=订阅成功或已存在, false=ros 未连接
   */
  function subscribe(topic: string, msgType: string, maxHz?: number): boolean {
    const ros = rosInstance.value
    if (!ros) return false

    const topicName = canonicalRosTopic(topic)
    if (!topicName || !msgType) return false
    if (topicSpecs.get(topicName) === msgType) return true  // 已订阅相同类型，跳过

    // 覆盖旧订阅（类型变化时）
    const existing = topicSubs.get(topicName)
    if (existing) { try { existing.unsubscribe() } catch { /* */ }; topicSubs.delete(topicName) }

    const throttle = maxHz && maxHz > 0 ? Math.round(1000 / maxHz) : 0
    const sub = new Topic({ ros, name: topicName, messageType: msgType, throttle_rate: throttle, queue_length: 10 })
    sub.subscribe((msg: any) => dispatchRos(topicName, msg))

    topicSubs.set(topicName, sub)
    topicSpecs.set(topicName, msgType)
    return true
  }

  /** 取消订阅单个话题 */
  function unsubscribe(topic: string) {
    const topicName = canonicalRosTopic(topic)
    const sub = topicSubs.get(topicName)
    if (!sub) return
    try { sub.unsubscribe() } catch { /* */ }
    topicSubs.delete(topicName)
    topicSpecs.delete(topicName)
  }

  /** 取消所有话题订阅（断开连接时调用） */
  function unsubscribeAll() {
    topicSubs.forEach(sub => { try { sub.unsubscribe() } catch { /* */ } })
    topicSubs.clear(); topicSpecs.clear()
  }

  // ── 服务调用 ──

  /**
   * 调用 ROS 服务
   * @param service   服务名
   * @param type      服务类型 (如 'ivg_interfaces/srv/ChangeTool')
   * @param request   请求参数对象
   * @param timeoutMs 超时 (默认 60s)
   * @returns Promise<服务响应>
   */
  function callService(service: string, type: string, request?: Record<string, unknown>, timeoutMs = 60000): Promise<any> {
    const ros = rosInstance.value
    if (!ros) return Promise.reject(new Error('not_connected'))
    const serviceName = canonicalRosTopic(service)
    if (!serviceName || !type) return Promise.reject(new Error('invalid_service_spec'))

    return new Promise((resolve, reject) => {
      let settled = false; let timer: ReturnType<typeof setTimeout> | null = null

      try {
        const svc = new Service({ ros, name: serviceName, serviceType: type })
        timer = setTimeout(() => { if (!settled) { settled = true; reject(new Error('service_timeout')) } }, timeoutMs)
        svc.callService(request ?? {}, (result: any) => { if (!settled) { settled = true; if (timer) clearTimeout(timer); resolve(result ?? {}) } },
          (err: any) => { if (!settled) { settled = true; if (timer) clearTimeout(timer); reject(new Error(String(err ?? 'service_failed'))) } })
      } catch (e) {
        if (!settled) { settled = true; reject(e) }
      }
    })
  }

  // ── 消息分发 ──

  /**
   * 注册 ROS 话题消息处理器
   * @param topic 话题名 (null=接收所有话题), 函数引用用于后续取消注册
   * @returns 取消注册函数 (组件卸载时调用，避免内存泄露)
   */
  function onRosJson(topic: string | null, fn: (msg: any, t: string) => void) {
    const entry: RosHandler = { topic, fn }
    rosHandlers.push(entry)
    return () => { const i = rosHandlers.indexOf(entry); if (i >= 0) rosHandlers.splice(i, 1) }
  }

  function clearRosHandlers() { rosHandlers.length = 0 }

  /**
   * 注册控制面消息处理器 (connection/error/close 事件)
   * @returns 取消注册函数
   */
  function onControlJson(fn: ControlHandler) {
    controlHandlers.push(fn)
    return () => { const i = controlHandlers.indexOf(fn); if (i >= 0) controlHandlers.splice(i, 1) }
  }

  function clearControlHandlers() { controlHandlers.length = 0 }

  /** 分发 ROS 话题消息到匹配的处理器 */
  function dispatchRos(topic: string, msg: any) {
    const ct = canonicalRosTopic(topic)
    for (const h of rosHandlers) {
      const match = h.topic === null || h.topic === topic || canonicalRosTopic(h.topic) === ct
      if (match) { try { h.fn(msg, topic) } catch { /* 隔离单个处理器的异常 */ } }
    }
  }

  function dispatchControl(obj: { op: string; message?: string }) {
    for (const h of controlHandlers) { try { h(obj) } catch { /* */ } }
  }

  return {
    connected: readonly(connected),
    ros: readonly(rosInstance),
    connect, disconnect, isConnected,
    subscribe, unsubscribe, unsubscribeAll,
    callService,
    onRosJson, clearRosHandlers,
    onControlJson, clearControlHandlers,
  }
}
