/**
 * useRos — ROS 连接管理的全局单例 composable
 *
 * 替代旧版: ivg_transport.js (298行) + ivg_runtime.js 中的 rosReconnect
 *
 * 核心职责:
 *   1. Ros WebSocket 连接生命周期 (connect/disconnect/自动重连)
 *   2. 话题订阅/取消订阅 (subscribe/unsubscribe)
 *   3. 服务调用 (callService)
 *   4. 消息分发 (onRosJson / onControlJson)
 *   5. 自动重连 (指数退避 12 次，最多 30s 间隔)
 *   6. 页面后台暂停/恢复 (visibilitychange/pagehide)
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
type LogHandler = (event: { type: 'subscribe' | 'unsubscribe' | 'service_call' | 'service_result' | 'service_error'; detail?: string }) => void

// ═══════════════════════ 模块级单例状态 ═══════════════════════

// Ros 对象不能放入 Vue 响应式系统 — isConnected getter 读 JS 私有字段 #isConnected，
// Vue Proxy 拦截 Reflect.get 会抛 "Cannot read from private field"
let rosInstance: Ros | null = null
const connected = ref(false)
const topicSubs = new Map<string, Topic>()
const topicSpecs = new Map<string, string>()
const rosHandlers: RosHandler[] = []
const controlHandlers: ControlHandler[] = []
const logHandlers: LogHandler[] = []
let connectPromise: Promise<void> | null = null
let reconnectGen = 0
let reconnectAttempts = 0
let reconnectTimer: ReturnType<typeof setTimeout> | null = null
const RECONNECT_MAX = 12
let pageRealtimePaused = false

// ═══════════════════════ 页面生命周期（模块级） ═══════════════════════

let _reconnectTrigger: (() => void) | null = null

function setReconnectTrigger(fn: () => void): void { _reconnectTrigger = fn }

function modulePageShouldPause(): boolean {
  return typeof document !== 'undefined' && document.visibilityState === 'hidden'
}

function moduleClearReconnectTimer(): void {
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null }
}

if (typeof document !== 'undefined') {
  document.addEventListener('visibilitychange', () => {
    if (modulePageShouldPause()) {
      if (!pageRealtimePaused) {
        pageRealtimePaused = true
        moduleClearReconnectTimer()
        reconnectAttempts = 0
        try { rosInstance?.close() } catch { /* */ }
        for (const h of controlHandlers) { try { h({ op: 'close', message: '页面后台已暂停' }) } catch { /* */ } }
      }
    } else {
      if (pageRealtimePaused) {
        pageRealtimePaused = false
        reconnectAttempts = 0
        _reconnectTrigger?.()
      }
    }
  })
}
if (typeof window !== 'undefined') {
  window.addEventListener('pagehide', () => {
    pageRealtimePaused = true
    moduleClearReconnectTimer()
    reconnectAttempts = 0
    try { rosInstance?.close() } catch { /* */ }
  })
  window.addEventListener('online', () => {
    if (!connected.value && !pageRealtimePaused) {
      reconnectAttempts = 0
      _reconnectTrigger?.()
    }
  })
}

// ═══════════════════════ 导出函数 ═══════════════════════

export function useRos() {
  const { rosbridgeWsUrl, load: loadRuntime } = useRuntime()

  /** 是否已连接到 rosbridge */
  function isConnected(): boolean { return connected.value }

  // ── 连接管理 ──

  /** 建立 rosbridge WebSocket 连接（自动去重，同一时间只有一个连接尝试） */
  async function connect(): Promise<void> {
    // 首次调用时注册为页面恢复/在线的重连触发器
    setReconnectTrigger(() => connect())
    if (connectPromise) return connectPromise
    if (modulePageShouldPause()) {
      pageRealtimePaused = true  // 等 visibilitychange 触发 resume → connect
      return
    }
    pageRealtimePaused = false
    clearReconnectTimer()
    const myGen = ++reconnectGen

    connectPromise = (async () => {
      try {
        await loadRuntime()
        const url = rosbridgeWsUrl()
        disconnect(false)

        return await new Promise<void>((resolve, reject) => {
          let settled = false
          try {
            const ros = new Ros({ url: url as string })
            rosInstance = ros

            ros.on('connection', () => {
              if (settled) return; settled = true
              connected.value = true
              reconnectAttempts = 0
              clearReconnectTimer()
              dispatchControl({ op: 'connection' })
              resolve()
            })
            ros.on('error', (err: any) => {
              if (settled) return; settled = true
              connected.value = false
              dispatchControl({ op: 'error', message: String(err ?? 'ros_error') })
              if (myGen === reconnectGen) scheduleReconnect()
              reject(new Error('ros_error'))
            })
            ros.on('close', () => {
              connected.value = false
              dispatchControl({ op: 'close', message: 'ros_closed' })
              if (!pageRealtimePaused && myGen === reconnectGen) scheduleReconnect()
            })
          } catch (e) {
            settled = true; connected.value = false
            if (myGen === reconnectGen) scheduleReconnect()
            reject(e)
          }
        })
      } finally {
        connectPromise = null
      }
    })()

    return connectPromise
  }

  function scheduleReconnect(): void {
    if (reconnectAttempts >= RECONNECT_MAX) {
      dispatchControl({ op: 'error', message: `重连已达上限 (${RECONNECT_MAX}次)，请刷新页面` })
      return
    }
    if (pageRealtimePaused) return
    const delay = Math.min(30000, 1000 * Math.pow(2, reconnectAttempts))
    reconnectAttempts++
    const attempt = reconnectAttempts
    const remain = Math.round(delay / 1000)
    dispatchControl({ op: 'close', message: `已断开：${remain}s 后自动重连（${attempt}/${RECONNECT_MAX}）` })
    reconnectTimer = setTimeout(() => {
      reconnectTimer = null
      connect().catch(() => { /* */ })
    }, delay)
  }

  function clearReconnectTimer(): void {
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null }
  }

  /** 断开连接并清理所有订阅 */
  function disconnect(clearReconnect = true): void {
    unsubscribeAll()
    connectPromise = null
    if (clearReconnect) { clearReconnectTimer(); reconnectAttempts = 0 }
    try { rosInstance?.close() } catch { /* */ }
    rosInstance = null
    connected.value = false
  }

  // ── 话题订阅 ──

  function subscribe(topic: string, msgType: string, maxHz?: number): boolean {
    const ros = rosInstance
    if (!ros) return false

    const topicName = canonicalRosTopic(topic)
    if (!topicName || !msgType) return false
    if (topicSpecs.get(topicName) === msgType) return true

    const existing = topicSubs.get(topicName)
    if (existing) { try { existing.unsubscribe() } catch { /* */ }; topicSubs.delete(topicName) }

    const throttle = maxHz && maxHz > 0 ? Math.round(1000 / maxHz) : 0
    const sub = new Topic({ ros, name: topicName, messageType: msgType, throttle_rate: throttle, queue_length: 10 })
    sub.subscribe((msg: any) => dispatchRos(topicName, msg))

    topicSubs.set(topicName, sub)
    topicSpecs.set(topicName, msgType)
    dispatchLog({ type: 'subscribe', detail: topicName })
    return true
  }

  function unsubscribe(topic: string): void {
    const topicName = canonicalRosTopic(topic)
    const sub = topicSubs.get(topicName)
    if (!sub) return
    try { sub.unsubscribe() } catch { /* */ }
    topicSubs.delete(topicName)
    topicSpecs.delete(topicName)
    dispatchLog({ type: 'unsubscribe', detail: topicName })
  }

  function unsubscribeAll(): void {
    if (connected.value) {
      topicSubs.forEach(sub => { try { sub.unsubscribe() } catch { /* */ } })
    }
    topicSubs.clear(); topicSpecs.clear()
  }

  // ── 服务调用 ──

  function callService(service: string, type: string, request?: Record<string, unknown>, timeoutMs = 60000): Promise<any> {
    const ros = rosInstance
    if (!ros) return Promise.reject(new Error('not_connected'))
    const serviceName = canonicalRosTopic(service)
    if (!serviceName || !type) return Promise.reject(new Error('invalid_service_spec'))

    const start = performance.now()
    dispatchLog({ type: 'service_call', detail: serviceName })

    return new Promise((resolve, reject) => {
      let settled = false; let timer: ReturnType<typeof setTimeout> | null = null
      try {
        const svc = new Service({ ros, name: serviceName, serviceType: type })
        timer = setTimeout(() => { if (!settled) { settled = true; dispatchLog({ type: 'service_error', detail: `${serviceName} timeout` }); reject(new Error('service_timeout')) } }, timeoutMs)
        svc.callService(request ?? {}, (result: any) => { if (!settled) { settled = true; if (timer) clearTimeout(timer); const ok = result?.success !== false; dispatchLog({ type: 'service_result', detail: `${ok ? '✓' : '✗'} ${serviceName} (${(performance.now() - start).toFixed(0)}ms)` }); resolve(result ?? {}) } },
          (err: any) => { if (!settled) { settled = true; if (timer) clearTimeout(timer); dispatchLog({ type: 'service_error', detail: `✗ ${serviceName} ${String(err ?? 'service_failed')}` }); reject(new Error(String(err ?? 'service_failed'))) } })
      } catch (e) { if (!settled) { settled = true; reject(e) } }
    })
  }

  // ── 消息分发 ──

  function onRosJson(topic: string | null, fn: (msg: any, t: string) => void) {
    const entry: RosHandler = { topic, fn }
    rosHandlers.push(entry)
    return () => { const i = rosHandlers.indexOf(entry); if (i >= 0) rosHandlers.splice(i, 1) }
  }

  function clearRosHandlers() { rosHandlers.length = 0 }

  function onControlJson(fn: ControlHandler) {
    controlHandlers.push(fn)
    return () => { const i = controlHandlers.indexOf(fn); if (i >= 0) controlHandlers.splice(i, 1) }
  }

  function clearControlHandlers() { controlHandlers.length = 0 }

  function dispatchRos(topic: string, msg: any) {
    const ct = canonicalRosTopic(topic)
    for (const h of rosHandlers) {
      const match = h.topic === null || h.topic === topic || canonicalRosTopic(h.topic) === ct
      if (match) { try { h.fn(msg, topic) } catch { /* */ } }
    }
  }

  function dispatchControl(obj: { op: string; message?: string }) {
    for (const h of controlHandlers) { try { h(obj) } catch { /* */ } }
  }

  function dispatchLog(event: { type: 'subscribe' | 'unsubscribe' | 'service_call' | 'service_result' | 'service_error'; detail?: string }) {
    for (const h of logHandlers) { try { h(event) } catch { /* */ } }
  }

  return {
    connected: readonly(connected),
    connect, disconnect, isConnected,
    subscribe, unsubscribe, unsubscribeAll,
    callService,
    onRosJson, clearRosHandlers,
    onControlJson, clearControlHandlers,
    onLog: (fn: LogHandler) => { logHandlers.push(fn); return () => { const i = logHandlers.indexOf(fn); if (i >= 0) logHandlers.splice(i, 1) } },
  }
}
