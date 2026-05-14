/**
 * useRuntime — 从 FastAPI BFF 获取运行时配置
 *
 * 替代旧版: ivg_runtime.js + runtime_provider.js
 *
 * 调用链路: 浏览器 → GET /api/v1/runtime → FastAPI 网关 → YAML 配置
 * 缓存策略: 首次 fetch 后缓存在模块级变量，后续调用直接返回缓存
 *
 * 用法:
 *   const { config, rosbridgeWsUrl } = useRuntime()
 *   const wsUrl = rosbridgeWsUrl()  // → ws://host:8090/ws/rosbridge
 */
import { useFetch } from '@vueuse/core'

/** BFF 返回的运行时配置字段 */
interface RuntimeConfig {
  rosbridge_port?: number
  rosbridge_ws_path?: string
  web_video_port?: number
  web_video_proxy_prefix?: string
  camera_stream_path?: string
  ivg_ws_control?: string
}

let cached: RuntimeConfig | null = null

export function useRuntime() {
  const config = ref<RuntimeConfig | null>(cached)
  const loading = ref(!cached)

  /** 从 BFF 加载配置（首次调用时自动触发） */
  async function load() {
    if (cached) { config.value = cached; loading.value = false; return cached }
    const { data } = await useFetch('/api/v1/runtime').json<RuntimeConfig>()
    cached = data.value ?? {}
    config.value = cached
    loading.value = false
    return cached
  }

  /**
   * 构建 rosbridge WebSocket URL
   * 优先级: 显式控制通道 → 运行时配置路径 → 默认 /ws/rosbridge
   */
  function rosbridgeWsUrl(): string {
    const rt = config.value ?? {}
    if (rt.ivg_ws_control) return rt.ivg_ws_control
    const path = rt.rosbridge_ws_path || '/ws/rosbridge'
    const proto = location.protocol === 'https:' ? 'wss:' : 'ws:'
    return `${proto}//${location.host}${path.startsWith('/') ? path : `/${path}`}`
  }

  // 首次引用时自动开始加载（但不阻塞 — 调用方自行 await）
  if (!cached) load()

  return { config: readonly(config), loading: readonly(loading), load, rosbridgeWsUrl }
}
