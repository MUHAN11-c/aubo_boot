/**
 * useMJPEGStream — 相机 MJPEG 视频流 URL 构建
 *
 * 替代旧版: ivg_web_video.js (124行)
 *
 * 自动根据运行时配置构建流 URL:
 *   - 优先使用网关代理 (/api/ivg/proxy/web-video/stream)
 *   - 回退到直连 web_video_server
 *
 * 用法:
 *   const { cameraStreamUrl } = useMJPEGStream(ref('/camera/color/image_raw'))
 *   <img :src="cameraStreamUrl()" />
 */
import { useRuntime } from './useRuntime'
import { encodeTopicQueryValue } from '@/lib/utils'

export function useMJPEGStream(topic: string | Ref<string>) {
  const { config } = useRuntime()

  /** 解析 Ref<string> — 保证 topic 无论是 ref 还是普通字符串都能正常工作 */
  const topicVal = computed(() => unref(topic))

  /**
   * 构建 MJPEG 流 URL
   * @param streamId  流 ID (用于区分同一话题的多个流，可选)
   * @param quality   JPEG 质量 1-100，默认 85
   */
  function cameraStreamUrl(streamId?: string, quality = 85): string {
    const rt = config.value ?? {}
    // streamId 未指定时从话题名自动生成唯一标识
    const sid = streamId || topicVal.value.replace(/\//g, '_').replace(/^_/, '') || 'cam'

    // 优先: 显式的相机流路径
    if (rt.camera_stream_path) {
      const base = `${location.origin}${rt.camera_stream_path}`
      const q = new URLSearchParams({ topic: topicVal.value, stream_id: sid })
      q.set('quality', String(quality))
      return `${base}?${q}`
    }

    // 默认: 通过网关代理
    const proxy = rt.web_video_proxy_prefix || '/api/ivg/proxy/web-video'
    const pre = proxy.endsWith('/') ? proxy.slice(0, -1) : proxy
    return `${location.origin}${pre}/stream?topic=${encodeTopicQueryValue(topicVal.value)}&type=mjpeg&client_id=${encodeURIComponent(sid)}&quality=${quality}`
  }

  return { cameraStreamUrl }
}
