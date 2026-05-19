<script setup lang="ts">
/**
 * LogView — 系统日志面板
 *
 * 替代旧版: log_panel.js (295行)
 *
 * 功能:
 *   - console 拦截 (log/warn/error/info/debug → 日志流)
 *   - 全局错误捕获 (window.onerror + unhandledrejection)
 *   - Transport 钩子 (rosbridge 连接/订阅/服务调用事件 — 通过 useRos.onLog)
 *   - 过滤搜索 (文本 + 来源级别)
 *   - 暂停/恢复/清空/导出 TXT
 */
import { useRos } from '@/composables/ros/useRos'

const MAX_ENTRIES = 800; const MAX_DOM = 400

interface LogEntry { ts: string; level: string; source: string; msg: string }
const logs = ref<LogEntry[]>([])
const paused = ref(false)
const autoScroll = ref(true)
const filterText = ref('')
const levelFilters = reactive<Record<string, boolean>>({ log: true, warn: true, error: true, info: true, debug: true, transport: true, service: true, mode: true, lifecycle: true })

const SOURCE_COLORS: Record<string, string> = { console: '#94a3b8', global: '#ef4444', promise: '#ef4444', rosbridge: '#22c55e', subscribe: '#60a5fa', unsubscribe: '#94a3b8', service: '#c084fc', service_err: '#ef4444', driver_mode: '#3b82f6', robot_status: '#34d399', log_panel: '#f59e0b', lifecycle: '#eab308' }

function ts() { const d = new Date(); return `${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}:${String(d.getSeconds()).padStart(2, '0')}.${String(d.getMilliseconds()).padStart(3, '0')}` }

function addLog(level: string, source: string, msg: string) {
  if (paused.value || !levelFilters[level] && !levelFilters[source]) return
  logs.value.push({ ts: ts(), level, source, msg })
  if (logs.value.length > MAX_ENTRIES) logs.value = logs.value.slice(-MAX_ENTRIES)
}

const container = ref<HTMLElement | null>(null)
const displayedLogs = computed(() => { let arr = logs.value.length > MAX_DOM ? logs.value.slice(-MAX_DOM) : logs.value; const q = filterText.value.trim().toLowerCase(); if (q) arr = arr.filter(e => `${e.ts} [${e.source}] ${e.msg}`.toLowerCase().includes(q)); return arr })

watch(displayedLogs, () => { if (autoScroll.value) nextTick(() => { const el = container.value; if (el) el.scrollTop = el.scrollHeight }) })

onMounted(() => {
  // 拦截 console
  (['log', 'warn', 'error', 'info', 'debug'] as const).forEach(level => {
    const orig = (console as any)[level]
    ;(console as any)[level] = (...args: any[]) => {
      addLog(level, 'console', args.map((a: any) => a instanceof Error ? a.message : typeof a === 'object' ? JSON.stringify(a) : String(a)).join(' '))
      orig.apply(console, args)
    }
  })
  window.addEventListener('error', e => addLog('error', 'global', e.message || 'Unknown error'))
  window.addEventListener('unhandledrejection', e => addLog('error', 'promise', (e.reason?.message) || String(e.reason)))

  // 页面生命周期事件（与旧版 log_panel.js:205-216 对齐）
  document.addEventListener('visibilitychange', () => {
    addLog('lifecycle', 'lifecycle', document.hidden ? '页面隐藏 (后台)' : '页面可见 (前台)')
  })
  window.addEventListener('beforeunload', () => addLog('lifecycle', 'lifecycle', '页面即将关闭'))
  window.addEventListener('pagehide', () => addLog('lifecycle', 'lifecycle', '页面已隐藏'))

  addLog('info', 'log_panel', '日志面板已就绪 — 等待事件…')
  addLog('lifecycle', 'lifecycle', '页面加载完成')

  // Transport 钩子 (通过 useRos.onLog + onControlJson)
  const { onLog, onControlJson } = useRos()
  onLog(event => {
    if (event.type === 'subscribe') addLog('transport', 'subscribe', `✓ ${event.detail || ''}`)
    if (event.type === 'unsubscribe') addLog('transport', 'unsubscribe', event.detail || '')
    if (event.type === 'service_result') addLog('transport', 'service', event.detail || '')
    if (event.type === 'service_error') addLog('error', 'service_err', event.detail || '')
  })
  onControlJson((ctrl: any) => {
    if (ctrl?.op === 'connection') addLog('transport', 'rosbridge', 'WebSocket 已连接')
    if (ctrl?.op === 'close') addLog('transport', 'rosbridge', '连接关闭')
    if (ctrl?.op === 'error') addLog('error', 'rosbridge', '错误: ' + (ctrl.message || ''))
  })
})

function exportLogs() { const text = logs.value.map(e => `${e.ts} [${e.source}] ${e.level.toUpperCase()}  ${e.msg}`).join('\n'); const b = new Blob([text], { type: 'text/plain' }); const a = document.createElement('a'); a.href = URL.createObjectURL(b); a.download = `ivg_log_${new Date().toISOString().slice(0, 10)}.txt`; a.click() }
</script>

<template>
  <div class="max-w-6xl mx-auto px-4 py-6">
    <div class="flex items-center justify-between mb-4"><h1 class="text-xl font-bold text-slate-900">系统日志</h1><span class="text-xs text-slate-400">{{ logs.length }} 条</span></div>
    <!-- 工具栏 -->
    <div class="flex flex-wrap items-center gap-2 mb-3">
      <input v-model="filterText" placeholder="搜索…" class="text-sm border border-slate-300 rounded px-2 py-1 w-48" />
      <el-button size="small" @click="paused = !paused">{{ paused ? '▶ 恢复' : '⏸ 暂停' }}</el-button>
      <el-button size="small" @click="autoScroll = !autoScroll">{{ autoScroll ? '自动滚动:开' : '自动滚动:关' }}</el-button>
      <el-button size="small" @click="logs = []">清空</el-button>
      <el-button size="small" @click="exportLogs">导出 TXT</el-button>
      <div class="flex gap-1 ml-2"><el-button v-for="key in Object.keys(levelFilters)" :key="key" size="small" :type="levelFilters[key] ? 'primary' : 'default'" plain @click="levelFilters[key] = !levelFilters[key]">{{ key }}</el-button></div>
    </div>
    <!-- 日志列表 -->
    <div ref="container" class="bg-slate-900 text-xs font-mono rounded-lg p-3 h-[70vh] overflow-auto">
      <div v-if="displayedLogs.length === 0" class="text-slate-500 text-center mt-8">等待事件…</div>
      <div v-for="(e, i) in displayedLogs" :key="i" class="flex gap-2 py-0.5 hover:bg-white/5"><span class="text-slate-500 shrink-0">{{ e.ts }}</span><span class="shrink-0" :style="{ color: SOURCE_COLORS[e.source] || '#94a3b8' }">[{{ e.source }}]</span><span :class="e.level === 'error' ? 'text-red-400' : e.level === 'warn' ? 'text-yellow-300' : 'text-slate-300'">{{ e.msg }}</span></div>
    </div>
  </div>
</template>
