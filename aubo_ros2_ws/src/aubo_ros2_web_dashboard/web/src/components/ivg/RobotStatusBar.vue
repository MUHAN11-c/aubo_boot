<script setup lang="ts">
/**
 * RobotStatusBar — 底部机械臂状态栏
 *
 * 替代旧版: ivg_status_bar.js (202行)
 *
 * 显示 5 个状态指示灯:
 *   - 在线: 机械臂是否在通信
 *   - 使能: 电机是否使能
 *   - 运动: 是否正在运动
 *   - 规划: MoveIt 规划状态 (空闲/规划中/执行中/错误)
 *   - 模式: 驱动模式 (真实/仿真)
 *
 * 数据来源: /aubo_driver/robot_status + /aubo/mode
 *
 * 外观: 固定在页面底部，半透明背景，使用 Element Plus el-tag
 */
import { useRos } from '@/composables/useRos'
import { useDashboardSettings } from '@/composables/useDashboardSettings'
import { canonicalRosTopic } from '@/lib/utils'
import { ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE, MODE_TOPIC, MODE_TYPE } from '@/constants/ros'

const { isConnected, subscribe, onRosJson, onControlJson } = useRos()
const settings = useDashboardSettings()
const robotStatusTopic = computed(() => settings.rosName('topic-robot', ROBOT_STATUS_TOPIC))

// ═══════════════════════ 状态数据 ═══════════════════════

const isOnline = ref(false)
const enable = ref(false)
const inMotion = ref(false)
const planningStatus = ref('')
const driverMode = ref('')

// ═══════════════════════ 派生显示 ═══════════════════════

const planningText = computed(() => {
  const map: Record<string, string> = { idle: '空闲', planning: '规划中', executing: '执行中', error: '错误' }
  return map[planningStatus.value] ?? (planningStatus.value || '—')
})

const planningType = computed(() => {
  if (planningStatus.value === 'error') return 'danger' as const
  if (planningStatus.value === 'executing' || planningStatus.value === 'planning') return 'warning' as const
  return 'success' as const
})

const modeText = computed(() => {
  if (driverMode.value === 'real') return '真实'
  if (driverMode.value === 'simulation') return '仿真'
  return driverMode.value || '—'
})

const modeType = computed(() => {
  if (driverMode.value === 'real') return 'success' as const
  if (driverMode.value === 'simulation') return 'warning' as const
  return 'info' as const
})

// ═══════════════════════ 话题处理 ═══════════════════════

onRosJson(null, (msg: any, topic: string) => {
  if (canonicalRosTopic(topic) !== canonicalRosTopic(robotStatusTopic.value)) return
  isOnline.value = !!msg.is_online
  enable.value = !!msg.enable
  inMotion.value = !!msg.in_motion
  planningStatus.value = msg.planning_status ?? ''
})

onRosJson(MODE_TOPIC, (msg: any) => {
  driverMode.value = typeof msg.data === 'string' ? msg.data : ''
})

// ═══════════════════════ 订阅管理 ═══════════════════════

function setupSubs() {
  if (!isConnected()) return
  subscribe(robotStatusTopic.value, settings.topicType('topic-robot', ROBOT_STATUS_TYPE), 10)
  subscribe(MODE_TOPIC, MODE_TYPE, 1)
}

onControlJson((c) => { if (c.op === 'connection') setupSubs() })
watch(isConnected, v => { if (v) setupSubs() })
watch(() => settings.version.value, () => { if (isConnected()) setupSubs() })
</script>

<template>
  <footer
    class="fixed bottom-0 left-0 right-0 bg-white/95 backdrop-blur border-t border-slate-200 px-4 h-[calc(2rem+env(safe-area-inset-bottom,0px))] pb-[env(safe-area-inset-bottom,0px)] flex items-center gap-3 z-50 overflow-x-auto"
    aria-label="机械臂状态"
  >
    <span class="text-xs text-slate-400 font-medium shrink-0">IVG</span>

    <div class="flex items-center gap-3">
      <!-- 在线 -->
      <div class="flex items-center gap-1">
        <span class="text-xs text-slate-500">在线</span>
        <el-tag :type="isOnline ? 'success' : 'info'" size="small" round>{{ isOnline ? '在线' : '离线' }}</el-tag>
      </div>
      <!-- 使能 -->
      <div class="flex items-center gap-1">
        <span class="text-xs text-slate-500">使能</span>
        <el-tag :type="enable ? 'success' : 'info'" size="small" round>{{ enable ? '已使能' : '未使能' }}</el-tag>
      </div>
      <!-- 运动 -->
      <div class="flex items-center gap-1">
        <span class="text-xs text-slate-500">运动</span>
        <el-tag :type="inMotion ? 'warning' : 'info'" size="small" round>{{ inMotion ? '运动中' : '静止' }}</el-tag>
      </div>
      <!-- 规划 -->
      <div class="flex items-center gap-1">
        <span class="text-xs text-slate-500">规划</span>
        <el-tag :type="planningType" size="small" round>{{ planningText }}</el-tag>
      </div>
      <!-- 模式 -->
      <div class="flex items-center gap-1">
        <span class="text-xs text-slate-500">模式</span>
        <el-tag :type="modeType" size="small" round>{{ modeText }}</el-tag>
      </div>
    </div>
  </footer>
</template>
