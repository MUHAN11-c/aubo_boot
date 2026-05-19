<script setup lang="ts">
/**
 * CoffeeLatteView — 咖啡拉花面板 (重构版)
 *
 * 使用子组件 + useLatteSession 统一状态管理喵~
 *
 * 两条独立调用路径:
 *   【非ROS】预览 → BFF HTTP (useLattePreview)
 *   【ROS】  执行 → rosbridge Service (useLatteExecution)
 */
import { useRos } from '@/composables/ros/useRos'
import { useRosService } from '@/composables/ros/useRosService'
import { useLatteExecution } from '@/composables/ros/useLatteExecution'
import { useLattePreview } from '@/composables/api/useLattePreview'
import type { LattePreviewResponse, LatteStartPose } from '@/composables/api/useLattePreview'
import { useJointChart } from '@/composables/viz/useJointChart'
import { useDashboardSettings } from '@/composables/settings/useDashboardSettings'
import { useLatteSession } from '@/composables/latte/useLatteSession'
import { canonicalRosTopic } from '@/lib/utils'
import Robot3dViewer from '@/components/ivg/Robot3dViewer.vue'
import LattePatternSelector from '@/components/latte/LattePatternSelector.vue'
import LatteCupConfigPanel from '@/components/latte/LatteCupConfigPanel.vue'
import LattePourConfigPanel from '@/components/latte/LattePourConfigPanel.vue'
import LatteTransformControls from '@/components/latte/LatteTransformControls.vue'
import LatteControls from '@/components/latte/LatteControls.vue'
import {
  LATTE_DI_STATUS_TOPIC, LATTE_DI_STATUS_TYPE,
  ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE,
  JOINT_STATES_TOPIC, JOINT_STATES_TYPE,
} from '@/constants/ros'

// ═══════════════════════ Composables ═══════════════════════
const { preview: previewLatte, loading: latteLoading } = useLattePreview()
const { execute: executeLatte, executing: latteExecuting } = useLatteExecution()
const { setLatteDo } = useRosService()
const { isConnected, connect, subscribe, onRosJson, onControlJson } = useRos()
const settings = useDashboardSettings()
const sess = useLatteSession()

// ═══════════════════════ 话题名 ═══════════════════════
const latteDiTopic = computed(() => settings.rosName('latte-di-topic', LATTE_DI_STATUS_TOPIC))
const robotStatusTopic = computed(() => settings.rosName('topic-robot', ROBOT_STATUS_TOPIC))
const jointStatesTopic = computed(() => settings.rosName('topic-joints', JOINT_STATES_TOPIC))
const toolStatusTopic = computed(() => settings.rosName('topic-tool-status', '/tool_changer_status'))
const urdfParam = computed(() => settings.raw('urdf-param', '/robot_state_publisher:robot_description'))
const fixedFrame = computed(() => settings.raw('tf-fixed-frame', 'base_link'))
function sameTopic(topic: string, expected: string): boolean {
  return canonicalRosTopic(topic) === canonicalRosTopic(expected)
}

// ═══════════════════════ 连接状态 ═══════════════════════
const connStatus = ref('正在连接…')
const connOk = ref(false)
onControlJson((c) => {
  if (c.op === 'connection') { connStatus.value = '已连接'; connOk.value = true }
  if (c.op === 'close') { connStatus.value = '已断开，重连中…'; connOk.value = false }
  if (c.op === 'error') { connStatus.value = '通信错误'; connOk.value = false }
})

// ═══════════════════════ DI 信号灯 ═══════════════════════
const di2 = ref(false); const di3 = ref(false); const di4 = ref(false)
function parseDi(msg: any) {
  let text = typeof msg === 'string' ? msg : msg?.data ? String(msg.data) : ''
  if (!text) return
  try { const obj = JSON.parse(text); if (obj && typeof obj === 'object') { di2.value = !!obj.di2; di3.value = !!obj.di3; di4.value = !!obj.di4; return } } catch { /* */ }
  const parts = text.split(',').map((s: string) => s.trim())
  if (parts.length >= 3) { di2.value = parts[0] === '1'; di3.value = parts[1] === '1'; di4.value = parts[2] === '1' }
}

// ═══════════════════════ DO 开关 ═══════════════════════
const do2On = ref(false); const do4On = ref(false)

// ═══════════════════════ 末端位姿 ═══════════════════════
const robotPose = reactive({
  x: '—', y: '—', z: '—', qx: '—', qy: '—', qz: '—', qw: '—',
  roll: '—', pitch: '—', yaw: '—',
})
const robotPoseReady = computed(() => robotPose.x !== '—' && robotPose.qx !== '—')

function getStartPose(): LatteStartPose | undefined {
  if (!robotPoseReady.value) return undefined
  const x = parseFloat(robotPose.x); const y = parseFloat(robotPose.y); const z = parseFloat(robotPose.z)
  if (!isFinite(x) || !isFinite(y) || !isFinite(z)) return undefined
  const qx = parseFloat(robotPose.qx); const qy = parseFloat(robotPose.qy)
  const qz = parseFloat(robotPose.qz); const qw = parseFloat(robotPose.qw)
  if (!isFinite(qx) || !isFinite(qy) || !isFinite(qz) || !isFinite(qw)) return undefined
  return { x, y, z, qx, qy, qz, qw }
}

// ═══════════════════════ 关节曲线 ═══════════════════════
const { canvasRef: chartCanvasRef, legendRef: chartLegendRef, pushSample: pushJointSample, reset: resetJointChart, observeResize: observeJointChart } = useJointChart()

// ═══════════════════════ 监控区折叠 ═══════════════════════
const MONITORING_KEY = 'ivg_vision_monitoring_collapsed'
const monitoringCollapsed = ref(false)
try { monitoringCollapsed.value = localStorage.getItem(MONITORING_KEY) === '1' } catch { /* */ }
function toggleMonitoring() {
  monitoringCollapsed.value = !monitoringCollapsed.value
  try { localStorage.setItem(MONITORING_KEY, monitoringCollapsed.value ? '1' : '0') } catch { /* */ }
  if (!monitoringCollapsed.value) nextTick(() => observeJointChart())
}

// ═══════════════════════ 消息处理 ═══════════════════════
onRosJson(null, (msg: any, topic: string) => {
  if (sameTopic(topic, latteDiTopic.value)) parseDi(msg)
})
onRosJson(null, (msg: any, topic: string) => {
  if (!sameTopic(topic, robotStatusTopic.value)) return
  if (!msg) return
  const p = msg.cartesian_position_xyz || {}
  const o = msg.cartesian_position?.orientation || {}
  const r = msg.cartesian_rpy || {}
  robotPose.x = p.x?.toFixed(4) ?? '—'; robotPose.y = p.y?.toFixed(4) ?? '—'; robotPose.z = p.z?.toFixed(4) ?? '—'
  robotPose.qx = o.x?.toFixed(4) ?? '—'; robotPose.qy = o.y?.toFixed(4) ?? '—'; robotPose.qz = o.z?.toFixed(4) ?? '—'; robotPose.qw = o.w?.toFixed(4) ?? '—'
  const rr = Number(r.x), rp = Number(r.y), ry = Number(r.z)
  if (isFinite(rr) && isFinite(rp) && isFinite(ry)) {
    robotPose.roll = (rr * 180 / Math.PI).toFixed(1) + '°'
    robotPose.pitch = (rp * 180 / Math.PI).toFixed(1) + '°'
    robotPose.yaw = (ry * 180 / Math.PI).toFixed(1) + '°'
  }
})
onRosJson(null, (msg: any, topic: string) => {
  if (!sameTopic(topic, jointStatesTopic.value)) return
  if (!msg) return
  pushJointSample(msg.name ?? [], msg.position ?? [])
})

// ═══════════════════════ 订阅 ═══════════════════════
function setupSubs() {
  if (!isConnected()) return
  subscribe(latteDiTopic.value, settings.topicType('latte-di-topic', LATTE_DI_STATUS_TYPE), 10)
  subscribe(robotStatusTopic.value, settings.topicType('topic-robot', ROBOT_STATUS_TYPE), 10)
  subscribe(jointStatesTopic.value, settings.topicType('topic-joints', JOINT_STATES_TYPE), 30)
}
onControlJson((c) => { if (c.op === 'connection') setupSubs() })
watch(isConnected, v => { if (v) setupSubs() })
if (isConnected()) setupSubs()
onMounted(() => {
  observeJointChart()
  settings.loadSettings().then(() => { if (isConnected()) setupSubs() }).catch(() => {})
  if (!isConnected()) connect().catch(() => {})
})
watch(() => settings.version.value, () => { if (isConnected()) setupSubs() })

// ═══════════════════════ 预览 ═══════════════════════
async function runLattePreview() {
  sess.message.value = ''
  sess.result.value = null
  sess.saveRpy()
  try {
    const req = sess.buildRequest('preview', getStartPose())
    req.mode = undefined  // BFF preview doesn't use mode field
    const data = await previewLatte(req as any)
    sess.result.value = data
    sess.message.value = data.message || 'OK'
  } catch (e: any) {
    sess.message.value = `预览失败: ${String(e?.message ?? e)}`
  }
}

// ═══════════════════════ 执行 ═══════════════════════
async function runLatteAction() {
  sess.message.value = '执行中…'
  try {
    const req = sess.buildRequest('action', getStartPose())
    const result = await executeLatte(req as any)
    sess.message.value = result.message || '执行完成'
  } catch (e: any) {
    sess.message.value = `执行失败: ${String(e?.message ?? e)}`
  }
}

// ═══════════════════════ 生成模式判断 ═══════════════════════
const isGenerated = computed(() => sess.patternType.value !== '')
</script>

<template>
  <div class="ivg-run-page w-full max-w-none px-2 sm:px-3 py-2 overflow-x-hidden">
    <!-- ═══ 顶部 ═══ -->
    <div class="ivg-topbar flex items-center justify-between mb-2 bg-white rounded-lg border border-slate-200 px-3 py-2">
      <div class="flex items-center gap-3">
        <h1 class="text-lg font-bold text-slate-900">机械臂咖啡拉花</h1>
        <el-tag size="small" type="warning" round>演示</el-tag>
      </div>
      <div class="flex items-center gap-2">
        <a href="/settings" target="_blank" rel="noopener" class="text-xs text-blue-500 hover:text-blue-600 no-underline">话题与服务设置</a>
        <span class="text-xs" :class="connOk ? 'text-green-600' : connStatus === '正在连接…' ? 'text-slate-400' : 'text-red-500'">{{ connStatus }}</span>
      </div>
    </div>

    <!-- ═══ 主内容 ═══ -->
    <div class="ivg-main-grid grid grid-cols-1 min-[921px]:grid-cols-[minmax(0,1fr)_minmax(15rem,22vw)] gap-2 min-h-0">
      <!-- 左栏: 3D -->
      <div class="ivg-visual-grid min-w-0 grid grid-cols-1 min-[921px]:grid-cols-2 gap-2 content-start">
        <section class="bg-white rounded-lg border border-slate-200 overflow-hidden" aria-label="机械臂 URDF 模型">
          <h2 class="text-xs font-bold text-slate-500 uppercase px-3 pt-3 pb-1">机械臂模型</h2>
          <Robot3dViewer :urdf-param="urdfParam" :fixed-frame="fixedFrame" :tool-status-topic="toolStatusTopic"
                         :trajectory-overlay="sess.result.value" class="w-full" />
        </section>
        <section class="bg-white rounded-lg border border-slate-200 p-4" aria-label="咖啡流程示意">
          <h2 class="text-xs font-bold text-slate-500 uppercase mb-3">咖啡流程示意</h2>
          <div class="flex items-center gap-3 text-sm">
            <span class="flex items-center gap-1.5"><span class="w-6 h-6 rounded-full bg-blue-100 text-blue-700 flex items-center justify-center text-xs font-bold">1</span>出杯/定位</span>
            <span class="text-slate-300">→</span>
            <span class="flex items-center gap-1.5"><span class="w-6 h-6 rounded-full bg-blue-100 text-blue-700 flex items-center justify-center text-xs font-bold">2</span>萃取与奶泡</span>
            <span class="text-slate-300">→</span>
            <span class="flex items-center gap-1.5"><span class="w-6 h-6 rounded-full bg-blue-100 text-blue-700 flex items-center justify-center text-xs font-bold">3</span>拉花执行</span>
          </div>
        </section>
      </div>

      <!-- 右栏: 控制面板 -->
      <div class="min-w-0 space-y-2">
        <!-- IO 开关 -->
        <div class="bg-white rounded-lg border border-slate-200 p-4">
          <h2 class="text-sm font-bold text-slate-700 mb-3">拉花工序与 IO</h2>
          <div class="space-y-3">
            <div class="flex items-center justify-between">
              <div><span class="text-sm font-medium text-slate-700">咖啡开关</span><el-tag size="small" class="ml-2">DO4</el-tag></div>
              <el-switch v-model="do4On" />
            </div>
            <div class="flex items-center justify-between">
              <div><span class="text-sm font-medium text-slate-700">打花开关</span><el-tag size="small" class="ml-2">DO2</el-tag></div>
              <el-switch v-model="do2On" />
            </div>
          </div>
        </div>

        <!-- 拉花轨迹调试面板 -->
        <div class="bg-white rounded-lg border border-slate-200 p-4">
          <h2 class="text-sm font-bold text-slate-700 mb-3">拉花轨迹调试</h2>

          <!-- 轨迹源选择 -->
          <LattePatternSelector
            :pattern-type="sess.patternType.value" :episode-idx="sess.episodeIdx.value"
            :tulip-layers="sess.tulipLayers.value"
            @update:pattern-type="sess.patternType.value = $event"
            @update:episode-idx="sess.episodeIdx.value = $event"
            @update:tulip-layers="sess.tulipLayers.value = $event"
          />

          <!-- 杯子参数 (仅生成模式) -->
          <LatteCupConfigPanel
            :visible="isGenerated"
            :center-x="sess.cupCenterX.value" :center-y="sess.cupCenterY.value"
            :surface-z="sess.cupSurfaceZ.value" :radius="sess.cupRadius.value"
            @update:center-x="sess.cupCenterX.value = $event"
            @update:center-y="sess.cupCenterY.value = $event"
            @update:surface-z="sess.cupSurfaceZ.value = $event"
            @update:radius="sess.cupRadius.value = $event"
          />

          <!-- 倾倒参数 (仅生成模式, 折叠) -->
          <LattePourConfigPanel
            :visible="isGenerated" :advanced-open="sess.pourAdvancedOpen.value"
            :mix-height="sess.pourMixHeight.value"
            :draw-height="sess.pourDrawHeight.value"
            :finish-height="sess.pourFinishHeight.value"
            :wiggle-amp="sess.pourWiggleAmp.value"
            :wiggle-freq="sess.pourWiggleFreq.value"
            :max-vel="sess.pourMaxVel.value"
            :max-acc="sess.pourMaxAcc.value"
            :max-jerk="sess.pourMaxJerk.value"
            :anti-slosh="sess.pourAntiSlosh.value"
            @toggle="sess.pourAdvancedOpen.value = !sess.pourAdvancedOpen.value"
            @update:mix-height="sess.pourMixHeight.value = $event"
            @update:draw-height="sess.pourDrawHeight.value = $event"
            @update:finish-height="sess.pourFinishHeight.value = $event"
            @update:wiggle-amp="sess.pourWiggleAmp.value = $event"
            @update:wiggle-freq="sess.pourWiggleFreq.value = $event"
            @update:max-vel="sess.pourMaxVel.value = $event"
            @update:max-acc="sess.pourMaxAcc.value = $event"
            @update:max-jerk="sess.pourMaxJerk.value = $event"
            @update:anti-slosh="sess.pourAntiSlosh.value = $event"
          />

          <!-- RPY + Speed -->
          <LatteTransformControls
            :roll="sess.rpyRoll.value" :pitch="sess.rpyPitch.value" :yaw="sess.rpyYaw.value"
            :speed-scale="sess.speedScale.value" :tool-id="sess.toolId.value"
            @update:roll="sess.rpyRoll.value = $event"
            @update:pitch="sess.rpyPitch.value = $event"
            @update:yaw="sess.rpyYaw.value = $event"
            @update:speed-scale="sess.speedScale.value = $event"
            @save-rpy="sess.saveRpy()"
          />

          <!-- 预览/执行按钮 + 结果 -->
          <LatteControls
            :preview-loading="latteLoading" :exec-executing="latteExecuting"
            :ros-connected="connOk"
            :message="sess.message.value"
            :success="sess.result.value?.success ?? null"
            :num-frames="sess.result.value?.num_frames ?? 0"
            :path-length="sess.result.value?.path_length ?? 0"
            :tcp-waypoints="sess.result.value?.tcp_path?.length ?? 0"
            :spout-waypoints="sess.result.value?.spout_path?.length ?? 0"
            @preview="runLattePreview" @execute="runLatteAction"
          />
        </div>
      </div>
    </div>

    <!-- ═══ 底部监控 ═══ -->
    <div class="mt-2" :class="{ 'is-monitoring-collapsed': monitoringCollapsed }">
      <button
        class="w-full flex items-center justify-between bg-white rounded-lg border border-slate-200 px-4 py-2 text-sm font-medium text-slate-600 hover:bg-slate-50 transition-colors"
        :aria-expanded="!monitoringCollapsed" aria-controls="monitoring-bundle" @click="toggleMonitoring">
        <span>关节曲线与末端位姿</span>
        <span class="text-xs text-slate-400">{{ monitoringCollapsed ? '展开' : '收起' }}</span>
      </button>
      <div v-show="!monitoringCollapsed" id="monitoring-bundle" class="ivg-monitoring-grid grid grid-cols-1 md:grid-cols-2 gap-2 mt-2">
        <div class="bg-slate-900 rounded-lg border border-slate-700 p-3">
          <h2 class="text-xs font-bold text-slate-400 uppercase mb-2">关节角曲线</h2>
          <div ref="chartLegendRef" class="flex flex-wrap gap-2 mb-2" role="list" />
          <canvas ref="chartCanvasRef" width="640" height="220" class="w-full" aria-label="各关节位置随时间变化" />
        </div>
        <div class="space-y-3">
          <div class="bg-white rounded-lg border border-slate-200 p-4">
            <h2 class="text-sm font-bold text-slate-500 uppercase mb-3">机械臂末端位姿</h2>
            <div class="grid grid-cols-4 gap-2 text-xs">
              <div><span class="text-slate-400">X</span><br/><span class="font-mono">{{ robotPose.x }}</span></div>
              <div><span class="text-slate-400">Y</span><br/><span class="font-mono">{{ robotPose.y }}</span></div>
              <div><span class="text-slate-400">Z</span><br/><span class="font-mono">{{ robotPose.z }}</span></div>
              <div><span class="text-slate-400">Roll</span><br/><span class="font-mono">{{ robotPose.roll }}</span></div>
              <div><span class="text-slate-400">Pitch</span><br/><span class="font-mono">{{ robotPose.pitch }}</span></div>
              <div><span class="text-slate-400">Yaw</span><br/><span class="font-mono">{{ robotPose.yaw }}</span></div>
              <div><span class="text-slate-400">QX</span><br/><span class="font-mono">{{ robotPose.qx }}</span></div>
              <div><span class="text-slate-400">QY</span><br/><span class="font-mono">{{ robotPose.qy }}</span></div>
              <div><span class="text-slate-400">QZ</span><br/><span class="font-mono">{{ robotPose.qz }}</span></div>
              <div><span class="text-slate-400">QW</span><br/><span class="font-mono">{{ robotPose.qw }}</span></div>
            </div>
          </div>
          <div class="bg-white rounded-lg border border-slate-200 p-4">
            <h2 class="text-sm font-bold text-slate-500 uppercase mb-3">反馈信号灯（DI）</h2>
            <div class="space-y-3">
              <div class="flex items-center gap-3">
                <span class="w-3 h-3 rounded-full transition-colors" :class="di2 ? 'bg-green-500 shadow-[0_0_6px] shadow-green-500' : 'bg-slate-300'" />
                <span class="text-sm text-slate-600">咖啡反馈</span><el-tag size="small">DI2</el-tag>
              </div>
              <div class="flex items-center gap-3">
                <span class="w-3 h-3 rounded-full transition-colors" :class="di4 ? 'bg-amber-500 shadow-[0_0_6px] shadow-amber-500' : 'bg-slate-300'" />
                <span class="text-sm text-slate-600">警告反馈</span><el-tag size="small">DI4</el-tag>
              </div>
              <div class="flex items-center gap-3">
                <span class="w-3 h-3 rounded-full transition-colors" :class="di3 ? 'bg-green-500 shadow-[0_0_6px] shadow-green-500' : 'bg-slate-300'" />
                <span class="text-sm text-slate-600">打花反馈</span><el-tag size="small">DI3</el-tag>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <footer class="mt-2 bg-white rounded-lg border border-slate-200 px-3 py-1.5 flex items-center gap-2 text-xs" role="contentinfo">
      <span class="text-slate-400">连接状态</span>
      <span :class="connOk ? 'text-green-600' : connStatus === '正在连接…' ? 'text-slate-400' : 'text-red-500'">{{ connStatus }}</span>
    </footer>
  </div>
</template>
