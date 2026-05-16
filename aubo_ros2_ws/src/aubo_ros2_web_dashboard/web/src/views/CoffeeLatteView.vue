<script setup lang="ts">
/**
 * CoffeeLatteView — 咖啡拉花面板
 *
 * 替代旧版: coffee_latte_panel.html + coffee_latte_io.js (115行)
 * 复用 vision_grasp_panel.js 的订阅编排（3D模型/关节曲线/末端位姿）
 *
 * 功能:
 *   - 机械臂 URDF 3D 模型 (Robot3dViewer)
 *   - DO 开关 (咖啡 DO4 / 打花 DO2) — 通过 /set_latte_do2, /set_latte_do4 服务
 *   - DI 反馈灯 (DI2/DI3/DI4) — 通过 /latte_di_status 话题
 *   - 流程步骤示意
 *   - 关节曲线 + 末端位姿（可折叠监控区）
 *
 * 数据流:
 *   DI: rosbridge ← /latte_di_status (JSON/CSV) → parseDi → 信号灯状态
 *   DO: 点击开关 → 纯前端切换 UI 状态（与旧版 coffee_latte_io.js 一致）喵~
 */
import { useRos } from '@/composables/useRos'
import { useJointChart } from '@/composables/useJointChart'
import { useDashboardSettings } from '@/composables/useDashboardSettings'
import { canonicalRosTopic } from '@/lib/utils'
import Robot3dViewer from '@/components/ivg/Robot3dViewer.vue'
import {
  LATTE_DI_STATUS_TOPIC, LATTE_DI_STATUS_TYPE,
  ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE,
  JOINT_STATES_TOPIC, JOINT_STATES_TYPE,
  TF_TOPIC, TF_STATIC_TOPIC, TF_TYPE,
} from '@/constants/ros'

const { isConnected, subscribe, onRosJson, onControlJson } = useRos()
const settings = useDashboardSettings()

const latteDiTopic = computed(() => settings.rosName('latte-di-topic', LATTE_DI_STATUS_TOPIC))
const robotStatusTopic = computed(() => settings.rosName('topic-robot', ROBOT_STATUS_TOPIC))
const jointStatesTopic = computed(() => settings.rosName('topic-joints', JOINT_STATES_TOPIC))
const tfTopic = computed(() => settings.rosName('topic-tf', TF_TOPIC))
const tfStaticTopic = computed(() => settings.rosName('topic-tf-static', TF_STATIC_TOPIC))
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

/** 解析 DI 消息 — 支持 JSON {"di2":1,"di3":0,"di4":1} 和 CSV "1,0,1" 两种格式 */
function parseDi(msg: any) {
  let text = typeof msg === 'string' ? msg : msg?.data ? String(msg.data) : ''
  if (!text) return
  try { const obj = JSON.parse(text); if (obj && typeof obj === 'object') { di2.value = !!obj.di2; di3.value = !!obj.di3; di4.value = !!obj.di4; return } } catch { /* 尝试 CSV */ }
  const parts = text.split(',').map((s: string) => s.trim())
  if (parts.length >= 3) { di2.value = parts[0] === '1'; di3.value = parts[1] === '1'; di4.value = parts[2] === '1' }
}

// ═══════════════════════ DO 开关 ═══════════════════════

const do2On = ref(false); const do4On = ref(false)

function toggleDo(_id: 2 | 4, _next: boolean) {
  // 旧版 coffee_latte_io.js 明确为纯前端切换，不调用后端服务喵~
}

// ═══════════════════════ 末端位姿 ═══════════════════════

const robotPose = reactive({
  x: '—', y: '—', z: '—',
  qx: '—', qy: '—', qz: '—', qw: '—',
  roll: '—', pitch: '—', yaw: '—',
})

const robotPoseCache = ref('')

// ═══════════════════════ 关节曲线 ═══════════════════════

const { canvasRef: chartCanvasRef, legendRef: chartLegendRef, pushSample: pushJointSample, reset: resetJointChart, observeResize: observeJointChart } = useJointChart()

// ═══════════════════════ 监控区折叠 ═══════════════════════

const MONITORING_KEY = 'ivg_vision_monitoring_collapsed'
const monitoringCollapsed = ref(false)
try {
  monitoringCollapsed.value = localStorage.getItem(MONITORING_KEY) === '1'
} catch { /* */ }

function toggleMonitoring(): void {
  monitoringCollapsed.value = !monitoringCollapsed.value
  try { localStorage.setItem(MONITORING_KEY, monitoringCollapsed.value ? '1' : '0') } catch { /* */ }
  if (!monitoringCollapsed.value) {
    nextTick(() => observeJointChart())
  }
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

  robotPose.x = p.x?.toFixed(4) ?? '—'
  robotPose.y = p.y?.toFixed(4) ?? '—'
  robotPose.z = p.z?.toFixed(4) ?? '—'
  robotPose.qx = o.x?.toFixed(4) ?? '—'
  robotPose.qy = o.y?.toFixed(4) ?? '—'
  robotPose.qz = o.z?.toFixed(4) ?? '—'
  robotPose.qw = o.w?.toFixed(4) ?? '—'

  const rr = Number(r.x), rp = Number(r.y), ry = Number(r.z)
  if (isFinite(rr) && isFinite(rp) && isFinite(ry)) {
    robotPose.roll = (rr * 180 / Math.PI).toFixed(1) + '°'
    robotPose.pitch = (rp * 180 / Math.PI).toFixed(1) + '°'
    robotPose.yaw = (ry * 180 / Math.PI).toFixed(1) + '°'
  }

  // 缓存位姿 HTML（与旧版 robotPoseCache 行为一致）
  if (p.x != null && o.x != null) {
    robotPoseCache.value = JSON.stringify({
      x: robotPose.x, y: robotPose.y, z: robotPose.z,
      qx: robotPose.qx, qy: robotPose.qy, qz: robotPose.qz, qw: robotPose.qw,
      roll: robotPose.roll, pitch: robotPose.pitch, yaw: robotPose.yaw,
    })
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
  subscribe(tfTopic.value, settings.topicType('topic-tf', TF_TYPE), 30)
  subscribe(tfStaticTopic.value, settings.topicType('topic-tf-static', TF_TYPE), 1)
}

onControlJson((c) => { if (c.op === 'connection') setupSubs() })
watch(isConnected, v => { if (v) setupSubs() })
if (isConnected()) setupSubs()

onMounted(() => {
  observeJointChart()
  settings.loadSettings().then(() => { if (isConnected()) setupSubs() }).catch(() => {})
})

watch(() => settings.version.value, () => { if (isConnected()) setupSubs() })
</script>

<template>
  <div class="ivg-run-page w-full max-w-none px-2 sm:px-3 py-2 overflow-x-hidden">
    <!-- ═══ 顶部工具栏 ═══ -->
    <div class="ivg-topbar flex items-center justify-between mb-2 bg-white rounded-lg border border-slate-200 px-3 py-2">
      <div class="flex items-center gap-3">
        <h1 class="text-lg font-bold text-slate-900">机械臂咖啡拉花</h1>
        <el-tag size="small" type="warning" round>演示</el-tag>
      </div>
      <div class="flex items-center gap-2">
        <a href="/settings" target="_blank" rel="noopener" class="text-xs text-blue-500 hover:text-blue-600 no-underline">话题与服务设置</a>
        <span class="text-xs" :class="connOk ? 'text-green-600' : connStatus === '正在连接…' ? 'text-slate-400' : 'text-red-500'">
          {{ connStatus }}
        </span>
      </div>
    </div>

    <!-- ═══ 主内容区: 左(3D+流程) / 右(IO控制) ═══ -->
    <div class="ivg-main-grid grid grid-cols-1 min-[921px]:grid-cols-[minmax(0,1fr)_minmax(15rem,22vw)] gap-2 min-h-0">
      <!-- 左栏: 3D 模型 + 流程示意 -->
      <div class="ivg-visual-grid min-w-0 grid grid-cols-1 min-[921px]:grid-cols-2 gap-2 content-start">
        <!-- 机械臂 3D 模型 -->
        <section class="bg-white rounded-lg border border-slate-200 overflow-hidden" aria-label="机械臂 URDF 模型">
          <h2 class="text-xs font-bold text-slate-500 uppercase px-3 pt-3 pb-1">机械臂模型</h2>
          <Robot3dViewer
            :urdf-param="urdfParam"
            :fixed-frame="fixedFrame"
            :tool-status-topic="toolStatusTopic"
            class="w-full"
          />
        </section>

        <!-- 流程示意 -->
        <section class="bg-white rounded-lg border border-slate-200 p-4" aria-label="咖啡流程示意">
          <h2 class="text-xs font-bold text-slate-500 uppercase mb-3">咖啡流程示意</h2>
          <div class="flex items-center gap-3 text-sm">
            <span class="flex items-center gap-1.5"><span class="w-6 h-6 rounded-full bg-blue-100 text-blue-700 flex items-center justify-center text-xs font-bold">1</span>出杯/定位</span>
            <span class="text-slate-300">→</span>
            <span class="flex items-center gap-1.5"><span class="w-6 h-6 rounded-full bg-blue-100 text-blue-700 flex items-center justify-center text-xs font-bold">2</span>萃取与奶泡</span>
            <span class="text-slate-300">→</span>
            <span class="flex items-center gap-1.5"><span class="w-6 h-6 rounded-full bg-blue-100 text-blue-700 flex items-center justify-center text-xs font-bold">3</span>拉花执行</span>
          </div>
          <p class="text-xs text-slate-400 mt-2">右侧 IO 状态接入后，可在此叠加图文或相机画面。</p>
        </section>
      </div>

      <!-- 右栏: IO 控制 -->
      <div class="min-w-0 space-y-2">
        <!-- 工序开关 DO -->
        <div class="bg-white rounded-lg border border-slate-200 p-4">
          <h2 class="text-sm font-bold text-slate-700 mb-3">拉花工序与 IO</h2>
          <p class="text-xs text-slate-400 mb-3">工序开关（DO）与连接日志；反馈信号灯（DI）见底部监控区。</p>
          <h3 class="text-xs font-bold text-slate-500 uppercase mb-3">工序开关（DO）</h3>
          <div class="space-y-3">
            <div class="flex items-center justify-between">
              <div>
                <span class="text-sm font-medium text-slate-700">咖啡开关</span>
                <el-tag size="small" class="ml-2">DO4</el-tag>
              </div>
                <el-switch v-model="do4On" @change="(v) => toggleDo(4, Boolean(v))" />
            </div>
            <div class="flex items-center justify-between">
              <div>
                <span class="text-sm font-medium text-slate-700">打花开关</span>
                <el-tag size="small" class="ml-2">DO2</el-tag>
              </div>
                <el-switch v-model="do2On" @change="(v) => toggleDo(2, Boolean(v))" />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ═══ 底部: 关节曲线与末端位姿（可折叠） ═══ -->
    <div class="mt-2" :class="{ 'is-monitoring-collapsed': monitoringCollapsed }">
      <!-- 折叠/展开按钮 -->
      <button
        class="w-full flex items-center justify-between bg-white rounded-lg border border-slate-200 px-4 py-2 text-sm font-medium text-slate-600 hover:bg-slate-50 transition-colors"
        :aria-expanded="!monitoringCollapsed"
        aria-controls="monitoring-bundle"
        @click="toggleMonitoring"
      >
        <span>关节曲线与末端位姿</span>
        <span class="text-xs text-slate-400">{{ monitoringCollapsed ? '展开' : '收起' }}</span>
      </button>

      <!-- 监控内容 -->
      <div v-show="!monitoringCollapsed" id="monitoring-bundle" class="ivg-monitoring-grid grid grid-cols-1 md:grid-cols-2 gap-2 mt-2">
        <!-- 关节角曲线 -->
        <div class="bg-slate-900 rounded-lg border border-slate-700 p-3">
          <h2 class="text-xs font-bold text-slate-400 uppercase mb-2">关节角曲线</h2>
          <div ref="chartLegendRef" class="flex flex-wrap gap-2 mb-2" role="list" aria-label="关节与曲线颜色" />
          <canvas ref="chartCanvasRef" width="640" height="220" class="w-full" aria-label="各关节位置随时间变化" />
        </div>

        <!-- 位姿读数 + DI 反馈灯 -->
        <div class="space-y-3">
          <!-- 机械臂末端位姿 -->
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

          <!-- 反馈信号灯 DI -->
          <div class="bg-white rounded-lg border border-slate-200 p-4">
            <h2 class="text-sm font-bold text-slate-500 uppercase mb-3">反馈信号灯（DI）</h2>
            <div class="space-y-3">
              <div class="flex items-center gap-3">
                <span class="w-3 h-3 rounded-full transition-colors" :class="di2 ? 'bg-green-500 shadow-[0_0_6px] shadow-green-500' : 'bg-slate-300'" />
                <span class="text-sm text-slate-600">咖啡反馈</span>
                <el-tag size="small">DI2</el-tag>
              </div>
              <div class="flex items-center gap-3">
                <span class="w-3 h-3 rounded-full transition-colors" :class="di4 ? 'bg-amber-500 shadow-[0_0_6px] shadow-amber-500' : 'bg-slate-300'" />
                <span class="text-sm text-slate-600">警告反馈</span>
                <el-tag size="small">DI4</el-tag>
              </div>
              <div class="flex items-center gap-3">
                <span class="w-3 h-3 rounded-full transition-colors" :class="di3 ? 'bg-green-500 shadow-[0_0_6px] shadow-green-500' : 'bg-slate-300'" />
                <span class="text-sm text-slate-600">打花反馈</span>
                <el-tag size="small">DI3</el-tag>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ═══ 底部连接状态栏 ═══ -->
    <footer class="mt-2 bg-white rounded-lg border border-slate-200 px-3 py-1.5 flex items-center gap-2 text-xs" role="contentinfo" aria-label="连接状态">
      <span class="text-slate-400">连接状态</span>
      <span :class="connOk ? 'text-green-600' : connStatus === '正在连接…' ? 'text-slate-400' : 'text-red-500'">
        {{ connStatus }}
      </span>
    </footer>
  </div>
</template>
