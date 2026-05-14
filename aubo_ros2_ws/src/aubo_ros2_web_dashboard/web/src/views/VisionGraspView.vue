<script setup lang="ts">
/**
 * VisionGraspView — 视觉抓取面板（核心页面）
 *
 * 替代旧版: vision_grasp_panel.html + vision_grasp_panel.js (657行) + 12个子模块
 *
 * 页面结构:
 *   ┌──────────────────────────────────────────┐
 *   │  顶部工具栏: 模式切换 + 连接状态           │
 *   ├──────────┬───────────────────────────────┤
 *   │ 左栏     │ 右栏                          │
 *   │ 3D 模型  │ GraspControls (抓取控制)       │
 *   │ 相机画面  │ ToolSwapBar  (工具快换)        │
 *   │          │ 服务日志                       │
 *   ├──────────┴───────────────────────────────┤
 *   │  底部: PoseCard + 关节角                  │
 *   └──────────────────────────────────────────┘
 *
 * 数据流:
 *   rosbridge WebSocket ←→ useRos (全局单例)
 *     → onRosJson 处理消息 → 更新响应式状态 → 模板自动渲染
 *     → callService 调用服务 → Promise → 更新日志
 */
import { useRos } from '@/composables/useRos'
import { useMJPEGStream } from '@/composables/useMJPEGStream'
import { ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE, JOINT_STATES_TOPIC, JOINT_STATES_TYPE, TF_TOPIC, TF_STATIC_TOPIC, TF_TYPE } from '@/constants/ros'

// ═══════════════════════ 通信层 ═══════════════════════

const { isConnected, connect, subscribe, unsubscribeAll, onRosJson, onControlJson } = useRos()

// ═══════════════════════ 页面状态 ═══════════════════════

/** 抓取模式: workpiece(视觉估计) / graspnet(AI大模型) */
const graspMode = ref<'workpiece' | 'graspnet'>('workpiece')
/** 工件编号 (仅工件模式) */
const objectId = ref('')
/** 连接状态文本 */
const connStatus = ref('未连接')
const connOk = ref(false)
/** 服务调用日志 */
const svcLog = ref('等待操作…')

// ═══════════════════════ 传感器数据 ═══════════════════════

/** 末端位姿 (从 /aubo_driver/robot_status 解析) */
const robotPose = reactive({ x: '—', y: '—', z: '—', roll: '—', pitch: '—', yaw: '—' })
/** 关节角弧度 (从 /joint_states 解析) */
const jointAngles = ref<number[]>([])

// ═══════════════════════ 相机流 ═══════════════════════

const colorTopic = ref('/camera/color/image_raw')
const { cameraStreamUrl } = useMJPEGStream(colorTopic)

// ═══════════════════════ 日志 ═══════════════════════

function log(msg: string) {
  svcLog.value = `${new Date().toLocaleTimeString()} ${msg}`
}

// ═══════════════════════ 话题消息处理 ═══════════════════════

// 机器人状态 → 解析末端位姿
onRosJson(ROBOT_STATUS_TOPIC, (msg: any) => {
  const p = msg?.cartesian_position_xyz || {}
  const r = msg?.cartesian_rpy || {}
  robotPose.x = p.x?.toFixed(4) ?? '—'
  robotPose.y = p.y?.toFixed(4) ?? '—'
  robotPose.z = p.z?.toFixed(4) ?? '—'
  robotPose.roll = r.x != null ? (r.x * 180 / Math.PI).toFixed(1) + '°' : '—'
  robotPose.pitch = r.y != null ? (r.y * 180 / Math.PI).toFixed(1) + '°' : '—'
  robotPose.yaw = r.z != null ? (r.z * 180 / Math.PI).toFixed(1) + '°' : '—'
})

// 关节状态 → 提取 position 数组
onRosJson(JOINT_STATES_TOPIC, (msg: any) => {
  if (Array.isArray(msg?.position)) jointAngles.value = msg.position.map((v: number) => Number(v))
})

// ═══════════════════════ 连接管理 ═══════════════════════

onControlJson((c) => {
  if (c.op === 'connection') { connStatus.value = '已连接'; connOk.value = true; setupSubs() }
  if (c.op === 'close') { connStatus.value = '已断开，重连中…'; connOk.value = false; unsubscribeAll() }
  if (c.op === 'error') { connStatus.value = '通信错误'; connOk.value = false }
})

/** 建立所有话题订阅 */
function setupSubs() {
  if (!isConnected()) return
  subscribe(ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE, 50)
  subscribe(JOINT_STATES_TOPIC, JOINT_STATES_TYPE, 30)
  subscribe(TF_TOPIC, TF_TYPE, 30)
  subscribe(TF_STATIC_TOPIC, TF_TYPE, 1)
}

// 页面加载后自动连接
onMounted(() => { connect() })
</script>

<template>
  <div class="max-w-7xl mx-auto px-4 py-4">
    <!-- ═══ 顶部工具栏 ═══ -->
    <div class="flex items-center justify-between mb-4 bg-white rounded-lg border border-slate-200 px-4 py-2">
      <div class="flex items-center gap-3">
        <h1 class="text-lg font-bold text-slate-900">视觉抓取面板</h1>
        <!-- 模式切换 — 工件(视觉估计) / AI大模型 -->
        <el-radio-group v-model="graspMode" size="small">
          <el-radio-button value="workpiece">工件（视觉估计）</el-radio-button>
          <el-radio-button value="graspnet">AI大模型抓取</el-radio-button>
        </el-radio-group>
      </div>
      <!-- 连接状态指示 -->
      <span class="text-xs" :class="connOk ? 'text-green-600' : connStatus === '正在连接…' ? 'text-slate-400' : 'text-red-500'">
        {{ connStatus }}
      </span>
    </div>

    <!-- ═══ 主内容区: 左(3D+相机) / 右(控制面板) ═══ -->
    <div class="grid grid-cols-1 lg:grid-cols-3 gap-4">
      <!-- 左栏: 3D 模型 + 相机画面 -->
      <div class="lg:col-span-2 space-y-4">
        <!-- 机械臂 3D 模型 (ros3d 集成区域) -->
        <div class="bg-white rounded-lg border border-slate-200 p-3 min-h-[360px] flex items-center justify-center text-slate-400 text-sm">
          机械臂 3D 模型区域 — ros3d Viewer (可用时集成)
        </div>
        <!-- 相机实时画面 (MJPEG) -->
        <div class="bg-white rounded-lg border border-slate-200 p-3">
          <h3 class="text-xs font-bold text-slate-500 uppercase mb-2">相机画面</h3>
          <img v-if="isConnected()" :src="cameraStreamUrl()" class="w-full rounded max-h-[400px] object-contain bg-slate-100" alt="相机实时画面" />
          <div v-else class="w-full h-[300px] flex items-center justify-center text-slate-400 text-sm bg-slate-50 rounded">
            等待 rosbridge 连接…
          </div>
        </div>
      </div>

      <!-- 右栏: 控制面板 -->
      <div class="space-y-4">
        <!-- 抓取控制（工件/AI模式自动切换按钮组） -->
        <GraspControls
          :mode="graspMode"
          :object-id="objectId"
          @update:object-id="objectId = $event"
          @log="log"
        />
        <!-- 工具快换 -->
        <ToolSwapBar @log="log" />
        <!-- 服务调用日志 (黑底绿字终端风格) -->
        <div class="bg-slate-900 rounded-lg p-3 text-xs font-mono text-green-400 min-h-[3em] whitespace-pre-wrap leading-relaxed">
          {{ svcLog }}
        </div>
      </div>
    </div>

    <!-- ═══ 底部: 末端位姿 + 关节角 ═══ -->
    <div class="grid grid-cols-1 md:grid-cols-2 gap-4 mt-4">
      <!-- 末端位姿卡片 -->
      <PoseCard v-bind="robotPose" />
      <!-- 关节角列表 -->
      <div class="bg-white rounded-lg border border-slate-200 p-4">
        <h3 class="text-xs font-bold text-slate-500 uppercase mb-2">关节角 (rad)</h3>
        <div v-if="jointAngles.length" class="text-xs font-mono text-slate-600 space-y-0.5">
          <div v-for="(v, i) in jointAngles" :key="i" class="flex justify-between py-0.5">
            <span class="text-slate-400">J{{ i + 1 }}</span>
            <span>{{ v.toFixed(4) }}</span>
          </div>
        </div>
        <span v-else class="text-xs text-slate-400">等待 /joint_states…</span>
      </div>
    </div>
  </div>
</template>
