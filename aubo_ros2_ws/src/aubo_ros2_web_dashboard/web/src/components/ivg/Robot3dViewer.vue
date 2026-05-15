<script setup lang="ts">
/**
 * Robot3dViewer — 机械臂 3D URDF 模型实时渲染组件
 *
 * 替代旧版: vision_urdf_panel.js (urdf_panel.js + session.js + tf_clients.js + patches.js)
 *
 * 功能:
 *   - Three.js 场景 (SceneManager)
 *   - 从 rosbridge rosapi 加载 URDF 参数
 *   - 构建 Three.js Object3D 模型树
 *   - 订阅 /tf + /tf_static + /joint_states 实时更新关节
 *   - 工具快换时 reload（无闪烁切换）
 *   - 自动 camera focus + resize 响应
 *
 * Props:
 *   rosInstance  — ROSLIB.Ros 实例（由 useRos 提供）
 *   urdfParam    — URDF 参数名 (如 '/robot_state_publisher:robot_description')
 *   fixedFrame   — TF 固定坐标系 (默认 'base_link')
 *   toolStatusTopic — 工具快换状态话题
 */
import { SceneManager } from '@/lib/three_urdf/SceneManager'
import { parseUrdf } from '@/lib/three_urdf/UrdfParser'
import { UrdfModel } from '@/lib/three_urdf/UrdfModel'
import { TfUpdater } from '@/lib/three_urdf/TfUpdater'
import { useRos } from '@/composables/useRos'

const props = withDefaults(defineProps<{
  urdfParam?: string
  fixedFrame?: string
  toolStatusTopic?: string
}>(), {
  urdfParam: '/robot_state_publisher:robot_description',
  fixedFrame: 'base_link',
  toolStatusTopic: '/tool_changer_status',
})

const emit = defineEmits<{ ready: []; error: [msg: string] }>()

const hostRef = ref<HTMLElement | null>(null)
const hintRef = ref<HTMLElement | null>(null)

const { subscribe, onRosJson, onControlJson, isConnected, callService } = useRos()

// sceneMgr 必须用 shallowRef — 模板 v-if 依赖响应式追踪。
// Three.js 对象不能深度代理，shallowRef 只追踪 .value 替换。
const sceneMgr = shallowRef<SceneManager | null>(null)
let urdfModel: UrdfModel | null = null
let tfUpdater: TfUpdater | null = null
let currentToolId: string | null = null
let jointUpdateTimer: ReturnType<typeof setInterval> | null = null
let urdfFocusTimer: ReturnType<typeof setInterval> | null = null
let urdfCameraPrimed = false
let initAttempted = false

const MESH_BASE = `${location.origin}/api/ivg/robot-mesh/`

function showHint(html: string): void {
  if (hintRef.value) hintRef.value.innerHTML = html
}

// ── URDF 加载 ──

async function loadUrdfParam(): Promise<string> {
  const result: any = await callService(
    '/robot_state_publisher/get_parameters',
    'rcl_interfaces/srv/GetParameters',
    { names: ['robot_description'] }
  )
  if (result?.values?.length > 0 && result.values[0]?.string_value) {
    return result.values[0].string_value
  }
  throw new Error(`URDF 参数为空`)
}

// ── 构建 3D 模型 ──

function buildModel(urdfXml: string): void {
  if (!sceneMgr.value) return

  // 移除旧模型
  if (urdfModel) {
    sceneMgr.value.removeObject(urdfModel.root)
    urdfModel = null
  }

  const robot = parseUrdf(urdfXml, MESH_BASE)
  urdfModel = new UrdfModel(robot, MESH_BASE, () => {
    // URDF mesh 加载完成
    urdfCameraPrimed = false
    startUrdfFocusTimer()
    emit('ready')

    // 启动关节更新循环
    startJointUpdateLoop()
  })

  sceneMgr.value.addObject(urdfModel.root)
}

// ── 无闪烁 reload（工具快换） ──

function reloadUrdf(urdfXml: string): void {
  if (!sceneMgr || !urdfModel) return

  const robot = parseUrdf(urdfXml, MESH_BASE)
  const newModel = new UrdfModel(robot, MESH_BASE, () => {
    // 新模型就绪 → 添加新 → 移除旧
    const oldModel = urdfModel
    if (oldModel && sceneMgr.value) sceneMgr.value.removeObject(oldModel.root)
    if (sceneMgr.value) sceneMgr.value.addObject(newModel.root)
    urdfModel = newModel
    urdfCameraPrimed = false
    startUrdfFocusTimer()
    startJointUpdateLoop()
  })
}

// ── 关节更新 ──

function startJointUpdateLoop(): void {
  if (jointUpdateTimer) clearInterval(jointUpdateTimer)
  jointUpdateTimer = setInterval(() => {
    if (!urdfModel || !tfUpdater) return
    const activeJoints = urdfModel.getActiveJoints()
    for (const j of activeJoints) {
      tfUpdater.updateJointTransform(j.linkObj.object, j.name, j.linkObj.jointType, j.axis)
    }
  }, 30) // ~30 Hz 更新
}

// ── URDF camera focus 定时器 ──

function startUrdfFocusTimer(): void {
  if (urdfFocusTimer) clearInterval(urdfFocusTimer)
  let ticks = 0
  urdfFocusTimer = setInterval(() => {
    ticks++
    if (!urdfModel || !sceneMgr.value) {
      clearInterval(urdfFocusTimer!); urdfFocusTimer = null; return
    }
    const sm = sceneMgr.value
    if (!urdfCameraPrimed) {
      try {
        sm.focusOnObject(urdfModel.root, { distanceFactor: 2.8, minDistance: 0.48 })
        urdfCameraPrimed = true
        clearInterval(urdfFocusTimer!); urdfFocusTimer = null
      } catch { /* */ }
    }
    if (ticks > 48) {
      clearInterval(urdfFocusTimer!); urdfFocusTimer = null
      if (!urdfCameraPrimed) {
        try { sm.focusOnObject(urdfModel!.root, { distanceFactor: 3, minDistance: 0.55 }) } catch { /* */ }
      }
    }
  }, 250)
}

// ── 生命周期 ──

async function init(): Promise<void> {
  const host = hostRef.value
  if (!host) return

  // 创建场景
  sceneMgr.value = new SceneManager(host)
  tfUpdater = new TfUpdater()

  // 订阅 TF
  subscribe('/tf', 'tf2_msgs/msg/TFMessage', 30)
  subscribe('/tf_static', 'tf2_msgs/msg/TFMessage', 1)
  subscribe('/joint_states', 'sensor_msgs/msg/JointState', 30)

  // TF 消息处理
  onRosJson('/tf', (msg: any) => tfUpdater?.ingestTfMessage(msg))
  onRosJson('/tf_static', (msg: any) => tfUpdater?.ingestTfMessage(msg))
  onRosJson('/joint_states', (msg: any) => tfUpdater?.ingestJointStates(msg))

  // 工具状态监听 → 快换时 reload
  if (props.toolStatusTopic) {
    subscribe(props.toolStatusTopic, 'ivg_interfaces/msg/ToolChangerStatus', 2)
    onRosJson(props.toolStatusTopic, (msg: any) => {
      const newId = msg?.tool_id
      if (!newId) return
      if (currentToolId === null) {
        currentToolId = newId
      } else if (newId !== currentToolId) {
        currentToolId = newId
        // 延迟 200ms 后 reload URDF（等 robot_description 参数更新）
        setTimeout(() => {
          loadUrdfParam().then(reloadUrdf).catch(() => {})
        }, 200)
      }
    })
  }

  // 加载 URDF
  try {
    showHint('<strong>加载机械臂 URDF 模型...</strong>')
    const urdfXml = await loadUrdfParam()
    buildModel(urdfXml)
    showHint('')
  } catch (e: any) {
    const msg = `机械臂加载失败：${String(e.message || e)}。请检查参数 ${props.urdfParam}。`
    showHint(`<strong style="color:red">${msg}</strong>`)
    emit('error', msg)
  }
}

function stop(): void {
  if (jointUpdateTimer) { clearInterval(jointUpdateTimer); jointUpdateTimer = null }
  if (urdfFocusTimer) { clearInterval(urdfFocusTimer); urdfFocusTimer = null }
  if (sceneMgr.value) { sceneMgr.value.stop(); sceneMgr.value = null }
  urdfModel = null
  tfUpdater = null
  currentToolId = null
  urdfCameraPrimed = false
  initAttempted = false
}

// 连接后自动启动 — watch isConnected 函数（内部读 connected.value，响应式追踪）
watch(isConnected, (v) => {
  if (v && hostRef.value && !sceneMgr.value && !initAttempted) { initAttempted = true; init() }
})

onMounted(() => {
  if (isConnected() && hostRef.value && !initAttempted) { initAttempted = true; init() }
})

onUnmounted(() => stop())
</script>

<template>
  <div class="relative w-full h-[400px] rounded-lg border border-slate-200 bg-white overflow-hidden">
    <div ref="hostRef" class="w-full h-[400px]" title="拖拽旋转视角，滚轮缩放">
      <div v-if="!sceneMgr" class="w-full h-full flex items-center justify-center text-slate-400 text-sm">
        等待 ROS 连接以加载机械臂模型...
      </div>
    </div>
    <div
      v-if="!sceneMgr"
      ref="hintRef"
      class="absolute bottom-4 left-1/2 -translate-x-1/2 bg-slate-900/90 text-white text-xs px-4 py-2 rounded-lg shadow-lg"
    />
  </div>
</template>
