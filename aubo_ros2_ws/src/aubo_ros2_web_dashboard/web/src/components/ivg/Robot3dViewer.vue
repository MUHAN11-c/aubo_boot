<script setup lang="ts">
/**
 * Robot3dViewer — 机械臂 3D URDF 模型实时渲染组件
 *
 * 替代旧版: vision_urdf_panel.js (urdf_panel.js + session.js + tf_clients.js + patches.js)
 *
 * 功能:
 *   - Three.js 场景 (SceneManager)
 *   - 从 rosbridge rosapi 加载 URDF 参数
 *   - 构建 Three.js Object3D 模型
 *   - 订阅 /tf + /tf_static，按 fixedFrame→link TF 更新模型位置关系
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
import { useDashboardSettings } from '@/composables/useDashboardSettings'
import * as THREE from 'three'
import { STLLoader } from 'three/addons/loaders/STLLoader.js'

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
const settings = useDashboardSettings()

// sceneMgr 必须用 shallowRef — 模板 v-if 依赖响应式追踪。
// Three.js 对象不能深度代理，shallowRef 只追踪 .value 替换。
const sceneMgr = shallowRef<SceneManager | null>(null)
let urdfModel: UrdfModel | null = null
let tfUpdater: TfUpdater | null = null
let currentToolId: string | null = null
let toolModel: { id: string; root: THREE.Group; offsetPos: THREE.Vector3; offsetQuat: THREE.Quaternion } | null = null
let toolModelLoadSeq = 0
let linkUpdateTimer: ReturnType<typeof setInterval> | null = null
let urdfFocusTimer: ReturnType<typeof setInterval> | null = null
let urdfCameraPrimed = false
let initAttempted = false

const MESH_BASE = `${location.origin}/api/ivg/robot-mesh/`
const tfTopic = computed(() => settings.rosName('topic-tf', '/tf'))
const tfStaticTopic = computed(() => settings.rosName('topic-tf-static', '/tf_static'))
const jointStatesTopic = computed(() => settings.rosName('topic-joints', '/joint_states'))

const TOOL_MODELS: Record<string, {
  mesh: string
  offset: { position: [number, number, number]; orientation: [number, number, number, number] }
}> = {
  gripper0: {
    mesh: `${MESH_BASE}aubo_description/meshes/visual/gripper0_link.stl`,
    offset: { position: [0, 0, 0.033], orientation: [0, 0, 0, 1] },
  },
  gripper1: {
    mesh: `${MESH_BASE}aubo_description/meshes/visual/gripper1_link.stl`,
    offset: { position: [0, 0, 0.033], orientation: [0, 0, 0, 1] },
  },
  gripper2: {
    mesh: `${MESH_BASE}aubo_description/meshes/visual/gripper2_link.stl`,
    offset: { position: [0, 0, 0.033], orientation: [0, 0, 0.7071, 0.7071] },
  },
  gripper1coffeecup: {
    mesh: `${MESH_BASE}aubo_description/meshes/visual/gripper1coffeecup_link.stl`,
    offset: { position: [0, 0, 0.033], orientation: [0, 0, 1, 0] },
  },
  gripper1milkcup: {
    mesh: `${MESH_BASE}aubo_description/meshes/visual/gripper1milkcup_link.stl`,
    offset: { position: [0, 0, 0.033], orientation: [0, 0, -0.7071, 0.7071] },
  },
}

function showHint(html: string): void {
  if (hintRef.value) hintRef.value.innerHTML = html
}

function sameTopic(topic: string, expected: string): boolean {
  const norm = (v: string) => String(v || '').trim().replace(/^\/+/, '')
  return norm(topic) === norm(expected)
}

// ── URDF 加载 ──

async function loadUrdfParam(): Promise<string> {
  const spec = parseUrdfParam(props.urdfParam)
  const result: any = await callService(
    `${spec.node}/get_parameters`,
    'rcl_interfaces/srv/GetParameters',
    { names: [spec.parameter] }
  )
  if (result?.values?.length > 0 && result.values[0]?.string_value) {
    return result.values[0].string_value
  }
  throw new Error(`URDF 参数为空`)
}

function parseUrdfParam(raw?: string): { node: string; parameter: string } {
  const spec = String(raw || '/robot_state_publisher:robot_description').trim()
  if (spec.includes(':')) {
    const [nodeRaw, paramRaw] = spec.split(':', 2)
    const node = nodeRaw.startsWith('/') ? nodeRaw : `/${nodeRaw.replace(/^\/+/, '')}`
    return { node, parameter: paramRaw || 'robot_description' }
  }
  return { node: '/robot_state_publisher', parameter: spec.replace(/^\/+/, '') || 'robot_description' }
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
  let model: UrdfModel | null = null
  let readyBeforeAssign = false
  const commitModel = (nextModel: UrdfModel) => {
    urdfModel = nextModel
    sceneMgr.value?.addObject(nextModel.root)
    urdfCameraPrimed = false
    nextModel.flattenLinksToRoot()
    startUrdfFocusTimer()
    startLinkUpdateLoop()
    emit('ready')
  }
  model = new UrdfModel(robot, MESH_BASE, () => {
    // URDF mesh 加载完成喵~
    if (!model) { readyBeforeAssign = true; return }
    commitModel(model)
  })
  if (readyBeforeAssign) commitModel(model)
}

// ── 无闪烁 reload（工具快换） ──

function reloadUrdf(urdfXml: string): void {
  if (!sceneMgr.value || !urdfModel) return

  const robot = parseUrdf(urdfXml, MESH_BASE)
  let newModel: UrdfModel | null = null
  let readyBeforeAssign = false
  const swapModel = (model: UrdfModel) => {
    // 新模型就绪 → 添加新 → 移除旧
    const oldModel = urdfModel
    if (sceneMgr.value) sceneMgr.value.addObject(model.root)
    if (oldModel && sceneMgr.value) sceneMgr.value.removeObject(oldModel.root)
    urdfModel = model
    urdfCameraPrimed = false
    startUrdfFocusTimer()
    model.flattenLinksToRoot()
    startLinkUpdateLoop()
  }
  newModel = new UrdfModel(robot, MESH_BASE, () => {
    if (!newModel) { readyBeforeAssign = true; return }
    swapModel(newModel)
  })
  if (readyBeforeAssign) swapModel(newModel)
}

// ── 前端工具模型：工具状态只影响 Web 显示，不依赖动态 robot_description 喵~ ──

function removeToolModel(): void {
  if (toolModel?.root && sceneMgr.value) sceneMgr.value.removeObject(toolModel.root)
  toolModel = null
}

function loadToolMesh(toolId: string): void {
  const spec = TOOL_MODELS[toolId]
  if (!spec || !sceneMgr.value) {
    removeToolModel()
    return
  }
  if (toolModel?.id === toolId) return

  const seq = ++toolModelLoadSeq
  const loader = new STLLoader()
  loader.load(spec.mesh, (geometry) => {
    if (seq !== toolModelLoadSeq || !sceneMgr.value) return
    geometry.computeVertexNormals()
    removeToolModel()
    const root = new THREE.Group()
    root.name = `web_tool_${toolId}`
    const mesh = new THREE.Mesh(
      geometry,
      new THREE.MeshPhongMaterial({ color: 0xb8c2cc, shininess: 30 })
    )
    root.add(mesh)
    const [x, y, z] = spec.offset.position
    const [qx, qy, qz, qw] = spec.offset.orientation
    toolModel = {
      id: toolId,
      root,
      offsetPos: new THREE.Vector3(x, y, z),
      offsetQuat: new THREE.Quaternion(qx, qy, qz, qw).normalize(),
    }
    sceneMgr.value.addObject(root)
  }, undefined, () => {
    if (seq === toolModelLoadSeq) removeToolModel()
    emit('error', `工具模型加载失败: ${toolId}`)
  })
}

function updateToolModelTransform(): void {
  if (!toolModel || !tfUpdater) return
  const baseToMount = tfUpdater.getTransform('kuaihuan_Link', props.fixedFrame)
  if (!baseToMount) return
  const offset = toolModel.offsetPos.clone().applyQuaternion(baseToMount.rotation)
  toolModel.root.position.copy(baseToMount.translation).add(offset)
  toolModel.root.quaternion.copy(baseToMount.rotation).multiply(toolModel.offsetQuat)
}

// ── TF link 更新 ──

function startLinkUpdateLoop(): void {
  if (linkUpdateTimer) clearInterval(linkUpdateTimer)
  linkUpdateTimer = setInterval(() => {
    if (!urdfModel || !tfUpdater) return
    for (const link of urdfModel.getLinkObjects()) {
      tfUpdater.updateLinkTransform(link.object, link.name, props.fixedFrame)
    }
    updateToolModelTransform()
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
  subscribe(tfTopic.value, settings.topicType('topic-tf', 'tf2_msgs/msg/TFMessage'), 30)
  subscribe(tfStaticTopic.value, settings.topicType('topic-tf-static', 'tf2_msgs/msg/TFMessage'), 1)
  subscribe(jointStatesTopic.value, settings.topicType('topic-joints', 'sensor_msgs/msg/JointState'), 30)

  // TF 消息处理
  onRosJson(null, (msg: any, topic: string) => {
    if (sameTopic(topic, tfTopic.value) || sameTopic(topic, tfStaticTopic.value)) tfUpdater?.ingestTfMessage(msg)
    // joint_states 保留订阅用于与旧监控链路一致；3D link 位姿以 TF 为唯一来源喵~
    if (sameTopic(topic, jointStatesTopic.value)) tfUpdater?.ingestJointStates(msg)
  })

  // 工具状态监听 → 快换时 reload
  if (props.toolStatusTopic) {
    subscribe(props.toolStatusTopic, settings.topicType('topic-tool-status', 'ivg_interfaces/msg/ToolChangerStatus'), 2)
    onRosJson(null, (msg: any, topic: string) => {
      if (!sameTopic(topic, props.toolStatusTopic)) return
      const newId = msg?.is_connected ? String(msg?.tool_id || '') : ''
      if (newId === currentToolId) return
      currentToolId = newId
      if (newId) loadToolMesh(newId)
      else removeToolModel()
    })
  }

  // 加载 URDF
  try {
    showHint('<strong>加载机械臂 URDF 模型...</strong>')
    const urdfXml = await loadUrdfParam()
    buildModel(urdfXml)
    showHint('')
  } catch (e: any) {
    const msg = `机械臂加载失败：${String(e.message || e)}。请检查参数 ${props.urdfParam} 与固定坐标系 ${props.fixedFrame}。`
    showHint(`<strong style="color:red">${msg}</strong>`)
    emit('error', msg)
  }
}

function stop(): void {
  if (linkUpdateTimer) { clearInterval(linkUpdateTimer); linkUpdateTimer = null }
  if (urdfFocusTimer) { clearInterval(urdfFocusTimer); urdfFocusTimer = null }
  removeToolModel()
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
  settings.loadSettings().then(() => {
    if (isConnected() && hostRef.value && !initAttempted) { initAttempted = true; init() }
  }).catch(() => {})
})

watch(() => [props.urdfParam, props.fixedFrame], () => {
  if (!sceneMgr.value || !isConnected()) return
  loadUrdfParam().then(reloadUrdf).catch((e: any) => emit('error', String(e?.message || e)))
})

onUnmounted(() => stop())
</script>

<template>
  <div class="relative w-full h-[clamp(260px,36vh,400px)] rounded-lg border border-slate-200 bg-white overflow-hidden">
    <div ref="hostRef" class="w-full h-[clamp(260px,36vh,400px)]" title="拖拽旋转视角，滚轮缩放">
      <div v-if="!sceneMgr" class="w-full h-full flex items-center justify-center text-slate-400 text-sm">
        等待 ROS 连接以加载机械臂模型...
      </div>
    </div>
    <div class="absolute left-3 top-3 rounded-md bg-white/90 px-2.5 py-2 text-[11px] leading-tight text-slate-600 shadow-sm ring-1 ring-slate-200 backdrop-blur" aria-label="坐标轴颜色图例">
      <div class="mb-1 font-semibold text-slate-700">坐标轴（Z 向上）</div>
      <div class="flex items-center gap-2">
        <span class="inline-flex items-center gap-1"><span class="h-2 w-2 rounded-full bg-red-500" />X</span>
        <span class="inline-flex items-center gap-1"><span class="h-2 w-2 rounded-full bg-green-500" />Y</span>
        <span class="inline-flex items-center gap-1"><span class="h-2 w-2 rounded-full bg-blue-500" />Z</span>
      </div>
    </div>
    <div
      v-if="!sceneMgr"
      ref="hintRef"
      class="absolute bottom-4 left-1/2 -translate-x-1/2 bg-slate-900/90 text-white text-xs px-4 py-2 rounded-lg shadow-lg"
    />
  </div>
</template>
