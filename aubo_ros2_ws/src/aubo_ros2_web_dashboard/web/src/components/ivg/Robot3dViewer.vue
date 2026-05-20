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
import { useRos } from '@/composables/ros/useRos'
import { useDashboardSettings } from '@/composables/settings/useDashboardSettings'
import * as THREE from 'three'
import { STLLoader } from 'three/addons/loaders/STLLoader.js'

const props = withDefaults(defineProps<{
  urdfParam?: string
  fixedFrame?: string
  toolStatusTopic?: string
  trajectoryOverlay?: {
    tcp_path: { x: number; y: number; z: number }[]
    spout_path: { x: number; y: number; z: number }[]
    cup_pose: { x: number; y: number; z: number }
    workspace_bounds: { x_min: number; x_max: number; y_min: number; y_max: number; z_min: number; z_max: number }
  } | null
}>(), {
  urdfParam: '/robot_state_publisher:robot_description',
  fixedFrame: 'base_link',
  toolStatusTopic: '/tool_changer_status',
  trajectoryOverlay: null,
})

const emit = defineEmits<{ ready: []; error: [msg: string] }>()

const hostRef = ref<HTMLElement | null>(null)
const hintRef = ref<HTMLElement | null>(null)

const { subscribe, unsubscribe, onRosJson, onControlJson, isConnected, callService } = useRos()
const settings = useDashboardSettings()

// sceneMgr 必须用 shallowRef — 模板 v-if 依赖响应式追踪。
// Three.js 对象不能深度代理，shallowRef 只追踪 .value 替换。
const sceneMgr = shallowRef<SceneManager | null>(null)
let urdfModel: UrdfModel | null = null
let tfUpdater: TfUpdater | null = null
let currentToolId: string | null = null
let toolModel: { id: string; root: THREE.Group; offsetPos: THREE.Vector3; offsetQuat: THREE.Quaternion } | null = null
let toolModelLoadSeq = 0
let linkUpdateRafId: number | null = null
let lastLinkUpdateMs = 0
let urdfFocusTimer: ReturnType<typeof setInterval> | null = null
let urdfCameraPrimed = false
let initAttempted = false
/** 异步初始化代数 — stop() 递增使在途 async 操作失效，防止竞态条件喵~ */
let initGen = 0

const MESH_BASE = `${location.origin}/api/ivg/robot-mesh/`
const tfTopic = computed(() => settings.rosName('topic-tf', '/tf'))
const tfStaticTopic = computed(() => settings.rosName('topic-tf-static', '/tf_static'))
const jointStatesTopic = computed(() => settings.rosName('topic-joints', '/joint_states'))

// ── 工具几何数据（动态从 BFF /api/v1/tool-geometries 获取，消除与 tools.yaml 的 DRY 违规）──

interface ToolGeometry {
  name: string
  type: string
  mesh_url: string
  attach_offset: {
    position: [number, number, number]
    orientation: [number, number, number, number]
  }
}

const toolGeometries = ref<Record<string, ToolGeometry>>({})

/** 硬编码回退数据（BFF 不可用时使用，与 tools.yaml 当前内容一致）喵~ */
const TOOL_GEOMETRIES_FALLBACK: Record<string, ToolGeometry> = {
  gripper0: {
    name: '气动夹爪 φ40', type: 'gripper',
    mesh_url: `${MESH_BASE}aubo_description/meshes/visual/gripper0_link.stl`,
    attach_offset: { position: [0, 0, 0.033], orientation: [0, 0, 0, 1] },
  },
  gripper1: {
    name: '电动夹爪 A', type: 'gripper',
    mesh_url: `${MESH_BASE}aubo_description/meshes/visual/gripper1_link.stl`,
    attach_offset: { position: [0, 0, 0.033], orientation: [0, 0, 0, 1] },
  },
  gripper2: {
    name: '电动夹爪 φ60', type: 'gripper',
    mesh_url: `${MESH_BASE}aubo_description/meshes/visual/gripper2_link.stl`,
    attach_offset: { position: [0, 0, 0.033], orientation: [0, 0, 0.7071, 0.7071] },
  },
  gripper1coffeecup: {
    name: '咖啡杯工具', type: 'other',
    mesh_url: `${MESH_BASE}aubo_description/meshes/visual/gripper1coffeecup_link.stl`,
    attach_offset: { position: [0, 0, 0.033], orientation: [0, 0, 1, 0] },
  },
  gripper1milkcup: {
    name: '牛奶杯工具', type: 'other',
    mesh_url: `${MESH_BASE}aubo_description/meshes/visual/gripper1milkcup_link.stl`,
    attach_offset: { position: [0, 0, 0.033], orientation: [0, 0, -0.7071, 0.7071] },
  },
}

async function fetchToolGeometries(): Promise<void> {
  try {
    const resp = await fetch('/api/v1/tool-geometries')
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`)
    const data = await resp.json()
    toolGeometries.value = data as Record<string, ToolGeometry>
  } catch (e) {
    console.warn('[Robot3dViewer] 无法获取工具几何数据，使用回退值:', e)
    toolGeometries.value = TOOL_GEOMETRIES_FALLBACK
  }
}

function showHint(html: string): void {
  if (hintRef.value) hintRef.value.innerHTML = html
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
    const urdf = result.values[0].string_value
    console.log('[Robot3dViewer] URDF 获取成功, 大小:', urdf.length, 'bytes, 前100字符:', urdf.substring(0, 100))
    return urdf
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
  console.log('[Robot3dViewer] buildModel 开始, URDF 大小:', urdfXml.length)

  // 移除旧模型
  if (urdfModel) {
    sceneMgr.value.removeObject(urdfModel.root)
    urdfModel = null
  }

  const robot = parseUrdf(urdfXml, MESH_BASE)
  let model: UrdfModel | null = null
  let readyBeforeAssign = false
  const commitModel = (nextModel: UrdfModel) => {
    console.log('[Robot3dViewer] commitModel: 添加到场景, links:', nextModel.links.size, 'root children:', nextModel.root.children.length)
    urdfModel = nextModel
    sceneMgr.value?.addObject(nextModel.root)
    urdfCameraPrimed = false
    nextModel.flattenLinksToRoot()
    console.log('[Robot3dViewer]  flattenLinksToRoot 完成, scene children:', sceneMgr.value?.scene.children.length)
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
  const spec = toolGeometries.value[toolId]
  if (!spec || !sceneMgr.value) {
    removeToolModel()
    return
  }
  if (toolModel?.id === toolId) return

  const seq = ++toolModelLoadSeq
  const loader = new STLLoader()
  loader.load(spec.mesh_url, (geometry) => {
    if (seq !== toolModelLoadSeq || !sceneMgr.value) return
    geometry.computeVertexNormals()
    removeToolModel()
    const root = new THREE.Group()
    root.name = `web_tool_${toolId}`
    const mesh = new THREE.Mesh(
      geometry,
      // MeshPhongMaterial — 对齐 RViz2 OGRE Phong 着色模型喵~
      new THREE.MeshPhongMaterial({ color: 0xb8c2cc, specular: 0x111111, shininess: 30 })
    )
    root.add(mesh)
    const [x, y, z] = spec.attach_offset.position
    const [qx, qy, qz, qw] = spec.attach_offset.orientation
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

// ── 轨迹叠加层 (拉花轨迹预览) ──

let trajOverlayGroup: THREE.Group | null = null

/** 清除旧的轨迹叠加层 (含 GPU 资源释放) 喵~ */
function clearTrajectoryOverlay(): void {
  if (trajOverlayGroup) {
    trajOverlayGroup.traverse((child) => {
      if (child instanceof THREE.Mesh || child instanceof THREE.Line || child instanceof THREE.LineSegments) {
        child.geometry?.dispose()
        if (Array.isArray(child.material)) child.material.forEach(m => m.dispose())
        else child.material?.dispose()
      }
    })
    if (sceneMgr.value) sceneMgr.value.removeObject(trajOverlayGroup)
  }
  trajOverlayGroup = null
}

/** 渲染轨迹叠加层到场景 喵~ */
function renderTrajectoryOverlay(data: NonNullable<typeof props.trajectoryOverlay>): void {
  if (!sceneMgr.value) return
  clearTrajectoryOverlay()

  const group = new THREE.Group()
  group.name = 'latte_trajectory_overlay'

  // TCP 路径 — 绿色实线
  if (data.tcp_path?.length > 1) {
    const pts = data.tcp_path.map(p => new THREE.Vector3(p.x, p.y, p.z))
    const geom = new THREE.BufferGeometry().setFromPoints(pts)
    const line = new THREE.Line(geom, new THREE.LineBasicMaterial({ color: 0x22c55e, linewidth: 2 })) // green-500
    line.name = 'tcp_path'
    group.add(line)
  }

  // 壶嘴路径 — 蓝色实线
  if (data.spout_path?.length > 1) {
    const pts = data.spout_path.map(p => new THREE.Vector3(p.x, p.y, p.z))
    const geom = new THREE.BufferGeometry().setFromPoints(pts)
    const line = new THREE.Line(geom, new THREE.LineBasicMaterial({ color: 0x3b82f6, linewidth: 2 })) // blue-500
    line.name = 'spout_path'
    group.add(line)
  }

  // 杯子位姿 — 黄色坐标轴 + 小方块
  if (data.cup_pose) {
    const { x, y, z } = data.cup_pose
    const axes = new THREE.AxesHelper(0.08)
    axes.position.set(x, y, z)
    axes.name = 'cup_axes'
    group.add(axes)

    const cube = new THREE.Mesh(
      new THREE.BoxGeometry(0.04, 0.04, 0.04),
      new THREE.MeshBasicMaterial({ color: 0xeab308 }) // yellow-500
    )
    cube.position.set(x, y, z)
    cube.name = 'cup_cube'
    group.add(cube)
  }

  // 工作空间安全边界 — 红色半透明线框
  if (data.workspace_bounds) {
    const b = data.workspace_bounds
    const sx = b.x_max - b.x_min; const sy = b.y_max - b.y_min; const sz = b.z_max - b.z_min
    const cx = (b.x_max + b.x_min) / 2; const cy = (b.y_max + b.y_min) / 2; const cz = (b.z_max + b.z_min) / 2
    const boxGeom = new THREE.BoxGeometry(sx, sy, sz)
    const edges = new THREE.EdgesGeometry(boxGeom)
    const wireframe = new THREE.LineSegments(
      edges,
      new THREE.LineBasicMaterial({ color: 0xef4444 }) // red-500
    )
    wireframe.position.set(cx, cy, cz)
    wireframe.name = 'workspace_bounds'
    group.add(wireframe)
  }

  sceneMgr.value.addObject(group)
  trajOverlayGroup = group
}

// 监听 trajectoryOverlay prop 变化 (整体替换,无需 deep) 喵~
const _pendingOverlay = ref<any>(null)
watch(() => props.trajectoryOverlay, (data) => {
  if (!data) { clearTrajectoryOverlay(); _pendingOverlay.value = null; return }
  if (!sceneMgr.value) { _pendingOverlay.value = data; return }
  renderTrajectoryOverlay(data)
  _pendingOverlay.value = null
})
// sceneMgr 就绪后渲染缓存的 overlay 喵~
watch(sceneMgr, (sm) => {
  if (sm && _pendingOverlay.value) {
    renderTrajectoryOverlay(_pendingOverlay.value)
    _pendingOverlay.value = null
  }
})

// ── TF link 更新 ──

function startLinkUpdateLoop(): void {
  if (linkUpdateRafId) cancelAnimationFrame(linkUpdateRafId)
  const MIN_INTERVAL_MS = 30  // ~33Hz, 与 /tf 话题发布频率匹配
  function tick(nowMs: number) {
    if (nowMs - lastLinkUpdateMs >= MIN_INTERVAL_MS) {
      lastLinkUpdateMs = nowMs
      if (urdfModel && tfUpdater) {
        for (const link of urdfModel.getLinkObjects()) {
          tfUpdater.updateLinkTransform(link.object, link.name, props.fixedFrame)
        }
        updateToolModelTransform()
      }
    }
    linkUpdateRafId = requestAnimationFrame(tick)
  }
  lastLinkUpdateMs = performance.now()
  linkUpdateRafId = requestAnimationFrame(tick)
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
  console.log('[Robot3dViewer] 订阅 TF:', tfTopic.value, tfStaticTopic.value, jointStatesTopic.value)
  subscribe(tfTopic.value, settings.topicType('topic-tf', 'tf2_msgs/msg/TFMessage'), 30)
  subscribe(tfStaticTopic.value, settings.topicType('topic-tf-static', 'tf2_msgs/msg/TFMessage'), 1)
  subscribe(jointStatesTopic.value, settings.topicType('topic-joints', 'sensor_msgs/msg/JointState'), 30)

  // TF 消息处理 — 使用目标话题名替代 null 通配，避免每条消息触发过滤喵~
  let tfFirst = true
  const handleTf = (msg: any) => {
    if (tfFirst) { console.log('[Robot3dViewer] 首次收到 /tf 消息, transforms:', msg?.transforms?.length || 0); tfFirst = false }
    tfUpdater?.ingestTfMessage(msg)
  }
  let jsFirst = true
  const handleJointStates = (msg: any) => {
    if (jsFirst) { console.log('[Robot3dViewer] 首次收到 /joint_states 消息, names:', msg?.name); jsFirst = false }
    tfUpdater?.ingestJointStates(msg)
  }
  onRosJson(tfTopic.value, handleTf)
  onRosJson(tfStaticTopic.value, handleTf)
  onRosJson(jointStatesTopic.value, handleJointStates)

  const curGen = initGen  // 记录当前代数，await 后检查是否已卸载/重挂载喵~

  // 获取工具几何数据（在工具状态订阅之前，避免竞态）喵~
  await fetchToolGeometries()
  if (curGen !== initGen) return  // 组件在 await 期间已卸载或重挂载，丢弃本次初始化喵~

  // 工具状态监听 → 快换时 reload
  if (props.toolStatusTopic) {
    subscribe(props.toolStatusTopic, settings.topicType('topic-tool-status', 'ivg_interfaces/msg/ToolChangerStatus'), 2)
    onRosJson(props.toolStatusTopic, (msg: any) => {
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
    if (curGen !== initGen) return  // await 期间场景可能已 dispose 喵~
    buildModel(urdfXml)
    showHint('')
  } catch (e: any) {
    if (curGen !== initGen) return  // 场景已 dispose，无需报错喵~
    const msg = `机械臂加载失败：${String(e.message || e)}。请检查参数 ${props.urdfParam} 与固定坐标系 ${props.fixedFrame}。`
    showHint(`<strong style="color:red">${msg}</strong>`)
    emit('error', msg)
  }
}

function stop(): void {
  // 递增代数使所有在途 async init() 操作失效，防止操作已 dispose 的场景喵~
  initGen++
  if (linkUpdateRafId) { cancelAnimationFrame(linkUpdateRafId); linkUpdateRafId = null }
  if (urdfFocusTimer) { clearInterval(urdfFocusTimer); urdfFocusTimer = null }
  clearTrajectoryOverlay()
  removeToolModel()
  // 取消 ROS 话题订阅 — 未取消会导致 rosbridge 持续推送，浪费带宽喵~
  unsubscribe(tfTopic.value)
  unsubscribe(tfStaticTopic.value)
  unsubscribe(jointStatesTopic.value)
  if (props.toolStatusTopic) unsubscribe(props.toolStatusTopic)
  // 清理 TF 缓存，避免下一实例继承旧的 TF 树状态喵~
  tfUpdater?.clear()
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
