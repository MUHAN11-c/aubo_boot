<script setup lang="ts">
/**
 * VisionGraspView — 视觉抓取面板（核心页面）
 *
 * 替代旧版: vision_grasp_panel.html + vision_grasp_panel.js (649行) + 12个子模块
 *
 * 页面结构:
 *   ┌──────────────────────────────────────────────────────────────────┐
 *   │  顶部工具栏: 模式切换 + 连接状态                                   │
 *   ├──────────┬───────────────────────────────────────────────────────┤
 *   │ 左栏     │ 右栏                                                  │
 *   │ 3D 模型  │ GraspControls (抓取控制按钮)                           │
 *   │ 相机/结果 │ ToolSwapBar  (工具快换)                               │
 *   │          │ 调试 XYZ 移动                                         │
 *   │          │ 服务日志                                              │
 *   ├──────────┴───────────────────────────────────────────────────────┤
 *   │  底部（可折叠）: 关节曲线 + 末端位姿 + VPE/AI 状态               │
 *   └──────────────────────────────────────────────────────────────────┘
 *
 * 与旧版行为对齐的关键点:
 *   - 末端位姿缓存 (robotPoseCache): 数据空时不闪回默认值
 *   - AI位姿: 富文本HTML格式化 (pose_card格式: 位姿section + 四元数section)
 *   - AI模式: 左栏相机快照 + 结果图同步刷新
 *   - ivg_display 回退: RobotStatus.ivg_display 字段优先显示
 *   - 工件模式"停止"按钮: 调 SetBool(false) 停止循环
 */
import { useRos } from '@/composables/ros/useRos'
import { useMJPEGStream } from '@/composables/api/useMJPEGStream'
import { useJointChart } from '@/composables/viz/useJointChart'
import { useProjectionOverlay } from '@/composables/viz/useProjectionOverlay'
import { useRosService } from '@/composables/ros/useRosService'
import { useDashboardSettings } from '@/composables/settings/useDashboardSettings'
import { ivgQuatNormalize } from '@/lib/tf_math'
import type { Quat } from '@/lib/tf_math'
import { canonicalRosTopic } from '@/lib/utils'
import Robot3dViewer from '@/components/ivg/Robot3dViewer.vue'
import ToolSwapBar from '@/components/grasp/ToolSwapBar.vue'
import {
  ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE,
  JOINT_STATES_TOPIC, JOINT_STATES_TYPE,
  TF_TOPIC, TF_STATIC_TOPIC, TF_TYPE,
} from '@/constants/ros'

const { isConnected, connect, subscribe, unsubscribeAll, onRosJson, onControlJson, callService } = useRos()
const { callSetBool } = useRosService()
const settings = useDashboardSettings()

const robotStatusTopic = computed(() => settings.rosName('topic-robot', ROBOT_STATUS_TOPIC))
const jointStatesTopic = computed(() => settings.rosName('topic-joints', JOINT_STATES_TOPIC))
const tfTopic = computed(() => settings.rosName('topic-tf', TF_TOPIC))
const tfStaticTopic = computed(() => settings.rosName('topic-tf-static', TF_STATIC_TOPIC))
const vpeStatusTopic = computed(() => settings.rosName('topic-vpe-status', '/system_status'))
const graspPosesTopic = computed(() => settings.rosName('topic-grasp-poses', '/grasp_poses_base'))
const resultTopic = computed(() => settings.optionalRosName('topic-result', ''))
const toolStatusTopic = computed(() => settings.rosName('topic-tool-status', '/tool_changer_status'))
const urdfParam = computed(() => settings.raw('urdf-param', '/robot_state_publisher:robot_description'))
const fixedFrame = computed(() => settings.raw('tf-fixed-frame', 'base_link'))
const loopGraspService = computed(() => settings.rosName('svc-loop-grasp-control', '/loop_grasp_control'))
const graspnetCaptureService = computed(() => settings.rosName('svc-graspnet-capture', '/graspnet_capture_control'))
const publishGraspsLoopService = computed(() => settings.rosName('svc-publish-grasps-loop', '/publish_grasps_worker_loop_control'))
const executeGraspService = computed(() => settings.rosName('svc-execute-single-grasp', '/execute_single_grasp'))
const debugMoveService = computed(() => settings.rosName('svc-debug-move-to-xyz', '/debug/move_to_xyz'))

function sameTopic(topic: string, expected: string): boolean {
  return canonicalRosTopic(topic) === canonicalRosTopic(expected)
}

// ═══════════════════════ 抓取模式 ═══════════════════════

const graspMode = ref<'workpiece' | 'graspnet'>('workpiece')
const objectId = ref('')

// ═══════════════════════ 连接状态 ═══════════════════════

const connStatus = ref('正在连接…')
const connOk = ref(false)

// ═══════════════════════ 末端位姿 (与旧版 pose_card.js 格式对齐) ═══════════════════════

const robotPose = reactive({
  x: '—', y: '—', z: '—',
  qx: '—', qy: '—', qz: '—', qw: '—',
  roll: '—', pitch: '—', yaw: '—',
})

/** 缓存上次有效位姿 HTML，数据空时不闪回默认值（与旧版 robotPoseCache 行为一致） */
const robotPoseCache = ref('')

/** ivg_display 字段回退显示 */
const ivgDisplay = ref('')

// ═══════════════════════ 关节角 ═══════════════════════

const jointAngles = ref<number[]>([])
const jointNames = ref<string[]>([])

// ═══════════════════════ VPE / AI 状态 ═══════════════════════

const vpeStatus = ref('')
const graspPoseHtml = ref('')
const graspPoseRaw = ref<any>(null)

// ═══════════════════════ 相机 ═══════════════════════

const colorTopic = computed(() => settings.rosName('topic-color', '/camera/color/image_raw'))
const { cameraStreamUrl, cameraSnapshotUrl } = useMJPEGStream(colorTopic)
const { cameraStreamUrl: resultStreamUrl } = useMJPEGStream(resultTopic)

/** 从 color 话题动态推导 camera_info 话题名 */
function buildCameraInfoTopic(topic: string): string {
  const t = (topic || '/camera/color/image_raw').trim()
  if (/\/camera_info$/.test(t)) return t
  if (/\/image(_raw|_color)?$/.test(t)) return t.replace(/\/image(_raw|_color)?$/, '/camera_info')
  return `${t.replace(/\/+$/, '')}/camera_info`
}

/** 四元数 → RPY (欧拉角 ZYX，度) */
function quatToRpyDeg(q: Quat): { roll: number; pitch: number; yaw: number } | null {
  const nq = ivgQuatNormalize(q)
  const sqx = nq.x * nq.x, sqy = nq.y * nq.y, sqz = nq.z * nq.z, sqw = nq.w * nq.w
  const eulerX = Math.atan2(2 * (nq.w * nq.x + nq.y * nq.z), sqw + sqz - sqx - sqy)
  const sinp = 2 * (nq.w * nq.y - nq.z * nq.x)
  const eulerY = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp)
  const eulerZ = Math.atan2(2 * (nq.w * nq.z + nq.x * nq.y), sqw + sqx - sqy - sqz)
  const radToDeg = (rad: number) => rad * (180 / Math.PI)
  return { roll: radToDeg(eulerX), pitch: radToDeg(eulerY), yaw: radToDeg(eulerZ) }
}

/** 格式化为旧版 pose_card 双section HTML */
function formatPoseBlockHtml(poseXyz: { x: string; y: string; z: string }, quat: { qx: string; qy: string; qz: string; qw: string }, rpyStr: { roll: string; pitch: string; yaw: string }): string {
  return `<div class="pose-card__body">
<section class="pose-card__section"><h3 class="pose-card__section-title">位姿 m · °</h3>
<div class="pose-card__triple"><div class="pose-card__pill"><span>X</span><span>${poseXyz.x}</span></div><div class="pose-card__pill"><span>Y</span><span>${poseXyz.y}</span></div><div class="pose-card__pill"><span>Z</span><span>${poseXyz.z}</span></div></div>
<div class="pose-card__triple"><div class="pose-card__pill"><span>R</span><span>${rpyStr.roll}</span></div><div class="pose-card__pill"><span>P</span><span>${rpyStr.pitch}</span></div><div class="pose-card__pill"><span>Y</span><span>${rpyStr.yaw}</span></div></div>
</section>
<section class="pose-card__section"><h3 class="pose-card__section-title">四元数 x y z w</h3>
<div class="pose-card__quad"><div class="pose-card__pill"><span>X</span><span>${quat.qx}</span></div><div class="pose-card__pill"><span>Y</span><span>${quat.qy}</span></div><div class="pose-card__pill"><span>Z</span><span>${quat.qz}</span></div><div class="pose-card__pill"><span>W</span><span>${quat.qw}</span></div></div>
</section></div>`
}

/** 防抖刷新 AI 模式相机快照（snapshot 失败时回退到 MJPEG 流） */
let graspSnapTimer: ReturnType<typeof setTimeout> | null = null
function scheduleGraspSnapshotRefresh(): void {
  if (graspMode.value !== 'graspnet') return
  if (graspSnapTimer) clearTimeout(graspSnapTimer)
  graspSnapTimer = setTimeout(() => {
    const ts = Date.now()

    // 左栏相机快照（旧版 cam-mjpeg，用于底部投影底图）
    const camImg = document.getElementById('cam-mjpeg-img') as HTMLImageElement | null
    if (camImg) {
      const url = cameraSnapshotUrl(`vision_color_grasp_${ts}`)
      const fallbackUrl = cameraStreamUrl()
      const onErr = () => { camImg.src = fallbackUrl }
      camImg.addEventListener('error', onErr, { once: true })
      camImg.src = url
    }

    // 结果图快照（投影叠加底图）
    const resultImg = document.getElementById('result-mjpeg-img') as HTMLImageElement | null
    if (resultImg && !resultImg.hidden) {
      const url = cameraSnapshotUrl(`vision_projection_grasp_${ts}`)
      const fallbackUrl = cameraStreamUrl()
      const onErr = () => { resultImg.src = fallbackUrl }
      resultImg.addEventListener('error', onErr, { once: true })
      resultImg.src = url
    }
  }, 400)
}

// ═══════════════════════ 关节曲线 ═══════════════════════

const { canvasRef: chartCanvasRef, legendRef: chartLegendRef, pushSample: pushJointSample, reset: resetJointChart, observeResize: observeJointChart } = useJointChart()

// ═══════════════════════ 投影叠加 ═══════════════════════

const { canvasRef: projCanvasRef, stackRef: projStackRef, setCameraInfo, setGraspMsg, ingestTfMessage, scheduleDraw: scheduleProjDraw, reset: resetProj } = useProjectionOverlay()

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

// ═══════════════════════ 服务日志 ═══════════════════════

const svcLog = ref('等待操作…')

function log(msg: string): void {
  svcLog.value = `${new Date().toLocaleTimeString()} ${msg}`
  if (msg.includes('错误') || msg.includes('失败')) {
    console.warn(`[vision] ${new Date().toLocaleTimeString()} ${msg}`)
  } else {
    console.log(`[vision] ${new Date().toLocaleTimeString()} ${msg}`)
  }
}

// ═══════════════════════ 话题消息处理 ═══════════════════════

// 机器人状态 → 解析末端位姿（含 robotPoseCache + ivg_display 回退）
onRosJson(null, (msg: any, topic: string) => {
  if (!sameTopic(topic, robotStatusTopic.value)) return
  if (!msg) return

  // ivg_display 回退（旧版 pose_card.js:128）
  if (msg.ivg_display != null && String(msg.ivg_display).trim()) {
    ivgDisplay.value = String(msg.ivg_display).trim()
  } else {
    ivgDisplay.value = ''
  }

  const p = msg.cartesian_position_xyz || {}
  const o = msg.cartesian_position?.orientation || {}
  const r = msg.cartesian_rpy || {}

  // 检查是否有有效位姿数据
  const hasPos = [p.x, p.y, p.z].some((v: any) => isFinite(Number(v)))
  const hasOri = [o.x, o.y, o.z, o.w].some((v: any) => isFinite(Number(v)))

  if (!hasPos && !hasOri) {
    // 无位姿数据 → 保留缓存值，不闪回默认
    return
  }

  robotPose.x = p.x != null && isFinite(p.x) ? Number(p.x).toFixed(4) : '—'
  robotPose.y = p.y != null && isFinite(p.y) ? Number(p.y).toFixed(4) : '—'
  robotPose.z = p.z != null && isFinite(p.z) ? Number(p.z).toFixed(4) : '—'
  robotPose.qx = o.x != null && isFinite(o.x) ? Number(o.x).toFixed(4) : '—'
  robotPose.qy = o.y != null && isFinite(o.y) ? Number(o.y).toFixed(4) : '—'
  robotPose.qz = o.z != null && isFinite(o.z) ? Number(o.z).toFixed(4) : '—'
  robotPose.qw = o.w != null && isFinite(o.w) ? Number(o.w).toFixed(4) : '—'

  // RPY: 优先用 cartesian_rpy 字段，否则从四元数计算
  const rr = Number(r.x), rp = Number(r.y), ry = Number(r.z)
  if (isFinite(rr) && isFinite(rp) && isFinite(ry)) {
    robotPose.roll = (rr * 180 / Math.PI).toFixed(1) + '°'
    robotPose.pitch = (rp * 180 / Math.PI).toFixed(1) + '°'
    robotPose.yaw = (ry * 180 / Math.PI).toFixed(1) + '°'
  } else if (hasOri) {
    const oriQuat = { x: Number(o.x) || 0, y: Number(o.y) || 0, z: Number(o.z) || 0, w: Number(o.w) || 1 }
    const rpyDeg = quatToRpyDeg(oriQuat)
    if (rpyDeg) {
      robotPose.roll = rpyDeg.roll.toFixed(1) + '°'
      robotPose.pitch = rpyDeg.pitch.toFixed(1) + '°'
      robotPose.yaw = rpyDeg.yaw.toFixed(1) + '°'
    }
  }

  // 缓存有效位姿
  if (hasPos && hasOri) {
    robotPoseCache.value = formatPoseBlockHtml(
      { x: robotPose.x, y: robotPose.y, z: robotPose.z },
      { qx: robotPose.qx, qy: robotPose.qy, qz: robotPose.qz, qw: robotPose.qw },
      { roll: robotPose.roll, pitch: robotPose.pitch, yaw: robotPose.yaw }
    )
  }
})

// 关节状态 → 曲线图 + 数值
onRosJson(null, (msg: any, topic: string) => {
  if (!sameTopic(topic, jointStatesTopic.value)) return
  if (!msg) return
  const names: string[] = msg.name ?? []
  const pos: number[] = msg.position ?? []
  jointNames.value = names
  jointAngles.value = pos.map((v: number) => Number(v))
  pushJointSample(names, pos)
})

// TF → 投影叠加 + 3D 更新
onRosJson(null, (msg: any, topic: string) => {
  if (sameTopic(topic, tfTopic.value) || sameTopic(topic, tfStaticTopic.value)) ingestTfMessage(msg)
})

// VPE 状态（工件模式）
onRosJson(null, (msg: any, topic: string) => {
  if (!sameTopic(topic, vpeStatusTopic.value)) return
  vpeStatus.value = msg?.data ? String(msg.data) : ''
})

// 相机信息 → 投影用（动态推导话题名）
onRosJson(null, (msg, topic) => {
  if (topic === buildCameraInfoTopic(colorTopic.value)) {
    setCameraInfo(msg)
  }
})

// AI 抓取位姿 → 投影 + 快照 + 格式化 HTML
onRosJson(null, (msg: any, topic: string) => {
  if (!sameTopic(topic, graspPosesTopic.value)) return
  graspPoseRaw.value = msg
  setGraspMsg(msg)
  scheduleGraspSnapshotRefresh()

  // 格式化为旧版 pose_card 双section HTML
  if (msg?.poses?.length) {
    const pose = msg.poses[0]
    const px = pose?.position?.x, py = pose?.position?.y, pz = pose?.position?.z
    const qx = pose?.orientation?.x, qy = pose?.orientation?.y, qz = pose?.orientation?.z, qw = pose?.orientation?.w

    if (isFinite(Number(px)) && isFinite(Number(py)) && isFinite(Number(pz))
      && isFinite(Number(qx)) && isFinite(Number(qy)) && isFinite(Number(qz)) && isFinite(Number(qw))) {
      const rpyDeg = quatToRpyDeg({ x: Number(qx), y: Number(qy), z: Number(qz), w: Number(qw) || 1 })
      graspPoseHtml.value = formatPoseBlockHtml(
        { x: Number(px).toFixed(4), y: Number(py).toFixed(4), z: Number(pz).toFixed(4) },
        { qx: Number(qx).toFixed(4), qy: Number(qy).toFixed(4), qz: Number(qz).toFixed(4), qw: Number(qw).toFixed(4) },
        { roll: rpyDeg ? rpyDeg.roll.toFixed(1) + '°' : '—', pitch: rpyDeg ? rpyDeg.pitch.toFixed(1) + '°' : '—', yaw: rpyDeg ? rpyDeg.yaw.toFixed(1) + '°' : '—' }
      )
    }
  }
})

// ═══════════════════════ 连接管理 ═══════════════════════

onControlJson((c) => {
  if (c.op === 'connection') { connStatus.value = '已连接'; connOk.value = true; setupSubs() }
  if (c.op === 'close') { connStatus.value = c.message || '已断开，重连中…'; connOk.value = false; unsubscribeAll(); resetAll() }
  if (c.op === 'error') { connStatus.value = '通信错误'; connOk.value = false }
})

function setupSubs(): void {
  if (!isConnected()) return
  subscribe(robotStatusTopic.value, settings.topicType('topic-robot', ROBOT_STATUS_TYPE), 50)
  subscribe(jointStatesTopic.value, settings.topicType('topic-joints', JOINT_STATES_TYPE), 30)
  subscribe(tfTopic.value, settings.topicType('topic-tf', TF_TYPE), 30)
  subscribe(tfStaticTopic.value, settings.topicType('topic-tf-static', TF_TYPE), 1)
  subscribe(vpeStatusTopic.value, settings.topicType('topic-vpe-status', 'std_msgs/msg/String'), 10)
  subscribe(buildCameraInfoTopic(colorTopic.value), 'sensor_msgs/msg/CameraInfo', 5)
  subscribe(graspPosesTopic.value, settings.topicType('topic-grasp-poses', 'geometry_msgs/msg/PoseArray'), 15)
}

function resetAll(): void {
  resetJointChart()
  resetProj()
  graspPoseHtml.value = ''
  graspPoseRaw.value = null
  vpeStatus.value = ''
  ivgDisplay.value = ''
}

// ═══════════════════════ 抓取操作 ═══════════════════════

/** 工件模式: 执行单次抓取 */
async function doSingleGrasp(): Promise<void> {
  log('执行单次抓取…')
  try {
    const useVisual = graspMode.value === 'workpiece'
    const r: any = await callService(executeGraspService.value, settings.serviceType('svc-execute-single-grasp', 'ivg_interfaces/srv/ExecuteGraspPose'), {
      object_id: objectId.value, use_visual_estimation: useVisual,
    })
    log(`✓ ${r?.success ? '成功' : '失败'} ${r?.message || ''}`)
  } catch (e: any) { log(`✗ 错误: ${e}`) }
}

/** 工件模式: 停止循环 */
async function doStop(): Promise<void> {
  try { await callSetBool(loopGraspService.value, false); log('已停止') }
  catch (e: any) { log(`✗ 错误: ${e}`) }
}

/** 工件模式: 启动循环 */
async function doLoopGrasp(start: boolean): Promise<void> {
  log(start ? '启动循环抓取…' : '停止循环')
  try { await callSetBool(loopGraspService.value, start); log(start ? '✓ 循环已启动' : '✓ 已停止') }
  catch (e: any) { log(`✗ 错误: ${e}`) }
}

/** AI 模式: 采集开关 */
async function doCapture(start: boolean): Promise<void> {
  try { await callSetBool(graspnetCaptureService.value, start); log(start ? '✓ 采集开始' : '✓ 采集停止') }
  catch (e: any) { log(`✗ 错误: ${e}`) }
}

/** AI 模式: 循环抓取开关 */
async function doGraspnetLoop(start: boolean): Promise<void> {
  try { await callSetBool(publishGraspsLoopService.value, start); log(start ? '✓ 循环开启' : '✓ 循环关闭') }
  catch (e: any) { log(`✗ 错误: ${e}`) }
}

// ═══════════════════════ 调试 XYZ ═══════════════════════

const dbgXyz = reactive({ x: 0.3741, y: 0.3039, z: 0.4755, vel: 0.3, acc: 0.2 })

async function doDebugMoveXyz(): Promise<void> {
  const { x, y, z, vel, acc } = dbgXyz
  log(`Move XYZ → (${x.toFixed(3)}, ${y.toFixed(3)}, ${z.toFixed(3)}) v=${vel} a=${acc}`)
  try {
    const r: any = await callService(debugMoveService.value, settings.serviceType('svc-debug-move-to-xyz', 'ivg_interfaces/srv/MoveToPose'), {
      target_pose: { position: { x, y, z }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
      target_joints: [0, 0, 0, 0, 0, 0],
      use_joints: false,
      velocity_factor: vel,
      acceleration_factor: acc,
    })
    log(`✓ ${r?.success ? '成功' : '失败'} ${r?.message || ''}`)
  } catch (e: any) {
    log(`✗ 错误: ${e}`)
  }
}

// 页面加载后自动连接（仅当未连接时，避免关闭已有连接）
onMounted(() => {
  observeJointChart()
  settings.loadSettings().then(() => { if (isConnected()) setupSubs() }).catch(() => {})
  if (!isConnected()) connect().catch(() => {})
})

watch(() => settings.version.value, () => { if (isConnected()) setupSubs() })

// 模式切换时同步投影
watch(graspMode, () => {
  if (graspMode.value === 'graspnet') {
    scheduleGraspSnapshotRefresh()
  }
  scheduleProjDraw()
})
</script>

<template>
  <div class="ivg-run-page w-full max-w-none px-2 sm:px-3 py-2 overflow-x-hidden">
    <!-- ═══ 顶部工具栏 ═══ -->
    <div class="ivg-topbar flex items-center justify-between mb-2 bg-white rounded-lg border border-slate-200 px-3 py-2">
      <div class="flex items-center gap-3">
        <h1 class="text-lg font-bold text-slate-900">视觉抓取面板</h1>
        <el-radio-group v-model="graspMode" size="small">
          <el-radio-button value="workpiece">工件（视觉估计）</el-radio-button>
          <el-radio-button value="graspnet">AI大模型抓取</el-radio-button>
        </el-radio-group>
      </div>
      <div class="flex items-center gap-2">
        <a href="/settings" target="_blank" rel="noopener" class="text-xs text-blue-500 hover:text-blue-600 no-underline">话题与服务设置</a>
        <span class="text-xs" :class="connOk ? 'text-green-600' : connStatus === '正在连接…' ? 'text-slate-400' : 'text-red-500'">
          {{ connStatus }}
        </span>
      </div>
    </div>

    <!-- ═══ 主内容区: 左(3D+相机) / 右(控制面板) ═══ -->
    <div class="ivg-main-grid grid grid-cols-1 min-[921px]:grid-cols-[minmax(0,1fr)_minmax(15rem,22vw)] gap-2 min-h-0">
      <!-- 左栏: 3D 模型 + 结果视图（旧版同排双面板） -->
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

        <!-- 结果视图（相机画面 + 投影叠加层） -->
        <section class="bg-white rounded-lg border border-slate-200 p-3 min-w-0 overflow-hidden" aria-label="识别结果视图">
          <h2 class="text-xs font-bold text-slate-500 uppercase mb-2">
            {{ graspMode === 'graspnet' ? 'AI 抓取位姿投影视图' : '识别结果图像' }}
          </h2>
          <div v-show="graspMode === 'workpiece'" class="w-full">
            <img
              v-if="isConnected() && resultTopic"
              :src="resultStreamUrl()"
              class="w-full h-[clamp(260px,36vh,400px)] rounded object-cover bg-slate-100"
              alt="识别结果图像"
            />
            <div v-else-if="isConnected()" class="w-full h-[clamp(260px,36vh,400px)] bg-slate-50 rounded" aria-hidden="true" />
            <div v-else class="w-full h-[300px] flex items-center justify-center text-slate-400 text-sm bg-slate-50 rounded">
              等待 rosbridge 连接…
            </div>
          </div>
          <div v-show="graspMode === 'graspnet'" class="w-full">
            <div ref="projStackRef" class="relative w-full h-[clamp(260px,36vh,400px)] rounded overflow-hidden bg-slate-100">
              <img
                v-if="isConnected()"
                id="result-mjpeg-img"
                :src="cameraStreamUrl()"
                class="w-full h-full object-cover"
                alt="AI 抓取投影底图"
              />
              <div v-else class="w-full h-full flex items-center justify-center text-slate-400 text-sm">
                等待 rosbridge 连接…
              </div>
              <canvas
                ref="projCanvasRef"
                id="result-overlay-canvas"
                class="absolute inset-0 w-full h-full pointer-events-none"
                hidden
              />
            </div>
          </div>
        </section>
      </div>

      <!-- 右栏: 控制面板 -->
      <div class="min-w-0 space-y-2">
        <!-- 工件编号输入（仅工件模式） -->
        <div v-show="graspMode === 'workpiece'" class="bg-white rounded-lg border border-slate-200 p-4">
          <label class="text-xs text-slate-500" for="object-id">目标工件编号</label>
          <input
            id="object-id"
            v-model="objectId"
            class="w-full text-sm border border-slate-300 rounded px-2 py-1.5 mt-1"
            placeholder="与现场 object_id 一致"
            autocomplete="off"
          />
        </div>

        <!-- 抓取控制按钮 -->
        <div class="bg-white rounded-lg border border-slate-200 p-4">
          <!-- 工件模式 -->
          <template v-if="graspMode === 'workpiece'">
            <h3 class="text-sm font-bold text-slate-700 mb-3">工件抓取</h3>
            <div class="flex flex-wrap gap-1.5">
              <el-button size="small" type="primary" @click="doSingleGrasp">执行单次抓取</el-button>
              <el-button size="small" type="danger" @click="doStop">停止</el-button>
              <el-button size="small" type="success" @click="doLoopGrasp(true)">启动循环</el-button>
              <el-button size="small" type="danger" @click="doLoopGrasp(false)">停循环</el-button>
            </div>
          </template>

          <!-- AI 大模型模式 -->
          <template v-else>
            <h3 class="text-sm font-bold text-slate-700 mb-3">AI 大模型抓取</h3>
            <div class="flex flex-wrap gap-1.5">
              <el-button size="small" type="primary" @click="doCapture(true)">开始采集</el-button>
              <el-button size="small" type="danger" @click="doCapture(false)">停止采集</el-button>
              <el-button size="small" type="success" @click="doGraspnetLoop(true)">循环抓取开</el-button>
              <el-button size="small" type="warning" @click="doGraspnetLoop(false)">循环抓取关</el-button>
            </div>
            <!-- AI 抓取位姿显示 -->
            <div v-if="graspPoseHtml" class="mt-3 p-2 bg-slate-50 rounded text-xs" v-html="graspPoseHtml" />
          </template>
        </div>

        <!-- 工具快换 -->
        <ToolSwapBar @log="log" />

        <!-- 调试 XYZ 移动 -->
        <div class="bg-white rounded-lg border border-slate-200 p-4">
          <h3 class="text-sm font-bold text-slate-700 mb-2">调试 — 笛卡尔直线运动</h3>
          <div class="grid grid-cols-3 gap-1 mb-2 text-xs">
            <label>X:<input v-model.number="dbgXyz.x" step="0.01" type="number" class="w-full border border-slate-300 rounded px-1 py-0.5 text-xs" /></label>
            <label>Y:<input v-model.number="dbgXyz.y" step="0.01" type="number" class="w-full border border-slate-300 rounded px-1 py-0.5 text-xs" /></label>
            <label>Z:<input v-model.number="dbgXyz.z" step="0.01" type="number" class="w-full border border-slate-300 rounded px-1 py-0.5 text-xs" /></label>
            <label>V:<input v-model.number="dbgXyz.vel" step="0.05" min="0.05" max="1.0" type="number" class="w-full border border-slate-300 rounded px-1 py-0.5 text-xs" /></label>
            <label>A:<input v-model.number="dbgXyz.acc" step="0.05" min="0.05" max="1.0" type="number" class="w-full border border-slate-300 rounded px-1 py-0.5 text-xs" /></label>
            <el-button size="small" class="mt-1" @click="doDebugMoveXyz">Move XYZ</el-button>
          </div>
        </div>

        <!-- 服务调用日志 -->
        <div class="bg-slate-900 rounded-lg p-3 text-xs font-mono text-green-400 min-h-[3em] whitespace-pre-wrap leading-relaxed">
          {{ svcLog }}
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

        <!-- 位姿读数 -->
        <div class="space-y-3">
          <!-- 机械臂末端位姿 -->
          <div class="bg-white rounded-lg border border-slate-200 p-4">
            <h2 class="text-sm font-bold text-slate-500 uppercase mb-3">机械臂末端位姿</h2>
            <!-- 位姿 HTML 显示（优先缓存，其次实时，最后 ivg_display） -->
            <div v-if="robotPoseCache" class="text-xs" v-html="robotPoseCache" />
            <div v-else-if="ivgDisplay" class="text-xs text-slate-600 whitespace-pre-wrap">{{ ivgDisplay }}</div>
            <div v-else class="grid grid-cols-4 gap-2 text-xs">
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

          <!-- VPE 状态（工件模式） -->
          <div v-show="graspMode === 'workpiece'" class="bg-white rounded-lg border border-slate-200 p-4">
            <h2 class="text-sm font-bold text-slate-500 uppercase mb-2">视觉位姿估计状态</h2>
            <pre class="text-xs text-slate-600 whitespace-pre-wrap">{{ vpeStatus || '等待数据…' }}</pre>
          </div>

          <!-- AI 抓取位姿（AI 模式） -->
          <div v-show="graspMode === 'graspnet'" class="bg-white rounded-lg border border-slate-200 p-4">
            <h2 class="text-sm font-bold text-slate-500 uppercase mb-2">AI大模型最终抓取位姿</h2>
            <div v-if="graspPoseHtml" class="text-xs" v-html="graspPoseHtml" />
            <div v-else class="text-xs text-slate-600">等待 AI 抓取位姿…</div>
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

    <!-- 隐藏的 AI 模式左栏底图（旧版 cam-mjpeg，用于快照） -->
    <div hidden aria-hidden="true">
      <img id="cam-mjpeg-img" alt="" />
    </div>
  </div>
</template>
