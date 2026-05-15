/**
 * useProjectionOverlay — 抓取位姿 Canvas 2D 投影叠加层 composable
 *
 * 替代旧版: vision_grasp/projection_overlay.js (322行)
 *
 * 功能:
 *   - 在相机图像上叠加绘制抓取姿态投影
 *   - 支持 TF 坐标系链转换（相机→抓取位姿）
 *   - 绘制夹爪形状（手指/腕部） + 标签
 *
 * 依赖: tf_math.ts（四元数/变换数学，已存在）
 *
 * 用法:
 *   const { canvasRef, stackRef, setCameraInfo, setGraspMsg, ingestTfMessage, scheduleDraw } = useProjectionOverlay()
 */
import {
  normalizeFrameId, ivgCloneTransform, ivgQuatNormalize,
  ivgRotateVectorByQuat, ivgComposeTransforms,
  ivgFindRelativeTransform, ivgIdentityTransform,
  type Transform, type Vec3, type Quat
} from '@/lib/tf_math'

function ivgVec3(x = 0, y = 0, z = 0): Vec3 { return { x, y, z } }
function ivgVec3Add(a: Vec3, b: Vec3): Vec3 { return { x: (a.x || 0) + (b.x || 0), y: (a.y || 0) + (b.y || 0), z: (a.z || 0) + (b.z || 0) } }
function ivgApplyTransformPoint(tf: Transform, point: Vec3): Vec3 {
  const t = ivgCloneTransform(tf)
  const q = ivgQuatNormalize(t.rotation as unknown as Quat)
  const p = ivgVec3(point.x || 0, point.y || 0, point.z || 0)
  return ivgVec3Add(ivgRotateVectorByQuat(p, q), t.translation)
}

interface TfEdge { parent: string; transform: Transform }

export function useProjectionOverlay() {
  const canvasRef = ref<HTMLCanvasElement | null>(null)
  const stackRef = ref<HTMLElement | null>(null)

  const state = {
    cameraInfo: null as any,
    cameraFrame: '',
    graspMsg: null as any,
    tfEdges: {} as Record<string, TfEdge>,
    drawRaf: null as number | null,
  }

  function clearCanvas(): void {
    const canvas = canvasRef.value
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    ctx.clearRect(0, 0, canvas.width || 0, canvas.height || 0)
    canvas.hidden = true
  }

  function syncCanvasSize(): { ctx: CanvasRenderingContext2D; width: number; height: number } | null {
    const canvas = canvasRef.value
    const stack = stackRef.value
    if (!canvas || !stack) return null
    const rect = stack.getBoundingClientRect()
    const width = Math.max(1, Math.round(rect.width))
    const height = Math.max(1, Math.round(rect.height))
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width; canvas.height = height
    }
    const ctx = canvas.getContext('2d')
    return ctx ? { ctx, width, height } : null
  }

  function projectPointToImage(point: Vec3, info: any, width: number, height: number): { x: number; y: number; depth: number } | null {
    if (!info?.k || info.k.length < 9) return null
    if (!isFinite(point.x) || !isFinite(point.y) || !isFinite(point.z) || point.z <= 0.01) return null
    const fx = Number(info.k[0]) || 0
    const fy = Number(info.k[4]) || 0
    const cx = Number(info.k[2]) || 0
    const cy = Number(info.k[5]) || 0
    if (!(fx > 0) || !(fy > 0)) return null
    const u = fx * (point.x / point.z) + cx
    const v = fy * (point.y / point.z) + cy
    const iw = Number(info.width) || width
    const ih = Number(info.height) || height
    return { x: (u / iw) * width, y: (v / ih) * height, depth: point.z }
  }

  function drawSegment(ctx: CanvasRenderingContext2D, a: { x: number; y: number } | null, b: { x: number; y: number } | null, style: any): void {
    if (!a || !b) return
    ctx.save()
    if (style?.dash) ctx.setLineDash(style.dash)
    ctx.strokeStyle = style?.outlineColor || 'rgba(15,23,42,0.92)'
    ctx.lineWidth = (style?.lineWidth || 2) + (style?.outlineWidth || 2.5)
    ctx.lineCap = 'round'; ctx.lineJoin = 'round'
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke()
    ctx.setLineDash([])
    ctx.strokeStyle = style?.color || '#ffffff'
    ctx.lineWidth = style?.lineWidth || 2
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke()
    ctx.restore()
  }

  function roundedRect(ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number, r: number): void {
    const rr = Math.max(0, Math.min(r, Math.min(w, h) / 2))
    ctx.beginPath()
    ctx.moveTo(x + rr, y); ctx.lineTo(x + w - rr, y)
    ctx.quadraticCurveTo(x + w, y, x + w, y + rr); ctx.lineTo(x + w, y + h - rr)
    ctx.quadraticCurveTo(x + w, y + h, x + w - rr, y + h); ctx.lineTo(x + rr, y + h)
    ctx.quadraticCurveTo(x, y + h, x, y + h - rr); ctx.lineTo(x, y + rr)
    ctx.quadraticCurveTo(x, y, x + rr, y); ctx.closePath()
  }

  function drawBadge(ctx: CanvasRenderingContext2D, x: number, y: number, text: string, opts?: any): void {
    const padX = 8; const boxH = 22
    ctx.save(); ctx.font = opts?.font || 'bold 12px sans-serif'
    const boxW = Math.ceil(ctx.measureText(text).width) + padX * 2
    ctx.fillStyle = opts?.fillStyle || 'rgba(15,23,42,0.84)'
    ctx.strokeStyle = opts?.strokeStyle || 'rgba(255,255,255,0.25)'
    ctx.lineWidth = 1.5
    roundedRect(ctx, x, y, boxW, boxH, 9)
    ctx.fill(); ctx.stroke()
    ctx.fillStyle = opts?.textColor || '#f8fafc'
    ctx.textBaseline = 'middle'
    ctx.fillText(text, x + padX, y + boxH / 2)
    ctx.restore()
  }

  function drawProjectedPose(ctx: CanvasRenderingContext2D, width: number, height: number, poseCam: { position: Vec3; orientation: Quat }, rank: number): boolean {
    const origin3 = ivgVec3(poseCam.position.x || 0, poseCam.position.y || 0, poseCam.position.z || 0)
    const q = ivgQuatNormalize(poseCam.orientation)
    const info = state.cameraInfo
    const jawWidth = rank === 0 ? 0.09 : 0.075
    const fingerLength = rank === 0 ? 0.075 : 0.06
    const wristLength = rank === 0 ? 0.05 : 0.04
    const bridgeOffset = 0.012

    const leftRoot3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(jawWidth / 2, 0, bridgeOffset), q))
    const rightRoot3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(-jawWidth / 2, 0, bridgeOffset), q))
    const leftTip3 = ivgVec3Add(leftRoot3, ivgRotateVectorByQuat(ivgVec3(0, 0, fingerLength), q))
    const rightTip3 = ivgVec3Add(rightRoot3, ivgRotateVectorByQuat(ivgVec3(0, 0, fingerLength), q))
    const wrist3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(0, 0, -wristLength), q))
    const approach3 = ivgVec3Add(origin3, ivgRotateVectorByQuat(ivgVec3(0, 0, fingerLength + 0.035), q))

    const origin2 = projectPointToImage(origin3, info, width, height)
    if (!origin2) return false
    const leftRoot2 = projectPointToImage(leftRoot3, info, width, height)
    const rightRoot2 = projectPointToImage(rightRoot3, info, width, height)
    const leftTip2 = projectPointToImage(leftTip3, info, width, height)
    const rightTip2 = projectPointToImage(rightTip3, info, width, height)
    const wrist2 = projectPointToImage(wrist3, info, width, height)
    const approach2 = projectPointToImage(approach3, info, width, height)

    const mainColor = rank === 0 ? '#f59e0b' : 'rgba(56,189,248,0.95)'
    const subColor = rank === 0 ? 'rgba(251,191,36,0.95)' : 'rgba(224,242,254,0.96)'
    const lineWidth = rank === 0 ? 3.8 : 2.6

    if (leftRoot2 && leftTip2) drawSegment(ctx, leftRoot2, leftTip2, { color: subColor, lineWidth, outlineWidth: 3 })
    if (rightRoot2 && rightTip2) drawSegment(ctx, rightRoot2, rightTip2, { color: subColor, lineWidth, outlineWidth: 3 })
    if (leftRoot2 && rightRoot2) drawSegment(ctx, leftRoot2, rightRoot2, { color: mainColor, lineWidth: lineWidth + 0.4, outlineWidth: 3.2 })
    if (wrist2) drawSegment(ctx, wrist2, origin2, { color: mainColor, lineWidth: lineWidth - 0.2, outlineWidth: 2.8 })
    if (approach2) drawSegment(ctx, origin2, approach2, { color: '#fde68a', lineWidth: rank === 0 ? 3 : 2.2, outlineWidth: 2.4, dash: [6, 5] })

    const radius = rank === 0 ? 8 : 5.5
    ctx.fillStyle = rank === 0 ? '#f59e0b' : '#38bdf8'
    ctx.strokeStyle = 'rgba(15,23,42,0.94)'
    ctx.lineWidth = rank === 0 ? 3 : 2.2
    ctx.beginPath(); ctx.arc(origin2.x, origin2.y, radius, 0, Math.PI * 2); ctx.fill(); ctx.stroke()

    drawBadge(ctx, origin2.x + radius + 6, origin2.y - radius - 18, rank === 0 ? '最终抓取' : `#${rank + 1}`, {
      fillStyle: rank === 0 ? 'rgba(245,158,11,0.92)' : 'rgba(15,23,42,0.78)',
      strokeStyle: rank === 0 ? 'rgba(255,255,255,0.4)' : 'rgba(148,163,184,0.35)',
      textColor: rank === 0 ? '#111827' : '#f8fafc',
      font: rank === 0 ? 'bold 12px sans-serif' : 'bold 11px sans-serif',
    })
    return true
  }

  function drawOverlay(): void {
    const size = syncCanvasSize()
    if (!size) { clearCanvas(); return }
    const { ctx, width, height } = size
    ctx.clearRect(0, 0, width, height)

    const info = state.cameraInfo
    if (!info) { clearCanvas(); return }

    // 构建投影帧候选列表
    const projFrames: string[] = []
    for (const f of [state.cameraFrame,
      info?.header?.frame_id,
      'camera_color_optical_frame', 'camera_color_frame',
      'camera_link', 'camera_frame']) {
      const nf = normalizeFrameId(f)
      if (nf && !projFrames.includes(nf)) projFrames.push(nf)
    }

    // 查找抓取位姿帧
    const graspFrames = Object.keys(state.tfEdges).filter(name => /^grasp_pose_\d+$/.test(name))
    graspFrames.sort((a, b) => Number(a.replace('grasp_pose_', '')) - Number(b.replace('grasp_pose_', '')))

    let drawn = 0
    if (graspFrames.length > 0) {
      let tfCamToGrasp: Transform | null = null
      for (const pf of projFrames) {
        tfCamToGrasp = ivgFindRelativeTransform(pf, graspFrames[0], state.tfEdges as any)
        if (tfCamToGrasp) break
      }
      if (tfCamToGrasp) {
        if (drawProjectedPose(ctx, width, height, { position: tfCamToGrasp.translation, orientation: tfCamToGrasp.rotation as unknown as Quat }, 0)) drawn = 1
      }
    }

    // 回退: 从 PoseArray 消息
    if (drawn === 0) {
      const gm = state.graspMsg
      const poseFrame = normalizeFrameId(gm?.header?.frame_id)
      if (gm?.poses?.length && poseFrame) {
        let baseToCam: Transform | null = null
        for (const pf of projFrames) {
          baseToCam = ivgFindRelativeTransform(poseFrame, pf, state.tfEdges as any)
          if (baseToCam) break
        }
        if (baseToCam) {
          const pose = gm.poses[0]
          if (pose?.position && pose?.orientation) {
            const originCam = ivgApplyTransformPoint(baseToCam, pose.position)
            const rotCam = ivgComposeTransforms(baseToCam, { translation: { x: 0, y: 0, z: 0 }, rotation: pose.orientation }).rotation
            if (drawProjectedPose(ctx, width, height, { position: originCam, orientation: rotCam as unknown as Quat }, 0)) drawn = 1
          }
        }
      }
    }

    if (drawn === 0) { clearCanvas(); return }
    const canvas = canvasRef.value
    if (canvas) canvas.hidden = false
  }

  function scheduleDraw(): void {
    if (state.drawRaf != null) return
    state.drawRaf = requestAnimationFrame(() => { state.drawRaf = null; drawOverlay() })
  }

  function reset(): void {
    state.cameraInfo = null; state.cameraFrame = ''
    state.graspMsg = null; state.tfEdges = {}
    scheduleDraw()
  }

  function setCameraInfo(msg: any): void {
    state.cameraInfo = msg
    state.cameraFrame = normalizeFrameId(msg?.header?.frame_id)
    scheduleDraw()
  }

  function setGraspMsg(msg: any): void {
    state.graspMsg = msg || null
    scheduleDraw()
  }

  function ingestTfMessage(msg: any): void {
    const arr = msg?.transforms ?? []
    for (const t of arr) {
      const child = normalizeFrameId(t?.child_frame_id)
      const parent = normalizeFrameId(t?.header?.frame_id)
      if (!child || !parent || !t.transform) continue
      state.tfEdges[child] = { parent, transform: ivgCloneTransform(t.transform) }
    }
    scheduleDraw()
  }

  onUnmounted(() => {
    if (state.drawRaf != null) cancelAnimationFrame(state.drawRaf)
  })

  return { canvasRef, stackRef, clearCanvas, scheduleDraw, reset, setCameraInfo, setGraspMsg, ingestTfMessage }
}
