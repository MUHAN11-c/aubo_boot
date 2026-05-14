// tf_math.ts — TF 四元数/变换/坐标系链数学库
// 从 view3d/tf_clients.js (212行) 搬移，函数签名完全不变
// 框架无关纯逻辑，供 projection_overlay 等模块使用

export interface Vec3 { x: number; y: number; z: number }
export interface Quat extends Vec3 { w: number }
export interface Transform { translation: Vec3; rotation: Quat }

export function normalizeFrameId(frame: string): string {
  return String(frame ?? '').trim().replace(/^\/+/, '')
}

export function ivgIdentityTransform(): Transform {
  return {
    translation: { x: 0, y: 0, z: 0 },
    rotation: { x: 0, y: 0, z: 0, w: 1 },
  }
}

export function ivgCloneTransform(tf: Partial<Transform>): Transform {
  const tr = tf.translation ?? { x: 0, y: 0, z: 0 }
  const rot = tf.rotation ?? { x: 0, y: 0, z: 0, w: 1 }
  return {
    translation: { x: Number(tr.x) || 0, y: Number(tr.y) || 0, z: Number(tr.z) || 0 },
    rotation: { x: Number(rot.x) || 0, y: Number(rot.y) || 0, z: Number(rot.z) || 0, w: Number(rot.w) || 1 },
  }
}

export function ivgQuatNormalize(q: Partial<Quat>): Quat {
  const x = Number(q.x ?? 0), y = Number(q.y ?? 0), z = Number(q.z ?? 0), w = Number(q.w ?? 1)
  const n = Math.hypot(x, y, z, w) || 1
  return { x: x / n, y: y / n, z: z / n, w: w / n }
}

export function ivgQuatMultiply(a: Quat, b: Quat): Quat {
  return {
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  }
}

export function ivgQuatConjugate(q: Quat): Quat {
  return { x: -q.x, y: -q.y, z: -q.z, w: q.w }
}

export function ivgRotateVectorByQuat(v: Vec3, q: Quat): Vec3 {
  const p: Quat = { x: v.x, y: v.y, z: v.z, w: 0 }
  const qp = ivgQuatMultiply(q, p)
  const out = ivgQuatMultiply(qp, ivgQuatConjugate(q))
  return { x: out.x, y: out.y, z: out.z }
}

export function ivgComposeTransforms(a: Partial<Transform>, b: Partial<Transform>): Transform {
  const ta = ivgCloneTransform(a)
  const tb = ivgCloneTransform(b)
  const qa = ivgQuatNormalize(ta.rotation)
  const qb = ivgQuatNormalize(tb.rotation)
  const vb = ivgRotateVectorByQuat(tb.translation, qa)
  const q = ivgQuatNormalize(ivgQuatMultiply(qa, qb))
  return {
    translation: { x: ta.translation.x + vb.x, y: ta.translation.y + vb.y, z: ta.translation.z + vb.z },
    rotation: { x: q.x, y: q.y, z: q.z, w: q.w },
  }
}

export function ivgInvertTransform(tf: Partial<Transform>): Transform {
  const src = ivgCloneTransform(tf)
  const q = ivgQuatNormalize(src.rotation)
  const qi = ivgQuatConjugate(q)
  const t = ivgRotateVectorByQuat({ x: -src.translation.x, y: -src.translation.y, z: -src.translation.z }, qi)
  return { translation: { x: t.x, y: t.y, z: t.z }, rotation: { x: qi.x, y: qi.y, z: qi.z, w: qi.w } }
}

interface TfEdge { parent: string; transform: Transform }

export function ivgBuildTfPath(frame: string, edges: Record<string, TfEdge>): Array<{ frame: string; transform: Transform }> {
  const out = [{ frame, transform: ivgIdentityTransform() }]
  const seen = new Set([frame])
  let cur = frame
  let acc = ivgIdentityTransform()
  for (let i = 0; i < 256; i++) {
    const edge = edges[cur]
    if (!edge?.parent) break
    acc = ivgComposeTransforms(edge.transform, acc)
    cur = edge.parent
    if (seen.has(cur)) break
    seen.add(cur)
    out.push({ frame: cur, transform: acc })
  }
  return out
}

export function ivgFindRelativeTransform(
  sourceFrame: string, targetFrame: string, edges: Record<string, TfEdge>
): Transform | null {
  const src = normalizeFrameId(sourceFrame)
  const dst = normalizeFrameId(targetFrame)
  if (!src || !dst) return null
  if (src === dst) return ivgIdentityTransform()
  const srcPath = ivgBuildTfPath(src, edges)
  const dstPath = ivgBuildTfPath(dst, edges)
  const srcMap: Record<string, Transform> = {}
  srcPath.forEach(e => { srcMap[e.frame] = e.transform })
  for (const entry of dstPath) {
    if (!Object.prototype.hasOwnProperty.call(srcMap, entry.frame)) continue
    return ivgComposeTransforms(ivgInvertTransform(srcMap[entry.frame]), entry.transform)
  }
  return null
}
