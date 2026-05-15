/**
 * UrdfParser — URDF XML 解析器
 *
 * 替代旧版: ROS3D.Urdf (ros3djs internal)
 *
 * 解析 URDF XML 字符串，提取 link/joint/material/visual 结构。
 * 支持 package:// 路径 → /api/ivg/robot-mesh/ 转换。
 */

export interface UrdfOrigin {
  xyz: [number, number, number]
  rpy: [number, number, number]
}

export interface UrdfMaterial {
  name: string
  color?: { r: number; g: number; b: number; a: number }
  texture?: string
}

export interface UrdfMesh {
  filename: string          // 已解析 package:// → /api/ivg/robot-mesh/{pkg}/{path}
  scale?: [number, number, number]
}

export interface UrdfVisual {
  origin: UrdfOrigin
  geometry: { mesh?: UrdfMesh; box?: { size: [number, number, number] }; cylinder?: { radius: number; length: number }; sphere?: { radius: number } }
  material?: UrdfMaterial
}

export interface UrdfLink {
  name: string
  visual?: UrdfVisual
}

export interface UrdfJoint {
  name: string
  type: string  // 'revolute' | 'prismatic' | 'fixed' | 'continuous' | 'floating' | 'planar'
  parent: string
  child: string
  origin: UrdfOrigin
  axis: [number, number, number]
  limits?: { lower: number; upper: number; effort: number; velocity: number }
}

export interface UrdfRobot {
  name: string
  links: UrdfLink[]
  joints: UrdfJoint[]
  materials: Map<string, UrdfMaterial>
  linkMap: Map<string, UrdfLink>
}

const DEFAULT_ORIGIN: UrdfOrigin = { xyz: [0, 0, 0], rpy: [0, 0, 0] }

function parseOrigin(el: Element): UrdfOrigin {
  const o = el.querySelector(':scope > origin')
  if (!o) return { ...DEFAULT_ORIGIN }
  const xyz = (o.getAttribute('xyz') || '0 0 0').split(/\s+/).map(Number) as [number, number, number]
  const rpy = (o.getAttribute('rpy') || '0 0 0').split(/\s+/).map(Number) as [number, number, number]
  return { xyz: [xyz[0] || 0, xyz[1] || 0, xyz[2] || 0], rpy: [rpy[0] || 0, rpy[1] || 0, rpy[2] || 0] }
}

function parseMaterial(el: Element): UrdfMaterial | undefined {
  const mat = el.querySelector(':scope > material')
  if (!mat) return undefined
  const name = mat.getAttribute('name') || ''
  const colorEl = mat.querySelector(':scope > color')
  const rgba = (colorEl?.getAttribute('rgba') || '0.8 0.8 0.8 1.0').split(/\s+/).map(Number)
  return { name, color: { r: rgba[0] ?? 0.8, g: rgba[1] ?? 0.8, b: rgba[2] ?? 0.8, a: rgba[3] ?? 1.0 } }
}

function resolvePackageUrl(filename: string, meshBase: string): string {
  if (filename.startsWith('package://')) {
    const rest = filename.slice('package://'.length)
    return `${meshBase}${rest}`
  }
  return filename
}

function parseGeometry(visualEl: Element, meshBase: string): UrdfVisual['geometry'] {
  const geom: UrdfVisual['geometry'] = {}
  // URDF 结构: <visual><geometry><mesh/></geometry></visual>
  // geometry 是 visual 的子元素，mesh/box/cylinder/sphere 在 geometry 内部
  const geomEl = visualEl.querySelector(':scope > geometry')
  if (!geomEl) return geom

  const meshEl = geomEl.querySelector(':scope > mesh')
  if (meshEl) {
    const filename = meshEl.getAttribute('filename') || ''
    const scale = (meshEl.getAttribute('scale') || '1 1 1').split(/\s+/).map(Number) as [number, number, number]
    geom.mesh = { filename: resolvePackageUrl(filename, meshBase), scale }
  }
  const boxEl = geomEl.querySelector(':scope > box')
  if (boxEl) {
    const size = (boxEl.getAttribute('size') || '0 0 0').split(/\s+/).map(Number) as [number, number, number]
    geom.box = { size }
  }
  const cylEl = geomEl.querySelector(':scope > cylinder')
  if (cylEl) {
    geom.cylinder = { radius: Number(cylEl.getAttribute('radius') || 0), length: Number(cylEl.getAttribute('length') || 0) }
  }
  const sphEl = geomEl.querySelector(':scope > sphere')
  if (sphEl) {
    geom.sphere = { radius: Number(sphEl.getAttribute('radius') || 0) }
  }
  return geom
}

export function parseUrdf(xml: string, meshBase: string): UrdfRobot {
  const parser = new DOMParser()
  const doc = parser.parseFromString(xml, 'text/xml')
  const robotEl = doc.querySelector('robot')
  if (!robotEl) throw new Error('URDF: <robot> root element not found')

  const robotName = robotEl.getAttribute('name') || 'robot'
  const links: UrdfLink[] = []
  const joints: UrdfJoint[] = []
  const materials = new Map<string, UrdfMaterial>()
  const linkMap = new Map<string, UrdfLink>()

  // 先解析全局 material
  robotEl.querySelectorAll(':scope > material').forEach((m) => {
    const mat = parseMaterial(m)
    if (mat) materials.set(mat.name, mat)
  })

  // 解析 links
  robotEl.querySelectorAll(':scope > link').forEach((linkEl) => {
    const name = linkEl.getAttribute('name') || ''
    const visualEl = linkEl.querySelector(':scope > visual')
    let visual: UrdfVisual | undefined
    if (visualEl) {
      const origin = parseOrigin(visualEl)
      const geometry = parseGeometry(visualEl, meshBase)
      let material = parseMaterial(visualEl)
      if (!material && materials.size > 0) {
        // 尝试使用第一个全局 material
        material = materials.values().next().value
      }
      visual = { origin, geometry, material }
    }
    const link: UrdfLink = { name, visual }
    links.push(link)
    linkMap.set(name, link)
  })

  // 解析 joints
  robotEl.querySelectorAll(':scope > joint').forEach((jointEl) => {
    const name = jointEl.getAttribute('name') || ''
    const type = jointEl.getAttribute('type') || 'fixed'
    const parentEl = jointEl.querySelector(':scope > parent')
    const childEl = jointEl.querySelector(':scope > child')
    const parent = parentEl?.getAttribute('link') || ''
    const child = childEl?.getAttribute('link') || ''
    const origin = parseOrigin(jointEl)
    const axisStr = (jointEl.querySelector(':scope > axis')?.getAttribute('xyz') || '1 0 0').split(/\s+/).map(Number)
    const axis: [number, number, number] = [
      isFinite(axisStr[0]) ? axisStr[0] : 1,
      isFinite(axisStr[1]) ? axisStr[1] : 0,
      isFinite(axisStr[2]) ? axisStr[2] : 0,
    ]
    const limitEl = jointEl.querySelector(':scope > limit')
    const limits = limitEl ? {
      lower: Number(limitEl.getAttribute('lower') || 0),
      upper: Number(limitEl.getAttribute('upper') || 0),
      effort: Number(limitEl.getAttribute('effort') || 0),
      velocity: Number(limitEl.getAttribute('velocity') || 0),
    } : undefined

    joints.push({ name, type, parent, child, origin, axis, limits })
  })

  return { name: robotName, links, joints, materials, linkMap }
}
