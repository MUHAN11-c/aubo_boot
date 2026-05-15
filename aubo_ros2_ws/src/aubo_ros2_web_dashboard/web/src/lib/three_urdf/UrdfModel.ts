/**
 * UrdfModel — URDF → Three.js Object3D 树构建器
 *
 * 替代旧版: ROS3D.UrdfClient + ROS3D.Urdf (ros3djs)
 *
 * 功能:
 *   - 根据解析后的 URDF 构建 Three.js Object3D 层级
 *   - 通过 STLLoader / ColladaLoader 加载网格
 *   - 支持 reload（工具快换：后台构建新树 → 完成 → 替换）
 *   - 应用 URDF material 颜色
 */
import * as THREE from 'three'
import { STLLoader } from 'three/addons/loaders/STLLoader.js'
import type { UrdfRobot, UrdfJoint, UrdfLink, UrdfVisual } from './UrdfParser'

export interface LinkObject {
  name: string
  object: THREE.Group
  visual: THREE.Group | null
  jointType: string
  parentName: string | null
  axis?: [number, number, number]  // URDF 关节轴
}

export class UrdfModel {
  root: THREE.Group
  links: Map<string, LinkObject> = new Map()
  private _meshBase: string
  private _ready = false
  private _onReady: (() => void) | null = null
  private _meshLoadCount = 0
  private _meshDoneCount = 0
  private _building = false

  constructor(robot: UrdfRobot, meshBase: string, onReady?: () => void) {
    this._meshBase = meshBase
    this._onReady = onReady || null
    this.root = new THREE.Group()
    this.root.name = robot.name
    this._build(robot)
  }

  private _build(robot: UrdfRobot): void {
    this._building = true
    // 先为每个 link 创建一个 Group
    const linkGroups = new Map<string, THREE.Group>()
    for (const link of robot.links) {
      const group = new THREE.Group()
      group.name = link.name
      linkGroups.set(link.name, group)
    }

    // 加载 visual meshes
    for (const link of robot.links) {
      const group = linkGroups.get(link.name)!
      if (link.visual) {
        this._loadVisual(link.visual, (obj) => {
          obj.name = `${link.name}_visual`
          group.add(obj)
        })
      }
    }

    // 构建 joint 层级：child link → parent link
    // 找到根 links（不是任何 joint 的 child 的 links）
    const children = new Set(robot.joints.map(j => j.child))
    const rootLinks = robot.links.filter(l => !children.has(l.name))

    // 根 links 直接添加到 root
    for (const rl of rootLinks) {
      const group = linkGroups.get(rl.name)!
      this.root.add(group)
      this.links.set(rl.name, { name: rl.name, object: group, visual: group.children.length > 0 ? group : null, jointType: 'fixed', parentName: null })
    }

    // 处理 joints：将 child link 放到 parent link 下
    for (const joint of robot.joints) {
      const childGroup = linkGroups.get(joint.child)
      const parentGroup = linkGroups.get(joint.parent)
      if (!childGroup) {
        // 链接不存在 link 的 joint（如 tool_tcp → 虚拟 link）
        // 创建一个空 group 容纳视觉
        continue
      }

      // 创建 joint transform group
      const jointTransform = new THREE.Group()
      jointTransform.name = `${joint.name}_joint`

      // 应用 joint origin
      const o = joint.origin
      jointTransform.position.set(o.xyz[0], o.xyz[1], o.xyz[2])
      const euler = new THREE.Euler(o.rpy[0], o.rpy[1], o.rpy[2], 'XYZ')
      const originQuat = new THREE.Quaternion().setFromEuler(euler)
      jointTransform.quaternion.copy(originQuat)
      // 保存 joint origin 静态位移，供 TfUpdater 恢复（updateJointTransform 会覆盖 rotation/position）
      jointTransform.userData._jointOriginPos = new THREE.Vector3(o.xyz[0], o.xyz[1], o.xyz[2])
      jointTransform.userData._jointOriginQuat = originQuat.clone()

      // 将 child group 放到 joint transform 下
      jointTransform.add(childGroup)

      // 将 joint transform 放到 parent group 下
      if (parentGroup) {
        parentGroup.add(jointTransform)
      } else {
        this.root.add(jointTransform)
      }

      this.links.set(joint.child, {
        name: joint.child,
        object: childGroup,
        visual: childGroup.children.length > 0 ? childGroup : null,
        jointType: joint.type,
        parentName: joint.parent,
        axis: joint.axis,
      })
      this.links.set(joint.name + '_joint', {
        name: joint.name,
        object: jointTransform,
        visual: null,
        jointType: joint.type,
        parentName: joint.parent,
        axis: joint.axis,
      })
    }

    // 如果没有 root links，则所有 links 都是 children，需要把顶层 parent 加到 root
    if (rootLinks.length === 0 && robot.joints.length > 0) {
      // 找到最顶层 parent（不在任何 joint 的 child 中）
      const topParents = new Set(robot.joints.map(j => j.parent))
      for (const j of robot.joints) topParents.delete(j.child)
      for (const tp of topParents) {
        const group = linkGroups.get(tp)
        if (group && !this.root.children.includes(group)) {
          this.root.add(group)
          this.links.set(tp, { name: tp, object: group, visual: group.children.length > 0 ? group : null, jointType: 'fixed', parentName: null })
        }
      }
    }

    this._building = false
    // 所有同步 mesh（DAE box 占位）和 hierarchy 都已完成，检查异步 mesh 是否就绪
    this._tryFinish()
  }

  private _loadVisual(visual: UrdfVisual, onObj: (obj: THREE.Group) => void): void {
    const group = new THREE.Group()
    const o = visual.origin
    group.position.set(o.xyz[0], o.xyz[1], o.xyz[2])
    const euler = new THREE.Euler(o.rpy[0], o.rpy[1], o.rpy[2], 'XYZ')
    group.quaternion.setFromEuler(euler)

    const mat = visual.material
    const color = mat?.color
    const meshMat = new THREE.MeshPhongMaterial({
      color: color ? new THREE.Color(color.r, color.g, color.b) : 0xcccccc,
      transparent: color ? color.a < 1 : false,
      opacity: color?.a ?? 1.0,
      flatShading: false,
    })

    if (visual.geometry.mesh) {
      this._meshLoadCount++
      const filename = visual.geometry.mesh.filename
      const scale = visual.geometry.mesh.scale || [1, 1, 1]

      if (filename.endsWith('.dae')) {
        // DAE/Collada 无法直接通过 STLLoader 加载，但 collision 目录下有同名的 .stl 文件
        // 将 visual/linkX.dae → collision/linkX.stl 作为降级加载
        const stlFilename = filename.replace(/visual\//, 'collision/').replace(/\.dae$/i, '.stl')
        this._loadStl(stlFilename, meshMat, scale, (mesh) => {
          group.add(mesh)
          this._meshDoneCount++
          this._checkReady()
          onObj(group)
        }, () => {
          // STL 降级也失败 → 微型 box 占位
          this._loadFallbackBox(group, meshMat, scale, () => { this._meshDoneCount++; this._checkReady(); onObj(group) })
        })
      } else {
        this._loadStl(filename, meshMat, scale, (mesh) => {
          group.add(mesh)
          this._meshDoneCount++
          this._checkReady()
          onObj(group)
        }, () => {
          // STL 加载失败，使用 box 占位
          this._loadFallbackBox(group, meshMat, scale, () => { this._meshDoneCount++; this._checkReady(); onObj(group) })
        })
      }
    } else if (visual.geometry.box) {
      const s = visual.geometry.box.size
      const box = new THREE.Mesh(new THREE.BoxGeometry(s[0], s[1], s[2]), meshMat)
      group.add(box)
      onObj(group)
    } else if (visual.geometry.cylinder) {
      const c = visual.geometry.cylinder
      const cyl = new THREE.Mesh(new THREE.CylinderGeometry(c.radius, c.radius, c.length, 32), meshMat)
      group.add(cyl)
      onObj(group)
    } else if (visual.geometry.sphere) {
      const s = visual.geometry.sphere
      const sph = new THREE.Mesh(new THREE.SphereGeometry(s.radius, 32, 32), meshMat)
      group.add(sph)
      onObj(group)
    } else {
      onObj(group)
    }
  }

  private _loadStl(url: string, material: THREE.Material, scale: number[], onOk: (mesh: THREE.Mesh) => void, onErr: () => void): void {
    const loader = new STLLoader()
    loader.load(url, (geometry) => {
      geometry.computeVertexNormals()
      const mesh = new THREE.Mesh(geometry, material)
      mesh.scale.set(scale[0] || 1, scale[1] || 1, scale[2] || 1)
      onOk(mesh)
    }, undefined, () => {
      console.warn('[UrdfModel] STL load failed:', url)
      onErr()
    })
  }

  private _loadFallbackBox(group: THREE.Group, material: THREE.Material, scale: number[], done: () => void): void {
    // 用 box 占位
    const box = new THREE.Mesh(new THREE.BoxGeometry(0.05 * (scale[0] || 1), 0.05 * (scale[1] || 1), 0.05 * (scale[2] || 1)), material)
    group.add(box)
    done()
  }

  private _checkReady(): void {
    // _loadFallbackBox 的同步回调会在此触发，但 hierarchy 可能尚未构建完
    // 改为延迟到 _build() 末尾的 _tryFinish()
    this._tryFinish()
  }

  private _tryFinish(): void {
    if (this._ready) return
    // 必须等待 hierarchy 构建完成 + 所有 mesh (含异步 STL) 加载完成
    if (this._building) return
    if (this._meshDoneCount < this._meshLoadCount) return
    this._ready = true
    this._onReady?.()
  }

  isReady(): boolean { return this._ready }

  /**
   * 获取指定 link（或 joint transform）的 Object3D
   * 用于 TfUpdater 更新关节角度
   */
  getLinkObject(name: string): LinkObject | undefined {
    return this.links.get(name)
  }

  /** 获取所有可动关节（revolute/continuous/prismatic），含 URDF 关节轴 */
  getActiveJoints(): Array<{ name: string; linkObj: LinkObject; axis: [number, number, number] }> {
    const result: Array<{ name: string; linkObj: LinkObject; axis: [number, number, number] }> = []
    for (const [name, obj] of this.links) {
      if (name.endsWith('_joint') && (obj.jointType === 'revolute' || obj.jointType === 'continuous' || obj.jointType === 'prismatic')) {
        result.push({ name: name.replace('_joint', ''), linkObj: obj, axis: obj.axis || [0, 0, 1] })
      }
    }
    return result
  }
}
