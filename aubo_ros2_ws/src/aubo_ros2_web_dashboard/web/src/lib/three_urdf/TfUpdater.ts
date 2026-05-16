/**
 * TfUpdater — TF + joint_states → Three.js transform 更新器
 *
 * 替代旧版: IvgRos3dTfClient (tf_clients.js) + ROS3D.Urdf TF following
 *
 * 功能:
 *   - 维护 TF 树 (从 /tf, /tf_static 消息更新)
 *   - 监听 /joint_states 更新关节角
 *   - 计算 link transforms 并更新 Three.js Object3D
 *   - 支持 rosbridge WebSocket (通过回调注入)
 */
import * as THREE from 'three'
import {
  normalizeFrameId,
  ivgCloneTransform,
  ivgFindRelativeTransform,
  type Transform,
} from '@/lib/tf_math'

interface TfEdge {
  parent: string
  translation: { x: number; y: number; z: number }
  rotation: { x: number; y: number; z: number; w: number }
}

export interface JointState {
  name: string[]
  position: number[]
}

export class TfUpdater {
  private _edges = new Map<string, TfEdge>()
  private _jointPositions = new Map<string, number>()
  private _jointNames: string[] = []
  private _onUpdate: (() => void) | null = null

  /** 处理 /tf 或 /tf_static 消息 */
  ingestTfMessage(msg: any): void {
    const arr = msg?.transforms ?? []
    for (const t of arr) {
      const child = normalizeFrameId(t?.child_frame_id)
      const parent = normalizeFrameId(t?.header?.frame_id)
      if (!child || !parent || !t.transform) continue
      this._edges.set(child, {
        parent,
        translation: {
          x: Number(t.transform.translation?.x) || 0,
          y: Number(t.transform.translation?.y) || 0,
          z: Number(t.transform.translation?.z) || 0,
        },
        rotation: {
          x: Number(t.transform.rotation?.x) || 0,
          y: Number(t.transform.rotation?.y) || 0,
          z: Number(t.transform.rotation?.z) || 0,
          w: Number(t.transform.rotation?.w) || 1,
        },
      })
    }
  }

  /** 处理 /joint_states 消息 */
  ingestJointStates(msg: any): void {
    const names: string[] = msg?.name ?? []
    const positions: number[] = msg?.position ?? []
    if (names.length === 0) return
    this._jointNames = names
    for (let i = 0; i < names.length; i++) {
      this._jointPositions.set(names[i], Number(positions[i]) || 0)
    }
  }

  get jointNames(): string[] { return this._jointNames }
  get jointPositions(): Map<string, number> { return this._jointPositions }

  /** 获取从 base_frame 到 target_frame 的相对变换 */
  getTransform(targetFrame: string, baseFrame: string): { translation: THREE.Vector3; rotation: THREE.Quaternion } | null {
    const tf = normalizeFrameId(targetFrame)
    const bf = normalizeFrameId(baseFrame)
    if (!tf || !bf) return null
    if (tf === bf) {
      return { translation: new THREE.Vector3(0, 0, 0), rotation: new THREE.Quaternion(0, 0, 0, 1) }
    }
    const edges: Record<string, { parent: string; transform: Transform }> = {}
    for (const [child, edge] of this._edges) {
      edges[child] = {
        parent: edge.parent,
        transform: {
          translation: edge.translation,
          rotation: edge.rotation,
        },
      }
    }
    const tfBaseToTarget = ivgFindRelativeTransform(bf, tf, edges)
    if (!tfBaseToTarget) return null
    return {
      translation: new THREE.Vector3(
        tfBaseToTarget.translation.x,
        tfBaseToTarget.translation.y,
        tfBaseToTarget.translation.z
      ),
      rotation: new THREE.Quaternion(
        tfBaseToTarget.rotation.x,
        tfBaseToTarget.rotation.y,
        tfBaseToTarget.rotation.z,
        tfBaseToTarget.rotation.w
      ).normalize(),
    }
  }

  /** 按 fixedFrame→link TF 更新单个 link，等价旧版 ROS3D TF following 喵~ */
  updateLinkTransform(obj: THREE.Object3D, linkName: string, fixedFrame: string): boolean {
    const tf = this.getTransform(linkName, fixedFrame)
    if (!tf) return false
    obj.position.copy(tf.translation)
    obj.quaternion.copy(tf.rotation)
    return true
  }

  /**
   * 更新单个 joint transform 对象（用于关节角驱动的旋转/平移）
   * @param obj   joint transform 的 Three.js Object3D (Group)
   * @param jointName  关节名（对应 joint_states 中的 name）
   * @param jointType  关节类型
   * @param axis  关节轴 (URDF 定义)
   */
  updateJointTransform(obj: THREE.Object3D, jointName: string, jointType: string, axis?: [number, number, number]): void {
    const pos = this._jointPositions.get(jointName)
    if (pos === undefined || !isFinite(pos)) return

    // 恢复 joint origin 静态位移（在 _build() 中保存于 userData）
    const originPos: THREE.Vector3 | undefined = obj.userData._jointOriginPos
    const originQuat: THREE.Quaternion | undefined = obj.userData._jointOriginQuat

    if (jointType === 'revolute' || jointType === 'continuous') {
      const ax = axis || [0, 0, 1]
      // 恢复 URDF joint origin 的静态旋转，再绕局部轴叠加关节角
      if (originQuat) {
        obj.quaternion.copy(originQuat)
      } else {
        obj.rotation.set(0, 0, 0)
      }
      obj.rotateOnAxis(new THREE.Vector3(ax[0], ax[1], ax[2]).normalize(), pos)
    } else if (jointType === 'prismatic') {
      const ax = axis || [1, 0, 0]
      const dir = new THREE.Vector3(ax[0], ax[1], ax[2]).normalize()
      // 恢复 URDF joint origin 的静态位移，再加上关节平移
      if (originPos) {
        obj.position.copy(dir.clone().multiplyScalar(pos).add(originPos))
      } else {
        obj.position.copy(dir.multiplyScalar(pos))
      }
    }
  }

  get edges(): Map<string, TfEdge> { return this._edges }

  clear(): void {
    this._edges.clear()
    this._jointPositions.clear()
    this._jointNames = []
  }
}
