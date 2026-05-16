/**
 * SceneManager — Three.js 3D 场景管理器
 *
 * 替代旧版: ROS3D.Viewer (ros3djs)
 *
 * 功能:
 *   - Three.js 场景/渲染器/相机/OrbitControls 初始化
 *   - 网格 + 坐标轴
 *   - ResizeObserver 自适应
 *   - requestAnimationFrame 渲染循环
 *   - 触摸设备 pixelRatio 限制
 */
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'

const AXES_SCALE = 0.35
const GRID_COLOR = '#cbd5e1'
const GRID_CELLS = 10
const DESKTOP_PIXEL_RATIO_MAX = 1.5
const ROS_UP = new THREE.Vector3(0, 0, 1)

export class SceneManager {
  scene: THREE.Scene
  renderer: THREE.WebGLRenderer
  camera: THREE.PerspectiveCamera
  controls: OrbitControls
  private _host: HTMLElement
  private _animId = 0
  private _resizeObs: ResizeObserver | null = null
  private _grid: THREE.GridHelper | null = null
  private _axes: THREE.AxesHelper | null = null

  constructor(host: HTMLElement) {
    this._host = host

    // 场景
    this.scene = new THREE.Scene()
    this.scene.background = new THREE.Color('#ffffff')

    // 相机
    const w = host.clientWidth || 400
    const h = Math.max(240, host.clientHeight || Math.round(w * 0.72))
    this.camera = new THREE.PerspectiveCamera(45, w / Math.max(1, h), 0.1, 100)
    // ROS/RViz 使用 Z-up，Three.js 默认 Y-up；viewer 侧改为 Z-up，避免旋转 TF/URDF 数据喵~
    this.camera.up.copy(ROS_UP)
    this.camera.position.set(3, -3, 2.2)

    // 渲染器
    this.renderer = new THREE.WebGLRenderer({ antialias: !this._isCoarsePointer() })
    this.renderer.setSize(w, h)
    this.renderer.setPixelRatio(this._computePixelRatio())
    host.appendChild(this.renderer.domElement)

    // OrbitControls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement)
    this.controls.zoomSpeed = 0.5
    this.controls.target.set(0, 0, 0)
    this.controls.update()

    // 网格
    this._grid = new THREE.GridHelper(GRID_CELLS, GRID_CELLS, GRID_COLOR, GRID_COLOR)
    // GridHelper 默认是 XZ 平面（Y-up 地面），这里旋到 ROS 的 XY 平面（Z-up 地面）喵~
    this._grid.rotation.x = Math.PI / 2
    this.scene.add(this._grid)

    // 坐标轴：Three.js AxesHelper 默认配色为 X=红、Y=绿、Z=蓝喵~
    this._axes = new THREE.AxesHelper(AXES_SCALE)
    this.scene.add(this._axes)

    // 灯光
    const ambient = new THREE.AmbientLight(0xffffff, 0.6)
    this.scene.add(ambient)
    const directional = new THREE.DirectionalLight(0xffffff, 0.5)
    directional.position.set(1, 2, 1)
    this.scene.add(directional)

    // Resize
    this._resizeObs = new ResizeObserver(() => this._onResize())
    this._resizeObs.observe(host)

    // 渲染循环
    this._animId = requestAnimationFrame(() => this._renderLoop())
  }

  addObject(obj: THREE.Object3D): void {
    this.scene.add(obj)
  }

  removeObject(obj: THREE.Object3D): void {
    this.scene.remove(obj)
  }

  resize(w: number, h: number): void {
    this.renderer.setSize(w, h)
    this.camera.aspect = w / Math.max(1, h)
    this.camera.updateProjectionMatrix()
  }

  focusOnObject(obj: THREE.Object3D, opts?: { distanceFactor?: number; minDistance?: number }): void {
    const center = new THREE.Vector3()
    let radius = 0.25
    try {
      obj.getWorldPosition(center)
      obj.traverse((node) => {
        const mesh = node as THREE.Mesh
        if (!mesh.geometry?.boundingSphere) return
        mesh.geometry.computeBoundingSphere()
        const bs = mesh.geometry.boundingSphere!
        const c = bs.center.clone()
        mesh.localToWorld(c)
        center.copy(c)
        radius = Math.max(radius, bs.radius)
      })
    } catch { /* */ }

    const distFactor = opts?.distanceFactor ?? 2.8
    const minDist = opts?.minDistance ?? 0.48
    const dist = Math.max(minDist, radius * distFactor)
    this.camera.up.copy(ROS_UP)
    this.camera.position.set(center.x + dist, center.y - dist * 0.72, center.z + Math.max(dist * 0.52, radius * 1.8))
    this.camera.lookAt(center)
    this.controls.target.copy(center)
    this.controls.update()
  }

  stop(): void {
    if (this._animId) cancelAnimationFrame(this._animId)
    if (this._resizeObs) { this._resizeObs.disconnect(); this._resizeObs = null }
    this.controls.dispose()
    this.renderer.dispose()
    this._host.innerHTML = ''
  }

  private _isCoarsePointer(): boolean {
    return typeof globalThis.matchMedia === 'function' && globalThis.matchMedia('(pointer: coarse)').matches
  }

  private _computePixelRatio(): number {
    if (this._isCoarsePointer()) return 1
    const dpr = Number(globalThis.devicePixelRatio) || 1
    return Math.max(1, Math.min(dpr, DESKTOP_PIXEL_RATIO_MAX))
  }

  private _onResize(): void {
    const w = this._host.clientWidth || 400
    const h = Math.max(240, this._host.clientHeight || Math.round(w * 0.72))
    this.resize(w, h)
  }

  private _renderLoop(): void {
    this._animId = requestAnimationFrame(() => this._renderLoop())
    this.controls.update()
    this.renderer.render(this.scene, this.camera)
  }
}
