// ros3d_patches.ts — ros3d/Three.js 补丁
// 从 view3d/patches.js (71行) 搬移，功能不变

/** 修复 ros3d 内嵌 Three.js Object3D.add 的透明材质 bug */
export function installIvgThreeSafeAddPatch(viewer3d: any) {
  if (!viewer3d?.scene) return
  const scene = viewer3d.scene
  const proto = Object.getPrototypeOf(Object.getPrototypeOf(scene))
  if (!proto || typeof proto.add !== 'function') {
    try { console.warn('[ivg/three] 无法从内嵌 Viewer.scene 解析 Object3D.prototype，跳过 safe-add') } catch { /* */ }
    return
  }
  if (proto.__ivgSafeAddPatched) return
  proto.__ivgSafeAddPatched = true
  const nativeAdd = proto.add
  proto.add = function (...args: any[]) {
    const filtered: any[] = []
    for (const obj of args) {
      if (!obj || obj.isObject3D !== true) continue
      try {
        if (typeof obj.traverse === 'function') {
          obj.traverse((node: any) => {
            if (!node || node.isMesh !== true || !node.material) return
            const materials = Array.isArray(node.material) ? node.material : [node.material]
            for (const mat of materials) {
              if (!mat) continue
              if (mat.transparent === true && typeof mat.opacity === 'number' && mat.opacity === 0) {
                mat.opacity = 1; mat.transparent = false; mat.needsUpdate = true
              }
            }
          })
        }
      } catch { /* ignore */ }
      filtered.push(obj)
    }
    if (filtered.length === 0) return this
    return nativeAdd.apply(this, filtered)
  }
}
