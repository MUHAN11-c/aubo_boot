// patches.js — ros3d Object3D polyfill + Three.js safe-add patch + 材质升级喵~
import { logBus } from '../core/log-bus.js';
const ivgObject3dAddPatchedPrototypes =
		typeof WeakSet !== 'undefined' ? new WeakSet() : null;
	function installIvgThreeSafeAddPatchOnPrototype(object3dPrototype) {
		if (!object3dPrototype || typeof object3dPrototype.add !== 'function') return;
		if (ivgObject3dAddPatchedPrototypes && ivgObject3dAddPatchedPrototypes.has(object3dPrototype)) return;
		if (object3dPrototype.__ivgSafeAddPatched) return;
		const nativeAdd = object3dPrototype.add;
		object3dPrototype.add = function () {
			const filtered = [];
			for (let i = 0; i < arguments.length; i++) {
				const obj = arguments[i];
				if (!obj) continue;
				if (obj.isObject3D !== true) continue;
				try {
					if (typeof obj.traverse === 'function') {
						obj.traverse(function (node) {
							if (!node || node.isMesh !== true || !node.material) return;
							const materials = Array.isArray(node.material) ? node.material : [node.material];
							for (let mi = 0; mi < materials.length; mi++) {
								const mat = materials[mi];
								if (!mat) continue;
								// 修复 opacity=0 的透明网格 → 恢复不透明
								if (mat.transparent === true && typeof mat.opacity === 'number' && mat.opacity === 0) {
									mat.opacity = 1;
									mat.transparent = false;
									mat.needsUpdate = true;
								}
								// MeshBasicMaterial → MeshPhongMaterial 升级
								// ros3djs 默认给无 <material> 的 STL 网格用 MeshBasicMaterial(0x999999)
								// 这种材质完全不受光照影响 → 平面灰色无立体感
								// 升级为 MeshPhongMaterial 对齐 RViz2 的 OGRE Phong 渲染喵~
								if (mat.isMeshBasicMaterial && !mat.__ivgUpgraded) {
									mat.__ivgUpgraded = true;
									try {
										node.material = new THREE.MeshPhongMaterial({
											color: mat.color,
											specular: 0x222222,
											shininess: 40,
											opacity: mat.opacity,
											transparent: mat.transparent,
											side: mat.side,
										});
										if (typeof mat.dispose === 'function') mat.dispose();
									} catch (e) { /* 材质升级失败则保持原状 */ }
								}
							}
						});
					}
				} catch (eNorm) {
				}
				filtered.push(obj);
			}
			if (filtered.length === 0) return this;
			return nativeAdd.apply(this, filtered);
		};
		if (ivgObject3dAddPatchedPrototypes) ivgObject3dAddPatchedPrototypes.add(object3dPrototype);
		object3dPrototype.__ivgSafeAddPatched = true;
	}
	function ivgRos3dEmbeddedObject3DClass(viewer3d) {
		if (!viewer3d || !viewer3d.scene) return null;
		const object3dProto = Object.getPrototypeOf(Object.getPrototypeOf(viewer3d.scene));
		const Ctor = object3dProto && object3dProto.constructor;
		return typeof Ctor === 'function' ? Ctor : null;
	}
	function installIvgRos3dEmbeddedThreeSafeAddPatch(viewer3d) {
		if (!viewer3d || !viewer3d.scene) return;
		const scene = viewer3d.scene;
		const object3dPrototype = Object.getPrototypeOf(Object.getPrototypeOf(scene));
		if (!object3dPrototype || typeof object3dPrototype.add !== 'function') {
			try {
				logBus.addLog('warn', 'view3d', '无法解析 Object3D.prototype，跳过 safe-add');
			} catch (e) {
			}
			return;
		}
		installIvgThreeSafeAddPatchOnPrototype(object3dPrototype);
	}
const IVGView3DPatches = {
	installIvgThreeSafeAddPatchOnPrototype,
	ivgRos3dEmbeddedObject3DClass,
	installIvgRos3dEmbeddedThreeSafeAddPatch
};
globalThis.IVGView3DPatches = IVGView3DPatches;
export {
	installIvgThreeSafeAddPatchOnPrototype,
	ivgRos3dEmbeddedObject3DClass,
	installIvgRos3dEmbeddedThreeSafeAddPatch,
	IVGView3DPatches
};
