// patches.js — ros3d Object3D polyfill + Three.js safe-add patch
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
								if (mat.transparent === true && typeof mat.opacity === 'number' && mat.opacity === 0) {
									mat.opacity = 1;
									mat.transparent = false;
									mat.needsUpdate = true;
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
				console.warn('[ivg/three] 无法从内嵌 Viewer.scene 解析 Object3D.prototype，跳过 safe-add');
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
