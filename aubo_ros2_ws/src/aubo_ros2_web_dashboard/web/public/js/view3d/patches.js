/**
 * 3D 兼容补丁层：
 * - 统一只使用 ros3d 内嵌 THREE，不再依赖页面单独加载 three.min.js。
 * - 这里只保留 dashboard 自己的运行时补丁；RobotWebTools 本体兼容应优先下沉到 src/robotwebtools。
 * - 这里不创建视图，只提供 session 启动后围绕 Viewer.scene 的安全补丁函数。
 */

	/** ros3d 自带内嵌 THREE；按 prototype 去重打补丁。 */
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
					/* ignore */
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
				/* ignore */
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
