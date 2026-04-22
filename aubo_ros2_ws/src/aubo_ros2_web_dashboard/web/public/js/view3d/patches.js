/**
 * 3D 兼容补丁层：
 * - 处理 ros3d 内嵌 THREE 与页面 THREE 不是同一实例的问题。
 * - 处理 MeshLoader 仅注册小写扩展名的问题。
 * - 这里不创建视图，只提供 session 启动前必须执行的补丁函数。
 */
(function (global) {
	'use strict';

	/** npm ros3d 构建内嵌独立 THREE，与页面 `three.min.js` 非同实例；按 prototype 去重打补丁。 */
	const ivgObject3dAddPatchedPrototypes =
		typeof WeakSet !== 'undefined' ? new WeakSet() : null;

	function ivgInstallMeshLoaderCasePatch() {
		if (typeof ROS3D === 'undefined' || !ROS3D.MeshLoader || !ROS3D.MeshLoader.loaders) return;
		if (ROS3D.MeshLoader.__ivgCasePatched) return;
		const loaders = ROS3D.MeshLoader.loaders;
		const keys = Object.keys(loaders);
		for (let i = 0; i < keys.length; i++) {
			const k = keys[i];
			const up = String(k).toUpperCase();
			if (!Object.prototype.hasOwnProperty.call(loaders, up)) {
				loaders[up] = loaders[k];
			}
		}
		ROS3D.MeshLoader.__ivgCasePatched = true;
	}

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

	/** 页面全局 THREE（若有）。 */
	function installIvgThreeSafeAddPatch() {
		if (typeof THREE === 'undefined' || !THREE.Object3D) return;
		installIvgThreeSafeAddPatchOnPrototype(THREE.Object3D.prototype);
	}

	function ivgRos3dEmbeddedObject3DClass(viewer3d) {
		if (!viewer3d || !viewer3d.scene) return typeof THREE !== 'undefined' ? THREE.Object3D : null;
		const object3dProto = Object.getPrototypeOf(Object.getPrototypeOf(viewer3d.scene));
		const Ctor = object3dProto && object3dProto.constructor;
		return typeof Ctor === 'function' ? Ctor : typeof THREE !== 'undefined' ? THREE.Object3D : null;
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

	global.IVGView3DPatches = {
		ivgInstallMeshLoaderCasePatch,
		installIvgThreeSafeAddPatchOnPrototype,
		installIvgThreeSafeAddPatch,
		ivgRos3dEmbeddedObject3DClass,
		installIvgRos3dEmbeddedThreeSafeAddPatch
	};
})(typeof window !== 'undefined' ? window : globalThis);
