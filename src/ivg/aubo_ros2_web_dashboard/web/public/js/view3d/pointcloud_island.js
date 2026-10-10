// pointcloud_island.js — 渲染新岛点云组件（three 0.164 ESM）
//
// 与旧 pointcloud_viewer.js（r89 全局 THREE + 自定义 ShaderMaterial attribute 语法）等价，
// 数据链不变：foxglove MessageReader 解码的 PointCloud2 对象 → updatePoints(pc2)。
// 现代管线：BufferGeometry 预分配复用 + setDrawRange + PointsMaterial(vertexColors)；
// XYZ 直填 Float32Array（DataView 读 float32，正确处理 byteOffset 嵌套）。
// 颜色：rgb packed / intensity / Z 高度伪彩三档，与旧实现同规则。
//
// 用法：
//   import { createModernPointCloudViewer } from './pointcloud_island.js';
//   const v = createModernPointCloudViewer({ scene, maxPoints: 300000, pointSize: 0.003 });
//   v.updatePoints(decodedPC2); v.dispose();

import * as THREE from 'three';

function toUint8Array(data) {
	if (data instanceof Uint8Array) return data;
	if (data && data.buffer instanceof ArrayBuffer) {
		return new Uint8Array(data.buffer, data.byteOffset || 0, data.byteLength ?? data.buffer.byteLength);
	}
	// foxglove reader 对无界基本类型数组可能返回数字键对象/普通数组
	if (typeof data === 'object' && data !== null) {
		const len = data.length ?? Object.keys(data).length;
		const out = new Uint8Array(len);
		let i = 0;
		for (const k of Object.keys(data)) out[i++] = data[k] & 0xff;
		return out;
	}
	return null;
}

export function createModernPointCloudViewer(options) {
	const opts = Object.assign({
		maxPoints: 300000,
		pointSize: 0.003,
		visible: true,
	}, options);
	if (!opts.scene) throw new Error('options.scene 是必填项');

	const scene = opts.scene;
	const maxPoints = opts.maxPoints;

	const positionArr = new Float32Array(maxPoints * 3);
	const colorArr = new Float32Array(maxPoints * 3);
	const geometry = new THREE.BufferGeometry();
	const posAttr = new THREE.BufferAttribute(positionArr, 3);
	const colAttr = new THREE.BufferAttribute(colorArr, 3);
	geometry.setAttribute('position', posAttr);
	geometry.setAttribute('color', colAttr);
	geometry.setDrawRange(0, 0);

	const material = new THREE.PointsMaterial({
		size: opts.pointSize,
		vertexColors: true,
		sizeAttenuation: true,
		transparent: true,
		opacity: 0.9,
		depthWrite: false,
	});
	const points = new THREE.Points(geometry, material);
	points.visible = opts.visible;
	points.frustumCulled = false;
	points.name = 'pointcloud-modern';
	scene.add(points);

	let pointCount = 0;
	let frameCount = 0;

	function updatePoints(pc2) {
		if (!pc2 || pc2.data === undefined) return;
		frameCount++;
		const rawData = toUint8Array(pc2.data);
		if (!rawData || !rawData.buffer) {
			if (frameCount === 1) console.warn('[pc-island] data 字段不可解析');
			return;
		}
		const dv = new DataView(rawData.buffer, rawData.byteOffset, rawData.byteLength);

		const fields = pc2.fields || [];
		const pointStep = pc2.point_step || 0;
		const total = (pc2.height || 1) * (pc2.width || 0);
		const n = Math.min(total, maxPoints);

		let xOff = -1, yOff = -1, zOff = -1, iOff = -1, hasRgb = false, hasIntensity = false;
		for (const f of fields) {
			if (f.name === 'x') xOff = f.offset;
			else if (f.name === 'y') yOff = f.offset;
			else if (f.name === 'z') zOff = f.offset;
			else if (f.name === 'rgb') { iOff = f.offset; hasRgb = true; }
			else if (f.name === 'intensity') { iOff = f.offset; hasIntensity = true; }
		}
		if (xOff < 0 || yOff < 0 || zOff < 0) {
			if (frameCount === 1) console.warn('[pc-island] 无 xyz 字段');
			return;
		}

		for (let i = 0; i < n; i++) {
			const base = i * pointStep;
			positionArr[i * 3] = dv.getFloat32(base + xOff, true);
			positionArr[i * 3 + 1] = dv.getFloat32(base + yOff, true);
			positionArr[i * 3 + 2] = dv.getFloat32(base + zOff, true);

			if (iOff >= 0 && hasRgb) {
				const packed = dv.getUint32(base + iOff, true) >>> 0;
				colorArr[i * 3] = (packed & 0xff) / 255;
				colorArr[i * 3 + 1] = ((packed >>> 8) & 0xff) / 255;
				colorArr[i * 3 + 2] = ((packed >>> 16) & 0xff) / 255;
			} else if (iOff >= 0 && hasIntensity) {
				const v = dv.getFloat32(base + iOff, true);
				colorArr[i * 3] = v;
				colorArr[i * 3 + 1] = v;
				colorArr[i * 3 + 2] = v;
			} else {
				const z = positionArr[i * 3 + 2];
				let t = (z + 0.5) / 1.5;
				if (t < 0) t = 0;
				if (t > 1) t = 1;
				colorArr[i * 3] = 0.2 + 0.3 * t;
				colorArr[i * 3 + 1] = 0.5 + 0.5 * (1 - t);
				colorArr[i * 3 + 2] = 1.0 - 0.5 * t;
			}
		}

		pointCount = n;
		posAttr.needsUpdate = true;
		colAttr.needsUpdate = true;
		geometry.setDrawRange(0, n);
		if (frameCount === 1) console.log('[pc-island] 首帧', n, 'points, fields:', fields.map((f) => f.name).join(','));
	}

	function setVisible(v) { points.visible = !!v; }
	function setPointSize(s) { material.size = s; }
	function dispose() {
		scene.remove(points);
		geometry.dispose();
		material.dispose();
	}

	return {
		updatePoints,
		setVisible,
		setPointSize,
		dispose,
		get pointCount() { return pointCount; },
		get points() { return points; },
	};
}
