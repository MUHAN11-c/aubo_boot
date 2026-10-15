// pointcloud_island.js — 渲染新岛点云组件（three 0.164 ESM）
//
// 与旧 pointcloud_viewer.js（r89 全局 THREE + 自定义 ShaderMaterial attribute 语法）等价，
// 数据链不变：foxglove MessageReader 解码的 PointCloud2 对象 → updatePoints(pc2)。
// 现代管线：BufferGeometry 预分配复用 + setDrawRange + PointsMaterial(vertexColors)。
// 解析单源在 pointcloud_parse.js（fields[].offset + DataView 读数；rgb packed /
// intensity / Z 高度伪彩三档颜色），viz.js 与本组件共用。

import * as THREE from 'three';
import { parsePointCloud2 } from './pointcloud_parse.js';

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
		if (!pc2) return;
		frameCount++;
		const { count, error } = parsePointCloud2(pc2,
			{ positions: positionArr, colors: colorArr, maxPoints });
		if (error) {
			if (frameCount === 1) {
				console.warn('[pc-island]', error === 'data' ? 'data 字段不可解析' : '无 xyz 字段');
			}
			return;
		}
		pointCount = count;
		posAttr.needsUpdate = true;
		colAttr.needsUpdate = true;
		geometry.setDrawRange(0, count);
		if (frameCount === 1) {
			console.log('[pc-island] 首帧', count, 'points, fields:',
				(pc2.fields || []).map((f) => f.name).join(','));
		}
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
