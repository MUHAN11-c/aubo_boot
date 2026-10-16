// pointcloud_parse.test.js — PointCloud2 共享解析器单元测试
//
// 覆盖：fields[].offset 语义（非零偏移）、point_step 步进、packed rgb、
// intensity（float32/uint8）、float64 xyz、maxPoints 截断、坏输入容错。

import { describe, expect, it } from 'vitest';
import { parsePointCloud2, toUint8Array } from '../pointcloud_parse.js';

const F32 = 7;
const F64 = 8;
const U8 = 2;

function makeCloud({ points, fields, pointStep, width, height, extra }) {
	const total = width * height;
	const buf = new ArrayBuffer(total * pointStep);
	const dv = new DataView(buf);
	for (let i = 0; i < total; i++) {
		const base = i * pointStep;
		for (const f of fields) {
			const v = points[i]?.[f.name];
			if (v === undefined) continue;
			if (f.name === 'rgb') {
				// PCL 惯例：packed rgb 按位塞进 float32 字段（声明类型与位内容无关）
				dv.setUint32(base + f.offset, v, true);
			} else if (f.datatype === F32) dv.setFloat32(base + f.offset, v, true);
			else if (f.datatype === F64) dv.setFloat64(base + f.offset, v, true);
			else if (f.datatype === U8) dv.setUint8(base + f.offset, v);
		}
	}
	return {
		width, height, point_step: pointStep, fields,
		data: new Uint8Array(buf),
		...extra,
	};
}

describe('parsePointCloud2', () => {
	it('xyz float32 前置布局（offset 0/4/8, step 12）读出真实坐标', () => {
		const cloud = makeCloud({
			width: 2, height: 1, pointStep: 12,
			fields: [
				{ name: 'x', offset: 0, datatype: F32 },
				{ name: 'y', offset: 4, datatype: F32 },
				{ name: 'z', offset: 8, datatype: F32 },
			],
			points: [
				{ x: 0.5, y: -1.25, z: 2.0 },
				{ x: 10.5, y: 0, z: -3.5 },
			],
		});
		const positions = new Float32Array(6);
		const { count, error } = parsePointCloud2(cloud, { positions });
		expect(error).toBeNull();
		expect(count).toBe(2);
		expect([...positions]).toEqual([0.5, -1.25, 2.0, 10.5, 0, -3.5]);
	});

	it('非零 offset（intensity 前置，step 16）按 fields[].offset 读取', () => {
		// 旧 viz 实现硬编码 0/1/2 字节读取的回归测试：坐标在 offset 4/8/12
		const cloud = makeCloud({
			width: 1, height: 1, pointStep: 16,
			fields: [
				{ name: 'intensity', offset: 0, datatype: F32 },
				{ name: 'x', offset: 4, datatype: F32 },
				{ name: 'y', offset: 8, datatype: F32 },
				{ name: 'z', offset: 12, datatype: F32 },
			],
			points: [{ intensity: 42, x: 1, y: 2, z: 3 }],
		});
		const positions = new Float32Array(3);
		const { count, error } = parsePointCloud2(cloud, { positions });
		expect(error).toBeNull();
		expect(count).toBe(1);
		expect([...positions]).toEqual([1, 2, 3]);
	});

	it('packed rgb → 0..1 颜色', () => {
		// rgb 以 float32 存储（PCL 惯例），按位重解释为 packed uint32
		const rgb = (r, g, b) => (r | (g << 8) | (b << 16)) >>> 0;
		const cloud = makeCloud({
			width: 2, height: 1, pointStep: 16,
			fields: [
				{ name: 'x', offset: 0, datatype: F32 },
				{ name: 'y', offset: 4, datatype: F32 },
				{ name: 'z', offset: 8, datatype: F32 },
				{ name: 'rgb', offset: 12, datatype: F32 },
			],
			points: [
				{ x: 0, y: 0, z: 0, rgb: rgb(255, 0, 0) },
				{ x: 1, y: 0, z: 0, rgb: rgb(0, 255, 128) },
			],
		});
		const positions = new Float32Array(6);
		const colors = new Float32Array(6);
		const { count, error } = parsePointCloud2(cloud, { positions, colors });
		expect(error).toBeNull();
		expect(count).toBe(2);
		expect(colors[0]).toBeCloseTo(1.0);   // r=255 → 1
		expect(colors[1]).toBeCloseTo(0.0);
		expect(colors[2]).toBeCloseTo(0.0);
		expect(colors[3]).toBeCloseTo(0.0);
		expect(colors[4]).toBeCloseTo(1.0);   // g
		expect(colors[5]).toBeCloseTo(128 / 255, 3);
	});

	it('intensity float32 灰度 + uint8 intensity 归一', () => {
		const mk = (type, v1, v2) => makeCloud({
			width: 2, height: 1, pointStep: 16,
			fields: [
				{ name: 'x', offset: 0, datatype: F32 },
				{ name: 'y', offset: 4, datatype: F32 },
				{ name: 'z', offset: 8, datatype: F32 },
				{ name: 'intensity', offset: 12, datatype: type },
			],
			points: [
				{ x: 0, y: 0, z: 0, intensity: v1 },
				{ x: 1, y: 0, z: 0, intensity: v2 },
			],
		});
		const positions = new Float32Array(6);
		const colors = new Float32Array(6);
		parsePointCloud2(mk(F32, 0.25, 0.75), { positions, colors });
		expect(colors[0]).toBeCloseTo(0.25);
		expect(colors[3]).toBeCloseTo(0.75);

		parsePointCloud2(mk(U8, 64, 255), { positions, colors });
		expect(colors[0]).toBeCloseTo(64 / 255, 3);
		expect(colors[3]).toBeCloseTo(1.0);
	});

	it('float64 xyz 支持', () => {
		const cloud = makeCloud({
			width: 1, height: 1, pointStep: 24,
			fields: [
				{ name: 'x', offset: 0, datatype: F64 },
				{ name: 'y', offset: 8, datatype: F64 },
				{ name: 'z', offset: 16, datatype: F64 },
			],
			points: [{ x: 0.125, y: 3.5, z: -0.75 }],
		});
		const positions = new Float32Array(3);
		const { count, error } = parsePointCloud2(cloud, { positions });
		expect(error).toBeNull();
		expect(count).toBe(1);
		expect(positions[0]).toBeCloseTo(0.125);
		expect(positions[1]).toBeCloseTo(3.5);
		expect(positions[2]).toBeCloseTo(-0.75);
	});

	it('maxPoints 超限 stride 采样（不再截尾）+ height×width 总数语义', () => {
		const cloud = makeCloud({
			width: 4, height: 2, pointStep: 12,   // 8 点
			fields: [
				{ name: 'x', offset: 0, datatype: F32 },
				{ name: 'y', offset: 4, datatype: F32 },
				{ name: 'z', offset: 8, datatype: F32 },
			],
			points: Array.from({ length: 8 }, (_, i) => ({ x: i, y: 0, z: 0 })),
		});
		const positions = new Float32Array(30);
		const { count } = parsePointCloud2(cloud, { positions, maxPoints: 5 });
		// stride=ceil(8/5)=2 → 采 i=0,2,4,6（空间均匀，count=4 ≤ maxPoints）
		expect(count).toBe(4);
		const xs = [...positions].filter((_, k) => k % 3 === 0);
		expect(xs.slice(0, count)).toEqual([0, 2, 4, 6]);
	});

	it('organized 云（height>1）stride 采样覆盖全部行（旧截断会丢下半幅）', () => {
		const cloud = makeCloud({
			width: 4, height: 4, pointStep: 12,   // 16 点，行 r = floor(i/4)
			fields: [
				{ name: 'x', offset: 0, datatype: F32 },
				{ name: 'y', offset: 4, datatype: F32 },
				{ name: 'z', offset: 8, datatype: F32 },
			],
			points: Array.from({ length: 16 }, (_, i) => ({ x: i, y: 0, z: 0 })),
		});
		const positions = new Float32Array(64);
		const { count } = parsePointCloud2(cloud, { positions, maxPoints: 8 });
		// stride=2 → i=0,2,…,14：4 行各贡献 2 点
		expect(count).toBe(8);
		const xs = [...positions.slice(0, count * 3)].filter((_, k) => k % 3 === 0);
		expect(xs).toEqual([0, 2, 4, 6, 8, 10, 12, 14]);
		// 每行都有采样点（旧行为截断到前 8 点只剩第 0/1 行）
		const rows = xs.map((x) => Math.floor(x / 4));
		expect(new Set(rows).size).toBe(4);
	});

	it('缺 xyz 字段 → error=fields；data 缺失 → error=data', () => {
		const positions = new Float32Array(3);
		expect(parsePointCloud2({
			width: 1, height: 1, point_step: 4,
			fields: [{ name: 'intensity', offset: 0, datatype: F32 }],
			data: new Uint8Array(4),
		}, { positions })).toEqual({ count: 0, error: 'fields' });
		expect(parsePointCloud2({ width: 1, height: 1, fields: [] }, { positions }))
			.toEqual({ count: 0, error: 'data' });
	});

	it('toUint8Array 兼容 TypedArray 视图与普通数组', () => {
		const base = new ArrayBuffer(8);
		const view = new Uint8Array(base, 2, 4);
		view.set([9, 8, 7, 6]);
		const out = toUint8Array(view);
		expect(out instanceof Uint8Array).toBe(true);
		expect([...out]).toEqual([9, 8, 7, 6]);
		expect(toUint8Array([1, 2, 3]).length).toBe(3);
		expect(toUint8Array(null)).toBeNull();
	});
});
