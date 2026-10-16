// pointcloud_parse.js — PointCloud2 → positions/colors 纯解析（零 DOM/three 依赖）
//
// 单源提取自 pointcloud_island.js（viz.js 与渲染岛共用）：
//   - 按 fields[].offset + point_step 用 DataView 读数（正确处理 byteOffset 嵌套，
//     杜绝把 data 字节当坐标值的旧 bug）
//   - xyz 支持 FLOAT32/FLOAT64（真实驱动偶有 float64 点云）
//   - 颜色三档：rgb packed / intensity / Z 高度伪彩（colors 传 null 则跳过）
// 调用方提供输出缓冲（热路径零分配），返回 {count, error}：
//   error: null | 'data'（data 字段不可解析）| 'fields'（无 xyz 字段）
//   count=0 且 error 非空时缓冲内容未定义，调用方应隐藏/清空显示。

// PointField datatype 常量（sensor_msgs/PointField）
const FLOAT32 = 7;
const FLOAT64 = 8;

export function toUint8Array(data) {
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

/**
 * 解析 PointCloud2 到调用方缓冲。
 * @param {*} pc2 foxglove CDR 解码后的 PointCloud2 对象
 * @param {{positions: Float32Array, colors?: Float32Array|null, maxPoints?: number}} out
 * @returns {{count: number, error: string|null}}
 */
export function parsePointCloud2(pc2, out) {
	if (!pc2 || pc2.data === undefined) return { count: 0, error: 'data' };
	const positions = out.positions;
	const colors = out.colors || null;
	const maxPoints = out.maxPoints ?? (positions.length / 3) | 0;
	const rawData = toUint8Array(pc2.data);
	if (!rawData || !rawData.buffer) return { count: 0, error: 'data' };
	const dv = new DataView(rawData.buffer, rawData.byteOffset, rawData.byteLength);

	const fields = pc2.fields || [];
	const pointStep = Number(pc2.point_step) || 0;
	if (pointStep <= 0) return { count: 0, error: 'fields' };
	const total = (pc2.height || 1) * (pc2.width || 0);

	// 超上限用 stride 空间均匀采样（旧实现截断尾部：organized 云 height>1 时
	// 直接丢掉下半幅画面；stride 保住视野覆盖，rviz 稠密云同理降采样显示）
	const stride = total > maxPoints ? Math.ceil(total / maxPoints) : 1;

	let xOff = -1, yOff = -1, zOff = -1, iOff = -1;
	let hasRgb = false, hasIntensity = false;
	let xyzFloat64 = false;
	let intensityType = FLOAT32;
	for (const f of fields) {
		if (f.name === 'x') xOff = Number(f.offset);
		else if (f.name === 'y') yOff = Number(f.offset);
		else if (f.name === 'z') zOff = Number(f.offset);
		else if (f.name === 'rgb') { iOff = Number(f.offset); hasRgb = true; }
		else if (f.name === 'intensity') { iOff = Number(f.offset); hasIntensity = true; intensityType = Number(f.datatype) || FLOAT32; }
	}
	if (xOff < 0 || yOff < 0 || zOff < 0) return { count: 0, error: 'fields' };
	if (Number(fields.find((f) => f.name === 'x')?.datatype) === FLOAT64) {
		// xyz float64 点云（部分真实驱动）；坐标读取统一按 float64
		xyzFloat64 = true;
	}

	const readScalar = (offset, type) => {
		switch (type) {
			case 1: return dv.getInt8(offset);          // INT8
			case 2: return dv.getUint8(offset);         // UINT8
			case 3: return dv.getInt16(offset, true);   // INT16
			case 4: return dv.getUint16(offset, true);  // UINT16
			case 5: return dv.getInt32(offset, true);   // INT32
			case 6: return dv.getUint32(offset, true);  // UINT32
			case 8: return dv.getFloat64(offset, true); // FLOAT64
			default: return dv.getFloat32(offset, true); // FLOAT32
		}
	};
	const intensityScale = (intensityType === 2 || intensityType === 4 || intensityType === 6)
		? 255 : 1; // 整型 intensity 归一到 0..1（浮型假定已在 0..1）

	let o = 0;
	for (let i = 0; i < total && o < maxPoints; i += stride) {
		const base = i * pointStep;
		if (base + pointStep > rawData.byteLength) break; // data 短于声明（坏帧）即止
		const x = xyzFloat64 ? dv.getFloat64(base + xOff, true) : dv.getFloat32(base + xOff, true);
		const y = xyzFloat64 ? dv.getFloat64(base + yOff, true) : dv.getFloat32(base + yOff, true);
		const z = xyzFloat64 ? dv.getFloat64(base + zOff, true) : dv.getFloat32(base + zOff, true);
		positions[o * 3] = x;
		positions[o * 3 + 1] = y;
		positions[o * 3 + 2] = z;

		if (colors) {
			if (iOff >= 0 && hasRgb) {
				// PCL 惯例：rgb 字段以 float32 存储，按位重解释为 packed uint32
				const packed = dv.getUint32(base + iOff, true) >>> 0;
				colors[o * 3] = (packed & 0xff) / 255;
				colors[o * 3 + 1] = ((packed >>> 8) & 0xff) / 255;
				colors[o * 3 + 2] = ((packed >>> 16) & 0xff) / 255;
			} else if (iOff >= 0 && hasIntensity) {
				const v = readScalar(base + iOff, intensityType) / intensityScale;
				colors[o * 3] = v;
				colors[o * 3 + 1] = v;
				colors[o * 3 + 2] = v;
			} else {
				// Z 高度伪彩（与旧 pointcloud_island 同规则）
				let t = (z + 0.5) / 1.5;
				if (t < 0) t = 0;
				if (t > 1) t = 1;
				colors[o * 3] = 0.2 + 0.3 * t;
				colors[o * 3 + 1] = 0.5 + 0.5 * (1 - t);
				colors[o * 3 + 2] = 1.0 - 0.5 * t;
			}
		}
		o++;
	}
	return { count: o, error: null };
}
