/**
 * AI 大模型抓取：在识别结果图像上叠加 PointCloud2 投影与 GraspNet MarkerArray（圆柱），
 * 并在相机图像上投影 RViz 风格的地面网格 + RGB 坐标轴（依赖 TF 与 CameraInfo）。
 * 依赖：ivg_rosbridge_bytes.js（IVGRosbridgeBytes）
 */
(function (global) {
	'use strict';

	const MARKER_CYLINDER = 3;
	const MARKER_ACTION_DELETEALL = 3;

	/** 网格所在平面为候选系 local z=0（与 RViz Ground plane 类似，按序取第一个 TF 可达的系） */
	const GRID_FRAME_CANDIDATES = ['base_link', 'world', 'map', 'odom', 'camera_link'];
	const GRID_EXTENT_M = 1.25;
	const GRID_STEP_M = 0.1;
	const AXIS_LEN_M = 0.18;
	/**
	 * 单帧解析/绘制上限；须与 IVG 原生 subscribe 的 max_points 一致，否则桥端会先截断到默认 8000。
	 * ROS 侧 ivg_pointcloud_web_throttle 的 max_points 建议 ≥ 此值。
	 */
	const PC_BRIDGE_MAX_POINTS = 24000;
	const PC_MAX_SAMPLES = PC_BRIDGE_MAX_POINTS;
	/** 半宽像素；略小以便高密度时点更利落 */
	const PC_DOT_HALF = 1.85;

	/** ROS / TF / Marker 的 frame_id 有的带前导 /，与 tf2 树不一致会导致 lookup 失败 */
	function canonicalFrame(s) {
		const t = String(s || '').trim();
		if (!t) return '';
		return t.startsWith('/') ? t.slice(1) : t;
	}

	function bytesToUint8(data) {
		if (!data) return null;
		if (data instanceof Uint8Array) return data;
		if (typeof data === 'string') {
			try {
				const bin = atob(data);
				const out = new Uint8Array(bin.length);
				for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
				return out;
			} catch (e) {
				return null;
			}
		}
		if (Array.isArray(data)) return new Uint8Array(data);
		if (data.data && Array.isArray(data.data)) return new Uint8Array(data.data);
		return null;
	}

	/** @type {Float32Array|null} */
	let latestK = null;
	/** @type {string} */
	let camOpticalFrame = '';
	let lastPc = null;
	let lastMarkers = null;
	/** 当前叠加点云订阅话题（用于 depth_registered 下 TF 缺失时的启发式） */
	let lastPcSubscribeTopic = '';
	let drawPending = false;
	/** 从底图 MJPEG 按像素采样颜色（depth_registered 仅 xyz 时与相机 RGB 对齐） */
	let _ivgChromaCv = null;

	/** 无向边：p_to = mat * p_from（齐次列向量） */
	const edges = new Map();
	function edgeKey(a, b) {
		return `${a}\n${b}`;
	}

	function quatToMat4(qx, qy, qz, qw) {
		const x = qx;
		const y = qy;
		const z = qz;
		const w = qw;
		const x2 = x + x;
		const y2 = y + y;
		const z2 = z + z;
		const xx = x * x2;
		const xy = x * y2;
		const xz = x * z2;
		const yy = y * y2;
		const yz = y * z2;
		const zz = z * z2;
		const wx = w * x2;
		const wy = w * y2;
		const wz = w * z2;
		const m = new Float32Array(16);
		m[0] = 1 - (yy + zz);
		m[1] = xy + wz;
		m[2] = xz - wy;
		m[3] = 0;
		m[4] = xy - wz;
		m[5] = 1 - (xx + zz);
		m[6] = yz + wx;
		m[7] = 0;
		m[8] = xz + wy;
		m[9] = yz - wx;
		m[10] = 1 - (xx + yy);
		m[11] = 0;
		m[12] = 0;
		m[13] = 0;
		m[14] = 0;
		m[15] = 1;
		return m;
	}

	function setTranslate(m, tx, ty, tz) {
		m[12] = tx;
		m[13] = ty;
		m[14] = tz;
	}

	/** column-major（与 three.js / WebGL 一致）：out = a * b */
	function multiplyMat4(a, b) {
		const o = new Float32Array(16);
		for (let c = 0; c < 4; c++) {
			const b0 = b[c * 4 + 0];
			const b1 = b[c * 4 + 1];
			const b2 = b[c * 4 + 2];
			const b3 = b[c * 4 + 3];
			o[c * 4 + 0] = a[0] * b0 + a[4] * b1 + a[8] * b2 + a[12] * b3;
			o[c * 4 + 1] = a[1] * b0 + a[5] * b1 + a[9] * b2 + a[13] * b3;
			o[c * 4 + 2] = a[2] * b0 + a[6] * b1 + a[10] * b2 + a[14] * b3;
			o[c * 4 + 3] = a[3] * b0 + a[7] * b1 + a[11] * b2 + a[15] * b3;
		}
		return o;
	}

	function invertRigid(m) {
		const r0 = m[0];
		const r1 = m[1];
		const r2 = m[2];
		const r4 = m[4];
		const r5 = m[5];
		const r6 = m[6];
		const r8 = m[8];
		const r9 = m[9];
		const r10 = m[10];
		const tx = m[12];
		const ty = m[13];
		const tz = m[14];
		const o = new Float32Array(16);
		o[0] = r0;
		o[1] = r4;
		o[2] = r8;
		o[3] = 0;
		o[4] = r1;
		o[5] = r5;
		o[6] = r9;
		o[7] = 0;
		o[8] = r2;
		o[9] = r6;
		o[10] = r10;
		o[11] = 0;
		o[12] = -(r0 * tx + r1 * ty + r2 * tz);
		o[13] = -(r4 * tx + r5 * ty + r6 * tz);
		o[14] = -(r8 * tx + r9 * ty + r10 * tz);
		o[15] = 1;
		return o;
	}

	function identity4() {
		const m = new Float32Array(16);
		m[0] = m[5] = m[10] = m[15] = 1;
		return m;
	}

	function transformVec3(m, x, y, z) {
		const nx = m[0] * x + m[4] * y + m[8] * z + m[12];
		const ny = m[1] * x + m[5] * y + m[9] * z + m[13];
		const nz = m[2] * x + m[6] * y + m[10] * z + m[14];
		return [nx, ny, nz];
	}

	function addTfEdges(msg) {
		const arr = msg && msg.transforms;
		if (!Array.isArray(arr)) return;
		for (let i = 0; i < arr.length; i++) {
			const tr = arr[i];
			const parent = canonicalFrame(tr && tr.header && tr.header.frame_id);
			const child = canonicalFrame(tr && tr.child_frame_id);
			const tf = tr && tr.transform;
			const t = tf && tf.translation;
			const r = tf && tf.rotation;
			if (!parent || !child || !t || !r) continue;
			const R = quatToMat4(Number(r.x), Number(r.y), Number(r.z), Number(r.w));
			setTranslate(R, Number(t.x), Number(t.y), Number(t.z));
			const inv = invertRigid(R);
			/* p_parent = R * p_child */
			edges.set(edgeKey(child, parent), R);
			edges.set(edgeKey(parent, child), inv);
		}
	}

	/** p_target = T * p_source */
	function lookupTransform(target, source) {
		const tgt = canonicalFrame(target);
		const src = canonicalFrame(source);
		if (!tgt || !src) return null;
		if (tgt === src) return identity4();
		const q = [{ f: src, T: identity4() }];
		const seen = new Set([src]);
		let qi = 0;
		while (qi < q.length) {
			const cur = q[qi++];
			if (cur.f === tgt) return cur.T;
			for (const [k, mat] of edges) {
				const nl = k.indexOf('\n');
				if (nl < 0) continue;
				const from = k.slice(0, nl);
				const to = k.slice(nl + 1);
				if (from !== cur.f || seen.has(to)) continue;
				seen.add(to);
				/* p_to = mat * p_from，p_from = cur.T * p_source → p_to = mat * cur.T * p_source */
				q.push({ f: to, T: multiplyMat4(mat, cur.T) });
			}
		}
		return null;
	}

	function parseCameraInfo(msg) {
		if (!msg || !msg.k || !Array.isArray(msg.k) || msg.k.length < 9) {
			return;
		}
		const fid = canonicalFrame(msg.header && msg.header.frame_id);
		camOpticalFrame = fid;
		latestK = new Float32Array(9);
		for (let i = 0; i < 9; i++) latestK[i] = Number(msg.k[i]);
	}

	function fieldOffset(fields, name) {
		if (!Array.isArray(fields)) return -1;
		for (let i = 0; i < fields.length; i++) {
			if (fields[i] && fields[i].name === name) return fields[i].offset | 0;
		}
		return -1;
	}

	/** sensor_msgs/PointField.datatype（ROS 2） */
	const PF_UINT8 = 2;
	const PF_UINT32 = 6;
	const PF_FLOAT32 = 7;

	function findPcField(fields, name) {
		if (!Array.isArray(fields)) return null;
		for (let i = 0; i < fields.length; i++) {
			const f = fields[i];
			if (f && f.name === name) return f;
		}
		return null;
	}

	function unpackRgbPacked32(packed) {
		const u = packed >>> 0;
		return [(u >> 16) & 255, (u >> 8) & 255, u & 255];
	}

	/**
	 * 读取单点颜色；支持 FLOAT32/UINT32 打包 rgb、UINT8 的 rgb 连续区、分字段 r/g/b。
	 * @returns {number[]|null} [r,g,b] 0–255
	 */
	function readPointColor(fields, dv, base, le) {
		const fr = findPcField(fields, 'r');
		const fg = findPcField(fields, 'g');
		const fb = findPcField(fields, 'b');
		if (fr && fg && fb) {
			const dt = fr.datatype | 0;
			const oR = fr.offset | 0;
			const oG = fg.offset | 0;
			const oB = fb.offset | 0;
			if (dt === PF_UINT8) {
				return [dv.getUint8(base + oR), dv.getUint8(base + oG), dv.getUint8(base + oB)];
			}
			if (dt === PF_FLOAT32) {
				const r = Math.round(Math.max(0, Math.min(1, dv.getFloat32(base + oR, le))) * 255);
				const g = Math.round(Math.max(0, Math.min(1, dv.getFloat32(base + oG, le))) * 255);
				const b = Math.round(Math.max(0, Math.min(1, dv.getFloat32(base + oB, le))) * 255);
				return [r, g, b];
			}
		}
		for (let ni = 0; ni < 2; ni++) {
			const nm = ni === 0 ? 'rgb' : 'rgba';
			const f = findPcField(fields, nm);
			if (!f) continue;
			const off = f.offset | 0;
			const dt = f.datatype | 0;
			if (dt === PF_FLOAT32) {
				const packed = dv.getFloat32(base + off, le) >>> 0;
				return unpackRgbPacked32(packed);
			}
			if (dt === PF_UINT32) {
				return unpackRgbPacked32(dv.getUint32(base + off, le));
			}
			if (dt === PF_UINT8 && (f.count | 0) >= 3) {
				return [dv.getUint8(base + off), dv.getUint8(base + off + 1), dv.getUint8(base + off + 2)];
			}
		}
		return null;
	}

	function getImageChromaBuffer(img, nw, nh) {
		if (!img || !img.complete || nw < 2 || nh < 2) return null;
		if (!_ivgChromaCv) _ivgChromaCv = document.createElement('canvas');
		if (_ivgChromaCv.width !== nw || _ivgChromaCv.height !== nh) {
			_ivgChromaCv.width = nw;
			_ivgChromaCv.height = nh;
		}
		const tcx = _ivgChromaCv.getContext('2d', { willReadFrequently: true });
		if (!tcx) return null;
		tcx.imageSmoothingEnabled = true;
		tcx.imageSmoothingQuality = 'high';
		tcx.drawImage(img, 0, 0, nw, nh);
		try {
			return tcx.getImageData(0, 0, nw, nh);
		} catch (e) {
			return null;
		}
	}

	function sampleChromaFromImageData(idata, u, v, nw, nh) {
		const x = Math.max(0, Math.min(nw - 1.000001, u));
		const y = Math.max(0, Math.min(nh - 1.000001, v));
		const x0 = Math.floor(x);
		const y0 = Math.floor(y);
		const x1 = Math.min(nw - 1, x0 + 1);
		const y1 = Math.min(nh - 1, y0 + 1);
		const fx = x - x0;
		const fy = y - y0;
		const d = idata.data;
		const o = (ix, iy) => ((iy * nw + ix) | 0) * 4;
		const i00 = o(x0, y0);
		const i10 = o(x1, y0);
		const i01 = o(x0, y1);
		const i11 = o(x1, y1);
		const lerp = (a, b, t) => a + (b - a) * t;
		const ch = c =>
			Math.max(
				0,
				Math.min(
					255,
					Math.round(lerp(lerp(d[i00 + c], d[i10 + c], fx), lerp(d[i01 + c], d[i11 + c], fx), fy))
				)
			);
		return [ch(0), ch(1), ch(2)];
	}

	/**
	 * depth_registered 点云常与彩色像素对齐，但部分驱动仍标 camera_*_depth_optical_frame；
	 * 浏览器侧若未收到相机静 TF，lookup 会断。此时用 I 将点当作已在 color optical 下投影。
	 */
	function registeredSameCameraOpticalIdentity(pcFrame, colorOpticalFrame, topicPath) {
		const tp = String(topicPath || '');
		if (tp.indexOf('depth_registered') < 0) return false;
		const a = canonicalFrame(pcFrame);
		const b = canonicalFrame(colorOpticalFrame);
		const ds = '_depth_optical_frame';
		const cs = '_color_optical_frame';
		if (!a.endsWith(ds) || !b.endsWith(cs)) return false;
		return a.slice(0, -ds.length) === b.slice(0, -cs.length);
	}

	function parsePointCloud2(msg) {
		const Rb = global.IVGRosbridgeBytes;
		const u8 =
			Rb && typeof Rb.toUint8 === 'function' ? Rb.toUint8(msg.data) : bytesToUint8(msg.data);
		if (!u8 || !msg.fields) {
			return;
		}
		const xo = fieldOffset(msg.fields, 'x');
		const yo = fieldOffset(msg.fields, 'y');
		const zo = fieldOffset(msg.fields, 'z');
		if (xo < 0 || yo < 0 || zo < 0) {
			return;
		}
		const ps = msg.point_step | 0;
		const total = (msg.width * msg.height) | 0;
		if (ps <= 0 || total <= 0) {
			return;
		}
		const le = msg.is_bigendian !== true;
		const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
		const maxPts = PC_MAX_SAMPLES;
		let stride = 1;
		if (total > maxPts) stride = Math.ceil(total / maxPts);
		const n = Math.min(maxPts, Math.ceil(total / stride));
		const out = new Float32Array(n * 3);
		const baseProbe = 0;
		const hasMsgColor =
			baseProbe + ps <= u8.byteLength && readPointColor(msg.fields, dv, baseProbe, le) != null;
		const rgbBytes = hasMsgColor ? new Uint8Array(n * 3) : null;
		let oi = 0;
		for (let i = 0; i < n; i++) {
			const base = i * stride * ps;
			if (base + ps > u8.byteLength) break;
			out[oi++] = dv.getFloat32(base + xo, le);
			out[oi++] = dv.getFloat32(base + yo, le);
			out[oi++] = dv.getFloat32(base + zo, le);
			if (rgbBytes) {
				const j = oi - 3;
				const col = readPointColor(msg.fields, dv, base, le);
				if (col) {
					rgbBytes[j] = col[0];
					rgbBytes[j + 1] = col[1];
					rgbBytes[j + 2] = col[2];
				} else {
					rgbBytes[j] = 210;
					rgbBytes[j + 1] = 210;
					rgbBytes[j + 2] = 214;
				}
			}
		}
		lastPc = {
			frame: canonicalFrame(msg.header && msg.header.frame_id),
			pts: out.subarray(0, oi),
			rgb: rgbBytes && oi > 0 ? rgbBytes.subarray(0, oi) : null
		};
	}

	function parseMarkerArray(msg) {
		const arr = msg && (msg.markers || msg);
		if (!Array.isArray(arr)) {
			return;
		}
		const out = [];
		for (let i = 0; i < arr.length; i++) {
			const mk = arr[i];
			if (!mk) continue;
			if ((mk.action | 0) === MARKER_ACTION_DELETEALL) {
				out.length = 0;
				continue;
			}
			if ((mk.type | 0) !== MARKER_CYLINDER) continue;
			if ((mk.action | 0) !== 0) continue;
			const hdr = mk.header || {};
			const fid = canonicalFrame(hdr.frame_id);
			const p = mk.pose && mk.pose.position;
			const q = mk.pose && mk.pose.orientation;
			const sc = mk.scale || {};
			if (!p || !q) continue;
			const R = quatToMat4(Number(q.x), Number(q.y), Number(q.z), Number(q.w));
			setTranslate(R, Number(p.x), Number(p.y), Number(p.z));
			const hz = Number(sc.z) || 0.01;
			const c = mk.color || {};
			out.push({
				frame: fid,
				mat: R,
				halfZ: hz * 0.5,
				r: Number(c.r) != null ? Number(c.r) : 1,
				g: Number(c.g) != null ? Number(c.g) : 1,
				b: Number(c.b) != null ? Number(c.b) : 1,
				a: c.a != null ? Number(c.a) : 1
			});
		}
		lastMarkers = out;
	}

	function imageLetterbox(img) {
		const nw = img.naturalWidth || 640;
		const nh = img.naturalHeight || 480;
		const cw = img.clientWidth || nw;
		const ch = img.clientHeight || nh;
		const scale = Math.min(cw / nw, ch / nh);
		const dw = nw * scale;
		const dh = nh * scale;
		const ox = (cw - dw) * 0.5;
		const oy = (ch - dh) * 0.5;
		return { nw, nh, cw, ch, scale, ox, oy, dw, dh };
	}

	function projectPinhole(x, y, z, K) {
		if (!(z > 1e-6)) return null;
		const fx = K[0];
		const fy = K[4];
		const cx = K[2];
		const cy = K[5];
		const u = fx * (x / z) + cx;
		const v = fy * (y / z) + cy;
		return [u, v];
	}

	function mapPixelToCanvas(u, v, box) {
		return [box.ox + u * box.scale, box.oy + v * box.scale];
	}

	function strokeSegmentInGridFrame(ctx, T_cam_from_grid, ax, ay, az, bx, by, bz, K, box) {
		const pa = transformVec3(T_cam_from_grid, ax, ay, az);
		const pb = transformVec3(T_cam_from_grid, bx, by, bz);
		if (pa[2] <= 1e-4 || pb[2] <= 1e-4) return;
		const ua = projectPinhole(pa[0], pa[1], pa[2], K);
		const ub = projectPinhole(pb[0], pb[1], pb[2], K);
		if (!ua || !ub) return;
		const c0 = mapPixelToCanvas(ua[0], ua[1], box);
		const c1 = mapPixelToCanvas(ub[0], ub[1], box);
		ctx.beginPath();
		ctx.moveTo(c0[0], c0[1]);
		ctx.lineTo(c1[0], c1[1]);
		ctx.stroke();
	}

	/** 在相机前可视时绘制：z=0 网格 + 原点 RGB 轴 + 系名 */
	function drawRvizStyleGridAndAxes(ctx, targetCamFrame, K, box) {
		if (!targetCamFrame) return;
		let T = null;
		let gridFrame = '';
		for (let i = 0; i < GRID_FRAME_CANDIDATES.length; i++) {
			const fid = GRID_FRAME_CANDIDATES[i];
			const Ti = lookupTransform(targetCamFrame, fid);
			if (Ti) {
				T = Ti;
				gridFrame = fid;
				break;
			}
		}
		if (!T) return;

		ctx.save();
		ctx.strokeStyle = 'rgba(175, 188, 210, 0.45)';
		ctx.lineWidth = 1;
		const E = GRID_EXTENT_M;
		const S = GRID_STEP_M;
		let g;
		for (g = -E; g <= E + 1e-9; g += S) {
			strokeSegmentInGridFrame(ctx, T, -E, g, 0, E, g, 0, K, box);
		}
		for (g = -E; g <= E + 1e-9; g += S) {
			strokeSegmentInGridFrame(ctx, T, g, -E, 0, g, E, 0, K, box);
		}

		const L = AXIS_LEN_M;
		ctx.lineWidth = 2.25;
		ctx.strokeStyle = 'rgba(255, 75, 85, 0.95)';
		strokeSegmentInGridFrame(ctx, T, 0, 0, 0, L, 0, 0, K, box);
		ctx.strokeStyle = 'rgba(65, 220, 105, 0.95)';
		strokeSegmentInGridFrame(ctx, T, 0, 0, 0, 0, L, 0, K, box);
		ctx.strokeStyle = 'rgba(85, 155, 255, 0.95)';
		strokeSegmentInGridFrame(ctx, T, 0, 0, 0, 0, 0, L, K, box);

		const O = transformVec3(T, 0, 0, 0);
		if (O[2] > 1e-4) {
			const uo = projectPinhole(O[0], O[1], O[2], K);
			if (uo && uo[0] >= -40 && uo[0] <= box.nw + 40 && uo[1] >= -40 && uo[1] <= box.nh + 40) {
				const co = mapPixelToCanvas(uo[0], uo[1], box);
				ctx.font = '600 11px ui-sans-serif,system-ui,sans-serif';
				ctx.lineWidth = 3;
				ctx.strokeStyle = 'rgba(0,0,0,0.55)';
				ctx.fillStyle = 'rgba(255,255,255,0.92)';
				const label = gridFrame;
				ctx.strokeText(label, co[0] + 5, co[1] - 5);
				ctx.fillText(label, co[0] + 5, co[1] - 5);
			}
		}
		ctx.restore();
	}

	function draw(img, canvas) {
		drawPending = false;
		if (!img || !canvas || img.hidden || !latestK) {
			if (canvas && canvas.getContext) {
				const c = canvas.getContext('2d');
				if (c) c.clearRect(0, 0, canvas.width, canvas.height);
			}
			return;
		}
		const ctx = canvas.getContext('2d');
		if (!ctx) return;
		ctx.imageSmoothingEnabled = false;
		const box = imageLetterbox(img);
		if (box.cw < 2 || box.ch < 2) {
			return;
		}
		if (canvas.width !== box.cw || canvas.height !== box.ch) {
			canvas.width = box.cw;
			canvas.height = box.ch;
		}
		ctx.clearRect(0, 0, box.cw, box.ch);
		const K = latestK;
		const target = camOpticalFrame;

		drawRvizStyleGridAndAxes(ctx, target, K, box);

		if (lastPc && lastPc.pts && lastPc.pts.length >= 3 && target) {
			let T = lookupTransform(target, lastPc.frame);
			if (!T && canonicalFrame(target) === canonicalFrame(lastPc.frame)) T = identity4();
			if (
				!T &&
				registeredSameCameraOpticalIdentity(lastPc.frame, target, lastPcSubscribeTopic)
			) {
				T = identity4();
			}
			if (T) {
				const pts = lastPc.pts;
				const pcRgb = lastPc.rgb;
				const step = 3;
				const h = PC_DOT_HALF;
				const s = h + h;
				const neutralFill = 'rgba(232, 232, 236, 0.92)';
				const useMsgRgb = pcRgb && pcRgb.length >= pts.length;
				const chromaId =
					!useMsgRgb && img && img.complete && img.naturalWidth >= 2
						? getImageChromaBuffer(img, box.nw, box.nh)
						: null;
				if (!useMsgRgb && !chromaId) ctx.fillStyle = neutralFill;
				for (let i = 0; i + 2 < pts.length; i += step) {
					const wx = pts[i];
					const wy = pts[i + 1];
					const wz = pts[i + 2];
					const p = transformVec3(T, wx, wy, wz);
					const uv = projectPinhole(p[0], p[1], p[2], K);
					if (!uv) continue;
					if (uv[0] < 0 || uv[0] > box.nw || uv[1] < 0 || uv[1] > box.nh) continue;
					const cc = mapPixelToCanvas(uv[0], uv[1], box);
					if (useMsgRgb) {
						ctx.fillStyle = `rgba(${pcRgb[i]},${pcRgb[i + 1]},${pcRgb[i + 2]},0.94)`;
					} else if (chromaId) {
						const rgb = sampleChromaFromImageData(chromaId, uv[0], uv[1], box.nw, box.nh);
						ctx.fillStyle = `rgba(${rgb[0]},${rgb[1]},${rgb[2]},0.93)`;
					}
					const px = Math.round(cc[0] - h);
					const py = Math.round(cc[1] - h);
					const pw = Math.max(1, Math.round(s));
					const ph = Math.max(1, Math.round(s));
					ctx.fillRect(px, py, pw, ph);
				}
			}
		}

		if (lastMarkers && lastMarkers.length && target) {
			for (let mi = 0; mi < lastMarkers.length; mi++) {
				const mk = lastMarkers[mi];
				let Tm = lookupTransform(target, mk.frame);
				if (!Tm && canonicalFrame(target) === canonicalFrame(mk.frame)) Tm = identity4();
				if (!Tm) continue;
				const M = multiplyMat4(Tm, mk.mat);
				const hz = mk.halfZ;
				const a0 = transformVec3(M, 0, 0, -hz);
				const a1 = transformVec3(M, 0, 0, hz);
				const uv0 = projectPinhole(a0[0], a0[1], a0[2], K);
				const uv1 = projectPinhole(a1[0], a1[1], a1[2], K);
				if (!uv0 || !uv1) continue;
				const c0 = mapPixelToCanvas(uv0[0], uv0[1], box);
				const c1 = mapPixelToCanvas(uv1[0], uv1[1], box);
				ctx.strokeStyle = `rgba(${Math.round(mk.r * 255)},${Math.round(mk.g * 255)},${Math.round(
					mk.b * 255
				)},${Math.min(1, mk.a)})`;
				ctx.lineWidth = 2.75;
				ctx.beginPath();
				ctx.moveTo(c0[0], c0[1]);
				ctx.lineTo(c1[0], c1[1]);
				ctx.stroke();
			}
		}
	}

	function scheduleDraw(img, canvas) {
		if (drawPending) return;
		drawPending = true;
		requestAnimationFrame(() => draw(img, canvas));
	}

	const api = {
		reset() {
			edges.clear();
			latestK = null;
			camOpticalFrame = '';
			lastPc = null;
			lastMarkers = null;
			lastPcSubscribeTopic = '';
			drawPending = false;
		},

		onTf(msg) {
			addTfEdges(msg);
		},

		onTfStatic(msg) {
			addTfEdges(msg);
		},

		onCameraInfo(msg) {
			parseCameraInfo(msg);
		},

		onPointCloud2(msg) {
			parsePointCloud2(msg);
		},

		onMarkerArray(msg) {
			parseMarkerArray(msg);
		},

		scheduleDraw,

		/**
		 * @param {*} transport ivgTransport
		 * @param {object} topics normalized absolute names
		 * @param {HTMLElement} imgEl
		 * @param {HTMLCanvasElement} canvasEl
		 */
		registerSubscriptions(transport, topics, imgEl, canvasEl) {
			const t = topics || {};
			const pc = t.pc;
			const mk = t.markers;
			const ci = t.camInfo;
			const tf = t.tf || '/tf';
			const tfs = t.tfStatic || '/tf_static';
			lastPcSubscribeTopic = pc ? String(pc) : '';
			if (pc) {
				transport.onRosJson(pc, m => {
					api.onPointCloud2(m);
					api.scheduleDraw(imgEl, canvasEl);
				});
				transport.subscribe({
					topic: pc,
					msgType: 'sensor_msgs/msg/PointCloud2',
					maxHz: 12,
					maxPoints: PC_BRIDGE_MAX_POINTS
				});
			}
			if (mk) {
				transport.onRosJson(mk, m => {
					api.onMarkerArray(m);
					api.scheduleDraw(imgEl, canvasEl);
				});
				transport.subscribe({ topic: mk, msgType: 'visualization_msgs/msg/MarkerArray', maxHz: 12 });
			}
			if (ci) {
				transport.onRosJson(ci, m => {
					api.onCameraInfo(m);
					api.scheduleDraw(imgEl, canvasEl);
				});
				transport.subscribe({ topic: ci, msgType: 'sensor_msgs/msg/CameraInfo', maxHz: 2 });
			}
			if (tf) {
				transport.onRosJson(tf, m => {
					api.onTf(m);
					api.scheduleDraw(imgEl, canvasEl);
				});
				transport.subscribe({ topic: tf, msgType: 'tf2_msgs/msg/TFMessage', maxHz: 8 });
			}
			if (tfs) {
				transport.onRosJson(tfs, m => {
					api.onTfStatic(m);
					api.scheduleDraw(imgEl, canvasEl);
				});
				transport.subscribe({ topic: tfs, msgType: 'tf2_msgs/msg/TFMessage', maxHz: 2 });
			}
		}
	};

	global.IVGGraspnetOverlay = api;
})(typeof window !== 'undefined' ? window : globalThis);
