/**
 * topics_lab 可视化渲染器：
 * - 输入消息类型、消息体和可选 canvas。
 * - 输出 HTML 片段和是否使用 canvas。
 * - 不管理订阅、节流和 DOM 挂载时机。
 */
/* global IVGRosbridgeBytes, IVGTopicsLabRenderPreview */
(function (global) {
	'use strict';

	const preview = global.IVGTopicsLabRenderPreview;
	if (!preview) throw new Error('IVGTopicsLabRenderPreview 未加载');

	const safeJson = preview.safeJson;
	const typeMatch = preview.typeMatch;

	const VIZ_CANVAS_BG = '#e8ecf2';
	const LASER_STROKE = '#15803d';
	const LASER_ARC_STROKE = '#94a3b8';

	function renderScalar(msg) {
		if (msg && Object.prototype.hasOwnProperty.call(msg, 'data')) {
			const d = msg.data;
			if (typeof d === 'boolean' || typeof d === 'number' || typeof d === 'string') {
				return `<div class="viz-scalar">${String(d)}</div>`;
			}
		}
		return null;
	}

	function renderTwist(msg) {
		let t = msg;
		if (msg && msg.twist) t = msg.twist;
		if (!t || !t.linear) return null;
		const L = t.linear;
		const A = t.angular || {};
		const rows = [
			['linear.x', L.x], ['linear.y', L.y], ['linear.z', L.z],
			['angular.x', A.x], ['angular.y', A.y], ['angular.z', A.z]
		];
		let html = '<table class="viz-table"><tbody>';
		for (let i = 0; i < rows.length; i++) {
			html += `<tr><th>${rows[i][0]}</th><td>${rows[i][1]}</td></tr>`;
		}
		html += '</tbody></table>';
		return html;
	}

	function renderPose(msg) {
		let p = msg;
		if (msg && msg.pose) p = msg.pose;
		if (!p || !p.position) return null;
		const pos = p.position;
		const o = p.orientation || {};
		let html = '<table class="viz-table"><tbody>';
		html += `<tr><th>position</th><td>x=${pos.x} y=${pos.y} z=${pos.z}</td></tr>`;
		html += `<tr><th>orientation</th><td>x=${o.x} y=${o.y} z=${o.z} w=${o.w}</td></tr>`;
		html += '</tbody></table>';
		return html;
	}

	function renderJointState(msg) {
		if (!msg || !Array.isArray(msg.name)) return null;
		let html = '<table class="viz-table"><thead><tr><th>name</th><th>position</th><th>velocity</th><th>effort</th></tr></thead><tbody>';
		const n = msg.name.length;
		for (let i = 0; i < n; i++) {
			html += `<tr><td>${msg.name[i]}</td><td>${msg.position && msg.position[i] !== undefined ? msg.position[i] : ''}</td><td>${msg.velocity && msg.velocity[i] !== undefined ? msg.velocity[i] : ''}</td><td>${msg.effort && msg.effort[i] !== undefined ? msg.effort[i] : ''}</td></tr>`;
		}
		html += '</tbody></table>';
		return html;
	}

	function renderImu(msg) {
		if (!msg) return null;
		let html = '<table class="viz-table"><tbody>';
		if (msg.orientation) {
			const o = msg.orientation;
			html += `<tr><th>orientation</th><td>x=${o.x} y=${o.y} z=${o.z} w=${o.w}</td></tr>`;
		}
		if (msg.angular_velocity) {
			const av = msg.angular_velocity;
			html += `<tr><th>angular_velocity</th><td>x=${av.x} y=${av.y} z=${av.z}</td></tr>`;
		}
		if (msg.linear_acceleration) {
			const la = msg.linear_acceleration;
			html += `<tr><th>linear_acceleration</th><td>x=${la.x} y=${la.y} z=${la.z}</td></tr>`;
		}
		html += '</tbody></table>';
		return html;
	}

	function renderBattery(msg) {
		if (!msg) return null;
		let html = '<table class="viz-table"><tbody>';
		const keys = ['voltage', 'temperature', 'current', 'charge', 'capacity', 'percentage', 'power_supply_status'];
		for (let i = 0; i < keys.length; i++) {
			if (Object.prototype.hasOwnProperty.call(msg, keys[i])) {
				html += `<tr><th>${keys[i]}</th><td>${msg[keys[i]]}</td></tr>`;
			}
		}
		html += '</tbody></table>';
		return html;
	}

	function renderRange(msg) {
		if (!msg || typeof msg.range !== 'number') return null;
		let html = '<table class="viz-table"><tbody>';
		html += `<tr><th>range</th><td>${msg.range}</td></tr>`;
		if (msg.min_range !== undefined) html += `<tr><th>min_range</th><td>${msg.min_range}</td></tr>`;
		if (msg.max_range !== undefined) html += `<tr><th>max_range</th><td>${msg.max_range}</td></tr>`;
		if (msg.radiation_type !== undefined) html += `<tr><th>radiation_type</th><td>${msg.radiation_type}</td></tr>`;
		html += '</tbody></table>';
		return html;
	}

	function renderNavSatFix(msg) {
		if (!msg || typeof msg.latitude !== 'number') return null;
		let html = '<table class="viz-table"><tbody>';
		html += `<tr><th>latitude</th><td>${msg.latitude}</td></tr>`;
		html += `<tr><th>longitude</th><td>${msg.longitude}</td></tr>`;
		html += `<tr><th>altitude</th><td>${msg.altitude}</td></tr>`;
		if (msg.status) html += `<tr><th>status</th><td>${safeJson(msg.status, 400)}</td></tr>`;
		html += '</tbody></table>';
		return html;
	}

	function renderLaserScan(msg, canvas) {
		if (!msg || !Array.isArray(msg.ranges)) return '<p class="hint">无效的 LaserScan</p>';
		const w = canvas.parentElement.clientWidth || 400;
		const h = Math.min(360, Math.floor(w * 0.6));
		canvas.width = w;
		canvas.height = h;
		const ctx = canvas.getContext('2d');
		ctx.fillStyle = VIZ_CANVAS_BG;
		ctx.fillRect(0, 0, w, h);
		const cx = w * 0.5;
		const cy = h * 0.92;
		const maxR = Math.min(cx, cy) * 0.95;
		const amin = msg.angle_min;
		const inc = msg.angle_increment;
		const ranges = msg.ranges;
		ctx.strokeStyle = LASER_STROKE;
		ctx.lineWidth = 1;
		ctx.beginPath();
		for (let i = 0; i < ranges.length; i++) {
			const r = ranges[i];
			if (!isFinite(r) || r <= 0 || r > 1e6) continue;
			const ang = amin + i * inc;
			const px = cx + r / (msg.range_max || 10) * maxR * Math.cos(ang);
			const py = cy - r / (msg.range_max || 10) * maxR * Math.sin(ang);
			if (i === 0) ctx.moveTo(px, py);
			else ctx.lineTo(px, py);
		}
		ctx.stroke();
		ctx.strokeStyle = LASER_ARC_STROKE;
		ctx.beginPath();
		ctx.arc(cx, cy, maxR, Math.PI, 2 * Math.PI);
		ctx.stroke();
		return '<p class="hint">LaserScan 极坐标预览（简化，非 RViz 语义）</p>';
	}

	function renderOccupancyGrid(msg, canvas) {
		if (!msg || !msg.info || !Array.isArray(msg.data)) return '<p class="hint">Invalid OccupancyGrid</p>';
		const W = msg.info.width;
		const H = msg.info.height;
		if (W * H !== msg.data.length) {
			return '<p class="hint">Grid size mismatch (width×height vs data)</p>';
		}
		const maxW = canvas.parentElement.clientWidth || 400;
		const scale = Math.max(1, Math.floor(maxW / W));
		canvas.width = W * scale;
		canvas.height = H * scale;
		const ctx = canvas.getContext('2d');
		const img = ctx.createImageData(W, H);
		const d = img.data;
		for (let i = 0; i < msg.data.length; i++) {
			const v = msg.data[i];
			const o = i * 4;
			if (v < 0) { d[o] = 80; d[o + 1] = 80; d[o + 2] = 120; d[o + 3] = 255; }
			else if (v === 0) { d[o] = 255; d[o + 1] = 255; d[o + 2] = 255; d[o + 3] = 255; }
			else {
				const g = 255 - Math.min(255, v * 2);
				d[o] = g; d[o + 1] = g; d[o + 2] = g; d[o + 3] = 255;
			}
		}
		const tmp = document.createElement('canvas');
		tmp.width = W;
		tmp.height = H;
		tmp.getContext('2d').putImageData(img, 0, 0);
		ctx.imageSmoothingEnabled = false;
		ctx.drawImage(tmp, 0, 0, W * scale, H * scale);
		return `<p class="hint">resolution ${msg.info.resolution} m/cell · origin (${msg.info.origin.position.x}, ${msg.info.origin.position.y})</p>`;
	}

	function renderCompressedImage(msg) {
		if (!msg || !msg.format || !msg.data) return null;
		const u8 = IVGRosbridgeBytes.toUint8(msg.data);
		if (!u8) return '<p class="hint">无法解析压缩图像数据</p>';
		if (window.__labBlobUrl) {
			try { URL.revokeObjectURL(window.__labBlobUrl); } catch (e) { /* ignore */ }
		}
		const blob = new Blob([u8], { type: `image/${msg.format.indexOf('png') !== -1 ? 'png' : 'jpeg'}` });
		window.__labBlobUrl = URL.createObjectURL(blob);
		return `<img src="${window.__labBlobUrl}" alt="compressed" style="max-width:100%;height:auto;border-radius:4px"/>`;
	}

	function renderPointCloud2(msg) {
		if (!msg || !msg.fields) return null;
		let rows = '<table class="viz-table"><tbody>';
		rows += `<tr><th>height × width</th><td>${msg.height} × ${msg.width}</td></tr>`;
		rows += `<tr><th>point_step</th><td>${msg.point_step}</td></tr>`;
		rows += `<tr><th>row_step</th><td>${msg.row_step}</td></tr>`;
		rows += `<tr><th>fields</th><td>${msg.fields.map(f => f.name + ' (' + f.datatype + '×' + f.count + ')').join(', ')}</td></tr>`;
		if (msg.header && msg.header.frame_id) {
			rows += `<tr><th>frame_id</th><td>${msg.header.frame_id}</td></tr>`;
		}
		rows += '</tbody></table>';
		const raw = IVGRosbridgeBytes.toUint8(msg.data);
		const bytes = raw ? raw.length : 0;
		rows += `<p class="hint">Payload ~${bytes} bytes。三维显示请用「3D 图」+ ros3djs；彩色图可经 <a href="https://github.com/RobotWebTools/web_video_server" target="_blank" rel="noopener">web_video_server</a> MJPEG。快捷条「3D: 点云」默认 <code>/camera/depth_registered/points</code>。</p>`;
		return rows;
	}

	function renderPath(msg) {
		if (!msg || !Array.isArray(msg.poses)) return null;
		const n = msg.poses.length;
		if (n === 0) return '<p class="hint">Empty path</p>';
		const last = msg.poses[n - 1];
		const p = last.pose ? last.pose.position : null;
		return `<p class="hint">poses: ${n}</p>${p ? renderPose(last.pose) : ''}`;
	}

	function renderPoseArray(msg) {
		if (!msg || !Array.isArray(msg.poses)) return null;
		const poses = msg.poses;
		const n = poses.length;
		const fid = (msg.header && msg.header.frame_id) ? msg.header.frame_id : '';
		const maxShow = Math.min(12, n);
		let html = `<p class="hint">poses: ${n}${fid ? ` · frame <code>${fid}</code>` : ''}</p>`;
		html += '<table class="viz-table"><thead><tr><th>#</th><th>x</th><th>y</th><th>z</th><th>qx</th><th>qy</th><th>qz</th><th>qw</th></tr></thead><tbody>';
		for (let i = 0; i < maxShow; i++) {
			const po = poses[i];
			const pos = po.position || {};
			const ori = po.orientation || {};
			html += `<tr><td>${i}</td><td>${pos.x}</td><td>${pos.y}</td><td>${pos.z}</td><td>${ori.x}</td><td>${ori.y}</td><td>${ori.z}</td><td>${ori.w}</td></tr>`;
		}
		html += '</tbody></table>';
		if (n > maxShow) html += `<p class="hint">… 其余 ${n - maxShow} 条见 JSON</p>`;
		return html;
	}

	function renderMarkerArray(msg) {
		if (!msg) return null;
		const arr = msg.markers || msg;
		if (!Array.isArray(arr)) return null;
		const n = arr.length;
		const maxShow = Math.min(12, n);
		let head = `<p class="hint">markers: ${n}（抓取可视化常用）</p>`;
		head += '<table class="viz-table"><thead><tr><th>#</th><th>id</th><th>type</th><th>ns</th><th>frame</th><th>xyz</th><th>scale</th></tr></thead><tbody>';
		for (let i = 0; i < maxShow; i++) {
			const m = arr[i];
			const pos = (m.pose && m.pose.position) ? m.pose.position : {};
			const sc = m.scale || {};
			head += `<tr><td>${i}</td><td>${m.id != null ? m.id : ''}</td><td>${m.type != null ? m.type : ''}</td><td>${m.ns || ''}</td><td>${(m.header && m.header.frame_id) || ''}</td><td>${Number(pos.x).toFixed(3)}, ${Number(pos.y).toFixed(3)}, ${Number(pos.z).toFixed(3)}</td><td>${Number(sc.x || 0).toFixed(2)}×${Number(sc.y || 0).toFixed(2)}×${Number(sc.z || 0).toFixed(2)}</td></tr>`;
		}
		head += '</tbody></table>';
		if (n > maxShow) head += '<p class="hint">… 其余见 JSON</p>';
		head += `<details class="viz-details"><summary class="viz-summary">首条原始 JSON</summary><pre class="raw">${safeJson(arr[0] || {}, 4000)}</pre></details>`;
		return head;
	}

	function renderOdometry(msg) {
		if (!msg || !msg.pose || !msg.twist) return null;
		return `<h4 class="viz-subheading">pose</h4>${renderPose(msg.pose)}<h4 class="viz-subheading viz-subheading--spaced">twist</h4>${renderTwist(msg.twist)}`;
	}

	function renderGenericNumericArray(msg) {
		if (!msg || !msg.layout || !Array.isArray(msg.data)) return null;
		const dim = (msg.layout.dim || []).map(d => `${d.label}=${d.size}`).join(', ');
		const sample = msg.data.slice(0, 32).join(', ');
		return `<p class="hint">${dim}</p><pre class="raw">data[0..31]: ${sample}${msg.data.length > 32 ? ' …' : ''}</pre>`;
	}

	function renderVisualization(msgType, msg, canvas) {
		if (typeMatch(msgType, 'LaserScan')) return { html: renderLaserScan(msg, canvas), usedCanvas: true };
		if (typeMatch(msgType, 'OccupancyGrid')) return { html: renderOccupancyGrid(msg, canvas), usedCanvas: true };
		if (typeMatch(msgType, 'CompressedImage')) return { html: renderCompressedImage(msg), usedCanvas: false };
		if (typeMatch(msgType, 'PointCloud2')) return { html: renderPointCloud2(msg), usedCanvas: false };
		if (typeMatch(msgType, 'Path')) return { html: renderPath(msg), usedCanvas: false };
		if (typeMatch(msgType, 'PoseArray')) {
			const pra = renderPoseArray(msg);
			if (pra) return { html: pra, usedCanvas: false };
		}
		if (typeMatch(msgType, 'MarkerArray')) return { html: renderMarkerArray(msg), usedCanvas: false };
		if (typeMatch(msgType, 'Marker') && !typeMatch(msgType, 'MarkerArray')) {
			return { html: `<pre class="raw">${safeJson(msg, 8000)}</pre>`, usedCanvas: false };
		}
		if (typeMatch(msgType, 'Odometry')) return { html: renderOdometry(msg), usedCanvas: false };
		if (typeMatch(msgType, 'JointState')) return { html: renderJointState(msg), usedCanvas: false };
		if (typeMatch(msgType, 'Imu')) return { html: renderImu(msg), usedCanvas: false };
		if (typeMatch(msgType, 'BatteryState')) return { html: renderBattery(msg), usedCanvas: false };
		if (typeMatch(msgType, 'Range')) {
			const rg = renderRange(msg);
			if (rg) return { html: rg, usedCanvas: false };
		}
		if (typeMatch(msgType, 'NavSatFix')) {
			const ns = renderNavSatFix(msg);
			if (ns) return { html: ns, usedCanvas: false };
		}
		if (typeMatch(msgType, 'Twist')) {
			const tw = renderTwist(msg);
			if (tw) return { html: tw, usedCanvas: false };
		}
		if (typeMatch(msgType, 'Pose')) {
			const ps = renderPose(msg);
			if (ps) return { html: ps, usedCanvas: false };
		}
		if (
			typeMatch(msgType, 'Float32MultiArray') ||
			typeMatch(msgType, 'Float64MultiArray') ||
			typeMatch(msgType, 'Int32MultiArray') ||
			typeMatch(msgType, 'Int8MultiArray') ||
			typeMatch(msgType, 'UInt8MultiArray') ||
			typeMatch(msgType, 'UInt16MultiArray')
		) {
			const ga = renderGenericNumericArray(msg);
			if (ga) return { html: ga, usedCanvas: false };
		}
		const sc = renderScalar(msg);
		if (sc) return { html: sc, usedCanvas: false };
		return {
			html: '<p class="hint">无专用视图；右侧为完整 JSON。可在 <code>js/topics_lab/render_visualizers.js</code> 中扩展。</p>',
			usedCanvas: false
		};
	}

	global.IVGTopicsLabRenderVisualizers = {
		renderVisualization
	};
})(typeof window !== 'undefined' ? window : globalThis);
