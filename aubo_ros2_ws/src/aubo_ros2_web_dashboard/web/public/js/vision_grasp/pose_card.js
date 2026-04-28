// pose_card.js — HTML renderers for end-effector / grasp pose cards
import { ivgQuatNormalize } from '../view3d/tf_clients.js';
function fmtPoseNum(v, digits) {
	const n = Number(v);
	return isFinite(n) ? n.toFixed(digits == null ? 4 : digits) : '--';
}
function numFirst(...vals) {
	for (let i = 0; i < vals.length; i++) {
		const n = Number(vals[i]);
		if (isFinite(n)) return n;
	}
	return NaN;
}
function escapeHtml(value) {
	return String(value == null ? '' : value)
		.replace(/&/g, '&amp;')
		.replace(/</g, '&lt;')
		.replace(/>/g, '&gt;')
		.replace(/"/g, '&quot;')
		.replace(/'/g, '&#39;');
}
function normalizeRobotStatusJson(raw) {
	if (!raw || typeof raw !== 'object' || Array.isArray(raw)) return raw;
	const hasFlat =
		raw.cartesian_position != null ||
		raw.cartesian_position_xyz != null ||
		raw.cartesian_rpy != null ||
		Array.isArray(raw.joint_position_deg) ||
		Array.isArray(raw.joint_position_rad);
	if (hasFlat) return raw;
	const inner = raw.msg || raw.data;
	if (inner && typeof inner === 'object' && !Array.isArray(inner)) return inner;
	return raw;
}
function poseToRpyDeg(pose) {
	if (!pose || !pose.orientation) return null;
	const q = ivgQuatNormalize({
		x: Number(pose.orientation.x) || 0,
		y: Number(pose.orientation.y) || 0,
		z: Number(pose.orientation.z) || 0,
		w: Number(pose.orientation.w) || 1
	});
	const sqx = q.x * q.x;
	const sqy = q.y * q.y;
	const sqz = q.z * q.z;
	const sqw = q.w * q.w;
	const clampY = Math.max(-1, Math.min(1, 2 * (q.x * q.z + q.y * q.w)));
	const eulerX = Math.atan2(2 * (q.x * q.w - q.y * q.z), sqw - sqx - sqy + sqz);
	const eulerY = Math.asin(clampY);
	const eulerZ = Math.atan2(2 * (q.z * q.w - q.x * q.y), sqw + sqx - sqy - sqz);
	const radToDeg = rad => rad * (180 / Math.PI);
	return {
		roll: radToDeg(eulerZ),
		pitch: radToDeg(eulerY),
		yaw: radToDeg(eulerX)
	};
}
function formatPoseBlockHtml(pose, rpyDeg) {
	if (!pose || !pose.position || !pose.orientation) {
		return '<div class="pose-card__empty">暂无位姿数据</div>';
	}
	const rpy = rpyDeg || poseToRpyDeg(pose);
	const px = fmtPoseNum(pose.position.x, 4);
	const py = fmtPoseNum(pose.position.y, 4);
	const pz = fmtPoseNum(pose.position.z, 4);
	const qx = fmtPoseNum(pose.orientation.x, 4);
	const qy = fmtPoseNum(pose.orientation.y, 4);
	const qz = fmtPoseNum(pose.orientation.z, 4);
	const qw = fmtPoseNum(pose.orientation.w, 4);
	const rpyRows =
		rpy != null
			? `<div class="pose-card__triple pose-card__triple--rpy">
					<div class="pose-card__pill pose-card__pill--rpy"><span class="pose-card__pill-key">R</span><span class="pose-card__pill-val">${fmtPoseNum(rpy.roll, 1)}°</span></div>
					<div class="pose-card__pill pose-card__pill--rpy"><span class="pose-card__pill-key">P</span><span class="pose-card__pill-val">${fmtPoseNum(rpy.pitch, 1)}°</span></div>
					<div class="pose-card__pill pose-card__pill--rpy"><span class="pose-card__pill-key">Y</span><span class="pose-card__pill-val">${fmtPoseNum(rpy.yaw, 1)}°</span></div>
				</div>`
			: '';
	return `<div class="pose-card__body">
			<section class="pose-card__section pose-card__section--pose6">
				<h3 class="pose-card__section-title">位姿 <span class="pose-card__unit-badge pose-card__unit-badge--muted" aria-hidden="true" title="位置 m，姿态角 °">m · °</span></h3>
				<div class="pose-card__triple">
					<div class="pose-card__pill pose-card__pill--pos"><span class="pose-card__pill-key">X</span><span class="pose-card__pill-val">${px}</span></div>
					<div class="pose-card__pill pose-card__pill--pos"><span class="pose-card__pill-key">Y</span><span class="pose-card__pill-val">${py}</span></div>
					<div class="pose-card__pill pose-card__pill--pos"><span class="pose-card__pill-key">Z</span><span class="pose-card__pill-val">${pz}</span></div>
				</div>
				${rpyRows}
			</section>
			<section class="pose-card__section pose-card__section--quat">
				<h3 class="pose-card__section-title">四元数 <span class="pose-card__unit-badge pose-card__unit-badge--muted" title="顺序 x y z w">x y z w</span></h3>
				<div class="pose-card__quad">
					<div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">X</span><span class="pose-card__pill-val">${qx}</span></div>
					<div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">Y</span><span class="pose-card__pill-val">${qy}</span></div>
					<div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">Z</span><span class="pose-card__pill-val">${qz}</span></div>
					<div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">W</span><span class="pose-card__pill-val">${qw}</span></div>
				</div>
			</section>
		</div>`;
}
function robotPoseHtmlIsRenderable(html) {
	return (
		typeof html === 'string' &&
		(html.indexOf('pose-card__body') !== -1 || html.indexOf('pose-card__body--ivg-text') !== -1)
	);
}
function formatRobotPoseHtml(msg) {
	msg = normalizeRobotStatusJson(msg);
	if (!msg || typeof msg !== 'object') return '<div class="pose-card__empty">等待末端位姿…</div>';
	const cp = msg.cartesian_position || {};
	const cpos = cp.position || {};
	const xyz = msg.cartesian_position_xyz || {};
	const x = numFirst(xyz.x, cpos.x);
	const y = numFirst(xyz.y, cpos.y);
	const z = numFirst(xyz.z, cpos.z);
	const ori = cp.orientation || {};
	const hasPos = [x, y, z].some(v => isFinite(v));
	const hasOri = [ori.x, ori.y, ori.z, ori.w].some(v => isFinite(Number(v)));
	const pose = {
		position: { x, y, z },
		orientation: {
			x: isFinite(Number(ori.x)) ? Number(ori.x) : 0,
			y: isFinite(Number(ori.y)) ? Number(ori.y) : 0,
			z: isFinite(Number(ori.z)) ? Number(ori.z) : 0,
			w: isFinite(Number(ori.w)) ? Number(ori.w) : 1
		}
	};
	const rpySrc = msg.cartesian_rpy || {};
	const hasPose = hasPos || hasOri;
	if (!hasPose) {
		if (msg.ivg_display != null && String(msg.ivg_display).trim()) {
			return `<div class="pose-card__body pose-card__body--ivg-text"><pre class="pose-card__ivg-pre">${escapeHtml(String(msg.ivg_display))}</pre></div>`;
		}
		return '<div class="pose-card__empty">等待末端位姿…</div>';
	}
	const rr = Number(rpySrc.x);
	const rp = Number(rpySrc.y);
	const ry = Number(rpySrc.z);
	const rpyDeg =
		[rr, rp, ry].every(v => isFinite(v))
			? { roll: rr, pitch: rp, yaw: ry }
			: poseToRpyDeg(pose);
	return formatPoseBlockHtml(pose, rpyDeg);
}
function formatFinalGraspPoseHtml(msg) {
	if (!msg || !Array.isArray(msg.poses) || msg.poses.length === 0) {
		return '<div class="pose-card__empty">等待 AI 抓取位姿…</div>';
	}
	const pose = msg.poses[0];
	return formatPoseBlockHtml(pose, null);
}
export {
	escapeHtml,
	robotPoseHtmlIsRenderable,
	formatRobotPoseHtml,
	formatFinalGraspPoseHtml
};
