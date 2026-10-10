// pose-card.js — 末端位姿 / AI 抓取位姿 HTML 格式化
// 供 vision-grasp / latte / monitor 共用喵~
//
// 合并自 vision_grasp/pose_card.js (RobotStatus 消息解析) +
// components/pose-card.js (扁平 pose 对象处理)
//
// 用法:
//   import { formatRobotPoseHtml, formatFinalGraspPoseHtml, robotPoseHtmlIsRenderable } from '../components/pose-card.js';

import { quatToRpyDeg } from '../core/tf-math.js';
import { escapeHtml as _escapeHtml } from '../core/utils.js';

// 重新导出 escapeHtml 供 latte/main.js 等外部调用者使用
export { _escapeHtml as escapeHtml };

// ── 内部工具 ─────────────────────────────────────────────────────────────────

function fmtPoseNum(v, digits) {
    const n = Number(v);
    return (isFinite(n) ? n : 0).toFixed(digits == null ? 3 : digits);
}

function numFirst(...vals) {
    for (let i = 0; i < vals.length; i++) {
        const n = Number(vals[i]);
        if (isFinite(n)) return n;
    }
    return NaN;
}

/** 解包 RobotStatus 嵌套结构 (cartesian_position / msg.data 等) 喵~ */
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

/** 从 pose.orientation {x,y,z,w} 计算 RPY ° (ZYX 外旋 = XYZ 内旋) 喵~ */
function poseOriToRpyDeg(pose) {
    if (!pose || !pose.orientation) return null;
    const qx = Number(pose.orientation.x) || 0;
    const qy = Number(pose.orientation.y) || 0;
    const qz = Number(pose.orientation.z) || 0;
    const qw = Number(pose.orientation.w) || 1;
    return quatToRpyDeg({ x: qx, y: qy, z: qz, w: qw });
}

/** 将 pose 对象格式化为富 HTML 卡片 喵~ */
function formatPoseBlockHtml(pose, rpyDeg) {
    if (!pose || !pose.position || !pose.orientation) {
        // 无数据时渲染全零位姿
        pose = {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 }
        };
        rpyDeg = { roll: 0, pitch: 0, yaw: 0 };
    }
    const rpy = rpyDeg || poseOriToRpyDeg(pose);
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
                <h3 class="pose-card__section-title">四元数 <span class="pose-card__unit-badge pose-card__unit-badge--muted" title="顺序 x y z w">x y z w</span></h3>
                <div class="pose-card__quad">
                    <div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">X</span><span class="pose-card__pill-val">${qx}</span></div>
                    <div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">Y</span><span class="pose-card__pill-val">${qy}</span></div>
                    <div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">Z</span><span class="pose-card__pill-val">${qz}</span></div>
                    <div class="pose-card__pill pose-card__pill--q"><span class="pose-card__pill-key">W</span><span class="pose-card__pill-val">${qw}</span></div>
                </div>
            </section>
        </div>`;
}

// ── 公开 API ─────────────────────────────────────────────────────────────────

/**
 * 判断位姿数据是否可渲染
 * - 传入字符串 (HTML): 检查是否包含 pose-card__body (vision_grasp 用法)
 * - 传入对象: 检查 x/y/z/qx/qy/qz/qw 是否全部有效 (latte 用法)
 */
export function robotPoseHtmlIsRenderable(input) {
    if (typeof input === 'string') {
        return input.indexOf('pose-card__body') !== -1 || input.indexOf('pose-card__body--ivg-text') !== -1;
    }
    if (input && typeof input === 'object' && typeof input.x === 'number') {
        return isFinite(input.x) && isFinite(input.y) && isFinite(input.z) &&
               isFinite(input.qx) && isFinite(input.qy) && isFinite(input.qz) && isFinite(input.qw);
    }
    return false;
}

/**
 * 格式化末端位姿为 HTML
 * - vision_grasp 用法: 传入原始 RobotStatus ROS 消息 (含 cartesian_position 等)
 * - latte 用法: 传入扁平 pose 对象 {x, y, z, qx, qy, qz, qw}
 */
export function formatRobotPoseHtml(input) {
    // 扁平 pose 对象 (latte 用法)
    if (input && typeof input === 'object' && typeof input.x === 'number' &&
        typeof input.qx === 'number') {
        const pose = {
            position: { x: input.x, y: input.y, z: input.z },
            orientation: { x: input.qx, y: input.qy, z: input.qz, w: input.qw }
        };
        return formatPoseBlockHtml(pose, null);
    }

    // 原始 ROS 消息 (vision_grasp 用法)
    const msg = normalizeRobotStatusJson(input);
    if (!msg || typeof msg !== 'object') return formatPoseBlockHtml(null, null);

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
    const hasPose = hasPos || hasOri;
    if (!hasPose) {
        if (msg.ivg_display != null && String(msg.ivg_display).trim()) {
            return `<div class="pose-card__body pose-card__body--ivg-text"><pre class="pose-card__ivg-pre">${_escapeHtml(String(msg.ivg_display))}</pre></div>`;
        }
        return '<div class="pose-card__empty">等待末端位姿...</div>';
    }
    const rpySrc = msg.cartesian_rpy || {};
    const rr = Number(rpySrc.x);
    const rp = Number(rpySrc.y);
    const ry = Number(rpySrc.z);
    const radToDeg = rad => rad * (180 / Math.PI);
    const rpyDeg =
        [rr, rp, ry].every(v => isFinite(v))
            ? { roll: radToDeg(rr), pitch: radToDeg(rp), yaw: radToDeg(ry) }
            : poseOriToRpyDeg(pose);
    return formatPoseBlockHtml(pose, rpyDeg);
}

/** 格式化 AI 大模型最终抓取位姿 (PoseArray 首元素) 喵~ */
export function formatFinalGraspPoseHtml(msg) {
    if (!msg || !Array.isArray(msg.poses) || msg.poses.length === 0) {
        return formatPoseBlockHtml(null, null);
    }
    const pose = msg.poses[0];
    return formatPoseBlockHtml(pose, null);
}
