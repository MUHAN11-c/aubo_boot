// tf-math.js — 四元数/旋转矩阵/变换运算（纯数学，零外部依赖）
// 从 view3d/tf_clients.js 提取，供 vision-grasp / latte / monitor 各页共用喵~
//
// 四元数约定: Hamilton (xyzw), 与 tf2::Quaternion::setRPY() 一致
// 参考: Vue 3 版 lib/tf_math.ts (功能对齐)

// ── 基本类型 ──────────────────────────────────────────────────────────────────

export function normalizeFrameId(frame) {
    return String(frame || '').trim().replace(/^\/+/, '');
}

export function ivgIdentityTransform() {
    return {
        translation: { x: 0, y: 0, z: 0 },
        rotation: { x: 0, y: 0, z: 0, w: 1 },
    };
}

export function ivgCloneTransform(tf) {
    const src = tf || {};
    const tr = src.translation || {};
    const rot = src.rotation || {};
    return {
        translation: {
            x: Number(tr.x) || 0,
            y: Number(tr.y) || 0,
            z: Number(tr.z) || 0,
        },
        rotation: {
            x: Number(rot.x) || 0,
            y: Number(rot.y) || 0,
            z: Number(rot.z) || 0,
            w: Number(rot.w) || 1,
        },
    };
}

// ── 四元数运算 ────────────────────────────────────────────────────────────────

export function ivgQuatNormalize(q) {
    const x = Number(q && q.x) || 0;
    const y = Number(q && q.y) || 0;
    const z = Number(q && q.z) || 0;
    const w = Number(q && q.w) || 1;
    const n = Math.hypot(x, y, z, w) || 1;
    return { x: x / n, y: y / n, z: z / n, w: w / n };
}

export function ivgQuatMultiply(a, b) {
    return {
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

export function ivgQuatConjugate(q) {
    return { x: -q.x, y: -q.y, z: -q.z, w: q.w };
}

export function ivgRotateVectorByQuat(v, q) {
    const p = { x: v.x, y: v.y, z: v.z, w: 0 };
    const qp = ivgQuatMultiply(q, p);
    const out = ivgQuatMultiply(qp, ivgQuatConjugate(q));
    return { x: out.x, y: out.y, z: out.z };
}

// ── 变换运算 ──────────────────────────────────────────────────────────────────

export function ivgComposeTransforms(a, b) {
    const ta = ivgCloneTransform(a);
    const tb = ivgCloneTransform(b);
    const qa = ivgQuatNormalize(ta.rotation);
    const qb = ivgQuatNormalize(tb.rotation);
    const vb = ivgRotateVectorByQuat(tb.translation, qa);
    const q = ivgQuatNormalize(ivgQuatMultiply(qa, qb));
    return {
        translation: {
            x: ta.translation.x + vb.x,
            y: ta.translation.y + vb.y,
            z: ta.translation.z + vb.z,
        },
        rotation: { x: q.x, y: q.y, z: q.z, w: q.w },
    };
}

export function ivgInvertTransform(tf) {
    const src = ivgCloneTransform(tf);
    const q = ivgQuatNormalize(src.rotation);
    const qi = ivgQuatConjugate(q);
    const t = ivgRotateVectorByQuat(
        { x: -src.translation.x, y: -src.translation.y, z: -src.translation.z },
        qi,
    );
    return {
        translation: { x: t.x, y: t.y, z: t.z },
        rotation: { x: qi.x, y: qi.y, z: qi.z, w: qi.w },
    };
}

// ── TF 树路径解析 ─────────────────────────────────────────────────────────────

export function ivgBuildTfPath(frame, edges) {
    const out = [{ frame, transform: ivgIdentityTransform() }];
    const seen = new Set([frame]);
    let cur = frame;
    let acc = ivgIdentityTransform();
    for (let i = 0; i < 256; i++) {
        const edge = edges[cur];
        if (!edge || !edge.parent) break;
        acc = ivgComposeTransforms(edge.transform, acc);
        cur = edge.parent;
        if (seen.has(cur)) break;
        seen.add(cur);
        out.push({ frame: cur, transform: acc });
    }
    return out;
}

export function ivgFindRelativeTransform(sourceFrame, targetFrame, edges) {
    const src = normalizeFrameId(sourceFrame);
    const dst = normalizeFrameId(targetFrame);
    if (!src || !dst) return null;
    if (src === dst) return ivgIdentityTransform();
    const srcPath = ivgBuildTfPath(src, edges);
    const dstPath = ivgBuildTfPath(dst, edges);
    const srcMap = Object.create(null);
    srcPath.forEach(entry => { srcMap[entry.frame] = entry.transform; });
    for (let i = 0; i < dstPath.length; i++) {
        const entry = dstPath[i];
        if (!Object.prototype.hasOwnProperty.call(srcMap, entry.frame)) continue;
        return ivgComposeTransforms(ivgInvertTransform(srcMap[entry.frame]), entry.transform);
    }
    return null;
}

// ── 四元数 → RPY (弧度) ─────────────────────────────────────────────────────

export function quatToRpy(q) {
    const nq = ivgQuatNormalize(q);
    const { x, y, z, w } = nq;
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);
    const sinp = 2 * (w * y - z * x);
    const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);
    return { roll, pitch, yaw };
}

export function quatToRpyDeg(q) {
    const { roll, pitch, yaw } = quatToRpy(q);
    const rad2deg = 180 / Math.PI;
    return {
        roll: roll * rad2deg,
        pitch: pitch * rad2deg,
        yaw: yaw * rad2deg,
    };
}
