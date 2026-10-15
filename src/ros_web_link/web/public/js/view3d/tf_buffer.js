// tf_buffer.js — L4 TF 缓冲（foxglove 数据源 + Lichtblick TransformTree）
//
// 职责：
//   - foxgloveTfTree() 单例：订阅 /tf + /tf_static（经自研 ros_client，CDR 解码），
//     喂 vendor transforms（Lichtblick TransformTree，见 vendor/transforms/README.md）
//   - IvgFoxgloveTfClient：IvgRos3dTfClient 的鸭子类型替换（接口逐一对齐）——
//     旧岛（ros3d r89 的 SceneNode/UrdfClient/LaserScan）无改动换源，
//     彻底去掉 rosbridge action + tf2_web_republisher 依赖。
//
// 时间语义：latest 查询（tf2 TimePointZero 等价）用远未来时间 + 大 maxDelta
// 夹取到最新条目（findClosestTransforms 的 clamp 分支）；静态变换天然由单条目表达。
// 订阅回调节流：transThres/angularThres 变化阈值 + 10Hz 上限（对齐 ROS2TFClient 语义）。

import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';
import { normalizeFrameId } from '../core/tf-math.js';
import { TransformTree, Transform, ObjectPool } from '../vendor/transforms/tf_tree_bundle.mjs';

// bigint 时间（ns）。
// “latest 查询”语义（tf2 TimePointZero 等价）：查询时间取远未来 + 容差取超大值
// → 每条边（含 t=0 的静态单条目边）各自 clamp 到**最新**条目。
// 注意容差必须 ≥ 当前纪元时间（~1.8e18 ns），2^60（≈1.15e18）不够，取 2^62。
const FAR_FUTURE = 2n ** 62n;
const LATEST_MAX_DELTA = 2n ** 62n;

let _tree = null;
let _pool = null;
let _wired = false;

/** 共享 TF 树单例（页面级；连接由 ros_client 单例管理） */
function foxgloveTfTree() {
	if (!_tree) {
		// 构造姿势对齐上游 Renderer.ts：ObjectPool(Transform.Empty) + TransformTree(pool)
		_pool = new ObjectPool(Transform.Empty, { maxCapacity: 50 * 5000 });
		_tree = new TransformTree(_pool);
	}
	if (!_wired) {
		_wired = true;
		const client = foxgloveRosClient();
		client.subscribe('/tf', 'tf2_msgs/msg/TFMessage', (m) => {
			_ingest(m.data, false);
		}, { dispatchMode: 'latest' });
		client.subscribe('/tf_static', 'tf2_msgs/msg/TFMessage', (m) => {
			_ingest(m.data, true);
		}, { dispatchMode: 'immediate' });
	}
	return _tree;
}

function _ingest(tfMessage, isStatic) {
	if (!tfMessage || !Array.isArray(tfMessage.transforms)) return;
	const tree = _tree;
	for (const t of tfMessage.transforms) {
		if (!t || !t.header || !t.child_frame_id) continue;
		const stamp = t.header.stamp || {};
		// 静态变换无历史语义：时间归 0（单条目，任意时刻为真）
		const sec = isStatic ? 0n : BigInt(stamp.sec ?? 0);
		const nsec = isStatic ? 0n : BigInt(stamp.nanosec ?? 0);
		const time = sec * 1_000_000_000n + nsec;
		const tr = t.transform || {};
		const q = tr.rotation || {};
		const p = tr.translation || {};
		// Transform 构造姿势对齐上游 Renderer.addTransform：pool.acquire + setPositionRotation
		const transform = _pool.acquire();
		transform.setPositionRotation(
			[Number(p.x) || 0, Number(p.y) || 0, Number(p.z) || 0],
			[Number(q.x) || 0, Number(q.y) || 0, Number(q.z) || 0, Number(q.w) || 1],
		);
		tree.addTransform(t.child_frame_id, t.header.frame_id, time, transform);
	}
}

/** frame → fixedFrame 的最新相对变换（tf2 lookupTransform(fixed, frame, 0) 语义） */
function lookupLatest(tree, frame, fixedFrame) {
	if (frame === fixedFrame) {
		return {
			translation: { x: 0, y: 0, z: 0 },
			rotation: { x: 0, y: 0, z: 0, w: 1 },
		};
	}
	const f = tree.frame(frame);
	if (!f) return null;
	// 远未来查询：所有边 clamp 到各自最新条目（latest 语义）
	const t = FAR_FUTURE;
	const input = {
		position: { x: 0, y: 0, z: 0 },
		orientation: { x: 0, y: 0, z: 0, w: 1 },
	};
	const output = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
	const pose = tree.apply(output, input, fixedFrame, undefined, frame, t, t, LATEST_MAX_DELTA);
	if (!pose) return null;
	return {
		translation: { x: pose.position.x, y: pose.position.y, z: pose.position.z },
		rotation: pose.orientation,
	};
}

function IvgFoxgloveTfClient(fixedFrame, opts) {
	const o = opts || {};
	this.fixedFrame = normalizeFrameId(fixedFrame) || 'base_link';
	this._tree = foxgloveTfTree();
	this._client = foxgloveRosClient();
	this._subs = new Map(); // frame → Set<cb>
	this._lastSent = new Map(); // frame → {t, q} 上次下发（阈值节流）
	this._transThres = typeof o.transThres === 'number' ? o.transThres : 0.01;
	this._angularThres = typeof o.angularThres === 'number' ? o.angularThres : 0.05;
	this._rateHz = typeof o.rate === 'number' ? o.rate : 10;
	this._timer = setInterval(() => this._pump(), Math.max(30, Math.floor(1000 / this._rateHz)));
	this._tfHook = null;
	// /tf 到达即尝试一次快速 pump（降低显示延迟），节流由 rate 兜底
	this._tfHook = (typed) => {
		if (typed && typed.topic === '/tf') this._pump();
	};
	void this._tfHook;
}

IvgFoxgloveTfClient.prototype._pump = function () {
	if (this._subs.size === 0) return;
	for (const [frame, cbs] of this._subs) {
		const tfm = lookupLatest(this._tree, frame, this.fixedFrame);
		if (!tfm) continue;
		const q = tfm.rotation;
		const last = this._lastSent.get(frame);
		if (last) {
			const dt = Math.hypot(tfm.translation.x - last.tx, tfm.translation.y - last.ty, tfm.translation.z - last.tz);
			// 粗略角差：四元数点积（1 - dot ≈ 半角平方量级）
			const dot = Math.abs(q.x * last.qx + q.y * last.qy + q.z * last.qz + q.w * last.qw);
			if (dt < this._transThres && dot > 1 - this._angularThres * this._angularThres * 0.5) continue;
		}
		this._lastSent.set(frame, { tx: tfm.translation.x, ty: tfm.translation.y, tz: tfm.translation.z, qx: q.x, qy: q.y, qz: q.z, qw: q.w });
		for (const cb of [...cbs]) {
			try { cb(tfm); } catch (e) { /* 消费方异常不阻断 */ }
		}
	}
};

IvgFoxgloveTfClient.prototype.getTransform = function (frame) {
	const f = normalizeFrameId(frame);
	if (!f) return null;
	return lookupLatest(this._tree, f, this.fixedFrame);
};

IvgFoxgloveTfClient.prototype.hasTransformFor = function (frame) {
	const f = normalizeFrameId(frame);
	if (!f) return false;
	return this.getTransform(f) !== null;
};

IvgFoxgloveTfClient.prototype.subscribe = function (frame, cb) {
	const f = normalizeFrameId(frame);
	if (!f || typeof cb !== 'function') return;
	if (!this._subs.has(f)) this._subs.set(f, new Set());
	this._subs.get(f).add(cb);
	return () => this.unsubscribe(f, cb);
};

IvgFoxgloveTfClient.prototype.unsubscribe = function (frame, cb) {
	const f = normalizeFrameId(frame);
	const set = this._subs.get(f);
	if (!set) return;
	set.delete(cb);
	if (set.size === 0) {
		this._subs.delete(f);
		this._lastSent.delete(f);
	}
};

IvgFoxgloveTfClient.prototype.setFixedFrame = function (frame) {
	const f = normalizeFrameId(frame);
	if (!f || f === this.fixedFrame) return false;
	this.fixedFrame = f;
	this._lastSent.clear();
	this._pump();
	return true;
};

IvgFoxgloveTfClient.prototype.dispose = function () {
	if (this._timer) {
		clearInterval(this._timer);
		this._timer = null;
	}
	this._subs.clear();
	this._lastSent.clear();
};

IvgFoxgloveTfClient.prototype.getFrameList = function () {
	try {
		return this._tree.frameList().map((e) => e.value);
	} catch (_) {
		return [];
	}
};

export {
	foxgloveTfTree,
	lookupLatest,
	IvgFoxgloveTfClient,
};
