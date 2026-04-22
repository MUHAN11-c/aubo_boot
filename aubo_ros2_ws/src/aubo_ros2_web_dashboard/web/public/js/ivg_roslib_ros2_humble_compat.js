/**
 * ROS 2 Humble + rosbridge：subscribe 校验要求 throttle_rate、queue_length、queue_size 为整数。
 * ros3djs 等旧代码会显式传 null，roslib@2 会原样序列化，导致「Invalid value: None」。
 * 在加载 ros2d/ros3d 之前对 ROSLIB.Topic 做薄包装（行为对齐官方 Topic 默认值）。
 */
(function () {
	var R = typeof globalThis !== 'undefined' && globalThis.ROSLIB;
	if (!R || typeof R.Topic !== 'function') return;
	var Orig = R.Topic;
	function nonNegInt(v, dflt) {
		var n = typeof v === 'number' ? v : parseInt(String(v), 10);
		if (!Number.isFinite(n) || n < 0) return dflt;
		return n;
	}
	function normOpts(opts) {
		var o = Object.assign({}, opts);
		if (o.throttle_rate == null || o.throttle_rate === '') {
			o.throttle_rate = 0;
		} else {
			o.throttle_rate = nonNegInt(o.throttle_rate, 0);
		}
		if (o.queue_length == null || o.queue_length === '') {
			o.queue_length = 0;
		} else {
			o.queue_length = nonNegInt(o.queue_length, 0);
		}
		if (o.queue_size == null || o.queue_size === '') {
			o.queue_size = 100;
		} else {
			o.queue_size = nonNegInt(o.queue_size, 100);
		}
		return o;
	}
	function TopicHumbleCompat(opts) {
		return new Orig(normOpts(opts));
	}
	TopicHumbleCompat.prototype = Orig.prototype;
	if (typeof Object.setPrototypeOf === 'function') {
		Object.setPrototypeOf(TopicHumbleCompat, Orig);
	}
	R.Topic = TopicHumbleCompat;
})();

/**
 * ros3d.min.js 打包时对 ROSLIB 做 ``i(ROSLIB)`` 快照（见 bundle 末尾 ``})({}, ROSLIB)``）；
 * 若加载 ros3d **当时** ``ROSLIB.URDF_MESH`` 等不存在，闭包内 ``r.URDF_MESH`` 恒为 ``undefined``，
 * ``geometry.type === r.URDF_MESH`` 永不成立，机械臂 mesh 分支不会执行。
 * 必须在 **ros3d.min.js 之前** 写入这些枚举（仅缺省时设置，不覆盖已有值）。
 */
(function () {
	var R = typeof globalThis !== 'undefined' && globalThis.ROSLIB;
	if (!R) return;
	if (typeof R.URDF_MESH === 'number') return;
	R.URDF_SPHERE = 0;
	R.URDF_BOX = 1;
	R.URDF_CYLINDER = 2;
	R.URDF_MESH = 3;
})();
