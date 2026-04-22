/**
 * 共享 DOM 缓存工具：
 * - 面向仍使用静态 HTML + 经典脚本的页面。
 * - 统一 topics_lab / vision_grasp 的 getElementById 缓存方式，减少高频回调里的重复查询。
 * - 默认不缓存 null，避免脚本早于 DOM 就绪时把“未找到”永久记住。
 */
(function (global) {
	'use strict';

	function createDomCache(root) {
		const doc = root && typeof root.getElementById === 'function' ? root : document;
		const cache = Object.create(null);
		return function getById(id) {
			if (Object.prototype.hasOwnProperty.call(cache, id)) {
				const cached = cache[id];
				if (cached != null) return cached;
			}
			const node = doc.getElementById(id);
			if (node != null) cache[id] = node;
			return node;
		};
	}

	global.IVGDomCache = {
		createDomCache
	};
})(typeof window !== 'undefined' ? window : globalThis);
