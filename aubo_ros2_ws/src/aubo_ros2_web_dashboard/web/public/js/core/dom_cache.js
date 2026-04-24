/**
 * 共享 DOM 缓存工具：
 * - 面向静态 HTML 页面与入口 ES module。
 * - 统一各页面的 getElementById 缓存方式，减少高频回调里的重复查询。
 * - 默认不缓存 null，避免脚本早于 DOM 就绪时把“未找到”永久记住。
 */

function createDomCache(root) {
	const doc = root && typeof root.getElementById === 'function' ? root : document;
	const cache = Object.create(null);
	/** 缓存成功的非 null 节点；null 不缓存，避免 DOM 未就绪时永久命中未创建元素。 */
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

const IVGDomCache = {
	createDomCache
};

globalThis.IVGDomCache = IVGDomCache;

export { createDomCache, IVGDomCache };
