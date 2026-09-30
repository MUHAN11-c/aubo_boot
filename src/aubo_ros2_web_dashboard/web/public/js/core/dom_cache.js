// dom_cache.js — lazy DOM element cache, no null entries
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
const IVGDomCache = {
	createDomCache
};
globalThis.IVGDomCache = IVGDomCache;
export { createDomCache, IVGDomCache };
