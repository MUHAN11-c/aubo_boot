/**
 * 全站导航：为 .ivg-global-nav 内相对路径 .html 链接附加当前页 search（继承 ?ros_mode= 等）；
 * 根据 data-ivg-page 标记当前页链接 aria-current / .is-active（index / vision / latte / console）。
 */
(() => {
	const q = typeof window.location.search === 'string' ? window.location.search : '';

	function applyQueryToInternalNavLinks(root) {
		const base = root || document;
		base.querySelectorAll('.ivg-global-nav a[href]').forEach(a => {
			const h = a.getAttribute('href');
			if (!h || h.indexOf('#') === 0) return;
			if (/^https?:\/\//i.test(h)) return;
			if (h.indexOf('.html') === -1) return;
			const path = h.split('?')[0];
			a.setAttribute('href', path + q);
		});
	}

	function setActiveNav(root) {
		const nav = (root || document).querySelector('.ivg-global-nav[data-ivg-page]');
		if (!nav) return;
		const page = nav.getAttribute('data-ivg-page');
		if (!page) return;
		nav.querySelectorAll('.ivg-global-nav__link[data-ivg-nav]').forEach(a => {
			if (a.getAttribute('data-ivg-nav') === page) {
				a.setAttribute('aria-current', 'page');
				a.classList.add('is-active');
			} else {
				a.removeAttribute('aria-current');
				a.classList.remove('is-active');
			}
		});
	}

	function init(root) {
		applyQueryToInternalNavLinks(root);
		setActiveNav(root);
	}

	if (document.readyState === 'loading') {
		document.addEventListener('DOMContentLoaded', () => { init(); });
	} else {
		init();
	}
})();
