/**
 * 全站导航：为 .ivg-global-nav 内相对路径 .html 链接附加当前页 search（继承 ?rosbridge_port= 等）；
 * 根据 data-ivg-page 标记当前页链接 aria-current / .is-active（index / vision / latte）。
 * 全屏：仅非电脑端（无 pointer: fine 的触控环境）；电脑鼠标环境不显示全屏、不自动全屏。
 * 平板横屏可尝试自动全屏；Safari 无手势时首次触摸/点击再请求。主屏幕 Web App 不重复请求。
 */
(() => {
	const q = typeof window.location.search === 'string' ? window.location.search : '';

	const FS_TITLE_ENTER = '全屏显示页面（iPad：若无效请用 Safari 添加到主屏幕）';
	const FS_TITLE_EXIT = '退出全屏';

	function getFullscreenElement() {
		return document.fullscreenElement || document.webkitFullscreenElement || null;
	}

	async function enterFullscreen() {
		const el = document.documentElement;
		if (el.requestFullscreen) {
			await el.requestFullscreen();
			return;
		}
		if (el.webkitRequestFullscreen) {
			el.webkitRequestFullscreen();
			return;
		}
		throw new Error('Fullscreen API 不可用');
	}

	async function exitFullscreen() {
		if (document.exitFullscreen) {
			await document.exitFullscreen();
			return;
		}
		if (document.webkitExitFullscreen) {
			document.webkitExitFullscreen();
			return;
		}
	}

	function syncFullscreenButton(btn) {
		if (!btn) return;
		const on = !!getFullscreenElement();
		btn.classList.toggle('is-fullscreen', on);
		btn.setAttribute('aria-pressed', on ? 'true' : 'false');
		btn.setAttribute('aria-label', on ? '退出全屏' : '进入全屏');
		btn.textContent = on ? '退出全屏' : '全屏';
		btn.title = on ? FS_TITLE_EXIT : FS_TITLE_ENTER;
	}

	function shouldEnableFullscreenControl() {
		try {
			/* 存在精细指针（典型为鼠标）即视为电脑端，不提供全屏 */
			if (window.matchMedia('(pointer: fine)').matches) return false;
			return window.matchMedia('(hover: none) and (pointer: coarse)').matches;
		} catch (e) {
			return false;
		}
	}

	/** 仅 iPad 10 类横屏：与布局媒体一致，竖屏手机不自动全屏 */
	function shouldAutoEnterFullscreenTablet() {
		if (!shouldEnableFullscreenControl()) return false;
		try {
			return (
				window.matchMedia('(orientation: landscape)').matches &&
				window.matchMedia('(min-width: 1024px)').matches &&
				window.matchMedia('(max-width: 1366px)').matches
			);
		} catch (e) {
			return false;
		}
	}

	function armFirstInteractionFullscreen(btn) {
		const opts = { capture: true, passive: true };
		function cleanup() {
			document.removeEventListener('touchstart', onInteract, opts);
			document.removeEventListener('click', onInteract, opts);
		}
		async function onInteract() {
			cleanup();
			if (getFullscreenElement()) {
				if (btn) syncFullscreenButton(btn);
				return;
			}
			try {
				await enterFullscreen();
			} catch (e) {
				console.warn('[ivg_site_nav] 首次手势全屏:', e);
			}
			if (btn) syncFullscreenButton(btn);
		}
		document.addEventListener('touchstart', onInteract, opts);
		document.addEventListener('click', onInteract, opts);
	}

	function initTabletAutoFullscreen(root) {
		if (!shouldAutoEnterFullscreenTablet()) return;

		try {
			if (typeof navigator !== 'undefined' && navigator.standalone === true) return;
		} catch (e) {
			/* ignore */
		}

		const btn = (root || document).getElementById('ivg-nav-fullscreen-btn');

		(async () => {
			try {
				await enterFullscreen();
			} catch (e) {
				/* Safari 常因缺少用户手势拒绝，改由首次触摸/点击触发 */
			}
			if (btn) syncFullscreenButton(btn);
			if (!getFullscreenElement()) {
				armFirstInteractionFullscreen(btn);
			}
		})();
	}

	function initFullscreenButton(root) {
		const btn = (root || document).getElementById('ivg-nav-fullscreen-btn');
		if (!btn) return;
		if (!shouldEnableFullscreenControl()) return;

		const sync = () => syncFullscreenButton(btn);

		btn.addEventListener('click', () => {
			(async () => {
				try {
					if (getFullscreenElement()) {
						await exitFullscreen();
					} else {
						await enterFullscreen();
					}
				} catch (e) {
					console.warn('[ivg_site_nav] 全屏:', e);
				}
				sync();
			})();
		});

		document.addEventListener('fullscreenchange', sync);
		document.addEventListener('webkitfullscreenchange', sync);
		sync();
	}

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
		initFullscreenButton(root);
		initTabletAutoFullscreen(root);
	}

	if (document.readyState === 'loading') {
		document.addEventListener('DOMContentLoaded', () => { init(); });
	} else {
		init();
	}
})();
