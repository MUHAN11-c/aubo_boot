// ivg-site-nav Web Component — 全站统一导航栏
// 用法: <ivg-site-nav page="monitor"></ivg-site-nav>

const NAV_LINKS = [
  { href: 'index.html',               page: 'index',    label: '门户' },
  { href: 'vision_grasp_panel.html',  page: 'vision',   label: '视觉抓取' },
  { href: 'coffee_latte_panel.html',  page: 'latte',    label: '咖啡拉花' },
  { href: 'tf_monitor_panel.html',    page: 'monitor',  label: '监控面板' },
  { href: 'log_panel.html',           page: 'log',      label: '日志' },
  { href: 'settings_panel.html',      page: 'settings', label: '设置' },
];

const FS_TITLE_ENTER = '全屏显示页面（iPad：若无效请用 Safari 添加到主屏幕）';
const FS_TITLE_EXIT  = '退出全屏';
const ID_BTN = 'ivg-nav-fullscreen-btn';

function hasTouch() {
  try { return window.matchMedia('(hover: none) and (pointer: coarse)').matches; }
  catch (_) { return false; }
}

function isTabletLandscape() {
  try {
    return hasTouch() && window.matchMedia('(orientation: landscape)').matches
      && window.matchMedia('(min-width: 1024px)').matches
      && window.matchMedia('(max-width: 1366px)').matches;
  } catch (_) { return false; }
}

class IvgSiteNav extends HTMLElement {
  connectedCallback() {
    const page = this.getAttribute('page') || '';
    const q = window.location.search || '';

    const linksHtml = NAV_LINKS.map(function (l) {
      var act = l.page === page;
      return '<li><a class="ivg-global-nav__link' + (act ? ' is-active' : '') + '"'
        + ' href="' + l.href + q + '" data-ivg-nav="' + l.page + '"'
        + (act ? ' aria-current="page"' : '') + '>' + l.label + '</a></li>';
    }).join('');

    this.className = 'ivg-global-nav';
    this.setAttribute('aria-label', '灵视IVG 站内导航');
    this.innerHTML =
      '<div class="ivg-global-nav__inner">'
      + '<div class="ivg-global-nav__start">'
      + '<a class="ivg-global-nav__brand" href="index.html' + q + '">灵视 <span>IVG</span></a>'
      + '<ul class="ivg-global-nav__list" role="list">' + linksHtml + '</ul>'
      + '</div>'
      + '<div class="ivg-global-nav__actions">'
      + '<button type="button" class="ivg-global-nav__fs-btn" id="' + ID_BTN + '"'
      + ' aria-pressed="false" aria-label="进入全屏" title="' + FS_TITLE_ENTER + '">全屏</button>'
      + '</div></div>';

    this._initFS();
    if (isTabletLandscape()) this._autoFS();
  }

  _fsEl() { return document.fullscreenElement || document.webkitFullscreenElement || null; }

  async _enter() {
    var el = document.documentElement;
    if (el.requestFullscreen) { await el.requestFullscreen(); return; }
    if (el.webkitRequestFullscreen) el.webkitRequestFullscreen();
  }

  async _exit() {
    if (document.exitFullscreen) await document.exitFullscreen();
    else if (document.webkitExitFullscreen) document.webkitExitFullscreen();
  }

  _syncBtn() {
    var btn = this.querySelector('#' + ID_BTN);
    if (!btn) return;
    var on = !!this._fsEl();
    btn.classList.toggle('is-fullscreen', on);
    btn.setAttribute('aria-pressed', on ? 'true' : 'false');
    btn.setAttribute('aria-label', on ? '退出全屏' : '进入全屏');
    btn.textContent = on ? '退出全屏' : '全屏';
    btn.title = on ? FS_TITLE_EXIT : FS_TITLE_ENTER;
  }

  _initFS() {
    if (!hasTouch()) {
      var act = this.querySelector('.ivg-global-nav__actions');
      if (act) act.style.display = 'none';
      return;
    }
    var self = this;
    var btn = this.querySelector('#' + ID_BTN);
    if (!btn) return;
    function sync() { self._syncBtn(); }
    btn.addEventListener('click', function () {
      (async function () {
        try { self._fsEl() ? await self._exit() : await self._enter(); }
        catch (e) { console.warn('[ivg-nav] 全屏:', e); }
        sync();
      })();
    });
    document.addEventListener('fullscreenchange', sync);
    document.addEventListener('webkitfullscreenchange', sync);
    sync();
  }

  _autoFS() {
    try { if (navigator.standalone === true) return; } catch (_) {}
    var self = this;
    function go() {
      (async function () {
        try { await self._enter(); } catch (_) {}
        self._syncBtn();
        if (!self._fsEl()) {
          document.addEventListener('touchstart', go, { capture: true, passive: true, once: true });
          document.addEventListener('click', go, { capture: true, passive: true, once: true });
        }
      })();
    }
    go();
  }
}

customElements.define('ivg-site-nav', IvgSiteNav);
