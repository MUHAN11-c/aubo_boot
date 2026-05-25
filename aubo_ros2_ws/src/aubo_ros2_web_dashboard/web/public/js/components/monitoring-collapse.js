// monitoring-collapse.js — 监控区折叠/展开共享组件
// 供 vision_grasp_panel.js / latte/main.js 共用，消除 ~80 行重复代码喵~
//
// 用法:
//   import { createMonitoringCollapse } from '../components/monitoring-collapse.js';
//   const mc = createMonitoringCollapse({ getById: $, jointChart, ... });
//   mc.bindEvents();
//   mc.scheduleSyncMinHeight();

const DEFAULT_STORAGE_KEY = 'ivg_vision_monitoring_collapsed';
const DEFAULT_SECTION_ID = 'layout-monitoring-section';
const DEFAULT_BUNDLE_ID = 'layout-monitoring-bundle';
const DEFAULT_TOGGLE_ID = 'btn-monitoring-toggle';

export function createMonitoringCollapse(opts) {
    const options = opts || {};
    const $ = options.getById || (id => document.getElementById(id));
    const jointChart = options.jointChart || null;  // 可选: 有图表时折叠展开触发 resize
    const storageKey = options.storageKey || DEFAULT_STORAGE_KEY;
    const sectionId = options.sectionId || DEFAULT_SECTION_ID;
    const bundleId = options.bundleId || DEFAULT_BUNDLE_ID;
    const toggleBtnId = options.toggleBtnId || DEFAULT_TOGGLE_ID;
    const floorPx = options.floorPx || 200;  // 位姿列最小高度

    let _minHeightRaf = 0;
    let _resizeObserver = null;

    function sectionEl() { return $(sectionId); }
    function bundleEl() { return $(bundleId); }
    function poseCol() {
        const b = bundleEl();
        return b && b.querySelector('.layout-monitoring-pose-col');
    }

    function scheduleSyncMinHeight() {
        if (_minHeightRaf) return;
        _minHeightRaf = requestAnimationFrame(() => {
            _minHeightRaf = 0;
            syncMinHeight();
        });
    }

    function syncMinHeight() {
        const section = sectionEl();
        const bundle = bundleEl();
        const col = poseCol();
        if (!bundle || !col) return;
        if (!section || section.classList.contains('is-monitoring-collapsed')) {
            bundle.style.removeProperty('--ivg-monitoring-bundle-min-px');
            return;
        }
        const intrinsic = Math.ceil(col.scrollHeight);
        bundle.style.setProperty('--ivg-monitoring-bundle-min-px', `${Math.max(floorPx, intrinsic)}px`);
    }

    function applyCollapsed(collapsed) {
        const section = sectionEl();
        const btn = $(toggleBtnId);
        if (!section || !btn) return;
        section.classList.toggle('is-monitoring-collapsed', collapsed);
        btn.setAttribute('aria-expanded', collapsed ? 'false' : 'true');
        const hint = btn.querySelector('.layout-monitoring-toggle__hint');
        if (hint) hint.textContent = collapsed ? '展开' : '收起';
        try {
            localStorage.setItem(storageKey, collapsed ? '1' : '0');
        } catch (_) { /* ignore quota / private mode */ }
        if (!collapsed) {
            requestAnimationFrame(() => {
                if (jointChart && typeof jointChart.observeResize === 'function') {
                    jointChart.observeResize();
                }
                window.dispatchEvent(new Event('resize'));
                scheduleSyncMinHeight();
                requestAnimationFrame(() => scheduleSyncMinHeight());
            });
        } else {
            const b = bundleEl();
            if (b) b.style.removeProperty('--ivg-monitoring-bundle-min-px');
        }
    }

    function bindEvents() {
        const btn = $(toggleBtnId);
        if (!btn) return;

        let initialCollapsed = false;
        try {
            initialCollapsed = localStorage.getItem(storageKey) === '1';
        } catch (_) { /* use expanded */ }
        applyCollapsed(initialCollapsed);

        btn.addEventListener('click', () => {
            const section = sectionEl();
            applyCollapsed(!(section && section.classList.contains('is-monitoring-collapsed')));
        });

        // ResizeObserver on pose column
        const col = poseCol();
        if (col && typeof ResizeObserver !== 'undefined') {
            _resizeObserver = new ResizeObserver(() => scheduleSyncMinHeight());
            _resizeObserver.observe(col);
        }
        window.addEventListener('resize', scheduleSyncMinHeight);
        if (document.fonts && typeof document.fonts.ready !== 'undefined' && document.fonts.ready.then) {
            document.fonts.ready.then(() => scheduleSyncMinHeight()).catch(() => {});
        }
    }

    /** 销毁 (页面卸载时调用) 喵~ */
    function destroy() {
        if (_resizeObserver) { _resizeObserver.disconnect(); _resizeObserver = null; }
        if (_minHeightRaf) { cancelAnimationFrame(_minHeightRaf); _minHeightRaf = 0; }
    }

    return {
        applyCollapsed,
        bindEvents,
        scheduleSyncMinHeight,
        syncMinHeight,
        destroy,
    };
}
