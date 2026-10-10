// lifecycle.js — 页面生命周期管理器
// 统一处理 visibilitychange / pagehide / online 事件喵~
//
// 用法:
//   import { createPageLifecycle } from '../core/lifecycle.js';
//   createPageLifecycle({
//     onInit: async () => { ... },
//     onCleanup: () => { ... },
//     onPause: () => { ... },
//     onResume: () => { ... },
//   });

export function createPageLifecycle(opts = {}) {
    const { onInit, onCleanup, onPause, onResume } = opts;

    let _initialized = false;

    async function _init() {
        if (_initialized) return;
        _initialized = true;
        if (onInit) await onInit();
    }

    function _cleanup() {
        if (onCleanup) onCleanup();
        if (onPause) onPause(); // pagehide 时也 pause
    }

    function _onVisibility() {
        if (!_initialized) return;
        if (document.visibilityState === 'hidden') {
            if (onPause) onPause();
        } else {
            if (onResume) onResume();
        }
    }

    if (typeof document !== 'undefined') {
        document.addEventListener('DOMContentLoaded', _init);
        document.addEventListener('visibilitychange', _onVisibility);
        window.addEventListener('pagehide', _cleanup);
    }

    return {
        init: _init,
        cleanup: _cleanup,
    };
}
