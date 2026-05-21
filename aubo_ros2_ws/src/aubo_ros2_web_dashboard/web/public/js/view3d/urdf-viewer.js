// urdf-viewer.js — ros3djs URDF 3D 查看器生命周期封装
// 从 vision_grasp/urdf_panel.js 提取，供 vision-grasp / latte 共用喵~
//
// 用法:
//   import { createUrdfViewer } from '../view3d/urdf-viewer.js';
//   const viewer = createUrdfViewer({ ros, getById: $, hostId: 'vision-urdf-host' });
//   viewer.start();
//   viewer.stop();  // 页面卸载时清理

import * as ROSLIB from 'roslib';
import { IvgRos3dView3dSession } from './session.js';

export function createUrdfViewer(opts = {}) {
    const ros = opts.ros;
    const $ = opts.getById || (id => document.getElementById(id));
    const hostId = opts.hostId || 'vision-urdf-host';
    const innerId = opts.innerId || (hostId + '-inner');
    const forceUrdfOnly = opts.forceUrdfOnly !== false;

    let _session = null;
    let _resizeObs = null;
    let _toolStatusSub = null;

    function _layout() {
        if (!_session || !_session.viewer3d) return;
        const host = $(hostId);
        if (!host || typeof _session.viewer3d.resize !== 'function') return;
        const w = Math.max(120, host.clientWidth || 400);
        const h = Math.max(240, host.clientHeight || Math.round(w * 0.72));
        try { _session.viewer3d.resize(w, h); } catch (e) { /* */ }
    }

    function start() {
        const host = $(hostId);
        if (!host) return;
        if (!ros || !ros.isConnected) {
            console.warn('[urdf-viewer] ros 未连接，跳过 3D 启动喵~');
            return;
        }
        stop();

        try {
            _session = new IvgRos3dView3dSession(ros, $, {
                view3dHostId: hostId,
                viewerInnerId: innerId,
                forceUrdfOnly,
            });
            _session.start();

            requestAnimationFrame(() => _layout());

            if (typeof ResizeObserver !== 'undefined') {
                _resizeObs = new ResizeObserver(() => _layout());
                _resizeObs.observe(host);
            }

            // 工具切换时自动重载 URDF
            let currentToolId = null;
            _toolStatusSub = new ROSLIB.Topic({
                ros,
                name: '/tool_changer_status',
                messageType: 'tool_changer_interface/msg/ToolChangerStatus',
                throttle_rate: 500,
            });
            _toolStatusSub.subscribe(msg => {
                const newId = msg && msg.tool_id;
                if (!newId) return;
                if (currentToolId === null) {
                    currentToolId = newId;
                } else if (newId !== currentToolId) {
                    currentToolId = newId;
                    setTimeout(() => {
                        if (_session && typeof _session.reloadUrdf === 'function') {
                            _session.reloadUrdf();
                        }
                    }, 200);
                }
            });
        } catch (e) {
            console.warn('[urdf-viewer] 3D 查看器启动失败:', e);
        }
    }

    function stop() {
        if (_toolStatusSub) {
            try { _toolStatusSub.unsubscribe(); } catch (e) { /* */ }
            _toolStatusSub = null;
        }
        if (_resizeObs) {
            try { _resizeObs.disconnect(); } catch (e) { /* */ }
            _resizeObs = null;
        }
        if (_session) {
            try { _session.stop(); } catch (e) { /* */ }
            _session = null;
        }
    }

    function getSession() {
        return _session;
    }

    return { start, stop, layout: _layout, getSession };
}
