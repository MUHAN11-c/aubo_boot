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

// localStorage 键
var LS_KEY = 'ivg_last_tool_id';

export function createUrdfViewer(opts = {}) {
    const ros = opts.ros;
    const $ = opts.getById || (id => document.getElementById(id));
    const hostId = opts.hostId || 'vision-urdf-host';
    const innerId = opts.innerId || (hostId + '-inner');
    const forceUrdfOnly = opts.forceUrdfOnly !== false;

    let _session = null;
    let _resizeObs = null;
    let _toolStatusSub = null;
    let _currentToolId = '';
    let _initialized = false;

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
            const viewerOpts = opts.viewerOpts || {};
            _session = new IvgRos3dView3dSession(ros, $, {
                view3dHostId: hostId,
                viewerInnerId: innerId,
                forceUrdfOnly,
                ...viewerOpts,
            });
            _session.start();

            requestAnimationFrame(() => _layout());

            if (typeof ResizeObserver !== 'undefined') {
                _resizeObs = new ResizeObserver(() => _layout());
                _resizeObs.observe(host);
            }

            // 初始状态：优先 localStorage（ROS 状态有最多 5s 延迟）喵~
            var stored = '';
            try { stored = localStorage.getItem(LS_KEY) || ''; } catch (e) {}
            if (stored) {
                _currentToolId = stored;
                _initialized = true;
            }

            // 订阅工具状态 → tool_id 变化 → reloadUrdf 显示新夹爪喵~
            _toolStatusSub = new ROSLIB.Topic({
                ros,
                name: '/tool_changer_status',
                messageType: 'ivg_interfaces/msg/ToolChangerStatus',
                throttle_rate: 500,
            });
            _toolStatusSub.subscribe(msg => {
                if (!msg) return;
                var newId = String(msg.tool_id || '');
                var connected = msg.is_connected !== false;

                if (!_initialized) {
                    // 首条消息：以 ROS 真实状态为准喵~
                    _initialized = true;
                    _currentToolId = newId;
                    try { localStorage.setItem(LS_KEY, newId); } catch (e) {}
                    return;
                }

                if (connected && newId !== _currentToolId) {
                    _currentToolId = newId;
                    try { localStorage.setItem(LS_KEY, newId); } catch (e) {}
                    // 延迟等后端 URDF 参数生效后重载喵~
                    setTimeout(() => {
                        if (_session && typeof _session.reloadUrdf === 'function') {
                            _session.reloadUrdf();
                        }
                    }, 500);
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
        _currentToolId = '';
        _initialized = false;
    }

    function getSession() {
        return _session;
    }

    function getCurrentToolId() {
        return _currentToolId;
    }

    function addTrajectoryLine(waypoints, color, linewidth) {
        if (_session && typeof _session.addTrajectoryLine === 'function') {
            _session.addTrajectoryLine(waypoints, color, linewidth);
        }
    }
    function clearTrajectoryLines() {
        if (_session && typeof _session.clearTrajectoryLines === 'function') {
            _session.clearTrajectoryLines();
        }
    }

    return { start, stop, layout: _layout, getSession, getCurrentToolId, addTrajectoryLine, clearTrajectoryLines };
}
