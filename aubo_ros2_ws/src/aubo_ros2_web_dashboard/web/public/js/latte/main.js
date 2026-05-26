// latte/main.js — 咖啡拉花面板独立入口
// 不复用 vision_grasp_panel.js！独立初始化 3D 查看器 + 关节图 + 连接管理喵~
//
// 架构:
//   3D 查看器 ← view3d/urdf-viewer.js (ros3djs)
//   关节图     ← components/joint-chart.js
//   位姿显示   ← components/pose-card.js
//   监控区折叠 ← components/monitoring-collapse.js (共享)
//   连接管理   ← core/ros.js
//   拉花控制   ← latte/latte_controls.js (图案选择/杯子参数/预览/执行)
//
// 不从 vision-grasp/ 导入任何模块！

import { ros } from '../core/ros.js';
import { loadIvgRuntime } from '../core/runtime_provider.js';
import { rosbridgeWebSocketUrlFromRuntime } from '../ivg_runtime.js';
import { createPageLifecycle } from '../core/lifecycle.js';
import { createUrdfViewer } from '../view3d/urdf-viewer.js';
import { createJointChartController } from '../components/joint-chart.js';
import { createMonitoringCollapse } from '../components/monitoring-collapse.js';
import { formatRobotPoseHtml, robotPoseHtmlIsRenderable } from '../components/pose-card.js';
import { createDomCache } from '../core/dom_cache.js';
import { canonicalRosTopic, rosMsgArrayField } from '../core/utils.js';
import { logBus } from '../core/log-bus.js';
import { initLatteControls } from '../latte/latte_controls.js';

const TAG = '[latte/main]';
const PAGE_STREAM_SUFFIX = `${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 8)}`;

const $ = typeof createDomCache === 'function'
    ? createDomCache(document)
    : (id) => document.getElementById(id);

// ── 组件实例 ────────────────────────────────────────────────────────────────
let urdfViewer = null;
let jointChart = null;
let monitoringCollapse = null;
let latteControls = null;
let _poseCache = { value: '' };

// ── 话题订阅 ────────────────────────────────────────────────────────────────

function setupSubscriptions() {
    if (!ros.isConnected) return;
    window.__ivgPoseReady = false;

    // 机械臂状态 → 位姿显示 + 状态栏
    ros.subscribe('/robot_status', 'ivg_interfaces/msg/RobotStatus', (msg) => {
        if (!msg || !msg.cartesian_position) return;
        const cp = msg.cartesian_position;
        // 缓存末端位姿供拉花预览使用
        window.__ivgLastEEPose = {
            x: cp.position?.x || 0, y: cp.position?.y || 0, z: cp.position?.z || 0,
            qx: cp.orientation?.x || 0, qy: cp.orientation?.y || 0,
            qz: cp.orientation?.z || 0, qw: cp.orientation?.w || 1,
        };
        const wasReady = window.__ivgPoseReady;
        window.__ivgPoseReady = true;
        if (!wasReady) {
            document.dispatchEvent(new CustomEvent('latte:pose-ready'));
        }
        const poseEl = $('pose-text');
        if (poseEl && robotPoseHtmlIsRenderable(window.__ivgLastEEPose)) {
            const html = formatRobotPoseHtml(window.__ivgLastEEPose);
            if (html !== _poseCache.value) {
                _poseCache.value = html;
                poseEl.innerHTML = html;
            }
        }
    });

    // 超时兜底: 5s 后仍未收到 robot_status → 允许预览 (后端会使用原点)
    setTimeout(() => {
        if (!window.__ivgPoseReady) {
            window.__ivgPoseReady = true;
            document.dispatchEvent(new CustomEvent('latte:pose-ready'));
            logBus.addLog('warn', 'ros_manager', 'robot_status 超时未到达(5s), 预览将使用原点位置', {}, 'latte');
        }
    }, 5000);

    // 关节角 → 图表
    ros.subscribe('/joint_states', 'sensor_msgs/msg/JointState', (msg) => {
        const names = rosMsgArrayField(msg, 'name');
        const pos = rosMsgArrayField(msg, 'position');
        if (jointChart) jointChart.pushSample(names, pos);
    });

    // 驱动模式
    ros.subscribe('/aubo/mode', 'std_msgs/msg/String', (msg) => {
        const modeStr = (msg?.data || '').toLowerCase();
        const badge = $('mode-badge');
        if (badge) {
            const isReal = modeStr.includes('real') || modeStr.includes('true');
            badge.className = 'mode-badge ' + (modeStr ? (isReal ? 'real' : 'sim') : 'unknown');
            badge.innerHTML = `<span class="mode-dot"></span>${isReal ? '真机' : modeStr.includes('sim') ? '仿真' : '未知'}`;
        }
    });

    logBus.addLog('info', 'ros_manager', '咖啡拉花话题订阅已建立', {}, 'latte');

    // 通知 coffee_latte_io.js 等依赖方：ROS 就绪，可以订阅 DI/DO 喵~
    window.dispatchEvent(new CustomEvent('latte:ros-ready'));
}

// ── 初始化 ──────────────────────────────────────────────────────────────────

async function init() {
    logBus.addLog('info', 'lifecycle', '咖啡拉花面板启动中...', {}, 'latte');

    // 1. 加载运行时配置
    await loadIvgRuntime();

    // 2. 尽早初始化 DOM 组件（与 vision_grasp 对齐，在慢速网络连接之前）喵~
    jointChart = createJointChartController({
        getById: $,
        maxSamples: 280,
        lineColors: ['#2563eb', '#16a34a', '#d97706', '#db2777', '#7c3aed', '#0d9488'],
    });
    jointChart.observeResize();

    monitoringCollapse = createMonitoringCollapse({ getById: $, jointChart, maxPx: 200 });
    monitoringCollapse.bindEvents();
    monitoringCollapse.scheduleSyncMinHeight();
    requestAnimationFrame(() => monitoringCollapse.scheduleSyncMinHeight());

    // 3. 连接 rosbridge（慢速网络操作）
    const url = rosbridgeWebSocketUrlFromRuntime(globalThis.__IVG_RUNTIME);
    await ros.connect(url);

    // 4. 启动 3D 查看器（依赖 ros._transport）
    urdfViewer = createUrdfViewer({
        ros: ros._transport?.ros,
        getById: $,
        hostId: 'vision-urdf-host',
        innerId: 'latte-urdf-inner',
        viewerOpts: {
            background: '#303030',
            intensity: 0.65,
            gridEnabled: true,
        },
    });
    urdfViewer.start();

    // 5. 建立话题订阅
    setupSubscriptions();

    // 6. 初始化拉花参数控制 (复用现有 latte_controls.js)
    latteControls = initLatteControls();

    // 7. 3D 预览轨迹渲染监听
    setupLattePreviewListener();

    // 8. 操作日志显示
    setupLatteLogDisplay();

    // 9. URDF 显示夹爪下拉框（仅仿真，不影响快换）喵~
    setupDisplayToolSelect();

    logBus.addLog('info', 'lifecycle', '咖啡拉花面板就绪', {}, 'latte');
}

function setupLatteLogDisplay() {
    const body = document.getElementById('latte-log-body');
    if (!body) return;

    const MAX_LINES = 200;

    function _timeStr() {
        return new Date().toLocaleTimeString('zh-CN', { hour12: false });
    }

    function _phaseBadge(phase) {
        if (!phase) return '';
        const labels = {
            start: '开始', in_progress: '进行中', completed: '完成',
            failed: '失败', blocked: '阻塞', skipped: '跳过',
            stop: '停止', home: '归零',
        };
        const label = labels[phase] || phase;
        return '<span class="latte-log-line__phase latte-log-line__phase--' + phase + '">' + label + '</span>';
    }

    logBus.onLog(function (entry) {
        if (!entry || entry.feature !== 'latte') return;

        var line = document.createElement('div');
        var levelCls = '';
        if (entry.level === 'error') levelCls = ' latte-log-line--error';
        else if (entry.level === 'warn') levelCls = ' latte-log-line--warn';
        if (entry.meta && entry.meta.success === true) levelCls += ' latte-log-line--ok';
        if (entry.meta && entry.meta.success === false) levelCls += ' latte-log-line--error';

        line.className = 'latte-log-line' + levelCls;

        var timeHtml = '<span class="latte-log-line__time">' + _timeStr() + '</span>';
        var badgeHtml = _phaseBadge(entry.meta?.phase);
        var msg = (entry.msg || '').replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
        var msgHtml = '<span class="latte-log-line__msg">' + msg + '</span>';

        line.innerHTML = timeHtml + badgeHtml + msgHtml;
        body.appendChild(line);

        while (body.children.length > MAX_LINES) {
            body.removeChild(body.firstChild);
        }
        body.scrollTop = body.scrollHeight;
    });

    // 清空按钮
    const clearBtn = document.getElementById('latte-log-clear');
    if (clearBtn) {
        clearBtn.addEventListener('click', () => { body.innerHTML = ''; });
    }
}

function setupLattePreviewListener() {
    document.addEventListener('latte:preview-ready', (e) => {
        const d = e.detail;
        if (!d || !d.waypoints || d.waypoints.length === 0) {
            logBus.addLog('warn', 'view3d', '预览: waypoints 为空, 跳过 3D 渲染', { waypoint_count: d?.waypoints?.length || 0 }, 'latte');
            return;
        }
        if (!urdfViewer) {
            logBus.addLog('warn', 'view3d', '预览: 3D 查看器未初始化 (urdfViewer=null), 无法渲染轨迹', {}, 'latte');
            return;
        }
        const session = typeof urdfViewer.getSession === 'function' ? urdfViewer.getSession() : null;
        if (!session || !session.viewer3d) {
            logBus.addLog('warn', 'view3d', '预览: 3D 场景未就绪 (viewer3d=' + (session ? 'null' : 'no-session') + '), 无法渲染轨迹', {}, 'latte');
            return;
        }
        if (typeof urdfViewer.clearTrajectoryLines === 'function') {
            urdfViewer.clearTrajectoryLines();
            urdfViewer.addTrajectoryLine(d.waypoints, '#ff6b6b', 2);
            logBus.addLog('info', 'view3d', '3D 轨迹已渲染: ' + d.waypoints.length + ' 个 waypoint', { waypoint_count: d.waypoints.length }, 'latte');
        }
    });
}

function setupDisplayToolSelect() {
    const sel = document.getElementById('latte-display-tool');
    if (!sel) return;

    // 初始值从 localStorage 恢复喵~
    var currentToolId = '';
    try { currentToolId = localStorage.getItem('ivg_last_tool_id') || ''; } catch (_) { /* */ }
    if (currentToolId) {
        var opt = sel.querySelector('option[value="' + currentToolId + '"]');
        if (opt) sel.value = currentToolId;
    }

    sel.addEventListener('change', async () => {
        var toolId = sel.value;
        logBus.addLog('info', 'service', 'URDF 显示切换: ' + (toolId || '无工具'), {
            phase: 'start', tool_id: toolId
        }, 'latte');

        try {
            var result = await ros.callService('/set_display_tool', 'ivg_interfaces/srv/ChangeTool', { tool_id: toolId }, 5000);
            if (result && result.success) {
                try { localStorage.setItem('ivg_last_tool_id', toolId); } catch (_) { /* */ }
                // 等后端 URDF 参数生效后重载 3D 喵~
                setTimeout(() => {
                    if (urdfViewer) {
                        var session = urdfViewer.getSession();
                        if (session && typeof session.reloadUrdf === 'function') {
                            session.reloadUrdf();
                        }
                    }
                }, 600);
                logBus.addLog('info', 'service', 'URDF 显示已切换: ' + (toolId || '无工具'), {
                    phase: 'completed', tool_id: toolId
                }, 'latte');
            } else {
                logBus.addLog('warn', 'service', 'URDF 显示切换失败: ' + (result?.message || '未知错误'), {
                    phase: 'failed', tool_id: toolId, message: result?.message || ''
                }, 'latte');
            }
        } catch (e) {
            logBus.addLog('error', 'service', 'URDF 显示切换异常: ' + (e.message || e), {
                phase: 'failed', tool_id: toolId, error: String(e.message || e)
            }, 'latte');
        }
    });

    logBus.addLog('info', 'system', 'URDF 显示夹爪选择器就绪 current=' + (currentToolId || '无工具'), {}, 'latte');
}

function cleanup() {
    if (latteControls && typeof latteControls.cleanup === 'function') { latteControls.cleanup(); latteControls = null; }
    if (urdfViewer) { if (urdfViewer.clearTrajectoryLines) urdfViewer.clearTrajectoryLines(); urdfViewer.stop(); urdfViewer = null; }
    if (jointChart) { jointChart.reset(); jointChart = null; }
    if (monitoringCollapse) { monitoringCollapse.destroy(); monitoringCollapse = null; }
    ros.disconnect();
    logBus.addLog('info', 'lifecycle', '咖啡拉花面板已清理', {}, 'latte');
}

// ── 页面生命周期 ────────────────────────────────────────────────────────────

createPageLifecycle({
    onInit: init,
    onCleanup: cleanup,
    onPause: () => ros.pause(),
    onResume: async () => {
        ros.resume();
        const url = rosbridgeWebSocketUrlFromRuntime(globalThis.__IVG_RUNTIME);
        await ros.connect(url);
        if (urdfViewer) urdfViewer.start();
    },
});
