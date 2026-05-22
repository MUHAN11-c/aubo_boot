// latte/main.js — 咖啡拉花面板独立入口
// 不复用 vision_grasp_panel.js！独立初始化 3D 查看器 + 关节图 + 连接管理喵~
//
// 架构:
//   3D 查看器 ← view3d/urdf-viewer.js (ros3djs)
//   关节图     ← components/joint-chart.js
//   位姿显示   ← components/pose-card.js
//   监控区折叠 ← components/monitoring-collapse.js (共享)
//   连接管理   ← core/ros.js
//   拉花控制   ← latte/controls.js, latte/preview.js, latte/execute.js, latte/io.js
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
let _poseCache = { value: '' };

// ── 话题订阅 ────────────────────────────────────────────────────────────────

function setupSubscriptions() {
    if (!ros.isConnected) return;

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

        const poseEl = $('pose-text');
        if (poseEl && robotPoseHtmlIsRenderable(window.__ivgLastEEPose)) {
            const html = formatRobotPoseHtml(window.__ivgLastEEPose);
            if (html !== _poseCache.value) {
                _poseCache.value = html;
                poseEl.innerHTML = html;
            }
        }
    });

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

    logBus.addLog('info', 'ros_manager', '咖啡拉花话题订阅已建立');
}

// ── 初始化 ──────────────────────────────────────────────────────────────────

async function init() {
    logBus.addLog('info', 'lifecycle', '咖啡拉花面板启动中...');

    // 1. 加载运行时配置
    await loadIvgRuntime();

    // 2. 连接 rosbridge
    const url = rosbridgeWebSocketUrlFromRuntime(globalThis.__IVG_RUNTIME);
    await ros.connect(url);

    // 3. 启动 3D 查看器
    urdfViewer = createUrdfViewer({
        ros: ros._transport?.ros,
        getById: $,
        hostId: 'vision-urdf-host',
        innerId: 'latte-urdf-inner',
    });
    urdfViewer.start();

    // 4. 创建关节图
    jointChart = createJointChartController({
        getById: $,
        maxSamples: 280,
        lineColors: ['#2563eb', '#16a34a', '#d97706', '#db2777', '#7c3aed', '#0d9488'],
    });
    jointChart.observeResize();

    // 5. 创建监控区折叠组件（共享，消除重复代码）喵~
    monitoringCollapse = createMonitoringCollapse({ getById: $, jointChart });
    monitoringCollapse.bindEvents();
    monitoringCollapse.scheduleSyncMinHeight();
    requestAnimationFrame(() => monitoringCollapse.scheduleSyncMinHeight());

    // 6. 建立话题订阅
    setupSubscriptions();

    // 7. 初始化拉花参数控制 (复用现有 latte_controls.js)
    initLatteControls();

    // 8. 3D 预览轨迹渲染监听
    setupLattePreviewListener();

    logBus.addLog('info', 'lifecycle', '咖啡拉花面板就绪');
}

function setupLattePreviewListener() {
    document.addEventListener('latte:preview-ready', (e) => {
        const d = e.detail;
        if (!d || !d.waypoints || d.waypoints.length === 0) {
            logBus.addLog('warn', 'view3d', '预览: waypoints 为空');
            return;
        }
        if (urdfViewer && typeof urdfViewer.clearTrajectoryLines === 'function') {
            urdfViewer.clearTrajectoryLines();
            urdfViewer.addTrajectoryLine(d.waypoints, '#ff6b6b', 2);
            logBus.addLog('info', 'view3d', '3D 轨迹已渲染: ' + d.waypoints.length + ' 个 waypoint');
        }
    });
}

function cleanup() {
    if (urdfViewer) { if (urdfViewer.clearTrajectoryLines) urdfViewer.clearTrajectoryLines(); urdfViewer.stop(); urdfViewer = null; }
    if (jointChart) { jointChart.reset(); jointChart = null; }
    if (monitoringCollapse) { monitoringCollapse.destroy(); monitoringCollapse = null; }
    ros.disconnect();
    logBus.addLog('info', 'lifecycle', '咖啡拉花面板已清理');
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
