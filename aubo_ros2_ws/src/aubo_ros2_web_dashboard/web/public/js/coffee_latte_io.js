// coffee_latte_io.js — 咖啡拉花 DI 反馈灯 + DO 开关
// 依赖: ros.js (RosManager 单例), logBus (日志总线)
// 链路: 浏览器 ↔ rosbridge ← /latte_di_status (String)
//       DO 按钮 → /set_latte_do2, /set_latte_do4 (std_srvs/SetBool)

import { ros } from './core/ros.js';
import { logBus } from './core/log-bus.js';

const TAG = '[latte_io]';

// 话题名（可从 settings localStorage 覆盖）喵~
const STORAGE_KEY = 'ivg_vision_grasp_topics_v3';

function getSetting(key, fallback) {
    try {
        const raw = localStorage.getItem(STORAGE_KEY);
        if (raw) {
            const obj = JSON.parse(raw);
            if (obj && obj[key] && obj[key].trim()) return obj[key].trim();
        }
    } catch (_) { /* ignore */ }
    return fallback;
}

// ── DI 反馈灯 ──────────────────────────────────────────────────────────────

function setLamp(elId, on) {
    const el = document.getElementById(elId);
    if (!el) return;
    const prev = el.getAttribute('data-state');
    el.setAttribute('data-state', on ? 'on' : 'off');
    return prev !== (on ? 'on' : 'off');
}

function parseDiMessage(msg) {
    let text = '';
    if (typeof msg === 'string') {
        text = msg;
    } else if (msg && typeof msg.data === 'string') {
        text = msg.data;
    } else if (msg && msg.data !== undefined) {
        text = String(msg.data);
    } else {
        return;
    }

    // JSON: {"di2":1, "di3":0, "di4":1}
    try {
        const obj = JSON.parse(text);
        if (obj && typeof obj === 'object') {
            _updateDi('di2', !!obj.di2);
            _updateDi('di3', !!obj.di3);
            _updateDi('di4', !!obj.di4);
            return;
        }
    } catch (_) { /* fall through */ }

    // CSV: "1,0,1"
    const parts = text.split(',').map(s => s.trim());
    if (parts.length >= 3) {
        _updateDi('di2', parts[0] === '1');
        _updateDi('di3', parts[1] === '1');
        _updateDi('di4', parts[2] === '1');
    }
}

const DI_LABELS = { di2: '咖啡反馈 DI2', di3: '打花反馈 DI3', di4: '预警反馈 DI4' };

function _updateDi(name, on) {
    const changed = setLamp('latte-' + name + '-lamp', on);
    if (changed) {
        logBus.addLog('info', 'topic', 'DI 反馈: ' + DI_LABELS[name] + '=' + (on ? 'ON' : 'OFF'), {
            di: name,
            state: on ? 'on' : 'off',
        });
    }
}

// ── DO 开关（调用 ROS 服务）────────────────────────────────────────────────

const DO_SERVICES = {
    do2: { svc: '/set_latte_do2', label: '打花开关 DO2' },
    do4: { svc: '/set_latte_do4', label: '咖啡开关 DO4' },
};

async function toggleDo(name) {
    const info = DO_SERVICES[name];
    if (!info) return;

    const btn = document.getElementById('latte-' + name + '-toggle');
    if (!btn) return;

    const pressed = btn.getAttribute('aria-pressed') === 'true';
    const next = !pressed;

    // 乐观更新: 先切换 UI
    btn.setAttribute('aria-pressed', next ? 'true' : 'false');
    btn.textContent = next ? '开' : '关';
    btn.classList.toggle('is-on', next);

    logBus.addLog('info', 'service', 'DO 开关: ' + info.label + ' → ' + (next ? 'ON' : 'OFF'), {
        do: name, service: info.svc, target: next,
    });

    // 调用 ROS 服务
    try {
        const result = await ros.callService(info.svc, 'std_srvs/srv/SetBool', { data: next }, 10000);
        if (result.success === false) {
            _rollbackDoUI(btn, pressed);
            logBus.addLog('warn', 'service', info.svc + ' 返回 success=false, UI 已回滚', {
                service: info.svc,
            });
        } else {
            logBus.addLog('info', 'service', '✓ ' + info.svc + ' → success=' + result.success, {
                service: info.svc, success: result.success, message: result.message,
            });
        }
    } catch (e) {
        _rollbackDoUI(btn, pressed);
        logBus.addLog('error', 'service', '✗ ' + info.svc + ' 失败: ' + String(e) + ', UI 已回滚', {
            service: info.svc, error: String(e),
        });
    }
}

function _rollbackDoUI(btn, prev) {
    btn.setAttribute('aria-pressed', prev ? 'true' : 'false');
    btn.textContent = prev ? '开' : '关';
    btn.classList.toggle('is-on', prev);
}

// ── 订阅 ───────────────────────────────────────────────────────────────────

function subscribeDi() {
    const topic = getSetting('latte-di-topic', '/latte_di_status');

    ros.subscribe(topic, 'std_msgs/msg/String', msg => {
        parseDiMessage(msg);
    });

    logBus.addLog('info', 'topic', '已订阅 DI 状态: ' + topic, { topic });
}

// ── 入口 ───────────────────────────────────────────────────────────────────

(function init() {
    logBus.addLog('info', 'system', '咖啡拉花 IO 初始化中...');

    // DO 按钮：调用 ROS 服务
    const do2 = document.getElementById('latte-do2-toggle');
    const do4 = document.getElementById('latte-do4-toggle');
    if (do2) do2.addEventListener('click', () => toggleDo('do2'));
    if (do4) do4.addEventListener('click', () => toggleDo('do4'));

    // DI 订阅：等 ros.js 连接就绪后订阅
    // 由于 ros.js 在 latte/main.js 中连接，这里先轮询等待
    (function waitForRos() {
        let ticks = 0;
        const timer = setInterval(() => {
            if (ros.isConnected) {
                clearInterval(timer);
                subscribeDi();
                logBus.addLog('info', 'system', '咖啡拉花 IO 就绪 (DI 订阅 + DO 服务)');
            }
            if (++ticks > 120) {
                clearInterval(timer);
                logBus.addLog('warn', 'system', '咖啡拉花 IO: ros.js 连接超时，DI 未订阅');
            }
        }, 500);
    })();
})();
