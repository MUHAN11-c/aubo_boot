/**
 * 咖啡拉花 IO 绑定模块
 * 连接 DO2/DO4 按钮与 DI2/DI3/DI4 信号灯到 rosbridge 服务/话题。
 * 由 coffee_latte_panel.html 加载，依赖 vision_grasp_panel.js 已建立的 ROS 连接。
 */
import { ivgTransport } from '../ivg_transport.js';

const DO2_ON = '开';
const DO2_OFF = '关';
const DO4_ON = '开';
const DO4_OFF = '关';

function initLatteIO() {
	const btnDo2 = document.getElementById('latte-do2-toggle');
	const btnDo4 = document.getElementById('latte-do4-toggle');
	const lampDi2 = document.getElementById('latte-di2-lamp');
	const lampDi3 = document.getElementById('latte-di3-lamp');
	const lampDi4 = document.getElementById('latte-di4-lamp');
	const log = document.getElementById('svc-log');

	function svcLog(msg) {
		if (!log) return;
		const p = document.createElement('p');
		p.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
		log.prepend(p);
		if (log.children.length > 20) log.lastChild.remove();
	}

	function setLamp(el, state) {
		if (!el) return;
		el.setAttribute('data-state', state ? 'on' : 'off');
		el.style.backgroundColor = state ? '#4caf50' : '#555';
	}

	// ── DO 按钮绑定 ──
	function toggleDO(btn, ioIndex, onText, offText) {
		const svcName = ioIndex === 2 ? '/set_latte_do2' : '/set_latte_do4';
		const current = btn.getAttribute('aria-pressed') === 'true';
		const next = !current;

		ivgTransport.callService({
			service: svcName,
			type: 'std_srvs/srv/SetBool',
			request: { data: next }
		}).then(r => {
			if (r.success) {
				btn.setAttribute('aria-pressed', String(next));
				btn.textContent = next ? onText : offText;
				btn.classList.toggle('latte-io-toggle--on', next);
				svcLog(`${svcName} → ${next ? onText : offText}`);
			} else {
				svcLog(`${svcName} 失败: ${r.message || 'unknown'}`);
			}
		}).catch(e => {
			svcLog(`${svcName} 错误: ${e.message || e}`);
		});
	}

	if (btnDo2) {
		btnDo2.disabled = false;
		btnDo2.onclick = () => toggleDO(btnDo2, 2, DO2_ON, DO2_OFF);
	}
	if (btnDo4) {
		btnDo4.disabled = false;
		btnDo4.onclick = () => toggleDO(btnDo4, 4, DO4_ON, DO4_OFF);
	}

	// ── DI 状态订阅 ──
	try {
		const diTopic = new ROSLIB.Topic({
			ros: ivgTransport._ros,
			name: '/latte_di_status',
			messageType: 'std_msgs/msg/String',
			throttle_rate: 2000
		});
		diTopic.subscribe(msg => {
			if (typeof msg.data !== 'string') return;
			// 格式: "DO2=ON DO4=OFF | DI2=HI DI3=LO DI4=HI"
			const di2Match = msg.data.match(/DI2=(\w+)/);
			const di3Match = msg.data.match(/DI3=(\w+)/);
			const di4Match = msg.data.match(/DI4=(\w+)/);
			if (di2Match) setLamp(lampDi2, di2Match[1] === 'HI');
			if (di3Match) setLamp(lampDi3, di3Match[1] === 'HI');
			if (di4Match) setLamp(lampDi4, di4Match[1] === 'HI');
		});
	} catch (e) {
		console.warn('latte_io: DI 订阅失败', e);
	}
}

// 延迟初始化，等待 vision_grasp_panel.js 建立 ROS 连接
setTimeout(initLatteIO, 2000);
