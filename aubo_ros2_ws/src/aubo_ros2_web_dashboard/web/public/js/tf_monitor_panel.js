// tf_monitor_panel.js — TF topic monitor with 3D viewer and 2D map
import { ivgPorts } from './ivg_runtime.js';
import { ivgTransport } from './ivg_transport.js';
const TF_MSG = 'tf2_msgs/msg/TFMessage';
const STORAGE_KEY = 'ivg_tf_monitor_topics_v1';
const ROS_RECONNECT_MAX = 12;
function normalizeFrameId(v) {
	const s = String(v || '').trim();
	if (!s) return '';
	return s.replace(/^\/+/, '');
}
function getById(id) {
	return document.getElementById(id);
}
function loadTopicsFromStorage() {
	try {
		const raw = localStorage.getItem(STORAGE_KEY);
		if (!raw) return;
		const o = JSON.parse(raw);
		if (!o || typeof o !== 'object') return;
		const tf = getById('topic-tf');
		const st = getById('topic-tf-static');
		if (tf && typeof o['topic-tf'] === 'string' && o['topic-tf'].trim()) tf.value = o['topic-tf'].trim();
		if (st && typeof o['topic-tf-static'] === 'string' && o['topic-tf-static'].trim()) st.value = o['topic-tf-static'].trim();
	} catch (_) {
	}
}
function saveTopicsToStorage() {
	const tf = getById('topic-tf');
	const st = getById('topic-tf-static');
	try {
		localStorage.setItem(
			STORAGE_KEY,
			JSON.stringify({
				'topic-tf': tf ? String(tf.value || '').trim() : '/tf',
				'topic-tf-static': st ? String(st.value || '').trim() : '/tf_static'
			})
		);
	} catch (_) {
	}
}
function canonicalTopic(t) {
	const s = String(t || '').trim();
	if (!s) return '';
	return s.startsWith('/') ? s : `/${s}`;
}
const childToParent = new Map();
let lastTfMsgAt = 0;
let lastStaticAt = 0;
let rafRender = 0;
function ingestTfMessage(msg) {
	const arr = msg && Array.isArray(msg.transforms) ? msg.transforms : [];
	for (let i = 0; i < arr.length; i++) {
		const t = arr[i];
		const parent = normalizeFrameId(t && t.header && t.header.frame_id);
		const child = normalizeFrameId(t && t.child_frame_id);
		if (!parent || !child) continue;
		childToParent.set(child, parent);
	}
}
function scheduleRender() {
	if (rafRender) return;
	rafRender = requestAnimationFrame(() => {
		rafRender = 0;
		renderTree();
	});
}
function getRoots() {
	const children = new Set(childToParent.keys());
	const roots = new Set();
	childToParent.forEach(parent => {
		if (!children.has(parent)) roots.add(parent);
	});
	return roots;
}
function buildChildrenMap() {
	const m = new Map();
	childToParent.forEach((parent, child) => {
		if (!m.has(parent)) m.set(parent, new Set());
		m.get(parent).add(child);
	});
	return m;
}
function renderTreeUl(frame, childrenMap, depth) {
	const kids = childrenMap.get(frame);
	const li = document.createElement('li');
	li.className = 'ivg-tf-tree__item';
	li.setAttribute('role', 'treeitem');
	li.setAttribute('aria-level', String(depth + 1));
	const span = document.createElement('span');
	span.className = 'ivg-tf-tree__frame';
	span.textContent = frame;
	li.appendChild(span);
	if (kids && kids.size) {
		const sorted = [...kids].sort((a, b) => a.localeCompare(b));
		const ul = document.createElement('ul');
		ul.className = 'ivg-tf-tree';
		ul.setAttribute('role', 'group');
		for (let i = 0; i < sorted.length; i++) {
			ul.appendChild(renderTreeUl(sorted[i], childrenMap, depth + 1));
		}
		li.appendChild(ul);
	}
	return li;
}
function renderTree() {
	const host = getById('tf-tree-root');
	const meta = getById('tf-tree-meta');
	const placeholder = getById('tf-tree-placeholder');
	if (!host) return;
	if (childToParent.size === 0) {
		if (placeholder) {
			placeholder.hidden = false;
			placeholder.textContent = ivgTransport.isConnected()
				? '已连接，尚未收到 TF 消息（检查 robot_state_publisher / tf 发布）。'
				: '未连接。';
		}
		if (meta) meta.textContent = '';
		return;
	}
	if (placeholder) placeholder.hidden = true;
	const childrenMap = buildChildrenMap();
	const roots = [...getRoots()].sort((a, b) => a.localeCompare(b));
	host.textContent = '';
	const outer = document.createElement('div');
	outer.className = roots.length > 1 ? 'ivg-tf-tree__multi-root' : '';
	if (roots.length === 0) {
		const p = document.createElement('p');
		p.className = 'tf-monitor-tree-placeholder';
		p.textContent = '无法确定根节点（可能存在环或未收到完整静态 TF）。';
		host.appendChild(p);
	} else {
		const ul = document.createElement('ul');
		ul.className = 'ivg-tf-tree';
		ul.setAttribute('role', 'tree');
		for (let i = 0; i < roots.length; i++) {
			ul.appendChild(renderTreeUl(roots[i], childrenMap, 0));
		}
		outer.appendChild(ul);
		host.appendChild(outer);
	}
	if (meta) {
		const parts = [`边数 ${childToParent.size}`];
		if (lastTfMsgAt) parts.push(`动态 TF ${new Date(lastTfMsgAt).toLocaleTimeString()}`);
		if (lastStaticAt) parts.push(`静态 ${new Date(lastStaticAt).toLocaleTimeString()}`);
		meta.textContent = parts.join(' · ');
	}
}
function setConnStatus(text, ok) {
	const el = getById('conn-status');
	if (!el) return;
	el.textContent = text;
	if (ok === true) el.className = 'status ok';
	else if (ok === false) el.className = 'status off';
	else el.className = 'status pending';
}
const rosReconnect = ivgPorts.createRosReconnectState();
let connectInFlight = false;
function unsubscribeTf() {
	childToParent.clear();
	lastTfMsgAt = 0;
	lastStaticAt = 0;
	ivgTransport.clearRosHandlers();
	ivgTransport.unsubscribeAll();
	scheduleRender();
}
function startTfSubscriptions() {
	unsubscribeTf();
	const tf = canonicalTopic(getById('topic-tf') && getById('topic-tf').value) || '/tf';
	const st = canonicalTopic(getById('topic-tf-static') && getById('topic-tf-static').value) || '/tf_static';
	ivgTransport.onRosJson(st, msg => {
		lastStaticAt = Date.now();
		ingestTfMessage(msg);
		scheduleRender();
	});
	ivgTransport.subscribe({ topic: st, msgType: TF_MSG, maxHz: 2 });
	ivgTransport.onRosJson(tf, msg => {
		lastTfMsgAt = Date.now();
		ingestTfMessage(msg);
		scheduleRender();
	});
	ivgTransport.subscribe({ topic: tf, msgType: TF_MSG, maxHz: 20 });
}
function connect() {
	if (connectInFlight) return;
	connectInFlight = true;
	ivgPorts.clearRosReconnectTimer(rosReconnect);
	const myGen = ivgPorts.bumpRosReconnectGen(rosReconnect);
	setConnStatus('正在连接…', null);
	ivgTransport.close();
	void (async () => {
		try {
			await ivgTransport.loadRuntime();
			await ivgTransport.connectControl();
			if (myGen !== rosReconnect.gen) return;
			rosReconnect.attempts = 0;
			ivgPorts.clearRosReconnectTimer(rosReconnect);
			ivgTransport.clearControlJsonHandlers();
			ivgTransport.onControlJson(o => {
				if (!o || typeof o !== 'object') return;
				if (o.op === 'error') {
					setConnStatus('已连接但发生错误，准备重连…', false);
				}
				if (o.op === 'close') {
					setConnStatus('连接已断开，准备重连…', false);
					unsubscribeTf();
					ivgPorts.scheduleRosReconnect(rosReconnect, connect, {
						maxAttempts: ROS_RECONNECT_MAX,
						onSchedule(delayMs, attempt, max) {
							setConnStatus(`已断开：${Math.round(delayMs / 1000)}s 后自动重连（${attempt}/${max}）`, false);
						},
						onExhausted() {
							setConnStatus('已断开（已达自动重连上限，请刷新页面）', false);
						}
					});
				}
			});
			setConnStatus('已连接', true);
			startTfSubscriptions();
		} catch (e) {
			if (myGen !== rosReconnect.gen) return;
			setConnStatus('连接错误', false);
			unsubscribeTf();
			ivgPorts.scheduleRosReconnect(rosReconnect, connect, {
				maxAttempts: ROS_RECONNECT_MAX,
				onSchedule(delayMs, attempt, max) {
					setConnStatus(`已断开：${Math.round(delayMs / 1000)}s 后自动重连（${attempt}/${max}）`, false);
				},
				onExhausted() {
					setConnStatus('已断开（已达自动重连上限，请刷新页面）', false);
				}
			});
		} finally {
			connectInFlight = false;
		}
	})();
}
function bindModal() {
	const openBtn = getById('btn-topic-settings-open');
	const closeBtn = getById('btn-topic-settings-close');
	const backdrop = getById('tf-topic-backdrop');
	const modal = getById('tf-topic-modal');
	if (!openBtn || !modal) return;
	function openModal() {
		modal.removeAttribute('hidden');
		modal.setAttribute('aria-hidden', 'false');
		closeBtn?.focus();
	}
	function closeModal() {
		modal.setAttribute('hidden', '');
		modal.setAttribute('aria-hidden', 'true');
		openBtn.focus();
	}
	openBtn.addEventListener('click', openModal);
	closeBtn?.addEventListener('click', closeModal);
	backdrop?.addEventListener('click', closeModal);
	modal.addEventListener('keydown', e => {
		if (e.key === 'Escape') closeModal();
	});
	getById('btn-topic-restore-defaults')?.addEventListener('click', () => {
		const tf = getById('topic-tf');
		const st = getById('topic-tf-static');
		if (tf) tf.value = '/tf';
		if (st) st.value = '/tf_static';
		try {
			localStorage.removeItem(STORAGE_KEY);
		} catch (_) {
		}
	});
	getById('btn-topic-save-reconnect')?.addEventListener('click', () => {
		saveTopicsToStorage();
		closeModal();
		connect();
	});
}
document.addEventListener('DOMContentLoaded', () => {
	void (async () => {
		await ivgPorts.loadRuntime();
		loadTopicsFromStorage();
		bindModal();
		ivgPorts.wireOnlineRosReconnect(rosReconnect, connect);
		connect();
	})();
});
