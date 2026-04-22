/**
 * 视觉抓取页面设置控制器：
 * - 负责话题/服务配置的 DOM 读写与 localStorage 持久化。
 * - 负责设置弹窗开关。
 * - 主页面脚本只保留“何时保存/何时重连”的业务编排。
 */
(function (global) {
	'use strict';

	function createVisionSettingsController(opts) {
		const options = opts || {};
		const $ = options.getById || function (id) { return document.getElementById(id); };
		const allIds = options.allIds || [];
		const defaults = options.defaults || {};
		const sanitizeTopicValue = options.sanitizeTopicValue || function (_id, value) { return String(value || ''); };
		const sanitizeTopicConfig = options.sanitizeTopicConfig || function (cfg) { return cfg; };
		const storageKey = options.storageKey || 'ivg_vision_grasp_topics_v3';
		const doc = options.documentRef || document;

		function applyDefaultsToDom() {
			allIds.forEach(id => {
				const el = $(id);
				if (el && defaults[id] !== undefined) el.value = sanitizeTopicValue(id, defaults[id]);
			});
		}

		function readFromDom() {
			const out = {};
			allIds.forEach(id => {
				const el = $(id);
				out[id] = sanitizeTopicValue(id, el ? el.value : '');
			});
			return out;
		}

		function loadFromStorage() {
			const keys = [storageKey, 'ivg_vision_grasp_topics_v2', 'ivg_vision_grasp_topics_v1'];
			for (let k = 0; k < keys.length; k++) {
				try {
					const raw = localStorage.getItem(keys[k]);
					if (!raw) continue;
					const o = sanitizeTopicConfig(JSON.parse(raw));
					if (!o || typeof o !== 'object') continue;
					const ok = allIds.every(id => {
						if (typeof o[id] !== 'string') return false;
						if (id === 'topic-result') return true;
						return o[id].length > 0;
					});
					if (!ok) continue;
					allIds.forEach(id => {
						const el = $(id);
						if (el) el.value = o[id];
					});
					return true;
				} catch (e) {
					/* try next key */
				}
			}
			return false;
		}

		function saveToStorage() {
			try {
				localStorage.setItem(storageKey, JSON.stringify(sanitizeTopicConfig(readFromDom())));
			} catch (e) {
				/* ignore quota / private mode */
			}
		}

		function clearStorage() {
			try {
				localStorage.removeItem(storageKey);
			} catch (e) {
				/* ignore */
			}
		}

		function modalOpen() {
			const m = $('topic-settings-modal');
			return m && !m.hasAttribute('hidden');
		}

		function openModal() {
			const m = $('topic-settings-modal');
			if (!m) return;
			m.removeAttribute('hidden');
			m.setAttribute('aria-hidden', 'false');
			doc.body.style.overflow = 'hidden';
			const first = $('topic-color');
			if (first) first.focus();
		}

		function closeModal() {
			const m = $('topic-settings-modal');
			if (!m) return;
			m.setAttribute('hidden', '');
			m.setAttribute('aria-hidden', 'true');
			doc.body.style.overflow = '';
			const btn = $('btn-topic-settings-open');
			if (btn) btn.focus();
		}

		return {
			applyDefaultsToDom,
			readFromDom,
			loadFromStorage,
			saveToStorage,
			clearStorage,
			modalOpen,
			openModal,
			closeModal
		};
	}

	global.IVGVisionSettings = {
		createVisionSettingsController
	};
})(typeof window !== 'undefined' ? window : globalThis);
