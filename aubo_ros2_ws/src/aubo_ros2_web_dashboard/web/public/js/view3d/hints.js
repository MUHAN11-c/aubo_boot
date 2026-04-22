/**
 * 3D 页面提示工具：
 * - 把点云/URDF 的提示 DOM 操作从 session 与 loader 里抽出来。
 * - 保留旧页面依赖的 class/id 约定，避免样式和行为回归。
 */
(function (global) {
	'use strict';

	function removeView3dPc2Hint(hostEl) {
		if (hostEl) hostEl.querySelectorAll('.ivg-pc2-hint').forEach(n => n.remove());
		const leg = document.getElementById('view3d-pc2-hint');
		if (leg) leg.remove();
	}

	function showView3dPc2Hint(hostEl, html) {
		removeView3dPc2Hint(hostEl);
		if (!hostEl) return;
		if (hostEl.hasAttribute('data-ivg-hide-hints')) return;
		const d = document.createElement('div');
		d.className = 'hint ivg-pc2-hint';
		d.style.marginTop = '0.5rem';
		d.innerHTML = html;
		hostEl.appendChild(d);
	}

	function removeView3dUrdfHint(hostEl) {
		if (hostEl) hostEl.querySelectorAll('.ivg-urdf-hint').forEach(n => n.remove());
		const leg = document.getElementById('view3d-urdf-hint');
		if (leg) leg.remove();
	}

	function showView3dUrdfHint(hostEl, html) {
		removeView3dUrdfHint(hostEl);
		if (!hostEl) return;
		if (hostEl.hasAttribute('data-ivg-hide-hints')) return;
		const d = document.createElement('div');
		d.className = 'hint ivg-urdf-hint';
		d.style.marginTop = '0.5rem';
		d.innerHTML = html;
		hostEl.appendChild(d);
	}

	global.IVGView3DHints = {
		removeView3dPc2Hint,
		showView3dPc2Hint,
		removeView3dUrdfHint,
		showView3dUrdfHint
	};
})(typeof window !== 'undefined' ? window : globalThis);
