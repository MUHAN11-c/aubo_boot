/**
 * 3D 页面提示工具（ES module）：
 * - 把 URDF 相关提示 DOM 操作从 session 与 loader 里抽出来。
 * - 保留旧页面依赖的 class/id 约定，避免样式和行为回归。
 */

	/** 移除 URDF 提示节点（.ivg-urdf-hint / #view3d-urdf-hint）。 */
	function removeView3dUrdfHint(hostEl) {
		if (hostEl) hostEl.querySelectorAll('.ivg-urdf-hint').forEach(n => n.remove());
		const leg = document.getElementById('view3d-urdf-hint');
		if (leg) leg.remove();
	}

	/** 在宿主下追加 URDF 提示 div。 */
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

export {
	removeView3dUrdfHint,
	showView3dUrdfHint
};
