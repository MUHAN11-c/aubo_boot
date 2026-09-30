// hints.js — URDF loading placeholder UI
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
export {
	removeView3dUrdfHint,
	showView3dUrdfHint
};
