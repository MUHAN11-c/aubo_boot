/**
 * 咖啡拉花面板 — 纯前端演示：顶视杯口 + 奶泡拉花路径动画；无 roslib / 无后端。
 */
(() => {
    const canvas = document.getElementById('latte-preview-canvas');
    const logEl = document.getElementById('latte-log-text');
    const statusEl = document.getElementById('latte-demo-status');
    const btnStart = document.getElementById('btn-latte-demo-start');
    const btnReset = document.getElementById('btn-latte-demo-reset');

    if (!canvas || !canvas.getContext) return;

    const ctx = canvas.getContext('2d');
    let DPR = Math.min(window.devicePixelRatio || 1, 2);
    const W = 640;
    const H = 480;
    let animId = null;
    let animT = 0;

    function resizeCanvas() {
		canvas.width = Math.round(W * DPR);
		canvas.height = Math.round(H * DPR);
		ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
		drawFrame(0);
	}

    function logLine(msg) {
		if (!logEl) return;
		const t = new Date().toLocaleTimeString('zh-CN', { hour12: false });
		logEl.textContent = `[${t}] ${msg}\n${logEl.textContent}`;
		const lines = logEl.textContent.split('\n');
		if (lines.length > 80) logEl.textContent = lines.slice(0, 80).join('\n');
	}

    function setStatus(idle, text) {
		if (!statusEl) return;
		statusEl.textContent = text;
		statusEl.classList.toggle('is-idle', !!idle);
	}

    /** 杯口椭圆中心、半轴 */
    function cupGeom() {
		const cx = W * 0.5;
		const cy = H * 0.48;
		const rx = Math.min(W, H) * 0.28;
		const ry = rx * 0.88;
		return { cx, cy, rx, ry };
	}

    function drawCupBase() {
		const g = cupGeom();
		ctx.save();
		ctx.fillStyle = '#c4a574';
		ctx.strokeStyle = '#94a3b8';
		ctx.lineWidth = 3;
		ctx.beginPath();
		ctx.ellipse(g.cx, g.cy, g.rx, g.ry, 0, 0, Math.PI * 2);
		ctx.fill();
		ctx.stroke();

		ctx.fillStyle = '#3d2914';
		ctx.beginPath();
		ctx.ellipse(g.cx, g.cy + 4, g.rx * 0.88, g.ry * 0.88, 0, 0, Math.PI * 2);
		ctx.fill();

		ctx.fillStyle = '#6b4423';
		ctx.beginPath();
		ctx.ellipse(g.cx, g.cy + 2, g.rx * 0.82, g.ry * 0.82, 0, 0, Math.PI * 2);
		ctx.fill();

		ctx.fillStyle = '#a67c52';
		ctx.beginPath();
		ctx.ellipse(g.cx, g.cy - 2, g.rx * 0.75, g.ry * 0.75, 0, 0, Math.PI * 2);
		ctx.fill();
		ctx.restore();
	}

    /**
	 * 标准心形参数曲线（归一化中心 0,0），再缩放到杯内奶面。
	 * tParam 0..1 用于逐步描边。
	 */
    function heartPoint(tFull) {
		const t = ((tFull % 1) + 1) % 1;
		const a = t * Math.PI * 2;
		const x = 16 * Math.pow(Math.sin(a), 3);
		const y = -(13 * Math.cos(a) - 5 * Math.cos(2 * a) - 2 * Math.cos(3 * a) - Math.cos(4 * a));
		return { x, y };
	}

    function drawLatteProgress(progress) {
		const g = cupGeom();
		const scale = Math.min(g.rx, g.ry) / 22;
		const steps = 200;
		const n = Math.floor(progress * steps);

		ctx.save();
		ctx.strokeStyle = 'rgba(255, 252, 245, 0.92)';
		ctx.lineWidth = 5;
		ctx.lineCap = 'round';
		ctx.lineJoin = 'round';
		ctx.beginPath();
		let first = true;
		for (let i = 0; i <= n; i++) {
			const t = (i / steps) * 0.92;
			const p = heartPoint(t);
			const px = g.cx + p.x * scale * 0.42;
			const py = g.cy + p.y * scale * 0.42 - 6;
			if (first) {
				ctx.moveTo(px, py);
				first = false;
			} else {
				ctx.lineTo(px, py);
			}
		}
		ctx.stroke();
		ctx.restore();
	}

    function drawFrame(latteProgress) {
		ctx.clearRect(0, 0, W, H);
		ctx.fillStyle = '#e8ecf2';
		ctx.fillRect(0, 0, W, H);
		drawCupBase();
		if (latteProgress > 0) drawLatteProgress(latteProgress);
	}

    function stopAnim() {
		if (animId != null) {
			cancelAnimationFrame(animId);
			animId = null;
		}
	}

    function runDemo() {
		stopAnim();
		const start = performance.now();
		const duration = 3200;
		setStatus(false, '演示中…');
		logLine('开始前端拉花路径演示（非真实机械臂）');
		btnStart.disabled = true;

		function frame(now) {
			animT = Math.min(1, (now - start) / duration);
			drawFrame(animT);
			if (animT < 1) {
				animId = requestAnimationFrame(frame);
			} else {
				animId = null;
				setStatus(true, '演示完成 · 无后端');
				btnStart.disabled = false;
				logLine('演示结束。接入 ROS 后可在此显示真实状态与相机画面。');
			}
		}
		animId = requestAnimationFrame(frame);
	}

    function resetDemo() {
		stopAnim();
		animT = 0;
		drawFrame(0);
		setStatus(true, '就绪 · 纯前端');
		btnStart.disabled = false;
		logLine('已重置预览画布。');
	}

    resizeCanvas();
    window.addEventListener('resize', () => {
		DPR = Math.min(window.devicePixelRatio || 1, 2);
		resizeCanvas();
	});

    if (btnStart) btnStart.addEventListener('click', runDemo);
    if (btnReset) btnReset.addEventListener('click', resetDemo);

    setStatus(true, '就绪 · 纯前端');
    logLine('咖啡拉花面板已加载：当前为占位 UI，后端话题与服务待接入。');
})();
