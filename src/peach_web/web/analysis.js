(function () {
  const $ = (id) => document.getElementById(id);
  const status = $('status');
  const result = $('result');

  function esc(value) {
    return String(value ?? '—').replace(/[&<>"']/g, (ch) => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
    }[ch]));
  }

  async function loadList() {
    const response = await fetch('/api/analysis/records');
    const data = await response.json();
    const list = $('records');
    list.innerHTML = '';
    (data.records || []).forEach((item) => {
      const option = document.createElement('option');
      option.value = item.path;
      option.label = item.markers.join(', ');
      list.appendChild(option);
    });
  }

  function render(report) {
    result.hidden = false;
    $('result-title').textContent = report.path;
    const geometry = report.geometry || {};
    const d95 = geometry.d95_mm || {};
    const margin = geometry.radial_margin_mm || {};
    const cards = [
      ['融合次数', geometry.fused_count],
      ['许可通过', geometry.allowed && geometry.allowed.true],
      ['许可未过', geometry.allowed && geometry.allowed.false],
      ['袋径范围 mm', d95.count ? d95.min + ' – ' + d95.max : '—'],
      ['径向余量 mm', margin.count ? margin.min + ' – ' + margin.max : '—'],
      ['账本条数', (report.outcomes || []).length],
    ];
    $('stats').innerHTML = cards.map(([name, value]) =>
      '<div><span>' + esc(name) + '</span><strong>' + esc(value) + '</strong></div>').join('');
    const outcomes = report.outcomes || [];
    const rounds = report.rounds || [];
    $('outcomes').innerHTML = '<div class="outcome-list">' + (outcomes.length ? outcomes : rounds).map((item) =>
      '<article><span>' + esc(item.target_id || item.request_id) + '</span><strong>' +
      esc(item.outcome ?? (item.success ? '测试通过' : item.error || '未通过')) +
      '</strong><small>' + esc(item.reason || item.elapsed_s) + '</small></article>').join('') +
      '</div>';
    const targets = geometry.targets || [];
    const rows = [];
    targets.forEach((target) => (target.points || []).forEach((point) => {
      rows.push('<tr><td>' + esc(target.target_id) + '</td><td>' + esc(point.name) +
        '</td><td>' + esc(point.xyz[0]) + '</td><td>' + esc(point.xyz[1]) + '</td><td>' + esc(point.xyz[2]) + '</td></tr>');
    }));
    $('points').innerHTML = rows.join('') || '<tr><td colspan="5">没有融合参考点</td></tr>';
    draw(targets);
    const images = report.images || [];
    $('images').innerHTML = images.length ? images.map((item) =>
      '<figure><img alt="' + esc(item.name) + '" src="' + esc(item.url) + '"><figcaption>' +
      esc(item.name) + '</figcaption></figure>').join('') :
      '<p class="empty">这份记录没有保存图片。参考点见左侧平面图和表格。</p>';
  }

  function draw(targets) {
    const plot = $('plot');
    const marks = [];
    targets.forEach((target) => (target.points || []).forEach((point) => {
      marks.push({name: point.name, x: point.xyz[0], y: point.xyz[1]});
    }));
    if (!marks.length) {
      plot.innerHTML = '<text x="20" y="40">没有可画的参考点</text>';
      return;
    }
    const xs = marks.map((item) => item.x);
    const ys = marks.map((item) => item.y);
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    const span = Math.max(maxX - minX, maxY - minY, 0.05);
    const mapX = (value) => 30 + ((value - minX) / span) * 280;
    const mapY = (value) => 220 - ((value - minY) / span) * 190;
    const colors = {袋底: '#087e58', 袋口: '#096e72', 剪切参考: '#bc3348'};
    const occupied = [];
    const labelY = (x, y) => {
      let candidate = y;
      for (let attempt = 0; attempt < 6; attempt += 1) {
        if (!occupied.some((box) => Math.abs(box.x - x) < 46 && Math.abs(box.y - candidate) < 12)) break;
        candidate += 12;
      }
      candidate = Math.min(candidate, 250);
      occupied.push({x, y: candidate});
      return candidate;
    };
    plot.innerHTML = marks.map((item) => {
      const px = mapX(item.x);
      const py = mapY(item.y);
      const tx = Math.min(px + 8, 296);
      return '<circle cx="' + px.toFixed(1) + '" cy="' + py.toFixed(1) +
        '" r="5" fill="' + (colors[item.name] || '#21343e') + '"></circle><text x="' +
        tx.toFixed(1) + '" y="' + labelY(px, py).toFixed(1) + '">' +
        esc(item.name) + '</text>';
    }).join('') +
      '<text x="16" y="268" fill="#5c707b">平面是 base_link 的 x / y，不是图像像素</text>';
  }

  $('pick').addEventListener('submit', async (event) => {
    event.preventDefault();
    const path = $('path').value.trim();
    status.className = 'empty';
    status.textContent = '正在解析 ' + path;
    try {
      const response = await fetch('/api/analysis?path=' + encodeURIComponent(path));
      const data = await response.json();
      if (!response.ok) throw new Error(data.error || response.status);
      status.textContent = (data.notes || []).join(' ');
      render(data);
    } catch (error) {
      status.className = 'error';
      status.textContent = error.message;
      result.hidden = true;
    }
  });

  loadList().catch((error) => {
    status.className = 'error';
    status.textContent = '记录列表不可用：' + error.message;
  });
}());
