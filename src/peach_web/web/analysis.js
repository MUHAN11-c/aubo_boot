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

  const outcomeName = {0: '周期完成', 1: '质量跳过', 2: '不可达', 3: '失败', 4: '取消'};
  const stageName = {
    perceive: '精化等待', observe: '感知识别', finalize: '精化等待',
    reconfirm: '再确认', approach_insert: '接近与套入',
    tool: '刀具', retreat: '撤离',
  };
  const skipName = {
    missing_mask: '缺掩膜', target_drift: '目标漂移', neighbor_gap: '邻距',
    near_duplicate: '重复视角', joint_state_unavailable: '无关节',
  };

  function percent(part, whole) {
    if (!whole) return '—';
    return Math.round(100 * part / whole) + '%';
  }

  function stageClass(name) {
    return 'stage-' + String(name).replace(/[^a-z0-9_]/g, '');
  }

  function renderCycle(host, stages) {
    const keys = Object.keys(stages);
    const values = keys.map((key) => Number(stages[key]) || 0);
    const total = values.reduce((sum, value) => sum + value, 0);
    if (!total) {
      host.innerHTML = '<p class="empty">没有数据</p>';
      return;
    }
    host.innerHTML = '<div class="cycle-stack">' + keys.map((key, index) =>
      '<i class="' + stageClass(key) + '" style="width:' +
      (100 * values[index] / total).toFixed(1) + '%"></i>').join('') +
      '</div><p class="cycle-total">合计 ' + total.toFixed(2) +
      ' s</p><div class="cycle-legend">' + keys.map((key, index) =>
      '<span><i class="' + stageClass(key) + '"></i>' +
      esc(stageName[key] || key) + ' ' + esc(values[index]) + ' s</span>').join('') +
      '</div>';
  }

  function bars(host, items, tone) {
    const max = Math.max(1, ...items.map((item) => item.value));
    host.innerHTML = items.length ? items.map((item) =>
      '<div class="hbar ' + (tone || '') + '"><span>' + esc(item.label) +
      '</span><span class="hbar-track"><i style="width:' +
      (100 * item.value / max).toFixed(1) + '%"></i></span><b>' +
      esc(item.value) + '</b></div>').join('') : '<p class="empty">没有数据</p>';
  }

  function render(report) {
    result.hidden = false;
    $('result-title').textContent = report.path;
    const rates = report.rates || {};
    const fruits = rates.fruits || 0;
    const geometry = report.geometry || {};
    const d95 = geometry.d95_mm || {};
    const cards = [
      ['运动周期完成', percent(rates.motion_completed, fruits), (rates.motion_completed || 0) + '/' + fruits],
      ['不可达', percent(rates.unreachable, fruits), (rates.unreachable || 0) + ' 颗'],
      ['质量跳过', percent(rates.quality_skipped, fruits), (rates.quality_skipped || 0) + ' 颗'],
      ['失败', percent(rates.failed, fruits), (rates.failed || 0) + ' 颗'],
      ['采摘确认', percent(rates.harvest_confirmed, fruits), '完成度 7'],
      ['单果中位', rates.elapsed_s_median != null ? rates.elapsed_s_median + ' s' : '—',
        d95.count ? '袋径 ' + d95.min + '–' + d95.max + ' mm' : ''],
    ];
    $('rates').innerHTML = cards.map(([name, value, note]) =>
      '<article><span>' + esc(name) + '</span><strong>' + esc(value) +
      '</strong><small>' + esc(note) + '</small></article>').join('');
    const outcomes = report.outcomes || [];
    const tally = {};
    outcomes.forEach((item) => {
      const name = outcomeName[item.outcome] || String(item.outcome ?? '未知');
      tally[name] = (tally[name] || 0) + 1;
    });
    bars($('outcome-bars'), Object.keys(tally).map((label) => ({label, value: tally[label]})));
    renderCycle($('stage-bars'), rates.stage_mean_s || {});
    $('cycle-note').textContent = '精化等待是派发该目标到再确认之前的时间，主要是等模型和许可。开批时的检测不在这段里。运动周期完成是套入后的撤离确认；采摘确认才是完成度 7。';
    const skips = report.skip_codes || {};
    const skipItems = Object.keys(skips).map((key) => ({
      label: skipName[key] || key, value: skips[key],
    })).sort((a, b) => b.value - a.value);
    const accepted = rates.frames_accepted || 0;
    const skipped = rates.frames_skipped || 0;
    bars($('skip-bars'), skipItems.length ? skipItems : [{label: '接受帧', value: accepted}], 'warn');
    if (accepted || skipped) {
      $('skip-bars').insertAdjacentHTML('afterbegin',
        '<p class="analysis-note">接受 ' + accepted + ' / ' + (accepted + skipped) + '</p>');
    }
    const rows = report.fruits || [];
    function completion(level) {
      const value = Number(level);
      if (value >= 7) return '采摘确认';
      if (value >= 6) return '撤退确认';
      if (value >= 5) return '切断确认';
      if (Number.isFinite(value)) return String(value);
      return '—';
    }
    function cell(value, digits, unit) {
      if (value == null || value === '') return '—';
      const number = Number(value);
      if (!Number.isFinite(number)) return '—';
      return (digits == null ? String(value) : number.toFixed(digits)) + (unit || '');
    }
    function xyz(points, name) {
      const point = (points || []).find((item) => item.name === name);
      if (!point || !point.xyz) return '—';
      return point.xyz.map((value) => Number(value).toFixed(3)).join(' ');
    }
    function center(item) {
      const point = item.center_m;
      if (!point || point.length < 2) return '—';
      return Number(point[0]).toFixed(3) + ' ' + Number(point[1]).toFixed(3);
    }
    function shot(item, label) {
      const caption = '<figcaption>' + esc(label) + '</figcaption>';
      if (!item) {
        return '<figure><div class="shot-missing">没有这一帧</div>' + caption + '</figure>';
      }
      return '<figure><img class="cycle-shot" alt="' + esc(label) + '" src="' +
        esc(item.url) + '">' + caption + '</figure>';
    }
    function stageBar(stages) {
      const total = (stages || []).reduce((sum, stage) => sum + Number(stage.s || 0), 0);
      if (!total) return '';
      const bar = stages.map((stage) =>
        '<i class="' + stageClass(stage.name) +
        '" style="width:' + (100 * Number(stage.s || 0) / total).toFixed(1) + '%"></i>').join('');
      const caption = stages.map((stage) =>
        (stageName[stage.name] || stage.name) + ' ' + stage.s + ' s').join(' · ');
      return '<div class="stage-bar" title="' + esc(caption) + '">' + bar + '</div>';
    }
    function roundHeading(batch) {
      const name = String(batch || '');
      const timed = name.match(/_(\d{2})(\d{2})(\d{2})_r(\d+)/);
      if (timed) {
        return '第 ' + Number(timed[4]) + ' 轮 · ' +
          timed[1] + ':' + timed[2] + ':' + timed[3] + ' · ' + name;
      }
      return name || '未命名批次';
    }
    const groups = [];
    rows.forEach((item) => {
      const key = String(item.batch || '');
      let group = groups.find((entry) => entry.key === key);
      if (!group) {
        group = {key, items: []};
        groups.push(group);
      }
      group.items.push(item);
    });
    const body = groups.map((group) => {
      const heading = '<tr class="batch"><td colspan="13">' + esc(roundHeading(group.key)) + '</td></tr>';
      const lines = group.items.map((item, index) => {
        const done = item.outcome === 0;
        const tone = done ? 'ok' : 'bad';
        return '<tr class="shots ' + tone + '"><td colspan="13"><div class="shot-pair">' +
          shot(item.mask, '掩膜') + shot(item.debug, '检测框、分割和方向') +
          '</div></td></tr>' +
          '<tr class="' + tone + '">' +
          '<td class="sticky col-ord">' + (index + 1) + '</td>' +
          '<td class="sticky col-id">' + esc(item.target_id) + '</td>' +
          '<td class="outcome"><span class="fruit-pill">' +
          esc(outcomeName[item.outcome] || item.outcome) + '</span><small title="' +
          esc(item.reason) + '">' + esc(item.reason) + '</small></td>' +
          '<td class="time">' + cell(item.elapsed_s, null, ' s') + stageBar(item.stages || []) + '</td>' +
          '<td>' + esc(completion(item.completion_level)) + '</td>' +
          '<td>' + cell(item.d95_mm, 1) + '</td>' +
          '<td>' + cell(item.radial_margin_mm, 1) + '</td>' +
          '<td>' + cell(item.axial_margin_mm, 1) + '</td>' +
          '<td>' + esc(center(item)) + '</td>' +
          '<td>' + esc(xyz(item.points, '袋底')) + '</td>' +
          '<td>' + esc(xyz(item.points, '袋口')) + '</td>' +
          '<td>' + esc(xyz(item.points, '剪切参考')) + '</td>' +
          '<td>' + cell(item.view_count) + '</td></tr>';
      }).join('');
      return heading + lines;
    }).join('');
    $('cycles').innerHTML = body || '<tr><td colspan="13">没有逐果账本。</td></tr>';
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
