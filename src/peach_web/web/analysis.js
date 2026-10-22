(function () {
  const $ = (id) => document.getElementById(id);
  const status = $('status');
  const result = $('result');

  function esc(value) {
    return String(value ?? '—').replace(/[&<>"']/g, (ch) => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
    }[ch]));
  }

  // 页面 CSP 不允许 innerHTML 里的 style 属性生效，写进 CSSOM 才占到宽度。
  function mount(host, html) {
    host.innerHTML = html;
    host.querySelectorAll('[style]').forEach((node) => {
      node.style.cssText = node.getAttribute('style');
    });
  }

  function rateCard(title, block, note) {
    const rate = block && block.sleeve_rate != null ? percent(block.sleeve_ok, block.attempts) : '—';
    const harvest = block && block.harvest_rate != null ? percent(block.harvest_ok, block.attempts) : '—';
    const attempts = (block && block.attempts) || 0;
    const sleeve = (block && block.sleeve_ok) || 0;
    const cut = (block && block.harvest_ok) || 0;
    return '<article><span>' + esc(title) + '</span><strong>' + esc(rate) +
      '</strong><small>' + esc(note || (sleeve + '/' + attempts + ' · 采摘确认 ' + harvest + ' · ' + cut)) +
      '</small></article>';
  }

  async function loadSuccess() {
    const response = await fetch('/api/analysis/success');
    const data = await response.json();
    const host = $('success-rates');
    if (!host) return;
    host.innerHTML = rateCard('今天', data.today) + rateCard('全部记录', data.all);
    const sessions = (data.sessions || []).filter((item) => item.attempts);
    const box = $('success-sessions');
    if (!box) return;
    box.innerHTML = sessions.length ? sessions.map((item) => {
      const width = (100 * item.sleeve_ok / item.attempts).toFixed(1);
      return '<div class="hbar"><span>' + esc(item.name) +
        '</span><span class="hbar-track"><i style="width:' + width +
        '%"></i></span><b>' + esc(item.sleeve_ok + '/' + item.attempts) +
        '</b></div>';
    }).join('') : '<p class="empty">还没有逐次抓取记录</p>';
    mount(box, box.innerHTML);
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

  function stageRows(stages) {
    if (Array.isArray(stages)) {
      return stages.map((stage) => ({
        name: stage.name,
        s: Number(stage.s) || 0,
      }));
    }
    return Object.keys(stages || {}).map((name) => ({
      name, s: Number(stages[name]) || 0,
    }));
  }

  function boundaryTicks(rows, total) {
    let cursor = 0;
    return rows.map((row) => {
      cursor += row.s;
      if (row.s <= 0 || cursor >= total - 0.05) return '';
      return '<b class="tick" style="left:' + (100 * cursor / total).toFixed(1) + '%"></b>';
    }).join('');
  }

  function forcePieces(item, total) {
    const parts = (item.parts || []).filter((part) =>
      Number(part.end_s) > Number(part.start_s));
    const pieces = parts.length ? parts : [{
      kind: 'span', start_s: item.start_s, end_s: item.end_s,
    }];
    const marks = pieces.map((part) => {
      const start = Math.max(0, Number(part.start_s) || 0);
      const end = Math.min(total, Number(part.end_s) || start);
      if (end <= start) return '';
      const kind = (part.kind === 'fall' ? 'force-fall' :
        (part.kind === 'rise' ? 'force-rise' : 'force-span')) +
        (pieces.length === 1 ? ' solo' : '');
      const label = part.kind === 'fall' ? '下降' : (part.kind === 'rise' ? '上升' : '五点力');
      return '<i class="' + kind + '" style="left:' + (100 * start / total).toFixed(3) +
        '%;width:' + (100 * (end - start) / total).toFixed(3) +
        '%" title="' + esc(label + ' ' + part.start_s + '–' + part.end_s + ' s') +
        '"></i>';
    }).join('');
    const rise = parts.find((part) => part.kind === 'rise');
    const fall = parts.find((part) => part.kind === 'fall');
    const peak = rise && fall
      ? '<b class="peak" style="left:' + (100 * Number(rise.end_s) / total).toFixed(3) +
        '%" title="' + esc('峰值 ' + rise.end_s + ' s') + '"></b>'
      : '';
    return marks + peak;
  }

  function renderPlot(cycle) {
    const rows = stageRows(cycle.stages);
    const staged = rows.reduce((sum, row) => sum + row.s, 0);
    const total = Math.max(Number(cycle.elapsed_s) || 0, staged);
    if (!total) return '';
    const segments = rows.filter((row) => row.s > 0).map((row) => {
      const width = (100 * row.s / total).toFixed(1);
      const label = Number(width) >= 16 ? esc(stageName[row.name] || row.name) : '';
      return '<i class="' + stageClass(row.name) + '" style="flex:0 0 ' +
        width + '%">' + label + '</i>';
    }).join('');
    const forces = cycle.force || [];
    const ticks = boundaryTicks(rows, total);
    const marks = forces.map((item) => forcePieces(item, total)).join('');
    const legend = rows.map((row) =>
      '<span><i class="' + stageClass(row.name) + '"></i>' +
      esc(stageName[row.name] || row.name) + ' ' + esc(row.s) + ' s</span>').join('') +
      '<span><i class="force-rise"></i>力上升</span>' +
      '<span><i class="force-fall"></i>力下降</span>';
    const notes = forces.map((item) => {
      const where = (item.stages || []).map((name) => stageName[name] || name);
      const peak = item.peak_n == null ? '' :
        '峰值 ' + (item.peak_channel || '') + ' ' + item.peak_n + ' N。';
      const place = where.length ? '落在' + where.join('和') + '。' : '';
      const shaped = (item.parts || []).map((part) =>
        (part.kind === 'fall' ? '下降' : '上升') + ' ' +
        part.start_s + '–' + part.end_s + ' s').join('，');
      const shape = shaped ? shaped + '。' : '';
      return shape + place + peak;
    });
    const head = cycle.title ? '<p class="cycle-title">' + esc(cycle.title) + '</p>' : '';
    const forceNote = notes.length ? notes.map((line) => esc(line)).join(' ') :
      '这次没有对上的五点力接触段。';
    return '<article class="cycle-plot">' + head +
      '<div class="cycle-row"><span>阶段</span><div class="cycle-stack">' +
      segments + ticks + '</div></div>' +
      '<div class="cycle-row"><span>五点力</span><div class="cycle-force">' +
      marks + ticks + '</div></div>' +
      '<div class="cycle-row cycle-scale"><span></span><div><b>0</b><b>合计 ' +
      total.toFixed(2) + ' s</b></div></div>' +
      '<p class="cycle-force-note">' + forceNote + '</p>' +
      '<div class="cycle-legend">' + legend + '</div></article>';
  }

  function renderCycle(host, report) {
    const matched = report.force_cycles || [];
    if (matched.length) {
      const extra = (report.rates && report.rates.fruits > matched.length)
        ? '<p class="analysis-note">周期条画的是和五点力对上的这一次抓取。上面的终局数字仍是所选时间窗里的全部批次，那条平均不能代表这一次力落在哪一段。</p>'
        : '';
      mount(host, extra + matched.map((cycle) => renderPlot({
        title: (cycle.when ? cycle.when + ' · ' : '') +
          (cycle.target_id || '目标') + (cycle.batch ? ' · ' + cycle.batch : ''),
        stages: cycle.stages,
        elapsed_s: cycle.elapsed_s,
        force: cycle.force,
      })).join(''));
      return;
    }
    const mean = (report.rates || {}).stage_mean_s || {};
    if (!Object.keys(mean).length) {
      host.innerHTML = '<p class="empty">没有数据</p>';
      return;
    }
    mount(host, renderPlot({
      title: '成功周期平均',
      stages: mean,
      force: [],
    }));
  }

  function bars(host, items, tone) {
    const max = Math.max(1, ...items.map((item) => item.value));
    mount(host, items.length ? items.map((item) =>
      '<div class="hbar ' + (tone || '') + '"><span>' + esc(item.label) +
      '</span><span class="hbar-track"><i style="width:' +
      (100 * item.value / max).toFixed(1) + '%"></i></span><b>' +
      esc(item.value) + '</b></div>').join('') : '<p class="empty">没有数据</p>');
  }

  function render(report) {
    result.hidden = false;
    $('result-title').textContent = report.path;
    const rates = report.rates || {};
    const fruits = rates.fruits || 0;
    const geometry = report.geometry || {};
    const d95 = geometry.d95_mm || {};
    const cards = [
      ['这次成功率', percent(rates.motion_completed, fruits),
        (rates.motion_completed || 0) + '/' + fruits + ' · 完成度 6'],
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
    renderCycle($('stage-bars'), report);
    $('cycle-note').textContent = '阶段条从派发该目标到撤离。红色升到五点合力最大的那一帧，蓝色从这一帧降到仍不低于 0.08 N 的最后一帧；归零后的等待不画。竖线是阶段分界，力条上的黑线是峰值。';
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
      const bar = stages.filter((stage) => Number(stage.s) > 0).map((stage) => {
        const width = (100 * Number(stage.s) / total).toFixed(1);
        return '<i class="' + stageClass(stage.name) +
          '" style="flex:0 0 ' + width + '%"></i>';
      }).join('');
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
    mount($('cycles'), body || '<tr><td colspan="13">没有逐果账本。</td></tr>');
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

  loadSuccess().catch((error) => {
    const note = $('success-note');
    if (note) note.textContent = '成功率不可用：' + error.message;
  });
  loadList().catch((error) => {
    status.className = 'error';
    status.textContent = '记录列表不可用：' + error.message;
  });
}());
