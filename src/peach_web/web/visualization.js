(function (root) {
  'use strict';
  const finite = (v) => (typeof v === 'number' || typeof v === 'string' && v.trim() !== '') && Number.isFinite(Number(v));
  function frameInfo(meta, now) {
    if (!meta || !finite(meta.stamp_sec) || Number(meta.stamp_sec) <= 0) {
      return {available: false, stale: false, age: null, label: meta?.error ? '图像不可用：' + meta.error : '尚无图像'};
    }
    const age = finite(meta.age_s) ? Number(meta.age_s) : finite(now) ? Math.max(0, Number(now) - Number(meta.stamp_sec)) : null;
    const stale = meta.error === 'stale' || (finite(age) && age > 1);
    if (meta.error === 'clock_mismatch' || (finite(age) && age < 0)) {
      return {available:false,stale:false,age,label:'时钟不匹配 · 图像不可用'};
    }
    return {available: meta.available === true && finite(age) && !stale, stale, age,
      label: stale ? '图像陈旧' : !finite(age) ? '帧龄未报告' : meta.available === true ? '图像更新中' : '图像不可用：' + (meta.error || '等待编码')};
  }
  function modelInfo(state) {
    const diagnostics = state?.reconstruction?.diagnostics || {};
    const r = diagnostics.refined || {};
    const targets = state?.perception?.targets || {};
    const target = state?.ui?.selected_target_id || state?.job?.target_id || targets.selected_target_id;
    const epoch = state?.ui?.selected_scene_epoch ?? targets.scene_epoch;
    const context = diagnostics.refined_context || r;
    const known = Boolean(target && context.target_id && context.harvest_run_id && targets.harvest_run_id)
      && epoch != null && context.scene_epoch != null;
    const matches = known && context.target_id === target && context.scene_epoch === epoch
      && context.harvest_run_id === targets.harvest_run_id;
    const parts = [`当前目标 ${target || '未知'} / 场景 ${epoch ?? '未知'}`,
      known ? matches ? '模型身份匹配' : '身份不匹配 · 当前模型不可用' : '模型身份未知 · 等待场景/目标/运行证据',
      r.model_revision ? '诊断模型 ' + r.model_revision : '尚无模型版本'];
    parts.push(finite(r.stamp) ? '模型时间 ' + Number(r.stamp).toFixed(3) + ' s' : '模型时间未报告');
    if (finite(diagnostics.stamp)) parts.push('诊断发布 ' + Number(diagnostics.stamp).toFixed(3) + ' s');
    const age = state?.system?.topic_age_s?.['reconstruction.diagnostics'];
    parts.push(finite(age) ? '诊断接收距今 ' + Number(age).toFixed(1) + ' s' : '诊断新鲜度未报告');
    const rmse = r.refit_rmse ?? r.rmse;
    if (finite(rmse) && Number(rmse) >= 0) parts.push('点拟合 ' + (Number(rmse) * 1000).toFixed(1) + ' mm');
    if (r.fusion_consistency_available === false) parts.push('多视一致性不可用');
    else if (r.fusion_consistency_available === true && finite(r.fusion_landmark_scatter_rmse_m)) {
      parts.push('端点散布 ' + (Number(r.fusion_landmark_scatter_rmse_m) * 1000).toFixed(1) + ' mm');
    }
    if (r.reason === 'fused_axis_conflict') parts.push('错轴：仅诊断几何');
    return parts.join(' · ');
  }
  const escape = v => String(v ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  function analysisHTML(data) {
    if (!data) return '<p class="empty">等待模型与几何证据</p>';
    const m = data.metrics, d = data.decision, r = data.reachability;
    const number = (v, unit='mm', digits=1) => typeof v === 'number' && Number.isFinite(v)
      ? (unit==='mm' ? v*1000 : unit==='%' ? v*100 : v).toFixed(digits)+' '+unit : '—';
    const tiles = [['袋径',m.diameter_m],['剪切行程',m.cut_travel_m],['预抓取后撤',m.standoff_m],['横向偏差',m.lateral_error_m]];
    const values = [['径向 D95',m.d95_m,'mm'],['入口→剪切距离',m.entry_cut_m,'mm'],['轴向投影',m.axial_travel_m,'mm'],
      ['投影 − 下发行程',m.travel_error_m,'mm'],['初值→融合入口位移',m.entry_shift_m,'mm'],
      ['初值 / 融合轴夹角',m.initial_fused_angle_deg,'°'],['袋轴 / 实际工具轴',m.tool_axis_angle_deg,'°'],
      ['点云拟合 RMSE',m.refit_rmse_m,'mm'],['重拟合内点比',m.inlier_ratio,'%'],
      ['入口距 base 原点',m.entry_distance_m,'m'],['剪切点距 base 原点',m.cut_distance_m,'m']];
    const margin = (label,v) => `<div class="viz-budget ${v===null?'unknown':v<0?'bad':'good'}"><span>${label}</span><div class="viz-budget-track"><i data-margin-size="${v===null?0:Math.min(50,Math.abs(v)*1000)}" data-margin-left="${v<0?50-Math.min(50,Math.abs(v)*1000):50}"></i></div><b>${number(v)}</b></div>`;
    const route = [['P',data.scene.pregrasp,'预抓取'],['E',data.scene.entry,'袋外入口'],['C',data.scene.cut,'剪切参考']];
    const capabilities = [['几何','geometry_capability'],['预抓取','pregrasp_capability'],['套入','sleeve_capability'],['剪切','cut_capability']];
    return `<div class="viz-diagnosis ${escape(r.tone)}"><span>机械臂 / 路径证据</span><strong>${escape(r.label)}</strong><p>${escape(r.detail)}</p></div>
      <div class="viz-metric-grid">${tiles.map(([name,v])=>`<div><span>${name}</span><strong>${number(v)}</strong></div>`).join('')}</div>
      <div class="viz-route" aria-label="抓取路径关键点">${route.map(([id,p,name],i)=>`${i?'<span class="viz-route-arrow">→</span>':''}<div class="${p?'':'missing'}"><b>${id}</b><span>${name}</span></div>`).join('')}</div>
      <p class="viz-formula">Δ = 剪切点 − 入口 · 轴向 = Δ·axis · 横向 = ‖Δ − (Δ·axis)axis‖<br>虚线为几何参考线，规划轨迹以实际结果为准。</p>
      <div class="viz-permit ${d?.allowed===true?'good':d?.allowed===false?'bad':'unknown'}"><span>几何接触许可</span><b>${d?.allowed===true?'允许':d?.allowed===false?'未允许':'未知'}</b><small>${escape(d?.reason || '等待同场景许可')}</small></div>
      ${!d && data.scene.fused ? '<p class="viz-formula">几何已到 · 当前目标许可未到或身份不匹配；接触能力未知。</p>' : ''}
      <div class="viz-capabilities">${capabilities.map(([name,key])=>`<span class="${d?.[key]===0?'good':d?.[key]===1?'bad':'unknown'}">${name} · ${d?.[key]===0?'有效':d?.[key]===1?'无效':'未知'}</span>`).join('')}</div>
      ${margin('径向余量',m.radial_margin_m)}${margin('轴向余量',m.axial_margin_m)}
      <p class="viz-formula">余量以 0 为中线，负值向左。显示后端预算，页面不修改门限。</p>
      <details class="viz-calculations"><summary>计算与质量证据</summary><dl>${values.map(([name,v,unit])=>`<dt>${name}</dt><dd>${number(v,unit,unit==='m'?3:1)}</dd>`).join('')}</dl>
      <p class="viz-formula">${data.consistency===false?'单视角：多视一致性不可用':data.consistency===true?'已报告多视一致性':'多视一致性未报告'} · ${number(m.captured_views,'视角',0)}<br>原点距离不代表 IK 可达范围。</p></details>
      <details class="viz-keypoints"><summary>关键点坐标 · base_link / m</summary><div class="viz-points">${data.points.length?data.points.map(p=>`<div><span>${escape(p.name)}</span><code>${p.position.map(v=>v.toFixed(4)).join(' / ')}</code></div>`).join(''):'<p class="empty">无有效关键点</p>'}</div></details>
      ${data.flags.length?`<p class="viz-flags">${data.flags.map(escape).join(' · ')}</p>`:''}`;
  }
  function create(host) {
    const panel = document.createElement('section');
    panel.className = 'visualization-panel';
    panel.dataset.mode = 'camera';
    panel.innerHTML = `<div class="viz-heading"><div><p class="kicker">GEOMETRY & REACHABILITY</p><h2>视觉、几何与可达性</h2></div><div class="viz-tabs" role="tablist" aria-label="可视化画面"><button type="button" data-viz-tab="camera" aria-selected="true">相机图</button><button type="button" data-viz-tab="scene" aria-selected="false">三维对照</button></div></div>
      <div class="viz-workspace"><div class="viz-surfaces"><section class="viz-image-surface" aria-label="相机调试图"><div class="viz-toolbar"><label>图像来源 <select data-image-source><option value="perception">稳定识别图</option><option value="raw">即时检测图</option></select></label><button type="button" data-image-pause>暂停图像</button><button type="button" data-image-zoom>放大</button></div><div class="viz-image-box"><img alt="等待相机调试图" hidden><p data-image-empty>尚无图像</p></div><div class="viz-legend viz-image-legend"><span class="viz-image-axis">底 → 口主轴</span><span class="viz-image-cut">剪切参考</span><span>彩色轮廓：套袋分割</span></div><p class="viz-meta" data-image-status>等待图像元数据</p></section>
      <section class="viz-scene-surface" aria-label="目标与末端三维对照"><div class="viz-toolbar"><span>base_link · 米 · Z 向上</span><button type="button" data-scene-view="center">目标居中</button><button type="button" data-scene-view="top">俯视</button><button type="button" data-scene-view="side">侧视</button><button type="button" data-scene-view="reset">复位</button></div><div class="viz-scene-box" data-scene-host></div><div class="viz-fallback" hidden><p>三维不可用，显示二维 TCP 轨迹与数值对照。</p><div data-fallback-host></div></div><div class="viz-legend"><span class="viz-initial">感知初值</span><span class="viz-fused">融合结果</span><span class="viz-tool">实际末端</span><span class="viz-rejected">错轴诊断</span></div><p class="viz-meta" data-model-status></p><p class="viz-meta" data-tf-status></p><p class="viz-meta" data-axis-status></p></section></div>
      <aside class="viz-analysis" aria-label="关键计算与可达性分析" data-analysis></aside></div>
      <p class="viz-time-note">图像、模型与 TF 为异步观测，各自显示时间；旋转画面与暂停图像不会发送机器人指令。</p>
      <dialog class="viz-image-dialog"><button type="button" data-close-image>关闭大图</button><img alt="相机调试图放大"></dialog>`;
    host.appendChild(panel);
    const find = (selector) => panel.querySelector(selector);
    const image = find('.viz-image-box img');
    const empty = find('[data-image-empty]');
    const dialog = find('dialog');
    const zoomImage = dialog.querySelector('img');
    let scene;
    try { scene = root.PeachViz3D?.create(find('[data-scene-host]')); } catch (_) { scene = null; }
    function showFallback() {
      find('[data-scene-host]').hidden = true;
      find('.viz-fallback').hidden = false;
      const canvas = document.getElementById('tcp-canvas');
      if (canvas) find('[data-fallback-host]').appendChild(canvas);
    }
    if (!scene?.available) showFallback();
    let state = {}, trajectory = {}, source = 'perception', paused = false, active = true;
    let disposed = false, generation = 0, inFlight = false, controller = null, url = null, frame = null;
    const clock = () => root.performance?.now?.() ?? Date.now();
    let imageMetadata = null, imageMetadataReceivedAt = clock();
    let imageError = '';
    const listeners = [];
    function listen(el, event, fn) { el.addEventListener(event, fn); listeners.push([el, event, fn]); }
    listen(find('[data-scene-host]'), 'peach-viz3d-unavailable', showFallback);
    function showImageMeta() {
      const original = frame || state.images?.[source];
      const receivedAt = frame?._receivedAt ?? imageMetadataReceivedAt;
      const meta = original ? {...original, age_s: finite(original.age_s)
        ? Number(original.age_s) + (clock() - receivedAt) / 1000 : null} : null;
      const info = frameInfo(meta, null);
      const stamp = meta && finite(meta.stamp_sec) ? Number(meta.stamp_sec).toFixed(6) + ' s' : '—';
      const age = finite(info.age) ? info.age.toFixed(1) + ' s' : '—';
      find('[data-image-status]').textContent = `${source === 'raw' ? '即时检测图' : '稳定识别图'} · ${paused ? '显示已暂停 · ' + info.label : imageError || info.label} · 曝光 ${stamp} · 帧龄 ${age} · ${meta?.frame_id || '帧系未知'}`;
    }
    async function pullImage() {
      if (!disposed) showImageMeta();
      if (disposed || paused || !active || document.hidden || inFlight) return;
      const meta = state.images?.[source];
      if (!meta?.available && meta?.error !== 'stale') {
        empty.textContent = frameInfo(meta, state.system?.server_time).label;
        if (!url) empty.hidden = false;
        showImageMeta(); return;
      }
      const capturedSource = source, capturedGeneration = generation;
      const requestedAt = clock();
      inFlight = true;
      controller = new AbortController();
      const ownController = controller;
      const timeout = setTimeout(() => ownController.abort(), 6000);
      try {
        const response = await fetch(`/api/images/${capturedSource === 'raw' ? 'raw' : 'perception'}.jpg?t=${Date.now()}`, {signal: ownController.signal});
        if (!response.ok) {
          if (response.status === 404) {
            const unavailable = await response.json();
            if (capturedGeneration === generation && !disposed) {
              generation++; frame = null; image.hidden = true; empty.hidden = false;
              empty.textContent = '图像不可用：' + (unavailable.error || 'no_frame');
              if (dialog.open) dialog.close();
              image.removeAttribute('src'); zoomImage.removeAttribute('src');
              if (url) URL.revokeObjectURL(url); url = null;
              imageError = empty.textContent;
            }
          }
          throw new Error('HTTP ' + response.status);
        }
        const blob = await response.blob();
        if (disposed || generation !== capturedGeneration || source !== capturedSource || paused || !active
            || (!state.images?.[capturedSource]?.available && state.images?.[capturedSource]?.error !== 'stale')) return;
        const newUrl = URL.createObjectURL(blob);
        const previous = url;
        imageError = '';
        url = newUrl;
        image.src = newUrl; zoomImage.src = newUrl;
        image.alt = capturedSource === 'raw' ? '即时检测图' : '稳定识别图';
        image.hidden = false; empty.hidden = true;
        if (previous) URL.revokeObjectURL(previous);
        const ns = response.headers.get('X-Frame-Stamp-Ns');
        const responseAge = response.headers.get('X-Frame-Age-S');
        const ageAtRequest = finite(responseAge) ? Number(responseAge) : finite(meta.age_s) ? Number(meta.age_s) : null;
        frame = {...meta, available: true, error: '',
          age_s: ageAtRequest === null ? null : ageAtRequest + (clock() - requestedAt) / 1000,
          _receivedAt: clock(),
          stamp_sec: ns && finite(ns) ? Number(ns) / 1e9 : meta.stamp_sec,
          frame_id: response.headers.get('X-Frame-Id') || meta.frame_id};
        showImageMeta();
      } catch (error) {
        if (!disposed && capturedGeneration === generation && !ownController.signal.aborted) {
          imageError = '图像获取失败 · 保留上次画面 · ' + error.message;
          showImageMeta();
        }
      } finally { clearTimeout(timeout); inFlight = false; if (controller === ownController) controller = null; }
    }
    listen(find('[data-image-source]'), 'change', (event) => {
      source = event.target.value; generation++; controller?.abort();
      frame = null; imageError = ''; image.hidden = true; empty.hidden = false; empty.textContent = '等待所选图像';
      if (url) URL.revokeObjectURL(url); url = null; image.removeAttribute('src'); zoomImage.removeAttribute('src');
      showImageMeta(); pullImage();
    });
    listen(find('[data-image-pause]'), 'click', (event) => {
      paused = !paused;
      if (paused) { generation++; controller?.abort(); }
      event.target.textContent = paused ? '恢复图像' : '暂停图像';
      showImageMeta(); if (!paused) pullImage();
    });
    listen(find('[data-image-zoom]'), 'click', () => { if (url) dialog.showModal(); });
    listen(find('[data-close-image]'), 'click', () => dialog.close());
    panel.querySelectorAll('[data-viz-tab]').forEach((el) => listen(el, 'click', () => {
      panel.dataset.mode = el.dataset.vizTab;
      panel.querySelectorAll('[data-viz-tab]').forEach((button) => button.setAttribute('aria-selected', String(button === el)));
      scene?.resize();
      find(el.dataset.vizTab === 'camera' ? '.viz-image-surface' : '.viz-scene-surface')
        ?.scrollIntoView?.({behavior: 'smooth', block: 'nearest'});
    }));
    panel.querySelectorAll('[data-scene-view]').forEach((el) => listen(el, 'click', () => scene?.setView(el.dataset.sceneView)));
    const timer = setInterval(pullImage, 200);
    return {
      update(nextState, nextTrajectory) {
        state = nextState || {}; trajectory = nextTrajectory || {};
        if (state.images !== imageMetadata) { imageMetadata = state.images; imageMetadataReceivedAt = clock(); }
        if (!paused && !state.images?.[source]?.available && state.images?.[source]?.error !== 'stale') {
          generation++; controller?.abort();
          frame = null; image.hidden = true; empty.hidden = false;
          if (dialog.open) dialog.close();
          image.removeAttribute('src'); zoomImage.removeAttribute('src');
          if (url) URL.revokeObjectURL(url); url = null;
        }
        showImageMeta();
        find('[data-model-status]').textContent = modelInfo(state);
        const frames = trajectory.tool_frames || {};
        find('[data-tf-status]').textContent = ['tool_axis', 'sleeve_mouth', 'cutting_plane'].map((name) => {
          const tf = frames[name];
          return `${name}: ${tf?.available ? (finite(tf.stamp_sec) ? Number(tf.stamp_sec).toFixed(3) + ' s' : '时间未报告') : tf?.error || '缺失'}${finite(tf?.age_s) ? ' / 帧龄 ' + Number(tf.age_s).toFixed(1) + ' s' : ' / 帧龄未知'}`;
        }).join(' · ');
        scene?.update(state, trajectory);
        const normalized = root.PeachViz3D?.normalizeScene?.(state, trajectory);
        const angle = normalized?.axisAngleDeg ?? normalized?.metrics?.axisAngleDeg;
        find('[data-axis-status]').textContent = finite(angle) ? '袋轴 / 工具轴夹角 ' + Number(angle).toFixed(1) + '°' : '袋轴 / 工具轴夹角：证据不足';
        const analysisHost = find('[data-analysis]');
        const html = analysisHTML(root.PeachViz3D?.analyzeScene?.(state,trajectory));
        if (analysisHost.innerHTML !== html) {
          const opened = new Set(Array.from(analysisHost.querySelectorAll('details')).filter(el=>el.open).map(el=>el.className));
          analysisHost.innerHTML = html;
          analysisHost.querySelectorAll('details').forEach(el=>{el.open=opened.has(el.className);});
          analysisHost.querySelectorAll('[data-margin-size]').forEach(el=>{
            el.style.width=el.dataset.marginSize+'%'; el.style.left=el.dataset.marginLeft+'%';
          });
        }
        pullImage();
      },
      moveTo(nextHost) { if (nextHost && panel.parentElement !== nextHost) nextHost.appendChild(panel); scene?.resize(); },
      setActive(value) { active = value; if (!active) { generation++; controller?.abort(); } else { scene?.resize(); pullImage(); } },
      dispose() {
        disposed = true; clearInterval(timer); controller?.abort();
        listeners.forEach(([el, event, fn]) => el.removeEventListener(event, fn));
        scene?.dispose(); if (url) URL.revokeObjectURL(url); panel.remove();
      },
    };
  }
  const api = {frameInfo, modelInfo, analysisHTML, create};
  root.PeachVisualization = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
