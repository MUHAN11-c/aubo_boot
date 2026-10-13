(function (root) {
  'use strict';
  const finite = (v) => v !== null && v !== undefined && v !== '' && Number.isFinite(Number(v));
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
    const parts = [r.model_revision ? '模型 ' + r.model_revision : '尚无模型版本'];
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
  function create(host) {
    const panel = document.createElement('section');
    panel.className = 'visualization-panel';
    panel.dataset.mode = 'camera';
    panel.innerHTML = `<div class="viz-heading"><h2>定位与方向</h2><div class="viz-tabs" role="tablist" aria-label="可视化画面"><button type="button" data-viz-tab="camera" aria-selected="true">相机图</button><button type="button" data-viz-tab="scene" aria-selected="false">三维对照</button></div></div>
      <div class="viz-surfaces"><section class="viz-image-surface" aria-label="相机调试图"><div class="viz-toolbar"><label>图像来源 <select data-image-source><option value="perception">稳定识别图</option><option value="raw">即时检测图</option></select></label><button type="button" data-image-pause>暂停图像</button><button type="button" data-image-zoom>放大</button></div><div class="viz-image-box"><img alt="等待相机调试图" hidden><p data-image-empty>尚无图像</p></div><p class="viz-meta" data-image-status>等待图像元数据</p></section>
      <section class="viz-scene-surface" aria-label="目标与末端三维对照"><div class="viz-toolbar"><span>base_link · 米 · Z 向上</span><button type="button" data-scene-view="center">目标居中</button><button type="button" data-scene-view="top">俯视</button><button type="button" data-scene-view="side">侧视</button><button type="button" data-scene-view="reset">复位</button></div><div class="viz-scene-box" data-scene-host></div><div class="viz-fallback" hidden><p>三维不可用，显示二维 TCP 轨迹与数值对照。</p><div data-fallback-host></div></div><div class="viz-legend"><span class="viz-initial">感知初值</span><span class="viz-fused">融合结果</span><span class="viz-tool">实际末端</span><span class="viz-rejected">错轴诊断</span></div><p class="viz-meta" data-model-status></p><p class="viz-meta" data-tf-status></p><p class="viz-meta" data-axis-status></p></section></div>
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
      find('[data-image-status]').textContent = `${source === 'raw' ? '即时检测图' : '稳定识别图'} · ${paused ? '显示已暂停' : imageError || info.label} · 曝光 ${stamp} · 帧龄 ${age} · ${meta?.frame_id || '帧系未知'}`;
    }
    async function pullImage() {
      if (!disposed) showImageMeta();
      if (disposed || paused || !active || document.hidden || inFlight) return;
      const meta = state.images?.[source];
      if (!meta?.available) {
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
            || !state.images?.[capturedSource]?.available) return;
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
        frame = {...meta, available: true, error: '',
          age_s: (responseAge !== null && finite(responseAge) ? Number(responseAge) : Number(meta.age_s))
            + (clock() - requestedAt) / 1000,
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
  const api = {frameInfo, modelInfo, create};
  root.PeachVisualization = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
