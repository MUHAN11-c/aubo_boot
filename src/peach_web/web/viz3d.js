/* Read-only geometry in base_link metres. No ROS commands or inferred TF. */
(function (root) {
  'use strict';
  const COLORS = Object.freeze({initial: '#2864b4', fused: '#ba621c', tool: '#087f73', diagnostic: '#bc3348'});
  const vector = (value, size = 3) => Array.isArray(value) && value.length >= size &&
    value.slice(0, size).every(v => typeof v === 'number' && Number.isFinite(v)) ? value.slice(0, size) : null;
  const direction = value => { const v = vector(value); const n = v && Math.hypot(...v); return n > 1e-9 ? v.map(x => x / n) : null; };
  const sameTarget = (id, target) => Boolean(target) && id === target;
  function candidateGeometry(candidate, source) {
    return {bottom: vector(candidate.bag_bottom), neck: vector(candidate.bag_neck),
      entry: vector(candidate.entry_position), axis: direction(candidate.translation_direction),
      diameter: typeof candidate.diameter_m === 'number' && candidate.diameter_m > 0 ? candidate.diameter_m : null,
      stamp: source.stamp, model_revision: candidate.model_revision, source, diagnostic: false};
  }
  function normalizeScene(state = {}, trajectory = {}) {
    const job = state.job || {}; const targets = state.perception?.targets || {};
    const target = state.ui?.selected_target_id || job.target_id || state.task_executor?.state?.target_id || targets.selected_target_id || '';
    const sceneEpoch = state.ui?.selected_scene_epoch ?? targets.scene_epoch;
    const decision = state.reconstruction?.grasp_decision || {};
    const validEpoch = Number.isInteger(sceneEpoch) && sceneEpoch > 0;
    const decisionScoped = validEpoch && sameTarget(decision.target_id, target) && decision.scene_epoch === sceneEpoch &&
      (!decision.harvest_run_id || decision.harvest_run_id === targets.harvest_run_id);
    function modelScoped(...sources) {
      const epochs = sources.map(source => source?.scene_epoch).filter(epoch => epoch != null);
      const runs = sources.map(source => source?.harvest_run_id).filter(run => run != null);
      return validEpoch && epochs.length > 0 && epochs.every(epoch => epoch === sceneEpoch) &&
        Boolean(targets.harvest_run_id) && runs.length > 0 &&
        runs.every(run => Boolean(run) && run === targets.harvest_run_id);
    }
    const epochMatches = state.ui?.selected_scene_epoch == null || state.ui.selected_scene_epoch === targets.scene_epoch;
    const observation = (targets.observations || []).find(item => sameTarget(item.target_id, target));
    const initial = observation && epochMatches && targets.frame_id === 'base_link'
      ? candidateGeometry(observation.candidate || {}, targets) : null;
    const diagnostics = state.reconstruction?.diagnostics || {};
    const refined = diagnostics.refined;
    const poseSource = state.refined?.pose || {};
    const refinedCandidate = (poseSource.candidates || []).find(item => sameTarget(item.target_id, target));
    let fused = refinedCandidate && epochMatches && modelScoped(refinedCandidate, poseSource) && poseSource.frame_id === 'base_link'
      ? candidateGeometry(refinedCandidate, poseSource) : null;
    const refinedContext = diagnostics.refined_context;
    const rawScoped = refinedContext != null
      ? validEpoch && refinedContext.scene_epoch === sceneEpoch &&
        sameTarget(refinedContext.target_id, target) && Boolean(refinedContext.harvest_run_id) &&
        refinedContext.harvest_run_id === targets.harvest_run_id && modelScoped(refined, refinedContext)
      : sameTarget(refined?.target_id, target) && modelScoped(refined);
    if (refined && epochMatches && rawScoped && (!diagnostics.frame_id || diagnostics.frame_id === 'base_link')) {
      fused = {bottom: vector(refined.bottom), neck: vector(refined.neck), entry: fused?.entry || null,
        center: vector(refined.center), axis: direction(refined.axis),
        diameter: typeof refined.diameter === 'number' && refined.diameter > 0 ? refined.diameter : null,
        stamp: refined.stamp ?? diagnostics.stamp, model_revision: refined.model_revision ?? fused?.model_revision,
        source: refined, context: refinedContext, diagnostic: refined.ok !== true, reason: refined.reason || ''};
    }
    const grasp = decisionScoped ? decision : {};
    const coords = job.coords || {};
    const geometryFrame = epochMatches && decisionScoped &&
      (decision.frame_id || coords.frame_id) === 'base_link' && Boolean(direction(grasp.axis));
    const toolFrames = {};
    for (const name of ['tool_axis', 'sleeve_mouth', 'cutting_plane']) {
      const source = trajectory.tool_frames?.[name];
      if (!source?.available || source.frame_id !== 'base_link') continue;
      const position = vector(source.position); const quaternion = vector(source.orientation, 4);
      const length = quaternion && Math.hypot(...quaternion);
      if (!position || !(length > 1e-9)) continue;
      const T = root.PeachThree?.THREE;
      if (!T) continue;
      const rotation = new T.Quaternion(...quaternion).normalize();
      toolFrames[name] = {...source, position, orientation: rotation.toArray(),
        axis: new T.Vector3(0, 0, 1).applyQuaternion(rotation).toArray()};
    }
    const points = [];
    if (trajectory.frame_id === 'base_link' && Array.isArray(trajectory.xyz)) {
      for (let i = 0; i + 2 < trajectory.xyz.length; i += 3) {
        const p = vector(trajectory.xyz.slice(i, i + 3)); if (p) points.push(p);
      }
    }
    const bagAxis = fused?.axis; const toolAxis = toolFrames.tool_axis?.axis;
    const T = root.PeachThree?.THREE;
    const axisAngleDeg = T && bagAxis && toolAxis
      ? new T.Vector3(...bagAxis).angleTo(new T.Vector3(...toolAxis)) * 180 / Math.PI : null;
    return {target_id: target, scene_epoch: state.ui?.selected_scene_epoch ?? targets.scene_epoch,
      frame_id: 'base_link', initial, fused, axisAngleDeg, decision: decisionScoped ? decision : null,
      pregrasp: geometryFrame ? vector(grasp.pregrasp) : null,
      entry: geometryFrame ? vector(grasp.entry) : null,
      cut: geometryFrame ? vector(grasp.cut_pose) : null,
      grasp_axis: geometryFrame ? direction(grasp.axis) : null,
      tool_frames: toolFrames, trajectory: points};
  }
  // Geometry arithmetic is descriptive only. Reachability belongs to MoveIt.
  function analyzeScene(state = {}, trajectory = {}) {
    const scene = normalizeScene(state, trajectory), d = scene.decision || {};
    const scalar = v => typeof v === 'number' && Number.isFinite(v) ? v : null;
    const nonnegative = v => scalar(v) !== null && v >= 0 ? v : null;
    const ratio = v => nonnegative(v) !== null && v <= 1 ? v : null;
    const delta = (a, b) => a && b ? a.map((v, i) => v - b[i]) : null;
    const norm = v => v ? Math.hypot(...v) : null;
    const dot = (a, b) => a && b ? a.reduce((s, v, i) => s + v * b[i], 0) : null;
    const angle = (a, b) => a && b ? Math.acos(Math.max(-1, Math.min(1, dot(a,b)))) * 180 / Math.PI : null;
    const travel = delta(scene.cut, scene.entry), axial = dot(travel, scene.grasp_axis);
    const lateral = travel && axial !== null ? norm(travel.map((v,i) => v - axial * scene.grasp_axis[i])) : null;
    const reported = scalar(d.cut_travel_m), ref = scene.fused?.source || {};
    const scoped = state.job?.target_id === scene.target_id &&
      (!state.ui?.selected_scene_epoch || state.ui.selected_scene_epoch === state.perception?.targets?.scene_epoch);
    const job = scoped ? state.job : {}, completion = job?.completion || {};
    const arm = state.manipulation?.status || {};
    const text = [completion.reason, job?.why, scoped && arm.target_id === scene.target_id ? arm.message : ''].filter(Boolean).join(' · ');
    const planningFailure = /SKIPPED_UNREACHABLE|skipped_unreachable|no_ik|no_cartesian|MTC planning failed|精化几何可达性未通过/.test(text);
    const reached = Number.isInteger(completion.completion_level) && completion.completion_level >= 1;
    const level = completion.completion_level;
    const completionLabels = ['尚未到达预抓取','预抓取到达已记录','预抓取验证已记录',
      '套入到达已记录','刀指令受理已记录','切断确认已记录','撤退完成已记录','采摘确认已记录'];
    const reachability = {tone:planningFailure ? 'bad' : reached ? 'good' : 'unknown',
      label:planningFailure ? '规划 / 可达性未通过' : reached ? completionLabels[level] || '完成级别已记录' : '可达性尚未验证',
      detail:planningFailure ? completion.reason || (arm.target_id===scene.target_id ? arm.message : '') || text : reached ?
        `证据来自当前作业完成级别 ${level}。${level<3?'完整套入路径需另行验证。':level<7?'采摘确认尚无证据。':''}` : '几何许可不代表机械臂可达。请在单步调试中查询或只规划。'};
    const metrics = {
      standoff_m:norm(delta(scene.entry, scene.pregrasp)), entry_cut_m:norm(travel), axial_travel_m:axial,
      lateral_error_m:lateral, travel_error_m:axial !== null && reported !== null ? axial - reported : null,
      cut_travel_m:reported, entry_distance_m:norm(scene.entry), cut_distance_m:norm(scene.cut),
      entry_shift_m:norm(delta(scene.entry,scene.initial?.entry)),
      initial_fused_angle_deg:angle(scene.initial?.axis,scene.fused?.axis), tool_axis_angle_deg:scene.axisAngleDeg,
      diameter_m:scalar(d.diameter_m) ?? scene.fused?.diameter ?? null,
      d95_m:scalar(d.d95_m) > 0 ? d.d95_m : null,
      radial_margin_m:scalar(d.radial_margin_m), axial_margin_m:scalar(d.axial_margin_m),
      refit_rmse_m:nonnegative(ref.refit_rmse) ?? nonnegative(ref.rmse), inlier_ratio:ratio(ref.refit_inlier_ratio) ?? ratio(ref.inlier_ratio),
      captured_views:scene.fused ? scalar(state.reconstruction?.diagnostics?.captured_views) : null,
      axis_conflict_deg:scalar(ref.axis_conflict_deg), lateral_shift_m:scalar(ref.fusion_lateral_shift_m),
    };
    const points = [['预抓取',scene.pregrasp],['袋外入口',scene.entry],['袋底',scene.fused?.bottom],
      ['袋口 / 袋颈',scene.fused?.neck],['剪切参考',scene.cut],['重建中心',scene.fused?.center],
      ['实际 TCP',scene.tool_frames.tool_axis?.position]].filter(([,p])=>p).map(([name,position])=>({name,position}));
    return {scene,metrics,points,reachability,decision:scene.decision, flags:Array.isArray(ref.flags) ? ref.flags : [],
      consistency:scene.fused ? ref.fusion_consistency_available : null};
  }
  function unavailable(error) {
    return {available: false, error: String(error?.message || error), update() {}, setView() {}, resize() {}, dispose() {}};
  }
  function create(container) {
    let renderer; let controls; let observer; let scene; let labelLayer;
    const {THREE: T, OrbitControls} = root.PeachThree || {};
    if (!T || !OrbitControls) return unavailable('本地三维库未加载');
    try {
      renderer = new T.WebGLRenderer({antialias: true, alpha: true});
      renderer.setPixelRatio(Math.min(root.devicePixelRatio || 1, 2));
      renderer.setClearColor(0xf8fafb, 1);
      renderer.domElement.setAttribute('aria-label', 'base_link 坐标系三维几何，单位米，Z 向上');
      container.appendChild(renderer.domElement);
      scene = new T.Scene();
      const camera = new T.PerspectiveCamera(42, 1, 0.002, 100);
      camera.up.set(0, 0, 1); camera.position.set(0.8, -1, 0.8);
      controls = new OrbitControls(camera, renderer.domElement);
      controls.enableDamping = false; controls.autoRotate = false;
      controls.minDistance = 0.025; controls.maxDistance = 10;
      const grid = new T.GridHelper(2, 20, 0xc1d0d5, 0xe1e8eb);
      grid.rotation.x = Math.PI / 2; scene.add(grid);
      scene.add(new T.AxesHelper(0.16));
      const group = new T.Group(); scene.add(group);
      labelLayer = root.document.createElement('div');
      labelLayer.className = 'viz3d-labels'; container.appendChild(labelLayer);
      let labels = []; let disposed = false; let fitted = false; let signature = ''; let current = null;
      const point = p => new T.Vector3(...p);
      function release(object) {
        const geometries = new Set(); const materials = new Set();
        object.traverse(child => { if (child.geometry) geometries.add(child.geometry);
          for (const m of (Array.isArray(child.material) ? child.material : [child.material])) if (m) materials.add(m); });
        geometries.forEach(g => g.dispose()); materials.forEach(m => m.dispose());
      }
      function render() {
        if (disposed || root.document.hidden) return;
        renderer.render(scene, camera);
        const occupied = [];
        labels.forEach(({element, leader, position}) => {
          const p = position.clone().project(camera);
          element.hidden = p.z < -1 || p.z > 1 || Math.abs(p.x) > 1 || Math.abs(p.y) > 1;
          leader.hidden = element.hidden;
          if (element.hidden) return;
          const width = element.offsetWidth || element.textContent.length * 11 + 12;
          const height = element.offsetHeight || 22;
          const anchorX = (p.x+1)*container.clientWidth/2, anchorY = (1-p.y)*container.clientHeight/2;
          let left = anchorX+8, top = anchorY-18;
          for (let attempt=0;attempt<labels.length*4;attempt++) {
            const level = Math.floor(attempt/2);
            const offset = level===0 ? 0 : Math.ceil(level/2)*(level%2 ? 1 : -1)*(height+4);
            left = Math.max(0,Math.min(container.clientWidth-width,attempt%2 ? anchorX-width-8 : anchorX+8));
            top = Math.max(0,Math.min(container.clientHeight-height,anchorY-18+offset));
            if (!occupied.some(r=>left<r.right+4 && left+width>r.left-4 && top<r.bottom+3 && top+height>r.top-3)) break;
          }
          occupied.push({left,right:left+width,top,bottom:top+height});
          element.style.left = `${left}px`; element.style.top = `${top}px`;
          const dx = Math.max(left,Math.min(left+width,anchorX))-anchorX;
          const dy = Math.max(top,Math.min(top+height,anchorY))-anchorY;
          leader.style.left = `${anchorX}px`; leader.style.top = `${anchorY}px`;
          leader.style.width = `${Math.hypot(dx,dy)}px`;
          leader.style.transform = `rotate(${Math.atan2(dy,dx)}rad)`;
        });
      }
      function label(text, position, color) {
        if (!position) return;
        const element = root.document.createElement('span'); element.textContent = text;
        const leader = root.document.createElement('div'); leader.className = 'viz3d-leader';
        leader.style.borderColor = color; labelLayer.appendChild(leader);
        element.style.color = color; labelLayer.appendChild(element); labels.push({element, leader, position: point(position)});
      }
      function line(points, color, dashed = false) {
        if (points.length < 2 || points.some(p => !p)) return;
        const geometry = new T.BufferGeometry().setFromPoints(points.map(point));
        const material = dashed ? new T.LineDashedMaterial({color, dashSize: 0.009, gapSize: 0.006}) : new T.LineBasicMaterial({color});
        const object = new T.Line(geometry, material); if (dashed) object.computeLineDistances(); group.add(object);
      }
      function dot(position, color, name) {
        if (!position) return;
        const object = new T.Mesh(new T.SphereGeometry(0.004, 10, 8), new T.MeshBasicMaterial({color}));
        object.position.copy(point(position)); group.add(object); if (name) label(name, position, color);
      }
      function axis(position, normal, color, dashed) {
        if (position && normal) line([position, position.map((v, i) => v + normal[i] * 0.10)], color, dashed);
      }
      function ring(position, normal, diameter, color, dashed = false) {
        if (!position || !normal || !(diameter > 0)) return;
        const rotation = new T.Quaternion().setFromUnitVectors(new T.Vector3(0, 0, 1), point(normal));
        const points = Array.from({length: 49}, (_, i) => {
          const a = i / 48 * Math.PI * 2;
          return new T.Vector3(Math.cos(a) * diameter / 2, Math.sin(a) * diameter / 2, 0).applyQuaternion(rotation).add(point(position)).toArray();
        }); line(points, color, dashed);
      }
      function model(value, color, prefix) {
        if (!value) return;
        const diagnostic = value.diagnostic; const c = diagnostic ? COLORS.diagnostic : color;
        dot(value.bottom, c, `${prefix}袋底`); dot(value.neck, c, `${prefix}袋口`);
        dot(value.center, c, `${prefix}中心`);
        line([value.bottom, value.neck], c, diagnostic); axis(value.bottom || value.center || value.entry, value.axis, c, diagnostic);
        ring(value.neck, value.axis, value.diameter, c, diagnostic);
        if (diagnostic) label('错轴 / 拟合拒绝诊断', value.center || value.bottom || value.entry, c);
      }
      function fit(view = 'center') {
        const bounds = new T.Box3().setFromObject(group);
        if (bounds.isEmpty()) return;
        const center = bounds.getCenter(new T.Vector3());
        const radius = Math.max(bounds.getSize(new T.Vector3()).length() * 0.65, 0.10);
        let offset = camera.position.clone().sub(controls.target).normalize();
        if (view === 'top') offset.set(0, -0.001, 1).normalize();
        if (view === 'side') offset.set(0, -1, 0);
        if (view === 'reset') offset.set(0.8, -1, 0.8).normalize();
        controls.target.copy(center);
        camera.position.copy(center).addScaledVector(offset, radius / Math.sin(camera.fov * Math.PI / 360));
        controls.update(); fitted = true; render();
      }
      function update(state, trajectory) {
        if (disposed) return;
        current = normalizeScene(state, trajectory);
        // Only geometry affects drawing. Source ages and health labels belong to the HTML HUD.
        const next = JSON.stringify(current, (key, value) => ['source', 'age_s', 'stamp', 'stamp_sec'].includes(key) ? undefined : value);
        if (signature === next) return; signature = next;
        release(group); group.clear(); labels = []; labelLayer.replaceChildren();
        model(current.initial, COLORS.initial, '初值'); model(current.fused, COLORS.fused, '融合');
        dot(current.pregrasp, COLORS.fused, 'P · 预抓取'); dot(current.entry, COLORS.fused, 'E · 袋外入口');
        line([current.pregrasp, current.entry], COLORS.fused, true);
        dot(current.cut, COLORS.fused, 'C · 剪切参考');
        line([current.entry, current.cut], COLORS.fused, true);
        axis(current.entry, current.grasp_axis, COLORS.fused, false);
        ring(current.cut, current.grasp_axis, current.fused?.diameter, COLORS.fused, true);
        line(current.trajectory, COLORS.tool);
        const end = current.trajectory.at(-1); dot(end, COLORS.tool, '轨迹末端');
        for (const [name, frame] of Object.entries(current.tool_frames)) {
          dot(frame.position, COLORS.tool, name); axis(frame.position, frame.axis, COLORS.tool, false);
          if (name !== 'tool_axis') ring(frame.position, frame.axis, 0.035, COLORS.tool);
        }
        if (!fitted) fit('reset'); render();
      }
      function resize() {
        if (disposed) return;
        const width = Math.max(container.clientWidth, 1); const height = Math.max(container.clientHeight, 1);
        renderer.setSize(width, height, false); camera.aspect = width / height; camera.updateProjectionMatrix(); render();
      }
      controls.addEventListener('change', render);
      const visibility = () => { if (!root.document.hidden) render(); };
      root.document.addEventListener('visibilitychange', visibility);
      const contextLost = event => { event.preventDefault(); api.available = false; api.error = 'WebGL 上下文已丢失';
        container.dispatchEvent(new root.CustomEvent('peach-viz3d-unavailable', {detail: api.error})); };
      renderer.domElement.addEventListener('webglcontextlost', contextLost);
      if (root.ResizeObserver) { observer = new root.ResizeObserver(resize); observer.observe(container); }
      const api = {available: true, error: '', update, resize, setView(view) { if (!disposed && ['top', 'side', 'reset', 'center'].includes(view)) fit(view); },
        dispose() { if (disposed) return; disposed = true; observer?.disconnect();
          controls.removeEventListener('change', render); controls.dispose();
          root.document.removeEventListener('visibilitychange', visibility);
          renderer.domElement.removeEventListener('webglcontextlost', contextLost);
          release(scene); renderer.dispose(); renderer.forceContextLoss(); renderer.domElement.remove(); labelLayer.remove(); }};
      resize(); return api;
    } catch (error) {
      observer?.disconnect(); controls?.dispose();
      if (scene) scene.traverse(o => { o.geometry?.dispose(); if (o.material?.dispose) o.material.dispose(); });
      renderer?.dispose(); renderer?.domElement?.remove(); labelLayer?.remove(); return unavailable(error);
    }
  }
  root.PeachViz3D = Object.freeze({create, normalizeScene, analyzeScene, COLORS});
})(typeof window === 'undefined' ? globalThis : window);
