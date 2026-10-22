/* Classic browser module. Transport and backend authority remain in app.js. */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.PeachWorkbench = factory();
})(typeof window !== 'undefined' ? window : globalThis, function () {
  'use strict';
  const steps = [
    ['拍照与建场景', '让相机回到拍照位，再开启新的场景。'],
    ['扫场与选目标', '扫场完成后，从本场真实目标中手动选择。'],
    ['重建与核对', '核对积分证据、模型质量和分级许可。'],
    ['只规划', '查看路径与规划结果；本步不会执行轨迹。'],
    ['预抓取', '移动到预抓取位置并停住。'],
    ['完整周期', '执行完整套入与工具周期；以臂侧命令门为准。'],
  ];
  const escape = value => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  function requestId() {
    const bytes = new Uint8Array(4);
    if (typeof crypto !== 'undefined' && crypto.getRandomValues) crypto.getRandomValues(bytes);
    else for (let i = 0; i < 4; i++) bytes[i] = Math.floor(Math.random() * 256);
    return `web_${Date.now()}_${Array.from(bytes, b => b.toString(16).padStart(2, '0')).join('')}`;
  }
  function targets(state) {
    return (state?.perception?.targets?.observations || []).filter(t => t && typeof t.target_id === 'string' && t.target_id);
  }
  function epoch(state) { return state?.perception?.targets?.scene_epoch ?? state?.perception?.harvest?.scene_epoch ?? null; }
  function selectionFor(state, selection) {
    const current = epoch(state);
    return {scene_epoch: current, target_id: selection?.scene_epoch === current && targets(state).some(t => t.target_id === selection?.target_id) ? selection.target_id : ''};
  }
  function contextReason(state, context) {
    if (epoch(state) !== context.scene_epoch) return '场景 epoch 已变化，请重新选择目标';
    if (!context.target_id || !targets(state).some(t => t.target_id === context.target_id)) return '目标已失效，请重新选择';
    return '';
  }
  function motion(action, payload) {
    if (['survey_action','photo_pose_service','arm_service','run_harvest_action'].includes(action)) return true;
    if (action === 'execute_action') return String(payload.mode || 'PREVIEW').toUpperCase() !== 'PREVIEW';
    return action === 'control_service' && ['RESUME','EXIT_MAINTENANCE'].includes(payload.command);
  }
  function blockReason(action, payload, state) {
    if (state?.debug?.enabled !== true) return '后端调试入口未开启';
    const cancellation = action === 'cancel' || (action === 'control_service' && payload.command === 'CANCEL_NOW');
    if (state?.ui?.fresh === false && !cancellation) return '状态快照已陈旧，请等待恢复';
    if (motion(action, payload) && state?.debug?.motion_enabled !== true) return '后端动臂入口未放行';
    if (action === 'photo_pose_service') {
      if (state?.manipulation?.status?.contact_recovery_required || state?.task_executor?.state?.recovery_required) return '待人工恢复确认';
      if (state?.manipulation?.status?.running) return '臂周期正在运行';
    }
    return '';
  }
  function operationLock() {
    const keys = new Set();
    return {has: key => keys.has(key), async run(key, fn) {
      if (keys.has(key)) return null;
      keys.add(key);
      try { return await fn(); } finally { keys.delete(key); }
    }};
  }
  function button(label, action, payload = {}, confirm = false) {
    return `<button type="button" data-wb-action="${action}" data-wb-payload="${escape(JSON.stringify(payload))}"${confirm ? ' data-wb-confirm="1"' : ''}><span>${label}</span><small>${action}</small></button>`;
  }
  const cancel = (label, target) => button(label, 'cancel', {target});
  function reachabilityHTML(result, request) {
    const response = result?.body?.result || result?.body?.response || result?.body || {};
    if (!Array.isArray(response.reachable)) return '';
    const names = {no_ik:'预抓取无逆解',sleeve_no_ik:'套入终点无逆解',sleeve_no_cartesian:'套入笛卡尔路径未通过',
      invalid_frame:'坐标系无效',moveit_unavailable:'MoveIt 不可用'};
    return `<div class="wb-reach-results"><p>${request?.require_sleeve?'预抓取 + 套入路径':'预抓取'} · MoveIt 查询结果</p>${response.reachable.map((ok,i)=>{
      const code = response.error_codes?.[i] || '';
      return `<div class="wb-reach-row ${ok===true?'good':'bad'}"><b>位姿 ${i+1}</b><strong>${ok===true?'通过':names[code]||'未通过'}</strong><code>${escape(code)}</code></div>`;
    }).join('')}<small>查询通过只表示本次求解通过；执行轨迹仍由臂侧规划与授权。</small></div>`;
  }
  function stepActions(step) {
    if (step === 0) return button('回到拍照位','photo_pose_service',{},true) + button('开始新场景','begin_scene_service');
    if (step === 1) return button('扫描场景','survey_action',{},true) + cancel('取消扫场','survey_action');
    if (step === 2) return button('重建当前目标','build_action',{},true) + cancel('取消重建','build_action') + button('查询重建','recon_query_service') + button('完成积分','recon_finalize_service') + button('保存本次重建','recon_save_session_service') + button('重置重建','recon_reset_service',{},true);
    if (step === 3) return button('规划当前目标','execute_action',{mode:'PREVIEW'},true) + button('预览接近段','preview_approach_service') + button('预览完整路径','preview_full_service') + cancel('取消规划','execute_action');
    return button(step === 4 ? '执行到预抓取' : '执行完整周期','execute_action',{mode:step === 4 ? 'PREGRASP_ONLY':'FULL'},true) + cancel('取消执行','execute_action') + button('取消技能周期','skill_cancel_service');
  }
  function advancedHTML() {
    return `<details class="wb-advanced"><summary>高级操作 · 生命周期、批次与显式 IK</summary>
      <div class="wb-advanced-grid">
      <fieldset><legend>生命周期</legend><label>管理指令<select data-wb-field="lifecycle">${[['STARTUP','启动'],['PAUSE','暂停'],['RESUME','恢复'],['RESET','重置'],['SHUTDOWN','关闭']].map(([s,label])=>`<option value="${s}">${label} · ${s}</option>`).join('')}</select></label>${button('发送生命周期指令','manage_nodes_service',{},true)}${button('运行启动自检','selfcheck')}</fieldset>
      <fieldset><legend>人工恢复与武装</legend><p>完成现场核对后，手动发送恢复确认。</p>${button('确认技能恢复','ack_recovery_service',{},true)}${button('确认批次恢复','control_service',{command:'ACKNOWLEDGE_RECOVERY'},true)}${button('武装本周期','arm_service',{data:true},true)}${button('解除武装','arm_service',{data:false})}</fieldset>
      <fieldset><legend>批次控制</legend>${button('暂停批次','control_service',{command:'PAUSE'})}${button('恢复批次','control_service',{command:'RESUME'},true)}${button('跳过当前目标','control_service',{command:'SKIP_TARGET'},true)}${button('立即取消批次','control_service',{command:'CANCEL_NOW'})}${cancel('取消批次动作','run_harvest_action')}</fieldset>
      <fieldset><legend>启动批次</legend><label>意图<select data-wb-field="intent"><option value="PICK_SELECTED">采摘指定目标</option><option value="SURVEY_ONLY">仅扫场</option><option value="PICK_ALL">逐颗采摘</option></select></label><label>指定目标（一轮一颗，逗号处只留一个）<input data-wb-field="run-targets" autocomplete="off" placeholder="例如 target_0"></label>${button('启动采摘批次','run_harvest_action',{},true)}</fieldset>
      <fieldset class="wb-ik"><legend>显式位姿可达性查询</legend><p>输入 tcp_poses（米、单位四元数、frame_id）。不会从袋轴推造姿态。</p><label>请求 JSON<textarea data-wb-field="poses" rows="6" spellcheck="false" placeholder='{"tcp_poses":[{"frame_id":"base_link","position":{"x":0.4,"y":0,"z":0.3},"orientation":{"x":0,"y":0,"z":0,"w":1}}],"timeout_s":2,"require_sleeve":false}'></textarea></label>${button('查询可达性','check_reachability_service')}</fieldset>
      </div></details>`;
  }
  function create(container, options) {
    let state = options.getState() || {}, selection = selectionFor(state, {}), step = 0, disposed = false;
    const locks = operationLock(), results = [], listeners = [];
    container.innerHTML = `<div class="wb-shell"><header class="wb-head"><div><p class="kicker">PEACH · 实验工作流</p><h2>分步调试实验台</h2></div><button type="button" data-wb-new>新实验</button></header>
      <div class="wb-context"><label>实验标识<input data-wb-field="request" value="${requestId()}" autocomplete="off"></label><label>场景<input data-wb-field="scene" value="lab" autocomplete="off"></label><label>本场目标<select data-wb-field="target"><option value="">请选择本场目标</option></select></label><div class="wb-epoch">场景 epoch<strong data-wb-epoch>—</strong></div></div>
      <div class="wb-columns"><nav class="wb-steps" aria-label="实验步骤">${steps.map((s,i)=>`<button type="button" data-wb-step="${i}" aria-pressed="${i===0}"><b>${String(i+1).padStart(2,'0')}</b><span>${s[0]}</span></button>`).join('')}<p>切换步骤仅切换视图。<br>每次操作由你显式发起。</p></nav>
      <section class="wb-center"><div class="wb-step-heading"><p class="kicker" data-wb-level>步骤 01 / 06</p><h3 data-wb-title></h3><p data-wb-desc></p></div><div id="debug-visualization-host" class="wb-visualization-host"></div><details class="wb-evidence" open><summary>本场重建证据</summary><div data-wb-evidence></div></details></section>
      <aside class="wb-inspector"><h3>前置条件</h3><div data-wb-conditions></div><h3>本步操作</h3><label class="wb-checkbox"><input type="checkbox" data-wb-field="skip" checked>跳过观察位移 · skip_observation</label><div class="wb-actions" data-wb-operations></div><p data-wb-notice role="status" aria-live="polite">请选择步骤后显式发起操作。</p><h3>最近结果</h3><div data-wb-latest class="wb-latest">尚无请求</div></aside></div>${advancedHTML()}<details class="wb-json"><summary>请求 / 响应 JSON · 本次实验</summary><div data-wb-results></div></details></div>`;
    const find = name => container.querySelector(`[data-wb-${name}]`);
    const field = name => container.querySelector(`[data-wb-field="${name}"]`);
    const listen = (name, fn) => {container.addEventListener(name, fn); listeners.push([name, fn]);};
    function notice(text) { find('notice').textContent = text; if (options.log) options.log(text); }
    function getPayload(action, inline) {
      const base = {request_id:field('request').value.trim(),scene_key:field('scene').value.trim() || 'lab'};
      if (!base.request_id) throw new Error('实验标识不能为空，请新建实验');
      if (['begin_scene_service','survey_action'].includes(action)) return base;
      if (['build_action','execute_action'].includes(action)) {
        const context = {...selection}; const why = contextReason(options.getState(), context);
        if (why) throw new Error(why);
        return action === 'build_action' ? {request_id:base.request_id,target_id:context.target_id,scene_epoch:context.scene_epoch} : {request_id:base.request_id,target_id:context.target_id,...inline,skip_observation:field('skip').checked};
      }
      if (action === 'control_service') return {...inline,expected_state_seq:['PAUSE','CANCEL_NOW'].includes(inline.command) ? 0 : Number(options.getState()?.task_executor?.state?.state_seq) || 0,reason:'web 分步调试'};
      if (action === 'manage_nodes_service') return {command:field('lifecycle').value};
      if (action === 'run_harvest_action') {
        const ids = field('run-targets').value.split(',').map(s=>s.trim()).filter(Boolean);
        if (ids.some(id=>!targets(options.getState()).some(t=>t.target_id===id))) throw new Error('指定目标不属于本场快照');
        if (field('intent').value === 'PICK_SELECTED' && !ids.length) throw new Error('指定采摘需填写本场目标');
        return {...base,intent:field('intent').value,selection_mode:ids.length?'MANUAL':'AUTO',target_ids:ids};
      }
      if (action === 'check_reachability_service') {
        const payload = JSON.parse(field('poses').value);
        if (!Array.isArray(payload.tcp_poses) || !payload.tcp_poses.length) throw new Error('请显式填写 tcp_poses');
        payload.tcp_poses.forEach(p => {
          if (!p.frame_id || !['x','y','z'].every(k=>Number.isFinite(p.position?.[k])) || !['x','y','z','w'].every(k=>Number.isFinite(p.orientation?.[k]))) throw new Error('位姿需 frame_id、完整位置和四元数');
          const norm = Math.hypot(...['x','y','z','w'].map(k=>p.orientation[k]));
          if (Math.abs(norm-1)>0.01) throw new Error('姿态四元数必须归一化');
        });
        return payload;
      }
      return inline;
    }
    function refreshButtons() {
      container.querySelectorAll('[data-wb-action]').forEach(el => {
        const action = el.dataset.wbAction, inline = JSON.parse(el.dataset.wbPayload || '{}');
        const key = action === 'cancel' ? `cancel:${inline.target}` : action === 'control_service' && inline.command === 'CANCEL_NOW' ? 'control_service:CANCEL_NOW' : action;
        const reason = blockReason(action,inline,state) || (['build_action','execute_action'].includes(action) ? contextReason(state,selection) : '');
        const busy = locks.has(key) || (state.ui?.pending || []).includes(key);
        el.disabled = Boolean(reason) || busy;
        el.title = busy ? '请求正在等待结果；取消入口独立可用' : reason;
        el.setAttribute('aria-busy',String(busy));
      });
    }
    function showStep() {
      find('title').textContent = steps[step][0]; find('desc').textContent = steps[step][1];
      find('level').textContent = `步骤 ${String(step+1).padStart(2,'0')} / 06`;
      container.querySelectorAll('[data-wb-step]').forEach(el=>el.setAttribute('aria-pressed',String(Number(el.dataset.wbStep)===step)));
      find('operations').innerHTML = stepActions(step); refreshButtons();
    }
    function renderEvidence() {
      const recon = state.reconstruction || {}, diag = recon.diagnostics || {}, decision = recon.grasp_decision || {};
      const value = v => v === undefined || v === null ? '未报告' : typeof v === 'object' ? JSON.stringify(v) : String(v);
      const parameters = Object.fromEntries(Object.entries(state.params || {}).filter(([node]) => /reconstruction/.test(node)).map(([node, entries]) => [node, Object.fromEntries(Object.entries(entries || {}).filter(([key]) => /exposure|static|stationary|stability|velocity|capture|settle|motion|integration/.test(key)))]));
      const summaries = [
        ['状态 / 目标', `${value(diag.state)} / ${value(diag.target_id)}`],
        ['采集 / 拒帧', `${value(diag.captured_views)} / ${value(diag.rejected_views)}`],
        ['跳帧计数', value(diag.skip_counts)],
        ['TF 失败 / 延迟', `${value(diag.tf_failures)} / ${value(diag.tf_latency_ms)} ms`],
        ['独立机位覆盖', value(diag.view_coverage)],
        ['重拟合质量', value(diag.refined || diag.refit || recon.refit_quality)],
        ['融合 / 配准', value(diag.tsdf || diag.fusion || diag.registration)],
        ['分级许可（诊断）', `${value(decision.level ?? decision.decision_level ?? decision.clearance_level)} · allowed=${value(decision.allowed)}`],
      ];
      // Raw sections preserve all reported evidence, including new fusion/refit fields.
      find('evidence').innerHTML = `<p>静止积分与曝光 · 独立覆盖 · 重拟合与融合质量</p><dl>${summaries.map(([name,v])=>`<dt>${name}</dt><dd>${escape(v)}</dd>`).join('')}</dl>${[['曝光与静止参数镜像',parameters],['积分 / 跳帧 / 独立覆盖',diag],['模型与重拟合 / 融合（完整证据）',recon],['许可与预算（完整证据）',decision]].map(([title,body])=>`<details><summary>${title}</summary><pre>${escape(JSON.stringify(body,null,2))}</pre></details>`).join('')}`;
    }
    function renderConditions() {
      const arm = state.manipulation?.status || {}, task = state.task_executor?.state || {}, clearance = state.reconstruction?.grasp_decision || {};
      const yn = v=>v===true?'已开启':v===false?'未开启':'未报告';
      const rows = [['调试入口 · 必须门',yn(state.debug?.enabled)],['动臂入口 · 运动必须门',yn(state.debug?.motion_enabled)],['运行执行使能 · 臂侧门',yn(arm.execution_enabled ?? task.execution_enabled)],['抓取使能 · 臂侧门',yn(arm.grasp_enabled ?? task.grasp_enabled)],['工具使能 · 臂侧门',yn(arm.tool_enabled ?? task.tool_enabled)],['武装状态 · 臂侧门',yn(arm.armed ?? arm.execution_armed ?? arm.cycle_armed)],['恢复 / 运行 · 拍照门',`${yn(arm.contact_recovery_required || task.recovery_required)} / ${yn(arm.running)}`],['分级能力 · 诊断',['geometry','pregrasp','sleeve','cut'].map(key=>({0:'有效',1:'无效',2:'未知'})[clearance[key+'_capability']] || '未报告').join(' / ')],['失败门 · 诊断',String(clearance.failed_gate ?? '未报告')],['失败码 · 诊断',String(clearance.failure_code ?? arm.failure_code ?? '未报告')],['有效期 · 诊断',String(clearance.valid_until ?? '未报告')]];
      find('conditions').innerHTML = rows.map(([name,v])=>`<div class="wb-condition"><span>${name}</span><strong>${escape(v)}</strong></div>`).join('') + '<p class="wb-condition-note">展示不授权。分级许可、预算与有效期由后端和臂侧校验。</p>';
    }
    function update(next) {
      if (disposed) return; state = next || {}; const old = selection; selection = selectionFor(state,selection);
      if (old.target_id && !selection.target_id) notice('场景或目标已变化，请重新选择；已有结果证据保留。');
      const optionsHTML = '<option value="">请选择本场目标</option>' + targets(state).map(t=>`<option value="${escape(t.target_id)}">${escape(t.target_id)}${t.confirmed?' · 已确认':''}</option>`).join('');
      if (field('target').innerHTML !== optionsHTML) field('target').innerHTML = optionsHTML;
      field('target').value = selection.target_id; find('epoch').textContent = Number.isInteger(selection.scene_epoch) && selection.scene_epoch > 0 ? String(selection.scene_epoch) : '未报告';
      renderConditions(); renderEvidence(); refreshButtons();
      if (old.target_id !== selection.target_id || old.scene_epoch !== selection.scene_epoch) options.onContextChange?.();
    }
    listen('change', event => {
      if (event.target === field('target')) {
        selection = {scene_epoch:epoch(options.getState()),target_id:event.target.value};
        refreshButtons(); options.onContextChange?.();
      }
    });
    listen('click', async event => {
      const el = event.target.closest('button'); if (!el || !container.contains(el) || el.disabled) return;
      if (el.hasAttribute('data-wb-step')) {step=Number(el.dataset.wbStep);showStep();return;}
      if (el.hasAttribute('data-wb-new')) {field('request').value=requestId();notice('新实验标识已生成；历史结果仍可查看。');options.onContextChange?.();return;}
      if (!el.dataset.wbAction) return;
      const action = el.dataset.wbAction;
      try {
        const payload = getPayload(action,JSON.parse(el.dataset.wbPayload || '{}'));
        const captured = {...selection};
        const selected = ['build_action','execute_action'].includes(action);
        const batch = action === 'run_harvest_action';
        const guard = () => {
          const now = options.getState() || {};
          const reason = blockReason(action,payload,now); if (reason) return reason;
          if (selected) return contextReason(now,captured) || (selection.target_id !== captured.target_id || selection.scene_epoch !== captured.scene_epoch ? '所选目标已变化，请重新发起' : '');
          if (batch && (epoch(now) !== captured.scene_epoch || payload.target_ids.some(id=>!targets(now).some(t=>t.target_id===id)))) return '批次场景或目标已变化，请重新发起';
          return '';
        };
        const reason = guard(); if (reason) throw new Error(reason);
        const key = action === 'cancel' ? `cancel:${payload.target}` : action === 'control_service' && payload.command === 'CANCEL_NOW' ? 'control_service:CANCEL_NOW' : action;
        await locks.run(key,async () => {
          refreshButtons(); notice('等待请求结果；取消入口仍独立可用。');
          const confirmText = el.dataset.wbConfirm ? `确认${el.querySelector('span').textContent}？\n实验 ${field('request').value}\n目标 ${captured.target_id || '—'} · epoch ${captured.scene_epoch ?? '—'}\n${JSON.stringify(payload,null,2)}` : null;
          const result = await options.submit(action,payload,confirmText,guard);
          if (disposed || result === null || result === undefined) return;
          const entry = {time:new Date().toLocaleTimeString('zh-CN',{hour12:false}),action,request:payload,response:result}; results.unshift(entry); if(results.length>30) results.pop();
          const detail = result.body?.message || result.body?.result?.message || '查看请求 / 响应 JSON';
          find('latest').innerHTML = `<b>${escape(action)}</b><p>${escape(result.ok === true ? '请求成功' : result.status === 202 ? '仍在执行 · 可取消' : '未确认成功')}</p><p>${escape(detail)}</p>${action==='check_reachability_service'?reachabilityHTML(result,payload):''}`;
          find('results').innerHTML = results.map((r,i)=>`<details ${i?'':'open'}><summary>${escape(r.time)} · ${escape(r.action)}</summary><pre>${escape(JSON.stringify(r,null,2))}</pre></details>`).join('');
          notice(`${entry.time} · ${detail}`);
        });
      } catch(error) { if (!disposed) notice(`请求未发送：${error.message}`); }
      finally {if (!disposed) refreshButtons();}
    });
    showStep(); update(state);
    return {update,context: () => ({...selection}),dispose() {disposed=true;listeners.forEach(([name,fn])=>container.removeEventListener(name,fn));}};
  }
  return {create,selectionFor,contextReason,blockReason,operationLock,requestId,reachabilityHTML};
});
