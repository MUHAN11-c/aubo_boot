/* Project contracts are data; all ROS operations use the public audit/SDK APIs. */
'use strict';
let projectRunBusy=false, projectAbort, projectReport, selectedProject, projectBase, projectPendingRun, projectHasMutations=false;
const projectRows=new Map();
const projectStatusLabels={PASS:'通过',FAIL:'失败',CANCELLED:'已取消',NOT_RUN:'未执行'};
function projectControls() {
  const busy=projectRunBusy||checkBusy;
  $('project-select').disabled=busy;
  $('project-select-read').disabled=busy;
  $('project-select-none').disabled=busy;
  $('project-run').disabled=busy||!client?.connected||!observationConnection()?.connected||!selectedProject||!projectSelected().length||!!goal||!!serviceWait||busyCommands.has('project-run')||busyCommands.has('connect');
  $('project-stop').disabled=!projectRunBusy||projectAbort?.signal.aborted;
  $('project-export').disabled=!projectReport;
  $('project-run-bottom').disabled=$('project-run').disabled;
  $('project-stop-bottom').disabled=$('project-stop').disabled;
  for(const row of projectRows.values())row.input.disabled=busy;
  // The connection must survive action cleanup / parameter restoration.
  if(projectRunBusy){$('connect').disabled=true;$('disconnect').disabled=true;}
}
function projectSelected(){return [...projectRows].filter(([,row])=>row.input.checked).map(([id])=>id);}
function projectSelectionLabel(){
  const selected=projectSelected();const interactions=selectedProject?.cases.filter(c=>selected.includes(c.id)&&c.interactive).length??0;
  $('project-selection-state').textContent=`已选 ${selected.length} / ${selectedProject?.cases.length??0} 项 · 只读 ${selected.length-interactions} · 交互 ${interactions}`;
  $('project-bottom-state').textContent=$('project-selection-state').textContent;
  projectControls();
}
function projectResult(result) {
  const row=projectRows.get(result.id);if(!row)return;
  row.status.className=`audit-status ${result.status.toLowerCase()}`;
  row.status.textContent=projectStatusLabels[result.status]||result.status;
  row.duration.textContent=result.startedAt?`${(result.elapsedMs/1000).toFixed(2)} s`:'—';
  row.evidence.textContent=formatPreview({id:result.id,status:result.status,elapsedMs:result.elapsedMs,...(result.error?{error:result.error}:{}),evidence:result.evidence});
  row.details.open=result.status==='FAIL';
}
function projectRequest(value,runId=`manual-${Date.now().toString(36)}`){
  if(value==='$runId')return runId;
  if(Array.isArray(value))return value.map(v=>projectRequest(v,runId));
  if(value&&typeof value==='object')return Object.fromEntries(Object.entries(value).map(([k,v])=>[k,projectRequest(v,runId)]));
  return value;
}
function projectFillPanel(spec) {
  if(spec.topic){
    const t=spec.topic;$('topic-name').value=t.name;$('topic-type').value=t.type;
    fillQosSelect('topic-qos',t.qos);
    $('topic-compression').value=t.compression??'none';
    $('topic-throttle').value=t.throttleMs??0;
    if(spec.kind==='image'){applyImageConfig({topic:t.name,type:t.type,depthScale:t.depthScale,subscription:{qos:t.qos??'sensor',compression:t.compression??'none',throttleMs:t.throttleMs??0}});location.hash='media';}
    else location.hash=spec.kind==='cloud'?'scene':'topics';
  } else if(spec.kind==='parameters'||spec.kind==='parameterRoundtrip'){
    $('parameter-node').value=spec.node??spec.parameter.node;$('parameter-names').value=spec.names?.join(',')??spec.parameter.name;location.hash='parameters';
  } else if(spec.service){$('service-name').value=spec.service.name;$('service-type').value=spec.service.type;$('service-json').value=json(projectRequest(spec.service.request??{}));location.hash='services';}
  else if(spec.action){$('action-name').value=spec.action.name;$('action-type').value=spec.action.type;$('action-json').value=json(projectRequest(spec.action.goal));location.hash='actions';}
  else if(spec.kind==='lifecycle'){$('lifecycle-node').value=spec.node;location.hash='lifecycle';}
  else if(spec.kind==='interface'){
    $('interface-kind').value=spec.interfaceKind;$('interface-name').value=spec.endpoint;$('interface-type').value=spec.interfaceType;
    $('interface-result').closest('details').open=true;location.hash='overview';
  } else if(spec.kind==='tf')location.hash='scene';
  else {$('graph-result').closest('details').open=true;location.hash='overview';}
}
function projectRenderCases() {
  const fragment=document.createDocumentFragment();projectRows.clear();let group;
  for(const spec of selectedProject.cases){
    if(group!==(spec.group??'用例')){group=spec.group??'用例';const h=document.createElement('h3');h.className='case-group';h.textContent=group;fragment.append(h);}
    const article=document.createElement('article');article.className='audit-case';article.dataset.caseId=spec.id;
    const line=document.createElement('div');line.className='audit-case-line';
    const label=document.createElement('label');label.className='audit-case-label';
    const input=document.createElement('input');input.type='checkbox';input.checked=!spec.interactive&&spec.defaultSelected!==false;input.dataset.auditCase=spec.id;
    input.addEventListener('change',projectSelectionLabel);
    const title=document.createElement('span');title.textContent=spec.label;label.append(input,title);
    const badge=document.createElement('span');badge.className=spec.interactive?'case-risk interactive':'case-risk';badge.textContent=spec.interactive?(spec.mockOnly?'Mock 交互':'状态交互'):'只读';
    const status=document.createElement('span');status.className='audit-status not_run';status.textContent='未执行';
    const duration=document.createElement('span');duration.className='audit-duration';duration.textContent='—';
    const open=document.createElement('button');open.textContent='填入面板';open.className='case-open';open.addEventListener('click',()=>projectFillPanel(spec));
    line.append(label,badge,status,duration,open);
    const details=document.createElement('details');details.className='case-details';const summary=document.createElement('summary');summary.textContent=spec.description||`${spec.kind} · ${spec.topic?.name??spec.node??spec.service?.name??spec.action?.name??spec.endpoint??spec.id}`;
    const evidence=document.createElement('pre');evidence.textContent='尚无本轮执行证据';details.append(summary,evidence);article.append(line,details);fragment.append(article);
    projectRows.set(spec.id,{input,status,duration,evidence,details});
  }
  $('project-cases').replaceChildren(fragment);projectSelectionLabel();
}
function projectApply(id) {
  if(projectRunBusy)return;
  const project=projectBase.projects.find(p=>p.id===id);if(!project)throw Error(`没有项目配置：${id}`);
  closeSubscriptions();if(imageUrl)URL.revokeObjectURL(imageUrl);imageUrl=undefined;$('image-preview').removeAttribute('src');
  selectedProject=project;projectReport=undefined;
  runtimeConfig={...projectBase,...project.view};
  workbenchView?.setProfile(runtimeConfig);
  $('project-title').textContent=project.title;$('workspace-title').textContent=project.title;
  $('project-description').textContent=project.description??'';
  $('metric-check-kind').textContent='项目验收';$('metric-checks').textContent='—';
  $('project-report-json').textContent='尚未运行';
  $('project-progress').textContent='尚未运行';$('project-analysis').textContent='只读项默认选中。交互项需单独勾选；报告区分接口、数据与交互证据，不把未执行项当作通过。';
  for(const id of ['audit-pass','audit-fail','audit-not-run','audit-time'])$(id).textContent='—';
  $('project-cases').dataset.project=project.id;delete $('project-cases').dataset.complete;
  const view=runtimeConfig;
  $('scene-config').value=json(view.scene);$('scene-fixed').value=view.scene.fixedFrame;
  applyImageConfig(view.image);$('plot-topic').value=view.plot.topic;$('plot-type').value=view.plot.type;$('plot-fields').value=view.plot.fields.join(',');
  const topicTool=project.tools?.topic??project.cases.find(c=>c.topic)?.topic;
  $('topic-name').value=topicTool?.name??'';$('topic-type').value=topicTool?.type??'';
  fillQosSelect('topic-qos',topicTool?.qos);
  $('topic-compression').value=topicTool?.compression??'none';
  $('topic-throttle').value=topicTool?.throttleMs??0;
  $('interface-kind').value='topic';$('interface-name').value=topicTool?.name??'';$('interface-type').value=topicTool?.type??'';
  $('publish-name').value=project.tools?.publish?.name??'';$('publish-json').value=json(project.tools?.publish?.message??{});
  const firstService=project.cases.find(c=>c.service);
  $('service-name').value=firstService?.service.name??'';$('service-type').value=firstService?.service.type??'';$('service-json').value=json(projectRequest(firstService?.service.request??{}));
  const firstAction=project.cases.find(c=>c.action);
  $('action-name').value=firstAction?.action.name??'';$('action-type').value=firstAction?.action.type??'';$('action-json').value=json(projectRequest(firstAction?.action.goal??{}));
  const firstParameters=project.cases.find(c=>c.kind==='parameters');
  $('parameter-node').value=firstParameters?.node??'';$('parameter-names').value=firstParameters?.names.join(',')??'';$('parameter-json').value='[]';
  const firstLifecycle=project.cases.find(c=>c.kind==='lifecycle');$('lifecycle-node').value=firstLifecycle?.node??'';
  projectRenderCases();onlineControls();
  if(client?.connected)log('project','CONFIG',{id:project.id,connection:client.options.url});
}
function projectSummarize(report){
  // Keep the DOM preview bounded; projectReport remains complete for export.
  $('project-report-json').textContent=formatPreview(report);
  $('metric-check-kind').textContent='项目用例';$('metric-checks').textContent=`${report.totals.PASS}/${report.selectedIds.length}`;
  $('audit-pass').textContent=report.totals.PASS;
  $('audit-fail').textContent=`${report.totals.FAIL} / ${report.totals.CANCELLED}`;
  $('audit-not-run').textContent=report.totals.NOT_RUN;
  $('audit-time').textContent=`${(report.elapsedMs/1000).toFixed(1)}s`;
  const read=report.results.filter(r=>!r.interactive&&r.status==='PASS').length;
  const interactions=report.results.filter(r=>r.interactive&&r.status==='PASS').length;
  const issues=report.results.filter(r=>r.status==='FAIL'||r.status==='CANCELLED').map(r=>`${r.label}：${r.error??r.status}`);
  const notRun=report.results.filter(r=>r.status==='NOT_RUN').map(r=>r.label);
  const degraded=report.results.some(r=>r.status==='PASS'&&r.kind==='action'&&r.evidence?.result?.degraded===true);
  $('project-analysis').textContent=`本轮项目：${report.title}。连接：${report.connectionMode==='separate'?'分离控制与观测':'共用连接'}。只读通过 ${read} 项，交互通过 ${interactions} 项。${issues.length?'需要处理：'+issues.join('；')+'。':'所选检查已完成。'}${degraded?'Action 返回 degraded=true，场景质量不能据此判通过。':''}${notRun.length?'未执行：'+notRun.join('、')+'。':''}结果仅覆盖所选用例，未验证的业务流程仍未验收。`;
  const selected=report.results.filter(r=>report.selectedIds.includes(r.id));
  const allPassed=selected.length>0&&selected.every(r=>r.status==='PASS');
  const state=report.totals.FAIL?'failed':report.totals.CANCELLED?'cancelled':allPassed?'passed':'stopped';
  $('project-progress').textContent=`${state==='stopped'?'未完成 / 已停止':'本轮结束'} · ${report.runId}`;
  $('project-cases').dataset.complete=state;
  if(state==='stopped')$('project-analysis').textContent='本轮未完成所选用例；未执行项保留为 NOT_RUN。'+$('project-analysis').textContent;
}
command('project-run','project-analysis',async()=>{
  if(projectRunBusy||checkBusy||goal||serviceWait)throw Error('请先完成当前操作');
  const selected=projectSelected();if(!selected.length)throw Error('至少选择一项');
  const current=client,currentData=observationConnection();const currentProject=selectedProject;
  // Release manual subscriptions before checks so a prior form's throttle/QoS
  // cannot become a false data failure in the shared SDK connection.
  closeSubscriptions();if(imageUrl)URL.revokeObjectURL(imageUrl);imageUrl=undefined;$('image-preview').removeAttribute('src');
  show('scene-state','项目检查已暂停手动3D观测；检查结束后可重新开始');
  show('media-state','项目检查已暂停手动图像观测');show('plot-state','项目检查已暂停手动曲线');
  projectHasMutations=currentProject.cases.some(c=>selected.includes(c.id)&&c.interactive);
  projectRunBusy=true;projectAbort=new AbortController();projectReport=undefined;onlineControls();
  delete $('project-cases').dataset.complete;
  for(const spec of currentProject.cases)projectResult({id:spec.id,status:'NOT_RUN',startedAt:null,elapsedMs:0,evidence:{selected:selected.includes(spec.id)}});
  try{
    const engine=await optionalModule('audit');engine.validateProject(currentProject);
    if(client!==current||observationConnection()!==currentData||!current.connected||!currentData.connected)throw Error('连接已变化，请重新运行');
    projectPendingRun=engine.runProjectChecks(current,currentProject,selected,{observationClient:currentData,signal:projectAbort.signal,onResult:projectResult,onProgress:p=>{
      $('project-progress').textContent=`${p.completed}/${p.total} · ${p.phase==='running'?'正在检查':'已结束'}：${p.label}`;
      if(p.phase==='running'){const row=projectRows.get(p.caseId);if(row){row.status.textContent='运行中';row.status.className='audit-status running';}}
    }});
    const report=await projectPendingRun;
    projectReport=report;projectSummarize(report);log('project-report',report.totals.FAIL?'FAIL':'RESULT',{projectId:report.projectId,runId:report.runId,totals:report.totals});
    // undefined keeps the concise analysis above instead of replacing it with JSON.
  }finally{projectPendingRun=undefined;projectHasMutations=false;projectRunBusy=false;projectAbort=undefined;onlineControls();}
});
command('project-stop',null,()=>{projectAbort?.abort();$('project-progress').textContent='正在停止并等待恢复 / 收尾确认…';projectControls();return '已请求停止，本轮已发送的请求不会重放';});
command('project-select-read',null,()=>{for(const spec of selectedProject.cases)projectRows.get(spec.id).input.checked=!spec.interactive;projectSelectionLabel();});
command('project-select-none',null,()=>{for(const row of projectRows.values())row.input.checked=false;projectSelectionLabel();});
command('project-export',null,()=>{
  if(!projectReport)throw Error('先完成一次项目检查');
  const url=URL.createObjectURL(new Blob([json(projectReport)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download=`ros2-web-link-${projectReport.projectId}-${projectReport.runId}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
});
$('project-run-bottom').addEventListener('click',()=>$('project-run').click());
$('project-stop-bottom').addEventListener('click',()=>$('project-stop').click());
$('project-select').addEventListener('change',()=>{try{projectApply(value('project-select'));}catch(error){show('project-analysis',error.message);}});
// An unloading browser cannot guarantee network cleanup. Protect active
// mutations against accidental navigation and expose an explicit Stop button.
window.addEventListener('beforeunload',event=>{if(projectRunBusy&&projectHasMutations){event.preventDefault();event.returnValue='';}});
function stopProjectForPageHide(){projectAbort?.abort();return projectPendingRun;}

configReady.then(config=>{
  projectBase=structuredClone(config);
  if(!Array.isArray(projectBase.projects)||!projectBase.projects.length)throw Error('项目清单未提供，请重启新版 Web launch');
  $('project-select').replaceChildren(...projectBase.projects.map(project=>{const option=document.createElement('option');option.value=project.id;option.textContent=project.title;return option;}));
  $('project-select').value=config.project;projectApply(config.project);
}).catch(error=>{show('project-analysis',error.message);$('project-select').disabled=true;});
