/* UI is a consumer of the public SDK only. No ROS protocol/business code here. */
'use strict';
const $ = id => document.getElementById(id);
const {RosClient} = Ros2WebLink;
let client, topicStop, topicLast = 0, topicCount = 0, goal, serviceWait, checkBusy = false;
let observeStops = [], imageUrl, sceneView, mediaView, plotView;
let workbenchView;
let observationClient,connectionRequest=0;
const observationConnection=()=>observationClient??client;
let runtimeConfig;
let sceneRequest=0,imageRequest=0,plotRequest=0,observeRequest=0;
const modules = new Map();
const configReady = fetch('config.json').then(r=>{if(!r.ok)throw Error(`配置 HTTP ${r.status}`);return r.json();});
const records = [];
const busyCommands = new Set();
const json = value => JSON.stringify(value, null, 2);
const show = (id, value) => { $(id).textContent = formatPreview(value); };
const value = id => $(id).value.trim();
const parse = id => JSON.parse($(id).value);
// Preserve explicit project profiles in the visible form. Collapsing an object
// to a preset silently changes DDS reliability, history depth or durability.
function fillQosSelect(id,qos='sensor') {
  const select=$(id),profile=qos??'sensor';
  select.querySelectorAll('[data-project-qos]').forEach(option=>option.remove());
  const encoded=typeof profile==='string'?profile:JSON.stringify(profile);
  if(!Array.from(select.options).some(option=>option.value===encoded)){
    const option=document.createElement('option');option.value=encoded;
    option.dataset.projectQos=typeof profile==='string'?'preset':'json';
    option.textContent=typeof profile==='string'?`项目预设 · ${profile}`:`自定义 · ${profile.reliability} · ${profile.durability} · depth ${profile.depth}`;
    option.title=encoded;select.append(option);
  }
  select.value=encoded;
}
function readQosSelect(id) {
  const select=$(id);return select.selectedOptions[0]?.dataset.projectQos==='json'?JSON.parse(select.value):select.value;
}
const pause = ms => new Promise(resolve => setTimeout(resolve, ms));
// Stream previews must never stringify multi-megabyte binary arrays per frame.
function streamPreview(value,{maxNodes=256,maxChars=8192}={}){
  let nodes=maxNodes,chars=maxChars;const path=new WeakSet();
  const visit=(item,depth)=>{
    if(nodes--<=0||chars<=0)return '[preview budget]';
    if(typeof item==='string'){
      const text=previewText(item,Math.min(256,chars));chars-=text.length;
      return text.length<item.length?{characters:item.length,preview:text,truncated:true}:text;
    }
    if(item===null||typeof item==='boolean'||typeof item==='number'){
      chars-=String(item).length;return typeof item==='number'&&!Number.isFinite(item)?String(item):item;
    }
    if(typeof item==='bigint')return visit(`${item}n`,depth);
    if(typeof item!=='object')return `[${typeof item}]`;
    if(ArrayBuffer.isView(item))return {type:item.constructor.name,bytes:item.byteLength};
    if(Object.prototype.toString.call(item)==='[object ArrayBuffer]')return {type:'ArrayBuffer',bytes:item.byteLength};
    if(depth>=6)return '[depth limit]';
    if(path.has(item))return '[circular reference]';
    path.add(item);
    try{
      if(Array.isArray(item)){
        const first=[];
        for(let i=0;i<Math.min(item.length,16)&&nodes>0&&chars>0;i++)first.push(visit(item[i],depth+1));
        return first.length<item.length?{length:item.length,first,truncated:true}:first;
      }
      const result=Object.create(null);let count=0;
      // Object.entries eagerly reads every value, even if sliced afterward.
      for(const key in item){
        if(!Object.hasOwn(item,key))continue;
        if(count>=32||nodes<=0||chars<=0){result['[preview]']='more fields omitted';break;}
        const field=previewText(key,Math.min(128,chars));chars-=field.length;count++;
        try{result[field]=visit(item[key],depth+1);}catch{result[field]='[unreadable]';}
      }
      return result;
    }finally{path.delete(item);}
  };
  try{return visit(value,0);}catch{return '[unreadable]';}
}
function previewText(value,limit){
  let end=Math.max(0,Math.min(value.length,limit));
  if(end<value.length&&end>0&&value.charCodeAt(end-1)>=0xd800&&value.charCodeAt(end-1)<=0xdbff)end--;
  // Copy bounded text so a retained substring cannot pin a whole frame.
  return JSON.parse(JSON.stringify(value.slice(0,end)));
}
function formatPreview(value,limit=18000){
  if(typeof value==='string'){
    const suffix=value.length>limit?`… [original ${value.length} characters]`:'';
    return previewText(value,limit-suffix.length)+suffix;
  }
  const text=json(streamPreview(value,{maxChars:Math.floor(limit/2)}));
  // Keep structured excerpts valid JSON rather than clipping mid-token.
  return text.length<=limit?text:json({truncated:true,characters:text.length,preview:previewText(text,Math.floor((limit-64)/3))});
}
function log(operation, result, detail) {
  const snapshot=formatPreview(detail,4096);
  detail=typeof detail==='string'?snapshot:JSON.parse(snapshot);
  records.push({at:new Date().toISOString(),operation,result,detail});
  if(records.length>200) records.shift();
  // Render only the visible budget. Stored/exported records never revisit the
  // original ROS arrays or buffers, including Service/Action return values.
  let text='';
  for(let i=records.length-1;i>=0&&text.length<18000;i--){
    const r=records[i];
    const date=new Date(r.at);
    const local=`${date.toLocaleTimeString('zh-CN',{hour12:false})}.${String(date.getMilliseconds()).padStart(3,'0')}`;
    const line=`${text?'\n\n':''}${local}  ${r.result}  ${r.operation}\n${typeof r.detail==='string' ? r.detail : json(r.detail)}`;
    text+=line.slice(0,18000-text.length);
  }
  show('log',text);
}
function onlineControls() {
  const connected=!!client?.connected,connecting=busyCommands.has('connect');
  const auditing=typeof projectRunBusy!=='undefined'&&projectRunBusy;
  document.querySelectorAll('button').forEach(button=>{
    const busy=busyCommands.has(button.id);
    button.disabled=busy || (button.hasAttribute('data-online') && (!connected || connecting || checkBusy || auditing));
    button.setAttribute('aria-busy',String(busy));
  });
  const dataReady=!!observationConnection()?.connected;
  for(const id of ['topic-subscribe','observe-start','scene-start','media-start','plot-start'])$(id).disabled=connecting||!dataReady||auditing||checkBusy||busyCommands.has(id);
  $('connection-split').disabled=auditing||checkBusy||!!goal||!!serviceWait||busyCommands.has('connect')||[client,observationClient].some(link=>link&&['connecting','connected','reconnecting'].includes(link.state));
  $('self-check').disabled=connecting || !connected || !dataReady || auditing || checkBusy || !!goal || !!serviceWait || busyCommands.has('self-check');
  $('action-send').disabled=connecting || !connected || auditing || !!goal || checkBusy || busyCommands.has('action-send');
  $('action-cancel').disabled=!connected || !goal || busyCommands.has('action-cancel');
  $('service-abort').disabled=!serviceWait || busyCommands.has('service-abort');
  if(typeof projectControls==='function')projectControls();
  workbenchView?.syncConnection();
  const workbenchRun=$('workbench-root').querySelector('[data-wb="run"]');
  if(workbenchRun&&(auditing||checkBusy||connecting))workbenchRun.disabled=true;
}
function command(id, output, fn) {
  $(id).addEventListener('click', async()=>{
    // Ignore re-entry while this exact operation is pending; never queue clicks.
    if(busyCommands.has(id)) return;
    if(['topic-subscribe','observe-start','scene-start','media-start','plot-start'].includes(id))workbenchView?.pause();
    busyCommands.add(id);onlineControls();
    try {const result=await fn(); if(result !== undefined && output) show(output,result);log(id,'OK',result ?? '完成');}
    catch(error) {const detail={code:error.code || error.name,message:error.message};if(output) show(output,detail);log(id,'ERROR',detail);}
    finally {busyCommands.delete(id);onlineControls();}
  });
}
document.addEventListener('keydown',event=>{
  // Holding Enter on a focused button otherwise produces repeated native clicks.
  if(event.repeat && (event.key==='Enter' || event.key===' ') && event.target instanceof HTMLButtonElement) event.preventDefault();
},true);
function stopOptional() {
  sceneRequest++;imageRequest++;plotRequest++;
  sceneView?.dispose();sceneView=undefined;mediaView?.dispose();mediaView=undefined;plotView?.dispose();plotView=undefined;
}
// Optional bundles are loaded once. Their public components own all subscriptions
// and DOM/GPU resources, so other applications can mount the same components.
async function optionalModule(kind) {
  if(!modules.has(kind)) modules.set(kind,new Promise((resolve,reject)=>{
    const script=document.createElement('script');script.src=`ros2-web-${kind}.js`;
    script.onload=()=>resolve(window[{scene:'Ros2WebScene',media:'Ros2WebMedia',audit:'Ros2WebAudit',graph:'Ros2WebGraph',workbench:'Ros2WebWorkbench'}[kind]]);
    script.onerror=()=>{modules.delete(kind);script.remove();reject(Error(`无法加载 ${kind} 组件，请先构建前端`));};document.head.append(script);
  }));
  return modules.get(kind);
}
function stopClassicSubscriptions() {
  observeRequest++;
  stopOptional();
  topicStop?.();topicStop=undefined;observeStops.forEach(off=>off());observeStops=[];
  if(imageUrl)URL.revokeObjectURL(imageUrl);imageUrl=undefined;$('image-preview').removeAttribute('src');
  show('scene-state','已停止 3D，订阅和图形资源已释放');
  show('media-state','已停止图像');show('plot-state','已停止曲线，样本与订阅已释放');
  show('image-state','已停止标准数据订阅');
}
function closeSubscriptions() {workbenchView?.pause();stopClassicSubscriptions();}
command('workbench-load','workbench-state',async()=>{
  if(workbenchView)return '工作台已加载';
  const config=await configReady,module=await optionalModule('workbench');
  workbenchView=new module.WorkbenchView($('workbench-root'),{
    getControlClient:()=>client,getObservationClient:observationConnection,profile:runtimeConfig??config,
    beforeResume:()=>{if(projectRunBusy||checkBusy)throw Error('请等待当前验收结束');stopClassicSubscriptions();},
    onStatus:status=>show('workbench-state',status),
  });$('workbench-load').hidden=true;return '选择显示并启动；关系图可独立刷新。';
});
command('connect','connection-detail',async()=>{
  const request=++connectionRequest;
  closeSubscriptions();observationClient?.dispose();observationClient=undefined;client?.dispose();
  const config=await configReady;if(request!==connectionRequest)return '连接已取消';
  const options={url:value('ws-url'),transportLimits:config.transportLimits,maxPendingRequests:config.maxPendingRequests};
  const current=new RosClient(options);client=current;
  const data=$('connection-split').checked?new RosClient(options):undefined;observationClient=data;
  const labels={idle:'待连接',connecting:'连接中',connected:'已连接',reconnecting:'重新连接中',disconnected:'已断开',disposed:'已释放'};
  current.onState(state=>{
    if(client!==current)return;
    $('connection-state').textContent=labels[state];
    $('connection-state').className=`state ${state==='connected'?'online':'offline'}`;
    $('metric-state').textContent=state.toUpperCase();onlineControls();
    log('connection',state,options.url);
  });
  current.onStatus(message=>log('rosbridge-control','STATUS',message));
  $('observation-connection-state').textContent=data?'观测连接：待连接':'观测连接：共用控制连接';
  if(data){
    data.onState(state=>{if(observationClient!==data)return;$('observation-connection-state').textContent=`观测连接：${labels[state]}`;onlineControls();log('observation-connection',state,options.url);});
    data.onStatus(message=>log('rosbridge-observation','STATUS',message));
  }
  try{await Promise.all([current.connect(),...(data?[data.connect()]:[])]);}
  catch(error){if(data){data.dispose();current.dispose();}throw error;}
  return data?'已连接同一 rosbridge：控制 / 观测各一条连接。断线只恢复订阅，不重放操作。':'已连接 rosbridge。断线只恢复订阅；已发送请求的结果可能未知。';
});
command('disconnect','connection-detail',()=>{connectionRequest++;closeSubscriptions();client?.disconnect();observationClient?.disconnect();updateMetrics();return '已断开并释放本地观测。远端任务不会因页面断开而自动取消。';});
command('graph-refresh','graph-result',()=>client.graph());
command('interface-resolve','interface-result',async()=>{
  const type=await client.interfaceType(value('interface-kind'),value('interface-name'));
  $('interface-type').value=type;return {name:value('interface-name'),type};
});
command('interface-describe','interface-result',()=>client.describeInterface(value('interface-type')));
command('topic-subscribe','topic-result',()=>{
  topicStop?.();topicCount=0;topicLast=0;
  topicStop=observationConnection().subscribe(value('topic-name'),value('topic-type'),(message,info)=>{
    topicCount++;topicLast=info.receivedAt;show('topic-result',streamPreview(message));$('topic-count').textContent=`接收 ${topicCount} 条`;
  },{qos:readQosSelect('topic-qos'),compression:value('topic-compression'),throttleMs:Number(value('topic-throttle'))});return '订阅已登记，等待数据';
});
command('topic-unsubscribe','topic-result',()=>{topicStop?.();topicStop=undefined;return '已取消订阅';});
command('topic-publish',null,async()=>{
  const payload=parse('publish-json');
  const pub=client.publisher(value('publish-name'),value('topic-type'),{qos:readQosSelect('topic-qos')});
  try {await pause(300);pub.publish(payload);await pause(100);return '已发布一次；实际接收请核对订阅端';}
  finally {pub.dispose();}
});
command('service-call','service-result',async()=>{
  if(serviceWait) throw Error('已有请求等待中，请先停止等待');
  serviceWait=new AbortController();onlineControls();show('service-result','等待响应…');
  try {return await client.callService(value('service-name'),value('service-type'),parse('service-json'),
    {timeoutMs:Number(value('service-timeout')),signal:serviceWait.signal});}
  finally {serviceWait=undefined;}
});
command('service-abort',null,()=>{serviceWait?.abort();return '仅停止本地等待';});
command('action-send','action-result',async()=>{
  if(goal) throw Error('当前页面已有 Goal 等待终态');
  show('action-feedback','等待反馈');show('action-result','等待终态');
  goal=client.sendGoal(value('action-name'),value('action-type'),parse('action-json'),{
    timeoutMs:Number(value('action-timeout')),onFeedback:feedback=>show('action-feedback',feedback)});
  $('action-state').textContent='已发送 / 等待反馈';onlineControls();
  try {
    const result=await goal.result;
    $('action-state').textContent=({0:'未知 / 被拒绝 / 桥接失败',4:'成功',5:'已取消',6:'已中止'})[result.status] || `状态 ${result.status}`;
    return result;
  } catch(error) {$('action-state').textContent='结果未知';throw error;}
  finally {goal=undefined;}
});
command('action-cancel',null,()=>{goal?.cancel();$('action-state').textContent='已请求取消 / 等待终态';return '取消请求已发送';});
const params=()=>client.parameters(value('parameter-node'));
command('parameter-list','parameter-result',()=>params().list());
command('parameter-get','parameter-result',async()=>{
  const names=value('parameter-names').split(',').map(s=>s.trim()).filter(Boolean);
  const [values,descriptions]=await Promise.all([params().get(names),params().describe(names)]);
  return names.map((name,i)=>({name,...values.values[i],descriptor:descriptions.descriptors[i]}));
});
command('parameter-set','parameter-write-result',()=>params().set(parse('parameter-json')));
command('parameter-atomic','parameter-write-result',()=>params().setAtomically(parse('parameter-json')));
async function readLifecycle() {
  const node=value('lifecycle-node');
  const [state,available]=await Promise.all([
    client.lifecycle(node).getState(),
    client.lifecycle(node).availableTransitions(),
  ]);
  $('lifecycle-transition').replaceChildren(...available.available_transitions.map(t=>{
    const option=document.createElement('option');option.value=String(t.transition.id);
    option.textContent=`${t.transition.label} (${t.transition.id}) → ${t.goal_state.label}`;return option;
  }));return {state:state.current_state,available:available.available_transitions};
}
command('lifecycle-read','lifecycle-result',readLifecycle);
command('lifecycle-change','lifecycle-result',async()=>{
  const id=Number(value('lifecycle-transition'));if(!id) throw Error('请先读取可用转换');
  const response=await client.lifecycle(value('lifecycle-node')).change(id);
  return {response,...await readLifecycle()};
});
command('observe-start','observation-result',async()=>{
  const request=++observeRequest,current=observationConnection();
  observeStops.forEach(off=>off());observeStops=[];
  if(imageUrl)URL.revokeObjectURL(imageUrl);imageUrl=undefined;$('image-preview').removeAttribute('src');
  const image=runtimeConfig.image;
  // The standard panel shares the same validated payload boundary as ImageView.
  // A late optional load must not revive an explicitly stopped observation.
  const media=image.type==='sensor_msgs/msg/CompressedImage'?await optionalModule('media'):undefined;
  if(request!==observeRequest)return '标准数据启动已取消';
  if(observationConnection()!==current||!current.connected)throw Error('连接已变化，请重新开始观测');
  const latest={},scene=runtimeConfig.scene;
  for(const [key,topic,type,options] of [['tf',scene.tf,'tf2_msgs/msg/TFMessage',{qos:'reliable'}],
    ['joint_states',scene.jointStates,'sensor_msgs/msg/JointState',{qos:'sensor',...scene.jointStatesOptions}],
    ['points',scene.pointCloud,'sensor_msgs/msg/PointCloud2',{qos:'sensor',compression:'cbor',throttleMs:100,...scene.pointCloudOptions}],
    ['diagnostics',runtimeConfig.diagnostics,'diagnostic_msgs/msg/DiagnosticArray',{qos:'reliable'}]]) {
    observeStops.push(current.subscribe(topic,type,(message,info)=>{
      if(request!==observeRequest)return;
      latest[key]={topic,receivedAt:new Date(info.receivedAt).toLocaleTimeString(),message:streamPreview(message)};show('observation-result',latest);
    },options));
  }
  if(media)observeStops.push(current.subscribe(image.topic,'sensor_msgs/msg/CompressedImage',(message,info)=>{
    if(request!==observeRequest)return;
    try{
      const {bytes,mime}=media.compressedImagePayload(message,{maxPixels:image.maxPixels});
      const next=URL.createObjectURL(new Blob([new Uint8Array(bytes).buffer],{type:mime}));
      if(imageUrl)URL.revokeObjectURL(imageUrl);imageUrl=next;$('image-preview').src=next;
      $('image-state').textContent=`${message.header?.frame_id??'未知帧'} · ${bytes.length} B · ${new Date(info.receivedAt).toLocaleTimeString()}`;
    }catch(error){show('image-state',{code:'IMAGE_REJECTED',message:error.message});}
  },{qos:'sensor',throttleMs:500,compression:'cbor',...image.subscription}));
  else $('image-state').textContent='当前项目使用原始图像；在“图像与曲线”显示彩色/深度图';
  return '已订阅当前项目的标准消息；大数组保留摘要';
});
command('observe-stop',null,()=>{
  observeRequest++;
  observeStops.forEach(off=>off());observeStops=[];
  if(imageUrl) URL.revokeObjectURL(imageUrl);imageUrl=undefined;$('image-preview').removeAttribute('src');return '已停止标准数据订阅';
});


command('scene-start','scene-state',async()=>{
  const begun=performance.now();show('scene-state','正在加载 3D 组件…');
  const request=++sceneRequest;const current=observationConnection();const module=await optionalModule('scene');const moduleLoadMs=performance.now()-begun;
  if(request!==sceneRequest)return '3D 启动已取消';
  if(observationConnection()!==current || !current.connected)throw Error('连接已变化，请重新开始观测');
  sceneView?.dispose();sceneView=undefined;
  const options={...parse('scene-config'),fixedFrame:value('scene-fixed'),onStatus:status=>{show('scene-state',status);if(sceneView)show('scene-metrics',sceneView.performanceStats());}};
  const createdAt=performance.now();const next=new module.SceneView($('scene-view'),current,options);const constructorMs=performance.now()-createdAt;
  const startAt=performance.now();
  try {next.start();sceneView=next;}catch(error){next.dispose();throw error;}
  show('scene-bootstrap',{moduleLoadMs,constructorMs,startMs:performance.now()-startAt,totalMs:performance.now()-begun});
  document.querySelectorAll('[data-layer]').forEach(input=>sceneView.setLayer(input.dataset.layer,input.checked));
  return '等待 URDF、TF 与空间数据…';
});
command('scene-stop','scene-state',()=>{sceneRequest++;sceneView?.dispose();sceneView=undefined;return '已停止 3D，订阅和图形资源已释放';});
command('scene-frame',null,()=>{if(!sceneView)throw Error('先开始 3D 观测');sceneView.setFixedFrame(value('scene-fixed'));return '已切换固定坐标系';});
command('scene-camera',null,()=>{sceneView?.resetCamera();return '视角已复位';});
document.querySelectorAll('[data-layer]').forEach(input=>input.addEventListener('change',()=>sceneView?.setLayer(input.dataset.layer,input.checked)));
function applyImageConfig(config) {
  $('media-topic').value=config.topic;$('media-type').value=config.type;$('media-scale').value=config.depthScale??'';
  $('media-compression').value=config.subscription?.compression??'cbor';$('media-throttle').value=config.subscription?.throttleMs??500;
  fillQosSelect('media-qos',config.subscription?.qos);
}
command('media-image','media-state',async()=>{await configReady;applyImageConfig(runtimeConfig.image);return '已填入彩色图配置，点击显示图像开始';});
command('media-depth','media-state',async()=>{await configReady;if(!runtimeConfig.depthImage)throw Error('当前 profile 未配置深度图入口');applyImageConfig(runtimeConfig.depthImage);return '已填入深度图配置，确认单位后点击显示图像';});
command('media-start','media-state',async()=>{
  const request=++imageRequest;const current=observationConnection();const options={topic:value('media-topic'),type:value('media-type'),depthScale:value('media-scale')?Number(value('media-scale')):undefined,subscription:{qos:readQosSelect('media-qos'),compression:value('media-compression'),throttleMs:Number(value('media-throttle'))},onStatus:status=>show('media-state',status)};
  const module=await optionalModule('media');if(request!==imageRequest)return '图像启动已取消';if(observationConnection()!==current||!current.connected)throw Error('连接已变化，请重新显示图像');
  mediaView?.dispose();mediaView=undefined;const next=new module.ImageView($('media-view'),current,options);
  try{next.start();mediaView=next;}catch(error){next.dispose();throw error;}return '等待图像…';
});
command('media-stop','media-state',()=>{imageRequest++;mediaView?.dispose();mediaView=undefined;return '已停止图像';});
command('plot-start','plot-state',async()=>{
  const request=++plotRequest;const current=observationConnection();const options={topic:value('plot-topic'),type:value('plot-type'),fields:value('plot-fields').split(',').map(s=>s.trim()).filter(Boolean),onStatus:status=>show('plot-state',status)};
  const module=await optionalModule('media');if(request!==plotRequest)return '曲线启动已取消';if(observationConnection()!==current||!current.connected)throw Error('连接已变化，请重新开始曲线');
  plotView?.dispose();plotView=undefined;const next=new module.SeriesPlot($('plot-view'),current,options);
  try{next.start();plotView=next;}catch(error){next.dispose();throw error;}return '等待曲线数据…';
});
command('plot-stop','plot-state',()=>{plotRequest++;plotView?.dispose();plotView=undefined;return '已停止曲线，样本与订阅已释放';});

async function selfCheck() {
  if(checkBusy) return;checkBusy=true;onlineControls();$('check-results').replaceChildren();
  $('metric-check-kind').textContent='demo 通信回归';delete $('check-results').dataset.complete;$('metric-checks').textContent='…';
  let passed=0,failed=0;
  const assert=(test,message)=>{if(!test) throw Error(message);};
  const waitFor=async(test)=>{for(let i=0;i<150 && !test();i++) await pause(40);assert(test(),'等待数据超时');};
  const check=async(title,fn)=>{
    const li=document.createElement('li');li.textContent=`··· ${title}`;$('check-results').append(li);
    try {await fn();passed++;li.className='pass';li.textContent=`✓ ${title}`;}
    catch(error) {failed++;li.className='fail';li.textContent=`× ${title}：${error.message}`;}
    $('metric-checks').textContent=`${passed}/${passed+failed}`;
  };
  try {
    await check('图谱发现：节点、话题、服务与 Action',async()=>{
      const graph=await client.graph();assert(graph.topics.topics.includes('/web_link_demo/ticks'),'未启动自测节点');show('graph-result',graph);
    });
    await check('接口类型：图谱识别与完整字段查询',async()=>{
      assert(await client.interfaceType('topic','/web_link_demo/ticks')==='std_msgs/msg/String','Topic 类型异常');
      const details=await client.describeInterface('example_interfaces/srv/AddTwoInts');assert(details.request.typedefs.length>0&&details.response.typedefs.length>0,'服务字段查询失败');
    });
    await check('Topic：发布、回显与释放订阅',async()=>{
      let received=false;const token=`browser-${Date.now()}`;
      const off=observationConnection().subscribe('/web_link_demo/echo','std_msgs/msg/String',m=>{received=m.data===token;});
      const pub=client.publisher('/web_link_demo/input','std_msgs/msg/String');
      try {await pause(350);pub.publish({data:token});await waitFor(()=>received);}finally {off();pub.dispose();}
    });
    await check('QoS：Best effort 连续流 / Transient local 晚订阅',async()=>{
      let sensor=false,retained=false;
      const off1=observationConnection().subscribe('/web_link_demo/sensor','std_msgs/msg/Float64',()=>{sensor=true;},{qos:'sensor'});
      const off2=observationConnection().subscribe('/web_link_demo/retained','std_msgs/msg/String',m=>{retained=m.data==='retained-once';},{qos:'latched'});
      try {await waitFor(()=>sensor&&retained);} finally {off1();off2();}
    });
    await check('Service：标准请求与响应',async()=>{assert((await client.callService('/web_link_demo/add_two_ints','example_interfaces/srv/AddTwoInts',{a:7,b:8})).sum===15,'加法结果错误');});
    await check('Service：超时必须返回明确错误',async()=>{
      try {await client.callService('/web_link_demo/slow','std_srvs/srv/Trigger',{}, {timeoutMs:100});throw Error('请求未超时');}
      catch(error) {assert(error.code==='TIMEOUT',error.message);}
    });
    await check('参数：九类参数读取、描述与节点拒绝写入',async()=>{
      const p=client.parameters('/web_link_demo/demo');const list=await p.list();assert(list.names.includes('gain'),'参数列表异常');
      const names=['enabled','count','gain','label','bytes','bools','ints','doubles','strings'];
      const read=await p.get(names);assert(read.values.length===9,'参数读取异常');
      assert((await p.describe(['read_only'])).descriptors[0].read_only,'缺少只读描述');
      assert(!(await p.set([{name:'gain',type:3,value:-1}])).results[0].successful,'节点没有拒绝非法值');
      assert((await p.setAtomically([{name:'gain',type:3,value:1.0}])).result.successful,'原子写入失败');
    });
    await check('Action：反馈与成功终态 4',async()=>{
      let feedback=false;const r=await client.sendGoal('/web_link_demo/fibonacci','example_interfaces/action/Fibonacci',{order:5},{onFeedback:()=>{feedback=true;}}).result;
      assert(r.status===4&&feedback,'缺少反馈或成功结果');
    });
    await check('Action：请求取消并等待终态 5',async()=>{
      const h=client.sendGoal('/web_link_demo/fibonacci','example_interfaces/action/Fibonacci',{order:20});
      await pause(600);h.cancel();assert((await h.result).status===5,'没有收到取消终态');
    });
    await check('Action：拒绝与中止终态 6',async()=>{
      assert((await client.sendGoal('/web_link_demo/fibonacci','example_interfaces/action/Fibonacci',{order:-1}).result).status===0,'未拒绝非法目标');
      assert((await client.sendGoal('/web_link_demo/fibonacci','example_interfaces/action/Fibonacci',{order:13}).result).status===6,'未返回中止终态');
    });
    await check('观测示例：模型、静态 TF、Marker、Path 与深度消息',async()=>{
      const specs=[['robot_description','std_msgs/msg/String','latched'],['tf_static','tf2_msgs/msg/TFMessage','staticTf'],['markers','visualization_msgs/msg/MarkerArray','reliable'],['path','nav_msgs/msg/Path','reliable'],['depth','sensor_msgs/msg/Image','sensor']];
      const received=new Set();const offs=[];
      try{for(const [key,type,qos] of specs)offs.push(observationConnection().subscribe(`/web_link_demo/${key}`,type,()=>received.add(key),{qos,...(key==='depth'?{throttleMs:500}:{})}));await waitFor(()=>received.size===specs.length);}finally{offs.forEach(off=>off());}
    });
    await check('Lifecycle：原生状态与可用转换查询',async()=>{
      const state=await client.callService('/web_link_demo/lifecycle/get_state','lifecycle_msgs/srv/GetState');assert(state.current_state.label,'状态服务异常');
    });
  } finally {
    checkBusy=false;onlineControls();log('self-check',failed?'FAIL':'PASS',{passed,failed});
    $('check-results').dataset.complete=failed?'failed':'passed';
  }
  return {passed,failed};
}
command('self-check',null,selfCheck);
command('clear-log',null,()=>{records.length=0;show('log','记录已清空');});
command('export-log',null,()=>{
  const url=URL.createObjectURL(new Blob([json(records)],{type:'application/json'}));
  const a=document.createElement('a');a.href=url;a.download='ros2-web-link-selfcheck.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
});
function updateMetrics() {
  if(client) {
    const links=observationClient?[client,observationClient]:[client],stats=links.map(link=>link.stats());
    $('metric-subscriptions').textContent=stats.reduce((n,s)=>n+s.subscriptions,0);
    $('metric-pending').textContent=`${stats.reduce((n,s)=>n+s.pending,0)} / ${stats.reduce((n,s)=>n+s.pendingLimit,0)}`;
    show('transport-stats',observationClient?{mode:'separate',control:client.transportStats(),observation:observationClient.transportStats()}:client.transportStats());
  }
  $('topic-freshness').textContent=topicLast ? `${Math.round((Date.now()-topicLast)/1000)} 秒前收到${observationConnection()?.connected?'':' · 已断线'}`:'尚无数据';
}
const timer=setInterval(updateMetrics,500);
window.addEventListener('pagehide',()=>{
  clearInterval(timer);connectionRequest++;
  const finish=()=>{closeSubscriptions();workbenchView?.dispose();observationClient?.dispose();client?.dispose();if(imageUrl)URL.revokeObjectURL(imageUrl);};
  const cleanup=typeof stopProjectForPageHide==='function'?stopProjectForPageHide():undefined;
  if(cleanup)void cleanup.then(finish,finish);else finish();
});
onlineControls();
configReady.then(config=>{
  runtimeConfig=config;
  $('profile-state').textContent=`launch profile：${config.profile} · 资源包：${config.assetPackages.join(', ')} · 3D 与图像按需订阅`;
  $('scene-config').value=json(config.scene);$('scene-fixed').value=config.scene.fixedFrame;
  applyImageConfig(config.image);$('plot-topic').value=config.plot.topic;$('plot-type').value=config.plot.type;$('plot-fields').value=config.plot.fields.join(',');
  $('ws-url').value=`${location.protocol==='https:'?'wss':'ws'}://${location.hostname}:${config.wsPort}`;
}).catch(()=>{show('connection-detail','没有 config.json，请手动填写 rosbridge 地址。');});
