/** Execute the actual page connection handlers with controlled SDK/DOM edges.
 * No browser or ROS. This covers ownership/races, not rendered UI performance.
 */
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
const source=readFileSync(new URL('../web/app.js',import.meta.url),'utf8');
const start=source.indexOf("command('connect',"),end=source.indexOf("command('graph-refresh',",start);
assert.ok(start>=0&&end>start);
const helper=source.match(/^const observationConnection=.*$/m)?.[0]??'const observationConnection=()=>observationClient??client;';
function setup({split=false,rejectSecond=false,deferred=false}={}){
  const created=[],handlers=new Map(),nodes=new Map(),released=[],revoked=[];let ready;
  const configReady=deferred?new Promise(resolve=>{ready=resolve;}):Promise.resolve({});
  const node=id=>{if(!nodes.has(id))nodes.set(id,{checked:id==='connection-split'&&split,value:'ws://same.invalid:9090',textContent:'',className:'',removed:[],removeAttribute(name){this.removed.push(name);}});return nodes.get(id);};
  class FakeClient{
    constructor(options){this.options=options;this.state='idle';this.connected=false;this.states=[];this.disposeCalls=0;this.disconnectCalls=0;this.index=created.length;created.push(this);}
    onState(fn){this.states.push(fn);fn(this.state);return()=>{};}
    onStatus(){return()=>{};}
    emit(state){this.state=state;this.connected=state==='connected';for(const fn of this.states)fn(state);}
    connect(){if(rejectSecond&&this.index===1){this.emit('disconnected');return Promise.reject(Error('second connection rejected'));}this.emit('connected');return Promise.resolve();}
    disconnect(){this.disconnectCalls++;this.emit('disconnected');}
    stats(){return {subscriptions:0,pending:0,pendingLimit:256};}
    transportStats(){return {};}
    dispose(){this.disposeCalls++;this.emit('disposed');}
  }
  const context=vm.createContext({RosClient:FakeClient,configReady,$:node,value:id=>node(id).value,
    command:(id,_output,fn)=>handlers.set(id,fn),released,URL:{revokeObjectURL:url=>revoked.push(url)},show:(id,text)=>{node(id).textContent=text;},onlineControls(){},log(){},Promise});
  const optional=source.slice(source.indexOf('function stopOptional()'),source.indexOf('// Optional bundles'));
  const close=source.slice(source.indexOf('function stopClassicSubscriptions()'),start);
  const metricsStart=source.indexOf('function updateMetrics()');
  const tickStart=source.indexOf('const timer=setInterval(');
  const metrics=metricsStart>=0?source.slice(metricsStart,tickStart):'function updateMetrics(){'+source.slice(tickStart+'const timer=setInterval(()=>{'.length,source.indexOf('},500);',tickStart))+'}';
  vm.runInContext('let client,workbenchView,observationClient,connectionRequest=0,sceneRequest=0,imageRequest=0,plotRequest=0,observeRequest=0,sceneView,mediaView,plotView,topicStop,imageUrl,goal,topicLast=0;let observeStops=[];\n'+helper+'\n'+optional+'\n'+close+'\n'+metrics+'\n'+source.slice(start,end)+`
this.seed=()=>{sceneView={dispose:()=>released.push('scene')};mediaView={dispose:()=>released.push('image')};plotView={dispose:()=>released.push('plot')};topicStop=()=>released.push('topic');observeStops=[()=>released.push('standard1'),()=>released.push('standard2')];imageUrl='blob:owned-test';goal={cancel:()=>released.push('remote-goal-cancel')};};
this.snapshot=()=>({client,observationClient,sceneRequest,imageRequest,plotRequest,sceneView,mediaView,plotView,topicStop,observeStops,imageUrl,goal});`,context);
  return {created,handlers,nodes,node,ready,released,revoked,seed:()=>context.seed(),snapshot:()=>context.snapshot(),
    installSceneModule(load){context.optionalModule=load;context.performance=performance;context.parse=()=>({});context.document={querySelectorAll:()=>[]};
      const sceneBegin=source.indexOf("command('scene-start',"),sceneEnd=source.indexOf("command('scene-stop',",sceneBegin);
      assert.ok(sceneBegin>=0&&sceneEnd>sceneBegin);vm.runInContext(source.slice(sceneBegin,sceneEnd),context);}};
}
const cases={
  async handshakeGatesNewOperations(){
    const html=readFileSync(new URL('../web/index.html',import.meta.url),'utf8'),nodes=new Map(),handlers=new Map(),created=[];let rejectData;
    for(const [,id] of html.matchAll(/\bid="([^"]+)"/g))nodes.set(id,{querySelector:()=>null,disabled:false,checked:id==='connection-split',value:'ws://same.invalid:9090'});
    const buttons=[];
    for(const [,attrs] of html.matchAll(/<button\b([^>]*)>/g)){const id=attrs.match(/\bid="([^"]+)"/)?.[1];if(!id)continue;const n=nodes.get(id);n.id=id;n.hasAttribute=name=>new RegExp('\\b'+name+'\\b').test(attrs);n.setAttribute=()=>{};buttons.push(n);}
    class HalfConnected{
      constructor(options){this.options=options;this.connected=false;this.state='idle';this.callbacks=[];this.index=created.length;this.disposed=false;created.push(this);}
      onState(fn){this.callbacks.push(fn);fn(this.state);return()=>{};}
      onStatus(){return()=>{};}
      emit(state){this.state=state;this.connected=state==='connected';this.callbacks.forEach(fn=>fn(state));}
      connect(){if(this.index===0){this.emit('connected');return Promise.resolve();}this.emit('connecting');return new Promise((_resolve,reject)=>{rejectData=reject;});}
      dispose(){this.disposed=true;this.emit('disposed');}
    }
    const a=source.indexOf('function onlineControls()'),b=source.indexOf('function command(',a);
    const context=vm.createContext({RosClient:HalfConnected,configReady:Promise.resolve({}),$:id=>{assert.ok(nodes.has(id),id);return nodes.get(id);},value:id=>nodes.get(id).value,
      command:(id,_output,fn)=>handlers.set(id,fn),closeSubscriptions(){},log(){},projectControls(){},projectRunBusy:false,checkBusy:false,goal:undefined,serviceWait:undefined,busyCommands:new Set(['connect']),document:{querySelectorAll:()=>buttons},Promise});
    vm.runInContext('let client,workbenchView,observationClient,connectionRequest=0;\n'+helper+'\n'+source.slice(a,b)+'\n'+source.slice(start,end),context);
    const pending=handlers.get('connect')();void pending.catch(()=>{});
    try{await new Promise(resolve=>setImmediate(resolve));assert.equal(created.length,2);assert.equal(created[0].connected,true);assert.equal(created[1].state,'connecting');
      for(const id of ['service-call','action-send','parameter-set','topic-publish','scene-start'])assert.equal(nodes.get(id).disabled,true,`${id} enabled before paired handshake completed`);
      context.goal={};context.serviceWait={};vm.runInContext('onlineControls();',context);assert.equal(nodes.get('action-cancel').disabled,false);assert.equal(nodes.get('service-abort').disabled,false);
    }finally{rejectData?.(Error('controlled observation rejection'));await assert.rejects(pending,/controlled observation/);assert.ok(created.every(c=>c.disposed));}
  },
  async actualHtmlControls(){
    const html=readFileSync(new URL('../web/index.html',import.meta.url),'utf8'),nodes=new Map();
    for(const [,id] of html.matchAll(/\bid="([^"]+)"/g))nodes.set(id,{querySelector:()=>null,disabled:false});
    const buttons=[];
    for(const [,attrs] of html.matchAll(/<button\b([^>]*)>/g)){
      const id=attrs.match(/\bid="([^"]+)"/)?.[1];if(!id)continue;
      const button=nodes.get(id);button.id=id;button.hasAttribute=name=>new RegExp('\\b'+name+'\\b').test(attrs);button.setAttribute=()=>{};buttons.push(button);
    }
    const a=source.indexOf('function onlineControls()'),b=source.indexOf('function command(',a);assert.ok(a>=0&&b>a);
    const context=vm.createContext({client:undefined,workbenchView:undefined,observationClient:undefined,projectRunBusy:false,checkBusy:false,goal:undefined,serviceWait:undefined,busyCommands:new Set(),projectControls(){},document:{querySelectorAll:()=>buttons},$:id=>{assert.ok(nodes.has(id),`Unknown actual HTML id: ${id}`);return nodes.get(id);}});
    vm.runInContext(helper+'\n'+source.slice(a,b)+'\nonlineControls();',context);
    assert.equal(nodes.get('topic-subscribe').disabled,true);assert.equal(nodes.get('self-check').disabled,true);assert.equal(nodes.get('connection-split').disabled,false);
  },
  async shared(){const s=setup();await s.handlers.get('connect')();assert.equal(s.created.length,1);assert.equal(s.snapshot().observationClient,undefined);},
  async separate(){const s=setup({split:true});await s.handlers.get('connect')();assert.equal(s.created.length,2);assert.notEqual(s.snapshot().client,s.snapshot().observationClient);assert.equal(s.created[0].options.url,s.created[1].options.url);},
  async partialFailure(){const s=setup({split:true,rejectSecond:true});await assert.rejects(s.handlers.get('connect')(),/second/);assert.equal(s.created.length,2);for(const c of s.created)assert.equal(c.disposeCalls,1);},
  async manualDisconnectReleasesLocalResources(){
    const s=setup({split:true});await s.handlers.get('connect')();s.seed();
    const before=s.snapshot(),removedBefore=s.node('image-preview').removed.length;await s.handlers.get('disconnect')();
    assert.deepEqual(s.released,['scene','image','plot','topic','standard1','standard2'],'explicit disconnect retained local view/subscription resources');
    assert.deepEqual(s.revoked,['blob:owned-test']);assert.deepEqual(s.node('image-preview').removed.slice(removedBefore),['src']);
    const after=s.snapshot();for(const key of ['sceneView','mediaView','plotView','topicStop','imageUrl'])assert.equal(after[key],undefined,key);
    assert.equal(after.observeStops.length,0);
    for(const key of ['sceneRequest','imageRequest','plotRequest'])assert.equal(after[key],before[key]+1,`${key} must invalidate a pending optional startup`);
    for(const c of s.created)assert.equal(c.disconnectCalls,1);
    await s.handlers.get('disconnect')();assert.equal(s.released.length,6);assert.equal(s.revoked.length,1,'repeat disconnect revoked/released twice');
  },
  async disconnectInvalidatesAwaitingOptionalStartup(){
    const s=setup();await s.handlers.get('connect')();let resolveModule,created=0,disposed=0;
    const loaded=new Promise(resolve=>{resolveModule=resolve;});
    class View {constructor(){created++;}start(){}dispose(){disposed++;}setLayer(){}}
    s.installSceneModule(()=>loaded);
    const pending=s.handlers.get('scene-start')();
    await s.handlers.get('disconnect')();await s.handlers.get('connect')();
    resolveModule({SceneView:View});assert.equal(await pending,'3D 启动已取消');assert.equal(created,0,'old module completion created a view in the new session');
    await s.handlers.get('scene-start')();assert.equal(created,1,'positive control did not construct the view');
    await s.handlers.get('disconnect')();assert.equal(disposed,1);assert.equal(s.snapshot().sceneView,undefined);
  },
  async manualDisconnectRefreshesCountsWithoutTimerTick(){
    const s=setup({split:true});await s.handlers.get('connect')();s.seed();
    s.node('metric-subscriptions').textContent='9';s.node('metric-pending').textContent='2 / 512';
    await s.handlers.get('disconnect')();
    assert.equal(String(s.node('metric-subscriptions').textContent),'0','disconnect left the preceding interval metrics visible');
    assert.equal(s.node('metric-pending').textContent,'0 / 512');
  },
  async manualDisconnectDoesNotCancelRemoteGoal(){
    const s=setup();await s.handlers.get('connect')();s.seed();
    assert.equal(typeof s.snapshot().goal.cancel,'function');
    await s.handlers.get('disconnect')();assert.equal(s.snapshot().client.disconnectCalls,1);assert.ok(!s.released.includes('remote-goal-cancel'),'manual transport disconnect cancelled a remote goal');
  },
  async disconnectBoth(){const s=setup({split:true});await s.handlers.get('connect')();await s.handlers.get('disconnect')();assert.equal(s.created.length,2);for(const c of s.created)assert.equal(c.disconnectCalls,1);},
  async replaceBoth(){const s=setup({split:true});await s.handlers.get('connect')();const before=[...s.created];await s.handlers.get('connect')();assert.equal(s.created.length,4);for(const c of before)assert.equal(c.disposeCalls,1);for(const c of before)c.emit('disconnected');assert.equal(s.node('metric-state').textContent,'CONNECTED','old callbacks changed the new session');},
  async cancelBeforeConfig(){const s=setup({split:true,deferred:true});const pending=s.handlers.get('connect')();await s.handlers.get('disconnect')();s.ready({});await pending;assert.equal(s.created.length,0,'late config opened a cancelled connection');},
};
let failed=0;
for(const [name,check] of Object.entries(cases))try{await check();console.log(`PASS ${name}`);}catch(error){failed++;console.error(`FAIL ${name}: ${error.message}`);}
assert.equal(failed,0,`${failed} page connection boundaries failed`);
console.log(`PASS ${Object.keys(cases).length} actual UI connection ownership/race groups`);
