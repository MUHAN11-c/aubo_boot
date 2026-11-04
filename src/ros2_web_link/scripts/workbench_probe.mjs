/** Explicit configuration/lifecycle checks; browser/ROS evidence is separate. */
import assert from 'node:assert/strict';
const api=await import('../dist/workbench/index.js').catch(()=>({}));
assert.equal(typeof api.validateWorkspace,'function','workbench public configuration API is required');
const {DisplayRegistry,createDefaultRegistry,validateWorkspace,parseWorkspace,workspaceFromProfile,WorkbenchController,previewMessage}=api;
const registry=createDefaultRegistry();
const base=()=>({schemaVersion:1,fixedFrame:'world',tf:'/tf',tfStatic:'/tf_static',useSimTime:false,layout:'split',displays:[]});
const display=(id,plugin='raw',extra={})=>({id,name:id,plugin,enabled:true,inputs:plugin==='tf'?{}:{data:{topic:`/test/${id}`,type:'std_msgs/msg/Float64',qos:'sensor'}},options:{},...extra});
const cloud=id=>display(id,'cloud',{inputs:{data:{topic:`/${id}`,type:'sensor_msgs/msg/PointCloud2',qos:'sensor',compression:'cbor',throttleMs:100}},options:{maxPoints:100000}});
let groups=0;
const test=(name,fn)=>{fn();groups++;console.log('PASS',name);};
test('schema, plugin, identifiers, JSON and global budgets reject atomically',()=>{
 assert.equal(validateWorkspace(base(),registry).displays.length,0);
 for(const change of [{schemaVersion:2},{fixedFrame:''},{tf:'relative'},{useSimTime:'false'},{layout:'float'},{displays:[display('a'),display('a')]},{displays:[display('a','missing')]},{displays:Array.from({length:17},(_,i)=>display(`a${i}`))},{displays:Array.from({length:9},(_,i)=>display(`tf${i}`,'tf'))},{displays:Array.from({length:5},(_,i)=>cloud(`c${i}`))}])assert.throws(()=>validateWorkspace({...base(),...change},registry));
 assert.equal(validateWorkspace({...base(),displays:Array.from({length:4},(_,i)=>cloud(`c${i}`))},registry).displays.length,4);
 const disabled=cloud('disabled');disabled.enabled=false;assert.doesNotThrow(()=>validateWorkspace({...base(),displays:[...Array.from({length:4},(_,i)=>cloud(`c${i}`)),disabled]},registry));
 assert.throws(()=>parseWorkspace(' '.repeat(1048577),registry),/budget|MiB/i);
 assert.throws(()=>parseWorkspace('{"__proto__":{}}',registry),/key|schema|unsafe/i);
 const cycle=base();cycle.cycle=cycle;assert.throws(()=>validateWorkspace(cycle,registry));
 const getter=base();Object.defineProperty(getter,'bad',{enumerable:true,get(){throw Error('getter executed');}});assert.throws(()=>validateWorkspace(getter,registry),/accessor|plain|unsafe/i);
});
test('port types, QoS and adapter options are checked before subscription',()=>{
 for(const binding of [{topic:'bad'},{type:'std_msgs/String'},{qos:'foo'},{qos:{history:'keep_all',depth:1,reliability:'reliable',durability:'volatile'}},{throttleMs:-1},{compression:'png'}])assert.throws(()=>validateWorkspace({...base(),displays:[display('bad','raw',{inputs:{data:{topic:'/valid',type:'std_msgs/msg/String',...binding}}})]},registry));
 assert.throws(()=>validateWorkspace({...base(),displays:[display('bad','cloud')]},registry),/type/i);
 assert.throws(()=>validateWorkspace({...base(),displays:[display('bad','plot',{options:{fields:['data;alert(1)']}})]},registry));
 assert.throws(()=>validateWorkspace({...base(),displays:[cloud('bad'),cloud('bad2')].map(d=>({...d,options:{maxPoints:100001}}))},registry));
 assert.throws(()=>validateWorkspace({...base(),displays:[display('bad','image',{inputs:{data:{topic:'/image',type:'sensor_msgs/msg/Image'}},options:{depthScale:0,maxPixels:4000001}})]},registry));
 const a=display('a'),b=display('b');b.inputs.data.topic=a.inputs.data.topic;b.inputs.data.qos='reliable';assert.throws(()=>validateWorkspace({...base(),displays:[a,b]},registry),/same|conflict/i);
 b.enabled=false;assert.doesNotThrow(()=>validateWorkspace({...base(),displays:[a,b]},registry));
});
test('profile templates remain paused policy and preserve topic options',()=>{
 const w=workspaceFromProfile({scene:{fixedFrame:'world',tf:'/my/tf',tfStatic:'/my/static',robotDescription:'/robot',jointStates:'/joints',markers:'/markers',pointCloud:'/points',path:'/path'},image:{topic:'/image',type:'sensor_msgs/msg/Image',depthScale:.001},depthImage:{topic:'/depth',type:'sensor_msgs/msg/Image',depthScale:.001},plot:{topic:'/sensor',type:'std_msgs/msg/Float64',fields:['data']}},registry);
 assert.equal(w.displays.length,8);assert(w.displays.every(d=>d.enabled===false));assert.equal(w.tf,'/my/tf');assert.equal(w.displays.find(d=>d.plugin==='image').options.depthScale,.001);
});
test('message preview is bounded and never expands typed image/point buffers',()=>{
 const text=previewMessage({data:new Uint8Array(1000000),nested:{text:'x'.repeat(100000)},bad:NaN});assert(text.length<=8192);assert.match(text,/1000000|100000/);
 const cycle={};cycle.self=cycle;assert.doesNotThrow(()=>previewMessage(cycle));
 const schema={definitions:{request:{typedefs:[{type:'demo/srv/S',fieldnames:['a','b'],fieldtypes:['int64','int64']}]}}};assert.match(previewMessage(schema,9),/int64/,'IDL preview must retain field names and types');
});
test('local registration, peer lifetime, rollback, disconnect and explicit resume',()=>{
 const reg=new DisplayRegistry(),callbacks=new Map(),listeners=new Set();let starts=0,stops=0;
 const client={connected:true,onState(fn){listeners.add(fn);fn('connected');return()=>listeners.delete(fn);},subscribe(name,type,fn){const set=callbacks.get(name)??new Set();set.add(fn);callbacks.set(name,set);return()=>{set.delete(fn);if(!set.size)callbacks.delete(name);};}};
 reg.register({id:'custom',title:'Custom',spatial:false,ports:[{id:'data',types:['*']}],validate:o=>o,create(context,spec){let off;return{start(){if(spec.options.fail)throw Error('adapter failed');off=context.client.subscribe(spec.inputs.data.topic,spec.inputs.data.type,()=>{});starts++;},dispose(){if(off){off();off=undefined;stops++;}},stats(){return{ok:true};}};}});
 assert.throws(()=>reg.register(reg.get('custom')),/registered|duplicate/i);
 let prepared=0;const controller=new WorkbenchController({getObservationClient:()=>client,beforeResume(){prepared++;}},reg,base());
 controller.add(display('a','custom'));controller.add(display('b','custom',{inputs:{data:{topic:'/test/a',type:'std_msgs/msg/Float64',qos:'sensor'}}}));assert.equal(starts,0);
 controller.resume();assert.equal(prepared,1);assert.equal(callbacks.get('/test/a').size,2);
 controller.setEnabled('a',false);assert.equal(callbacks.get('/test/a').size,1);assert.equal(controller.stats().activeDisplays,1);
 const saved=controller.snapshot();assert.throws(()=>controller.add(display('fail','custom',{options:{fail:true}})),/adapter failed/);assert.deepEqual(controller.snapshot(),saved);assert.equal(callbacks.get('/test/a').size,1);
 assert.throws(()=>controller.replace({...base(),schemaVersion:99}));assert.equal(controller.stats().activeDisplays,1);
 controller.pause();assert.equal(callbacks.size,0);assert.equal(listeners.size,0);assert.equal(controller.snapshot().displays[1].enabled,true);
 const beforeImport=starts;controller.replace(saved);assert.equal(starts,beforeImport,'replace/import must not resume');
 controller.resume();client.connected=false;for(const fn of [...listeners])fn('disconnected');assert.equal(callbacks.size,0);assert.equal(listeners.size,0);assert.equal(controller.stats().running,false);
 client.connected=true;assert.equal(callbacks.size,0,'reconnection must not replay display sessions');controller.resume();controller.remove('b');assert.equal(callbacks.size,0);
 controller.dispose();controller.dispose();assert.throws(()=>controller.resume(),/disposed/i);assert.equal(starts,stops);
});
console.log(`PASS ${groups} workbench groups; no server, ROS process or GL context started`);
