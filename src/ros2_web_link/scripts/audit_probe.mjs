/** Hardware-free audit boundary probe. No ROS node, browser or test framework. */
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';

const target=process.argv[2]??new URL('../dist/audit/index.js',import.meta.url).href;
const {runProjectChecks,validateProject}=await import(target.startsWith('file:')?target:pathToFileURL(target).href);
const base=(cases,extra={})=>({id:'probe',title:'Audit boundary probe',cases,...extra});
const topic={id:'stream',label:'Fresh stream',kind:'topic',timeoutMs:1000,topic:{name:'/fresh',type:'sensor_msgs/msg/JointState',minSamples:2,maxAgeMs:500,requiredFields:['position[0]']}};
const service={id:'service',label:'Read service',kind:'service',interactive:true,service:{name:'/read',type:'std_srvs/srv/Trigger',expected:{success:true}}};
function fake(overrides={}) {
  const listeners=new Set();const subscriptions=new Map();const calls=[];
  const api={connected:true,options:{url:'ws://fake.invalid:9090'},calls,subscriptions,
    onState(fn){listeners.add(fn);fn('connected');return()=>listeners.delete(fn);},
    subscribe(name,type,fn){subscriptions.set(name,fn);return()=>subscriptions.delete(name);},
    emit(name,message){subscriptions.get(name)?.(message,{receivedAt:Date.now()});},
    callService:async(name,type,args)=>{calls.push({name,type,args});return {success:true};},
    ...overrides};return api;
}
let checks=0;
async function probe(label,fn){await fn();checks++;console.log(`PASS ${label}`);}
await probe('unknown cases fail before any remote operation',async()=>{
  const client=fake();await assert.rejects(()=>runProjectChecks(client,base([service]),['missing']),/unknown|Unknown|不存在/);assert.equal(client.calls.length,0);
  assert.throws(()=>validateProject(base([{id:'bad',label:'bad',kind:'invented'}])),/kind|unknown|Unknown/);
});
await probe('unselected items remain NOT_RUN and never produce remote work',async()=>{
  const client=fake();const report=await runProjectChecks(client,base([service]),[]);assert.equal(report.results[0].status,'NOT_RUN');assert.equal(report.totals.NOT_RUN,1);assert.equal(client.calls.length,0);
});
await probe('abort cleans subscriptions and leaves later cases NOT_RUN',async()=>{
  const client=fake();const control=new AbortController();const pending=runProjectChecks(client,base([topic,service]),['stream','service'],{signal:control.signal});
  await new Promise(resolve=>setTimeout(resolve,5));assert.equal(client.subscriptions.size,1);control.abort();
  const report=await pending;assert.equal(report.results[0].status,'CANCELLED');assert.equal(report.results[1].status,'NOT_RUN');assert.equal(client.subscriptions.size,0);assert.equal(client.calls.length,0);
});
await probe('distinct source stamps, required fields and bounded evidence',async()=>{
  const client=fake();const pending=runProjectChecks(client,base([topic]),['stream']);await new Promise(resolve=>setTimeout(resolve,1));
  const ms=Date.now();const message=(time)=>({header:{stamp:{sec:Math.floor(time/1000),nanosec:time%1000*1e6}},position:[1],data:new Array(50000).fill(9)});
  client.emit('/fresh',message(ms));client.emit('/fresh',message(ms));client.emit('/fresh',message(ms+1));
  const report=await pending;assert.equal(report.results[0].status,'PASS');assert.equal(report.results[0].evidence.samples,2);assert.ok(JSON.stringify(report).length<10000);assert.equal(client.subscriptions.size,0);
});
await probe('parameter restore failure is FAIL and retained in evidence',async()=>{
  let value=false,writes=0;const client=fake({parameters:()=>({get:async()=>({values:[{type:1,value}]}),set:async(items)=>{writes++;if(writes===2)throw new Error('restore denied');value=items[0].value;return {results:[{successful:true,reason:''}]};}})});
  const report=await runProjectChecks(client,base([{id:'roundtrip',label:'Roundtrip',kind:'parameterRoundtrip',interactive:true,parameter:{node:'/observe',name:'enabled',type:1,value:true}}]),['roundtrip']);
  assert.equal(writes,2);assert.equal(report.results[0].status,'FAIL');assert.match(report.results[0].error,/restore|恢复/);assert.match(JSON.stringify(report.results[0].evidence),/restore denied/);
});
await probe('setup failure still executes cleanup without sending a goal',async()=>{
  let sent=0;const client=fake({callService:async(name)=>{client.calls.push(name);if(name==='/setup')throw new Error('setup refused');return {success:true};},sendGoal:()=>{sent++;throw new Error('must not send');}});
  const spec={id:'act',label:'Action',kind:'action',interactive:true,setup:[{name:'/setup',type:'std_srvs/srv/Trigger'}],cleanup:[{name:'/cleanup',type:'std_srvs/srv/Trigger',expected:{success:true}}],action:{name:'/act',type:'example_interfaces/action/Fibonacci',goal:{order:2}}};
  const report=await runProjectChecks(client,base([spec]),['act']);assert.equal(report.results[0].status,'FAIL');assert.deepEqual(client.calls,['/setup','/cleanup']);assert.equal(sent,0);
});
await probe('mock-only guard rejects real plugin before any write',async()=>{
  const client=fake();const spec={...service,mockOnly:true,timeoutMs:500};const pending=runProjectChecks(client,base([spec],{mockGuard:{topic:'/robot_description',type:'std_msgs/msg/String',qos:'latched',field:'data',includes:'mock_components/GenericSystem'}}),['service']);
  await new Promise(resolve=>setTimeout(resolve,1));client.emit('/robot_description',{data:'<plugin>aubo_e5_hardware/AuboSystem</plugin>'});const report=await pending;
  assert.equal(report.results[0].status,'FAIL');assert.equal(client.calls.length,0);assert.equal(client.subscriptions.size,0);
});
await probe('mock-only without guard is rejected',async()=>{
  assert.throws(()=>validateProject(base([{...service,mockOnly:true}])),/guard|Guard/);
});
await probe('action stop cancels only its handle, waits terminal, then cleanup',async()=>{
  let cancelled=0,resolveResult,resultSignal;const control=new AbortController();const client=fake({sendGoal:(name,type,goal,options)=>{resultSignal=options.signal;return {id:'current-goal',result:new Promise(resolve=>{resolveResult=resolve;}),cancel:()=>{cancelled++;assert.equal(resultSignal.aborted,false);setTimeout(()=>resolveResult({status:5,successful:false,value:{}}),10);}};}});
  const spec={id:'act',label:'Action',kind:'action',interactive:true,timeoutMs:1000,cleanup:[{name:'/cleanup',type:'std_srvs/srv/Trigger'}],action:{name:'/act',type:'example_interfaces/action/Fibonacci',goal:{order:2}}};
  const pending=runProjectChecks(client,base([spec]),['act'],{signal:control.signal});await new Promise(resolve=>setTimeout(resolve,5));control.abort();const report=await pending;
  assert.equal(cancelled,1);assert.equal(report.results[0].status,'CANCELLED');assert.equal(report.results[0].evidence.terminalStatus,5);assert.equal(client.calls.at(-1).name,'/cleanup');assert.equal(resultSignal.aborted,true);
});
await probe('accepted setup must wait for applied state before sending a goal',async()=>{
  let sent=0;const client=fake({sendGoal:()=>{sent++;throw new Error('must not send');}});
  const state={name:'/status',type:'std_msgs/msg/String',qos:'reliable',jsonField:'data',expected:{execution_enabled:true,grasp_enabled:false,tool_enabled:false}};
  const spec={id:'act',label:'Action',kind:'action',interactive:true,timeoutMs:30,setup:[{name:'/setup',type:'std_srvs/srv/Trigger',expected:{success:true}}],afterSetup:[state],cleanup:[{name:'/cleanup',type:'std_srvs/srv/Trigger'}],action:{name:'/act',type:'example_interfaces/action/Fibonacci',goal:{order:2}}};
  const pending=runProjectChecks(client,base([spec]),['act']);await new Promise(resolve=>setTimeout(resolve,2));client.emit('/status',{data:JSON.stringify({execution_enabled:false,grasp_enabled:false,tool_enabled:false})});
  const report=await pending;assert.equal(report.results[0].status,'FAIL');assert.equal(sent,0);assert.equal(client.calls.at(-1).name,'/cleanup');assert.equal(client.subscriptions.size,0);
});
await probe('cleanup state confirmation failure overrides local cancellation',async()=>{
  const control=new AbortController();let resolveResult;const client=fake({sendGoal:()=>({id:'one',result:new Promise(resolve=>{resolveResult=resolve;}),cancel:()=>resolveResult({status:5,successful:false,value:{}})})});
  const state={name:'/status',type:'std_msgs/msg/String',qos:'reliable',jsonField:'data',expected:{execution_enabled:false,grasp_enabled:false,tool_enabled:false}};
  const spec={id:'act',label:'Action',kind:'action',interactive:true,timeoutMs:1000,cleanup:[{name:'/cleanup',type:'std_srvs/srv/Trigger'}],afterCleanup:[state],action:{name:'/act',type:'example_interfaces/action/Fibonacci',goal:{order:2}}};
  const pending=runProjectChecks(client,base([spec]),['act'],{signal:control.signal});await new Promise(resolve=>setTimeout(resolve,2));control.abort();await new Promise(resolve=>setTimeout(resolve,2));client.emit('/status',{data:'invalid JSON'});
  const report=await pending;assert.equal(report.results[0].status,'FAIL');assert.match(report.results[0].error,/清理|cleanup/);assert.equal(client.subscriptions.size,0);
});
await probe('failure continues other selected items and service assertions use exact run ID',async()=>{
  const client=fake({callService:async(name,type,args)=>{client.calls.push({name,args});if(name==='/bad')return {success:false};return {success:true,run_id:args.run_id};}});
  const bad={...service,id:'bad',service:{...service.service,name:'/bad'}};
  const good={...service,id:'good',service:{...service.service,name:'/good',request:{run_id:'$runId'},expected:{success:true,run_id:'$runId'}}};
  const report=await runProjectChecks(client,base([bad,good]),['bad','good']);assert.deepEqual(report.results.map(r=>r.status),['FAIL','PASS']);assert.equal(client.calls[1].args.run_id,report.runId);
});
await probe('restores a possibly completed write after cancellation',async()=>{
  let value=false,writes=0;const control=new AbortController();const client=fake({parameters:()=>({get:async()=>({values:[{type:1,value}]}),set:async(items)=>{writes++;value=items[0].value;if(writes===1){control.abort();throw new Error('wait aborted after send');}return {results:[{successful:true,reason:''}]};}})});
  const spec={id:'roundtrip',label:'Roundtrip',kind:'parameterRoundtrip',interactive:true,parameter:{node:'/observe',name:'enabled',type:1,value:true}};
  const report=await runProjectChecks(client,base([spec]),['roundtrip'],{signal:control.signal});assert.equal(report.results[0].status,'CANCELLED');assert.equal(writes,2);assert.equal(value,false);assert.equal(report.results[0].evidence.restoreConfirmed,true);
});
await probe('raw BGR and depth use decoder metadata, not DOM or demo images',async()=>{
  for(const [encoding,data,depthScale]of [['bgr8',[10,20,30],undefined],['16UC1',[0xe8,3],0.00025]]){
    const client=fake();const spec={id:'image',label:'Image',kind:'image',timeoutMs:200,topic:{name:'/camera',type:'sensor_msgs/msg/Image',minSamples:1,encoding,width:1,height:1,...(depthScale?{depthScale}:{})}};
    const pending=runProjectChecks(client,base([spec]),['image']);await new Promise(resolve=>setTimeout(resolve,1));client.emit('/camera',{width:1,height:1,step:data.length,encoding,is_bigendian:false,data});const report=await pending;
    assert.equal(report.results[0].status,'PASS');assert.equal(report.results[0].evidence.latest.encoding,encoding);assert.equal(report.results[0].evidence.latest.decodedPixels,1);assert.ok(!JSON.stringify(report).includes('pixels":['));
  }
});
await probe('PointCloud2 rejects invalid layout and keeps later selections runnable',async()=>{
  const client=fake();const cloud={id:'cloud',label:'Cloud',kind:'cloud',timeoutMs:200,topic:{name:'/cloud',type:'sensor_msgs/msg/PointCloud2',minSamples:1,minPoints:1}};
  const pending=runProjectChecks(client,base([cloud,service]),['cloud','service']);await new Promise(resolve=>setTimeout(resolve,1));client.emit('/cloud',{width:1,height:1,point_step:12,row_step:12,is_bigendian:false,fields:[{name:'x',offset:0,datatype:7,count:1}],data:new Array(12).fill(0)});
  const report=await pending;assert.deepEqual(report.results.map(r=>r.status),['FAIL','PASS']);assert.match(report.results[0].error,/requires scalar y/);assert.equal(client.subscriptions.size,0);
});
await probe('stale samples and incomplete TF are FAIL with concrete evidence',async()=>{
  const client=fake();const pending=runProjectChecks(client,base([topic]),['stream']);await new Promise(resolve=>setTimeout(resolve,1));client.emit('/fresh',{header:{stamp:{sec:1,nanosec:0}},position:[0]});let report=await pending;assert.equal(report.results[0].status,'FAIL');assert.match(report.results[0].error,/时间|age/);
  report=await runProjectChecks(client,base([{id:'tf',label:'TF',kind:'tf',target:'world',sources:['tcp'],timeoutMs:10}]),['tf']);assert.equal(report.results[0].status,'FAIL');assert.deepEqual(report.results[0].evidence.missing,['tcp']);assert.equal(client.subscriptions.size,0);
});
await probe('TF uses project-specific dynamic and static topics',async()=>{
  const client=fake();const spec={id:'tf',label:'Demo TF',kind:'tf',target:'demo_world',sources:['demo_tcp'],timeoutMs:200};
  const pending=runProjectChecks(client,base([spec],{view:{scene:{tf:'/demo/tf',tfStatic:'/demo/tf_static'}}}),['tf']);await new Promise(resolve=>setTimeout(resolve,2));
  assert.deepEqual([...client.subscriptions.keys()].sort(),['/demo/tf','/demo/tf_static']);
  client.emit('/demo/tf_static',{transforms:[{header:{frame_id:'demo_world',stamp:{sec:0,nanosec:0}},child_frame_id:'demo_tcp',transform:{translation:{x:0,y:0,z:1},rotation:{x:0,y:0,z:0,w:1}}}]});
  const report=await pending;assert.equal(report.results[0].status,'PASS');assert.deepEqual(report.results[0].evidence.missing,[]);assert.equal(client.subscriptions.size,0);
});
await probe('action cancellation errors and terminal timeout remain FAIL after successful cleanup',async()=>{
  for(const mode of ['throws','timeout','raced-success']){
    const control=new AbortController();let resultSignal,resolveResult,cancelled=0;
    const client=fake({sendGoal:(name,type,goal,options)=>{resultSignal=options.signal;return {id:'cancel-probe',result:new Promise(resolve=>{resolveResult=resolve;}),cancel:()=>{cancelled++;if(mode==='throws')throw new Error('cancel transport failed');if(mode==='raced-success')resolveResult({status:4,successful:true,value:{degraded:false,snapshot_id:'race'}});}};}});
    const spec={id:'act',label:'Action',kind:'action',interactive:true,timeoutMs:1000,cleanup:[{name:'/cleanup',type:'std_srvs/srv/Trigger',expected:{success:true}}],action:{name:'/act',type:'example_interfaces/action/Fibonacci',goal:{order:2}}};
    const pending=runProjectChecks(client,base([spec]),['act'],{signal:control.signal});await new Promise(resolve=>setTimeout(resolve,2));control.abort();const report=await pending;
    assert.equal(cancelled,1);assert.equal(report.results[0].status,'FAIL',mode);assert.equal(report.results[0].evidence.cancelConfirmed,false);assert.equal(client.calls.at(-1).name,'/cleanup');assert.equal(resultSignal.aborted,true);assert.match(report.results[0].error,/取消|CANCELED/);
    if(mode==='raced-success'){assert.equal(report.results[0].evidence.terminalStatus,4);assert.equal(report.results[0].evidence.result.snapshot_id,'race');}
  }
});
await probe('recorded timestamps are explicit and never silently relax live freshness',async()=>{
  for(const timestampMode of ['live','recorded']){
    const client=fake();const spec={id:'archive',label:'Archived frame',kind:'topic',timeoutMs:200,topic:{name:'/recorded',type:'sensor_msgs/msg/JointState',minSamples:1,timestampMode}};
    const pending=runProjectChecks(client,base([spec]),['archive']);await new Promise(resolve=>setTimeout(resolve,1));
    client.emit('/recorded',{header:{stamp:{sec:1,nanosec:0}},position:[0]});
    const report=await pending;assert.equal(report.results[0].status,timestampMode==='live'?'FAIL':'PASS');
    if(timestampMode==='recorded'){assert.equal(report.results[0].evidence.timestampMode,'recorded');assert.equal(report.results[0].evidence.liveFreshnessChecked,false);}
    assert.equal(client.subscriptions.size,0);
  }
  assert.throws(()=>validateProject(base([{...topic,topic:{...topic.topic,timestampMode:'pretend-live'}}])),/timestampMode/);
});
await probe('recorded loop rewind collects a new sample window',async()=>{
  const client=fake();const spec={id:'loop',label:'Loop',kind:'topic',timeoutMs:200,topic:{name:'/loop',type:'sensor_msgs/msg/JointState',timestampMode:'recorded',minSamples:3,minHz:0.5}};
  const pending=runProjectChecks(client,base([spec]),['loop']);await new Promise(resolve=>setTimeout(resolve,1));
  for(const sec of [100,101,99,100,101])client.emit('/loop',{header:{stamp:{sec,nanosec:0}},position:[0]});
  const report=await pending;assert.equal(report.results[0].status,'PASS');assert.equal(report.results[0].evidence.epochResets,1);assert.equal(report.results[0].evidence.samples,3);assert.equal(report.results[0].evidence.sourceHz,1);assert.equal(client.subscriptions.size,0);
});
console.log(`Audit boundary probe: ${checks} passed`);
