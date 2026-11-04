/** Actual audit engine, controlled SDK boundaries. No ROS / browser / hardware.
 * Observe which existing client receives I/O and whether cleanup survives loss
 * of the observation lane. The engine never owns these callers' connections.
 */
import assert from 'node:assert/strict';
const {runProjectChecks}=await import(process.env.WEB_LINK_AUDIT_CHANNELS_TARGET??new URL('../dist/audit/index.js',import.meta.url).href);
const tick=()=>new Promise(resolve=>setImmediate(resolve));
const project=(cases,extra={})=>({id:'channels',title:'Separate observation boundary',cases,...extra});
const stream={id:'stream',label:'Stream',kind:'topic',timeoutMs:1000,topic:{name:'/fresh',type:'sensor_msgs/msg/JointState',minSamples:1,requiredFields:['position[0]']}};
const service={id:'service',label:'Service',kind:'service',interactive:true,timeoutMs:1000,service:{name:'/read',type:'std_srvs/srv/Trigger',expected:{success:true}}};
function link(url='ws://same.invalid:9090'){
  const listeners=new Set(),subscriptions=new Map(),calls=[];
  const api={connected:true,options:{url},listeners,subscriptions,calls,disposed:false,
    onState(fn){listeners.add(fn);fn(api.connected?'connected':'disconnected');return()=>listeners.delete(fn);},
    subscribe(name,type,fn){subscriptions.set(name,fn);return()=>subscriptions.delete(name);},
    emit(name,value){subscriptions.get(name)?.(value,{receivedAt:Date.now()});},
    setState(state){api.connected=state==='connected';for(const fn of [...listeners])fn(state);},
    callService:async(name)=>{calls.push(name);return {success:true};},
    dispose(){api.disposed=true;},
  };return api;
}
function released(...clients){for(const c of clients){assert.equal(c.subscriptions.size,0);assert.equal(c.listeners.size,0);assert.equal(c.disposed,false,'audit must not dispose caller connection');}}
const checks={
  async sample(){
    const control=link(),data=link(),abort=new AbortController();
    const pending=runProjectChecks(control,project([stream]),['stream'],{observationClient:data,signal:abort.signal});void pending.catch(()=>{});
    try{await tick();assert.equal(control.subscriptions.size,0);assert.ok(data.subscriptions.has('/fresh'));data.emit('/fresh',{position:[1]});const report=await pending;assert.equal(report.totals.PASS,1);assert.equal(report.connectionMode,'separate');}
    finally{abort.abort();await pending;released(control,data);}
  },
  async tf(){
    const control=link(),data=link(),abort=new AbortController();
    const spec={id:'tf',label:'TF',kind:'tf',timeoutMs:1000,target:'world',sources:['camera']};
    const pending=runProjectChecks(control,project([spec]),['tf'],{observationClient:data,signal:abort.signal});void pending.catch(()=>{});
    try{await tick();assert.equal(control.subscriptions.size,0);assert.ok(data.subscriptions.has('/tf_static'));
      data.emit('/tf_static',{transforms:[{header:{frame_id:'world',stamp:{sec:1,nanosec:0}},child_frame_id:'camera',transform:{translation:{x:1,y:0,z:0},rotation:{x:0,y:0,z:0,w:1}}}]});assert.equal((await pending).totals.PASS,1);}
    finally{abort.abort();await pending;released(control,data);}
  },
  async service(){
    const control=link(),data=link();const report=await runProjectChecks(control,project([service]),['service'],{observationClient:data});
    assert.equal(report.totals.PASS,1);assert.deepEqual(control.calls,['/read']);assert.deepEqual(data.calls,[]);released(control,data);
  },
  async mockGuard(){
    const control=link(),data=link(),abort=new AbortController();
    const spec={...service,mockOnly:true};const p=project([spec],{mockGuard:{topic:'/robot_description',type:'std_msgs/msg/String',field:'data',includes:'mock_components/GenericSystem'}});
    const pending=runProjectChecks(control,p,['service'],{observationClient:data,signal:abort.signal});void pending.catch(()=>{});
    try{await tick();assert.ok(control.subscriptions.has('/robot_description'));assert.equal(data.subscriptions.size,0);control.emit('/robot_description',{data:'mock_components/GenericSystem'});assert.equal((await pending).totals.PASS,1);assert.deepEqual(control.calls,['/read']);}
    finally{abort.abort();await pending;released(control,data);}
  },
  async disconnectedData(){
    const control=link(),data=link(),abort=new AbortController();
    const pending=runProjectChecks(control,project([stream]),['stream'],{observationClient:data,signal:abort.signal});void pending.catch(()=>{});
    try{await tick();data.setState('disconnected');await tick();assert.equal(data.subscriptions.size,0);assert.equal(control.subscriptions.size,0);const report=await pending;assert.equal(report.totals.FAIL,1);assert.match(report.results[0].error,/连接|断/);assert.equal(control.connected,true);}
    finally{abort.abort();await pending;released(control,data);}
  },
  async cancelViaControl(){
    const control=link(),data=link(),abort=new AbortController();let terminal,cancelled=false;
    control.sendGoal=()=>({id:'goal',result:new Promise(resolve=>{terminal=resolve;}),cancel(){cancelled=true;terminal({status:5,successful:false,value:{}});}});
    const spec={id:'action',label:'Action',kind:'action',interactive:true,timeoutMs:1000,action:{name:'/task',type:'example_interfaces/action/Fibonacci',goal:{order:5}},cleanup:[{name:'/clean',type:'std_srvs/srv/Trigger',expected:{success:true}}]};
    const pending=runProjectChecks(control,project([spec]),['action'],{observationClient:data,signal:abort.signal});void pending.catch(()=>{});
    try{await tick();assert.equal(typeof terminal,'function');data.setState('disconnected');await tick();assert.equal(cancelled,true);const report=await pending;assert.equal(report.totals.FAIL,1);assert.equal(report.results[0].evidence.cancelConfirmed,true);assert.deepEqual(control.calls,['/clean']);assert.deepEqual(data.calls,[]);}
    finally{abort.abort();await pending;released(control,data);}
  },
  async restoreViaControl(){
    const control=link(),data=link(),writes=[];let value='original';
    control.parameters=()=>({get:async()=>({values:[{type:4,value}]}),set:async items=>{value=items[0].value;writes.push(value);if(writes.length===1)data.setState('disconnected');return {results:[{successful:true,reason:''}]};}});
    const spec={id:'parameter',label:'Parameter',kind:'parameterRoundtrip',interactive:true,timeoutMs:1000,parameter:{node:'/settings',name:'text',type:4,value:'changed'}};
    const report=await runProjectChecks(control,project([spec]),['parameter'],{observationClient:data});
    assert.equal(report.totals.FAIL,1);assert.equal(report.results[0].evidence.restoreConfirmed,true);assert.deepEqual(writes,['changed','original']);assert.equal(value,'original');released(control,data);
  },
  async differentGateway(){
    const control=link(),data=link('ws://other.invalid:9090');await assert.rejects(()=>runProjectChecks(control,project([service]),['service'],{observationClient:data}),/同一|same|URL/i);assert.equal(control.calls.length,0);released(control,data);
  },
  async sharedDefault(){
    const control=link();const report=await runProjectChecks(control,project([service]),['service']);assert.equal(report.totals.PASS,1);assert.deepEqual(control.calls,['/read']);assert.equal(report.connectionMode,'shared');released(control);
  },
};
let failures=0;
for(const [name,verify] of Object.entries(checks))try{await verify();console.log(`PASS ${name}`);}catch(error){failures++;console.error(`FAIL ${name}: ${error.message}`);}
assert.equal(failures,0,`${failures} audit channel boundaries failed`);
console.log(`PASS ${Object.keys(checks).length} audit control/observation routing and cleanup groups`);
