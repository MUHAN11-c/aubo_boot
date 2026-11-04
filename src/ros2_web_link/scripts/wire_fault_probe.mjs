/** Protocol fault injection on OWN ephemeral ws servers; no ROS or browser.
 * These are deliberately small packets and budgets, not a throughput benchmark.
 */
import assert from 'node:assert/strict';
import {once} from 'node:events';
import {WebSocketServer} from 'ws';
import {encode} from 'cbor2';
import {RosClient} from '../dist/sdk/index.js';
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
const until=async predicate=>{const end=Date.now()+2500;while(!predicate()&&Date.now()<end)await sleep(5);assert.ok(predicate(),'condition timed out');};
const budget={maxMessageBytes:4096,maxFragmentBytes:8192,maxFragmentMessages:4,maxFragmentsPerMessage:16,fragmentTimeoutMs:80,maxSendBufferedBytes:8192,maxCborDepth:32};
let passed=0;
async function fixture(name,body,limits={},options={}){
  if(process.env.WEB_LINK_WIRE_CASE&&!name.includes(process.env.WEB_LINK_WIRE_CASE))return;
  const server=new WebSocketServer({host:'127.0.0.1',port:0});await once(server,'listening');
  const sockets=[];server.on('connection',s=>{sockets.push(s);s.on('error',()=>{});});
  const NativeSocket=globalThis.WebSocket;let native;
  globalThis.WebSocket=class extends NativeSocket{constructor(...args){super(...args);native=this;}};
  const client=new RosClient({url:`ws://127.0.0.1:${server.address().port}`,reconnect:false,transportLimits:{...budget,...limits},...options});
  try{await client.connect();await until(()=>sockets.length>0);await body(client,sockets,server,native);passed++;console.log(`PASS ${name}`);}
  finally{client.dispose();globalThis.WebSocket=NativeSocket;for(const s of server.clients)s.terminate();await new Promise(r=>server.close(r));}
}
const fragment=(id,num,total,data)=>({op:'fragment',id,num,total,data});
const send=(socket,message)=>socket.send(JSON.stringify(message));
const text=data=>JSON.stringify({op:'publish',topic:'/probe',msg:{data}});
async function expectFault(client,socket,packets,code){
  const statuses=[];client.onStatus(s=>statuses.push(s));
  const result=client.callService('/pending','std_srvs/srv/Trigger',{}, {timeoutMs:1500}).then(()=>null,e=>e);
  for(const packet of packets)typeof packet==='string'?socket.send(packet):send(socket,packet);
  const error=await result;assert.equal(error?.code,code);assert.equal(client.connected,false);
  assert.ok(statuses.some(s=>s.code===code));assert.equal(client.stats().pending,0);
  const stats=client.transportStats();assert.equal(stats.fragmentMessages,0);assert.equal(stats.fragmentBytes,0);
}

await fixture('missing fragment expires and settles in-flight service',async(c,[s])=>{
  await expectFault(c,s,[fragment('lost',0,2,'{"op":')],'FRAGMENT_TIMEOUT');
});
await fixture('out-of-order and identical duplicates deliver one Unicode message',async(c,[s])=>{
  const values=[];c.subscribe('/probe','std_msgs/msg/String',m=>values.push(m.data));
  const payload=text('桃子 / 🌳'),chunks=[payload.slice(0,15),payload.slice(15,32),payload.slice(32)];
  for(const i of [2,0,0,1])send(s,fragment('valid',i,3,chunks[i]));
  await until(()=>values.length>0);assert.deepEqual(values,['桃子 / 🌳']);
  assert.equal(c.transportStats().fragmentBytes,0);assert.equal(c.connected,true);
});
for(const [name,num,total] of [['negative index',-1,2],['fractional index',.5,2],['huge total',0,1e9]]){
  await fixture(`invalid fragment ${name} closes session`,(c,[s])=>expectFault(c,s,[fragment('bad',num,total,'x')],'INVALID_FRAGMENT'));
}
await fixture('conflicting duplicate fails rather than corrupting assembly',(c,[s])=>expectFault(c,s,[fragment('dup',0,2,'a'),fragment('dup',0,2,'b')],'INVALID_FRAGMENT'));
await fixture('concurrent fragment IDs bounded',(c,[s])=>expectFault(c,s,[fragment('a',0,2,'a'),fragment('b',0,2,'b')],'FRAGMENT_LIMIT'),{maxFragmentMessages:1});
await fixture('aggregate fragment bytes bounded',(c,[s])=>expectFault(c,s,[fragment('a',0,2,'a'.repeat(80)),fragment('b',0,2,'b'.repeat(80))],'FRAGMENT_LIMIT'),{maxFragmentBytes:128});
await fixture('assembled message bytes bounded',(c,[s])=>expectFault(c,s,[fragment('a',0,2,'a'.repeat(2200)),fragment('a',1,2,'b'.repeat(2200))],'MESSAGE_TOO_LARGE'));
await fixture('oversized raw frame rejected before JSON parsing',(c,[s])=>expectFault(c,s,['x'.repeat(4097)],'MESSAGE_TOO_LARGE'));
await fixture('UTF-8 byte limit counts Chinese text',(c,[s])=>expectFault(c,s,[text('桃'.repeat(1500))],'MESSAGE_TOO_LARGE'));
await fixture('nested fragmentation cannot bypass bounds',(c,[s])=>expectFault(c,s,[fragment('outer',0,1,JSON.stringify(fragment('inner',0,2,'x')))],'INVALID_FRAGMENT'));
await fixture('malformed assembled JSON closes session',(c,[s])=>expectFault(c,s,[fragment('bad-json',0,1,'not-json')],'INVALID_FRAME'));
await fixture('upstream CBOR typed arrays preserved',async(c,[s])=>{
  const values=[];c.subscribe('/probe','sensor_msgs/msg/Image',m=>values.push(m));
  s.send(encode({op:'publish',topic:'/probe',msg:{data:new Uint8Array([1,2,255]),values:new Float32Array([.5,2])}}));
  await until(()=>values.length>0);assert.deepEqual([...values[0].data],[1,2,255]);assert.ok(values[0].data instanceof Uint8Array);assert.ok(values[0].values instanceof Float32Array);
});
await fixture('CBOR nesting limit enforced',async(c,[s])=>{
  const result=c.callService('/pending','std_srvs/srv/Trigger',{}, {timeoutMs:1500}).catch(e=>e);
  let node=0;for(let i=0;i<50;i++)node=[node];s.send(encode({op:'publish',topic:'/probe',msg:node}));
  assert.equal((await result).code,'INVALID_FRAME');assert.equal(c.connected,false);
});
await fixture('protocol PNG wrapper explicitly rejected',(c,[s])=>expectFault(c,s,[{op:'png',data:'AA=='}],'UNSUPPORTED_ENCODING'));
await fixture('send congestion rejected without replay or disconnect',async(c,[s])=>{
  s.pause();const pub=c.publisher('/probe','std_msgs/msg/String');let sent=0,code;
  for(let i=0;i<20;i++){try{pub.publish({data:'x'.repeat(100000)});sent++;}catch(e){code=e.code;break;}}
  assert.equal(code,'SEND_BUFFER_FULL');assert.ok(sent<20);assert.equal(c.connected,true);s.resume();
  await sleep(20);s.on('message',data=>{const m=JSON.parse(String(data));if(m.op==='call_service')send(s,{op:'service_response',id:m.id,result:true,values:{success:true}});});
  assert.deepEqual(await c.callService('/ok','std_srvs/srv/Trigger'),{success:true});pub.dispose();
},{maxMessageBytes:200000,maxSendBufferedBytes:250000});
await fixture('dispose clears partial bytes and timeout',async(c,[s])=>{
  send(s,fragment('pending',0,2,'a'));await until(()=>c.transportStats().fragmentMessages===1);c.dispose();
  assert.equal(c.transportStats().fragmentBytes,0);await sleep(120);assert.equal(c.state,'disposed');assert.equal(c.stats().listeners,0);
});
await fixture('refused subscribe and advertise leave no orphan resources',async c=>{
  const baseline=c.stats();assert.throws(()=>c.subscribe('/probe','std_msgs/msg/String',()=>{}),{code:'SEND_BUFFER_FULL'});
  assert.equal(c.stats().subscriptions,0);assert.equal(c.stats().listeners,baseline.listeners);
  assert.throws(()=>c.publisher('/probe','std_msgs/msg/String'),{code:'SEND_BUFFER_FULL'});assert.equal(c.stats().publishers,0);
},{maxSendBufferedBytes:64});
await fixture('consumer callback failure does not disconnect or hide later consumers',async(c,[s])=>{
  const statuses=[],values=[];c.onStatus(x=>statuses.push(x));
  c.subscribe('/probe','std_msgs/msg/String',()=>{throw Error('consumer failed');});
  c.subscribe('/probe','std_msgs/msg/String',m=>values.push(m.data));send(s,JSON.parse(text('still delivered')));
  await until(()=>values.length>0);assert.equal(c.connected,true);assert.ok(statuses.some(s=>s.code==='CALLBACK_ERROR'));
});
await fixture('publisher disposal closes congested session and always releases resource',async c=>{
  const pub=c.publisher('/'+ 'x'.repeat(12000),'std_msgs/msg/String');pub.publish({data:'a'});
  assert.doesNotThrow(()=>pub.dispose());assert.equal(c.stats().publishers,0);assert.equal(c.connected,false);
},{maxMessageBytes:32000,maxSendBufferedBytes:50000});
await fixture('unsubscribe closes congested session and always removes local listener',async c=>{
  const pub=c.publisher('/fill','std_msgs/msg/String');pub.publish({data:'x'.repeat(24000)});
  const off=c.subscribe('/'+ 'x'.repeat(12000),'std_msgs/msg/String',()=>{});
  assert.doesNotThrow(off);assert.equal(c.stats().subscriptions,0);assert.equal(c.connected,false);pub.dispose();
},{maxMessageBytes:32000,maxSendBufferedBytes:50000});
await fixture('lost fragment reconnects subscriptions without replaying service',async(c,sockets,server)=>{
  let serviceCalls=0;const values=[];
  sockets[0].on('message',data=>{const m=JSON.parse(String(data));if(m.op==='call_service')serviceCalls++;});
  server.on('connection',s=>s.on('message',data=>{if(JSON.parse(String(data)).op==='subscribe')send(s,JSON.parse(text('new session')));}));
  c.subscribe('/probe','std_msgs/msg/String',m=>values.push(m.data));
  await expectFault(c,sockets[0],[fragment('lost-again',0,2,'a')],'FRAGMENT_TIMEOUT');
  await until(()=>c.connected&&values.includes('new session'));assert.equal(serviceCalls,1);await sleep(100);assert.equal(c.connected,true);
},{},{reconnect:true,retryMs:20});
await fixture('deterministic wire violations do not reconnect in a loop',async(c,[s],server)=>{
  let reconnects=0;server.on('connection',()=>reconnects++);await expectFault(c,s,['x'.repeat(4097)],'MESSAGE_TOO_LARGE');
  await sleep(100);assert.equal(reconnects,0);assert.equal(c.state,'disconnected');
},{},{reconnect:true,retryMs:20});
for(const [label,value] of [['null',null],['undefined',undefined],['string','text'],['unprintable',{toString(){throw Error('bad conversion');}}]]){
  await fixture(`consumer callback throwing ${label} remains isolated`,async(c,[s])=>{
    let delivered=0;const statuses=[];c.onStatus(s=>statuses.push(s));
    c.subscribe('/probe','std_msgs/msg/String',()=>{throw value;});c.subscribe('/probe','std_msgs/msg/String',()=>delivered++);
    send(s,JSON.parse(text('healthy consumer')));await until(()=>delivered===1);assert.equal(c.connected,true);assert.ok(statuses.some(s=>s.code==='CALLBACK_ERROR'));
  });
}
await fixture('expired fragments rejected even before delayed timers execute',async(c,sockets,server,native)=>{
  let delivered=0;c.subscribe('/probe','std_msgs/msg/String',()=>delivered++);
  const payload=text('too late'),receive=m=>native.dispatchEvent(new MessageEvent('message',{data:JSON.stringify(m)}));
  receive(fragment('late',0,2,payload.slice(0,10)));
  const end=performance.now()+20;while(performance.now()<end){} // Reproduce a busy main thread; timers cannot run yet.
  receive(fragment('late',1,2,payload.slice(10)));
  assert.equal(delivered,0);assert.equal(c.connected,false);assert.equal(c.transportStats().lastError.code,'FRAGMENT_TIMEOUT');
},{fragmentTimeoutMs:5});
await fixture('expired service response fails before a delayed request timer executes',async(c,sockets,server,native)=>{
  const original=native.send,frames=[],baseline=c.stats().listeners;
  native.send=function(data){frames.push(JSON.parse(data));return original.call(this,data);};
  try{
    const outcome=c.callService('/late','std_srvs/srv/Trigger',{}, {timeoutMs:5}).then(value=>({value}),error=>({error}));
    const request=frames.find(m=>m.op==='call_service');assert.ok(request,'observe the actual outbound request ID');
    const end=performance.now()+20;while(performance.now()<end){} // Model a GL/decode long task; timers cannot run.
    native.dispatchEvent(new MessageEvent('message',{data:JSON.stringify({op:'service_response',id:request.id,result:true,values:{success:true}})}));
    assert.equal((await outcome).error?.code,'TIMEOUT');assert.equal(c.connected,true);
    assert.equal(c.stats().pending,0);assert.equal(c.stats().listeners,baseline);
    assert.equal(frames.filter(m=>m.op==='call_service').length,1,'no retry after the local deadline');
  }finally{native.send=original;}
});
await fixture('expired action feedback is not delivered and terminal result remains unknown',async(c,sockets,server,native)=>{
  const original=native.send,frames=[];let feedback=0;
  native.send=function(data){frames.push(JSON.parse(data));return original.call(this,data);};
  try{
    const goal=c.sendGoal('/late_action','example_interfaces/action/Fibonacci',{order:2},{timeoutMs:5,onFeedback:()=>feedback++});
    const outcome=goal.result.then(value=>({value}),error=>({error}));
    const request=frames.find(m=>m.op==='send_action_goal');assert.ok(request);
    const end=performance.now()+20;while(performance.now()<end){}
    native.dispatchEvent(new MessageEvent('message',{data:JSON.stringify({op:'action_feedback',id:request.id,values:{sequence:[0,1]}})}));
    assert.equal(feedback,0,'feedback received after the local deadline must not update a finished wait');
    assert.equal((await outcome).error?.code,'TIMEOUT');assert.equal(c.stats().pending,0);
    native.dispatchEvent(new MessageEvent('message',{data:JSON.stringify({op:'action_result',id:request.id,status:4,result:true,values:{sequence:[0,1,1]}})}));
    assert.equal(c.transportStats().ignoredResponses,1);assert.equal(c.connected,true);
    assert.equal(frames.filter(m=>m.op==='cancel_action_goal').length,0,'local timeout does not claim remote cancellation');
  }finally{native.send=original;}
});
await fixture('response processing crossing the request deadline cannot finish successfully',async(c,sockets,server,native)=>{
  const original=native.send,descriptor=Object.getOwnPropertyDescriptor(performance,'now');let request;
  native.send=function(data){request=JSON.parse(data);return original.call(this,data);};
  try{
    const outcome=c.callService('/processing','std_srvs/srv/Trigger',{}, {timeoutMs:1000}).then(value=>({value}),error=>({error}));
    assert.equal(request?.op,'call_service');
    const event=new MessageEvent('message',{data:JSON.stringify({op:'service_response',id:request.id,result:true,values:{success:true}})});
    const arrived=performance.now();let first=true;
    // The response enters before its deadline, but completion is observed after
    // it. A controlled monotonic clock avoids multi-megabyte timing fixtures.
    Object.defineProperty(performance,'now',{configurable:true,value:()=>{if(first){first=false;return arrived;}return arrived+2000;}});
    native.dispatchEvent(event);
    assert.equal((await outcome).error?.code,'TIMEOUT');assert.equal(c.stats().pending,0);
  }finally{if(descriptor)Object.defineProperty(performance,'now',descriptor);else delete performance.now;native.send=original;}
});
await fixture('expired correlated bridge status respects the local request deadline',async(c,sockets,server,native)=>{
  const original=native.send;let request;
  native.send=function(data){request=JSON.parse(data);return original.call(this,data);};
  try{
    const outcome=c.callService('/status','std_srvs/srv/Trigger',{}, {timeoutMs:5}).catch(error=>error);
    const end=performance.now()+20;while(performance.now()<end){}
    native.dispatchEvent(new MessageEvent('message',{data:JSON.stringify({op:'status',id:request.id,level:'error',msg:'late bridge error'})}));
    assert.equal((await outcome).code,'TIMEOUT');assert.equal(c.stats().pending,0);assert.equal(c.connected,true);
  }finally{native.send=original;}
});
await fixture('expired connection open cannot beat a delayed connect timer',async(c,sockets,server)=>{
  const Native=globalThis.WebSocket;
  // Preserve a real handshake/socket, delaying only delivery of its public
  // open callback to model a busy browser thread before SDK observation.
  globalThis.WebSocket=class extends Native{
    set onopen(callback){super.onopen=callback?event=>{const end=performance.now()+40;while(performance.now()<end){}callback.call(this,event);}:null;}
    get onopen(){return super.onopen;}
  };
  const late=new RosClient({url:c.options.url,reconnect:false,connectTimeoutMs:20});
  try{
    const outcome=await late.connect().then(()=>null,error=>error);
    assert.equal(outcome?.code,'DISCONNECTED');assert.equal(late.connected,false);assert.equal(late.stats().pending,0);
  }finally{late.dispose();globalThis.WebSocket=Native;}
});
await fixture('maximum valid request duration does not overflow after deadline rounding',async c=>{
  const descriptor=Object.getOwnPropertyDescriptor(performance,'now'),controller=new AbortController();let outcome='pending';
  try{
    // IEEE-754: (1.3 + 2147483647) - 1.3 rounds slightly above the
    // maximum timer delay. An unchecked ceil turns it into a 1ms timer.
    Object.defineProperty(performance,'now',{configurable:true,value:()=>1.3});
    const result=c.callService('/long_wait','std_srvs/srv/Trigger',{}, {timeoutMs:2147483647,signal:controller.signal})
      .then(()=>{outcome='success';},error=>{outcome=error.code;});
    if(descriptor)Object.defineProperty(performance,'now',descriptor);else delete performance.now;
    await sleep(20);assert.equal(outcome,'pending','a valid multi-day wait must not turn into an immediate timeout');
    controller.abort();await result;assert.equal(outcome,'WAIT_ABORTED');assert.equal(c.stats().pending,0);
  }finally{controller.abort();if(descriptor)Object.defineProperty(performance,'now',descriptor);else delete performance.now;}
});
await fixture('reentrant disconnect during connecting settles attempt',async(c,sockets,server)=>{
  c.disconnect();c.onState(state=>{if(state==='connecting')c.disconnect();});
  let outcome='pending';void c.connect().then(()=>{outcome='resolved';},e=>{outcome=e.code;});
  await sleep(80);assert.equal(outcome,'DISCONNECTED');assert.equal(c.state,'disconnected');
},{},{connectTimeoutMs:25});
await fixture('reentrant disconnect during reconnecting prevents retry',async(c,[s],server)=>{
  let reconnects=0;server.on('connection',()=>reconnects++);c.onState(state=>{if(state==='reconnecting')c.disconnect();});
  s.terminate();await sleep(120);assert.equal(reconnects,0);assert.equal(c.state,'disconnected');
},{},{reconnect:true,retryMs:20});
await fixture('malformed routing field is a terminal protocol error',(c,[s])=>expectFault(c,s,[{op:'publish',topic:{toString:null,valueOf:null},msg:{}}],'INVALID_FRAME'),{},{reconnect:true,retryMs:20});
await fixture('unknown or late response IDs cannot reach topic or lifecycle events',async(c,[s])=>{
  let delivered=0;c.subscribe('/probe','std_msgs/msg/String',()=>delivered++);
  for(const id of ['/probe','close','error','connection','web-link:expired'])send(s,{op:'service_response',id,result:true,values:{}});
  await sleep(40);assert.equal(c.connected,true);assert.equal(delivered,0);assert.equal(c.transportStats().ignoredResponses,5);
  s.on('message',data=>{const m=JSON.parse(String(data));if(m.op==='call_service')send(s,{op:'service_response',id:m.id,result:true,values:{success:true}});});
  assert.deepEqual(await c.callService('/ok','std_srvs/srv/Trigger'),{success:true});
},{},{reconnect:true,retryMs:20});
await fixture('publish cannot route into a lifecycle event',(c,[s])=>expectFault(c,s,[{op:'publish',topic:'close',msg:{}}],'INVALID_FRAME'),{},{reconnect:true,retryMs:20});
await fixture('service and action share a bounded admission budget without sending overflow',async(c,[s])=>{
  const frames=[],baseline=c.stats().listeners;
  s.on('message',data=>frames.push(JSON.parse(String(data))));
  const first=c.callService('/held','std_srvs/srv/Trigger',{}, {timeoutMs:1500}).catch(error=>error);
  const goal=c.sendGoal('/held_action','example_interfaces/action/Fibonacci',{order:2},{timeoutMs:1500});
  const result=goal.result.catch(error=>error);
  const rejected=await c.callService('/overflow','std_srvs/srv/Trigger',{}, {timeoutMs:35}).catch(error=>error);
  assert.equal(rejected?.code,'REQUEST_LIMIT','excess service must fail locally before a timeout or send');
  assert.throws(()=>c.sendGoal('/overflow_action','example_interfaces/action/Fibonacci',{order:2}),{code:'REQUEST_LIMIT'});
  await until(()=>frames.length===2);assert.equal(c.stats().pending,2);assert.equal(c.stats().pendingLimit,2);
  send(s,{op:'service_response',id:frames[0].id,result:true,values:{success:true}});
  assert.deepEqual(await first,{success:true});assert.equal(c.stats().pending,1);
  send(s,{op:'action_feedback',id:goal.id,values:{sequence:[0,1]}});
  goal.cancel();await until(()=>frames.some(m=>m.op==='cancel_action_goal'));
  assert.equal(c.stats().pending,1,'feedback and a cancel request are not a terminal result');
  send(s,{op:'action_result',id:goal.id,status:5,result:true,values:{sequence:[0,1]}});
  assert.equal((await result).status,5);assert.equal(c.stats().pending,0);assert.equal(c.stats().listeners,baseline);
  assert.ok(frames.every(m=>!String(m.service??m.action).includes('overflow')),'overflow must never reach the server');
},{},{maxPendingRequests:2});
await fixture('default admission limit bounds a synchronous burst and abort releases every slot',async(c,[s])=>{
  const frames=[],controllers=[],waits=[],baseline=c.stats().listeners;
  s.on('message',data=>frames.push(JSON.parse(String(data))));
  for(let i=0;i<256;i++){
    const controller=new AbortController();controllers.push(controller);
    waits.push(c.callService('/burst','std_srvs/srv/Trigger',{}, {timeoutMs:1500,signal:controller.signal}).catch(error=>error));
  }
  const rejected=await c.callService('/overflow','std_srvs/srv/Trigger',{}, {timeoutMs:35}).catch(error=>error);
  assert.equal(rejected?.code,'REQUEST_LIMIT');assert.equal(c.stats().pending,256);assert.equal(c.stats().pendingLimit,256);
  await until(()=>frames.length===256);controllers.forEach(controller=>controller.abort());
  assert.ok((await Promise.all(waits)).every(error=>error.code==='WAIT_ABORTED'));
  assert.equal(c.stats().pending,0);assert.equal(c.stats().listeners,baseline);
  s.on('message',data=>{const m=JSON.parse(String(data));if(m.service==='/after')send(s,{op:'service_response',id:m.id,result:true,values:{success:true}});});
  assert.deepEqual(await c.callService('/after','std_srvs/srv/Trigger'),{success:true});
  assert.equal(frames.filter(m=>m.service==='/overflow').length,0);
},{maxSendBufferedBytes:1000000});
await fixture('failed response status timeout abort and rejected send release admission slots',async(c,[s])=>{
  const baseline=c.stats().listeners;
  s.on('message',data=>{
    const m=JSON.parse(String(data));
    if(m.service==='/failed')send(s,{op:'service_response',id:m.id,result:false,values:'failed'});
    if(m.service==='/status')send(s,{op:'status',id:m.id,level:'error',msg:'denied'});
    if(m.service==='/ok')send(s,{op:'service_response',id:m.id,result:true,values:{success:true}});
  });
  for(const [service,args,options,expected] of [
    ['/failed',{}, {},'SERVICE_FAILED'],['/status',{}, {},'BRIDGE_ERROR'],
    ['/no_reply',{}, {timeoutMs:20},'TIMEOUT'],
    ['/already_aborted',{}, {signal:AbortSignal.abort()},'WAIT_ABORTED'],
    ['/oversized',{data:'x'.repeat(5000)}, {},'MESSAGE_TOO_LARGE'],
  ]){
    const error=await c.callService(service,'std_srvs/srv/Trigger',args,options).catch(error=>error);
    assert.equal(error?.code,expected);assert.equal(c.stats().pending,0);assert.equal(c.stats().listeners,baseline);
    assert.deepEqual(await c.callService('/ok','std_srvs/srv/Trigger'),{success:true});
  }
},{},{maxPendingRequests:1});
await fixture('disconnect releases a full budget and reconnect never replays requests',async(c,sockets)=>{
  const frames=[];sockets[0].on('message',data=>frames.push(JSON.parse(String(data))));
  const old=c.callService('/old','std_srvs/srv/Trigger').catch(error=>error);
  await until(()=>frames.length===1);c.disconnect();assert.equal((await old).code,'DISCONNECTED');assert.equal(c.stats().pending,0);
  await c.connect();await until(()=>sockets.length===2);await sleep(20);assert.equal(c.stats().pending,0);
  sockets[1].on('message',data=>{const m=JSON.parse(String(data));frames.push(m);send(sockets[1],{op:'service_response',id:m.id,result:true,values:{success:true}});});
  assert.deepEqual(await c.callService('/new','std_srvs/srv/Trigger'),{success:true});
  assert.equal(frames.filter(m=>m.service==='/old').length,1);assert.equal(c.stats().pending,0);
},{},{maxPendingRequests:1});
await fixture('invalid admission limits are rejected before a socket can be created',async c=>{
  for(const maxPendingRequests of [0,-1,1.5,NaN,Infinity,4097,null,'2',true]){
    assert.throws(()=>new RosClient({url:c.options.url,maxPendingRequests}),TypeError);
  }
  for(const maxPendingRequests of [1,4096]){
    const other=new RosClient({url:c.options.url,maxPendingRequests});
    assert.equal(other.stats().pendingLimit,maxPendingRequests);other.dispose();
  }
});
await fixture('validated admission budget cannot be overwritten through public options',async c=>{
  for(const invalid of [NaN,Infinity,100000,2]){
    assert.throws(()=>{c.options.maxPendingRequests=invalid;},TypeError);
    assert.equal(c.options.maxPendingRequests,1);assert.equal(c.stats().pendingLimit,1);
  }
  assert.throws(()=>Object.defineProperty(c.options,'maxPendingRequests',{value:100000}),TypeError);
},{},{maxPendingRequests:1});
await fixture('reentrant request option getters cannot overbook the wait budget',async(c,[s])=>{
  const frames=[],controller=new AbortController();let reads=0,nested;
  s.on('message',data=>frames.push(JSON.parse(String(data))));
  const options={get timeoutMs(){
    if(++reads===2)nested=c.callService('/nested','std_srvs/srv/Trigger',{}, {timeoutMs:1500,signal:controller.signal}).catch(error=>error);
    return 35;
  }};
  const rejected=await c.callService('/outer','std_srvs/srv/Trigger',{}, options).catch(error=>error);
  assert.equal(rejected?.code,'REQUEST_LIMIT');assert.equal(c.stats().pending,1);
  await until(()=>frames.length===1);assert.equal(frames[0].service,'/nested');
  controller.abort();assert.equal((await nested).code,'WAIT_ABORTED');assert.equal(c.stats().pending,0);
},{},{maxPendingRequests:1});
await fixture('throwing signal options never retain an admission slot or listeners',async c=>{
  const baseline=c.stats().listeners;let reads=0;
  const error=await c.callService('/bad_signal','std_srvs/srv/Trigger',{}, {get signal(){if(++reads===1)throw Error('signal getter failed');return undefined;}}).catch(error=>error);
  assert.match(error.message,/signal getter failed/);assert.equal(c.stats().pending,0);assert.equal(c.stats().listeners,baseline);
  for(const signal of [new AbortController(),{},null,false]){
    await assert.rejects(c.callService('/bad_signal','std_srvs/srv/Trigger',{}, {signal}),TypeError);
    assert.equal(c.stats().pending,0);assert.equal(c.stats().listeners,baseline);
  }
},{},{maxPendingRequests:1});
await fixture('failed abort registration rolls back and throwing abort cleanup still settles',async c=>{
  const baseline=c.stats().listeners;
  const signal={aborted:false,addEventListener(){throw Error('abort registration failed');},removeEventListener(){}};
  await assert.rejects(c.callService('/registration','std_srvs/srv/Trigger',{}, {signal}),/abort registration failed/);
  assert.equal(c.stats().pending,0);assert.equal(c.stats().listeners,baseline);
  const cleanup={aborted:false,addEventListener(){},removeEventListener(){throw Error('abort cleanup failed');}};
  await assert.rejects(c.callService('/cleanup','std_srvs/srv/Trigger',{}, {signal:cleanup,timeoutMs:20}),{code:'TIMEOUT'});
  assert.equal(c.stats().pending,0);assert.equal(c.stats().listeners,baseline);
},{},{maxPendingRequests:1});
await fixture('invalid budget names and shapes rejected before creating sockets',async c=>{
  for(const limits of [null,[],{toString:1},{unknown:1},{maxMessageBytes:0},{maxCborDepth:Infinity}]){
    assert.throws(()=>new RosClient({url:c.options.url,transportLimits:limits}),TypeError);
  }
});
await fixture('non-Error state and action feedback exceptions remain isolated',async(c,[s])=>{
  const statuses=[];c.onStatus(m=>statuses.push(m));const off=c.onState(()=>{throw null;});
  s.on('message',data=>{const m=JSON.parse(String(data));if(m.op==='send_action_goal'){
    send(s,{op:'action_feedback',id:m.id,values:{sequence:[0,1]}});
    send(s,{op:'action_result',id:m.id,status:4,result:true,values:{sequence:[0,1,1]}});
  }});
  const result=await c.sendGoal('/action','example_interfaces/action/Fibonacci',{order:2},{onFeedback:()=>{throw undefined;}}).result;
  assert.equal(result.status,4);assert.equal(c.connected,true);off();assert.ok(statuses.filter(s=>s.code==='CALLBACK_ERROR').length>=2);
});
console.log(`PASS ${passed} wire fault groups; every temporary server and client closed`);
