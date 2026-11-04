/** Sustained real-project read-only load: independent SDK clients plus a paused
 * ws reader. Own no ROS server, camera or robot. Never keep raw frame history.
 */
import assert from 'node:assert/strict';
import {readFileSync,mkdirSync,writeFileSync} from 'node:fs';
import {writeFile} from 'node:fs/promises';
import {once} from 'node:events';
import path from 'node:path';
import WebSocket from 'ws';
import {RosClient} from '../dist/sdk/index.js';
import {decodeImage} from '../dist/media/image.js';
import {decodePointCloud} from '../dist/scene/pointcloud.js';
import {TfBuffer} from '../dist/scene/tf.js';
import {assertClientReleased,assertStreamProgress,assertSlowProgress,recordRpcResult,diagnosticWriteError} from './load_progress.mjs';
import {createMetadataBuffer,createServiceWireTrace} from './service_wire_trace.mjs';

const url=process.argv[2]??'ws://127.0.0.1:9290';
const seconds=Number(process.env.WEB_LINK_MULTI_SECONDS??300),count=Number(process.env.WEB_LINK_MULTI_CLIENTS??3);
const queryPeriodMs=Number(process.env.WEB_LINK_MULTI_QUERY_MS??2000);
const controlSelection=process.env.WEB_LINK_MULTI_SEPARATE_CONTROL??'0';
assert.ok(['0','1'].includes(controlSelection),'WEB_LINK_MULTI_SEPARATE_CONTROL must be 0 or 1');
const separateControl=controlSelection==='1';
assert.ok(Number.isInteger(seconds)&&seconds>=30&&seconds<=3600);assert.ok(Number.isInteger(count)&&count>=2&&count<=6);
assert.ok(Number.isInteger(queryPeriodMs)&&queryPeriodMs>=100&&queryPeriodMs<=10000);
const out=process.env.WEB_LINK_EVIDENCE??'/tmp/ros2-web-multi';mkdirSync(out,{recursive:true});
const project=process.env.WEB_LINK_PROJECT_CONFIG?JSON.parse(readFileSync(process.env.WEB_LINK_PROJECT_CONFIG,'utf8')):await fetch(process.argv[3]??'http://127.0.0.1:8288/config.json').then(r=>r.json()).then(c=>c.projects.find(p=>p.id===c.project));
const scene=project.view.scene,color=project.cases.find(c=>c.kind==='image'&&!c.topic.depthScale).topic,depth=project.cases.find(c=>c.kind==='image'&&c.topic.depthScale).topic,cloud=project.cases.find(c=>c.kind==='cloud').topic,tfCase=project.cases.find(c=>c.kind==='tf');
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
const p95=values=>values.length?[...values].sort((a,b)=>a-b)[Math.floor((values.length-1)*.95)]:null;
const histories=[],clients=[],errors=[],memory=[];let slow,slowStopping=false,slowBytes=0,pauseCycles=0,clockRewinds=0,lastClock,peakRssMiB=0,peakHeapMiB=0;
const slowProgress={frames:0,paused:false,closed:false,readyState:0,lastFrameAt:Date.now(),resumedAt:null,resumeBaseline:0,resumeChecks:0};
const fail=message=>{if(errors.length<40)errors.push(String(message));};
const rpcBuffer=process.env.WEB_LINK_TRACE_SERVICE||process.env.WEB_LINK_WAIT_SERVICE?createMetadataBuffer({write:text=>writeFileSync(path.join(out,'rpc-trace.jsonl'),text,{flag:'a'}),onError:message=>fail(`RPC trace: ${message}`)}):undefined;
const rpcTrace=(phase,session,values={})=>{rpcBuffer?.push({phase,wall:Date.now()/1000,slot:session.slot,generation:session.generation,...values});};
const NativeSocket=globalThis.WebSocket;
const wireTrace=process.env.WEB_LINK_TRACE_SERVICE||process.env.WEB_LINK_WAIT_SERVICE?createServiceWireTrace(NativeSocket,{
  endpoint:'/rosapi/nodes',write:line=>writeFileSync(path.join(out,'service-wire.jsonl'),line,{flag:'a'}),
  onError:message=>fail(message),
}):undefined;
if(wireTrace)globalThis.WebSocket=wireTrace.WebSocket;
// At most one asynchronous flag write. Drain it after SDK disposal so a late
// file error cannot arrive after the final verdict has already been published.
let pendingStack;
const requestStack=session=>{if(pendingStack)return;pendingStack=Promise.resolve().then(()=>writeFile(path.join(out,'stack.request'),JSON.stringify({wall:Date.now()/1000,slot:session.slot,generation:session.generation}))).catch(error=>fail(diagnosticWriteError(()=>{throw error;}))).finally(()=>{pendingStack=undefined;});};
const drainStack=async()=>{await pendingStack;};

async function startClient(slot,generation){
  const client=new RosClient({url,reconnect:false});
  // Composition of existing public clients only. Both sockets still use the
  // same upstream rosbridge; this does not isolate DDS or its event loop.
  const control=separateControl?new RosClient({url,reconnect:false}):client;
  const session={slot,generation,client,control,tf:new TfBuffer(client,{tfTopic:scene.tf,tfStaticTopic:scene.tfStatic,clockTopic:'/clock'}),off:[],frames:{},rpc:[],rpcCalls:0,rpcFailed:0,stopped:false,startedAt:Date.now(),receivedAt:{},tfMissingAt:null,tfEverComplete:false,tfChecks:[]};
  clients[slot]=session; // Finally also owns a partially connected/subscribed generation.
  for(const [lane,link] of separateControl?[['data',client],['control',control]]:[['shared',client]]){
    link.onStatus(m=>{if(m.level==='error')fail(`client ${slot}.${generation} ${lane}: ${m.code??'BRIDGE'} ${m.msg}`);});
    let wasOnline=false;
    link.onState(state=>{if(state==='connected')wasOnline=true;else if(wasOnline&&!session.stopped)fail(`client ${slot}.${generation} ${lane}: unexpected ${state}`);});
    await (wireTrace?wireTrace.withSession({slot,generation},()=>link.connect()):link.connect());
  }
  session.tf.start();
  const observe=(key,spec,decoder)=>session.off.push(client.subscribe(spec.name,spec.type,message=>{
    try{decoder(message);session.frames[key]=(session.frames[key]??0)+1;session.receivedAt[key]=Date.now();}
    catch(e){fail(`client ${slot}.${generation} ${key}: ${e.message}`);}
  },{qos:spec.qos??'sensor',compression:'cbor',throttleMs:key==='cloud'?100:500}));
  observe('color',color,m=>{const image=decodeImage(m);assert.ok(image.width&&image.height);assert.ok(m.data instanceof Uint8Array);});
  observe('depth',depth,m=>{const image=decodeImage(m,{depthScale:depth.depthScale});assert.ok(image.width&&image.height);});
  observe('cloud',cloud,m=>{const points=decodePointCloud(m);assert.ok(points.count>0);});
  session.off.push(client.subscribe(scene.robotDescription,'std_msgs/msg/String',m=>{if(!m.data.includes('<robot'))fail('invalid URDF');session.frames.urdf=(session.frames.urdf??0)+1;},{qos:'latched'}));
  session.off.push(client.subscribe(scene.jointStates,'sensor_msgs/msg/JointState',m=>{if(m.name.length!==m.position.length)fail('invalid JointState');session.frames.joints=(session.frames.joints??0)+1;},{qos:'sensor'}));
  if(slot===0)session.off.push(client.subscribe('/clock','rosgraph_msgs/msg/Clock',m=>{const stamp=m.clock.sec+m.clock.nanosec/1e9;if(lastClock!==undefined&&stamp<lastClock)clockRewinds++;lastClock=stamp;},{qos:'sensor'}));
  return session;
}
function closeClient(session){
  if(session.stopped)return;session.stopped=true;
  for(const off of session.off.reverse())off();session.tf.dispose();session.client.dispose();
  if(session.control!==session.client)session.control.dispose();
  const clean=session.client.stats(),wire=session.client.transportStats();
  assertClientReleased(clean,wire);
  const controlCleanup=separateControl?session.control.stats():undefined;
  const controlWire=separateControl?session.control.transportStats():undefined;
  if(separateControl)assertClientReleased(controlCleanup,controlWire);
  histories.push({slot:session.slot,generation:session.generation,frames:session.frames,receivedAt:{...session.receivedAt},startedAt:session.startedAt,closedAt:Date.now(),tfChecks:session.tfChecks,rpcP95Ms:p95(session.rpc),rpcLatencySamples:session.rpc.length,rpcLatencyScope:'all successful calls',rpcCalls:session.rpcCalls,rpcFailed:session.rpcFailed,cleanup:clean,wire,controlCleanup,controlWire});
}
async function query(session){
  const begun=performance.now();
  rpcTrace('request',session);
  const slowTimer=process.env.WEB_LINK_TRACE_SERVICE?setTimeout(()=>requestStack(session),1000):undefined;
  let successful=false,elapsedMs=0;
  try{const nodes=await session.control.callService('/rosapi/nodes','rosapi_msgs/srv/Nodes',{}, {timeoutMs:3000});assert.ok(nodes.nodes.length);successful=true;elapsedMs=performance.now()-begun;rpcTrace('response',session,{elapsedMs});}
  catch(e){elapsedMs=performance.now()-begun;fail(`client ${session.slot}.${session.generation} query: ${e.code??e.message}`);rpcTrace('error',session,{elapsedMs,code:e.code??e.message});}
  finally{clearTimeout(slowTimer);recordRpcResult(session,elapsedMs,successful);}
  const missing=tfCase.sources.filter(source=>!session.tf.lookup(tfCase.target,source)),valid=missing.length===0;
  session.tfChecks.push({seconds:(Date.now()-session.startedAt)/1000,missing,frames:session.tf.frames(),stats:session.tf.stats()});if(session.tfChecks.length>12)session.tfChecks.shift();
  if(valid){session.tfMissingAt=null;session.tfEverComplete=true;}
  else if(session.tfMissingAt===null)session.tfMissingAt=Date.now();
  else {
    // Initial bag delay + discovery + the first common dynamic interval need
    // a separate warmup budget. Once complete, outages remain limited to 6s.
    const budget=session.tfEverComplete?6000:10000;
    if(Date.now()-session.tfMissingAt>budget)fail(`client ${session.slot}.${session.generation} TF unavailable for >${budget/1000}s`);
  }
}
let status='FAIL',failure;
try{
  for(let i=0;i<count;i++)await startClient(i,1);
  slow=new WebSocket(url,{maxPayload:16000000});slow.on('error',e=>fail(`slow reader: ${e.message}`));
  slow.on('close',code=>{slowProgress.closed=true;if(!slowStopping)fail(`slow reader closed unexpectedly: ${code}`);});
  slow.on('message',data=>{slowProgress.frames++;slowProgress.lastFrameAt=Date.now();slowBytes+=data.length;});
  let timer;try{await Promise.race([once(slow,'open'),new Promise((_,reject)=>{timer=setTimeout(()=>reject(Error('slow reader open timeout')),5000);})]);}finally{clearTimeout(timer);}
  for(const topic of [cloud,depth])slow.send(JSON.stringify({op:'subscribe',topic:topic.name,type:topic.type,compression:'cbor',queue_length:1,throttle_rate:0,qos:{history:'keep_last',depth:10,reliability:'reliable',durability:'volatile'}}));
  slowProgress.readyState=slow.readyState;slowProgress.lastFrameAt=Date.now();
  const begun=performance.now();let churn1=false,churn2=false,lastQuery=-queryPeriodMs/1000,lastSample=-1;
  while(performance.now()-begun<seconds*1000){
    const elapsed=(performance.now()-begun)/1000;
    const wantPause=elapsed%45>=10&&elapsed%45<25;
    if(wantPause!==slowProgress.paused){slowProgress.paused=wantPause;if(wantPause){slow.pause();pauseCycles++;}else{slowProgress.resumedAt=Date.now();slowProgress.resumeBaseline=slowProgress.frames;slow.resume();}}
    for(const session of clients)assertStreamProgress(session,Date.now());
    slowProgress.readyState=slow.readyState;assertSlowProgress(slowProgress,Date.now());
    if(elapsed>=seconds*.25&&!churn1){closeClient(clients[1]);await startClient(1,2);churn1=true;}
    if(elapsed>=seconds*.60&&!churn2){closeClient(clients[count-1]);await startClient(count-1,clients[count-1].generation+1);churn2=true;}
    if(elapsed-lastQuery>=queryPeriodMs/1000){await Promise.all(clients.map(query));lastQuery=elapsed;}
    if(elapsed-lastSample>=1){const m=process.memoryUsage(),rss=m.rss/1048576,heap=m.heapUsed/1048576;peakRssMiB=Math.max(peakRssMiB,rss);peakHeapMiB=Math.max(peakHeapMiB,heap);memory.push({seconds:Math.round(elapsed),rssMiB:rss,heapMiB:heap});if(memory.length>600)memory.shift();lastSample=elapsed;if(rss>768)throw Error('client RSS guard >768MiB');}
    if(errors.length)throw Error(errors.join('; '));await sleep(100);
  }
  if(slowProgress.paused){slowProgress.paused=false;slowProgress.resumedAt=Date.now();slowProgress.resumeBaseline=slowProgress.frames;slow.resume();}
  const resumeDeadline=Date.now()+8000;
  while(slowProgress.resumedAt!==null&&Date.now()<resumeDeadline){slowProgress.readyState=slow.readyState;assertSlowProgress(slowProgress,Date.now());await sleep(50);}
  slowProgress.readyState=slow.readyState;assertSlowProgress(slowProgress,Date.now());assert.equal(slowProgress.resumedAt,null,'last slow reader resume did not advance');
  const deadline=Date.now()+5000;
  while(Date.now()<deadline&&!clients.every(s=>tfCase.sources.every(source=>s.tf.lookup(tfCase.target,source))))await sleep(50);
  for(const s of clients){assertStreamProgress(s,Date.now());for(const key of ['color','depth','cloud','urdf','joints'])assert.ok(s.frames[key]>0,`client ${s.slot} missing ${key}`);assert.ok(tfCase.sources.every(source=>s.tf.lookup(tfCase.target,source)),`client ${s.slot} final TF incomplete`);assert.ok(p95(s.rpc)<500,`client ${s.slot} RPC p95 exceeds 500ms`);}
  const bagSeconds=Number(process.env.WEB_LINK_REPLAY_SECONDS??0),expectedRewinds=bagSeconds>0?Math.max(0,Math.floor(seconds/bagSeconds)-1):0;
  assert.ok(clockRewinds>=expectedRewinds,`expected >=${expectedRewinds} replay clock rewinds, got ${clockRewinds}`);
  assert.ok(slowProgress.frames>0&&pauseCycles>0);assert.equal(slowProgress.resumeChecks,pauseCycles);assert.deepEqual(errors,[]);status='PASS';
}catch(e){failure=e.message;if(process.env.WEB_LINK_TRACE_SERVICE||process.env.WEB_LINK_WAIT_SERVICE)await sleep(5000);throw e;}
finally{
  slowStopping=true;slow?.terminate();
  for(const session of clients)if(session){
    try{closeClient(session);}catch(error){status='FAIL';failure??=error.message;fail(`cleanup: ${error.message}`);}
  }
  globalThis.WebSocket=NativeSocket;
  await drainStack();
  wireTrace?.flush();
  rpcBuffer?.flush();
  if(errors.length){status='FAIL';failure??=errors.join('; ');}
  writeFileSync(path.join(out,'multi-client.json'),JSON.stringify({status,failure,url,project:project.id,seconds,queryPeriodMs,healthyClients:count,separateControl,socketsPerHealthyClient:separateControl?2:1,slowReader:{frames:slowProgress.frames,bytes:slowBytes,pauseCycles,resumeChecks:slowProgress.resumeChecks,closedBeforeCleanup:slowProgress.closed},clockRewinds,peakRssMiB,peakHeapMiB,memory,histories,errors,wireTrace:wireTrace?.stats(),rpcTrace:rpcBuffer?.stats()},null,2));
}
if(status!=='PASS')throw Error(failure??'client cleanup failed');
console.log(`PASS ${seconds}s, ${count} healthy clients + paused reader, ${clockRewinds} replay rewinds; re-created clients retain complete TF; all sessions released`);
