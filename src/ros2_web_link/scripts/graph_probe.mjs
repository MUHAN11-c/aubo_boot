/** Explicit graph contracts against the real collector; no ROS or renderer. */
import assert from 'node:assert/strict';
import {discoverGraph,graphElements} from '../dist/graph/model.js';

let groups=0;
async function check(name,run){await run();groups++;console.log(`PASS ${name}`);}
function fixture({nodes=['/lab/source','/lab/sink'],topics=['/lab/data'],details={},delay=0,fail=[],types=['std_msgs/msg/String']}={}){
  const listeners=new Set(),calls=[];let active=0,peak=0,detailActive=0,detailPeak=0;
  const main={topics:{topics,types},services:{services:['/lab/reset']},nodes:{nodes},action_servers:{action_servers:['/lab/task']}};
  const state={calls,listeners,get peak(){return peak;},get detailPeak(){return detailPeak;},get active(){return active;}};
  const client={connected:true,onState(fn){listeners.add(fn);fn('connected');return()=>listeners.delete(fn);},
    callService(endpoint,type,request={},options={}){
      calls.push({endpoint,type,request,options});active++;peak=Math.max(peak,active);
      if(endpoint==='/rosapi/node_details'){detailActive++;detailPeak=Math.max(detailPeak,detailActive);}
      return new Promise((resolve,reject)=>{
        let timer,done=false;
        const finish=(error,value)=>{if(done)return;done=true;active--;if(endpoint==='/rosapi/node_details')detailActive--;clearTimeout(timer);options.signal?.removeEventListener('abort',abort);error?reject(error):resolve(value);};
        const abort=()=>finish(Object.assign(new Error('Cancelled'),{code:'WAIT_ABORTED'}));
        if(options.signal?.aborted){abort();return;}options.signal?.addEventListener('abort',abort,{once:true});
        timer=setTimeout(()=>{
          if(fail.includes(request.node)||fail.includes(endpoint)){finish(new Error('Node disappeared'));return;}
          if(endpoint==='/rosapi/node_details')finish(null,details[request.node]??{publishing:request.node.endsWith('/source')?['/lab/data']:[],subscribing:request.node.endsWith('/sink')?['/lab/data']:[],services:request.node.endsWith('/source')?['/lab/reset']:[]});
          else finish(null,main[endpoint.split('/').at(-1)]);
        },delay);
      });
    },
  };
  return {client,state,disconnect(){client.connected=false;for(const fn of listeners)fn('disconnected');}};
}
await check('real declared directed edges, types, services/providers and Action list',async()=>{
  const f=fixture(),g=await discoverGraph(f.client);
  assert.equal(g.status,'complete');assert.equal(g.nodes.length,2);assert.equal(g.topics.length,1);
  assert.deepEqual(g.edges.map(e=>e.kind).sort(),['publish','subscribe']);
  assert.equal(g.topics[0].types[0],'std_msgs/msg/String');
  assert.deepEqual(g.services.find(s=>s.name==='/lab/reset').providers,['/lab/source']);
  assert.equal(g.actions[0].name,'/lab/task');assert.equal(g.capabilities.endpointQos,false);
  assert.equal(f.state.listeners.size,0);assert.equal(f.state.active,0);
  assert(f.state.calls.every(c=>c.endpoint.startsWith('/rosapi/')));
});
await check('node and Topic with identical names never alias',async()=>{
  const f=fixture({nodes:['/same'],topics:['/same'],details:{'/same':{publishing:['/same'],subscribing:['/same'],services:[]}}});
  const g=await discoverGraph(f.client);assert.notEqual(g.nodes[0].id,g.topics[0].id);
  assert.equal(new Set(graphElements(g).elements.map(e=>e.data.id)).size,4);
});
await check('vanished node retains successful data and marks partial',async()=>{
  const f=fixture({fail:['/lab/sink']}),g=await discoverGraph(f.client);
  assert.equal(g.status,'partial');assert(g.errors.length);assert.equal(g.edges.length,1);assert.equal(f.state.active,0);
});
await check('duplicate ROS node names are visibly ambiguous',async()=>{
  const g=await discoverGraph(fixture({nodes:['/lab/source','/lab/source']}).client);
  assert.equal(g.status,'partial');assert.equal(g.nodes[0].duplicates,2);
});
await check('node/topic/edge caps report partial and remain bounded',async()=>{
  const f=fixture({nodes:['/a','/b','/c'],topics:['/a','/b','/c'],types:['x/msg/A','x/msg/B','x/msg/C'],details:{'/a':{publishing:['/a','/b'],subscribing:['/b'],services:[]},'/b':{publishing:['/a'],subscribing:['/a'],services:[]}}});
  const g=await discoverGraph(f.client,{maxNodes:2,maxTopics:2,maxEdges:2});
  assert.equal(g.status,'partial');assert(g.nodes.length<=2&&g.topics.length<=2&&g.edges.length<=2);
});
await check('deadline and cancellation settle locally without leaving listeners',async()=>{
  for(const bySignal of [false,true]){
    const f=fixture({delay:100}),controller=new AbortController();
    const pending=discoverGraph(f.client,{timeoutMs:bySignal?500:20,signal:controller.signal});
    if(bySignal)setTimeout(()=>controller.abort(),10);
    const g=await pending;assert.equal(g.cancelled,true);assert.notEqual(g.status,'complete');
    assert.equal(f.state.active,0);assert.equal(f.state.listeners.size,0);
  }
});
await check('disconnect aborts current generation and never emits a complete graph',async()=>{
  const f=fixture({delay:100}),p=discoverGraph(f.client);setTimeout(()=>f.disconnect(),10);
  const g=await p;assert(g.cancelled);assert.notEqual(g.status,'complete');assert.equal(f.state.active,0);
});
await check('node-detail RPCs use configured finite concurrency and fresh timeout',async()=>{
  const f=fixture({nodes:Array.from({length:15},(_,i)=>`/node_${i}`),delay:2});
  const g=await discoverGraph(f.client,{concurrency:2});assert.equal(g.status,'complete');
  const details=f.state.calls.filter(c=>c.endpoint==='/rosapi/node_details');assert.equal(details.length,15);
  assert(f.state.peak<=4);assert(f.state.detailPeak<=2);assert(details.every(c=>c.options.timeoutMs>0&&c.options.timeoutMs<=3000));
});
await check('malformed names/details and missing list are errors rather than fake success',async()=>{
  const f=fixture({nodes:['bad name','/valid'],topics:['/good','bad'],types:['x/msg/A','x/msg/B'],details:{'/valid':{publishing:{length:1e12},subscribing:[],services:[]}}});
  const g=await discoverGraph(f.client);assert.equal(g.status,'partial');assert(g.errors.length);assert.equal(g.edges.length,0);
  await assert.rejects(()=>discoverGraph(f.client,{concurrency:0}),/concurrency/);
});
await check('already-aborted refresh sends no request',async()=>{
  const f=fixture(),signal=AbortSignal.abort(),g=await discoverGraph(f.client,{signal});
  assert.equal(g.cancelled,true);assert.equal(f.state.calls.length,0);assert.equal(f.state.listeners.size,0);
});
await check('node projection has real Topic labels and caps the pub-sub product',async()=>{
  const g=await discoverGraph(fixture().client),projected=graphElements(g,{mode:'nodes'});
  assert.equal(projected.elements.filter(e=>e.data.source).length,1);
  assert.equal(projected.elements.find(e=>e.data.source).data.label,'/lab/data');
  const f=fixture({nodes:['/a','/b'],details:{'/a':{publishing:['/lab/data'],subscribing:['/lab/data'],services:[]},'/b':{publishing:['/lab/data'],subscribing:['/lab/data'],services:[]}}});
  assert.equal(graphElements(await discoverGraph(f.client),{mode:'nodes',maxEdges:1}).truncated,true);
});
console.log(`PASS ${groups} graph collector/projection groups; no ROS node started`);
