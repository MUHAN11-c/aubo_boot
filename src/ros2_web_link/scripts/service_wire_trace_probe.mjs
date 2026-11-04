/** Actual native Node WebSocket + local ws server; no ROS or browser process. */
import assert from 'node:assert/strict';
import {existsSync} from 'node:fs';
import {once} from 'node:events';
import {WebSocketServer} from 'ws';
import {RosClient} from '../dist/sdk/index.js';

const target=new URL('./service_wire_trace.mjs',import.meta.url);
const factory=existsSync(target)?(await import(target)).createServiceWireTrace:undefined;
const Native=globalThis.WebSocket,endpoint='/rosapi/nodes';
const server=new WebSocketServer({host:'127.0.0.1',port:0});
const nativeSockets=new Set();
server.on('connection',socket=>{
  nativeSockets.add(socket);socket.on('close',()=>nativeSockets.delete(socket));
  socket.on('message',(data,binary)=>{
    if(!binary){try{const m=JSON.parse(data.toString());if(m.op==='call_service'){
      socket.send(JSON.stringify({op:'service_response',id:m.id,service:m.service,result:true,values:{nodes:['private-result']}}));return;
    }}catch{/* A malformed diagnostic candidate must still pass unchanged. */}}
    socket.send(data,{binary});
  });
});
await once(server,'listening');
const url=`ws://127.0.0.1:${server.address().port}`;
const make=options=>factory?factory(Native,options):{WebSocket:Native,withSession:(_tag,fn)=>fn(),flush:()=>{}};
const collect=records=>text=>records.push(...text.trimEnd().split('\n').map(line=>JSON.parse(line)));
const clients=[],sockets=[];
async function rpc(trace,slot=0,generation=1,timeoutMs=2000){
  globalThis.WebSocket=trace.WebSocket;
  const client=new RosClient({url,reconnect:false});clients.push(client);
  await trace.withSession({slot,generation},()=>client.connect());
  const result=await client.callService(endpoint,'rosapi_msgs/srv/Nodes',{}, {timeoutMs});
  assert.deepEqual(result.nodes,['private-result']);return client;
}
async function plain(trace,tag={slot:1,generation:2}){
  const socket=trace.withSession(tag,()=>new trace.WebSocket(url));sockets.push(socket);
  socket.binaryType='arraybuffer';await once(socket,'open');return socket;
}
try{
  if(process.env.WEB_LINK_TRACE_SLOW_PROBE){
    let writes=0;
    const trace=make({endpoint,write:()=>{writes++;const until=performance.now()+100;while(performance.now()<until){/* Controlled slow disk surrogate. */}}});
    const result=await rpc(trace,0,1,50);
    assert.equal(writes,0,'Diagnostic sink ran inside an RPC deadline');
    trace.flush();assert.equal(writes,1);result.dispose();
    console.log('PASS slow sink is deferred until explicit flush after RPC completion');
  }else{
  const records=[],trace=make({endpoint,write:collect(records)});
  const client=await rpc(trace,2,3);
  assert.equal(records.length,0,'Sink must stay outside the measured request');trace.flush();
  const send=records.find(r=>r.phase==='send'),receive=records.find(r=>r.phase==='receive');
  assert.ok(send&&receive,'Missing native send/receive service metadata');
  assert.equal(send.cid,receive.cid);assert.equal(send.slot,2);assert.equal(receive.generation,3);
  assert.ok(send.monotonic<=receive.monotonic);assert.ok(receive.wall>=send.wall);
  assert.equal(receive.result,true);assert.ok(!JSON.stringify(records).includes('private-result'));
  client.dispose();
  console.log('PASS actual SDK/native socket: exact CID, session tag, timing, payload privacy');

  const concurrent=[],parallel=make({endpoint,write:collect(concurrent)});
  const group=await Promise.all([rpc(parallel,0,1),rpc(parallel,1,2),rpc(parallel,2,3)]);
  parallel.flush();
  const sent=concurrent.filter(x=>x.phase==='send');assert.equal(sent.length,3);
  assert.deepEqual(sent.map(x=>[x.slot,x.generation]).sort(),[[0,1],[1,2],[2,3]]);
  for(const item of sent){const received=concurrent.find(x=>x.phase==='receive'&&x.cid===item.cid);
    assert.ok(received);assert.equal(received.slot,item.slot);assert.equal(received.generation,item.generation);}
  group.forEach(item=>item.dispose());
  console.log('PASS concurrent asynchronous SDK connections retain separate primitive tags');

  const filtered=[],filter=make({endpoint,write:collect(filtered)});
  const socket=await plain(filter);
  for(const value of [JSON.stringify({op:'publish',topic:'/other',msg:{data:'private'}}),
    '{"op":"service_response",',JSON.stringify({op:'service_response',service:endpoint,id:'large',values:'x'.repeat(17000)})]){
    const message=once(socket,'message');socket.send(value);assert.equal((await message)[0].data,value);
  }
  const binary=new Uint8Array(1048576);binary[10]=71;
  const message=once(socket,'message');socket.send(binary);const echoed=(await message)[0].data;
  assert.equal(echoed.byteLength,binary.byteLength);assert.equal(new Uint8Array(echoed)[10],71);
  filter.flush();assert.deepEqual(filtered,[]);socket.close();
  console.log('PASS binary/large/unrelated/malformed frames unchanged and absent from trace');

  const bounded=[],notices=[],limit=make({endpoint,maxRecords:2,maxBytes:4096,
    write:collect(bounded),onError:error=>notices.push(error)});
  const boundedClient=await rpc(limit);
  for(let i=0;i<9;i++)await boundedClient.callService(endpoint,'rosapi_msgs/srv/Nodes',{}, {timeoutMs:2000});
  limit.flush();assert.equal(bounded.length,2);assert.equal(notices.length,1);assert.ok(limit.stats().budgetExceeded);
  await boundedClient.callService(endpoint,'rosapi_msgs/srv/Nodes',{}, {timeoutMs:2000});limit.flush();
  assert.equal(bounded.length,2,'Flush must not renew the total record budget');
  assert.equal(limit.stats().bufferedRecords,0);assert.equal(limit.stats().bufferedBytes,0);
  boundedClient.dispose();
  const tiny=[],bytes=make({endpoint,maxBytes:1,write:line=>tiny.push(line)});
  (await rpc(bytes)).dispose();bytes.flush();assert.deepEqual(tiny,[]);assert.ok(bytes.stats().budgetExceeded);
  console.log('PASS record and UTF-8 byte budgets do not suppress or retry RPC');

  let writes=0;const broken=make({endpoint,write:()=>{writes++;throw Error('controlled ENOSPC');},onError:()=>{throw Error('secondary sink failure');}});
  const healthy=await rpc(broken);
  broken.flush();
  for(let i=0;i<9;i++){await healthy.callService(endpoint,'rosapi_msgs/srv/Nodes',{}, {timeoutMs:2000});broken.flush();}
  assert.equal(writes,8);assert.equal(broken.stats().sinkErrors,8);healthy.dispose();
  console.log('PASS diagnostic write and reporting errors remain isolated; max eight attempts');
  }
}finally{
  for(const client of clients)client.dispose();
  for(const socket of sockets)socket.close();
  globalThis.WebSocket=Native;
  for(const socket of nativeSockets)socket.terminate();
  await new Promise(resolve=>server.close(resolve));
}
