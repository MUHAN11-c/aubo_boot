// Explicit protocol evidence, not an application command or an SDK retry.
import assert from 'node:assert/strict';
import {writeFileSync} from 'node:fs';
import WebSocket from 'ws';
const socket=new WebSocket(process.argv[2]);
const replies=new Map();
socket.on('message',bytes=>{const message=JSON.parse(bytes.toString());const waiter=replies.get(message.id);if(waiter){replies.delete(message.id);waiter(message);}});
await new Promise((resolve,reject)=>{socket.once('open',resolve);socket.once('error',reject);});
const call=(id,service,timeout)=>new Promise((resolve,reject)=>{
  const timer=setTimeout(()=>{replies.delete(id);reject(Error(`No protocol response for ${id}`));},6000);
  replies.set(id,message=>{clearTimeout(timer);resolve(message);});
  socket.send(JSON.stringify({op:'call_service',id,service,args:{},timeout}));
});
try{
  // Give normal DDS discovery time, then require success and a separate failure.
  await new Promise(resolve=>setTimeout(resolve,1000));
  const success=await call('wait-probe-success','/web_link_demo/wait_probe',2);
  assert.equal(success.result,true);assert.equal(success.values.success,true);
  const timeout=await call('wait-probe-timeout','/web_link_demo/wait_probe',0.1);
  assert.equal(timeout.result,false);
  const other=await call('wait-probe-other','/rosapi/nodes',2);assert.equal(other.result,true);
  writeFileSync(process.argv[3],JSON.stringify({success:true,timeoutRejected:true,otherEndpoint:true},null,2));
}finally{socket.terminate();}
