/** Isolated CLI fault injection, never browser automation or a robot command.
 * Pause a ws reader using the library's public pause/resume API. A separate SDK
 * session must still receive real rosapi responses. Keep no payload history.
 */
import assert from 'node:assert/strict';
import {writeFileSync} from 'node:fs';
import {once} from 'node:events';
import WebSocket from 'ws';
import {RosClient} from '../dist/sdk/index.js';

const [url,configUrl,output]=process.argv.slice(2);
if(!url||!configUrl||!output)throw Error('usage: node backpressure_probe.mjs ws://host:port http://host:port/config.json evidence.json');
const config=await fetch(configUrl).then(r=>{if(!r.ok)throw Error(`HTTP ${r.status}`);return r.json();});
const project=config.projects.find(p=>p.id===config.project);
const cloud=project?.cases.find(c=>c.kind==='cloud')?.topic;
const depth=project?.cases.find(c=>c.kind==='image'&&c.topic.depthScale)?.topic;
assert.ok(cloud&&depth,'project must declare cloud and depth cases');
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const client=new RosClient({url,reconnect:false});
const slow=new WebSocket(url,{maxPayload:16_000_000});
let frames=0,bytes=0,peakRssMiB=0,closed=false;const latencies=[];
const report={url,project:project.id,pausedSeconds:15,topics:[cloud.name,depth.name]};
slow.on('message',data=>{frames++;bytes+=data.length;});
slow.on('error',error=>{report.socketError=error.message;});
slow.on('close',()=>{closed=true;});
const guard=setInterval(()=>{peakRssMiB=Math.max(peakRssMiB,process.memoryUsage().rss/1048576);},100);
const watchdog=setTimeout(()=>{slow.terminate();client.dispose();},35000);
try{
  await Promise.race([once(slow,'open'),sleep(5000).then(()=>{throw Error('slow socket open timeout');})]);
  await client.connect();
  for(const topic of [cloud,depth])slow.send(JSON.stringify({op:'subscribe',topic:topic.name,type:topic.type,compression:'cbor',queue_length:1,throttle_rate:0,qos:{history:'keep_last',depth:5,reliability:'reliable',durability:'volatile'}}));
  const warmDeadline=Date.now()+6000;
  while(!frames&&Date.now()<warmDeadline)await sleep(100);
  assert.ok(frames>0,'source delivered no messages before pause');
  slow.pause();report.framesBeforePause=frames;
  for(let i=0;i<15;i++){
    const at=performance.now();const response=await client.callService('/rosapi/nodes','rosapi_msgs/srv/Nodes',{}, {timeoutMs:2000});
    assert.ok(response.nodes.length>0);latencies.push(performance.now()-at);await sleep(1000);
  }
  report.framesWhilePaused=frames-report.framesBeforePause;
  assert.equal(report.framesWhilePaused,0,'reader was not actually paused');
  slow.resume();const beforeResume=frames;await sleep(2500);
  assert.ok(!closed&&frames>beforeResume,'stream failed to recover after resume');
  report.framesAfterResume=frames-beforeResume;report.payloadMiB=bytes/1048576;
  report.serviceP95Ms=[...latencies].sort((a,b)=>a-b)[Math.floor((latencies.length-1)*.95)];
  report.serviceCalls=latencies.length;report.status='PASS';
}catch(error){report.status='FAIL';report.error=error.message;throw error;}
finally{
  clearInterval(guard);clearTimeout(watchdog);slow.terminate();client.dispose();
  report.peakClientRssMiB=peakRssMiB;report.clientResources=client.stats();
  writeFileSync(output,JSON.stringify(report,null,2));
}
console.log(JSON.stringify(report));
console.log('PASS 15s paused reader, independent service responses and stream recovery; both sessions closed');
