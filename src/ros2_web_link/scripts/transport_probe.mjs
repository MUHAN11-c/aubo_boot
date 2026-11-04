/** Bounded real rosbridge transport benchmark. No browser automation or robot command. */
import assert from 'node:assert/strict';
import {mkdirSync,writeFileSync,readFileSync} from 'node:fs';
import path from 'node:path';
import {performance} from 'node:perf_hooks';
import {RosClient} from '../dist/sdk/index.js';
import {decodeImage} from '../dist/media/image.js';
import {decodePointCloud} from '../dist/scene/pointcloud.js';
import {TfBuffer} from '../dist/scene/tf.js';

const url=process.argv[2]??'ws://127.0.0.1:9290';
const seconds=Number(process.env.WEB_LINK_BENCH_SECONDS??8);
const encodings=(process.env.WEB_LINK_BENCH_ENCODINGS??'none,cbor').split(',');
const fragmentBytes=Number(process.env.WEB_LINK_FRAGMENT_BYTES??0);
if(!encodings.length||encodings.some(x=>!['none','cbor'].includes(x)))throw Error('WEB_LINK_BENCH_ENCODINGS must contain none and/or cbor');
if(fragmentBytes&&(!Number.isSafeInteger(fragmentBytes)||fragmentBytes<1024||fragmentBytes>16000000||encodings.includes('cbor')))throw Error('Fragment injection requires JSON-only and 1024..16000000 bytes');
if(!Number.isFinite(seconds)||seconds<2||seconds>60)throw Error('WEB_LINK_BENCH_SECONDS must be 2..60');
const out=process.env.WEB_LINK_EVIDENCE??'/tmp/ros2-web-transport';mkdirSync(out,{recursive:true});
const config=process.env.WEB_LINK_PROJECT_CONFIG?JSON.parse(readFileSync(process.env.WEB_LINK_PROJECT_CONFIG,'utf8')):await fetch(process.argv[3]??'http://127.0.0.1:8288/config.json').then(r=>{if(!r.ok)throw Error(`config HTTP ${r.status}`);return r.json();}).then(c=>c.projects.find(p=>p.id===c.project));
assert.ok(config?.view?.scene,'benchmark requires a project with scene and raw image cases');
const scene=config.view.scene,colorCase=config.cases.find(c=>c.kind==='image'&&!c.topic.depthScale),depthCase=config.cases.find(c=>c.kind==='image'&&c.topic.depthScale),cloudCase=config.cases.find(c=>c.kind==='cloud'),tfCase=config.cases.find(c=>c.kind==='tf');
assert.ok(colorCase&&depthCase&&cloudCase&&tfCase,'declare color/depth/cloud/TF cases in the project contract');
const NativeSocket=globalThis.WebSocket;
let wire;
// Instrument the native socket used by upstream roslib, before its decoder.
// Count application payload bytes, not TCP/TLS framing, without retaining frames.
globalThis.WebSocket=class extends NativeSocket {
  constructor(...args){super(...args);this.addEventListener('message',event=>{
    if(!wire)return;const bytes=typeof event.data==='string'?Buffer.byteLength(event.data):event.data.byteLength??event.data.size??0;
    wire.bytes+=bytes;wire.frames++;wire.maxFrame=Math.max(wire.maxFrame,bytes);
    if(typeof event.data==='string')wire.textFrames++;else wire.binaryFrames++;
    if(typeof event.data==='string'&&/"op"\s*:\s*"fragment"/.test(event.data.slice(0,160))){
      wire.fragmentFrames++;
      if(fragmentBytes){try{const m=JSON.parse(event.data);if(m.num===0&&wire.fragmentStarts.length<32)wire.fragmentStarts.push({id:m.id,total:m.total});}catch{}}
    }
  });}
  send(text){
    if(fragmentBytes&&typeof text==='string'){
      const message=JSON.parse(text);if(message.op==='subscribe'){message.fragment_size=fragmentBytes;text=JSON.stringify(message);}
    }
    super.send(text);
  }
};
const quantile=(values,q)=>values.length?[...values].sort((a,b)=>a-b)[Math.min(values.length-1,Math.floor(values.length*q))]:null;
const measurements=[];
try{
  for(const compression of encodings){
    wire={bytes:0,frames:0,maxFrame:0,textFrames:0,binaryFrames:0,fragmentFrames:0,fragmentStarts:[]};
    const client=new RosClient({url,reconnect:false});const stops=[];const samples={};const errors=[];const rpc=[];let peakHeap=0,peakRss=0;
    client.onStatus(message=>{if(message.level==='error')errors.push(`${message.code??'BRIDGE'}: ${message.msg}`);});
    const tf=new TfBuffer(client,{tfTopic:scene.tf,tfStaticTopic:scene.tfStatic,clockTopic:'/clock'});const mark=(name,fn)=>message=>{
      const s=samples[name]??={count:0,decodeMs:[],bytes:0};const start=performance.now();
      try{const result=fn(message);s.count++;s.latest=result;s.decodeMs.push(performance.now()-start);if(s.decodeMs.length>2000)s.decodeMs.shift();}
      catch(error){errors.push(`${name}: ${error.message}`);}
    };
    let timer;const begin=performance.now();
    try{
      await client.connect();tf.start();
      const binary={qos:'sensor',throttleMs:500,compression};
      stops.push(client.subscribe(colorCase.topic.name,colorCase.topic.type,mark('color',m=>{const d=decodeImage(m);return {width:d.width,height:d.height,decodedBytes:d.pixels.byteLength,wireBytes:typeof m.data==='string'?m.data.length:m.data.byteLength,dataType:typeof m.data==='string'?'base64':m.data.constructor.name};}),{...binary,qos:colorCase.topic.qos??'sensor'}));
      stops.push(client.subscribe(depthCase.topic.name,depthCase.topic.type,mark('depth',m=>{const d=decodeImage(m,{depthScale:depthCase.topic.depthScale});return {width:d.width,height:d.height,decodedBytes:d.pixels.byteLength,dataType:typeof m.data==='string'?'base64':m.data.constructor.name};}),{...binary,qos:depthCase.topic.qos??'sensor'}));
      stops.push(client.subscribe(cloudCase.topic.name,cloudCase.topic.type,mark('cloud',m=>{const d=decodePointCloud(m);return {sourcePoints:m.width*m.height,rawBytes:typeof m.data==='string'?Math.floor(m.data.length*3/4):m.data.byteLength,points:d.count,hasColors:!!d.colors,dataType:typeof m.data==='string'?'base64':m.data.constructor.name};}),{...binary,qos:cloudCase.topic.qos??'sensor',throttleMs:100}));
      stops.push(client.subscribe(scene.robotDescription,'std_msgs/msg/String',mark('urdf',m=>{assert.match(m.data,/<robot[\s>]/);return {xmlBytes:Buffer.byteLength(m.data)};}),{qos:'latched'}));
      stops.push(client.subscribe(scene.jointStates,'sensor_msgs/msg/JointState',mark('joints',m=>{assert.ok(m.name.length>0);assert.equal(m.position.length,m.name.length);return {joints:m.name.length};}),{qos:'sensor'}));
      timer=setInterval(()=>{const m=process.memoryUsage();peakHeap=Math.max(peakHeap,m.heapUsed);peakRss=Math.max(peakRss,m.rss);},250);
      for(let i=0;i<seconds;i++){
        const start=performance.now();await client.callService('/rosapi/nodes','rosapi_msgs/srv/Nodes',{}, {timeoutMs:3000});rpc.push(performance.now()-start);
        await new Promise(resolve=>setTimeout(resolve,1000));
      }
      const transforms=Object.fromEntries(tfCase.sources.map(f=>[f,!!tf.lookup(tfCase.target,f)]));
      const elapsedMs=performance.now()-begin;const stats=Object.fromEntries(Object.entries(samples).map(([k,s])=>[k,{count:s.count,receiveHz:s.count/(elapsedMs/1000),decodeP50Ms:quantile(s.decodeMs,.5),decodeP95Ms:quantile(s.decodeMs,.95),latest:s.latest}]));
      for(const name of ['color','depth','cloud','urdf','joints'])assert.ok(samples[name]?.count>0,`${name} produced no valid data; decoder errors: ${errors.join(';')||'none (no delivered frame)'}`);
      assert.ok(Object.values(transforms).every(Boolean),'incomplete TF');assert.equal(errors.length,0,errors.join(';'));
      if(fragmentBytes)assert.ok(wire.fragmentFrames>0,'native bridge did not produce fragments');
      measurements.push({compression,elapsedMs,wire:{...wire,MiBPerSecond:wire.bytes/1048576/(elapsedMs/1000)},transport:client.transportStats(),samples:stats,transforms,serviceP50Ms:quantile(rpc,.5),serviceP95Ms:quantile(rpc,.95),peakHeapMiB:peakHeap/1048576,peakRssMiB:peakRss/1048576,errors});
    }catch(error){measurements.push({compression,status:'FAIL',error:error.message,wire:{...wire},transport:client.transportStats(),errors,samples:Object.fromEntries(Object.entries(samples).map(([k,s])=>[k,{count:s.count,latest:s.latest}]))});throw error;}
    finally{clearInterval(timer);stops.reverse().forEach(stop=>stop());tf.dispose();assert.equal(client.stats().subscriptions,0);assert.equal(client.stats().pending,0);client.dispose();assert.equal(client.transportStats().fragmentMessages,0);assert.equal(client.transportStats().fragmentBytes,0);writeFileSync(path.join(out,'transport.json'),JSON.stringify({url,project:config.id,seconds,measurements},null,2));}
    writeFileSync(path.join(out,'transport.json'),JSON.stringify({url,project:config.id,seconds,measurements},null,2));console.log(JSON.stringify(measurements.at(-1)));
  }
}finally{wire=undefined;globalThis.WebSocket=NativeSocket;}
console.log('PASS selected real-project encodings, service under load and complete TF; all client subscriptions released');
