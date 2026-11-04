/** Actual SceneView clock/expiry callbacks, without DOM/WebGL/ROS entities. */
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {SceneView, TfBuffer} from '../dist/scene/index.js';

const originalNow=Date.now,originalRaf=globalThis.requestAnimationFrame;
let wallMs=1_000_000;
Date.now=()=>wallMs;globalThis.requestAnimationFrame=()=>1;
const fixtures=[];
function fixture(useSimTime) {
  const callbacks=new Map(),states=new Set(),reports=[],view=Object.create(SceneView.prototype);
  const client={onState:fn=>{states.add(fn);fn('connected');return()=>states.delete(fn);},subscribe:(name,type,callback)=>{
    if(!callbacks.has(name)){const members=new Set();const dispatch=message=>{for(const fn of [...members])fn(message);};dispatch.members=members;callbacks.set(name,dispatch);}
    const dispatch=callbacks.get(name);dispatch.members.add(callback);
    return()=>{dispatch.members.delete(callback);if(!dispatch.members.size)callbacks.delete(name);};
  }};
  Object.assign(view,{options:{useSimTime,fixedFrame:'world',robotDescription:'/robot',jointStates:'/joints',tf:'/tf',tfStatic:'/tf_static',markers:'/markers',pointCloud:'/points',path:'/path',onStatus:m=>reports.push(m)},
    client,tfBuffer:new TfBuffer(client,{clockTopic:useSimTime?'/clock':false}),
    off:[],children:new Set(),scene:new THREE.Scene(),joints:Object.create(null),pendingJoints:new Map(),markerMap:new Map(),axes:new Map(),layerEnabled:new Map(),retired:[],assetErrors:new Set(),animate:()=>{},generation:0,
    children:new Set(),disposed:false,controls:{update(){}},lastTfUpdate:0,loopCount:0,maxLoopGapMs:0,previousLoop:0,dirty:true,
    renderer:{getContext:()=>({isContextLost:()=>false}),render(){},info:{programs:[]}},renderErrors:0,renderSamples:[],renderCount:0,renderMaxMs:0,firstRenderMs:null,firstRenderCompleteAtMs:null,startedAt:performance.now(),programsSeen:new WeakSet(),programCreations:0,lastReport:performance.now()});
  fixtures.push({view,callbacks,states});view.start();
  view.tfBuffer.ingest({transforms:[{header:{frame_id:'world',stamp:{sec:0,nanosec:0}},child_frame_id:'sensor',transform:{translation:{x:0,y:0,z:0},rotation:{x:0,y:0,z:0,w:1}}}]},true);
  const marker={header:{frame_id:'world',stamp:{sec:0,nanosec:0}},ns:'clock',id:1,type:2,action:0,pose:{position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}},scale:{x:1,y:1,z:1},color:{r:1,g:0,b:0,a:1},points:[],colors:[],text:'',mesh_resource:'',frame_locked:false,lifetime:{sec:2,nanosec:0}};
  return {view,callbacks,reports,add:(overrides={})=>callbacks.get('/markers')({markers:[{...marker,...overrides}]}),
    observations:()=>{
      callbacks.get('/points')({header:marker.header,width:1,height:1,point_step:12,row_step:12,is_bigendian:false,fields:['x','y','z'].map((name,i)=>({name,offset:i*4,datatype:7,count:1})),data:new Uint8Array(12)});
      callbacks.get('/path')({header:marker.header,poses:[{pose:{position:{x:1,y:0,z:0}}}]});
    },
    clock:(sec,nanosec=0)=>callbacks.get('/clock')({clock:{sec,nanosec}}),render:()=>view.renderFrame()};
}
const cases=[
  ['standalone TF clock is opt-in and releases every subscription',()=>{
    const subscriptions=new Map();let listeners=0;
    const client={onState:fn=>{listeners++;fn('connected');return()=>{listeners--;};},subscribe:(name,type,fn)=>{subscriptions.set(name,fn);return()=>subscriptions.delete(name);}};
    const ordinary=new TfBuffer(client);
    try{ordinary.start();assert.equal(subscriptions.has('/clock'),false,'standalone TF must explicitly select a simulation clock');}
    finally{ordinary.dispose();}
    const recorded=new TfBuffer(client,{clockTopic:'/clock'});
    try{recorded.start();assert.equal(subscriptions.has('/clock'),true);}
    finally{recorded.dispose();}
    assert.equal(subscriptions.size,0);assert.equal(listeners,0);
  }],
  ['default system clock neither subscribes nor switches to unrelated /clock',()=>{
    wallMs=1_000_000;const f=fixture(undefined);
    assert.equal(f.callbacks.has('/clock'),false,'ROS clock must be explicitly enabled');
    f.add();wallMs+=2000;f.render();assert.equal(f.view.markerMap.size,0,'expiry is inclusive at its deadline');
  }],
  ['explicit ROS clock never mixes wall epoch before first tick',()=>{
    wallMs=1_000_000;const f=fixture(true);
    assert.equal(f.view.stats().clockSeconds,0);assert.equal(f.view.stats().clockReady,false);
    f.view.lastReport=-Infinity;f.add();f.render();assert.match(f.reports.at(-1)??'',/等待.*clock/);
    f.clock(10);f.render();assert.equal(f.view.stats().clockReady,true);
    assert.equal(f.view.markerMap.size,0,'pre-clock lifetime at ROS zero must expire after a forward jump past its deadline');
  }],
  ['paused ROS time freezes lifetime and exact deadline expires',()=>{
    const f=fixture(true);f.clock(10);f.add();let released=0;
    f.view.markerMap.get('clock:1').object.children[0].geometry.addEventListener('dispose',()=>released++);
    wallMs+=1_000_000;f.render();assert.equal(f.view.markerMap.size,1);assert.equal(released,0);
    f.clock(11,999999999);f.render();assert.equal(f.view.markerMap.size,1);
    f.clock(12);f.render();assert.equal(f.view.markerMap.size,0,'ROS deadline must use >=');assert.equal(released,1);
  }],
  ['zero lifetime persists; ROS rewind clears prior-scene markers',()=>{
    const f=fixture(true);f.clock(20);f.add({lifetime:{sec:0,nanosec:0}});f.clock(100);f.render();assert.equal(f.view.markerMap.size,1);
    f.clock(19);assert.equal(f.view.markerMap.size,0);
  }],
  ['malformed ROS clock cannot poison lifetime state',()=>{
    const f=fixture(true);f.clock(5);f.clock(6,1_000_000_000);assert.equal(f.view.stats().clockSeconds,5);assert.match(f.reports.at(-1)??'',/clock|Clock/);
    f.clock(NaN);assert.equal(f.view.stats().clockSeconds,5);
  }],
  ['one-nanosecond rewind near int32 max keeps Scene and actual TF in the same epoch',()=>{
    const f=fixture(true),sec=2147483647;f.clock(sec,1);f.add({lifetime:{sec:0,nanosec:0}});f.observations();
    f.view.tfBuffer.ingest({transforms:[{header:{frame_id:'world',stamp:{sec,nanosec:1}},child_frame_id:'tool',transform:{translation:{x:1,y:0,z:0},rotation:{x:0,y:0,z:0,w:1}}}]});
    assert.ok(f.view.tfBuffer.lookup('world','tool'));
    f.clock(sec,0);assert.equal(f.view.tfBuffer.lookup('world','tool'),undefined);
    assert.equal(f.view.markerMap.size,0);assert.equal(f.view.cloud,undefined);assert.equal(f.view.trace,undefined);
    assert.ok(f.view.tfBuffer.lookup('world','sensor'),'static TF survives rewind');
  }],
  ['submillisecond lifetime boundary retains integer nanoseconds at large epochs',()=>{
    const f=fixture(true),sec=2147483647,start=123456789,duration=200000;
    f.clock(sec,start);f.add({lifetime:{sec:0,nanosec:duration}});
    f.clock(sec,start+duration-1);f.render();assert.equal(f.view.markerMap.size,1,'must not expire one nanosecond early');
    f.clock(sec,start+duration);f.render();assert.equal(f.view.markerMap.size,0);
  }],
  ['reused Marker refreshes its lifetime and zero lifetime removes the deadline',()=>{
    const f=fixture(true);f.clock(10);f.add();const shape=f.view.markerMap.get('clock:1').object;
    f.clock(11);f.add();assert.equal(f.view.markerMap.get('clock:1').object,shape);
    f.clock(12);f.render();assert.equal(f.view.markerMap.size,1);
    f.add({lifetime:{sec:0,nanosec:0}});f.clock(100);f.render();assert.equal(f.view.markerMap.size,1);
  }],
  ['invalid time mode rejects before renderer allocation',()=>{
    assert.throws(()=>new SceneView({}, {}, {fixedFrame:'world',useSimTime:'false'}),/useSimTime.*boolean/);
  }]
];
let failed=0;
try {
  for(const [name,run] of cases){try{run();console.log('PASS',name);}catch(error){failed++;console.error('FAIL',name,error.message);}}
} finally {
  for(const {view,callbacks,states} of fixtures){view.off.forEach(off=>off());view.tfBuffer.dispose();view.resetData();view.flushRetired();assert.equal(callbacks.size,0);assert.equal(states.size,0);}
  Date.now=originalNow;globalThis.requestAnimationFrame=originalRaf;
}
if(failed)throw new Error(`${failed} scene time groups failed`);
console.log(`PASS ${cases.length} scene time groups; no WebGL context, ROS process or port created`);
