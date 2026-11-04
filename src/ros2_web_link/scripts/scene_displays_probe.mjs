/** Exercise real logical SceneView children using Three objects and a fake host GL. */
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {SceneView} from '../dist/scene/index.js';
const previousCancel=globalThis.cancelAnimationFrame;globalThis.cancelAnimationFrame=()=>{};

const callbacks=new Map(),states=new Set(),subscribeOptions=new Map();let bufferStarts=0,bufferDisposes=0,bufferClears=0,glDisposes=0,controlDisposes=0,resizeDisposes=0,rootClears=0;
const client={connected:true,onState(fn){states.add(fn);fn('connected');return()=>states.delete(fn);},subscribe(name,type,fn,options){subscribeOptions.set(name,options);const set=callbacks.get(name)??new Set();set.add(fn);callbacks.set(name,set);let stopped=false;return()=>{if(stopped)return;stopped=true;set.delete(fn);if(!set.size)callbacks.delete(name);};}};
const emit=(name,message)=>{for(const fn of [...(callbacks.get(name)??[])])fn(message);};
const transform={translation:{x:0,y:0,z:0},rotation:{x:0,y:0,z:0,w:1}};
const host=Object.create(SceneView.prototype);
Object.assign(host,{root:{replaceChildren(){rootClears++;}},client,options:{fixedFrame:'world',tf:'/tf',tfStatic:'/tf_static',robotDescription:'/robot',jointStates:'/joints',markers:'/markers',pointCloud:'/points',path:'/path',layers:[],useSimTime:true},
 scene:new THREE.Scene(),camera:new THREE.PerspectiveCamera(),renderer:{domElement:{removeEventListener(){}},dispose(){glDisposes++;},forceContextLoss(){},info:{render:{},memory:{},programs:[]}},
 controls:{dispose(){controlDisposes++;}},resize:{disconnect(){resizeDisposes++;}},
 tfBuffer:{start(){bufferStarts++;},dispose(){bufferDisposes++;},clear(){bufferClears++;},stats(){return{frames:1,lastReceivedAt:0};},frames(){return['world'];},lookup(to){return to==='world'?transform:undefined;}},
 off:[],children:new Set(),joints:Object.create(null),pendingJoints:new Map(),markerMap:new Map(),axes:new Map(),layerEnabled:new Map([['robot',false],['tf',false],['markers',false],['cloud',false],['path',false]]),retired:[],assetErrors:new Set(),restoreShaderHooks:[],generation:0,animate:()=>{}});
host.start();host.start();assert.equal(bufferStarts,1,'empty host start must be idempotent');
assert.deepEqual([...callbacks.keys()],['/clock'],'disabled legacy layers must not create data subscriptions');
const a=host.createDisplay({layers:['cloud'],pointCloud:'/cloud_a'}),b=host.createDisplay({layers:['cloud'],pointCloud:'/cloud_b'});
assert.equal(a.renderer,host.renderer);assert.equal(b.renderer,host.renderer);assert.equal(a.tfBuffer,host.tfBuffer);assert.equal(a.camera,b.camera);
a.start();b.start();assert.equal(bufferStarts,1);assert.deepEqual([...callbacks.keys()].sort(),['/clock','/cloud_a','/cloud_b']);
const cloud=n=>({header:{frame_id:'world',stamp:{sec:0,nanosec:0}},height:1,width:n,fields:[{name:'x',offset:0,datatype:7,count:1},{name:'y',offset:4,datatype:7,count:1},{name:'z',offset:8,datatype:7,count:1}],point_step:12,row_step:n*12,is_bigendian:false,data:new Uint8Array(n*12)});
emit('/clock',{clock:{sec:10,nanosec:0}});emit('/cloud_a',cloud(2));emit('/cloud_b',cloud(3));
assert.equal(a.stats().points,2);assert.equal(b.stats().points,3);assert.equal(host.stats().points,5);
const c=host.createDisplay({layers:['cloud'],pointCloud:'/cloud_b'});c.start();assert.equal(callbacks.get('/cloud_b').size,2);
emit('/cloud_b',cloud(4));b.dispose();assert.equal(callbacks.get('/cloud_b').size,1);assert.equal(c.stats().points,4);assert.equal(bufferDisposes,0);assert.equal(glDisposes,0);
host.setFixedFrame('map');assert.equal(a.options.fixedFrame,'map');assert.equal(c.options.fixedFrame,'map');
host.setFixedFrame('world');emit('/clock',{clock:{sec:5,nanosec:0}});assert.equal(a.stats().points,0);assert.equal(c.stats().points,0,'clock rewind clears every child temporal resource');
emit('/cloud_a',cloud(1));emit('/cloud_b',cloud(2));a.dispose();assert(!callbacks.has('/cloud_a'));assert.equal(c.stats().points,2);assert.equal(bufferClears,0,'child disposal must not clear shared TF');
assert.throws(()=>c.createDisplay({layers:['path'],path:'/p'}),/root|host/i);
const custom=host.createDisplay({layers:['markers','path','robot'],markers:'/custom_markers',path:'/custom_path',robotDescription:'/custom_robot',jointStates:'/custom_joints',markersOptions:{qos:'sensor'},pathOptions:{qos:'sensor'},robotDescriptionOptions:{qos:'reliable'}});custom.start();
assert.equal(subscribeOptions.get('/custom_markers').qos,'sensor');assert.equal(subscribeOptions.get('/custom_path').qos,'sensor');assert.equal(subscribeOptions.get('/custom_robot').qos,'reliable');custom.dispose();
host.dispose();host.dispose();assert.equal(callbacks.size,0);assert.equal(bufferDisposes,1);assert.equal(glDisposes,1);assert.equal(controlDisposes,1);assert.equal(resizeDisposes,1);assert.equal(rootClears,1);assert.equal(host.children.size,0);
assert(c.disposed);assert.throws(()=>host.createDisplay({layers:['cloud'],pointCloud:'/late'}),/disposed/i);
if(previousCancel===undefined)delete globalThis.cancelAnimationFrame;else globalThis.cancelAnimationFrame=previousCancel;
console.log('PASS shared SceneView layer subscriptions, independent streams, same context, clock rewind and resource ownership; real GL verified separately');
