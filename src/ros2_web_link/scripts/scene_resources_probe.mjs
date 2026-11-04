/** Exercise actual SceneView ROS callbacks with Three objects, without a browser renderer. */
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {SceneView} from '../dist/scene/index.js';
import {createShaderChecks} from '../dist/scene/shaders.js';
const callbacks=new Map(),inputErrors=[];const view=Object.create(SceneView.prototype);
Object.assign(view,{options:{robotDescription:'/robot',jointStates:'/joints',markers:'/markers',pointCloud:'/points',path:'/path',fixedFrame:'world',onStatus:m=>inputErrors.push(m)},client:{subscribe:(n,t,cb)=>{callbacks.set(n,cb);return()=>callbacks.delete(n);}},tfBuffer:{start(){}},off:[],children:new Set(),scene:new THREE.Scene(),joints:Object.create(null),pendingJoints:new Map(),markerMap:new Map(),axes:new Map(),layerEnabled:new Map(),retired:[],animate:()=>{},generation:0});
view.start();
function robot(names){return Object.assign(new THREE.Group(),{joints:Object.fromEntries(names.map(name=>[name,{jointType:'revolute'}])),writes:0,setJointValues(values){this.lastValues={...values};this.writes++;}});}
const observedRobot=robot(['known_joint','second_joint']);view.robot=observedRobot;
callbacks.get('/joints')({name:['known_joint'],position:[.75]});
for(let i=0;i<3000;i++)callbacks.get('/joints')({name:['another_robot_'+i],position:[i]});
assert.equal(Object.keys(view.joints).length,1,'unrelated joint names must not accumulate in the model value dictionary');
assert.equal(observedRobot.lastValues.known_joint,.75);
assert.equal(view.pendingJoints.size,1024);assert.equal(view.pendingJoints.has('another_robot_0'),false);
assert.equal(observedRobot.writes,1,'unrelated messages must not reapply the complete model');
callbacks.get('/joints')({name:['second_joint'],position:[.5]});
callbacks.get('/joints')({name:['known_joint'],position:[1]});
callbacks.get('/joints')({name:['known_joint'],position:[]});
callbacks.get('/joints')({name:['known_joint'],position:[NaN]});
assert.deepEqual(observedRobot.lastValues,{known_joint:1,second_joint:.5});
assert.equal(observedRobot.writes,3);assert.equal(Object.getPrototypeOf(view.joints),null);
// A small array-like payload must not control an unbounded numeric loop. Keep
// the red reproduction short; length 1e12 would otherwise freeze the browser.
callbacks.get('/joints')({name:{length:4},position:[]});
assert.match(inputErrors.at(-1)??'',/JointState.*array/);
let invalidLengthReads=0;
callbacks.get('/joints')({name:{get length(){invalidLengthReads++;return 1e12;}},position:[]});
assert.equal(invalidLengthReads,0,'array-like name length must never be inspected');
callbacks.get('/joints')({name:['known_joint'],position:{length:1,0:100}});
assert.match(inputErrors.at(-1)??'',/JointState.*array/);assert.equal(observedRobot.lastValues.known_joint,1);
callbacks.get('/joints')({name:['known_joint'],position:new Float64Array([1.5])});
assert.equal(observedRobot.lastValues.known_joint,1.5);
callbacks.get('/joints')({name:['future_joint'],position:[9]});
for(let i=0;i<1500;i++){
  callbacks.get('/joints')({name:['different_'+i,'future_joint'],position:[i,9]});
}
callbacks.get('/joints')({name:['__proto__','constructor','toString','x'.repeat(1025)],position:[1,2,3,4]});
assert.equal(view.pendingJoints.size,1024);assert.equal(view.pendingJoints.has('future_joint'),true);
assert.equal(view.pendingJoints.has('x'.repeat(1025)),false);
assert.equal(Object.keys(view.joints).length,2);assert.equal(Object.getPrototypeOf(view.joints),null);
console.log('PASS JointState partial updates remain model-scoped; unrelated names are bounded by count/length, LRU and prototype-safe');
const replacement=robot(['second_joint','future_joint']);view.robot=replacement;view.applyCachedJoints(replacement);
assert.deepEqual(replacement.lastValues,{second_joint:.5,future_joint:9});
assert.equal(Object.hasOwn(view.joints,'known_joint'),false);assert.equal(view.pendingJoints.has('future_joint'),false);
callbacks.get('/joints')({name:['future_joint'],position:[10]});
view.applyCachedJoints(replacement);assert.equal(replacement.lastValues.future_joint,10);
view.resetData();
callbacks.get('/joints')({name:['before_model'],position:[.25]});
assert.equal(Object.keys(view.joints).length,0);assert.equal(view.pendingJoints.size,1);
const later=robot(['before_model']);view.robot=later;view.applyCachedJoints(later);
assert.deepEqual(later.lastValues,{before_model:.25});assert.equal(view.pendingJoints.size,0);
console.log('PASS model replacement drops old joint names; bounded early JointState values apply when their URDF arrives');
const header={frame_id:'world',stamp:{sec:1,nanosec:0}};
function cloud(count){const data=new Uint8Array(count*12);const d=new DataView(data.buffer);for(let i=0;i<count*3;i++)d.setFloat32(i*4,i,true);return {header,width:count,height:1,point_step:12,row_step:count*12,is_bigendian:false,fields:['x','y','z'].map((name,i)=>({name,offset:i*4,datatype:7,count:1})),data};}
callbacks.get('/points')(cloud(2));const points=view.cloud.object,pointMaterial=points.material,pointGeometry=points.geometry;let disposed=0;pointGeometry.addEventListener('dispose',()=>disposed++);
callbacks.get('/points')(cloud(2));assert.equal(view.cloud.object,points);assert.equal(points.material,pointMaterial);assert.equal(points.geometry,pointGeometry);assert.equal(disposed,0);
callbacks.get('/points')(cloud(3));assert.equal(view.cloud.object,points);assert.equal(points.material,pointMaterial);assert.equal(disposed,1);assert.equal(points.geometry.getAttribute('position').count,3);
console.log('PASS cloud updates retain Points/material, reuse same-size GPU attributes, release resized geometry');
const trace=x=>({header,poses:[{pose:{position:{x,y:0,z:0}}},{pose:{position:{x:x+1,y:0,z:0}}}]});
callbacks.get('/path')(trace(0));const line=view.trace.object,material=line.material,geometry=line.geometry;callbacks.get('/path')(trace(2));assert.equal(view.trace.object,line);assert.equal(line.material,material);assert.equal(line.geometry,geometry);assert.equal(geometry.getAttribute('position').getX(0),2);
console.log('PASS Path updates retain material and update existing geometry');
const marker={header,ns:'probe',id:1,type:2,action:0,pose:{position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}},scale:{x:1,y:1,z:1},color:{r:1,g:0,b:0,a:1},points:[],colors:[],text:'',mesh_resource:'',frame_locked:false,lifetime:{sec:0,nanosec:0}};
callbacks.get('/markers')({markers:[marker]});const sphere=view.markerMap.get('probe:1').object.children[0];callbacks.get('/markers')({markers:[{...marker,pose:{...marker.pose,position:{x:2,y:0,z:0}},header:{...header,stamp:{sec:2,nanosec:0}}}]});assert.equal(view.markerMap.get('probe:1').object.children[0],sphere);assert.equal(view.markerMap.get('probe:1').pose.position.x,2);
callbacks.get('/markers')({markers:[{...marker,action:2}]});assert.equal(view.markerMap.size,0);
console.log('PASS unchanged Marker shape retains material across stamped pose updates; DELETE releases it');
callbacks.get('/markers')({markers:[marker,{...marker,ns:'other',id:2}]});const original=view.markerMap.get('probe:1').object;
callbacks.get('/markers')({markers:[{...marker,ns:'',action:3},marker]});
assert.ok(view.markerMap.get('probe:1').object===original,'DELETEALL + identical ADD must retain the shape within one atomic array');
assert.equal(view.markerMap.size,1);
callbacks.get('/markers')({markers:[{...marker,ns:'other',id:2}]});
callbacks.get('/markers')({markers:[{...marker,action:3}]});assert.equal(view.markerMap.size,1);assert.ok(view.markerMap.has('other:2'));
console.log('PASS atomic DELETEALL + ADD reconciliation and namespace-scoped deletion');
const old=view.markerMap.get('other:2').object;let oldDisposed=false;old.children[0].material.addEventListener('dispose',()=>{oldDisposed=true;});
callbacks.get('/markers')({markers:[{...marker,ns:'other',id:2,color:{r:0,g:1,b:0,a:1}}]});
assert.equal(old.parent,null);assert.equal(oldDisposed,false);view.flushRetired();assert.equal(oldDisposed,true);assert.equal(view.retired.length,0);
console.log('PASS changed shapes release GPU resources only after the replacement render can acquire shared shaders');
const gl={isContextLost:()=>false,LINK_STATUS:0x8b82,linkProgram(){},getProgramParameter:()=>false,getProgramInfoLog:()=>'persistent grid shader failed'};
view.shaderChecks=createShaderChecks(gl,{checkShaderErrors:true},'errors-only');gl.linkProgram({});view.shaderChecks.flush();
view.resetData();assert.equal(view.cloud,undefined);assert.equal(view.trace,undefined);assert.equal(Object.keys(view.joints).length,0);assert.equal(view.pendingJoints.size,0);assert.equal(Object.getPrototypeOf(view.joints),null);assert.match(view.shaderChecks.errors[0]??'',/persistent grid shader failed/);
console.log('PASS ROS data reset does not erase failures of programs retained by the renderer');
view.shaderChecks.dispose();view.off.forEach(off=>off());assert.equal(callbacks.size,0);
view.flushRetired();
// Invoke the real frame method with a controlled renderer. A failure inside the
// 500ms status throttle must remain visible on the later reporting frame.
const reports=[];const frame=Object.create(SceneView.prototype);
Object.assign(frame,{children:new Set(),disposed:false,animate:()=>{},controls:{update(){}},tfBuffer:{stats:()=>({lastReceivedAt:0,frames:0}),frames:()=>[]},lastTfUpdate:0,markerMap:new Map(),axes:new Map(),retired:[],layerEnabled:new Map(),scene:{},camera:{},options:{onStatus:m=>reports.push(m)},dirty:true,lastReport:performance.now(),loopCount:0,previousLoop:0,maxLoopGapMs:0,firstRenderMs:null,firstRenderCompleteAtMs:null,renderMaxMs:0,startedAt:performance.now(),renderSamples:[],renderErrors:0,renderCount:0,programCreations:0,programsSeen:new WeakSet(),assetErrors:new Set(),renderer:{getContext:()=>({isContextLost:()=>false}),render(){throw null;},info:{programs:[]}}});
const previousRaf=globalThis.requestAnimationFrame;globalThis.requestAnimationFrame=()=>1;
try{
  frame.renderFrame();assert.equal(frame.firstRenderCompleteAtMs,null);assert.equal(reports.length,0);
  frame.lastReport=performance.now()-1000;frame.renderFrame();assert.match(reports.at(-1),/绘制失败/);
  frame.renderer.render=()=>{};frame.dirty=true;frame.lastReport=performance.now()-1000;frame.renderFrame();
  assert.equal(frame.renderErrors,1);assert.equal(frame.renderCount,1);assert.notEqual(frame.firstRenderCompleteAtMs,null);assert.doesNotMatch(reports.at(-1),/绘制失败/);
}finally{if(previousRaf===undefined)delete globalThis.requestAnimationFrame;else globalThis.requestAnimationFrame=previousRaf;}
console.log('PASS render failures survive status throttling; only successful frames mark first completion');
console.log('PASS 9 resource lifecycle groups; all subscriptions released');
