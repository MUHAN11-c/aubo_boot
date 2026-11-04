/** Reproductions from the one fresh-context final review. No ROS/browser/GL. */
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {SceneView} from '../dist/scene/index.js';
import {GraphView} from '../dist/graph/index.js';
import {WorkbenchView,WorkbenchController,createDefaultRegistry,validateWorkspace} from '../dist/workbench/index.js';
const cases=[],test=(name,fn)=>cases.push([name,fn]);
const base=()=>({schemaVersion:1,fixedFrame:'world',tf:'/tf',tfStatic:'/static',useSimTime:false,layout:'split',displays:[]});
const raw={id:'r',name:'r',plugin:'raw',enabled:true,inputs:{data:{topic:'/data',type:'std_msgs/msg/String'}},options:{}};
test('import rejects non-string enums before replacing an active workspace',()=>{
 const reg=createDefaultRegistry();
 for(const field of ['layout','compression','reliability','durability']){
  const value=base();value.displays=[structuredClone(raw)];
  if(field==='layout')value.layout=['split'];
  else if(field==='compression')value.displays[0].inputs.data.compression=['cbor'];
  else value.displays[0].inputs.data.qos={history:'keep_last',depth:10,reliability:field==='reliability'?['reliable']:'reliable',durability:field==='durability'?['volatile']:'volatile'};
  assert.throws(()=>validateWorkspace(value,reg),undefined,`${field} must be a string enum`);
 }
});
function sceneFixture(layers=[]){
 const callbacks=new Map(),root=Object.create(SceneView.prototype),identity={translation:{x:0,y:0,z:0},rotation:{x:0,y:0,z:0,w:1}};
 Object.assign(root,{root:{replaceChildren(){}},client:{subscribe(name,type,fn){callbacks.set(name,fn);return()=>callbacks.delete(name);}},
  options:{fixedFrame:'world',tf:'/tf',tfStatic:'/static',robotDescription:'/robot',jointStates:'/joints',markers:'/markers',pointCloud:'/points',path:'/path',...(layers===undefined?{}:{layers})},
  children:new Set(),scene:new THREE.Scene(),camera:new THREE.PerspectiveCamera(),renderer:{getContext:()=>({isContextLost:()=>false}),render(){root.draws++;},domElement:{removeEventListener(){}},dispose(){},forceContextLoss(){},info:{programs:[],render:{calls:1,triangles:0},memory:{geometries:0,textures:0}}},draws:0,
  controls:{update(){},dispose(){}},resize:{disconnect(){}},tfBuffer:{start(){},dispose(){},stats:()=>({frames:1,lastReceivedAt:0}),frames:()=>[],lookup:()=>identity},
  off:[],joints:Object.create(null),pendingJoints:new Map(),markerMap:new Map(),axes:new Map(),retired:[],assetErrors:new Set(),restoreShaderHooks:[],layerEnabled:new Map(['robot','tf','markers','cloud','path'].map(id=>[id,layers===undefined||layers.includes(id)])),
  generation:0,animate:()=>{},renderSamples:[],renderCount:0,loopCount:0,renderErrors:0,renderMaxMs:0,firstRenderMs:null,startedAt:performance.now(),lastReport:performance.now(),programsSeen:new WeakSet(),programCreations:0,dirty:true});
 return {root,callbacks};
}
test('one shared rendered frame releases retired resources of every child',()=>{
 const {root,callbacks}=sceneFixture();root.start();const child=root.createDisplay({layers:['markers'],markers:'/changing'});child.start();let released=0;
 const marker={header:{frame_id:'world',stamp:{sec:0,nanosec:0}},ns:'test',id:1,type:2,action:0,pose:{position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}},scale:{x:1,y:1,z:1},color:{r:1,g:0,b:0,a:1},points:[],colors:[],text:'',mesh_resource:'',frame_locked:false,lifetime:{sec:0,nanosec:0}};
 try{
  callbacks.get('/changing')({markers:[marker]});child.markerMap.get('test:1').object.children[0].geometry.addEventListener('dispose',()=>{assert(root.draws>0,'retained shader resources must survive until replacement render');released++;});
  callbacks.get('/changing')({markers:[{...marker,scale:{x:2,y:1,z:1}}]});assert.equal(child.retired.length,1);assert.equal(released,0);
  root.renderFrame();assert.equal(child.retired.length,0,'shared render must drain child retirement queues');assert.equal(released,1);
 }finally{root.dispose();}
});
test('legacy pre-start setLayer remains visual and does not remove subscriptions',()=>{
 const {root,callbacks}=sceneFixture(undefined);delete root.options.layers;
 try{root.setLayer('cloud',false);root.start();root.setLayer('cloud',true);assert(callbacks.has('/points'),'legacy cloud subscription must exist after visual toggle');}
 finally{root.dispose();}
});
const snapshot={status:'complete',receivedAt:Date.now(),elapsedMs:1,cancelled:false,scope:'fixture',errors:[],omittedErrors:0,services:[],actions:[],capabilities:{},nodes:[],topics:[{id:'t',name:'/shared',types:['std_msgs/msg/String']}],edges:[]};
for(let i=0;i<46;i++){snapshot.nodes.push({id:`n${i}`,name:`/n${i}`,duplicates:1,details:'known'});snapshot.edges.push({id:`p${i}`,source:`n${i}`,target:'t',kind:'publish',topic:'/shared'},{id:`s${i}`,source:'t',target:`n${i}`,kind:'subscribe',topic:'/shared'});}
function viewFixture(graph){
 const values=new Map();for(const [id,value] of Object.entries({'graph-search':'','graph-mode':'nodes','graph-layout':'breadthfirst','graph-internal':false,'graph-state':''}))values.set(id,{value,checked:value,textContent:'',disabled:false});
 const view=Object.create(WorkbenchView.prototype);Object.assign(view,{graphView:graph,graphSnapshot:snapshot,graphSource:'ws://old:9090/',graphStale:false,el:id=>{if(!values.has(id))values.set(id,{textContent:'',value:''});return values.get(id);},input:id=>values.get(id),select:id=>values.get(id),controller:{syncConnection(){},stats:()=>({running:false})},stopGraph(){},syncGraphButtons(){}});return {view,values};
}
test('node projection cap is visible beside the graph',()=>{
 const graph=new GraphView();try{graph.setSnapshot(snapshot);const {view,values}=viewFixture(graph);view.filterGraph();assert(graph.stats().projectionTruncated);assert.match(values.get('graph-state').textContent,/截断|不完整/);}finally{graph.dispose();}
});
test('bridge identity change marks retained graph and endpoint lists as old',()=>{
 const {view,values}=viewFixture({stats:()=>({projectionTruncated:false})});Object.assign(view,{lastControl:{connected:true,options:{url:'ws://old:9090'}},options:{getControlClient:()=>({connected:true,options:{url:'ws://new:9090'}}),getObservationClient:()=>({connected:true})}});
 view.syncConnection();assert.equal(view.graphStale,true);assert.match(values.get('graph-state').textContent,/上一会话|过期|失效/);assert.match(values.get('graph-state').textContent,/old:9090/);
});
test('public controller stats identify per-display assets rather than empty host',()=>{
 const {root}=sceneFixture();root.start();const child=root.createDisplay({layers:['robot']});child.assetsLoaded=3;child.assetsTotal=4;child.robotParseMs=7;child.assetReadyMs=null;
 const controller=new WorkbenchController({getObservationClient:()=>undefined},createDefaultRegistry(),base());controller.sceneHost=root;controller.active.set('robot',{spec:{id:'robot',name:'robot',plugin:'robot',enabled:true},instance:child});
 try{const stats=controller.stats();assert.equal(stats.displays[0].assets?.assetsLoaded,3);assert.equal(stats.displays[0].assets?.assetsTotal,4);assert.equal(stats.displays[0].assets?.robotParseMs,7);assert.equal(Object.hasOwn(stats.renderer,'assetsLoaded'),false,'shared renderer metrics must not claim model asset ownership');}
 finally{controller.dispose();}
});
const oldRaf=globalThis.requestAnimationFrame,oldCancel=globalThis.cancelAnimationFrame;globalThis.requestAnimationFrame=()=>1;globalThis.cancelAnimationFrame=()=>{};
let failed=0;try{for(const [name,fn] of cases){try{fn();console.log('PASS',name);}catch(error){failed++;console.error('FAIL',name,error.message);}}}finally{if(oldRaf===undefined)delete globalThis.requestAnimationFrame;else globalThis.requestAnimationFrame=oldRaf;if(oldCancel===undefined)delete globalThis.cancelAnimationFrame;else globalThis.cancelAnimationFrame=oldCancel;}
assert.equal(failed,0,`${failed} review findings remain`);console.log(`PASS ${cases.length} final-review reproductions`);
