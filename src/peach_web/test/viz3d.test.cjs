const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const file = path.join(__dirname, '../web/viz3d.js');
function api() { const context = {window: {}}; vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../web/vendor/peach-three.js'),'utf8'), context); context.window.PeachThree=context.PeachThree; vm.runInNewContext(fs.existsSync(file) ? fs.readFileSync(file,'utf8') : '', context); return context.window.PeachViz3D; }
function fixture() { const s = {job:{target_id:'a',coords:{frame_id:'base_link',grasp_pregrasp:[0.1,0.2,0.3]},grasp:{axis:[0,0,1],cut_pose:[0.1,0.2,0.4]}},perception:{targets:{frame_id:'base_link',stamp:11,observations:[{target_id:'a',candidate:{bag_bottom:[0.1,0.2,0.3],bag_neck:[0.1,0.2,0.4],translation_direction:[0,0,1]}}]}},reconstruction:{diagnostics:{target_id:'a',refined:{ok:false,reason:'axis_mismatch',axis:[1,0,0],bottom:[0.1,0.2,0.3],neck:[0.1,0.2,0.4],model_revision:'a:2:3'}}}}; s.perception.targets.scene_epoch=3; s.perception.targets.harvest_run_id='run-current'; Object.assign(s.reconstruction.diagnostics.refined,{target_id:'a',scene_epoch:3,harvest_run_id:'run-current'}); s.reconstruction.diagnostics.scene_epoch=3; s.reconstruction.grasp_decision={...s.job.grasp,target_id:'a',scene_epoch:3,frame_id:'base_link',pregrasp:s.job.coords.grasp_pregrasp}; return s; }
test('normalization preserves source timestamps, revisions and rejected axis evidence', () => { const scene = api()?.normalizeScene(fixture(),{}); assert.ok(scene,'scene conversion exists'); assert.equal(scene.frame_id,'base_link'); assert.equal(scene.initial.stamp,11); assert.equal(scene.fused.model_revision,'a:2:3'); assert.equal(scene.fused.diagnostic,true); assert.deepEqual(Array.from(scene.fused.axis),[1,0,0]); });
test('actual tool axis comes exclusively from tool_axis TF quaternion', () => { const s=fixture(); s.robot={tcp:{quat:[0,0,0,1]}}; const scene=api()?.normalizeScene(s,{frame_id:'base_link',tool_frames:{tool_axis:{available:true,frame_id:'base_link',position:[1,2,3],orientation:[0,Math.SQRT1_2,0,Math.SQRT1_2],stamp_sec:15}}}); assert.ok(scene); assert.ok(Math.abs(scene.tool_frames.tool_axis.axis[0]-1)<1e-12); assert.ok(Math.abs(scene.tool_frames.tool_axis.axis[2])<1e-12); assert.equal(scene.tool_frames.tool_axis.stamp_sec,15); assert.equal(api().normalizeScene(s,{}).tool_frames.tool_axis,undefined); });
test('target switches never retain another target geometry', () => { const s=fixture(); s.job.target_id='b'; const scene=api()?.normalizeScene(s,{}); assert.ok(scene); assert.equal(scene.initial,null); assert.equal(scene.fused,null); });
test('invalid vectors, non-base TF and zero quaternions never create geometry', () => { const s=fixture(); s.perception.targets.observations[0].candidate.bag_bottom=[null,0,0]; s.reconstruction.diagnostics.refined.axis=[0,0,0]; const scene=api()?.normalizeScene(s,{frame_id:'camera',xyz:[1,2,3],tool_frames:{tool_axis:{available:true,frame_id:'base_link',position:[1,2,3],orientation:[0,0,0,0]}}}); assert.ok(scene); assert.equal(scene.initial.bottom,null); assert.equal(scene.fused.axis,null); assert.equal(scene.trajectory.length,0); assert.equal(scene.tool_frames.tool_axis,undefined); });
test('missing library or WebGL produces a usable unavailable fallback',()=> { const view=api()?.create({}); assert.ok(view); assert.equal(view.available,false); assert.ok(view.error); assert.doesNotThrow(()=>{view.update({},{});view.setView('center');view.resize();view.dispose();}); });
test('bag and actual tool axes report source-based angle or null',()=> { const s=fixture(); const scene=api().normalizeScene(s,{tool_frames:{tool_axis:{available:true,frame_id:'base_link',position:[1,2,3],orientation:[0,0,0,1]}}}); assert.equal(scene.axisAngleDeg,90); assert.equal(api().normalizeScene(s,{}).axisAngleDeg,null); });
test('explicitly selected target hides geometry for another target',()=> { const s=fixture(); s.ui={selected_target_id:'b',selected_scene_epoch:4}; const scene=api().normalizeScene(s,{}); assert.equal(scene.target_id,'b'); assert.equal(scene.initial,null); assert.equal(scene.fused,null); assert.equal(scene.pregrasp,null); });
test('selected scene epoch rejects old reconstruction geometry',()=> { const s=fixture(); s.perception.targets.scene_epoch=3; s.ui={selected_target_id:'a',selected_scene_epoch:4}; const scene=api().normalizeScene(s,{}); assert.equal(scene.initial,null); assert.equal(scene.fused,null); assert.equal(scene.pregrasp,null); });
function renderedApi() {
  const context={window:{}};
  vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../web/vendor/peach-three.js'),'utf8'),context);
  const T=context.PeachThree.THREE; const calls={renders:0,disposed:0};
  function element() {return {style:{},children:[],clientWidth:600,clientHeight:400,setAttribute(){},appendChild(e){this.children.push(e);},addEventListener(){},removeEventListener(){},remove(){},replaceChildren(){this.children=[];}};}
  class Renderer {constructor(){this.domElement=element();} setPixelRatio(){} setClearColor(){} setSize(){} render(scene,camera){calls.renders++;calls.scene=scene;calls.camera=camera;} dispose(){calls.disposed++;} forceContextLoss(){}}
  class Controls {constructor(camera){this.target=new T.Vector3();calls.controls=this;} addEventListener(){} removeEventListener(){} update(){} dispose(){calls.controlsDisposed=true;}}
  Object.assign(context.window,{PeachThree:{THREE:{...T,WebGLRenderer:Renderer},OrbitControls:Controls},document:{hidden:false,createElement:element,addEventListener(){},removeEventListener(){}}});
  vm.runInNewContext(fs.readFileSync(file,'utf8'),context);
  return {view:context.window.PeachViz3D.create(element()),calls};
}
test('Z-up views render on changes, preserve user camera and release resources',()=> {
  const {view,calls}=renderedApi(); assert.equal(view.available,true);
  const s=fixture();view.update(s,{});assert.deepEqual(Array.from(calls.camera.up.toArray()),[0,0,1]);
  calls.camera.position.set(2,-2,3); const position=calls.camera.position.toArray(); const count=calls.renders;
  view.update(s,{}); assert.equal(calls.renders,count);
  s.reconstruction.diagnostics.refined.axis=[0,1,0];view.update(s,{});
  assert.deepEqual(Array.from(calls.camera.position.toArray()),Array.from(position));
  view.setView('top'); const delta=calls.camera.position.clone().sub(calls.controls.target);assert.ok(delta.z>0); assert.ok(Math.abs(delta.x)<1e-12);
  view.setView('side');assert.ok(Math.abs(calls.camera.position.z-calls.controls.target.z)<1e-12);
  view.dispose();view.dispose();assert.equal(calls.disposed,1);assert.equal(calls.controlsDisposed,true);
  const last=calls.renders;view.update(s,{});view.resize();view.setView('reset');assert.equal(calls.renders,last);
});
test('refused decision with zero axis never manufactures origin references',()=> { const s=fixture(); s.reconstruction.grasp_decision={target_id:'a',frame_id:'base_link',allowed:false,axis:[0,0,0],pregrasp:[0,0,0],entry:[0,0,0],cut_pose:[0,0,0]}; const scene=api().normalizeScene(s,{});assert.equal(scene.pregrasp,null);assert.equal(scene.entry,null);assert.equal(scene.cut,null); });
test('same target in new scene rejects old decision and all old fused references',()=> {
  const s=fixture();s.perception.targets.scene_epoch=9;s.ui={selected_target_id:'a',selected_scene_epoch:9};
  s.reconstruction.grasp_decision={target_id:'a',scene_epoch:8,frame_id:'base_link',axis:[0,0,1],pregrasp:[1,2,3],entry:[1,2,4],cut_pose:[1,2,5]};
  const scene=api().normalizeScene(s,{});assert.equal(scene.fused,null);assert.equal(scene.pregrasp,null);assert.equal(scene.entry,null);assert.equal(scene.cut,null);
});
test('unknown model and decision epochs never acquire current-scene scope',()=> {
  const s=fixture();s.perception.targets.scene_epoch=9;s.ui={selected_target_id:'a',selected_scene_epoch:9};
  delete s.reconstruction.diagnostics.scene_epoch;delete s.reconstruction.diagnostics.refined.scene_epoch;delete s.reconstruction.grasp_decision;
  const scene=api().normalizeScene(s,{});assert.equal(scene.fused,null);assert.equal(scene.pregrasp,null);
});
test('model explicit old epoch overrides even a valid current decision',()=> {
  const s=fixture();s.perception.targets.scene_epoch=9;s.ui={selected_target_id:'a',selected_scene_epoch:9};
  s.reconstruction.grasp_decision={target_id:'a',scene_epoch:9,frame_id:'base_link',axis:[0,0,1]};
  s.reconstruction.diagnostics.scene_epoch=8;
  assert.equal(api().normalizeScene(s,{}).fused,null);
});
test('explicit model scope can retain rejected diagnostics without a decision',()=> {
  const s=fixture();s.perception.targets.scene_epoch=9;s.ui={selected_target_id:'a',selected_scene_epoch:9};
  s.reconstruction.diagnostics.scene_epoch=9;s.reconstruction.diagnostics.refined.scene_epoch=9;delete s.reconstruction.grasp_decision;
  const scene=api().normalizeScene(s,{});assert.equal(scene.fused.diagnostic,true);assert.equal(scene.pregrasp,null);
});
test('unavailable actual TF removes angle immediately',()=> {
  const s=fixture();s.perception.targets.scene_epoch=9;s.reconstruction.diagnostics.scene_epoch=9;s.reconstruction.diagnostics.refined.scene_epoch=9;delete s.reconstruction.grasp_decision;
  const scene=api().normalizeScene(s,{tool_frames:{tool_axis:{available:false,frame_id:'base_link',position:[1,2,3],orientation:[0,0,0,1]}}});
  assert.equal(scene.axisAngleDeg,null);assert.equal(scene.tool_frames.tool_axis,undefined);
});
test('decision without epoch cannot borrow a correctly scoped model for motion references',()=> {
  const s=fixture();delete s.reconstruction.grasp_decision.scene_epoch;
  const scene=api().normalizeScene(s,{});assert.ok(scene.fused);assert.equal(scene.pregrasp,null);assert.equal(scene.cut,null);
});
test('old pose candidate cannot bypass contradictory model epoch via current decision',()=> {
  const s=fixture();s.perception.targets.scene_epoch=9;s.reconstruction.grasp_decision.scene_epoch=9;
  s.refined={pose:{frame_id:'base_link',candidates:[{target_id:'a',bag_bottom:[2,3,4],bag_neck:[2,3,5],translation_direction:[0,0,1]}]}};
  s.reconstruction.diagnostics.scene_epoch=8;
  assert.equal(api().normalizeScene(s,{}).fused,null);
});
function rawContextFixture() {
  const s=fixture();delete s.reconstruction.diagnostics.scene_epoch;delete s.reconstruction.diagnostics.refined.scene_epoch;delete s.reconstruction.diagnostics.refined.harvest_run_id;delete s.reconstruction.diagnostics.refined.target_id;
  s.perception.targets.harvest_run_id='run-new';s.reconstruction.grasp_decision.scene_epoch=0;
  s.reconstruction.diagnostics.refined_context={target_id:'a',harvest_run_id:'run-new',scene_epoch:3};
  return s;
}
test('raw rejection retains diagnostic axis with verified target run epoch context',()=> {
  const s=rawContextFixture();const scene=api().normalizeScene(s,{});
  assert.ok(scene.fused);assert.equal(scene.fused.diagnostic,true);assert.deepEqual(Array.from(scene.fused.axis),[1,0,0]);assert.equal(scene.pregrasp,null);
});
test('typed target merge cannot rewrite raw rejected model provenance',()=> {
  const s=rawContextFixture();s.reconstruction.diagnostics.refined_context.target_id='old-target';
  assert.equal(api().normalizeScene(s,{}).fused,null);
});
test('old raw run context cannot borrow a valid current decision',()=> {
  const s=rawContextFixture();s.reconstruction.diagnostics.refined_context.harvest_run_id='run-old';
  s.reconstruction.grasp_decision.scene_epoch=3;
  assert.equal(api().normalizeScene(s,{}).fused,null);
});
test('old raw epoch context cannot borrow a valid current decision',()=> {
  const s=rawContextFixture();s.reconstruction.diagnostics.refined_context.scene_epoch=2;s.reconstruction.grasp_decision.scene_epoch=3;
  assert.equal(api().normalizeScene(s,{}).fused,null);
});
test('verified raw context never scopes an unversioned typed pose candidate',()=> {
  const s=rawContextFixture();delete s.reconstruction.diagnostics.refined;
  s.refined={pose:{frame_id:'base_link',candidates:[{target_id:'a',bag_bottom:[2,3,4],bag_neck:[2,3,5],translation_direction:[0,0,1]}]}};
  assert.equal(api().normalizeScene(s,{}).fused,null);
});
test('incomplete raw context never borrows current decision epoch',()=> {
  const s=rawContextFixture();delete s.reconstruction.diagnostics.refined_context.scene_epoch;s.reconstruction.grasp_decision.scene_epoch=3;
  assert.equal(api().normalizeScene(s,{}).fused,null);
});
test('new decision cannot revive old raw diagnostics after context removal',()=> {
  const s=rawContextFixture();delete s.reconstruction.diagnostics.refined_context;
  s.perception.targets.scene_epoch=9;s.perception.targets.harvest_run_id='run-new';s.reconstruction.grasp_decision.scene_epoch=9;
  s.reconstruction.diagnostics.harvest_run_id='run-old';
  assert.equal(api().normalizeScene(s,{}).fused,null);
});
test('new decision cannot relabel old typed pose without its own run and epoch',()=> {
  const s=rawContextFixture();delete s.reconstruction.diagnostics.refined;delete s.reconstruction.diagnostics.refined_context;
  s.reconstruction.grasp_decision.scene_epoch=3;
  s.refined={pose:{frame_id:'base_link',candidates:[{target_id:'a',bag_bottom:[2,3,4],bag_neck:[2,3,5],translation_direction:[0,0,1]}]}};
  assert.equal(api().normalizeScene(s,{}).fused,null);
});
test('typed pose with its own matching run and epoch remains visible',()=> {
  const s=fixture();delete s.reconstruction.diagnostics.refined;delete s.reconstruction.grasp_decision;
  s.refined={pose:{frame_id:'base_link',scene_epoch:3,harvest_run_id:'run-current',candidates:[{target_id:'a',bag_bottom:[2,3,4],bag_neck:[2,3,5],translation_direction:[0,0,1]}]}};
  assert.deepEqual(Array.from(api().normalizeScene(s,{}).fused.bottom),[2,3,4]);
});
test('typed pose with matching epoch but old run stays hidden',()=> {
  const s=fixture();delete s.reconstruction.diagnostics.refined;
  s.refined={pose:{frame_id:'base_link',scene_epoch:3,harvest_run_id:'run-old',candidates:[{target_id:'a',bag_bottom:[2,3,4],bag_neck:[2,3,5],translation_direction:[0,0,1]}]}};
  assert.equal(api().normalizeScene(s,{}).fused,null);
});
