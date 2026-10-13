const assert = require('node:assert/strict');
const test = require('node:test');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
let visual = {};
try { visual = require('../web/visualization.js'); } catch (error) {
  if (error.code !== 'MODULE_NOT_FOUND') throw error;
}

test('image metadata never presents missing or invalid timestamps as live', () => {
  assert.equal(typeof visual.frameInfo, 'function');
  assert.equal(visual.frameInfo(null, 20).available, false);
  assert.equal(visual.frameInfo({available: true, stamp_sec: NaN}, 20).available, false);
  assert.equal(visual.frameInfo({available: true, stamp_sec: 19.8}, 20).stale, false);
  assert.equal(visual.frameInfo({available: true, stamp_sec: 18}, 20).stale, true);
});

test('bad encoder evidence is distinct from a paused or stale camera view', () => {
  assert.equal(typeof visual.frameInfo, 'function');
  const failed = visual.frameInfo({available: false, error: 'encode_failed'}, 20);
  assert.equal(failed.available, false);
  assert.match(failed.label, /不可用/);
  const stale = visual.frameInfo({available: false, error: 'stale', stamp_sec: 18}, 20);
  assert.equal(stale.stale, true);
  assert.match(stale.label, /陈旧/);
});

test('ROS frame age takes precedence over a wall clock from another time domain', () => {
  const info = visual.frameInfo({available:true,stamp_sec:10,age_s:0.1},1791800000);
  assert.equal(info.stale,false);
  assert.equal(info.age,0.1);
});

test('negative ROS frame age cannot be presented as fresh', () => {
  const info=visual.frameInfo({available:true,stamp_sec:20,age_s:-1},null);
  assert.equal(info.available,false);
  assert.match(info.label,/时钟/);
});

test('model diagnostics distinguish point fit quality from unavailable single-view consistency', () => {
  assert.equal(typeof visual.modelInfo, 'function');
  const summary = visual.modelInfo({reconstruction: {diagnostics: {refined: {
    model_revision: 'target_0:1:2', refit_rmse: 0.003,
    fusion_pose_count: 1, fusion_consistency_available: false,
    fusion_landmark_scatter_rmse_m: null, reason: 'fused_axis_conflict',
  }}}});
  assert.match(summary, /target_0:1:2/);
  assert.match(summary, /3.0 mm/);
  assert.match(summary, /多视一致性不可用/);
  assert.match(summary, /错轴/);
});

test('diagnostic heartbeats do not impersonate a newly fitted model timestamp', () => {
  const summary = visual.modelInfo({system:{topic_age_s:{'reconstruction.diagnostics':0.2}},
    reconstruction:{diagnostics:{stamp:20,refined:{}}}});
  assert.match(summary,/模型时间未报告/);
  assert.match(summary,/诊断发布 20.000 s/);
  assert.match(summary,/诊断接收距今 0.2 s/);
});

test('an in-flight image cannot resurrect a stream invalidated by an encoder failure', async () => {
  const elements = new Map();
  const element = selector => {
    if (elements.has(selector)) return elements.get(selector);
    const el = {hidden:selector === '.viz-image-box img',dataset:{},textContent:'',
      addEventListener(){},removeEventListener(){},remove(){},appendChild(){},
      removeAttribute(name){delete this[name];},querySelector:s=>element(selector+' '+s),
      querySelectorAll:()=>[],showModal(){this.open=true;},close(){this.open=false;}};
    elements.set(selector,el); return el;
  };
  const panel = element('panel'); panel.querySelector = element;
  let deliver;
  const window = {};
  const context = vm.createContext({window,console,Date,Number,Math,AbortController,
    document:{hidden:false,createElement:()=>panel,getElementById:()=>null},
    setInterval:()=>1,clearInterval(){},setTimeout:()=>1,clearTimeout(){},
    fetch:()=>new Promise(resolve=>{deliver=resolve;}),
    URL:{createObjectURL:()=> 'blob:old-frame',revokeObjectURL(){}}});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../web/visualization.js'),'utf8'),context);
  const view = window.PeachVisualization.create({appendChild(){}});
  view.update({images:{perception:{available:true,stamp_sec:10,age_s:0.1}},system:{server_time:10.1}},{});
  view.update({images:{perception:{available:false,error:'encode_failed'}},system:{server_time:10.1}},{});
  deliver({ok:true,headers:{get:()=>null},blob:async()=>({})});
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(element('.viz-image-box img').hidden,true);
  assert.equal(element('.viz-image-box img').src,undefined);
  view.dispose();
});
