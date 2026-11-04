/* Copyright 2026 ros2_web_link contributors; SPDX-License-Identifier: Apache-2.0 */
// Explicit CPU-only boundary check. It invokes the actual SceneView frame path;
// the controlled GL surface cannot establish real GPU or browser performance.
import assert from 'node:assert/strict';
import {SceneView} from '../dist/scene/index.js';

function fixture(initialLost=false,loseDuringRender=false) {
  const control={lost:initialLost,renderCalls:0,reports:[]};
  const frame=Object.create(SceneView.prototype);
  Object.assign(frame,{
    children:new Set(),disposed:false,animate:()=>{},controls:{update(){}},
    tfBuffer:{stats:()=>({lastReceivedAt:0,frames:0}),frames:()=>[]},
    lastTfUpdate:0,markerMap:new Map(),axes:new Map(),retired:[],
    layerEnabled:new Map(),scene:{},camera:{},
    options:{onStatus:message=>control.reports.push(message)},
    dirty:true,lastReport:performance.now()-1000,loopCount:0,previousLoop:0,
    maxLoopGapMs:0,firstRenderMs:null,firstRenderCompleteAtMs:null,renderMaxMs:0,
    startedAt:performance.now(),renderSamples:[],renderErrors:0,renderCount:0,
    programCreations:0,programsSeen:new WeakSet(),assetErrors:new Set(),
    renderer:{
      getContext:()=>({isContextLost:()=>control.lost}),
      render(){control.renderCalls++;if(loseDuringRender)control.lost=true;},
      info:{programs:[]},
    },
  });
  return {frame,control};
}

const previousRaf=globalThis.requestAnimationFrame;
globalThis.requestAnimationFrame=()=>1;
try {
  const before=fixture(true);
  before.frame.renderFrame();
  assert.equal(before.frame.renderCount,0,'an already lost context must not count as a rendered frame');
  assert.equal(before.control.renderCalls,0,'do not call the renderer for a context already lost');
  assert.equal(before.frame.firstRenderCompleteAtMs,null);
  assert.equal(before.frame.renderErrors,1);
  assert.match(before.control.reports.at(-1),/WebGL/);

  const during=fixture(false,true);
  during.frame.renderFrame();
  assert.equal(during.control.renderCalls,1);
  assert.equal(during.frame.renderCount,0,'loss during submission must not mark first completion');
  assert.equal(during.frame.firstRenderCompleteAtMs,null);
  assert.equal(during.frame.renderErrors,1);

  const recovered=fixture();
  recovered.frame.renderFrame();
  assert.equal(recovered.frame.renderCount,1);
  const first=recovered.frame.firstRenderCompleteAtMs;
  assert.notEqual(first,null);
  let resets=0;recovered.frame.shaderChecks={resetPending(){resets++;},flush(){}};
  recovered.control.lost=true;recovered.frame.contextChanged();
  assert.equal(resets,1,'native loss must invalidate pending programs even before flush');
  recovered.frame.renderFrame();
  assert.equal(recovered.frame.renderCount,1);
  assert.equal(recovered.frame.firstRenderCompleteAtMs,first);
  recovered.control.lost=false;recovered.frame.contextChanged();
  assert.equal(resets,2,'native restore must also invalidate old pending handles');
  recovered.frame.renderFrame();
  assert.equal(recovered.frame.renderCount,2);
  assert.equal(recovered.frame.lastRenderError,undefined);
  assert.equal(recovered.frame.renderErrors,1,'history of lost submissions survives recovery');
  assert.equal(recovered.frame.firstRenderCompleteAtMs,first);

  recovered.frame.disposed=true;recovered.frame.dirty=false;
  recovered.frame.contextChanged();
  assert.equal(resets,2);assert.equal(recovered.frame.dirty,false,'late events cannot revive a disposed view');
  recovered.frame.renderFrame();
  assert.equal(recovered.frame.renderCount,2);
  assert.equal(recovered.control.renderCalls,2);
} finally {
  if(previousRaf===undefined)delete globalThis.requestAnimationFrame;
  else globalThis.requestAnimationFrame=previousRaf;
}
console.log('PASS 4 context-loss boundary groups: before/during submission, recovery history and disposal; no GPU performance claim');
