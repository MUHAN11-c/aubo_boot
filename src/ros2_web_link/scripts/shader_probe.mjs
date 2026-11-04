/** Public GL boundary fault check; no browser/ROS process or private Three API. */
import assert from 'node:assert/strict';
import {createShaderChecks} from '../dist/scene/shaders.js';
let linkCalls=0,statusCalls=0,logCalls=0;
const original=()=>{linkCalls++;};
const gl={isContextLost:()=>false,LINK_STATUS:0x8b82,linkProgram:original,getProgramParameter:p=>{statusCalls++;return p.valid;},getProgramInfoLog:()=>{logCalls++;return 'synthetic linker failure';}};
const debug={checkShaderErrors:true};
const checks=createShaderChecks(gl,debug,'errors-only');assert.equal(debug.checkShaderErrors,false);
gl.linkProgram({valid:true});checks.flush();assert.equal(linkCalls,1);assert.equal(statusCalls,1);assert.equal(logCalls,0);assert.deepEqual(checks.errors,[]);
console.log('PASS healthy shaders still validate LINK_STATUS without querying verbose logs');
gl.linkProgram({valid:false});checks.flush();assert.equal(logCalls,1);assert.match(checks.errors[0],/synthetic linker failure/);checks.flush();assert.equal(statusCalls,2);
console.log('PASS failed shader remains visible and each new program is checked once');
assert.throws(()=>createShaderChecks(gl,debug,'errors-only'),/already own/);
checks.dispose();checks.dispose();assert.deepEqual(checks.errors,[]);assert.equal(gl.linkProgram,original);assert.equal(debug.checkShaderErrors,true);
console.log('PASS exclusive context ownership and dispose release programs and restore original context/debug settings');
const readonly=Object.create(gl);Object.defineProperty(readonly,'linkProgram',{value:original,writable:false});
const fallback=createShaderChecks(readonly,debug,'errors-only');assert.equal(fallback.mode,'verbose');assert.equal(debug.checkShaderErrors,true);fallback.dispose();
console.log('PASS non-interceptable contexts retain upstream verbose checks');
const ignored=Object.create(gl);Object.defineProperty(ignored,'linkProgram',{get:()=>original,set:()=>{}});
const ignoredChecks=createShaderChecks(ignored,debug,'errors-only');assert.equal(ignoredChecks.mode,'verbose');assert.equal(debug.checkShaderErrors,true);ignoredChecks.dispose();
console.log('PASS silently ignored GL hooks also retain upstream validation');

// Model native WebGL's null status/log while lost, rather than pretending every
// null LINK_STATUS represents a genuine compiler failure.
let contextFailures=0;
for(const phase of ['before-flush','during-status','during-log']){
  let lost=phase==='before-flush',queries=0;
  const context={LINK_STATUS:0x8b82,linkProgram(){},isContextLost:()=>lost,
    getProgramParameter(){queries++;if(phase==='during-status')lost=true;return lost?null:false;},
    getProgramInfoLog(){if(phase==='during-log')lost=true;return lost?null:'real failure';}};
  const owner=createShaderChecks(context,{checkShaderErrors:true},'errors-only');
  try{
    context.linkProgram({});owner.flush();
    assert.deepEqual(owner.errors,[],'context loss at '+phase+' must not become a linker failure');
    if(phase==='before-flush')assert.equal(queries,0,'lost context needs no status query');
    console.log('PASS context loss '+phase+' does not invent shader errors');
  }catch(cause){contextFailures++;console.error('FAIL '+phase+': '+cause.message);}
  finally{owner.dispose();}
}
{
  let lost=false,queries=0,failureLog='real retained failure';
  const context={LINK_STATUS:0x8b82,linkProgram(){},isContextLost:()=>lost,
    getProgramParameter(program){queries++;return lost?null:program.valid;},getProgramInfoLog:()=>lost?null:failureLog};
  const owner=createShaderChecks(context,{checkShaderErrors:true},'errors-only');
  try{
    context.linkProgram({valid:false});owner.flush();assert.match(owner.errors[0],/real retained failure/);
    lost=true;context.linkProgram({valid:true});owner.flush();
    lost=false;context.linkProgram({valid:true});owner.flush();
    assert.equal(owner.errors.length,1);assert.equal(queries,2,'lost pending must be discarded, real history retained');
    failureLog='new failure after recovery';context.linkProgram({valid:false});owner.flush();
    assert.equal(queries,3);assert.equal(owner.errors.length,2);assert.match(owner.errors[1],/new failure after recovery/);
    console.log('PASS recovery accepts valid programs, preserves history and detects new real failures');
  }catch(cause){contextFailures++;console.error('FAIL recovery history: '+cause.message);}
  finally{owner.dispose();}
}
{
  const context={LINK_STATUS:0x8b82,linkProgram(){},isContextLost:()=>false,
    getProgramParameter:()=>true,getProgramInfoLog:()=>null};
  const debug={checkShaderErrors:true},owner=createShaderChecks(context,debug,'errors-only');
  Object.defineProperty(context,'linkProgram',{value:context.linkProgram,writable:false,configurable:true});
  try{
    assert.doesNotThrow(()=>owner.dispose(),'a rejected hook restoration must not abort remaining cleanup');
    assert.equal(debug.checkShaderErrors,true);
    const next=createShaderChecks(context,debug,'errors-only');next.dispose();
    console.log('PASS failed hook restoration still restores debug and releases exclusive ownership');
  }catch(cause){contextFailures++;console.error('FAIL independent cleanup: '+cause.message);}
  finally{Object.defineProperty(context,'linkProgram',{writable:true});owner.dispose();}
}
{
  let logCalls=0;
  const context={LINK_STATUS:0x8b82,linkProgram(){},isContextLost:()=>false,
    getProgramParameter:()=>null,getProgramInfoLog(){logCalls++;return null;}};
  const owner=createShaderChecks(context,{checkShaderErrors:true},'errors-only');
  try{
    context.linkProgram({});owner.flush();
    assert.match(owner.errors[0]??'',/无法验证/,'null in a live context is unavailable, not a false linker result');
    assert.equal(logCalls,0,'an unavailable handle must not trigger another info-log query');
    console.log('PASS unavailable status is distinct from a genuine failed link');
  }catch(cause){contextFailures++;console.error('FAIL unavailable status: '+cause.message);}
  finally{owner.dispose();}
}
{
  let queries=0;
  const context={LINK_STATUS:0x8b82,linkProgram(){},isContextLost:()=>false,
    getProgramParameter(program){queries++;return program.valid;},getProgramInfoLog:()=> 'retained real failure'};
  const owner=createShaderChecks(context,{checkShaderErrors:true},'errors-only');
  try{
    context.linkProgram({valid:false});owner.flush();
    context.linkProgram({valid:null});
    // Loss and restore may both occur before the next frame/flush. Scene's
    // native context event must invalidate pending handles independently.
    owner.resetPending();owner.resetPending();owner.flush();
    context.linkProgram({valid:true});owner.flush();
    assert.equal(queries,2,'a restored context must never inspect an old pending handle');
    assert.equal(owner.errors.length,1);assert.match(owner.errors[0],/retained real failure/);
    console.log('PASS event invalidation drops pending handles without erasing real failure history');
  }catch(cause){contextFailures++;console.error('FAIL pending event invalidation: '+cause.message);}
  finally{owner.dispose();}
}
if(contextFailures)process.exitCode=1;
